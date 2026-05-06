#!/usr/bin/env python3
"""
نظام تحكم MPC لصاروخ M130 — توجيه هندسي LOS + متحكم MPC
===========================================================
البنية:
  1. التوجيه: gamma_ref يُحسب هندسياً من زاوية خط البصر LOS إلى الهدف
     - لا يعتمد على أوامر التحكم ← لا تغذية راجعة ← لا تذبذب
  2. MPC (acados SQP_RTI) يتتبع gamma_ref/chi_ref مع القيود → زعانف
  3. المراحل: (1) boost altitude-hold  (2) LOS cruise/terminal

الواجهة: control_function(state_dict, t) -> np.array[4]
"""

import math
import numpy as np
import os
import sys
import ctypes
from pathlib import Path

_ACADOS_DIR = Path(__file__).resolve().parents[2] / 'acados-main'
if _ACADOS_DIR.exists():
    _lib_dir = str(_ACADOS_DIR / 'lib')
    os.environ.setdefault('ACADOS_SOURCE_DIR', str(_ACADOS_DIR))
    _ld = os.environ.get('LD_LIBRARY_PATH', '')
    if _lib_dir not in _ld:
        os.environ['LD_LIBRARY_PATH'] = _lib_dir + ':' + _ld
        try:
            ctypes.cdll.LoadLibrary(str(_ACADOS_DIR / 'lib' / 'libhpipm.so'))
            ctypes.cdll.LoadLibrary(str(_ACADOS_DIR / 'lib' / 'libblasfeo.so'))
            ctypes.cdll.LoadLibrary(str(_ACADOS_DIR / 'lib' / 'libacados.so'))
        except OSError:
            pass
    _acados_py = str(_ACADOS_DIR / 'interfaces' / 'acados_template')
    if _acados_py not in sys.path:
        sys.path.insert(0, _acados_py)

from dynamics.quaternion_utils import quaternion_to_euler

NX = 15  # no servo dynamics — ideal fins (command = execution)
NU = 3
_G = 9.80665


class MpcController:
    """
    متحكم MPC بتوجيه هندسي LOS.

    البنية:
      - التوجيه: gamma_ref = atan2(dh, dx) → زاوية خط البصر الهندسية إلى الهدف
        لا تعتمد على مخرجات التحكم ← لا تغذية راجعة ← لا تذبذب
      - MPC يتتبع gamma_ref/chi_ref مع المحافظة على الاستقرار
      - تشكيل زاوية الاصطدام: مزج سلس نحو زاوية الاصطدام المطلوبة قرب الهدف
    """

    def __init__(self, sim):
        cfg = sim.config
        mpc_cfg = cfg.get('autopilot', {}).get('mpc', {})

        self.N_horizon = mpc_cfg.get('N_horizon', 200)
        self.tf = mpc_cfg.get('tf', 4.0)
        self.t_ctrl = max(mpc_cfg.get('t_ctrl', 0.5), 0.2)
        self._dt_solve = mpc_cfg.get('dt_solve', 0.02)
        self._delta_rate_limit = math.radians(mpc_cfg.get('delta_rate_limit_dps', 250.0))

        target_cfg = cfg.get('target', {})
        self.target_x = target_cfg.get('range_m', 2600.0)
        launch_alt = cfg.get('launch', {}).get('altitude', 1200.0)
        self._launch_alt = launch_alt
        self.target_h = target_cfg.get('altitude', 0.0) - launch_alt

        mp = sim.mass_properties_model
        self.mass_full = mp.M_total
        self.mass_dry = mp.M_body
        self.burn_time = mp.t_burn
        self.thrust_avg = sim.propulsion.average_thrust
        # Compute plateau thrust: corrects for tail-off pulling the average down.
        # T_plateau = total_impulse / (burn_time - 0.75 * t_tail)
        # This is ~15-20% higher than average and matches mid-burn reality.
        self._t_tail = 1.0  # tail-off duration (s)
        if self.burn_time > self._t_tail:
            self.thrust_plateau = (sim.propulsion.total_impulse * sim.propulsion.thrust_scale
                                   / (self.burn_time - 0.75 * self._t_tail))
        else:
            self.thrust_plateau = self.thrust_avg
        self.Ixx_full, self.Ixx_dry = mp.Ixx_0, mp.Ixx_end
        self.Iyy_full, self.Iyy_dry = mp.Iyy_0, mp.Iyy_end
        self.Izz_full, self.Izz_dry = mp.Izz_0, mp.Izz_end

        # إزاحة مركز الثقل: xbc يتغير من 0 (وقود ممتلئ) إلى xcg_end - xcg_0 (بعد الاحتراق)
        rocket_cfg = cfg.get('rocket', {})
        self._xbc_max = rocket_cfg.get('xcg_end', 0.0) - rocket_cfg.get('xcg_0', 0.0)

        act_cfg = cfg.get('actuator', {})
        # Servo delay for forward prediction (not in model — handled externally)
        self._servo_delay_s = max(act_cfg.get('servo_delay_s',
                                   mpc_cfg.get('servo_delay_s', 0.0)), 0.0)
        # Command buffer for forward prediction
        self._cmd_buffer = []  # list of (timestamp, de, dr, da)
        # ──────────────────────────────────────────────────────────────────
        # Lookahead stage for dead-time compensation (pull fin commands from
        # future MPC stage). Auto-compute from actuator delays if not set.
        #
        # CORRECT FORMULA (must include tau_servo):
        #   total_delay   = transport(delay_steps × dt_sim) + servo τ(tau_servo)
        #   lookahead     = round(total_delay / dt_h)
        #
        # The MPC's NX=15 model has IDEAL FINS — no servo dynamics. The plant
        # has BOTH transport delay AND first-order servo lag (τ). If τ is not
        # included in lookahead, the closed loop exhibits a 5 Hz LIMIT CYCLE
        # in pitch rate (verified 6DOF sim 2026-05-02: ignoring τ → la=2 →
        # 5Hz amp 130, range error 14m; including τ → la=3 → range error 0m).
        #
        # PX4 firmware uses a single param RKT_MPC_SVO_DLY that holds the
        # precomputed total (with safety margin). The simulator MUST apply
        # the SAME rule so that standalone results match PX4/SITL/flight:
        #
        #   RKT_MPC_SVO_DLY = max(pure_delay + tau_servo + 40ms, 100ms)
        #
        # +40ms  = safety for delays the sim does NOT model:
        #          PX4 scheduling jitter, sensor pipeline, CAN contention,
        #          MPC solve variability, future-planning safety.
        # 100ms  = absolute floor (lookahead_stage ≥ 5 for MPC stability).
        # ──────────────────────────────────────────────────────────────────
        _SAFETY_MARGIN_S = 0.040   # +40ms (same as PX4 rule)
        _FLOOR_S         = 0.100   # 100ms absolute floor
        la_cfg = mpc_cfg.get('lookahead_stage', None)
        if la_cfg is not None:
            self._lookahead_stage = int(la_cfg)
        else:
            actuator = getattr(sim, 'actuator', None)
            act_delay = getattr(actuator, 'delay_steps', 0)
            # IMPORTANT: read tau_servo from the YAML CONFIG, not from the
            # actuator object. SecondOrderActuatorModel computes a misleading
            # `actuator.tau_servo = 1/zeta_wn` (e.g. 10ms for ζω=100), but
            # the EFFECTIVE first-order τ of the dominant slow pole IS the
            # YAML-supplied tau_servo (e.g. 31.4ms measured for XQPOWER).
            # The YAML value reflects the measured physical delay we must
            # compensate; the actuator's internal value is for its own math.
            tau_servo = float(act_cfg.get('tau_servo', 0.0))
            dt_h = self.tf / self.N_horizon  # MPC step size (s)
            dt_sim = 0.01                    # simulation step size (s)
            transport_s = act_delay * dt_sim
            total_delay_s = transport_s + tau_servo + _SAFETY_MARGIN_S
            total_delay_s = max(total_delay_s, _FLOOR_S)
            self._lookahead_stage = max(1, int(round(total_delay_s / dt_h)))
            if act_delay > 0 or tau_servo > 0:
                print(
                    f"MPC lookahead_stage={self._lookahead_stage} "
                    f"(auto: transport={transport_s*1000:.1f}ms + "
                    f"τ_servo={tau_servo*1000:.1f}ms + "
                    f"safety={_SAFETY_MARGIN_S*1000:.0f}ms = "
                    f"{total_delay_s*1000:.1f}ms / dt_h={dt_h*1000:.1f}ms, "
                    f"floor={_FLOOR_S*1000:.0f}ms)"
                )

        launch_cfg = cfg.get('launch', {})
        pitch_deg = launch_cfg.get('attitude_degrees', [0, 15, 0])[1]
        # γ_ref الطبيعي أثناء الاحتراق: atan2(T·sin θ − mg, T·cos θ)
        # هذه هي زاوية مسار الطيران (flight-path) الفعلية التي يحققها الصاروخ
        # طبيعياً مع قوة دفع ثابتة، لا زاوية الجسم pitch.
        _T_avg = float(self.thrust_avg)
        _m = float(self.mass_full)
        _th = math.radians(pitch_deg)
        _a_v = _T_avg * math.sin(_th) - _m * _G
        _a_h = _T_avg * math.cos(_th)
        self._gamma_natural_rad = math.atan2(_a_v, max(_a_h, 1.0))
        self._cruise_progress = mpc_cfg.get('cruise_progress', 0.65)  # hold altitude until 65% of range
        self._cruise_alt_target = None  # captured at peak after burnout

        lat_r = math.radians(launch_cfg.get('latitude', 16.457472))
        lon_r = math.radians(launch_cfg.get('longitude', 44.115361))
        sin_lat, cos_lat = math.sin(lat_r), math.cos(lat_r)
        sin_lon, cos_lon = math.sin(lon_r), math.cos(lon_r)
        _a = 6378137.0
        _e2 = 2 / 298.257223563 - (1 / 298.257223563) ** 2
        N_wgs = _a / math.sqrt(1 - _e2 * sin_lat ** 2)
        self._launch_ecef = np.array([
            (N_wgs + launch_alt) * cos_lat * cos_lon,
            (N_wgs + launch_alt) * cos_lat * sin_lon,
            (N_wgs * (1 - _e2) + launch_alt) * sin_lat,
        ])
        self._C_ecef_to_ned = np.array([
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [-sin_lon, cos_lon, 0.0],
            [-cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat],
        ])

        # LOS guidance parameters
        m130 = mpc_cfg.get('m130_guidance', {})
        self._impact_angle_deg = m130.get('impact_angle_deg', -30.0)
        # Impact blend: starts earlier (75% range = ~650m out) so γ_ref has
        # enough time to track impact_angle_deg before the rate-limiter caps
        # accumulated change. Original 0.93 worked only because MHE gamma was
        # biased steep; truth-state requires earlier blend.
        self._impact_blend_start = 0.90
        self._impact_blend_end   = 0.99
        self._roll_recovery_start = math.radians(5.0)
        self._roll_recovery_full = math.radians(20.0)

        self._solver = None
        self._solver_ready = False
        self._last_delta_e = 0.0
        self._last_delta_r = 0.0
        self._last_delta_a = 0.0
        self._last_fins = np.zeros(4)
        self._last_solve_t = -1.0
        self._last_t = 0.0
        self._last_solve_time_ms = 0.0
        self._last_solver_status = 0
        self._last_sqp_iterations = 0
        self._solve_count = 0

        self._consec_fails = 0
        self._consec_ok = 0
        self._warm = False

        self._cur_alt = 0.0
        self._cur_x = 0.0
        self._cur_y = 0.0
        # التهيئة عند زاوية الإطلاق — يمنع rate-limiter من تأخير وصول γ_ref
        # إلى القيمة الصحيحة (ذات الpitch=15°) في أول نداء لـ MPC.
        self._gamma_ref_prev = self._gamma_natural_rad
        self._chi_ref_prev = 0.0
        self._H_SCALE = 100.0

        # --- MHE / estimation settings ---
        est_cfg = cfg.get('estimation', {})
        self._use_estimation = (est_cfg.get('mode') == 'mhe'
                                and est_cfg.get('use_estimated_state_for_mpc', False))
        self._quality_gate_thr = est_cfg.get('mhe', {}).get('quality_gate_threshold', 0.3)
        self._blend_alpha = 0.0  # current blend factor
        self._wn_smooth = 7.0     # EMA-smoothed w_norm (init high = boost)

        self._build_solver()

    # ------------------------------------------------------------------
    # Solver build
    # ------------------------------------------------------------------
    @staticmethod
    def _source_hash(d):
        import hashlib
        h = hashlib.md5()
        for f in ['m130_acados_model.py', 'm130_ocp_setup.py', 'm130_mpc_autopilot.py']:
            p = os.path.join(d, f)
            if os.path.exists(p):
                h.update(open(p, 'rb').read())
        return h.hexdigest()

    def _build_solver(self):
        try:
            mpc_dir = str(Path(__file__).parent)
            if mpc_dir not in sys.path:
                sys.path.insert(0, mpc_dir)

            acados_lib = _ACADOS_DIR / 'lib'
            if acados_lib.exists():
                for n in ['libqpOASES_e.so', 'libblasfeo.so', 'libhpipm.so', 'libacados.so']:
                    p = acados_lib / n
                    if p.exists():
                        ctypes.CDLL(str(p), mode=ctypes.RTLD_GLOBAL)

            from m130_ocp_setup import create_m130_ocp
            from acados_template import AcadosOcpSolver

            h_min = min(0.0, self.target_h / self._H_SCALE) - 5.0
            ocp = create_m130_ocp(
                h_min=h_min, rate_limit_rad=self._delta_rate_limit,
                launch_alt_val=self._launch_alt,
                mass_full_val=self.mass_full, mass_dry_val=self.mass_dry,
                Ixx_full_val=self.Ixx_full, Ixx_dry_val=self.Ixx_dry,
                Iyy_full_val=self.Iyy_full, Iyy_dry_val=self.Iyy_dry,
                Izz_full_val=self.Izz_full, Izz_dry_val=self.Izz_dry,
                xbc_max_val=self._xbc_max,
            )
            ocp.solver_options.tf = self.tf
            ocp.solver_options.N_horizon = self.N_horizon

            json_path = os.path.join(mpc_dir, 'm130_mpc_autopilot.json')
            hash_file = os.path.join(mpc_dir, '.mpc_source_hash')
            cur_hash = self._source_hash(mpc_dir)
            rebuild = True
            if os.path.exists(hash_file):
                if open(hash_file).read().strip() == cur_hash:
                    rebuild = False
            if rebuild:
                if os.path.exists(json_path):
                    os.remove(json_path)
                print("MPC source changed — clean rebuild")
            open(hash_file, 'w').write(cur_hash)

            self._solver = AcadosOcpSolver(ocp, json_file=json_path)
            self._solver.options_set('warm_start_first_qp', 1)
            self._solver_ready = True

            # Create sim solver for Smith predictor (model-consistent forward prediction)
            if self._servo_delay_s > 1e-4:
                try:
                    from acados_template import AcadosSimSolver
                    self._sim_solver = AcadosSimSolver(ocp, json_file=json_path)
                    self._sim_dt = self.tf / self.N_horizon  # one MPC step
                    print(f"Smith predictor sim solver init (dt_sim={self._sim_dt:.4f}s)")
                except Exception as e:
                    print(f"Smith predictor sim solver FAILED: {e}")
                    self._sim_solver = None
            else:
                self._sim_solver = None

            print(f"MPC solver built (h_min={h_min:.2f})")

        except Exception as e:
            print(f"\n{'='*60}\nCRITICAL: MPC solver build FAILED: {e}\n{'='*60}")
            import traceback; traceback.print_exc()
            self._solver_ready = False

    # ==================================================================
    #  Geometric LOS Guidance
    # ==================================================================

    def _compute_los_guidance(self, x_pos, y_pos, altitude):
        """
        Compute gamma_ref and chi_ref from LOS geometry.

        gamma_los = atan2(dh, dx)
        Near the target, smoothly blend toward the desired impact angle.
        """
        dx = self.target_x - x_pos
        dh = self.target_h - altitude
        dy = -y_pos

        range_to_target = math.sqrt(dx * dx + dy * dy)

        # If past the target (dx < 0), LOS geometry still points toward it.
        # dx_safe only protects division/atan2, kept at small positive epsilon.
        dx_safe = max(dx, 0.5)

        # --- Pitch: LOS angle ---
        gamma_los = math.atan2(dh, max(range_to_target, 1.0))
        # If overshot (dx < 0): force steep dive toward target altitude
        if dx < 0.0:
            gamma_los = max(math.radians(-60.0), math.atan2(dh, max(range_to_target, 1.0)))

        # --- Impact angle shaping ---
        impact_rad = math.radians(self._impact_angle_deg)
        progress = 1.0 - dx_safe / max(self.target_x, 1.0)  # 0=launch, 1=target
        progress = max(0.0, min(1.0, progress))

        p0 = self._impact_blend_start  # 0.93
        p1 = self._impact_blend_end    # 0.995
        if p1 <= p0 + 1e-3:
            p1 = p0 + 0.05  # guard: prevent div-by-zero
        if progress < p0:
            k_impact = 0.0
        elif progress > p1:
            k_impact = 1.0
        else:
            s = (progress - p0) / (p1 - p0)
            k_impact = 3.0 * s * s - 2.0 * s * s * s  # smooth Hermite

        gamma_ref = (1.0 - k_impact) * gamma_los + k_impact * impact_rad

        # Clamp to safe range
        gamma_min = math.radians(min(-45.0, self._impact_angle_deg))
        gamma_ref = max(gamma_min, min(math.radians(15.0), gamma_ref))

        # --- Heading: LOS to target (use true dx, not clamped) ---
        # Chi reference — use actual dx (with tiny guard). See los_guidance.cpp
        # for the overshoot case rationale: clamping dx to 0.5 at negative dx
        # would rotate chi_ref sideways instead of back toward the target.
        dx_chi = dx
        if abs(dx_chi) < 0.5:
            dx_chi = 0.5 if dx >= 0.0 else -0.5
        chi_ref = math.atan2(dy, dx_chi)

        return gamma_ref, chi_ref, dx_safe

    # ==================================================================
    #  MPC state extraction
    # ==================================================================

    def _extract_mpc_state(self, state_dict):
        quat = state_dict['quaternion']
        roll, pitch_angle, yaw_angle = quaternion_to_euler(quat)

        vn = state_dict['vel_ned_launch']
        vx, vy, vz = vn[0], vn[1], vn[2]
        V = max(math.sqrt(vx * vx + vy * vy + vz * vz), 10.0)

        if V > 10.0:
            v_h = math.sqrt(vx**2 + vy**2)
            gamma = math.atan2(-vz, v_h)
            chi = math.atan2(vy, vx)
            alpha = pitch_angle - gamma
            # beta_body = -(ψ - χ): body-frame sideslip convention.
            # Positive beta = wind from right (v_body > 0).
            # This matches the model's beta_dot = -r - γ̇·sinφ + χ̇·cosγ·cosφ
            # and the negated Cm_yaw = -(Cm(β)) for restoring yaw moment.
            beta = -(yaw_angle - chi)
            beta = math.atan2(math.sin(beta), math.cos(beta))
        else:
            gamma = pitch_angle
            chi = yaw_angle
            alpha = 0.0
            beta = 0.0

        omega = state_dict['angular_velocity']
        phi = float(roll)

        # NX=15: no servo dynamics in model — delta_s should reflect ACTUAL
        # fin positions to reduce model-plant mismatch. With model 2 servo
        # (pure transport delay), actual positions differ significantly
        # from last commanded values. Always use actuator_positions when available.
        de_fin = self._last_delta_e
        dr_fin = self._last_delta_r
        da_fin = self._last_delta_a
        if 'actuator_positions' in state_dict:
            ap = state_dict['actuator_positions']
            de_fin = 0.25 * (-ap[0] - ap[1] + ap[2] + ap[3])
            dr_fin = 0.25 * (-ap[0] + ap[1] + ap[2] - ap[3])
            da_fin = 0.25 * ( ap[0] + ap[1] + ap[2] + ap[3])

        base = np.array([
            V, gamma, chi,
            float(omega[0]), float(omega[1]), float(omega[2]),
            alpha, beta, phi,
            self._cur_alt / self._H_SCALE,
            self._cur_x / 1000.0,
            self._cur_y / 1000.0,
            de_fin, dr_fin, da_fin,
        ])
        return base

    def _get_params(self, t):
        if t < self.burn_time:
            f = t / self.burn_time
            mass = self.mass_full - f * (self.mass_full - self.mass_dry)
            # Analytical thrust model: plateau + cubic tail-off
            t_tailoff_start = self.burn_time - self._t_tail
            if t > t_tailoff_start:
                s = (t - t_tailoff_start) / self._t_tail  # 0→1
                thrust = self.thrust_plateau * (1.0 - s) ** 3
            else:
                thrust = self.thrust_plateau
        else:
            mass = self.mass_dry
            thrust = 0.0
        return np.array([mass, thrust])

    def _get_mhe_params(self, t):
        """Return [mass, thrust, Ixx, Iyy, Izz, launch_alt] for MHE parameter vector."""
        p = self._get_params(t)
        mass, thrust = p[0], p[1]
        if t < self.burn_time:
            f = t / self.burn_time
            Ixx = self.Ixx_full - f * (self.Ixx_full - self.Ixx_dry)
            Iyy = self.Iyy_full - f * (self.Iyy_full - self.Iyy_dry)
            Izz = self.Izz_full - f * (self.Izz_full - self.Izz_dry)
        else:
            Ixx, Iyy, Izz = self.Ixx_dry, self.Iyy_dry, self.Izz_dry
        return np.array([mass, thrust, Ixx, Iyy, Izz, self._launch_alt])

    # ==================================================================
    #  Forward Euler initial guess
    # ==================================================================

    def _forward_guess(self, x0, t0):
        N = self.N_horizon
        dt_h = self.tf / N
        x_traj = np.zeros((N + 1, NX))
        x_traj[0] = x0.copy()

        for k in range(N):
            x = x_traj[k]
            t_k = t0 + k * dt_h
            p = self._get_params(t_k)
            mass, thrust = p[0], p[1]
            V_k = max(x[0], 10.0)
            gam = x[1]
            sg, cg = math.sin(gam), math.cos(gam)
            al = x[6]

            V_dot = (thrust * math.cos(al) - 0.5 * 1.225 * V_k**2 * 0.0133 * 0.30
                     - mass * _G * sg) / mass
            gam_dot = -_G * cg / V_k + thrust * math.sin(al) / (mass * V_k)

            xn = x.copy()
            xn[0] = max(V_k + V_dot * dt_h, 10.0)
            xn[1] = gam + gam_dot * dt_h
            xn[3:9] *= 0.95
            xn[9]  = x[9]  + V_k * sg * 0.01 * dt_h
            xn[10] = x[10] + V_k * cg * math.cos(x[2]) * 0.001 * dt_h
            xn[11] = x[11] + V_k * cg * math.sin(x[2]) * 0.001 * dt_h
            # No servo dynamics in NX=15 model: delta_s_dot = u (rate command)
            x_traj[k+1] = xn
        return x_traj

    # ==================================================================
    #  Forward prediction for servo delay compensation
    # ==================================================================

    def _predict_ahead(self, x_mpc, t, dt_pred):
        """
        Smith predictor for servo delay compensation.

        Strategy: Use the MPC's own previous solution trajectory to predict
        the dynamic state (V, gamma, rates, alpha, fin positions) that will
        exist when the servo actually executes our command (dt_pred seconds
        from now). We take these from stage `la_stage` of the previous
        solution. But we KEEP the current position (h, x, y) because the
        lookahead state has positions from a stale plan.

        This is a true Smith predictor: the model predicts its own future
        dynamics, and we plan for that future instead of the present.

        Args:
            x_mpc: current state vector (NX=15)
            t: current time
            dt_pred: servo transport delay (s)

        Returns:
            x_pred: predicted state at t + dt_pred
        """
        if dt_pred < 1e-4:
            return x_mpc.copy()

        # Determine which MPC stage corresponds to the delay
        dt_h = self.tf / self.N_horizon
        la_stage = max(1, min(int(round(dt_pred / dt_h)), self.N_horizon - 1))

        # Use the MPC's own previous solution to get the predicted dynamic
        # state at stage `la_stage`. Keep current positions (h, x, y).
        if self._warm and self._solve_count > 0:
            try:
                x_la = self._solver.get(la_stage, "x")
                if np.all(np.isfinite(x_la)):
                    x_pred = x_mpc.copy()
                    # Take dynamic states from lookahead: V, gamma, chi,
                    # p, q, r, alpha, beta, phi, delta_e, delta_r, delta_a
                    # Indices: 0-8, 12-14
                    for idx in [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 13, 14]:
                        x_pred[idx] = x_la[idx]
                    # Keep positions from current state: h(9), x(10), y(11)
                    return x_pred
            except Exception:
                pass

        # Fallback: simple fin-state adjustment
        x_pred = x_mpc.copy()
        x_pred[12] = self._last_delta_e
        x_pred[13] = self._last_delta_r
        x_pred[14] = self._last_delta_a
        return x_pred

    # ==================================================================
    #  Main control function
    # ==================================================================

    def control_function(self, state_dict: dict, t: float) -> np.ndarray:
        zero = np.zeros(4)

        if t <= self.t_ctrl:
            self._last_t = t
            return zero

        if not self._solver_ready:
            return zero

        V_vec = state_dict['velocity']
        airspeed = np.linalg.norm(V_vec)
        if airspeed < 10.0:
            self._last_t = t
            return zero

        if self._last_solve_t >= 0 and (t - self._last_solve_t) < self._dt_solve:
            return self._last_fins.copy()

        # -- Position extraction (long-range ECEF → NED) --
        pos = state_dict['position']
        alt_asl = state_dict['altitude_km'] * 1000.0
        altitude = alt_asl - self._launch_alt
        d_ecef = pos - self._launch_ecef
        ned = self._C_ecef_to_ned @ d_ecef
        x_pos, y_pos = ned[0], ned[1]

        self._cur_alt = altitude
        self._cur_x = x_pos
        self._cur_y = y_pos

        x_mpc = self._extract_mpc_state(state_dict)

        # --- Forward prediction for servo delay compensation ---
        # Predict the state that will exist when the servo actually executes
        # our command (τ_d seconds in the future). MPC then plans for that
        # future state instead of the current one, compensating for the delay.
        if self._servo_delay_s > 1e-4:
            x_mpc = self._predict_ahead(x_mpc, t, self._servo_delay_s)

        # Prune old entries from command buffer (keep only last 2s)
        cutoff = t - 2.0
        self._cmd_buffer = [e for e in self._cmd_buffer if e[0] > cutoff]

        # --- MHE integration: blend estimated state into MPC ---
        # Selective blend matching PX4's cooperative blend strategy:
        #   BLEND from MHE:  gamma, chi, h, x, y  (GPS-constrained, accurate)
        #   KEEP  from sim:  V, alpha, beta, phi   (MHE phi drifts without magnetometer)
        # Gate: w_norm < 5.0 (model-data fit is good).
        #   During boost (w_norm 6-7): blend=0 — thrust model mismatch.
        #   After burnout (w_norm < 4): blend=1 — GPS dominates.
        mhe_out = state_dict.get('mhe_output')
        if mhe_out is not None and getattr(mhe_out, 'valid', False):
            if self._use_estimation and mhe_out.quality >= self._quality_gate_thr:
                xh = mhe_out.x_hat
                wn = getattr(mhe_out, 'w_norm', 99.0)

                # EMA on w_norm: smooth solver noise before smoothstep.
                WN_EMA_ALPHA = 0.15
                self._wn_smooth = WN_EMA_ALPHA * wn + (1.0 - WN_EMA_ALPHA) * self._wn_smooth
                wn = self._wn_smooth

                # w_norm gate: smoothstep ramp 5→2
                WN_HI, WN_LO = 5.0, 2.0
                if wn <= WN_LO:
                    blend = 1.0
                elif wn < WN_HI:
                    s = (WN_HI - wn) / (WN_HI - WN_LO)
                    blend = s * s * (3.0 - 2.0 * s)  # smoothstep
                else:
                    blend = 0.0

                # Altitude fade-out: reduce blend below 20m AGL, zero below 5m
                alt_agl = x_mpc[9] * self._H_SCALE
                alt_hi, alt_lo = 20.0, 5.0
                if alt_agl < alt_hi:
                    alt_frac = max(0.0, (alt_agl - alt_lo) / (alt_hi - alt_lo))
                    blend *= alt_frac

                self._blend_alpha = blend

                if blend > 0.0:
                    x_mpc[1]  = (1.0 - blend) * x_mpc[1]  + blend * xh[1]   # gamma
                    x_mpc[2]  = (1.0 - blend) * x_mpc[2]  + blend * xh[2]   # chi
                    x_mpc[9]  = (1.0 - blend) * x_mpc[9]  + blend * xh[9]   # h
                    x_mpc[10] = (1.0 - blend) * x_mpc[10] + blend * xh[10]  # x
                    x_mpc[11] = (1.0 - blend) * x_mpc[11] + blend * xh[11]  # y
                # Update internal position tracking from blended estimate
                self._cur_alt = x_mpc[9] * self._H_SCALE
                self._cur_x = x_mpc[10] * 1000.0
                self._cur_y = x_mpc[11] * 1000.0

        if not np.all(np.isfinite(x_mpc)):
            print(f"MPC state NaN at t={t:.2f}")
            return self._last_fins.copy() if np.any(self._last_fins) else zero

        V = x_mpc[0]
        gamma = x_mpc[1]

        # Initialize gamma_ref_prev from actual gamma on first call
        if not self._warm:
            self._gamma_ref_prev = gamma
        chi = x_mpc[2]
        phi = x_mpc[8]

        # ==========================================================
        # 1. LOS Guidance — gamma_ref from pure geometry (no feedback loop)
        # ==========================================================
        gamma_ref_los, chi_ref_los, dx_to_target = self._compute_los_guidance(
            x_pos, y_pos, altitude
        )

        # Near impact (< 50 m): hold last commands — MPC optimization is
        # pointless for < 0.3 s and causes QP MINSTEP warnings.
        if dx_to_target < 50.0 and dx_to_target > 0 and self._warm:
            self._last_t = t
            return self._last_fins.copy()

        # Rate-limit gamma_ref to prevent sudden jumps from the geometric LOS
        # (e.g. when dx shrinks rapidly near the target). This is a reference
        # shaping hygiene step, not a controller — MPC itself handles all
        # dynamics via its weights and constraints.
        # gamma_ref rate limit: 25°/s (was 10°/s). The lower limit was tuned
        # for MHE-blended γ which was already biased steep; with truth-state
        # γ, 10°/s starves the terminal pitch-down (only ~10° of change in
        # the last 1s of flight). 25°/s leaves headroom for impact_angle
        # tracking while staying below MPC's pitch-rate authority.
        _MAX_GREF_RATE = math.radians(18.0)
        dt_ref = max(t - self._last_t, 1e-3)
        max_dg = _MAX_GREF_RATE * dt_ref

        prev_g = self._gamma_ref_prev
        prev_c = self._chi_ref_prev

        # During boost: track γ_natural (the flight-path angle naturally
        # produced by thrust at launch pitch). After burnout: hold level flight
        # (γ=0) until cruise_progress of range, then blend to γ_LOS for dive.
        t_rel_burn = t - (self.burn_time - 0.5)
        s_burn = 0.5 * (1.0 + math.tanh(t_rel_burn / 0.8))

        # Cruise→dive blend: 0=level flight, 1=full LOS
        progress = 1.0 - dx_to_target / max(self.target_x, 1.0)
        p0 = self._cruise_progress       # default 0.65 — start dive
        p1 = min(p0 + 0.10, 0.95)        # p0 + 0.10 — full LOS
        if progress < p0:
            s_dive = 0.0
        elif progress > p1:
            s_dive = 1.0
        else:
            s = (progress - p0) / (p1 - p0)
            s_dive = 3.0 * s * s - 2.0 * s * s * s  # smooth Hermite

        # Boost→cruise: γ_natural → level (γ=0)
        # Cruise→terminal: level → γ_LOS (dive)
        # Capture cruise altitude once at burnout — lock immediately
        if s_burn > 0.95 and self._cruise_alt_target is None:
            self._cruise_alt_target = altitude
        gamma_cruise = s_dive * gamma_ref_los
        gamma_ref_raw = (1.0 - s_burn) * self._gamma_natural_rad + s_burn * gamma_cruise
        chi_ref_raw   = chi_ref_los

        dg = gamma_ref_raw - prev_g
        dc = chi_ref_raw - prev_c
        gamma_ref = prev_g + max(-max_dg, min(max_dg, dg))
        chi_ref   = prev_c + max(-max_dg, min(max_dg, dc))
        self._gamma_ref_prev = gamma_ref
        self._chi_ref_prev   = chi_ref

        phi_ref = 0.0
        roll_abs = abs(phi)

        if roll_abs <= self._roll_recovery_start:
            roll_recovery = 0.0
        elif roll_abs >= self._roll_recovery_full:
            roll_recovery = 1.0
        else:
            roll_recovery = (
                (roll_abs - self._roll_recovery_start)
                / (self._roll_recovery_full - self._roll_recovery_start)
            )

        # ==========================================================
        # 3. MPC Setup
        # ==========================================================
        N = self.N_horizon
        solver = self._solver
        # Dynamically shorten horizon near target to prevent infeasibility
        if V > 30.0 and dx_to_target > 0:
            t_to_target = dx_to_target / V
            tf_use = max(2.0, min(self.tf, t_to_target * 0.8))
        else:
            tf_use = self.tf
        dt_h = tf_use / N

        solver.set(0, "lbx", x_mpc)
        solver.set(0, "ubx", x_mpc)

        # Weights — blended between altitude-hold and LOS guidance
        # Tail-off zone: thrust drops rapidly over `t_tail` seconds before
        # nominal burn_time (see get_params() thrust curve, which uses the
        # same window). During this zone, reduce gamma tracking (no thrust
        # to execute commands) and increase damping to suppress oscillations.
        # Post-burnout the window extends 2*t_tail to let weights settle.
        is_boost = (t < self.burn_time)
        t_tailoff_start = self.burn_time - self._t_tail
        t_tailoff_end = self.burn_time + 2.0 * self._t_tail
        in_tailoff = t_tailoff_start < t < t_tailoff_end

        if is_boost and not in_tailoff:
            gamma_base = 250.0      # dominant γ tracking during boost
            q_w = 200.0             # strong pitch damping
            r_w = 80.0
            de_rate_w = 120.0
            dr_rate_w = 60.0
            alpha_w_boost = 120.0
        elif in_tailoff:
            # Tail-off transition: heavily damp around burnout, then fade to coast.
            # Phase 1 (s_to 0→0.67, ~2s): ramp damping up to peak (around burnout)
            # Phase 2 (s_to 0.67→1.0, ~1s): smooth fade to coast values
            # This eliminates the weight discontinuity at the tailoff→coast boundary.
            s_to = min(1.0, (t - t_tailoff_start) / (t_tailoff_end - t_tailoff_start))
            s_peak = 0.67  # peak damping ~1s after burnout
            s_up = min(1.0, s_to / s_peak)  # 0→1 over first 67%

            # Active tailoff weights (peak at s_up=1)
            gamma_act = 250.0 * (1.0 - s_up) ** 2 + 2.0  # 252 → 2
            q_act     = 200.0 + 800.0 * s_up              # 200 → 1000
            r_act     = 80.0 + 200.0 * s_up
            de_act    = 120.0 * (1.0 - s_up) + 200.0 * s_up
            dr_act    = 60.0 + 100.0 * s_up
            alpha_act = 120.0 + 800.0 * s_up              # 120 → 920

            if s_to > s_peak:
                # Hermite fade from peak → coast (zero derivative at endpoints)
                s_f = (s_to - s_peak) / (1.0 - s_peak)
                h = 3.0 * s_f * s_f - 2.0 * s_f * s_f * s_f
                gamma_base   = gamma_act + (100.0 - gamma_act) * h
                q_w          = q_act     + (60.0  - q_act)     * h
                r_w          = r_act     + (40.0  - r_act)     * h
                de_rate_w    = de_act    + (20.0  - de_act)    * h
                dr_rate_w    = dr_act    + (20.0  - dr_act)    * h
                alpha_w_boost = alpha_act + (40.0  - alpha_act) * h
            else:
                gamma_base    = gamma_act
                q_w           = q_act
                r_w           = r_act
                de_rate_w     = de_act
                dr_rate_w     = dr_act
                alpha_w_boost = alpha_act
        else:
            gamma_base = 100.0
            q_w = 60.0
            r_w = 40.0
            de_rate_w = 20.0
            dr_rate_w = 20.0
            alpha_w_boost = 80.0  # higher alpha penalty during coast (was 40)

        # Adaptive damping: when gamma is near target but pitch rate is high,
        # boost q_w to prevent overshoot (the root cause of t=3-4s divergence).
        # Also boost alpha_w when |alpha| starts growing to catch divergence early.
        gamma_err_deg = abs(math.degrees(gamma - self._gamma_ref_prev))
        alpha_deg_abs = abs(math.degrees(x_mpc[6]))  # alpha state
        q_rate_abs = abs(math.degrees(x_mpc[4]))      # pitch rate state
        if is_boost and not in_tailoff:
            if gamma_err_deg < 5.0 and q_rate_abs > 3.0:
                q_w = q_w + 300.0 * (1.0 - gamma_err_deg / 5.0)
            if alpha_deg_abs > 1.0:
                alpha_w_boost = alpha_w_boost + 200.0 * min(alpha_deg_abs / 3.0, 3.0)
        else:
            # Coast/terminal: boost alpha penalty when alpha grows (delay makes this worse)
            if alpha_deg_abs > 2.0:
                alpha_w_boost = alpha_w_boost + 400.0 * min(alpha_deg_abs / 5.0, 4.0)
        # Altitude hold weight: active during cruise, fades out during dive
        # Altitude hold weight: active during cruise, stronger right after burnout to arrest climb
        if not is_boost and self._cruise_alt_target is not None:
            t_since_burn = t - self.burn_time
            # Stronger weight early (arrest climb momentum), settles to base
            early_boost = max(0.0, 1.0 - t_since_burn / 3.0)  # fades over 3s
            h_cruise_w = (15.0 + 35.0 * early_boost) * (1.0 - s_dive)
        else:
            h_cruise_w = 0.0
        h_ref_scaled = (self._cruise_alt_target / self._H_SCALE) if self._cruise_alt_target is not None else 0.0

        # chi tracking — مرفوع 3× لمواجهة انجراف الرياح بقوة (was max(25, 120*..))
        # رياح 8 m/s = ~110m انجراف بـ chi_w=120. مع 360 → تعويض أفضل (-44m).
        chi_w = max(75.0, 360.0 * (1.0 - 0.55 * roll_recovery))
        p_w = 20.0 + 160.0 * roll_recovery
        phi_w = 80.0 + 520.0 * roll_recovery
        da_rate_w = 10.0 - 7.0 * roll_recovery
        W = np.diag([
            h_cruise_w,                     # h — altitude hold during cruise
            gamma_base + 400.0,              # gamma — tracks flight path angle
            chi_w,                           # chi — heading
            p_w, q_w, r_w,                  # p, q, r — strong pitch/yaw damping
            alpha_w_boost,                   # alpha — phase-scheduled
            400.0,                           # beta — مرفوع 6.6× (was 60→240→400) — توازن بين قمع β وانجراف cross-range
            phi_w,                           # phi — bank angle
            de_rate_w, dr_rate_w, da_rate_w, # control rates
        ])

        W_e = np.diag([
            h_cruise_w * 0.5,          # h — terminal altitude
            300.0,                 # gamma
            max(40.0, chi_w),      # chi
            max(40.0, p_w), 60.0, 40.0,
            30.0, 150.0, max(40.0, phi_w),  # alpha,beta,phi: beta 40→50 تغليظ للـterminal yaw align
        ])

        # References: constant gamma_ref for all nodes, rate-limited between calls
        for k in range(N):
            t_k = t + k * dt_h
            solver.set(k, "p", self._get_params(t_k))
            solver.cost_set(k, "W", W)

            yref = np.zeros(12)
            yref[0] = h_ref_scaled
            yref[1] = gamma_ref
            yref[2] = chi_ref
            yref[8] = phi_ref
            solver.set(k, "yref", yref)

        solver.set(N, "p", self._get_params(t + self.tf))
        solver.cost_set(N, "W", W_e)

        yref_e = np.zeros(9)
        yref_e[0] = h_ref_scaled
        yref_e[1] = gamma_ref
        yref_e[2] = chi_ref
        yref_e[8] = phi_ref
        solver.set(N, "yref", yref_e)

        solver.set_new_time_steps(np.full(N, dt_h))

        # Warm start / initial guess
        if not self._warm:
            x_traj = self._forward_guess(x_mpc, t)
            for k in range(N + 1):
                solver.set(k, "x", x_traj[k])
            for k in range(N):
                solver.set(k, "u", np.zeros(NU))
        else:
            for k in range(N - 1):
                solver.set(k, "x", solver.get(k + 1, "x"))
                solver.set(k, "u", solver.get(k + 1, "u"))
            solver.set(N - 1, "x", solver.get(N, "x"))
        solver.set(0, "x", x_mpc)

        # ==========================================================
        # 4. Solve
        # ==========================================================
        n_rti = 5 if not self._warm else 3
        if self._consec_fails > 3:
            n_rti = 8
        ok = True
        status = 0
        for _ in range(n_rti):
            status = solver.solve()
            if status not in (0, 2):
                ok = False
                break

        if ok and not self._warm:
            self._consec_ok += 1
            if self._consec_ok >= 3:
                self._warm = True
        elif not ok:
            self._warm = False
            self._consec_ok = 0

        self._solve_count += 1

        u_opt = solver.get(0, "u")
        la = self._lookahead_stage
        if la < 1: la = 1
        if la > self.N_horizon: la = self.N_horizon
        x_la = solver.get(la, "x")
        valid = np.all(np.isfinite(u_opt)) and np.all(np.isfinite(x_la))

        if not ok or not valid:
            self._consec_fails += 1
            # Hold last good commands (don't decay — maintains terminal guidance)
            de = self._last_delta_e
            dr = self._last_delta_r
            da = self._last_delta_a
            if self._consec_fails >= 10:
                self._reinit(x_mpc)
        else:
            self._consec_fails = 0
            de = float(x_la[12])
            dr = float(x_la[13])
            da = float(x_la[14])

        self._last_delta_e = de
        self._last_delta_r = dr
        self._last_delta_a = da

        # Add command to buffer for forward prediction
        self._cmd_buffer.append((t, de, dr, da))
        self._last_t = t
        self._last_solve_t = t

        # Store solver diagnostics for snapshot recording | تخزين تشخيصات الحل للتسجيل
        self._last_solve_time_ms = solver.get_stats("time_tot") * 1e3
        self._last_solver_status = int(status)
        self._last_sqp_iterations = int(solver.get_stats("sqp_iter"))

        # Logging
        if self._solve_count <= 5 or self._solve_count % 50 == 0:
            t_tot = self._last_solve_time_ms
            sqp_it = self._last_sqp_iterations
            progress = 1.0 - dx_to_target / max(self.target_x, 1.0)
            print(
                f"MPC[{self._solve_count}] t={t:.2f} V={V:.0f} "
                f"gam={math.degrees(gamma):.1f} "
                f"roll={math.degrees(phi):.1f} "
                f"gref={math.degrees(gamma_ref):.1f} "
                f"alt={altitude:.0f} dx={dx_to_target:.0f} "
                f"prog={progress:.0%} rr={roll_recovery:.2f} "
                f"de={math.degrees(de):.1f} dr={math.degrees(dr):.1f} da={math.degrees(da):.1f} "
                f"st={status} {t_tot:.1f}ms it={sqp_it}"
            )

        # Fin mixing (X config)
        fins = np.array([
            da - de - dr,
            da - de + dr,
            da + de + dr,
            da + de - dr,
        ])
        self._last_fins = fins.copy()
        return fins

    def _reinit(self, x_mpc):
        self._solver.reset(reset_qp_solver_mem=1)
        x_traj = self._forward_guess(x_mpc, self._last_t)
        for k in range(self.N_horizon + 1):
            self._solver.set(k, "x", x_traj[k])
        for k in range(self.N_horizon):
            self._solver.set(k, "u", np.zeros(NU))
        self._warm = False
        self._consec_fails = 0
        self._consec_ok = 0
        print("MPC solver reinitialized")

