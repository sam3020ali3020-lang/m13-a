"""
MHE Estimator wrapper for M130 rocket — sliding window + arrival cost + fallback.

Usage:
    estimator = MheEstimator(sim_config)
    ...
    mhe_out = estimator.update(sensor_packet, control_input, params, t)
    x_hat = mhe_out.x_hat       # [17] estimated state
    d_hat = mhe_out.d_hat       # {'gyro_bias': [3], 'wind_ne': [2]}
"""

import logging
import os
import sys
import time
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np

# Ensure mpc/ directory is on sys.path for sibling imports
_MPC_DIR = str(Path(__file__).resolve().parent)
if _MPC_DIR not in sys.path:
    sys.path.insert(0, _MPC_DIR)

logger = logging.getLogger(__name__)

# State indices
IX_V, IX_GAMMA, IX_CHI = 0, 1, 2
IX_P, IX_Q, IX_R = 3, 4, 5
IX_ALPHA, IX_BETA, IX_PHI = 6, 7, 8
IX_H, IX_XG, IX_YG = 9, 10, 11
IX_BGX, IX_BGY, IX_BGZ = 12, 13, 14
IX_WN, IX_WE = 15, 16

NX_MHE = 17
NW_MHE = 17
NY_MEAS = 13


@dataclass
class MheOutput:
    x_hat: np.ndarray          # [17] estimated state
    d_hat: dict                # {'gyro_bias': [3], 'wind_ne': [2]}
    quality: float = 1.0       # 0.0 (bad) → 1.0 (excellent)
    w_norm: float = 0.0        # process noise RMS — low = good model-data fit
    status: int = 0            # 0=OK, 2=maxiter, 4=QP_fail
    solve_time_ms: float = 0.0
    valid: bool = True


class MheEstimator:
    """Moving Horizon Estimator using acados OCP solver."""

    def __init__(self, estimation_cfg: dict):
        from m130_mhe_ocp_setup import create_m130_mhe_solver

        mhe_cfg = estimation_cfg.get("mhe", {})
        self._N = int(mhe_cfg.get("horizon_steps", 20))
        self._dt = float(mhe_cfg.get("horizon_dt", 0.02))
        self._solve_rate_hz = float(mhe_cfg.get("solve_rate_hz", 50.0))
        self._solve_period = 1.0 / self._solve_rate_hz
        self._max_consec_fails = int(mhe_cfg.get("max_consecutive_failures", 10))
        self._quality_gate = float(mhe_cfg.get("quality_gate_threshold", 0.3))
        # Minimum measurements before first solve (default: half the horizon)
        self._min_init_measurements = int(mhe_cfg.get("min_init_measurements", max(self._N // 2, 5)))
        # Number of solves over which startup quality ramps from 0→1
        self._startup_ramp_solves = int(mhe_cfg.get("startup_ramp_solves", 5))

        self._solver = create_m130_mhe_solver(
            estimation_cfg=estimation_cfg,
            json_file="m130_mhe_ocp.json",
        )

        # Pre-compute per-stage weight matrices for GPS freshness gating.
        # W = block_diag(R, Q) [30×30];  W_0 = block_diag(R, Q, P0) [47×47]
        # GPS occupies y[7..12] → R rows/cols 7..12 → W rows/cols 7..12.
        # When GPS is stale (repeated sample), zero those entries so the
        # solver ignores the duplicated GPS data and relies on IMU + baro
        # + dynamics alone for that stage.
        self._W_fresh = np.array(self._solver.acados_ocp.cost.W)      # 30×30
        self._W_stale = self._W_fresh.copy()
        self._W_stale[7:13, :] = 0.0
        self._W_stale[:, 7:13] = 0.0
        self._W0_fresh = np.array(self._solver.acados_ocp.cost.W_0)   # 47×47
        self._W0_stale = self._W0_fresh.copy()
        self._W0_stale[7:13, :] = 0.0
        self._W0_stale[:, 7:13] = 0.0

        # Sliding window buffers
        self._meas_buf = []    # list of (t, y_meas[13], gps_fresh)
        self._ctrl_buf = []    # list of (t, u_fins[3])
        self._param_buf = []   # list of (t, p_full[9])

        # Arrival cost state
        self._x_bar = None     # arrival mean [17]

        # Fallback / safety
        self._consec_fails = 0
        self._last_valid = None
        self._last_solve_t = -1.0
        self._initialized = False
        self._solve_count = 0  # track solves for startup ramp

        logger.info(
            f"MheEstimator created: N={self._N}, dt={self._dt}s, "
            f"solve_rate={self._solve_rate_hz}Hz, "
            f"min_init_meas={self._min_init_measurements}"
        )

    def ready_to_init(self) -> bool:
        """Check if enough measurements have been collected to initialize."""
        return len(self._meas_buf) >= self._min_init_measurements

    def init_state(self, x0: np.ndarray):
        """Initialize estimator with a known state (e.g. from truth at t=0)."""
        x0 = np.asarray(x0, dtype=float).ravel()
        if x0.size != NX_MHE:
            raise ValueError(f"Expected {NX_MHE} states, got {x0.size}")
        self._x_bar = x0.copy()
        self._last_valid = MheOutput(
            x_hat=x0.copy(),
            d_hat={"gyro_bias": x0[IX_BGX:IX_BGZ + 1].copy(),
                   "wind_ne": x0[IX_WN:IX_WE + 1].copy()},
        )
        self._initialized = True

        # Warm-start all solver nodes
        for k in range(self._N + 1):
            self._solver.set(k, "x", x0)
        for k in range(self._N):
            self._solver.set(k, "u", np.zeros(NW_MHE))

    def push_measurement(self, t: float, y_meas: np.ndarray, gps_fresh: bool = True):
        """Add a sensor measurement vector [13] to the sliding window.

        Args:
            t: timestamp (s)
            y_meas: measurement vector [13]
            gps_fresh: True if GPS was updated this cycle (new sample),
                       False if y[7..12] are a repeat of the previous sample.
        """
        self._meas_buf.append((t, np.asarray(y_meas, dtype=float).ravel(), gps_fresh))

    def push_control_and_params(self, t: float, u_fins: np.ndarray,
                                params: np.ndarray):
        """Add control input [3] and model parameters [9] to the window."""
        self._ctrl_buf.append((t, np.asarray(u_fins, dtype=float).ravel()))
        self._param_buf.append((t, np.asarray(params, dtype=float).ravel()))

    def update(self, t: float) -> MheOutput:
        """
        Run MHE solve if enough data in the window and solve period elapsed.

        Returns:
            MheOutput with current best estimate.
        """
        if not self._initialized:
            return self._frozen_output("not initialized")

        # Rate limiting
        if self._last_solve_t >= 0 and (t - self._last_solve_t) < self._solve_period * 0.9:
            return self._last_valid

        n_available = len(self._meas_buf)
        if n_available < 2:
            return self._last_valid

        # Use at most N+1 measurements (window)
        n_use = min(n_available, self._N + 1)

        # Trim buffers to keep only needed data
        if n_available > self._N + 5:
            trim = n_available - self._N - 2
            self._meas_buf = self._meas_buf[trim:]
            self._ctrl_buf = self._ctrl_buf[trim:]
            self._param_buf = self._param_buf[trim:]
            n_available = len(self._meas_buf)
            n_use = min(n_available, self._N + 1)

        # Adjust horizon if we have fewer measurements than N+1
        N_use = n_use - 1  # number of intervals

        # --- Fill solver references ---
        meas_window = self._meas_buf[-n_use:]  # last n_use measurements
        ctrl_window = self._ctrl_buf[-(n_use):]  # corresponding controls
        param_window = self._param_buf[-(n_use):]

        # Stage 0: arrival cost + measurement + process noise
        y0 = meas_window[0][1]
        gps_fresh_0 = meas_window[0][2] if len(meas_window[0]) > 2 else True
        x_bar = self._x_bar if self._x_bar is not None else np.zeros(NX_MHE)
        yref_0 = np.concatenate([y0, np.zeros(NW_MHE), x_bar])
        self._solver.set(0, "yref", yref_0)
        # GPS freshness gating: use W_stale when GPS hasn't been updated
        self._solver.cost_set(0, "W", self._W0_fresh if gps_fresh_0 else self._W0_stale)

        # Stages 1..N-1: measurement + noise.  Fill ALL intermediate stages
        # so the NLP cost is well-defined during startup when the window is
        # not yet full.  For stages beyond the current measurement window we
        # repeat the most recent measurement (mirrors the C++ runtime in
        # mhe_estimator.cpp::update to keep behaviour consistent).
        y_last = meas_window[-1][1]
        for k in range(1, self._N):
            if k < len(meas_window):
                yk = meas_window[k][1]
                gps_fresh_k = meas_window[k][2] if len(meas_window[k]) > 2 else True
            else:
                yk = y_last
                gps_fresh_k = False  # repeated → stale
            yref_k = np.concatenate([yk, np.zeros(NW_MHE)])
            self._solver.set(k, "yref", yref_k)
            # GPS freshness gating
            self._solver.cost_set(k, "W", self._W_fresh if gps_fresh_k else self._W_stale)

        # Parameters for every interval 0..N-1 plus the terminal node.
        # Same fallback as the measurement loop: after n_use samples, freeze
        # on the newest param vector so every stage sees a well-defined p.
        p_last = param_window[-1][1] if param_window else np.zeros(9)
        for k in range(self._N):
            if k < len(param_window):
                p_k = param_window[k][1]
            else:
                p_k = p_last
            self._solver.set(k, "p", p_k)

        # Terminal stage parameters
        self._solver.set(self._N, "p", p_last)

        # --- Solve ---
        t0 = time.perf_counter()
        status = self._solver.solve()

        solve_ms = (time.perf_counter() - t0) * 1e3
        self._last_solve_t = t

        # Extract estimate from last node (= current time)
        x_hat = self._solver.get(self._N, "x")

        # Check validity
        if not np.all(np.isfinite(x_hat)):
            self._consec_fails += 1
            logger.warning(f"MHE NaN at t={t:.3f}, fail #{self._consec_fails}")
            if self._consec_fails >= self._max_consec_fails:
                logger.error("MHE frozen: too many consecutive failures")
                return self._frozen_output("NaN flood")
            # Any failure means no fresh estimate: mark INVALID so consumers
            # fall back to EKF immediately.  Returning valid=True with a
            # stale x_hat would let MPC solve on a frozen pose.
            if self._last_valid is None:
                return self._frozen_output("NaN before first valid")
            return replace(self._last_valid, valid=False, status=status)

        # Update arrival cost: shift window
        if N_use >= 2:
            self._x_bar = self._solver.get(1, "x").copy()

        # Quality metric: based on solver status and process noise magnitude
        w_norm = 0.0
        for k in range(min(N_use, self._N)):
            wk = self._solver.get(k, "u")
            w_norm += np.dot(wk, wk)
        w_norm = np.sqrt(w_norm / max(N_use, 1))

        quality = 1.0
        if status in (3, 4):
            quality *= 0.5
        if w_norm > 5.0:
            quality *= max(0.1, 1.0 - (w_norm - 5.0) / 15.0)

        # Bound saturation check: if wind or gyro bias estimates are near
        # their box constraint limits (>90%), it signals model mismatch —
        # the optimizer is pushing unobservable states to absorb errors.
        _WIND_BOUND = 5.0    # يَتَطابق مع lbx/ubx؛ الرياح غير قابلة للتقدير في طيران قصير
        _GBIAS_BOUND = 0.03   # matches acados solver bound (±0.03 rad/s)
        _SAT_THRESH = 0.90  # 90% of bound
        wind_sat = max(abs(x_hat[IX_WN]), abs(x_hat[IX_WE])) / _WIND_BOUND
        gbias_sat = max(abs(x_hat[IX_BGX]), abs(x_hat[IX_BGY]),
                        abs(x_hat[IX_BGZ])) / _GBIAS_BOUND
        max_sat = max(wind_sat, gbias_sat)
        if max_sat > _SAT_THRESH:
            # Linear penalty: 90%→quality*1.0, 100%→quality*0.3
            sat_penalty = max(0.3, 1.0 - (max_sat - _SAT_THRESH) / (1.0 - _SAT_THRESH) * 0.7)
            quality *= sat_penalty

        # Startup ramp: gradually increase quality over first few solves
        self._solve_count += 1
        if self._solve_count <= self._startup_ramp_solves:
            ramp = self._solve_count / self._startup_ramp_solves
            quality *= ramp

        self._consec_fails = 0
        d_hat = {
            "gyro_bias": x_hat[IX_BGX:IX_BGZ + 1].copy(),
            "wind_ne": x_hat[IX_WN:IX_WE + 1].copy(),
        }
        output = MheOutput(
            x_hat=x_hat.copy(),
            d_hat=d_hat,
            quality=quality,
            w_norm=w_norm,
            status=status,
            solve_time_ms=solve_ms,
            valid=True,
        )
        self._last_valid = output
        return output

    def get_last_estimate(self) -> MheOutput:
        """Return the last valid estimate."""
        if self._last_valid is not None:
            return self._last_valid
        return self._frozen_output("no estimate yet")

    def _frozen_output(self, reason: str = "") -> MheOutput:
        if self._last_valid is not None:
            out = MheOutput(
                x_hat=self._last_valid.x_hat.copy(),
                d_hat={k: v.copy() for k, v in self._last_valid.d_hat.items()},
                quality=0.0,
                status=-1,
                solve_time_ms=0.0,
                valid=False,
            )
            return out
        return MheOutput(
            x_hat=np.zeros(NX_MHE),
            d_hat={"gyro_bias": np.zeros(3), "wind_ne": np.zeros(2)},
            quality=0.0,
            status=-1,
            solve_time_ms=0.0,
            valid=False,
        )
