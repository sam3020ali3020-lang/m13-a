#!/usr/bin/env python3
"""
نظام توجيه M130 - M130 Guidance System
========================================

نظام توجيه مخصص لصاروخ M130 يعتمد على:
Custom guidance system for M130 rocket based on:

1. yaw_comm: أمر الانعراج (t < 5) - تصحيح الموقع والسرعة
   Yaw command (t < 5) - position and velocity error correction

2. yaw_comdd2: أمر الانعراج (t >= 5) - ملاحة تناسبية للانعراج
   Yaw command (t >= 5) - proportional navigation for yaw

3. pn2: أمر الميل (جميع المراحل) - ملاحة تناسبية منحازة مع زاوية اصطدام
   Pitch command (ALL phases) - biased proportional navigation with impact angle control

Author: 6DOF Team
Date: 2026-03-04
"""

import math
import numpy as np
from typing import Dict, Any, Optional
import logging

from dynamics.guidance import GuidanceSystem, GuidanceOutput, GuidancePhase

logger = logging.getLogger(__name__)


class M130GuidanceSystem(GuidanceSystem):
    """
    نظام توجيه M130
    M130 Guidance System

    يرث من GuidanceSystem ويستبدل دوال التوجيه بالدوال المخصصة لـ M130:
    Inherits from GuidanceSystem and replaces guidance functions with M130-specific ones:

    - pn2: ملاحة تناسبية منحازة (جميع المراحل) | Biased PN (all phases)
    - yaw_comm: أمر الانعراج (t < 5) | Yaw command (t < 5)
    - yaw_comdd2: أمر الانعراج (t >= 5) | Yaw command (t >= 5)
    """

    def __init__(self, config: Dict[str, Any], **kwargs):
        """
        تهيئة نظام توجيه M130
        Initialize M130 guidance system
        """
        super().__init__(config, **kwargs)

        guidance_config = config.get('guidance', {})
        m130_cfg = guidance_config.get('m130', {})

        # معاملات yaw_comm
        self.m130_k_guidance_yaw = m130_cfg.get('k_guidance_yaw', 0.008)
        self.m130_k_vz = m130_cfg.get('k_vz', 14.0)
        self.m130_ayc_limit = m130_cfg.get('ayc_limit', 2.0)

        # معاملات pn2
        self.m130_Npn = m130_cfg.get('Npn', 3.0)
        self.m130_tau_pn1 = m130_cfg.get('tau_pn1', 15.0)
        self.m130_impact_angle_deg = m130_cfg.get('impact_angle_deg', -60.0)
        self.m130_apc_limit_g = m130_cfg.get('apc_limit', 8.0)

        # ثوابت | Constants
        self._mag_g = 9.80665
        self._d2r = math.pi / 180.0

        # حالة pn2 (مكامل الانحياز) | pn2 state (bias integrator)
        self._pn2_initialized = False
        self._pn2_t_old = 0.0
        self._pn2_Bref = 0.0
        self._pn2_b_int = 0.0
        self._pn2_b1 = 0.0

        logger.info(
            f"M130GuidanceSystem initialized: "
            f"Npn={self.m130_Npn}, tau_pn1={self.m130_tau_pn1}, "
            f"impact_angle={self.m130_impact_angle_deg}°, "
            f"k_guidance_yaw={self.m130_k_guidance_yaw}"
        )

    def yaw_comm(self, Vzm: float, Zm: float) -> float:
        """
        أمر الانعراج - تصحيح الموقع والسرعة (t < 5)
        Yaw command - position and velocity error correction (t < 5)

        Args:
            Vzm: سرعة الصاروخ في اتجاه Z (m/s) | Missile Z velocity (East)
            Zm: موقع الصاروخ Z (m) | Missile Z position (East)

        Returns:
            أمر تسارع الانعراج | Yaw acceleration command
        """
        k_guidance_yaw = self.m130_k_guidance_yaw  # 0.008

        if abs(Vzm) < 30.0:
            k_vz = self.m130_k_vz  # 14.0
        else:
            k_vz = self.m130_k_vz

        vz_err = -k_vz * Vzm
        z_err = self.Ztrgt - Zm

        ayc = 0.8 * k_guidance_yaw * (z_err + vz_err)

        ayc_limit = self.m130_ayc_limit  # 2.0
        if abs(ayc) > ayc_limit:
            ayc = ayc_limit * ayc / abs(ayc)

        return ayc

    def yaw_comdd2(self, Xm: float, Zm: float,
                   Vxm: float, Vzm: float, t: float) -> float:
        """
        أمر الانعراج - ملاحة تناسبية (t >= 5)
        Yaw command - proportional navigation (t >= 5)

        Args:
            Xm: موقع الصاروخ X (m) | Missile X position (Forward/North)
            Zm: موقع الصاروخ Z (m) | Missile Z position (East)
            Vxm: سرعة الصاروخ X (m/s) | Missile X velocity (Forward)
            Vzm: سرعة الصاروخ Z (m/s) | Missile Z velocity (East)
            t: الزمن الحالي (s) | Current time

        Returns:
            أمر تسارع الانعراج | Yaw acceleration command
        """
        dx_tr = self.Xtrgt - Xm   # rtaz(0,0)
        dz_tr = self.Ztrgt - Zm   # rtaz(1,0)

        if t > 5:
            dx_tr = 300.0

        if abs(dx_tr) <= 1.0:
            dx_tr = 1.0

        rxz2 = dx_tr * dx_tr + dz_tr * dz_tr
        if rxz2 < 10000.0:
            rxz2 = 10000.0

        # vmaz(0,0) < 0 → الصاروخ تجاوز الهدف
        if Vxm < 0.0:
            return 0.0

        Vne = math.sqrt(Vxm * Vxm + Vzm * Vzm)
        qe_dot2 = (dz_tr * Vxm - dx_tr * Vzm) / rxz2

        ayc = 0.25 * qe_dot2 * Vne

        ayc_limit = self.m130_ayc_limit
        if abs(ayc) > ayc_limit:
            ayc = ayc_limit * ayc / abs(ayc)

        # تحديد إضافي عند ±2.0 | Additional clamp at ±2.0
        if ayc > 2.0:
            ayc = 2.0
        elif ayc < -2.0:
            ayc = -2.0

        return ayc

    def pn2(self, Xm: float, Ym: float,
            Vxm: float, Vym: float, Vzm: float,
            phi: float, quaternion: np.ndarray, t: float,
            apc_limit_genes: float) -> float:
        """
        أمر الميل - ملاحة تناسبية منحازة مع زاوية اصطدام (جميع المراحل)
        Pitch command - biased PN with impact angle control (ALL phases)

        Args:
            Xm: موقع الصاروخ X (m) | Missile X position (Forward)
            Ym: موقع الصاروخ Y (m) | Missile Y position (Up)
            Vxm: سرعة الصاروخ X (m/s) | Forward velocity
            Vym: سرعة الصاروخ Y (m/s) | Down velocity (NED D)
            Vzm: سرعة الصاروخ Z (m/s) | East velocity
            phi: زاوية الدوران (rad) | Roll angle
            quaternion: الرباعي [q0, q1, q2, q3] | Quaternion
            t: الزمن الحالي (s) | Current time
            apc_limit_genes: حد التسارع من flight_genes | Accel limit from flight_genes

        Returns:
            أمر تسارع الميل (g) | Pitch acceleration command (g-units)
        """
        # dxi = rtaz(0,0), dyi = -rtaz(2,0)
        dxi = self.Xtrgt - Xm   # المدى الأمامي | Forward range
        dyi = -(self.Ytrgt - Ym)   # فرق الارتفاع (موجب = هدف فوق) | Altitude diff (pos = target above)

        impact_angle1 = 0.5 * self.m130_impact_angle_deg  # 0.5 * -60 = -30

        # إيقاف التوجيه إذا تجاوز الهدف أفقياً فقط
        # Stop guidance only if past target horizontally
        if dxi < 0.0:
            return 0.0

        rxy = math.sqrt(dxi * dxi + dyi * dyi)

        if rxy < 10.0:
            rxy = 10.0

        if abs(dxi) < 1.0:
            dxi = 1.0

        # زاوية خط البصر | Line of sight angle
        teta_l = math.atan2(dyi, dxi)
        lambda_angle = teta_l

        # زاوية مسار الطيران | Flight path angle
        # Vym = NED Down, فـ -Vym = Up
        # gama = atan2(-vmaz(2,0), vmaz(0,0)) = atan2(V_up, V_forward)
        gama = math.atan2(Vym, Vxm)

        # السرعة الكلية | Total velocity
        Vm_t = math.sqrt(Vxm * Vxm + Vym * Vym + Vzm * Vzm)
        if Vm_t < 1.0:
            Vm_t = 1.0

        Npn = self.m130_Npn
        tau_pn1 = self.m130_tau_pn1
        mag_g = self._mag_g
        d2r = self._d2r

        # تهيئة الانحياز (أول استدعاء فقط) | Bias initialization (first call only)
        if not self._pn2_initialized:
            self._pn2_initialized = True
            self._pn2_t_old = t
            self._pn2_Bref = (1.0 - Npn) * impact_angle1 * d2r - gama + Npn * lambda_angle
        else:
            E_bias = self._pn2_Bref - self._pn2_b_int
            b1 = E_bias / tau_pn1

            acc_limit = apc_limit_genes
            b1_max = acc_limit * mag_g / Vm_t
            if abs(b1) > b1_max:
                b1 = math.copysign(b1_max, b1)

            dt_pn = t - self._pn2_t_old
            self._pn2_b_int += b1 * dt_pn
            self._pn2_t_old = t
            self._pn2_b1 = b1

        # أمر الملاحة التناسبية | Proportional navigation command
        sin_diff = math.sin(gama - lambda_angle)
        lambda_dot = (-Vm_t * sin_diff) / rxy
        gama_dot = Npn * lambda_dot
        ay_com = gama_dot * Vm_t + self._pn2_b1 * Vm_t

        # تعويض الجاذبية | Gravity compensation
        # Gb = mib * Gi → gravity in body frame
        C_bn = self._quaternion_to_dcm(quaternion)  # body → NED
        C_nb = C_bn.T                                # NED → body
        g_ned = np.array([0.0, 0.0, mag_g])          # gravity in NED (down)
        g_body = C_nb @ g_ned                         # gravity in body frame

        gravity_comp = math.cos(phi) * g_body[2] - math.sin(phi) * g_body[1]
        ac_pitch = (ay_com - gravity_comp) / mag_g

        # تشبع ±8g | Saturate ±8g
        pitch_limit = self.m130_apc_limit_g
        if ac_pitch > pitch_limit:
            ac_pitch = pitch_limit
        elif ac_pitch < -pitch_limit:
            ac_pitch = -pitch_limit

        return ac_pitch

    @staticmethod
    def _quaternion_to_dcm(q: np.ndarray) -> np.ndarray:
        """
        تحويل الرباعي إلى مصفوفة الدوران (body → NED)
        Convert quaternion to Direction Cosine Matrix (body → NED)

        Args:
            q: الرباعي [q0, q1, q2, q3] (scalar first)

        Returns:
            مصفوفة دوران 3×3 | 3×3 rotation matrix
        """
        q0, q1, q2, q3 = q

        C = np.array([
            [q0*q0 + q1*q1 - q2*q2 - q3*q3,  2*(q1*q2 - q0*q3),              2*(q1*q3 + q0*q2)],
            [2*(q1*q2 + q0*q3),                q0*q0 - q1*q1 + q2*q2 - q3*q3,  2*(q2*q3 - q0*q1)],
            [2*(q1*q3 - q0*q2),                2*(q2*q3 + q0*q1),              q0*q0 - q1*q1 - q2*q2 + q3*q3],
        ])
        return C

    def compute(
        self,
        nav_state: Dict,
        t: float,
        motor_on: bool = True,
        density: float = 1.225
    ) -> Optional[GuidanceOutput]:
        """
        حساب أوامر التوجيه لـ M130
        Compute guidance commands for M130

        المنطق | Logic:
        - pn2: أمر الميل في جميع المراحل | Pitch command in all phases
        - yaw_comm: أمر الانعراج عند t < 5 | Yaw command at t < 5
        - yaw_comdd2: أمر الانعراج عند t >= 5 | Yaw command at t >= 5
        """
        if not self.enabled:
            return None

        # تهيئة إذا لم تتم | Initialize if not done
        if not self.state.guidance_initialized:
            self.initialize(nav_state)

        # حساب dt | Calculate dt
        dt = t - self.state.last_update_time
        if dt <= 0:
            dt = 0.01
        self.state.last_update_time = t

        # الحصول على الموقع والسرعة | Get position and velocity
        velocity = nav_state.get('velocity', np.zeros(3))
        velocity_body = nav_state.get('velocity_body', velocity)
        position = nav_state.get('position', np.zeros(3))

        # تحديث جينات الطيران (لجدولة المعاملات → الطيار الآلي)
        # Update flight genes (for gain scheduling → autopilot)
        flight_genes_state = self.flight_genes.update(t, position, velocity, motor_on)

        missile_phase = flight_genes_state.current_phase
        guidance_phase = self._convert_phase(missile_phase)

        # حساب زاوية ألفا | Calculate alpha
        alpha = self.compute_alpha(velocity_body)

        # استخراج الرباعي وزوايا أويلر | Extract quaternion and Euler angles
        quaternion = nav_state.get('quaternion', np.array([1, 0, 0, 0]))
        euler = self._quaternion_to_euler(quaternion)
        phi = euler[0]  # زاوية الدوران | Roll angle

        # استخراج الموقع في إطار FUR | Extract position in FUR frame
        position_fur = nav_state.get('position_fur')
        if position_fur is None:
            position_fur = np.array([position[0], -position[2], position[1]])
        Xm = position_fur[0]  # Forward/North
        Ym = position_fur[1]  # Up
        Zm = position_fur[2]  # Right/East

        # استخراج السرعات | Extract velocities
        vel_aero = nav_state.get('velocity_for_aero', velocity)
        Vxm = vel_aero[0]  # North/Forward
        Vym = vel_aero[2]  # Down (NED D)
        Vzm = vel_aero[1]  # East/Right

        # حد التسارع من flight_genes | Accel limit from flight_genes
        apc_limit_genes = flight_genes_state.gain_pilot_output.apc_limit

        # ===================================================================
        # أمر الميل: pn2 في جميع المراحل
        # Pitch command: pn2 in ALL phases
        # ===================================================================
        pitch_accel_cmd = self.pn2(
            Xm, Ym, Vxm, Vym, Vzm, phi, quaternion, t, apc_limit_genes
        )

        # ===================================================================
        # أمر الانعراج: yaw_comm (t < 5) أو yaw_comdd2 (t >= 5)
        # Yaw command: yaw_comm (t < 5) or yaw_comdd2 (t >= 5)
        # ===================================================================
        if t < 5.0:
            yaw_accel_cmd = self.yaw_comm(Vzm, Zm)
        else:
            yaw_accel_cmd = self.yaw_comdd2(Xm, Zm, Vxm, Vzm, t)

        # ===================================================================
        # أمر الدوران: تثبيت الدوران
        # Roll command: roll stabilization
        # ===================================================================
        current_gains = flight_genes_state.current_gains
        roll_accel_cmd = self._generate_roll_accel_command(
            nav_state, current_gains.roll, dt
        )

        # تخزين بيانات الحالة | Store state data
        self.state.Vxm_pitch = Vxm
        self.state.Vym_pitch = Vym
        self.state.dx_tr_pitch = self.Xtrgt - Xm
        self.state.dy_tr_pitch = self.Ytrgt - Ym

        return GuidanceOutput(
            phase=guidance_phase,
            alpha=alpha,
            yaw_accel_cmd=yaw_accel_cmd,
            pitch_accel_cmd=pitch_accel_cmd,
            roll_accel_cmd=roll_accel_cmd,
            theta_cmd=0.0,
            apc_cmd=0.0,
            Xm=Xm,
            Ym=Ym,
            Zm=Zm,
            Xtrgt=self.Xtrgt,
            Ytrgt=self.Ytrgt,
            Ztrgt=self.Ztrgt,
            Vxm_pitch=Vxm,
            Vym_pitch=Vym,
            dx_tr_pitch=self.Xtrgt - Xm,
            dy_tr_pitch=self.Ytrgt - Ym,
            flight_genes_state=flight_genes_state,
        )

    def reset(self):
        """
        إعادة تعيين نظام التوجيه
        Reset guidance system
        """
        super().reset()
        self._pn2_initialized = False
        self._pn2_t_old = 0.0
        self._pn2_Bref = 0.0
        self._pn2_b_int = 0.0
        self._pn2_b1 = 0.0
        logger.info("M130GuidanceSystem reset")
