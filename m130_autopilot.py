#!/usr/bin/env python3
"""
نظام الطيار الآلي M130 - M130 Autopilot System
================================================

نظام طيار آلي مخصص لصاروخ M130 يتضمن:
Custom autopilot system for M130 rocket including:

1. PilotGain:  جدولة المكاسب حسب المرحلة والضغط الديناميكي
               Stage- and dynamic-pressure-dependent gain scheduling

2. AltitudeAutopilot (المرحلة 1):  تثبيت الارتفاع (PID) للمرحلة الأولى
                     Stage 1: Altitude hold (PID)

3. ApcAutopilot (المرحلة 2+): تتبع تسارع الميل
                Stage 2+: Pitch acceleration tracking

4. AutoPilot:  الدالة الرئيسية - تصفية المستشعرات + 3 قنوات تحكم + تحويل الإطار
               Main function - sensor filtering + 3 control channels + frame transform

المتغيرات متوافقة مع المتغيرات في الكود العام (AutopilotInput / AutopilotOutput).
Variables are compatible with the general code (AutopilotInput / AutopilotOutput).

تحويلات الوحدات (Unit Conversions):
- الضغط الديناميكي: q_dyn [Pa] → q [kgf/cm²] = q_dyn / 98066.5
  Dynamic pressure: q_dyn [Pa] → q [kgf/cm²] = q_dyn / 98066.5
- التسارع الجسمي: [m/s²] → [g] = value / 9.81
  Body acceleration: [m/s²] → [g] = value / 9.81

Author: 6DOF Team
Date: 2026-03-04
"""

import math
import numpy as np
from typing import Dict, Any, Optional
import logging

from dynamics.autopilot import (
    AutopilotInput,
    AutopilotOutput,
    AutopilotMode,
    FilteredSensorData,
)
from dynamics.quaternion_utils import quaternion_to_euler

logger = logging.getLogger(__name__)

# تحويل وحدات الضغط الديناميكي
# Dynamic pressure unit conversion factor: Pa → kgf/cm²
_PA_TO_KGFCM2 = 1 #1.0 / 98066.5

# تسارع الجاذبية
# Gravitational acceleration (m/s²)
_G = 9.81


def _limit(value: float, upper: float, lower: float) -> float:
    """تقييد القيمة بين حد أدنى وأقصى | Clamp value between lower and upper."""
    return max(lower, min(upper, value))


class M130AutopilotSystem:
    """
    نظام الطيار الآلي M130
    M130 Autopilot System

    يحوّل أوامر التوجيه (تسارع الميل والانعراج) إلى انحرافات الزعانف عبر:
    Converts guidance acceleration commands to fin deflections via:

    - تصفية IIR للسرعات الزاوية والتسارعات الجسمية
      IIR filtering of angular rates and body accelerations
    - تثبيت الارتفاع في المرحلة 1 (AltitudeAutopilot)
      Altitude hold in stage 1 (AltitudeAutopilot)
    - تتبع تسارع الميل في المرحلة 2+ (ApcAutopilot)
      Pitch acceleration tracking in stage 2+ (ApcAutopilot)
    - تحكم الانعراج المضمّن (inline yaw PID)
      Inline yaw PID control
    - تحكم الدوران (Roll PID)
      Roll PID control
    - تحويل إطار الجسم ← إطار التنفيذ
      Body → execute frame transformation

    تكوين (Configuration) via config['autopilot']['m130']:
      set_alt   : ارتفاع الهدف للمرحلة 1 (m) | Target altitude for stage 1
      t_stg1    : نهاية المرحلة 1 = وقت احتراق المحرك (s) | End of stage 1 = motor burn time
      t_ctrl    : تأخير بدء التحكم - زمن الإطلاق (s) | Control start delay - launcher time
    """

    def __init__(self, config: Dict[str, Any], **kwargs):
        """
        تهيئة نظام الطيار الآلي M130
        Initialize M130 autopilot system
        """
        self.config = config
        autopilot_config = config.get('autopilot', {})
        m130_config = autopilot_config.get('m130', {})

        # ---- معاملات من التكوين | Parameters from config ----
        # الارتفاع المستهدف في المرحلة 1 (m)
        self.set_alt: float = m130_config.get('set_alt', 100.0)
        # نهاية المرحلة 1 (ثانية) — من قيمة الكونفيج فقط، لا يعتمد على زمن الاطفاء
        # End of stage 1 (s) — from config value only, independent of motor burnout
        self.t_stg1: float = m130_config.get('t_stg1', 4.0)
        # تأخير بدء التحكم (ثانية)  - يتوافق مع زمن وجود الصاروخ على المنصة
        self.t_ctrl: float = m130_config.get('t_ctrl', 0.5)

        # ---- واجهة مطلوبة | Required interface attributes ----
        self.mode = AutopilotMode.FLIGHT_GENES

        # ---- متغيرات مخرجات للتسجيل | Output variables for logging ----
        self.last_pitch_accel_cmd: float = 0.0
        self.last_yaw_accel_cmd: float = 0.0
        self.last_accel_y_body_filtered: float = 0.0
        self.last_accel_z_body_filtered: float = 0.0

        # ---- حالات داخلية للمرشحات | Internal filter states ----
        # مرشح IIR للسرعات الزاوية (p, q_rate, r) بالراديان/ثانية
        # IIR filter for angular rates [p, q_rate, r] rad/s
        self._Wx_filter: float = 0.0   # معدل الدوران المرشح | Filtered roll rate
        self._Wy_filter: float = 0.0   # معدل الانحراف (yaw) المرشح | Filtered yaw rate
        self._Wz_filter: float = 0.0   # معدل الميل (pitch) المرشح | Filtered pitch rate

        # مرشح IIR للتسارعات الجسمية (مُعوَّضة بالدوران، مُقسَّمة على g)
        # IIR filter for roll-compensated body accelerations (÷ g)
        self._Ayb_filter: float = 0.0  # ay_b مُرشَّح | Filtered pitch-plane accel [g]
        self._Azb_filter: float = 0.0  # az_b مُرشَّح | Filtered yaw-plane accel [g]

        # ---- تكاملات قنوات التحكم | Control channel integrators ----
        # قناة الدوران | Roll channel
        self._out_integ_roll: float = 0.0
        # قناة الانعراج | Yaw channel
        self._out_integ_yaw: float = 0.0

        # ---- حالات متحكم الارتفاع | Altitude controller states ----
        self._alt_int_accum: float = 0.0   # متراكم التكامل | Integral accumulator
        self._alt_prev_herror: float = 0.0  # خطأ الارتفاع السابق | Previous altitude error
        self._alt_ed_filt: float = 0.0     # مرشح المشتق | Derivative filter

        # ---- حالات متحكم تسارع الميل | Pitch accel controller states ----
        self._apc_accum: float = 0.0  # متراكم التكامل | Integral accumulator

        # ---- تتبع انتقال المرحلة 1 → 2 | Stage 1 → 2 transition tracking ----
        self._stage1_completed: bool = False   # هل اكتملت المرحلة 1 | Stage 1 completed flag
        self._stage1_end_time: float = 0.0     # زمن نهاية المرحلة 1 | Stage 1 end time

        self.initialized: bool = False

        logger.info(
            f"M130AutopilotSystem initialized: set_alt={self.set_alt}m, "
            f"t_stg1={self.t_stg1}s (fallback), t_ctrl={self.t_ctrl}s"
        )

    # ==========================================================================
    # دوال واجهة الطيار الآلي | Autopilot interface methods
    # ==========================================================================

    def set_mode(self, mode: AutopilotMode) -> None:
        """تعيين وضع الطيار الآلي | Set autopilot mode."""
        self.mode = mode

    def reset(self) -> None:
        """إعادة تهيئة جميع الحالات | Reset all states."""
        self._Wx_filter = 0.0
        self._Wy_filter = 0.0
        self._Wz_filter = 0.0
        self._Ayb_filter = 0.0
        self._Azb_filter = 0.0
        self._out_integ_roll = 0.0
        self._out_integ_yaw = 0.0
        self._alt_int_accum = 0.0
        self._alt_prev_herror = 0.0
        self._alt_ed_filt = 0.0
        self._apc_accum = 0.0
        self._stage1_completed = False
        self._stage1_end_time = 0.0
        self.initialized = False
        logger.debug("M130AutopilotSystem reset")

    # ==========================================================================
    # تحديد المرحلة | Stage determination
    # ==========================================================================

    def _determine_stage(self, t: float, yaw_los_deg: float = 0.0) -> int:
        """
        تحديد مرحلة الطيران الحالية (3 مراحل)
        Determine current flight stage (3 stages):
          1: |yaw_los| <= 11° AND t < t_stg1  (احتراق المحرك | Motor burn)
          2: t < 100                           (الطيران الحر | Free flight)
          3: t >= 100                          (نهاية الطيران | Late flight)

        الانتقال من المرحلة 1 إلى 2 يحدث عندما تتجاوز زاوية خط البصر 11 درجة
        Transition from stage 1 to 2 occurs when |yaw LOS angle| > 11°
        يُستخدم t_stg1 كحد أقصى احتياطي
        t_stg1 is used as a fallback maximum time
        """
        if not self._stage1_completed:
            if abs(yaw_los_deg) > 11.0:
                self._stage1_completed = True
                self._stage1_end_time = t
                logger.info(f"Stage 1 -> 2 at t={t:.2f}s (|yaw_los|={abs(yaw_los_deg):.1f}deg > 11deg)")
            elif t >= self.t_stg1:
                self._stage1_completed = True
                self._stage1_end_time = t
                logger.info(f"Stage 1 -> 2 at t={t:.2f}s (t >= t_stg1={self.t_stg1}s fallback)")
            else:
                return 1

        if t < 100.0:
            return 2
        else:
            return 3

    # ==========================================================================
    # جدولة المكاسب | Gain scheduling  (PilotGain)
    # ==========================================================================

    def _pilot_gain(self, stage: int, q_kgf: float) -> Dict[str, float]:
        """
        حساب مكاسب الطيار الآلي حسب المرحلة والضغط الديناميكي
        Calculate autopilot gains based on stage and dynamic pressure.

        Args:
            stage:   مرحلة الطيران (1, 2, 3) | Flight stage (1, 2, 3)
            q_kgf:   الضغط الديناميكي [kgf/cm²] | Dynamic pressure [kgf/cm²]

        Returns:
            قاموس المكاسب | Gains dictionary
        """
        q = q_kgf# max(0.001, q_kgf)

        # ---- حدود المتحكمات | Controller limits ----
        out_integ_limit = 0.115 / q
        apc_limit = 2.5 * q
        ayc_limit = 2.5 * q
        Acc_sat = q / 2.0

        # ---- معامل K_acc الداخلي للمرحلة 2 | Internal K_acc for stage 2 ----
        K_acc_2 = [0.0115, -0.0656, 0.0349]

        # ---- مكاسب محددة للمرحلة | Stage-specific gains (for K_p/i_integ_acc) ----
        if stage == 1:
            if q < 1.7:
                K_p_integ_acc = 0.0
                # 0.7 * 1.5 * 0.5 * (-0.082)
                K_i_integ_acc = 0.7 * 1.5 * 0.5 * (-0.082)   # ≈ -0.04305
            else:
                K_p_integ_acc = 0.0
                # (1.0 * 0.5 * -0.082) / (2 * q)
                K_i_integ_acc = (0.5 * (-0.082)) / (2.0 * q)  # = -0.0205/q
        elif stage == 2:
            if q < 2.0:
                K_p_integ_acc = 1.0 * K_acc_2[0]              # = 0.0115
                K_i_integ_acc = 1.31 * K_acc_2[1]             # ≈ -0.085936
            else:
                K_p_integ_acc = -0.035 * K_acc_2[0]           # ≈ -0.0004025
                K_i_integ_acc = 1.721 * K_acc_2[1]            # ≈ -0.112898
        else:  # stage 3
            K_p_integ_acc = 1.0 * K_acc_2[0]
            K_i_integ_acc = 1.31 * K_acc_2[1]

        # ---- تطبيق التجاوزات النهائية | Apply final overrides (always) ----
        K_w_pitch = 0.0086 * q
        K_w_yaw = 0.1 * 0.0582 * 1.0    # = 0.00582 (ثابت | constant)
        K_w_roll = 1.0 * 0.0001
        K_p_integ_roll = 0.5 * 0.01     # = 0.005
        K_i_integ_roll = 0.0

        return {
            'K_w_pitch': K_w_pitch,
            'K_w_yaw': K_w_yaw,
            'K_w_roll': K_w_roll,
            'K_p_integ_acc': K_p_integ_acc,
            'K_i_integ_acc': K_i_integ_acc,
            'K_p_integ_roll': K_p_integ_roll,
            'K_i_integ_roll': K_i_integ_roll,
            'out_integ_limit': out_integ_limit,
            'apc_limit': apc_limit,
            'ayc_limit': ayc_limit,
            'Acc_sat': Acc_sat,
        }

    # ==========================================================================
    # حاسبة مُعامِل السرعة | Speed scaler (calc_speed_scaler_)
    # ==========================================================================

    def _calc_speed_scaler(self, vm: float) -> float:
        """
        حساب معامل تكييف السرعة لأسطح التحكم
        Calculate speed scaling factor for control surfaces.

        Args:
            vm: سرعة الهواء (m/s) | Airspeed (m/s)

        Returns:
            معامل السرعة (بلا أبعاد) | Speed scaler (dimensionless)
        """
        scaling_speed = 170.0     # سرعة مرجعية (m/s) | Reference speed
        airspeed_max = 278.0
        airspeed_min = max(30.0, 5.0)  # MIN_AIRSPEED_MIN = 5

        scale_min = min(0.5, scaling_speed / (2.0 * airspeed_max))   # ≈ 0.3057
        scale_max = max(2.0, scaling_speed / (0.7 * airspeed_min))   # ≈ 8.095

        if vm > 0.0001:
            speed_scaler = scaling_speed / vm
        else:
            speed_scaler = scale_max

        return _limit(speed_scaler, scale_max, scale_min)

    # ==========================================================================
    # متحكم الارتفاع | Altitude autopilot (AltitudeAutopilot)
    # ==========================================================================

    def _altitude_autopilot(
        self,
        sp: float,
        h_fb: float,
        q_kgf: float,
        ss: float,
        dt: float,
        t: float,
    ) -> float:
        """
        متحكم تثبيت الارتفاع (المرحلة 1)
        Altitude hold controller (stage 1).

        Args:
            sp:    نقطة الضبط (= 1.0) | Set point (= 1.0)
            h_fb:  التغذية الراجعة (= altitude / set_alt) | Feedback (= altitude / set_alt)
            q_kgf: الضغط الديناميكي [kgf/cm²] | Dynamic pressure [kgf/cm²]
            ss:    معامل السرعة | Speed scaler
            dt:    خطوة الزمن (s) | Time step (s)
            t:     الزمن الحالي (s) | Current time (s)

        Returns:
            أمر انحراف الميل (rad) | Pitch deflection command (rad)
        """
        # K_h = [K_p_error, K_p_rate, K_i_coeff, K_d_gain]
        K_h = [-0.0049, q_kgf * 0.15075, 0.5 * ss * 4.0 * 0.017, 0.25]

        h_error = (sp - h_fb)*1.8  # = 1 - altitude/set_alt

        # تكامل الخطأ | Integral accumulation (gain embedded inside)
        self._alt_int_accum += K_h[2] * h_error * dt
        self._alt_int_accum = _limit(self._alt_int_accum, 1.5, -1.5)

        # المصطلحات | Terms
        ep = K_h[1] * h_error *1
        ei = 3* K_h[2] * self._alt_int_accum
        ed = (K_h[3] * 1.7 * (h_error - self._alt_prev_herror) / dt) #if dt > 1e-9 else 0.0

        # مرشح المشتق IIR | Derivative IIR filter: ed_filt = (ed + q*ed_filt)/(q+1)
        q = q_kgf  #max(0.001, q_kgf)
        self._alt_ed_filt = (ed + q * self._alt_ed_filt) / (q + 1.0)

        # حدود | Limits
        lim_p_d = 0.3
        self._alt_ed_filt = _limit(self._alt_ed_filt, lim_p_d, -lim_p_d)
        ep = _limit(ep, 0.5 * lim_p_d, -0.5 * lim_p_d)

        u_ae = ep + ei + self._alt_ed_filt

        self._alt_prev_herror = h_error

        alti_lim = math.radians(10.0)
        u_cmd = _limit(u_ae, alti_lim, -alti_lim) *-1

        if t < 0.05:
            u_cmd = 0.0
        # else: 
            # u_cmd = 0.0
        return u_cmd

    # ==========================================================================
    # متحكم تسارع الميل | Pitch acceleration autopilot (ApcAutopilot)
    # ==========================================================================

    def _apc_autopilot(
        self,
        fz_fb: float,
        q_fb: float,
        ac_pitch: float,
        q_kgf: float,
        dt: float,
    ) -> float:
        """
        متحكم تتبع تسارع الميل (المرحلة 2+)
        Pitch acceleration tracking controller (stage 2+).

        Args:
            fz_fb:   التسارع الجسمي المُرشَّح Ayb_filter [g] | Filtered body accel [g]
            q_fb:    حد التخميد بالسرعة الزاوية = K_w_pitch * Wz_filter | Rate damping term
            ac_pitch: أمر تسارع الميل من التوجيه | Pitch accel command from guidance
            q_kgf:   الضغط الديناميكي [kgf/cm²] | Dynamic pressure [kgf/cm²]
            dt:      خطوة الزمن (s) | Time step (s)

        Returns:
            أمر انحراف الميل (rad) | Pitch deflection command (rad)
        """
        # K_acc[0]=معامل تناسبي, K_acc[1]=معامل تكاملي, K_acc[2]=تخميد سرعة زاوية
        # K_acc[0]=proportional, K_acc[1]=integral, K_acc[2]=rate damping
        K_acc = [
            1.0 * 1.0 * 0.0010981081,       # 0.0010981081
            -1.0 * 1.5 * 0.09587,            # -0.143805
            1.0 * 0.4581,                    # 0.4581
        ]

        apc_error = fz_fb - ac_pitch

        q =q_kgf # max(0.001, q_kgf)
        self._apc_accum += apc_error * dt *1.1
        self._apc_accum = _limit(self._apc_accum, 5.5 * q, -5.5 * q)

        u_ae = (
            K_acc[0] * apc_error
            + K_acc[1] * self._apc_accum
            + K_acc[2] * q_fb
        )

        u_cmd = _limit(u_ae, math.radians(13.0), -math.radians(13.0))
        return u_cmd

    # ==========================================================================
    # الدالة الرئيسية | Main compute (AutoPilot)
    # ==========================================================================

    def compute(self, autopilot_input: AutopilotInput) -> AutopilotOutput:
        """
        حساب أوامر انحراف الزعانف
        Compute fin deflection commands.

        Args:
            autopilot_input: مدخلات الطيار الآلي | Autopilot inputs

        Returns:
            AutopilotOutput with delta_roll, delta_pitch, delta_yaw (rad)
        """
        # ---- استخراج المدخلات | Extract inputs ----
        t = autopilot_input.time
        dt = max(autopilot_input.dt, 1e-9)
        q_dyn_pa = autopilot_input.q_dyn          # Pa
        angular_velocity = autopilot_input.angular_velocity   # [p, q_rate, r] rad/s
        accelerometer = autopilot_input.accelerometer         # [ax, ay, az] m/s²
        quaternion = autopilot_input.quaternion               # [w, x, y, z]
        vm = autopilot_input.airspeed                         # m/s
        altitude = autopilot_input.altitude                   # m (positive up)
        ac_pitch = autopilot_input.pitch_accel_cmd            # أمر تسارع الميل من التوجيه
        ayc = autopilot_input.yaw_accel_cmd                   # أمر تسارع الانعراج من التوجيه

        # زاوية خط البصر في مستوى الانعراج | Yaw LOS angle
        yaw_los_deg = autopilot_input.yaw_los_deg

        # تحويل الضغط الديناميكي إلى kgf/cm²
        q_kgf = q_dyn_pa  /10000 #max(0.001, q_dyn_pa * _PA_TO_KGFCM2)

        # تحديد المرحلة | Determine stage
        stage = self._determine_stage(t, yaw_los_deg)

        # ---- حارس الإطلاق | Launch guard ----
        # صفّر كل شيء وأعد إذا كنا لا نزال في مرحلة الإطلاق
        # Zero everything and return if still on launcher
        if t <= self.t_ctrl:
            self._Wx_filter = 0.0
            self._Wy_filter = 0.0
            self._Wz_filter = 0.0
            self._Ayb_filter = 0.0
            self._Azb_filter = 0.0
            self._out_integ_roll = 0.0
            self._out_integ_yaw = 0.0
            return AutopilotOutput(delta_roll=0.0, delta_pitch=0.0, delta_yaw=0.0)

        # ---- زاوية الدوران من الرباعي | Roll angle from quaternion ----
        roll, pitch, yaw = quaternion_to_euler(quaternion)
        phi = roll   # زاوية الدوران (rad) | Roll angle (rad)

        # ---- جدولة المكاسب | Gain scheduling ----
        gains = self._pilot_gain(stage, q_kgf)
        K_w_pitch = gains['K_w_pitch']
        K_w_yaw = gains['K_w_yaw']
        K_w_roll = gains['K_w_roll']
        K_p_integ_acc = gains['K_p_integ_acc']
        K_i_integ_acc = gains['K_i_integ_acc']
        K_p_integ_roll = gains['K_p_integ_roll']
        K_i_integ_roll = gains['K_i_integ_roll']
        out_integ_limit = gains['out_integ_limit']
        Acc_sat = gains['Acc_sat']

        # ---- تشبع الدوران | Roll saturation ----
        q = q_kgf # max(0.001, q_kgf)
        if q < 15.0:
            Roll_sat = 3.0 * (0.1396 + 0.03499 * q)
        else:
            Roll_sat = 3.0 * 0.6632
        Roll_sat *= 7.0   # عملياً، تشبع كبير جداً | Effectively very large (no limiting)

        # ---- معامل السرعة | Speed scaler ----
        ss = self._calc_speed_scaler(vm)

        # ---- استخراج السرعات الزاوية | Extract angular rates ----
        # Wb[0]=p (دوران | roll), Wb[1]=r_rate (انعراج | yaw), Wb[2]=q_rate (ميل | pitch)
        # ملاحظة: تطابق المتغيرات مع الكود C++ الأصلي:
        # Note: Variable mapping to match original C++ code:
        #   Wb(0,0) → angular_velocity[0] = p  (roll rate)
        #   Wb(1,0) → angular_velocity[1] = q_rate (pitch rate in body)
        #   Wb(2,0) → angular_velocity[2] = r  (yaw rate in body)
        # في الكود C++: Wx=roll, Wy=yaw(1), Wz=pitch(2)
        # In C++ code: Wx=roll, Wy=body-y rate, Wz=body-z rate
        Wx = angular_velocity[0]   # معدل الدوران | Roll rate
        Wy = -angular_velocity[2]   # معدل الميل في الجسم | Body pitch/yaw rate
        Wz = angular_velocity[1]   # معدل الانعراج في الجسم | Body yaw/pitch rate

        # ---- مرشح IIR للسرعات الزاوية | IIR filter for angular rates ----
        # (new + 3*old) / 4
        self._Wx_filter = (Wx + 3.0 * self._Wx_filter) / 4.0
        self._Wy_filter = (Wy + 3.0 * self._Wy_filter) / 4.0
        self._Wz_filter = (Wz + 3.0 * self._Wz_filter) / 4.0

        # ---- مرشح التسارع الجسمي | Body acceleration filter ----
        # ab[1] = body y accel (m/s²), ab[2] = body z accel (m/s²)
        Ay_body = accelerometer[2]   # التسارع الجسمي y | Body y acceleration (m/s²)
        Az_body = accelerometer[1]   # التسارع الجسمي z | Body z acceleration (m/s²)

        # تحويل بإطار الدوران وتطبيع بـ g
        # Roll-compensated transformation and normalization by g
        cos_phi = math.cos(phi)
        sin_phi = math.sin(phi)

        az_b = sin_phi * Ay_body / 9.8 + cos_phi * Az_body / 9.8
        self._Azb_filter = (az_b + 3.0 * self._Azb_filter) / 4.0

        ay_b = cos_phi * Ay_body / 9.8 - sin_phi * Az_body / 9.8
        self._Ayb_filter = (ay_b + 3.0 * self._Ayb_filter) / 4.0

        # ---- السرعات الزاوية المُعوَّضة بالدوران | Roll-compensated angular rates ----
        wzr = cos_phi * self._Wz_filter + sin_phi * self._Wy_filter   # معدل ميل إطار التنفيذ | Execute-frame pitch rate
        wyr = -sin_phi * self._Wz_filter + cos_phi * self._Wy_filter  # معدل انعراج إطار التنفيذ | Execute-frame yaw rate

        wy_compensate = K_w_yaw * wyr
        wz_compensate = K_w_pitch * wzr    # noqa: F841 (calculated but not used directly)
        wx_compensate = K_w_roll * self._Wx_filter

        # ============================================================
        # قناة الدوران | Roll channel
        # ============================================================
        phic = 0.0   # أمر الدوران = 0 | Roll command = 0
        du_roll = phic - phi
        du_roll = _limit(du_roll, Roll_sat, -Roll_sat)

        self._out_integ_roll += K_i_integ_roll * du_roll * dt
        self._out_integ_roll = _limit(self._out_integ_roll, 0.2, -0.2)

        du_compensate1 = self._out_integ_roll + K_p_integ_roll * du_roll
        delac = 3.0 * (du_compensate1 - wx_compensate)

        # ============================================================
        # قناة الانعراج | Yaw channel
        # ============================================================
        du_yaw = _limit(ayc, 2.0 * Acc_sat, -2.0 * Acc_sat)

        self._out_integ_yaw += K_i_integ_acc * du_yaw * dt
        self._out_integ_yaw = _limit(self._out_integ_yaw, out_integ_limit, -out_integ_limit)

        du_compensate1 = self._out_integ_yaw + K_p_integ_acc * du_yaw
        delrc0 = du_compensate1 - wy_compensate

        # ============================================================
        # قناة الميل | Pitch channel
        # ============================================================
        delec0 = 0.0
        if stage == 1:
            # تثبيت الارتفاع | Altitude hold
            delec0 = self._altitude_autopilot(
                sp=1.0,
                h_fb=altitude / self.set_alt,
                q_kgf=q_kgf,
                ss=ss,
                dt=dt,
                t=t,
            )
        elif stage >= 2:
            # تتبع تسارع الميل | Pitch acceleration tracking
            delec0 = self._apc_autopilot(
                fz_fb=self._Ayb_filter,
                q_fb=0.0086 * q_kgf * self._Wz_filter,
                ac_pitch=ac_pitch,
                q_kgf=q_kgf,
                dt=dt,
            )

        # ---- عكس الإشارات | Sign flip ----
        delec0 *= -1.0
        delrc0 *= -1.0

        # ============================================================
        # تحقق الأمان | Safety check
        # إذا كان معدل الدوران كبيراً وأمر الدوران كبيراً → صفّر الميل والانعراج
        # If high roll rate with large roll command → zero pitch and yaw
        # ============================================================
        roll_rate_limit = math.radians(90.0)
        roll_cmd_limit = math.radians(3.0)

        if abs(self._Wx_filter) > roll_rate_limit and abs(delac) >= roll_cmd_limit:
            delec = 0.0
            delrc = 0.0
            self._out_integ_yaw = 0.0
        elif stage < 4:
            # تحويل إطار الجسم ← إطار التنفيذ | Body → execute frame transform
            delec = delec0 * cos_phi - delrc0 * sin_phi
            delrc = delrc0 * cos_phi + delec0 * sin_phi
        else:
            # المرحلة 4 (إن وُجدت): بدون تحويل | Stage 4 (if exists): no transform
            delec = delec0
            delrc = delrc0

        # ---- تحديث متغيرات التسجيل | Update logging variables ----
        self.last_pitch_accel_cmd = ac_pitch
        self.last_yaw_accel_cmd = ayc
        self.last_accel_y_body_filtered = self._Ayb_filter
        self.last_accel_z_body_filtered = self._Azb_filter

        # ---- إنشاء المخرجات | Build output ----
        # delta_roll  = delac (قناة الدوران | roll channel aileron)
        # delta_pitch = delec (قناة الميل بعد تحويل الإطار | pitch after frame transform)
        # delta_yaw   = delrc (قناة الانعراج بعد تحويل الإطار | yaw after frame transform)
        return AutopilotOutput(
            delta_roll=-delac,
            delta_pitch=-delrc,
            delta_yaw= -delec,
        )
