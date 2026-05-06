#!/usr/bin/env python3
"""
آلة حالة مراحل الطيران (17 مرحلة) متوافقة مع PX4 rocket_sys.
"""

import logging
import numpy as np
from enum import IntEnum
from typing import Optional, TYPE_CHECKING
import warnings

from .enums import SeparationTrigger

if TYPE_CHECKING:
    from .frame_manager import FrameConfiguration

logger = logging.getLogger(__name__)


class FlightPhase(IntEnum):
    """17 مرحلة طيران (0-14 طبيعية، 254-255 طوارئ) متوافقة مع PX4."""

    # === Natural Flight Phases (0-14) ===
    SAFE_ON_PAD = 0          # آمن على القاذف - الصاروخ على الأرض، أنظمة معطلة
    PREFLIGHT = 1            # فحوصات ما قبل الإقلاع - فحص الأنظمة
    ARMED = 2                # جاهز للإطلاق - أنظمة نشطة، تسليح
    LAUNCH = 3               # كشف الإطلاق - الإقلاع الفعلي
    POWERED_ASCENT = 4       # صعود تحت الدفع - المحرك يعمل
    BURNOUT = 5              # انقطاع الدفع - انتهاء الوقود
    COAST_ASCENT = 6         # صعود بالقصور - طيران باليستي
    NEAR_SPACE_ASCENT = 7    # الغلاف الجوي العلوي (40-100 km)
    EXOATM_COAST = 8         # خارج الغلاف الجوي (>100 km)
    APOGEE = 9               # كشف القمة - أعلى نقطة في المسار
    MIDCOURSE = 10           # المسار الأوسط - التوجيه المتوسط
    TERMINAL = 11            # التوجيه النهائي - المرحلة الأخيرة
    IMPACT_PREP = 12         # تحضير الإصابة - تحضير للاصطدام
    IMPACT = 13              # الإصابة - الاصطدام بالهدف
    MISSION_COMPLETE = 14    # اكتمال المهمة - انتهاء ناجح

    # === Emergency Phases (254-255) ===
    ABORT = 254              # إنهاء طارئ - إيقاف فوري للمهمة
    SAFE_MODE = 255          # وضع آمن - استقرار آمن للصاروخ


class FlightPhaseManager:
    """
    Manages flight phase transitions and state machine logic.

    Supports multi-stage rockets with separation events.
    """

    def __init__(self, config, frame_configuration: Optional['FrameConfiguration'] = None):
        """
        Initialize flight phase manager.

        Args:
            config: Flight phase configuration dictionary.
            frame_configuration: Optional shared FrameConfiguration.
                                If provided, frame parameters are read from it
                                instead of config to keep subsystems synchronized.
        """

        # Start from SAFE_ON_PAD (not PREFLIGHT) to match PX4
        self.phase = FlightPhase.SAFE_ON_PAD
        self.phase_start_time = 0.0

        if frame_configuration is None:
            warnings.warn(
                "FlightPhaseManager created without shared FrameConfiguration. "
                "Frame parameters will be read from config dict, which may cause "
                "configuration divergence with other subsystems. "
                "For production use, pass a shared FrameConfiguration instance. "
                "(تحذير: تم إنشاء FlightPhaseManager بدون FrameConfiguration مشترك. "
                "سيتم قراءة معاملات الإطار من قاموس التكوين، مما قد يسبب تباعد التكوين "
                "مع الأنظمة الفرعية الأخرى. للاستخدام الإنتاجي، مرر مثيل FrameConfiguration مشترك.)",
                UserWarning,
                stacklevel=2
            )

        # Long-range mode flag for ECEF frame validation
        if frame_configuration is not None:
            self.long_range_mode = frame_configuration.long_range_mode
        else:
            self.long_range_mode = config.get('long_range_mode', False)

        # Configuration for SAFE_ON_PAD and ARMED phases
        self.preflight_checks_complete = False
        self.armed = False

        self.burnout_time = config.get('burnout_time_s', 5.0)
        self.burnout_accel_threshold = config.get('burnout_accel_g', 1.0) * 9.80665
        self.burnout_hold_time = config.get('burnout_hold_s', 0.2)
        self.burnout_detection_start: Optional[float] = None

        self.burnout_detection_ratio = config.get('burnout_detection_ratio', 0.8)

        self.apogee_vz_threshold = config.get('apogee_vz_threshold', 5.0)
        self.apogee_hold_time = config.get('apogee_hold_s', 0.5)
        self.apogee_detection_start: Optional[float] = None

        self.apogee_zero_threshold = config.get('apogee_zero_threshold', 0.5)

        # Track previous vertical velocity for apogee detection (sign reversal)
        self.previous_vz = None
        self.was_ascending = False

        # Configurable altitude/density thresholds
        self.near_space_altitude = config.get('near_space_altitude_m', 40000.0)
        self.exoatm_altitude = config.get('exoatm_altitude_m', 100000.0)
        self.near_space_density = config.get('near_space_density_kg_m3', 0.01)
        self.exoatm_density = config.get('exoatm_density_kg_m3', 1e-5)

        # Configurable terminal phase thresholds
        self.terminal_altitude = config.get('terminal_altitude_m', 5000.0)
        self.impact_prep_altitude = config.get('impact_prep_altitude_m', 100.0)
        self.impact_altitude = config.get('impact_altitude_m', 10.0)

        self.burnout_thrust_threshold = config.get('burnout_thrust_threshold_n', 100.0)

        # Configurable phase hold times
        self.launch_hold_time = config.get('launch_hold_s', 0.5)
        self.burnout_phase_hold_time = config.get('burnout_phase_hold_s', 0.5)
        self.apogee_phase_hold_time = config.get('apogee_phase_hold_s', 1.0)
        self.impact_hold_time = config.get('impact_hold_s', 0.5)

        self.burnout_detected = False

        # Multi-stage support
        self.current_stage = 1
        self.num_stages = config.get('num_stages', 1)
        self.separation_in_progress = False
        self.separation_complete = False
        self.separation_times = []
        self.stage_burnout_times = config.get('stage_burnout_times', [self.burnout_time])

        # Stage start time for relative time calculations in separation triggers
        self.stage_start_time = 0.0

        # Separation configuration
        self.separation_delay = config.get('separation_delay_s', 0.5)
        self.separation_armed = False

        # Ignition delay for next stage after separation
        ignition_delay_config = config.get('ignition_delay_s', 0.0)
        if isinstance(ignition_delay_config, list):
            self.stage_ignition_delays = ignition_delay_config
        else:
            self.stage_ignition_delays = [0.0] + [ignition_delay_config] * (self.num_stages - 1)

        # When True, the simulation orchestrator handles separation timing and execution
        self.external_separation_handling = config.get('external_separation_handling', False)

        # Separation trigger configuration
        self.separation_trigger = SeparationTrigger.from_config(
            config.get('separation_trigger', SeparationTrigger.BURNOUT)
        )
        self.separation_altitude = config.get('separation_altitude_m', 50000.0)
        self.separation_velocity = config.get('separation_velocity_ms', 1000.0)
        self.separation_time = config.get('separation_time_s', 10.0)

        # Configurable phases where separation is allowed
        default_separation_phases = [
            FlightPhase.BURNOUT,
            FlightPhase.COAST_ASCENT,
            FlightPhase.NEAR_SPACE_ASCENT,
            FlightPhase.EXOATM_COAST,
        ]
        separation_phases_config = config.get('separation_allowed_phases', None)
        if separation_phases_config is not None:
            self.separation_allowed_phases = self._parse_phase_list(separation_phases_config)
        else:
            self.separation_allowed_phases = set(default_separation_phases)

        # Auto-arm configuration for simulation
        auto_arm = config.get('auto_arm', False)
        if auto_arm:
            self.preflight_checks_complete = True
            self.armed = True
            self.phase = FlightPhase.ARMED
            logger.info("Auto-arm enabled: System armed and ready for launch (التسليح التلقائي مفعل)")

    def update(self, t, state, thrust, accel_body, density, altitude, vertical_velocity=None,
               critical_violation=None):
        """
        Update the flight phase.

        Args:
            t: Time (s).
            state: State vector.
            thrust: Thrust (N).
            accel_body: Body-frame acceleration (m/s^2).
            density: Air density (kg/m^3).
            altitude: Altitude (m).
            vertical_velocity: NED vertical velocity (m/s), positive down.
                              If None, derive it from the state in NED or ECEF mode.
            critical_violation: If True, switch to ABORT.

        Returns:
            Current flight phase.
        """

        if vertical_velocity is not None:
            vz = vertical_velocity
        else:
            velocity = state[3:6]
            if self.long_range_mode:
                pos_ecef = state[0:3]
                radius = np.linalg.norm(pos_ecef)
                if radius > 0.0:
                    radial_hat = pos_ecef / radius
                    vz = -float(np.dot(velocity, radial_hat))
                else:
                    warnings.warn(
                        "long_range_mode is enabled but position norm is zero; "
                        "falling back to state[5] for vertical velocity. "
                        "وضع المدى الطويل مفعل لكن متجه الموضع صفري؛ سيتم استخدام state[5].",
                        UserWarning
                    )
                    vz = velocity[2]
            else:
                vz = velocity[2]  # NED frame: positive is down

        previous_phase = self.phase

        # === ABORT Phase Logic ===
        if critical_violation and self.phase not in [FlightPhase.SAFE_ON_PAD, FlightPhase.MISSION_COMPLETE,
                                                      FlightPhase.ABORT, FlightPhase.SAFE_MODE]:
            self.phase = FlightPhase.ABORT
            self.phase_start_time = t
            logger.warning(f"Phase transition: {previous_phase.name} -> ABORT at t={t:.3f}s (critical violation)")
            return self.phase

        # === Multi-stage Separation Logic ===
        if self.num_stages > 1 and self.current_stage < self.num_stages:
            if not self.external_separation_handling:
                # Internal separation handling (legacy mode)
                if self.phase in self.separation_allowed_phases:
                    if not self.separation_in_progress and self.separation_armed:
                        velocity_mag = (state[3]**2 + state[4]**2 + state[5]**2)**0.5
                        if self._check_separation_condition(t, thrust, altitude, velocity_mag):
                            self.execute_separation(t)

                # Check if separation delay has passed and complete separation
                if self.separation_in_progress and not self.separation_complete:
                    if t - self.separation_times[-1] >= self.separation_delay:
                        self.complete_separation(t)
                        self.phase = FlightPhase.POWERED_ASCENT
                        self.phase_start_time = t
                        self.stage_start_time = t
                        self.burnout_detected = False
                        self.burnout_detection_start = None

        # === SAFE_ON_PAD Phase Logic ===
        if self.phase == FlightPhase.SAFE_ON_PAD:
            if self.preflight_checks_complete:
                self.phase = FlightPhase.PREFLIGHT
                self.phase_start_time = t

        # === PREFLIGHT Phase Logic ===
        elif self.phase == FlightPhase.PREFLIGHT:
            if self.armed:
                self.phase = FlightPhase.ARMED
                self.phase_start_time = t

        # === ARMED Phase Logic ===
        elif self.phase == FlightPhase.ARMED:
            if accel_body[0] > 2.0 * 9.80665:
                self.phase = FlightPhase.LAUNCH
                self.phase_start_time = t

        elif self.phase == FlightPhase.LAUNCH:
            if t - self.phase_start_time > self.launch_hold_time:
                self.phase = FlightPhase.POWERED_ASCENT
                self.phase_start_time = t
                self.stage_start_time = t

        elif self.phase == FlightPhase.POWERED_ASCENT:
            time_in_powered_ascent = t - self.phase_start_time
            if time_in_powered_ascent > self.burnout_time * self.burnout_detection_ratio:
                if accel_body[0] < self.burnout_accel_threshold and thrust < self.burnout_thrust_threshold:
                    if self.burnout_detection_start is None:
                        self.burnout_detection_start = t
                        logger.debug(f"Burnout detection started at t={t:.3f}s")
                    elif t - self.burnout_detection_start > self.burnout_hold_time:
                        self.phase = FlightPhase.BURNOUT
                        self.phase_start_time = t
                        self.burnout_detected = True
                        logger.info(f"Phase transition: POWERED_ASCENT -> BURNOUT at t={t:.3f}s")
                else:
                    self.burnout_detection_start = None

        elif self.phase == FlightPhase.BURNOUT:
            if t - self.phase_start_time > self.burnout_phase_hold_time:
                if altitude > self.near_space_altitude or density < self.near_space_density:
                    self.phase = FlightPhase.NEAR_SPACE_ASCENT
                else:
                    self.phase = FlightPhase.COAST_ASCENT
                self.phase_start_time = t

        elif self.phase == FlightPhase.COAST_ASCENT:
            if altitude > self.near_space_altitude or density < self.near_space_density:
                self.phase = FlightPhase.NEAR_SPACE_ASCENT
                self.phase_start_time = t
            elif self._check_apogee_condition(vz, t):
                self.phase = FlightPhase.APOGEE
                self.phase_start_time = t
                self._reset_apogee_detection()

        elif self.phase == FlightPhase.NEAR_SPACE_ASCENT:
            if altitude > self.exoatm_altitude or density < self.exoatm_density:
                self.phase = FlightPhase.EXOATM_COAST
                self.phase_start_time = t
            elif self._check_apogee_condition(vz, t):
                self.phase = FlightPhase.APOGEE
                self.phase_start_time = t
                self._reset_apogee_detection()

        elif self.phase == FlightPhase.EXOATM_COAST:
            if self._check_apogee_condition(vz, t):
                self.phase = FlightPhase.APOGEE
                self.phase_start_time = t
                self._reset_apogee_detection()

        elif self.phase == FlightPhase.APOGEE:
            if t - self.phase_start_time > self.apogee_phase_hold_time:
                self.phase = FlightPhase.MIDCOURSE
                self.phase_start_time = t

        elif self.phase == FlightPhase.MIDCOURSE:
            if altitude < self.terminal_altitude and vz > 0:
                self.phase = FlightPhase.TERMINAL
                self.phase_start_time = t

        elif self.phase == FlightPhase.TERMINAL:
            if altitude < self.impact_prep_altitude:
                self.phase = FlightPhase.IMPACT_PREP
                self.phase_start_time = t

        elif self.phase == FlightPhase.IMPACT_PREP:
            if altitude < self.impact_altitude:
                self.phase = FlightPhase.IMPACT
                self.phase_start_time = t

        elif self.phase == FlightPhase.IMPACT:
            if t - self.phase_start_time > self.impact_hold_time:
                self.phase = FlightPhase.MISSION_COMPLETE
                self.phase_start_time = t

        elif self.phase == FlightPhase.MISSION_COMPLETE:
            pass

        elif self.phase == FlightPhase.ABORT:
            pass

        elif self.phase == FlightPhase.SAFE_MODE:
            pass

        # Log phase transitions (except for already logged ones)
        if self.phase != previous_phase and self.phase != FlightPhase.BURNOUT:
            logger.info(f"Phase transition: {previous_phase.name} -> {self.phase.name} at t={t:.3f}s")

        return self.phase

    def _check_apogee_condition(self, vz, t):
        """
        Check whether apogee has been reached.

        Uses sign reversal in NED vertical velocity, with hold time for noise filtering.

        Args:
            vz: NED vertical velocity (m/s), negative up and positive down.
            t: Current time (s).

        Returns:
            bool: True if the apogee condition is met.
        """
        if self.previous_vz is not None:
            was_ascending = self.previous_vz < -self.apogee_vz_threshold
            is_near_zero_or_descending = vz > -self.apogee_zero_threshold

            if was_ascending:
                self.was_ascending = True

            if self.was_ascending and is_near_zero_or_descending:
                if self.apogee_detection_start is None:
                    self.apogee_detection_start = t
                    logger.debug(f"Apogee detection started at t={t:.3f}s, vz={vz:.2f} m/s")
                elif t - self.apogee_detection_start > self.apogee_hold_time:
                    logger.info(f"Apogee detected at t={t:.3f}s")
                    return True
            else:
                self.apogee_detection_start = None

        self.previous_vz = vz
        return False

    def _reset_apogee_detection(self):
        """Reset apogee detection state."""
        self.was_ascending = False
        self.previous_vz = None
        self.apogee_detection_start = None

    def _parse_phase_list(self, phase_list):
        """Parse phase values into a set of FlightPhase enums."""
        result = set()
        for phase_value in phase_list:
            if isinstance(phase_value, FlightPhase):
                result.add(phase_value)
            elif isinstance(phase_value, int):
                try:
                    result.add(FlightPhase(phase_value))
                except ValueError:
                    logger.warning(f"Invalid phase integer: {phase_value}")
            elif isinstance(phase_value, str):
                phase_name = phase_value.upper().strip()
                try:
                    result.add(FlightPhase[phase_name])
                except KeyError:
                    logger.warning(f"Invalid phase name: {phase_value}")
            else:
                logger.warning(f"Invalid phase type: {type(phase_value)}")
        return result

    def _check_separation_condition(self, t, thrust, altitude=0.0, velocity=0.0):
        """
        Check whether separation should occur (internal legacy mode).

        Args:
            t: Current time (s).
            thrust: Current thrust (N).
            altitude: Current altitude (m).
            velocity: Current velocity magnitude (m/s).

        Returns:
            bool: True if separation should occur.
        """
        if not self.separation_armed:
            return False

        if self.current_stage >= self.num_stages:
            return False

        if self.separation_in_progress:
            return False

        stage_time = t - self.stage_start_time

        if self.separation_trigger == SeparationTrigger.BURNOUT:
            current_burnout_time = self.stage_burnout_times[self.current_stage - 1] \
                if self.current_stage <= len(self.stage_burnout_times) else self.burnout_time
            if stage_time >= current_burnout_time and thrust < self.burnout_thrust_threshold:
                return True

        elif self.separation_trigger == SeparationTrigger.TIME:
            if stage_time >= self.separation_time:
                return True

        elif self.separation_trigger == SeparationTrigger.ALTITUDE:
            if altitude >= self.separation_altitude:
                return True

        elif self.separation_trigger == SeparationTrigger.VELOCITY:
            if velocity >= self.separation_velocity:
                return True

        elif self.separation_trigger == SeparationTrigger.MANUAL:
            return False

        return False

    # =========================================================================
    # Multi-Stage Support Methods (used by rocket_6dof_sim.py)
    # =========================================================================

    def arm_separation(self):
        """Arm the separation system."""
        self.separation_armed = True

    def get_separation_event_value(self, t, thrust, altitude=0.0, velocity=0.0):
        """
        Compute a continuous event value for separation detection.

        Positive means no separation; zero or negative means separation.
        This function has no side effects and may be called repeatedly.

        Args:
            t: Current time (s).
            thrust: Current thrust (N).
            altitude: Current altitude (m).
            velocity: Current velocity magnitude (m/s).

        Returns:
            float: Event value; positive = no separation, zero/negative = separation.
        """
        if not self.separation_armed:
            return 1.0

        if self.current_stage >= self.num_stages:
            return 1.0

        if self.separation_in_progress:
            return 1.0

        stage_time = t - self.stage_start_time

        if self.separation_trigger == SeparationTrigger.BURNOUT:
            current_burnout_time = self.stage_burnout_times[self.current_stage - 1] \
                if self.current_stage <= len(self.stage_burnout_times) else self.burnout_time

            # Guard: Don't trigger separation before motor has had time to ignite
            min_stage_time = current_burnout_time * 0.1
            if stage_time < min_stage_time:
                return current_burnout_time - stage_time

            time_margin = current_burnout_time - stage_time
            thrust_margin = thrust - self.burnout_thrust_threshold

            # Normalize thrust_margin to have similar scale as time_margin
            thrust_scale = 0.01
            thrust_margin_normalized = thrust_margin * thrust_scale

            # min() creates AND logic: both conditions must be satisfied
            return min(time_margin, thrust_margin_normalized)

        elif self.separation_trigger == SeparationTrigger.TIME:
            return self.separation_time - stage_time

        elif self.separation_trigger == SeparationTrigger.ALTITUDE:
            return self.separation_altitude - altitude

        elif self.separation_trigger == SeparationTrigger.VELOCITY:
            return self.separation_velocity - velocity

        elif self.separation_trigger == SeparationTrigger.MANUAL:
            return 1.0

        return 1.0

    def execute_separation(self, t):
        """
        Execute stage separation.

        Args:
            t: Current time (s)

        Returns:
            bool: True if separation was initiated
        """
        if self.current_stage >= self.num_stages:
            return False

        self.separation_in_progress = True
        self.separation_complete = False
        self.separation_times.append(t)

        return True

    def complete_separation(self, t):
        """
        Complete the separation process.

        Args:
            t: Current time (s)

        Returns:
            bool: True if separation was completed
        """
        if not self.separation_in_progress:
            return False

        self.current_stage += 1
        self.separation_in_progress = False
        self.separation_complete = True

        if self.current_stage <= len(self.stage_burnout_times):
            self.burnout_time = self.stage_burnout_times[self.current_stage - 1]
            self.burnout_detected = False
            self.burnout_detection_start = None

        # Reset apogee detection state for new stage
        self._reset_apogee_detection()

        return True

    def get_ignition_delay(self, stage_index: int = None) -> float:
        """
        Get ignition delay for a specific stage.

        Args:
            stage_index: 0-based stage index. If None, uses current stage.

        Returns:
            Ignition delay in seconds
        """
        if stage_index is None:
            stage_index = self.current_stage - 1

        if stage_index < len(self.stage_ignition_delays):
            return self.stage_ignition_delays[stage_index]
        return 0.0
