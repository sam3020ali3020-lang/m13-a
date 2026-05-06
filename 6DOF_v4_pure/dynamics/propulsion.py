#!/usr/bin/env python3
"""
Propulsion model: thrust curve loading (.H/CSV/DAT/RocketDataLoader), impulse integration,
telemetry calibration, and altitude-dependent thrust correction.

F = m_dot*V_exit + (P_exit - P_ambient)*A_exit → thrust increases with altitude.
"""

import numpy as np
from scipy.interpolate import interp1d
from scipy.integrate import trapezoid
from pathlib import Path
import re
import logging
from typing import Optional, Union, Dict, TYPE_CHECKING

if TYPE_CHECKING:
    from rocket_data_loader import ThrustCurve

logger = logging.getLogger(__name__)


class PropulsionDataError(Exception):
    """Exception raised when propulsion data cannot be loaded."""
    pass


class AdvancedPropulsionModel:
    """Rocket motor propulsion model with altitude-dependent thrust correction.
    نموذج دفع صاروخي مع تصحيح الدفع المعتمد على الارتفاع."""

    P_SEA_LEVEL = 101325.0

    def __init__(self, config: dict, thrust_curve: Optional['ThrustCurve'] = None):
        """Init propulsion model from config dict and optional pre-loaded ThrustCurve.
        Raises PropulsionDataError if no thrust data (unless motor_type='none')."""

        self.motor_type = config.get('motor_type', 'solid')

        # Stage start time: t_curve = t_sim - stage_start_time (for multi-stage)
        self.stage_start_time = config.get('stage_start_time', 0.0)

        # Store config for use in _parse_data_file
        self._config = config

        # Handle unpowered stages (motor_type='none')
        if self.motor_type == 'none':
            self._initialize_unpowered_stage(config)
            return

        # Load thrust curve from ThrustCurve object if provided (rocket_ws format)
        if thrust_curve is not None:
            self._load_from_thrust_curve_object(thrust_curve)
        elif 'thrust_curve_file' in config and config['thrust_curve_file'] is not None:
            self._load_thrust_curve_from_file(config['thrust_curve_file'])
        elif 'thrust_curve' in config:
            thrust_data = np.array(config['thrust_curve'], dtype=float)
            self.time_data = thrust_data[:, 0]
            self.thrust_data = thrust_data[:, 1]
            
            # Apply same normalization as file-loaded data (sorting, duplicate handling)
            self._normalize_thrust_data(source='config')
            
            # Load mass flow curve if available for accurate propellant tracking
            if 'mass_flow_curve' in config:
                mass_flow_data = np.array(config['mass_flow_curve'], dtype=float)
                if len(mass_flow_data) == len(self.time_data):
                    self._mass_flow_data = mass_flow_data
                    self._mass_flow_interp = interp1d(
                        self.time_data,
                        self._mass_flow_data,
                        kind='linear',
                        bounds_error=False,
                        fill_value=0.0
                    )
                    logger.info(f"Loaded mass flow curve: {len(mass_flow_data)} data points")
                else:
                    logger.warning(
                        f"mass_flow_curve length ({len(mass_flow_data)}) does not match "
                        f"thrust_curve length ({len(self.time_data)}). Ignoring mass flow data."
                    )
                    self._mass_flow_data = None
                    self._mass_flow_interp = None
            else:
                self._mass_flow_data = None
                self._mass_flow_interp = None
        else:
            raise PropulsionDataError(
                "No thrust curve data provided. Provide either thrust_curve object, "
                "thrust_curve_file path, or thrust_curve array in config."
            )

        # Negative thrust allowed (drag at end of burn) / القيم السالبة مسموحة (سحب نهاية الاحتراق)

        # Validate thrust data
        if len(self.time_data) < 2 or len(self.thrust_data) < 2:
            raise PropulsionDataError(
                f"Insufficient thrust data: {len(self.time_data)} time points, "
                f"{len(self.thrust_data)} thrust points. Need at least 2."
            )

        # burn_time_scale: new_time = time×k, new_thrust = thrust/k → same total_impulse
        self.burn_time_scale = config.get('burn_time_scale', 1.0)
        if self.burn_time_scale != 1.0:
            if self.burn_time_scale <= 0:
                raise PropulsionDataError(
                    f"burn_time_scale must be positive, got {self.burn_time_scale}"
                )
            logger.info(
                f"Applying burn_time_scale = {self.burn_time_scale:.4f}: "
                f"time × {self.burn_time_scale:.4f}, thrust / {self.burn_time_scale:.4f}"
            )
            self.time_data = self.time_data * self.burn_time_scale
            self.thrust_data = self.thrust_data / self.burn_time_scale
            # Also scale mass flow data if it exists
            if hasattr(self, '_mass_flow_data') and self._mass_flow_data is not None:
                self._mass_flow_data = self._mass_flow_data / self.burn_time_scale
                self._mass_flow_interp = interp1d(
                    self.time_data,
                    self._mass_flow_data,
                    kind='linear',
                    bounds_error=False,
                    fill_value=0.0
                )

        self.thrust_interp = interp1d(
            self.time_data,
            self.thrust_data,
            kind='linear',
            bounds_error=False,
            fill_value=0.0
        )

        self.total_impulse = self._calculate_total_impulse()

        self.burn_time = self._calculate_burn_time()

        if self.burn_time > 0:
            self.average_thrust = self.total_impulse / self.burn_time
        else:
            self.average_thrust = 0.0

        self.propellant_mass = config.get('propellant_mass', 238.0)  # kg

        # depletion_time: end of thrust curve (more accurate than burn_time for long tail-off)
        self.depletion_time = self.time_data[-1] if len(self.time_data) > 0 else 0.0
        
        # Average mass flow for backward compat
        if self.depletion_time > 0:
            self.mass_flow_rate = self.propellant_mass / self.depletion_time
        else:
            self.mass_flow_rate = 0.0
        
        # Thrust-proportional mass flow: mass_flow(t) = thrust(t) * propellant_mass / total_impulse
        # Ensures ∫mass_flow(t)dt = propellant_mass (mass conservation)
        if not hasattr(self, '_mass_flow_interp') or self._mass_flow_interp is None:
            if self.total_impulse > 0 and self.propellant_mass > 0:
                # Calculate mass flow proportional to thrust
                mass_flow_data = self.thrust_data * (self.propellant_mass / self.total_impulse)
                self._mass_flow_data = mass_flow_data
                self._mass_flow_interp = interp1d(
                    self.time_data,
                    mass_flow_data,
                    kind='linear',
                    bounds_error=False,
                    fill_value=0.0
                )
                logger.info(
                    f"Created thrust-proportional mass flow curve: "
                    f"peak mass_flow = {np.max(mass_flow_data):.3f} kg/s, "
                    f"avg mass_flow = {self.mass_flow_rate:.3f} kg/s"
                )
            else:
                self._mass_flow_data = None
                self._mass_flow_interp = None

        self.thrust_scale = config.get('thrust_scale', 1.0)

        # scale_mass_flow_with_thrust: True=maintain Isp, False=Isp changes with thrust_scale
        self.scale_mass_flow_with_thrust = config.get('scale_mass_flow_with_thrust', True)

        self.nozzle_exit_area = config.get('nozzle_exit_area', 0.0)
        self.reference_pressure = config.get('reference_pressure', self.P_SEA_LEVEL)

        default_altitude_correction = self.nozzle_exit_area > 0
        self.altitude_correction_enabled = config.get(
            'altitude_correction_enabled', default_altitude_correction
        )

        # Validate altitude correction configuration
        self._validate_altitude_correction_config(config)

        # motor_on mode: 'time' (t < burn_time) or 'thrust' (thrust > threshold)
        self.motor_on_mode = config.get('motor_on_mode', 'time')

        # Thrust threshold for motor_on detection (default: max(1N, 0.1% max thrust))
        max_thrust = np.max(self.thrust_data) if len(self.thrust_data) > 0 else 0.0
        default_threshold = max(1.0, 1e-3 * max_thrust)
        self.motor_on_thrust_threshold = config.get(
            'motor_on_thrust_threshold', default_threshold
        )

        # Min thrust for telemetry calibration scale factor (default: 100N)
        self.calibration_thrust_threshold = config.get('calibration_thrust_threshold', 100.0)

        # One-time warning flags
        self._warned_negative_thrust = False

        logger.info(f"Propulsion initialized:")
        logger.info(f"  Total Impulse: {self.total_impulse:.1f} N·s (raw)")
        logger.info(f"  Burn Time: {self.burn_time:.2f} s")
        logger.info(f"  Average Thrust: {self.average_thrust:.1f} N (raw)")
        if self.burn_time_scale != 1.0:
            logger.info(f"  Burn Time Scale: {self.burn_time_scale:.4f}")
        if self.thrust_scale != 1.0:
            logger.info(f"  Thrust Scale: {self.thrust_scale:.4f}")
            logger.info(f"  Scaled Total Impulse: {self.get_scaled_total_impulse():.1f} N·s")
            logger.info(f"  Scaled Average Thrust: {self.get_scaled_average_thrust():.1f} N")
        if self.altitude_correction_enabled:
            logger.info(f"  Altitude Correction: ENABLED")
            logger.info(f"    Nozzle Exit Area: {self.nozzle_exit_area:.6f} m²")
            logger.info(f"    Reference Pressure: {self.reference_pressure:.1f} Pa")
        else:
            logger.info(f"  Altitude Correction: DISABLED")

    def _initialize_unpowered_stage(self, config: dict):
        """Init zero-thrust model for unpowered stages (same interface as powered).
        تهيئة نموذج دفع صفري للمراحل غير المدفوعة (نفس الواجهة)."""
        # Create minimal zero-thrust data (2 points required for interpolation)
        self.time_data = np.array([0.0, 1.0])
        self.thrust_data = np.array([0.0, 0.0])
        
        # Create interpolator that always returns 0
        self.thrust_interp = interp1d(
            self.time_data,
            self.thrust_data,
            kind='linear',
            bounds_error=False,
            fill_value=0.0
        )
        
        # No mass flow for unpowered stages
        self._mass_flow_data = None
        self._mass_flow_interp = None
        
        # Zero propulsion characteristics
        self.total_impulse = 0.0
        self.burn_time = 0.0
        self.average_thrust = 0.0
        self.propellant_mass = config.get('propellant_mass', 0.0)
        self.depletion_time = 0.0
        self.mass_flow_rate = 0.0
        
        # Scaling (no effect for zero thrust, but needed for interface)
        self.thrust_scale = config.get('thrust_scale', 1.0)
        self.scale_mass_flow_with_thrust = config.get('scale_mass_flow_with_thrust', True)
        
        # No altitude correction for unpowered stages
        self.nozzle_exit_area = 0.0
        self.reference_pressure = self.P_SEA_LEVEL
        self.altitude_correction_enabled = False
        
        # Motor detection settings (always off for unpowered)
        self.motor_on_mode = 'time'
        self.motor_on_thrust_threshold = 0.0
        self.calibration_thrust_threshold = 0.0
        
        # Warning flags
        self._warned_negative_thrust = False
        
        logger.info(f"Propulsion initialized (UNPOWERED STAGE):")
        logger.info(f"  Motor Type: none (مرحلة غير مدفوعة)")
        logger.info(f"  Thrust: 0.0 N")
        logger.info(f"  Propellant Mass: {self.propellant_mass:.2f} kg")

    def _validate_altitude_correction_config(self, config: dict):
        """Validate altitude correction config and warn about common mistakes.
        التحقق من إعدادات تصحيح الارتفاع والتحذير من الأخطاء الشائعة."""
        if not self.altitude_correction_enabled:
            return
        
        # Warning 1: nozzle_exit_area is zero but altitude correction is enabled
        if self.nozzle_exit_area <= 0:
            logger.warning(
                "Altitude correction is enabled but nozzle_exit_area is zero or negative. "
                "No pressure thrust correction will be applied. "
                "Set nozzle_exit_area > 0 for altitude correction to take effect. "
                "تصحيح الارتفاع مفعّل لكن مساحة مخرج الفوهة صفر أو سالبة. "
                "لن يتم تطبيق تصحيح دفع الضغط."
            )
            return
        
        # Warning 2: reference_pressure is not sea level and not explicitly set
        if 'reference_pressure' not in config and self.reference_pressure == self.P_SEA_LEVEL:
            logger.info(
                "Altitude correction assumes thrust curve was measured at sea level "
                f"(reference_pressure={self.P_SEA_LEVEL:.0f} Pa). "
                "If your thrust curve was measured at a different pressure (e.g., vacuum), "
                "set 'reference_pressure' in config to match the test conditions. "
                "تصحيح الارتفاع يفترض أن منحنى الدفع تم قياسه عند مستوى سطح البحر. "
                "إذا تم قياس منحنى الدفع عند ضغط مختلف، حدد 'reference_pressure' في الإعدادات."
            )
        
        # Warning 3: reference_pressure is zero (vacuum) - thrust curve should be vacuum thrust
        if self.reference_pressure <= 0:
            logger.warning(
                f"reference_pressure is {self.reference_pressure:.1f} Pa (vacuum or negative). "
                "This assumes your thrust curve represents vacuum thrust. "
                "At sea level, this will REDUCE thrust by up to "
                f"{self.P_SEA_LEVEL * self.nozzle_exit_area:.1f} N. "
                "Verify this is the intended behavior. "
                "الضغط المرجعي صفر أو سالب (فراغ). هذا يفترض أن منحنى الدفع يمثل دفع الفراغ."
            )
        
        # Warning 4: Very large nozzle exit area (potential configuration error)
        # Typical nozzle exit areas are 0.001 to 0.5 m² for most rockets
        if self.nozzle_exit_area > 1.0:
            logger.warning(
                f"nozzle_exit_area ({self.nozzle_exit_area:.4f} m²) is unusually large. "
                "Typical values are 0.001 to 0.5 m² for most rockets. "
                "This could cause excessive altitude correction. "
                "Verify this value is correct. "
                "مساحة مخرج الفوهة كبيرة بشكل غير عادي. تحقق من صحة هذه القيمة."
            )
        
        # Info: Calculate maximum altitude thrust gain (at vacuum)
        max_pressure_gain = self.reference_pressure * self.nozzle_exit_area
        max_thrust = np.max(self.thrust_data) if len(self.thrust_data) > 0 else 0.0
        if max_thrust > 0:
            gain_percentage = (max_pressure_gain / max_thrust) * 100
            logger.debug(
                f"Maximum altitude thrust gain (at vacuum): {max_pressure_gain:.1f} N "
                f"({gain_percentage:.1f}% of max thrust)"
            )

    def get_scaled_total_impulse(self) -> float:
        """
        Get total impulse with thrust_scale applied.
        
        Returns:
            Total impulse in N·s, scaled by thrust_scale
        """
        return self.total_impulse * self.thrust_scale

    def get_scaled_average_thrust(self) -> float:
        """
        Get average thrust with thrust_scale applied.
        
        Returns:
            Average thrust in N, scaled by thrust_scale
        """
        return self.average_thrust * self.thrust_scale

    def get_scaled_mass_flow_rate(self) -> float:
        """
        Get mass flow rate with thrust_scale applied (if scale_mass_flow_with_thrust is True).
        
        Returns:
            Mass flow rate in kg/s, scaled by thrust_scale if scale_mass_flow_with_thrust is True
        """
        if self.scale_mass_flow_with_thrust:
            return self.mass_flow_rate * self.thrust_scale
        return self.mass_flow_rate

    def _load_from_thrust_curve_object(self, thrust_curve: 'ThrustCurve'):
        """
        Load thrust curve from ThrustCurve object (rocket_ws format).

        Args:
            thrust_curve: ThrustCurve dataclass from RocketDataLoader
        """
        logger.info("Loading thrust curve from ThrustCurve object (rocket_ws format)")

        # Copy arrays to avoid modifying the original ThrustCurve object
        self.time_data = np.array(thrust_curve.time, dtype=float)
        self.thrust_data = np.array(thrust_curve.thrust, dtype=float)

        # Store raw mass flow data temporarily (will be synchronized after normalization)
        raw_mass_flow = None
        if thrust_curve.mass_flow is not None:
            raw_mass_flow = np.array(thrust_curve.mass_flow, dtype=float)

        # Apply normalization FIRST (sorting, duplicate handling)
        # This may change time_data and thrust_data arrays
        self._normalize_thrust_data_with_mass_flow(
            source='ThrustCurve object',
            raw_mass_flow=raw_mass_flow
        )
        
        logger.info(f"Loaded thrust curve: {len(self.time_data)} data points")

    def _normalize_thrust_data_core(
        self,
        source: str = 'unknown',
        raw_mass_flow: Optional[np.ndarray] = None
    ) -> Optional[np.ndarray]:
        """Sort by time and handle duplicates; optionally sync mass_flow array.

        Returns synchronized mass_flow array or None."""
        if len(self.time_data) <= 1:
            return raw_mass_flow
        
        # Check if time data is strictly increasing
        time_diff = np.diff(self.time_data)
        if np.any(time_diff <= 0):
            logger.warning(
                f"Time data from {source} is not strictly increasing. "
                f"Sorting data by time. "
                f"بيانات الزمن من {source} ليست متزايدة بشكل صارم. "
                f"يتم ترتيب البيانات حسب الزمن."
            )
            sort_idx = np.argsort(self.time_data)
            self.time_data = self.time_data[sort_idx]
            self.thrust_data = self.thrust_data[sort_idx]
            # Synchronize mass flow with the same sort order
            if raw_mass_flow is not None and len(raw_mass_flow) == len(sort_idx):
                raw_mass_flow = raw_mass_flow[sort_idx]
        
        # Handle duplicate time values by averaging
        unique_times, unique_indices, counts = np.unique(
            self.time_data, return_inverse=True, return_counts=True
        )
        
        if len(unique_times) < len(self.time_data):
            num_duplicates = len(self.time_data) - len(unique_times)
            
            # Check if duplicate times have significantly different thrust values
            max_thrust_diff = 0.0
            for i, count in enumerate(counts):
                if count > 1:
                    mask = unique_indices == i
                    thrust_values = self.thrust_data[mask]
                    max_thrust_diff = max(max_thrust_diff,
                                          np.max(thrust_values) - np.min(thrust_values))
            
            max_thrust = np.max(self.thrust_data) if len(self.thrust_data) > 0 else 0.0
            if max_thrust > 0 and max_thrust_diff > 0.01 * max_thrust:
                logger.warning(
                    f"Found {num_duplicates} duplicate time values in {source} "
                    f"with thrust differences up to {max_thrust_diff:.2f} N. "
                    f"Averaging values at duplicate times. "
                    f"وُجدت {num_duplicates} قيم زمنية مكررة في {source} "
                    f"مع اختلافات في الدفع تصل إلى {max_thrust_diff:.2f} نيوتن. "
                    f"يتم حساب متوسط القيم عند الأزمنة المكررة."
                )
            else:
                logger.debug(
                    f"Found {num_duplicates} duplicate time values in {source}. "
                    f"Averaging values at duplicate times."
                )
            
            # Average thrust values for duplicate times
            averaged_thrust = np.zeros(len(unique_times))
            averaged_mass_flow = np.zeros(len(unique_times)) if raw_mass_flow is not None else None
            
            for i in range(len(unique_times)):
                mask = unique_indices == i
                averaged_thrust[i] = np.mean(self.thrust_data[mask])
                if raw_mass_flow is not None and len(raw_mass_flow) == len(self.thrust_data):
                    averaged_mass_flow[i] = np.mean(raw_mass_flow[mask])
            
            self.time_data = unique_times
            self.thrust_data = averaged_thrust
            raw_mass_flow = averaged_mass_flow
        
        return raw_mass_flow

    def _normalize_thrust_data(self, source: str = 'unknown') -> None:
        """Normalize thrust data (sort + deduplicate). Delegates to _normalize_thrust_data_core.
        تطبيع بيانات الدفع (ترتيب + إزالة التكرار)."""
        self._normalize_thrust_data_core(source=source, raw_mass_flow=None)

    def _normalize_thrust_data_with_mass_flow(
        self,
        source: str = 'unknown',
        raw_mass_flow: Optional[np.ndarray] = None
    ) -> None:
        """Normalize thrust data with synchronized mass flow (sort + deduplicate both).
        تطبيع بيانات الدفع مع مزامنة تدفق الكتلة."""
        # Initialize mass flow attributes
        self._mass_flow_data = None
        self._mass_flow_interp = None
        
        if len(self.time_data) <= 1:
            # Handle single-point case: just set mass flow if available
            if raw_mass_flow is not None and len(raw_mass_flow) >= 1:
                self._mass_flow_data = raw_mass_flow[:len(self.time_data)]
                if len(self._mass_flow_data) >= 2:
                    self._mass_flow_interp = interp1d(
                        self.time_data,
                        self._mass_flow_data,
                        kind='linear',
                        bounds_error=False,
                        fill_value=0.0
                    )
            return
        
        # Use core normalization logic
        normalized_mass_flow = self._normalize_thrust_data_core(
            source=source,
            raw_mass_flow=raw_mass_flow
        )
        
        # Build mass flow interpolator with synchronized data
        if normalized_mass_flow is not None and len(normalized_mass_flow) == len(self.time_data):
            self._mass_flow_data = normalized_mass_flow
            if len(self._mass_flow_data) >= 2:
                self._mass_flow_interp = interp1d(
                    self.time_data,
                    self._mass_flow_data,
                    kind='linear',
                    bounds_error=False,
                    fill_value=0.0
                )
                logger.debug(f"Built mass flow interpolator with {len(self._mass_flow_data)} points")
        elif normalized_mass_flow is not None:
            logger.warning(
                f"Mass flow data length ({len(normalized_mass_flow)}) does not match "
                f"time data length ({len(self.time_data)}) after normalization. "
                f"Ignoring mass flow data. "
                f"طول بيانات تدفق الكتلة ({len(normalized_mass_flow)}) لا يتطابق مع "
                f"طول بيانات الزمن ({len(self.time_data)}) بعد التطبيع. "
                f"يتم تجاهل بيانات تدفق الكتلة."
            )

    def _load_thrust_curve_from_file(self, filepath):
        """Load thrust curve from file (.H, .h, .dat, .csv)."""
        filepath = Path(filepath)

        if not filepath.exists():
            raise PropulsionDataError(f"Thrust curve file not found: {filepath}")

        if filepath.suffix == '.H' or filepath.suffix == '.h':
            self._parse_header_file(filepath)
        elif filepath.suffix == '.dat' or filepath.suffix == '.csv':
            self._parse_data_file(filepath)
        else:
            raise PropulsionDataError(f"Unknown thrust curve file format: {filepath.suffix}. Supported formats: .H, .h, .dat, .csv")

    def _parse_header_file(self, filepath):
        """Parse C/C++ header file: thrust_time[] and thrust_force[] arrays.
        تحليل ملف هيدر C/C++ مع بيانات الدفع."""
        with open(filepath, 'r') as f:
            content = f.read()

        time_match = re.search(r'thrust_time\[\]\s*=\s*\{([^}]+)\}', content)
        if time_match:
            time_str = time_match.group(1)
            self.time_data = np.array([float(x.strip()) for x in time_str.split(',') if x.strip()])
        else:
            raise PropulsionDataError(f"Could not find thrust_time array in header file: {filepath}")

        thrust_match = re.search(r'thrust_force\[\]\s*=\s*\{([^}]+)\}', content)
        if thrust_match:
            thrust_str = thrust_match.group(1)
            self.thrust_data = np.array([float(x.strip()) for x in thrust_str.split(',') if x.strip()])
        else:
            raise PropulsionDataError(f"Could not find thrust_force array in header file: {filepath}")

        min_len = min(len(self.time_data), len(self.thrust_data))
        self.time_data = self.time_data[:min_len]
        self.thrust_data = self.thrust_data[:min_len]

        # Initialize mass flow data as None (header files don't contain mass flow)
        self._mass_flow_data = None
        self._mass_flow_interp = None

        # Apply normalization (sorting, duplicate handling) for consistency with other sources
        self._normalize_thrust_data(source=f'header file {filepath}')

    def _parse_data_file(self, filepath):
        """Parse CSV/DAT file with auto-detection of delimiter and header.
        تحليل ملف بيانات مع كشف تلقائي للفاصل والهيدر."""
        try:
            filepath = Path(filepath)

            # الكشف عن الفاصل بفحص محتوى الملف
            delimiter = None
            skip_header = 0

            with open(filepath, 'r') as f:
                lines = f.readlines()

            # إيجاد أول سطر غير فارغ وغير تعليق
            first_data_line = None
            first_data_idx = 0
            for idx, line in enumerate(lines):
                stripped = line.strip()
                if stripped and not stripped.startswith('#'):
                    first_data_line = stripped
                    first_data_idx = idx
                    break

            if first_data_line is None:
                raise PropulsionDataError(f"No data found in file: {filepath}")

            # الكشف عن الفاصل: فاصلة، فاصلة منقوطة، أو مسافات بيضاء
            if ',' in first_data_line:
                delimiter = ','
            elif ';' in first_data_line:
                delimiter = ';'
            # else: delimiter=None means whitespace (numpy default)

            # Check if first data line is a header (non-numeric)
            parts = first_data_line.split(delimiter) if delimiter else first_data_line.split()
            try:
                float(parts[0].strip())
                # First line is numeric, no header
            except ValueError:
                # First line is header, skip it
                skip_header = first_data_idx + 1

            # Load data with ndmin=2 for consistent 2D shape (single-col vs multi-col)
            data = np.genfromtxt(
                filepath,
                delimiter=delimiter,
                skip_header=skip_header,
                comments='#',
                invalid_raise=False,
                ndmin=2  # Force 2D output for unambiguous shape detection
            )

            # إزالة الصفوف التي تحتوي على قيم NaN
            valid_rows = ~np.any(np.isnan(data), axis=1)
            data = data[valid_rows]

            # Validate: check for empty data after NaN removal
            if data.size == 0 or data.shape[0] == 0:
                raise PropulsionDataError(
                    f"No valid data remaining in {filepath} after removing NaN values. "
                    f"لم تتبق بيانات صالحة في {filepath} بعد إزالة قيم NaN."
                )

            # Determine if this is single-column (thrust only) or multi-column (time, thrust)
            if data.shape[1] == 1:
                # عمود واحد: الدفع فقط، توليد مصفوفة الزمن
                self.thrust_data = data[:, 0]

                # الحصول على الخطوة الزمنية من الإعدادات، مع تحذير إذا تم استخدام القيمة الافتراضية
                default_dt = 0.01
                dt = self._config.get('thrust_curve_dt', default_dt)

                if 'thrust_curve_dt' not in self._config:
                    logger.warning(
                        f"Single-column thrust file detected: {filepath}. "
                        f"Using default time step dt={default_dt}s. "
                        f"This may produce incorrect total_impulse and burn_time calculations "
                        f"if the actual sample rate differs. "
                        f"Set 'thrust_curve_dt' in config to specify the correct time step. "
                        f"تم اكتشاف ملف دفع بعمود واحد: {filepath}. "
                        f"يتم استخدام الخطوة الزمنية الافتراضية dt={default_dt}ث. "
                        f"قد ينتج عن ذلك حسابات خاطئة لـ total_impulse و burn_time "
                        f"إذا كان معدل العينات الفعلي مختلفاً. "
                        f"حدد 'thrust_curve_dt' في الإعدادات لتحديد الخطوة الزمنية الصحيحة."
                    )

                # استخدام arange لتجنب خطأ off-by-one
                self.time_data = np.arange(len(self.thrust_data)) * dt
            else:
                # Two or more columns: time, thrust, and optionally mass_flow
                self.time_data = data[:, 0]
                self.thrust_data = data[:, 1]
                
                # Load mass_flow from third column if available
                raw_mass_flow = None
                if data.shape[1] >= 3:
                    raw_mass_flow = data[:, 2]
                    # Clamp negative mass_flow values to 0
                    raw_mass_flow = np.maximum(raw_mass_flow, 0.0)
                    logger.debug(
                        f"Loaded mass_flow column from {filepath} with {len(raw_mass_flow)} values"
                    )
                
                # Validate: single-row time/thrust files cannot be interpolated
                if len(self.time_data) < 2:
                    raise PropulsionDataError(
                        f"File {filepath} contains only {len(self.time_data)} data row(s) "
                        f"with {data.shape[1]} columns. At least 2 rows are required for "
                        f"time/thrust interpolation. If this is a single-column thrust file, "
                        f"ensure it has only one column of thrust values. "
                        f"الملف {filepath} يحتوي على {len(self.time_data)} صف(وف) بيانات فقط "
                        f"مع {data.shape[1]} أعمدة. مطلوب صفان على الأقل للاستيفاء. "
                        f"إذا كان هذا ملف دفع بعمود واحد، تأكد من أنه يحتوي على عمود واحد فقط."
                    )
                
                # Use unified normalization with mass_flow synchronization
                self._normalize_thrust_data_with_mass_flow(
                    source=f'data file {filepath}',
                    raw_mass_flow=raw_mass_flow
                )
                
                # Final validation: ensure we have at least 2 points for interpolation
                if len(self.time_data) < 2:
                    raise PropulsionDataError(
                        f"After removing duplicates, only {len(self.time_data)} data point(s) "
                        f"remain in {filepath}. At least 2 points are required for interpolation. "
                        f"بعد إزالة التكرارات، بقيت {len(self.time_data)} نقطة بيانات فقط "
                        f"في {filepath}. مطلوب نقطتان على الأقل للاستيفاء."
                    )

            logger.debug(
                f"Loaded thrust curve from {filepath}: "
                f"{len(self.time_data)} points, "
                f"delimiter={'comma' if delimiter == ',' else 'semicolon' if delimiter == ';' else 'whitespace'}, "
                f"header={'yes' if skip_header > 0 else 'no'}"
            )

        except Exception as e:
            raise PropulsionDataError(f"Error parsing data file {filepath}: {e}")

    def _calculate_total_impulse(self):
        """Calculate total impulse by integrating thrust curve."""
        if len(self.time_data) < 2:
            return 0.0

        return trapezoid(self.thrust_data, self.time_data)

    def _calculate_burn_time(self):
        """Calculate burn time (thrust < 10% max) with linear interpolation at crossing.
        Returns 0.0 if no thrust data or max thrust ≈ 0."""
        if len(self.thrust_data) == 0:
            return 0.0

        max_thrust = np.max(self.thrust_data)
        
        # Handle zero or negligible thrust curves
        # A curve with max_thrust <= 0 represents a motor with no thrust
        if max_thrust <= 1e-9:
            return 0.0
        
        threshold = 0.1 * max_thrust

        above_threshold = self.thrust_data > threshold
        if not np.any(above_threshold):
            # This case should not occur if max_thrust > 0, but handle it safely
            return 0.0
        
        last_idx = np.where(above_threshold)[0][-1]
        
        # If last point above threshold is the last data point, return that time
        if last_idx >= len(self.thrust_data) - 1:
            return self.time_data[last_idx]
        
        # Linear interpolation to find exact crossing point
        # We have: thrust[last_idx] > threshold and thrust[last_idx+1] <= threshold
        # Find t where thrust(t) = threshold using linear interpolation
        t1 = self.time_data[last_idx]
        t2 = self.time_data[last_idx + 1]
        f1 = self.thrust_data[last_idx]
        f2 = self.thrust_data[last_idx + 1]
        
        # تجنب القسمة على صفر (لا يجب أن يحدث مع بيانات صالحة)
        if abs(f1 - f2) < 1e-12:
            return t1
        
        # Linear interpolation: t = t1 + (threshold - f1) * (t2 - t1) / (f2 - f1)
        t_crossing = t1 + (threshold - f1) * (t2 - t1) / (f2 - f1)
        
        return t_crossing

    def get_thrust(self, t: float, ambient_pressure: Optional[float] = None,
                   mach: float = 0.0, altitude: float = 0.0,
                   density: float = 1.225, speed_of_sound: float = 340.0,
                   dt: float = 0.01,
                   update_state: bool = True) -> Dict[str, Union[float, bool]]:
        """Get thrust at time t with optional altitude correction.
        F_altitude = F_reference + (P_reference - P_ambient) * A_exit

        Returns dict: thrust, thrust_base, thrust_pressure_gain, mass_flow (diagnostic),
                      specific_impulse, motor_on."""
        # Stage-relative time (multi-stage: thrust curve starts at t=0 per stage)
        t_stage = t - self.stage_start_time

        thrust_base = float(self.thrust_interp(t_stage)) * self.thrust_scale

        thrust_pressure_gain = 0.0
        # Altitude correction only when motor is burning (prevents thrust without mass flow)
        if (self.altitude_correction_enabled
                and ambient_pressure is not None
                and thrust_base > self.motor_on_thrust_threshold):
            pressure_diff = self.reference_pressure - ambient_pressure
            thrust_pressure_gain = pressure_diff * self.nozzle_exit_area

            # Clamp thrust_pressure_gain to prevent negative total thrust
            # This can happen if ambient_pressure > reference_pressure
            # (e.g., if reference_pressure was set to vacuum pressure)
            if thrust_base + thrust_pressure_gain < 0:
                if not self._warned_negative_thrust:
                    logger.warning(
                        f"Thrust pressure correction would result in negative thrust. "
                        f"thrust_base={thrust_base:.1f} N, thrust_pressure_gain={thrust_pressure_gain:.1f} N. "
                        f"Clamping thrust_pressure_gain to -{thrust_base:.1f} N. "
                        f"Check reference_pressure ({self.reference_pressure:.1f} Pa) and "
                        f"nozzle_exit_area ({self.nozzle_exit_area:.6f} m²) settings. "
                        f"تصحيح ضغط الدفع سيؤدي إلى دفع سالب. يتم تقييد القيمة."
                    )
                    self._warned_negative_thrust = True
                thrust_pressure_gain = -thrust_base  # Makes thrust = 0

        thrust = thrust_base + thrust_pressure_gain

        # تحديد ما إذا كان المحرك يعمل
        if self.motor_on_mode == 'thrust':
            # Improved mode: based on actual thrust (accounts for ignition delay and tail)
            motor_on = thrust_base > self.motor_on_thrust_threshold
        else:
            # Legacy mode: time-based (t_stage can be negative before ignition)
            # Use depletion_time (end of thrust curve) to include tail-off phase
            motor_on = 0 <= t_stage < self.depletion_time

        # حساب معدل تدفق الكتلة
        if motor_on:
            # Use interpolated mass flow curve (thrust-proportional or from data)
            # Use stage-relative time for interpolation
            if self._mass_flow_interp is not None:
                mass_flow = float(self._mass_flow_interp(t_stage))
                # Ensure non-negative and finite value
                if not np.isfinite(mass_flow) or mass_flow < 0:
                    mass_flow = 0.0
            else:
                # Fallback to average mass flow rate (only for edge cases like zero impulse)
                mass_flow = self.mass_flow_rate

            # Scale mass_flow with thrust_scale if enabled (default: True)
            # This maintains consistent Isp when thrust_scale is applied
            # Physics: If thrust is scaled by k, mass_flow should also scale by k
            # to maintain Isp = thrust / (mass_flow * g0)
            if self.scale_mass_flow_with_thrust:
                mass_flow = mass_flow * self.thrust_scale
        else:
            mass_flow = 0.0

        if mass_flow > 0:
            Isp = thrust / (mass_flow * 9.80665)
        else:
            Isp = 0.0

        return {
            'thrust': thrust,
            'thrust_base': thrust_base,
            'thrust_pressure_gain': thrust_pressure_gain,
            'mass_flow': mass_flow,
            'specific_impulse': Isp,
            'motor_on': motor_on
        }

    def get_total_impulse(self) -> float:
        """Get total impulse of the motor."""
        return self.total_impulse

    def get_burn_time(self) -> float:
        """Burn time (thrust < 10% max). For mass calc use get_depletion_time(). / وقت الاحتراق."""
        return self.burn_time

    def get_burnout_time(self) -> float:
        """Alias for get_burn_time() (consistency with mass_properties.py)."""
        return self.burn_time

    def get_depletion_time(self) -> float:
        """Full thrust curve duration (all propellant consumed). Differs from burn_time for long tail-off.
        وقت النفاد الكامل (استهلاك كل الوقود)."""
        return self.depletion_time

    def calibrate_from_telemetry(
        self,
        time_array: np.ndarray,
        accel_array: np.ndarray,
        mass_array: np.ndarray,
        aero_force_array: np.ndarray,
        gravity_body_x: Optional[Union[float, np.ndarray]] = None,
        ambient_pressure_array: Optional[np.ndarray] = None
    ) -> Dict[str, Union[float, np.ndarray]]:
        """
        Calibrate thrust curve from telemetry using Newton's 2nd law on body X-axis.
        معايرة منحنى الدفع من بيانات القياس عن بعد.

        T_x = m*a_x - F_aero_x - m*g_x  (kinematic accel, provide gravity_body_x)
        T_x = m*f_x - F_aero_x           (specific force/accelerometer, gravity_body_x=None)

        F_aero_x is SIGNED (negative for drag). All inputs in body frame (+X nose, +Y right, +Z down).

        Args:
            time_array: Time (s)
            accel_array: Body X accel (m/s²) - kinematic or specific force
            mass_array: Instantaneous mass (kg)
            aero_force_array: Aero force body X (N), signed (negative=drag)
            gravity_body_x: Gravity on body X (m/s²), provide if kinematic accel, None if specific force
            ambient_pressure_array: Ambient pressure (Pa), for altitude-corrected calibration

        Returns:
            dict: scale_factor, time, thrust_telemetry, thrust_sim
        """

        # Newton's 2nd law in body X-axis:
        # m*a_kin = T + F_aero + m*g  =>  T = m*a_kin - F_aero - m*g
        # OR if using specific force (f = a_kin - g):
        # m*f = T + F_aero  =>  T = m*f - F_aero
        
        if gravity_body_x is not None:
            # accel_array هو التسارع الحركي، اطرح الجاذبية
            thrust_telemetry = mass_array * (accel_array - gravity_body_x) - aero_force_array
        else:
            # accel_array هو القوة النوعية (الجاذبية مستبعدة بالفعل)
            thrust_telemetry = mass_array * accel_array - aero_force_array

        burn_mask = time_array < self.burn_time
        time_burn = time_array[burn_mask]
        thrust_burn = thrust_telemetry[burn_mask]

        # Apply thrust_scale to get actual simulated thrust for comparison
        thrust_base = self.thrust_interp(time_burn) * self.thrust_scale
        
        # Apply altitude correction if ambient_pressure_array is provided
        # Validate ambient_pressure_array length matches time_array
        if ambient_pressure_array is not None and len(ambient_pressure_array) != len(time_array):
            raise ValueError(
                f"ambient_pressure_array length ({len(ambient_pressure_array)}) must match "
                f"time_array length ({len(time_array)}). "
                f"طول ambient_pressure_array ({len(ambient_pressure_array)}) يجب أن يطابق "
                f"طول time_array ({len(time_array)})."
            )
        if ambient_pressure_array is not None and self.altitude_correction_enabled and self.nozzle_exit_area > 0:
            # Vectorized altitude correction: F = F_base + (P_ref - P_ambient) * A_exit
            pressure_burn = ambient_pressure_array[burn_mask]
            pressure_gain = (self.reference_pressure - pressure_burn) * self.nozzle_exit_area
            # Clamp to prevent negative thrust (same logic as get_thrust)
            thrust_sim = np.maximum(thrust_base + pressure_gain, 0.0)
        else:
            # Without ambient pressure, compare against base/reference-condition thrust
            thrust_sim = thrust_base

        # استخدم فقط النقاط ذات الدفع الكبير (عتبة قابلة للتكوين)
        valid_mask = thrust_sim > self.calibration_thrust_threshold
        if np.sum(valid_mask) > 0:
            scale_factor = np.mean(thrust_burn[valid_mask] / thrust_sim[valid_mask])
        else:
            scale_factor = 1.0

        return {
            'scale_factor': scale_factor,
            'time': time_burn,
            'thrust_telemetry': thrust_burn,
            'thrust_sim': thrust_sim
        }

    def set_thrust_scale(self, scale):
        """Set thrust scaling factor."""
        self.thrust_scale = scale
        logger.info(f"Thrust scale factor set to: {scale:.3f}")

    def set_stage_start_time(self, t: float):
        """Set stage start time for multi-stage rockets (thrust curve is relative to this).
        تعيين وقت بدء المرحلة (منحنى الدفع نسبي لهذا الوقت)."""
        self.stage_start_time = t
        logger.info(f"Stage start time set to: {t:.3f}s")
