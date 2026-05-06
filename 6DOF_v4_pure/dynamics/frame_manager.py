#!/usr/bin/env python3
"""
Frame Consistency Manager.

Centralizes 6DOF frame transforms and consistency checks.

Frames:
- ECEF for long-range tracking.
- Local NED, launch-fixed NED, and body.

Rules:
1. If use_launch_fixed_ned=True, state quaternion and guidance use launch-fixed NED.
2. Otherwise, the state quaternion uses local NED and guidance must be transformed.
3. include_earth_rate should match long_range_mode.
"""

import numpy as np
from typing import Dict, Optional, Tuple, Any, List
from dataclasses import dataclass, field
from enum import Enum
import logging
import warnings

# Import quaternion utilities from the single source of truth.
# Keep all quaternion operations and thresholds centralized in quaternion_utils.py.
from dynamics.quaternion_utils import (
    quaternion_multiply,
    rotation_matrix_to_quaternion_raw,
    normalize_quaternion,
    compute_velocity_aligned_quaternion as _compute_velocity_aligned_quat,
    QUAT_NORM_THRESHOLD,
    VECTOR_NORM_THRESHOLD,
)
from dynamics.quaternion_state_manager import QuaternionStateManager

logger = logging.getLogger(__name__)


class FrameMode(Enum):
    """
    Available reference frame modes
    """
    LAUNCH_FIXED_NED = "launch_fixed_ned"
    LOCAL_NED = "local_ned"
    FUR = "fur"  # Forward-Up-Right local frame / إطار أمام-أعلى-يمين المحلي


# FUR frame transform constants.
# NED -> FUR: X = X, Y = -Z, Z = Y.
# Rotation matrix from NED to FUR.
C_NED_TO_FUR = np.array([
    [1.0,  0.0,  0.0],   # X_FUR = X_NED (Forward = North)
    [0.0,  0.0, -1.0],   # Y_FUR = -Z_NED (Up = -Down)
    [0.0,  1.0,  0.0]    # Z_FUR = Y_NED (Right = East)
])

# Rotation matrix from FUR to NED frame (transpose of C_NED_TO_FUR)
C_FUR_TO_NED = C_NED_TO_FUR.T


def transform_ned_to_fur(vector_ned: np.ndarray) -> np.ndarray:
    """
    Transform a vector from NED to FUR frame.
    
    Args:
        vector_ned: Vector in NED frame [North, East, Down].
    
    Returns:
        Vector in FUR frame [Forward, Up, Right].
    """
    return C_NED_TO_FUR @ vector_ned


@dataclass(frozen=True)
class FrameConfiguration:
    """
    Shared immutable reference-frame configuration.

    Create once in the simulation and pass to all subsystems to keep
    frame parameters consistent and prevent accidental divergence.
    """
    long_range_mode: bool = False
    use_launch_fixed_ned: bool = True
    include_earth_rate: bool = False
    include_coriolis: bool = True
    include_centrifugal: bool = True
    
    launch_lat_rad: float = 0.0
    launch_lon_rad: float = 0.0
    launch_alt_m: float = 0.0
    
    # Note: dataclass freezing prevents reassignment, not in-place array mutation.
    # Do not modify these arrays after creation.
    launch_ecef: Optional[tuple] = None  # Changed to tuple for true immutability
    
    C_ecef_to_ned_launch: Optional[tuple] = None  # Changed to tuple for true immutability
    
    def get_launch_ecef_array(self) -> np.ndarray:
        """Get launch ECEF as numpy array."""
        if self.launch_ecef is None:
            return np.zeros(3)
        return np.array(self.launch_ecef)
    
    def get_C_ecef_to_ned_launch_array(self) -> Optional[np.ndarray]:
        """Get C_ecef_to_ned_launch as numpy array."""
        if self.C_ecef_to_ned_launch is None:
            return None
        return np.array(self.C_ecef_to_ned_launch).reshape(3, 3)


@dataclass
class FrameTransformCache:
    """
    Cache for frame transformation matrices.

    Avoids redundant computations and keeps per-timestep transforms consistent.
    """
    timestamp: float = -1.0
    C_ecef_to_ned_current: Optional[np.ndarray] = None
    C_ned_to_ecef_current: Optional[np.ndarray] = None
    C_ecef_to_ned_launch: Optional[np.ndarray] = None
    C_ned_current_to_launch: Optional[np.ndarray] = None
    C_ned_launch_to_current: Optional[np.ndarray] = None
    current_lat_rad: float = 0.0
    current_lon_rad: float = 0.0
    current_alt_m: float = 0.0
    
    def is_valid_for(self, timestamp: float, tolerance: float = 1e-9) -> bool:
        """Check if cache is valid for the given timestamp."""
        return abs(self.timestamp - timestamp) < tolerance
    
    def invalidate(self):
        """Invalidate the cache."""
        self.timestamp = -1.0


class FrameConsistencyError(Exception):
    """
    Frame consistency error
    
    Raised when frame configuration is inconsistent across subsystems.
    """
    pass


class DeprecatedKeyDict(dict):
    """
    Dictionary wrapper that emits deprecation warnings for deprecated keys.
    
    Example:
        >>> deprecated_keys = {'old_key': 'new_key'}
        >>> d = DeprecatedKeyDict({'new_key': 42, 'old_key': 42}, deprecated_keys)
        >>> d['old_key']  # Issues DeprecationWarning
        42
        >>> d['new_key']  # No warning
        42
    """
    
    def __init__(self, data: Dict[str, Any], deprecated_keys: Dict[str, str]):
        """
        Initialize the dictionary with deprecation mappings.

        Args:
            data: The dictionary data
            deprecated_keys: Mapping of deprecated key names to their replacements
                            {deprecated_key: replacement_key}
        """
        super().__init__(data)
        self._deprecated_keys = deprecated_keys
        self._warned_keys: set = set()  # Track which keys have already warned

    def _warn_if_deprecated(self, key: str, stacklevel: int = 3) -> None:
        """Emit a DeprecationWarning if key is deprecated (once per key)."""
        if key in self._deprecated_keys and key not in self._warned_keys:
            replacement = self._deprecated_keys[key]
            warnings.warn(
                f"Key '{key}' is deprecated and will be removed in a future version. "
                f"Use '{replacement}' instead. "
                f"المفتاح '{key}' مهمل وسيتم إزالته في إصدار مستقبلي. "
                f"استخدم '{replacement}' بدلاً من ذلك.",
                DeprecationWarning,
                stacklevel=stacklevel
            )
            self._warned_keys.add(key)

    def __getitem__(self, key: str) -> Any:
        """Get item with deprecation warning for deprecated keys."""
        self._warn_if_deprecated(key)
        return super().__getitem__(key)

    def __contains__(self, key: object) -> bool:
        """Check membership with deprecation warning for deprecated keys."""
        if isinstance(key, str):
            self._warn_if_deprecated(key)
        return super().__contains__(key)

    def get(self, key: str, default: Any = None) -> Any:
        """Get item with deprecation warning for deprecated keys."""
        self._warn_if_deprecated(key)
        return super().get(key, default)

    def pop(self, key: str, *args) -> Any:
        """Pop item with deprecation warning for deprecated keys."""
        self._warn_if_deprecated(key)
        return super().pop(key, *args)


class FrameConsistencyManager:
    """
    Central manager for frame transforms and consistency checks.

    Shares frame config/cache, validates subsystem settings,
    and provides transform utilities.

    Use a shared QuaternionStateManager to preserve sign continuity
    across subsystems.
    """
    
    COORD_TOLERANCE_RAD = 1e-4
    MATRIX_TOLERANCE = 1e-10
    
    def __init__(
        self,
        earth_model=None,
        quaternion_state_manager: QuaternionStateManager = None,
        require_shared_manager: bool = True
    ):
        """
        Initialize the frame manager.

        Args:
            earth_model: EarthModel for coordinate transforms.
            quaternion_state_manager: Shared QuaternionStateManager for sign continuity.
            require_shared_manager: If True, raise if no shared manager is provided.

        Raises:
            FrameConsistencyError: If a shared QuaternionStateManager is required but missing.
        """
        self.earth_model = earth_model
        self.config = FrameConfiguration()
        self._cache = FrameTransformCache()
        self._initialized = False
        self._validation_errors: List[str] = []
        self._validation_warnings: List[str] = []
        
        # Shared QuaternionStateManager is required by default for sign continuity.
        # Set require_shared_manager=False only for testing/standalone usage.
        if quaternion_state_manager is None:
            if require_shared_manager:
                error_msg = (
                    "FrameConsistencyManager REQUIRES a shared QuaternionStateManager. "
                    "This is MANDATORY to prevent sign continuity issues that cause "
                    "quaternion sign flips during frame transformations. "
                    "SOLUTION: Create a QuaternionStateManager and pass it to all subsystems. "
                    "For testing: qsm = QuaternionStateManager(); FrameConsistencyManager(quaternion_state_manager=qsm) "
                    "Or set require_shared_manager=False for standalone usage. "
                    "يتطلب FrameConsistencyManager مدير QuaternionStateManager مشترك. "
                    "هذا إلزامي لمنع مشاكل استمرارية الإشارة التي تسبب "
                    "انقلابات في إشارة الرباعي أثناء تحويلات الإطار. "
                    "الحل: أنشئ QuaternionStateManager ومرره لجميع الأنظمة الفرعية."
                )
                logger.error(error_msg)
                raise FrameConsistencyError(error_msg)
            else:
                # Create a local QuaternionStateManager for standalone/testing usage
                # إنشاء QuaternionStateManager محلي للاستخدام المستقل/الاختبار
                self.quaternion_state_manager = QuaternionStateManager()
                self._using_shared_quaternion_state_manager = False
                logger.warning(
                    "FrameConsistencyManager created with LOCAL QuaternionStateManager. "
                    "This may cause sign continuity issues if multiple subsystems are used. "
                    "For production use, pass a shared QuaternionStateManager. "
                    "(تم إنشاء مدير اتساق الإطارات مع QuaternionStateManager محلي. "
                    "قد يسبب هذا مشاكل استمرارية الإشارة إذا تم استخدام أنظمة فرعية متعددة.)"
                )
        else:
            self.quaternion_state_manager = quaternion_state_manager
            self._using_shared_quaternion_state_manager = True
            logger.debug(
                "FrameConsistencyManager using SHARED QuaternionStateManager "
                "(مدير اتساق الإطارات يستخدم QuaternionStateManager مشترك)"
            )
        
        # Intermediate quaternion sign continuity is managed by QuaternionStateManager.
        # See: get_intermediate_quaternion_reference(),
        # set_intermediate_quaternion_reference(), apply_intermediate_sign_continuity().
        
    def initialize(
        self,
        long_range_mode: bool,
        use_launch_fixed_ned: bool,
        include_earth_rate: bool,
        launch_lat_rad: float,
        launch_lon_rad: float,
        launch_alt_m: float,
        earth_model=None,
        include_coriolis: bool = True,
        include_centrifugal: bool = True,
        frame_configuration: Optional[FrameConfiguration] = None
    ):
        """
        Initialize frame configuration.

        Args:
            long_range_mode: Enable ECEF tracking.
            use_launch_fixed_ned: Use launch-fixed instead of local NED.
            include_earth_rate: Include Earth rotation.
            launch_lat_rad: Launch latitude (rad).
            launch_lon_rad: Launch longitude (rad).
            launch_alt_m: Launch altitude (m).
            earth_model: EarthModel instance.
            include_coriolis: Include Coriolis.
            include_centrifugal: Include centrifugal.
            frame_configuration: Optional shared FrameConfiguration overriding frame params.
        """
        if earth_model is not None:
            self.earth_model = earth_model
            
        if self.earth_model is None:
            raise ValueError(
                "EarthModel is required for FrameConsistencyManager. "
                "نموذج الأرض مطلوب لمدير اتساق الإطارات."
            )
        
        # Shared configuration pattern:
        # use the provided FrameConfiguration instance so all subsystems
        # share one source of truth and avoid parameter drift.
        if frame_configuration is not None:
            self.config = frame_configuration
            logger.info(
                "FrameConsistencyManager using SHARED FrameConfiguration instance. "
                "مدير اتساق الإطارات يستخدم مثيل FrameConfiguration مشترك."
            )
        else:
            # Create new FrameConfiguration with computed ECEF and rotation matrix
            launch_ecef = self.earth_model.lla_to_ecef(
                launch_lat_rad, launch_lon_rad, launch_alt_m
            )
            C_ecef_to_ned_launch = self.earth_model.rot_ecef_to_ned(
                launch_lat_rad, launch_lon_rad
            )
            
            # Convert numpy arrays to tuples for frozen dataclass immutability
            self.config = FrameConfiguration(
                long_range_mode=long_range_mode,
                use_launch_fixed_ned=use_launch_fixed_ned,
                include_earth_rate=include_earth_rate,
                include_coriolis=include_coriolis,
                include_centrifugal=include_centrifugal,
                launch_lat_rad=launch_lat_rad,
                launch_lon_rad=launch_lon_rad,
                launch_alt_m=launch_alt_m,
                launch_ecef=tuple(launch_ecef),
                C_ecef_to_ned_launch=tuple(C_ecef_to_ned_launch.flatten())
            )
        
        self._validate_configuration()
        
        self._initialized = True
        
        logger.info(
            f"FrameConsistencyManager initialized: "
            f"long_range_mode={self.config.long_range_mode}, "
            f"use_launch_fixed_ned={self.config.use_launch_fixed_ned}, "
            f"include_earth_rate={self.config.include_earth_rate}, "
            f"launch=({np.degrees(self.config.launch_lat_rad):.6f}°, {np.degrees(self.config.launch_lon_rad):.6f}°, {self.config.launch_alt_m:.1f}m)"
        )
        
    def _validate_configuration(self):
        """
        Validate frame configuration for internal consistency.
        """
        self._validation_errors.clear()
        self._validation_warnings.clear()
        
        if self.config.long_range_mode and not self.config.include_earth_rate:
            self._validation_warnings.append(
                "long_range_mode=True but include_earth_rate=False. "
                "This may cause errors for long-range trajectories. "
                "Recommended: set include_earth_rate=True when long_range_mode=True. "
                "long_range_mode=True لكن include_earth_rate=False. "
                "قد يسبب هذا أخطاء للمسارات طويلة المدى."
            )
            
        if not self.config.long_range_mode and self.config.include_earth_rate:
            self._validation_warnings.append(
                "long_range_mode=False but include_earth_rate=True. "
                "Earth rate effects are typically negligible for short-range (<50km). "
                "long_range_mode=False لكن include_earth_rate=True. "
                "تأثيرات دوران الأرض عادة مهملة للمدى القصير (<50 كم)."
            )
            
        C = self.config.get_C_ecef_to_ned_launch_array()
        if C is not None:
            det = np.linalg.det(C)
            if abs(det - 1.0) > self.MATRIX_TOLERANCE:
                self._validation_errors.append(
                    f"C_ecef_to_ned_launch is not a valid rotation matrix (det={det:.6f}). "
                    f"C_ecef_to_ned_launch ليست مصفوفة دوران صالحة."
                )
                
            orthogonality_error = np.linalg.norm(C @ C.T - np.eye(3))
            if orthogonality_error > self.MATRIX_TOLERANCE:
                self._validation_errors.append(
                    f"C_ecef_to_ned_launch is not orthogonal (error={orthogonality_error:.2e}). "
                    f"C_ecef_to_ned_launch ليست متعامدة."
                )
        
        for warning in self._validation_warnings:
            logger.warning(f"[FrameConsistencyManager] {warning}")
            
        if self._validation_errors:
            error_msg = "\n".join(self._validation_errors)
            logger.error(f"[FrameConsistencyManager] Configuration errors:\n{error_msg}")
            raise FrameConsistencyError(error_msg)
            
    def validate_subsystem_consistency(
        self,
        subsystem_name: str,
        subsystem_config: Dict[str, Any]
    ) -> Tuple[bool, List[str]]:
        """
        Validate subsystem frame settings against manager configuration.

        Args:
            subsystem_name: Subsystem name.
            subsystem_config: Subsystem configuration.

        Returns:
            (is_consistent, issues).
        """
        issues = []
        
        if 'include_earth_rate' in subsystem_config:
            if subsystem_config['include_earth_rate'] != self.config.include_earth_rate:
                issues.append(
                    f"{subsystem_name}.include_earth_rate={subsystem_config['include_earth_rate']} "
                    f"but FrameManager.include_earth_rate={self.config.include_earth_rate}. "
                    f"This WILL cause frame inconsistency errors."
                )
                
        if 'use_launch_fixed_ned' in subsystem_config:
            if subsystem_config['use_launch_fixed_ned'] != self.config.use_launch_fixed_ned:
                issues.append(
                    f"{subsystem_name}.use_launch_fixed_ned={subsystem_config['use_launch_fixed_ned']} "
                    f"but FrameManager.use_launch_fixed_ned={self.config.use_launch_fixed_ned}. "
                    f"This WILL cause quaternion frame mismatch."
                )
                
        if 'long_range_mode' in subsystem_config:
            if subsystem_config['long_range_mode'] != self.config.long_range_mode:
                issues.append(
                    f"{subsystem_name}.long_range_mode={subsystem_config['long_range_mode']} "
                    f"but FrameManager.long_range_mode={self.config.long_range_mode}. "
                    f"This WILL cause coordinate frame errors."
                )
        
        for coord_key in ['latitude_rad', 'launch_lat_rad']:
            if coord_key in subsystem_config:
                lat_diff = abs(subsystem_config[coord_key] - self.config.launch_lat_rad)
                if lat_diff > self.COORD_TOLERANCE_RAD:
                    issues.append(
                        f"{subsystem_name}.{coord_key} differs from launch latitude by "
                        f"{np.degrees(lat_diff):.6f}°. This may cause frame transformation errors."
                    )
                    
        for coord_key in ['longitude_rad', 'launch_lon_rad']:
            if coord_key in subsystem_config:
                lon_diff = abs(subsystem_config[coord_key] - self.config.launch_lon_rad)
                if lon_diff > self.COORD_TOLERANCE_RAD:
                    issues.append(
                        f"{subsystem_name}.{coord_key} differs from launch longitude by "
                        f"{np.degrees(lon_diff):.6f}°. This may cause frame transformation errors."
                    )
        
        # Check if shared QuaternionStateManager is being used
        # This is critical for sign continuity across subsystems
        if not self._using_shared_quaternion_state_manager:
            issues.append(
                f"FrameConsistencyManager is NOT using a shared QuaternionStateManager. "
                f"This WILL cause quaternion sign flips during frame transformations. "
                f"SOLUTION: Pass the simulation's QuaternionStateManager when creating "
                f"FrameConsistencyManager. "
                f"مدير اتساق الإطارات لا يستخدم QuaternionStateManager مشترك. "
                f"هذا سيسبب انقلابات في إشارة الرباعي أثناء تحويلات الإطارات. "
                f"الحل: مرر QuaternionStateManager الخاص بالمحاكاة عند إنشاء FrameConsistencyManager."
            )
        
        is_consistent = len(issues) == 0
        
        if not is_consistent:
            for issue in issues:
                logger.warning(f"[FrameConsistencyManager] {issue}")
                
        return is_consistent, issues
        
    def update_cache(
        self,
        timestamp: float,
        current_lat_rad: float,
        current_lon_rad: float,
        current_alt_m: float
    ):
        """
        Update transform cache for the current state.

        Args:
            timestamp: Simulation time.
            current_lat_rad: Current latitude (rad).
            current_lon_rad: Current longitude (rad).
            current_alt_m: Current altitude (m).
        """
        if not self._initialized:
            raise RuntimeError(
                "FrameConsistencyManager not initialized. Call initialize() first. "
                "مدير اتساق الإطارات غير مُهيأ. استدعِ initialize() أولاً."
            )
            
        self._cache.timestamp = timestamp
        self._cache.current_lat_rad = current_lat_rad
        self._cache.current_lon_rad = current_lon_rad
        self._cache.current_alt_m = current_alt_m
        
        if self.config.long_range_mode:
            self._cache.C_ecef_to_ned_current = self.earth_model.rot_ecef_to_ned(
                current_lat_rad, current_lon_rad
            )
            self._cache.C_ned_to_ecef_current = self._cache.C_ecef_to_ned_current.T
            
            self._cache.C_ecef_to_ned_launch = self.config.get_C_ecef_to_ned_launch_array()
            
            self._cache.C_ned_current_to_launch = (
                self._cache.C_ecef_to_ned_launch @ self._cache.C_ned_to_ecef_current
            )
            self._cache.C_ned_launch_to_current = self._cache.C_ned_current_to_launch.T
        else:
            self._cache.C_ecef_to_ned_current = np.eye(3)
            self._cache.C_ned_to_ecef_current = np.eye(3)
            self._cache.C_ecef_to_ned_launch = np.eye(3)
            self._cache.C_ned_current_to_launch = np.eye(3)
            self._cache.C_ned_launch_to_current = np.eye(3)
        
    def get_frame_data(self, position: np.ndarray, velocity: np.ndarray, timestamp: float) -> Dict[str, Any]:
        """
        Build frame data for the current state.

        Input frame by mode:
        - long_range_mode=True: ECEF position/velocity.
        - long_range_mode=False: NED position/velocity.

        Args:
            position: Position (m) in ECEF or NED by mode.
            velocity: Velocity (m/s) in ECEF or NED by mode.
            timestamp: Simulation time (s).

        Returns:
            Dict with altitude, lat/lon, transforms, NED/launch-NED
            position/velocity, state-frame velocity, and frame mode.
        """
        if not self._initialized:
            raise RuntimeError(
                "FrameConsistencyManager not initialized. Call initialize() first. "
                "مدير اتساق الإطارات غير مُهيأ. استدعِ initialize() أولاً."
            )
        
        # Frame-dependent inputs:
        # long_range_mode=True uses ECEF position/velocity;
        # otherwise inputs are NED at the launch site.
        
        if self.config.long_range_mode:
            # Long-range mode: Position and velocity are in ECEF frame
            lat, lon, alt = self.earth_model.ecef_to_lla(
                position[0], position[1], position[2]
            )
            altitude = alt  # WGS84 geodetic altitude (no clamping in long-range mode)
            
            self.update_cache(timestamp, lat, lon, altitude)
            
            # Transform ECEF velocity to local NED
            vel_ned = self._cache.C_ecef_to_ned_current @ velocity
            
            # Compute position relative to launch site in ECEF, then transform to launch NED
            pos_relative_ecef = position - self.config.get_launch_ecef_array()
            pos_ned_launch = self._cache.C_ecef_to_ned_launch @ pos_relative_ecef
            
            # Transform ECEF velocity to launch-fixed NED
            vel_ned_launch = self._cache.C_ecef_to_ned_launch @ velocity
        else:
            # Non-long-range mode: inputs are already launch-site NED.
            # Altitude is computed as -z (NED uses positive down).
            altitude = max(-position[2], 0.0)
            
            # Use launch site coordinates for frame transformations
            lat = self.config.launch_lat_rad
            lon = self.config.launch_lon_rad
            
            self.update_cache(timestamp, lat, lon, altitude)
            
            # In non-long-range mode, velocity is already in NED frame
            # For short-range, local NED ≈ launch-fixed NED (negligible difference)
            vel_ned = velocity.copy()
            
            # Position in launch-fixed NED is the input position itself
            pos_ned_launch = position.copy()
            
            # Velocity in launch-fixed NED is the input velocity itself
            vel_ned_launch = velocity.copy()
        
        frame_mode = (
            FrameMode.LAUNCH_FIXED_NED if self.config.use_launch_fixed_ned 
            else FrameMode.LOCAL_NED
        )
        
        # Deprecated key mapping:
        # maps old keys to replacements and emits DeprecationWarning on access.
        deprecated_keys = {
            # 'vel_ecef' is misleading because in non-long_range_mode,
            # the velocity is in NED frame, not ECEF.
            'vel_ecef': 'velocity_state_frame',
        }
        
        frame_data = {
            'altitude': altitude,
            'lat': lat,
            'lon': lon,
            'C_ecef_to_ned': self._cache.C_ecef_to_ned_current,
            'C_ned_to_ecef': self._cache.C_ned_to_ecef_current,
            'C_ecef_to_ned_launch': self._cache.C_ecef_to_ned_launch,
            'C_ned_current_to_launch': self._cache.C_ned_current_to_launch,
            'C_ned_launch_to_current': self._cache.C_ned_launch_to_current,
            'vel_ned': vel_ned,
            'vel_ned_launch': vel_ned_launch,
            'pos_ned_launch': pos_ned_launch,
            # Preferred key: velocity in the simulation state frame
            # (ECEF in long-range mode, NED otherwise).
            'velocity_state_frame': velocity,
            # Deprecated compatibility key.
            # Access emits DeprecationWarning; use velocity_state_frame instead.
            'vel_ecef': velocity,
            'frame_mode': frame_mode,
            'use_launch_fixed_ned': self.config.use_launch_fixed_ned,
        }
        
        # Return DeprecatedKeyDict which issues runtime warnings for deprecated keys
        return DeprecatedKeyDict(frame_data, deprecated_keys)
        
    def transform_launch_ned_quaternion_to_state_frame(
        self,
        q_launch_ned: np.ndarray,
        frame_data: Dict[str, Any]
    ) -> np.ndarray:
        """
        Transform launch-fixed NED quaternion to state frame.

        If use_launch_fixed_ned is True, return as-is;
        otherwise transform launch NED -> local NED.

        Args:
            q_launch_ned: Quaternion in launch-fixed NED [q0, q1, q2, q3].
            frame_data: Output of get_frame_data().

        Returns:
            Quaternion in state frame.
        """
        if q_launch_ned is None:
            return None

        if self.config.use_launch_fixed_ned:
            return q_launch_ned

        C_launch_to_current = frame_data['C_ned_launch_to_current']
        return self._transform_quaternion(q_launch_ned, C_launch_to_current)

    # Backward-compatible alias
    transform_nav_quaternion_to_state_frame = transform_launch_ned_quaternion_to_state_frame
        
    def _transform_quaternion(
        self,
        q_body_to_source_ned: np.ndarray,
        C_source_to_target: np.ndarray
    ) -> np.ndarray:
        """
        Transform a quaternion between NED frames.

        Args:
            q_body_to_source_ned: Quaternion (body -> source NED).
            C_source_to_target: Rotation matrix (source NED -> target NED).

        Returns:
            Quaternion (body -> target NED).

        Raises:
            DegenerateQuaternionError: If quaternion norm is below threshold.

        Note:
            Applies sign continuity to both the intermediate frame quaternion
            and the final body quaternion. Uses rotation_matrix_to_quaternion_raw()
            so sign continuity is handled only by QuaternionStateManager.
        """
        # Architectural safety: raw conversion has no sign-continuity option,
        # so continuity is applied only through QuaternionStateManager.
        q_source_to_target = rotation_matrix_to_quaternion_raw(
            C_source_to_target, 
            QUAT_NORM_THRESHOLD
        )
        
        # Single sign-continuity path for intermediate quaternions.
        # Updates the shared reference via QuaternionStateManager.
        q_source_to_target = self.quaternion_state_manager.apply_intermediate_sign_continuity(
            q_source_to_target
        )
        
        # Use unified quaternion_multiply from quaternion_utils
        q_body_to_target = quaternion_multiply(q_source_to_target, q_body_to_source_ned)
        
        # Use unified normalize_quaternion from quaternion_utils
        # This will raise DegenerateQuaternionError for degenerate quaternions (norm < threshold)
        q_body_to_target = normalize_quaternion(q_body_to_target, QUAT_NORM_THRESHOLD)
            
        # Use instance method for sign continuity (maintains state)
        q_body_to_target = self._ensure_quaternion_sign_continuity_with_state(q_body_to_target)
        
        return q_body_to_target
        
    def _ensure_quaternion_sign_continuity_with_state(self, q: np.ndarray) -> np.ndarray:
        """
        Ensure stateful quaternion sign continuity.

        Uses the shared QuaternionStateManager to keep quaternion signs smooth
        across frame transforms. This handles sign continuity only, not recovery
        from degenerate quaternions. For stateless use, call the utility function.
        """
        # Delegate to QuaternionStateManager for unified sign continuity state.
        # تفويض إلى QuaternionStateManager لاستمرارية الإشارة الموحدة.
        return self.quaternion_state_manager.apply_sign_continuity(q)
    
    def set_sign_continuity_reference(self, quaternion: np.ndarray) -> None:
        """
        Set the reference quaternion for sign continuity.

        Use this to synchronize sign-continuity state with the simulation,
        especially after initialization or recovery from a bad quaternion.

        Args:
            quaternion: Valid normalized reference quaternion [q0, q1, q2, q3].
        """
        if quaternion is not None:
            self.quaternion_state_manager.set_sign_continuity_reference(
                np.asarray(quaternion)
            )
        else:
            logger.warning(
                "Cannot set sign continuity reference: quaternion is None. "
                "لا يمكن تعيين مرجع استمرارية الإشارة: الرباعي هو None."
            )
    
    def reset_sign_continuity_reference(self) -> None:
        """
        Reset sign-continuity state.

        Clears both the main reference and the intermediate quaternion reference.
        The next quaternion becomes the new continuity baseline.
        """
        self.quaternion_state_manager.reset_sign_continuity_reference()
        self.quaternion_state_manager.reset_intermediate_quaternion_reference()
        logger.debug(
            "Sign continuity reference and intermediate quaternion state reset to None. "
            "تم إعادة تعيين مرجع استمرارية الإشارة وحالة الرباعي الوسيط إلى None."
        )
    
    def get_sign_continuity_reference(self) -> Optional[np.ndarray]:
        """
        Get the current sign continuity reference quaternion.
                
        Returns:
            The current sign continuity reference quaternion, or None if not set.
        """
        return self.quaternion_state_manager.get_sign_continuity_reference()
        
    def get_earth_rotation_rate_ned(self) -> np.ndarray:
        """
        Get Earth rotation rate in NED frame at launch site.
                
        Returns:
            Earth rotation rate vector in NED frame (rad/s)
        """
        if not self._initialized:
            return np.zeros(3)
            
        return self.earth_model.earth_rotation_rate_ned(self.config.launch_lat_rad)
        
    def get_transport_rate_ned(self, vel_ned: np.ndarray, altitude: float) -> np.ndarray:
        """
        Get transport rate in NED frame.

        This is the NED-frame rotation caused by motion over Earth's curvature.

        Args:
            vel_ned: Velocity in NED frame (m/s).
            altitude: Current altitude (m).

        Returns:
            Transport-rate vector in NED frame (rad/s).
        """
        if not self._initialized:
            return np.zeros(3)
            
        if self.config.use_launch_fixed_ned:
            return np.zeros(3)
            
        R_plus_h = self.earth_model.equatorial_radius + altitude
        if R_plus_h < 1000.0:
            return np.zeros(3)
            
        v_n, v_e, _ = vel_ned
        tan_lat = np.tan(self.config.launch_lat_rad)
        
        return np.array([
            v_e / R_plus_h,
            -v_n / R_plus_h,
            -v_e * tan_lat / R_plus_h
        ])
        
    def get_nav_frame_rotation_rate(self, vel_ned: np.ndarray, altitude: float) -> np.ndarray:
        """
        Get total navigation frame rotation rate (Earth + transport).
                
        Args:
            vel_ned: Velocity in NED frame (m/s)
            altitude: Current altitude (m)
            
        Returns:
            Total frame rotation rate in NED frame (rad/s)
        """
        omega_ie = self.get_earth_rotation_rate_ned()
        omega_en = self.get_transport_rate_ned(vel_ned, altitude)
        return omega_ie + omega_en
        
    @property
    def frame_mode(self) -> FrameMode:
        """Get current frame mode."""
        if self.config.use_launch_fixed_ned:
            return FrameMode.LAUNCH_FIXED_NED
        return FrameMode.LOCAL_NED
        
    @property
    def is_initialized(self) -> bool:
        """Check if manager is initialized."""
        return self._initialized
    
    @property
    def is_using_shared_quaternion_state_manager(self) -> bool:
        """
        Check whether a shared QuaternionStateManager is in use.
        التحقق من استخدام QuaternionStateManager مشترك.

        Returns:
            True when shared manager is passed to __init__.
            False when an internal independent manager is used.

        Note:
            True is recommended for consistent sign continuity across subsystems.
            يُنصح بـ True لضمان استمرارية إشارة متسقة عبر الأنظمة الفرعية.
        """
        return self._using_shared_quaternion_state_manager
        
    def get_configuration_summary(self) -> Dict[str, Any]:
        """
        Get a summary of the current frame configuration.
                
        Returns:
            Dictionary with configuration summary
        """
        return {
            'long_range_mode': self.config.long_range_mode,
            'use_launch_fixed_ned': self.config.use_launch_fixed_ned,
            'include_earth_rate': self.config.include_earth_rate,
            'include_coriolis': self.config.include_coriolis,
            'include_centrifugal': self.config.include_centrifugal,
            'launch_lat_deg': np.degrees(self.config.launch_lat_rad),
            'launch_lon_deg': np.degrees(self.config.launch_lon_rad),
            'launch_alt_m': self.config.launch_alt_m,
            'frame_mode': self.frame_mode.value,
            'initialized': self._initialized,
            'using_shared_quaternion_state_manager': self._using_shared_quaternion_state_manager,
        }
        
    def compute_velocity_aligned_quaternion(
        self,
        velocity_ned: np.ndarray,
        previous_quaternion: Optional[np.ndarray] = None,
        min_airspeed_m_s: Optional[float] = None
    ) -> np.ndarray:
        """
        Compute velocity-aligned quaternion (body +X -> velocity).
        حساب رباعي محاذاة السرعة (محور +X للجسم مع متجه السرعة).

        Args:
            velocity_ned: Air-relative velocity in NED frame [vn, ve, vd] (m/s)
            previous_quaternion: Previous quaternion for sign continuity (optional)
            min_airspeed_m_s: Minimum airspeed threshold for velocity alignment (optional, default 50.0)

        Returns:
            Quaternion [q0, q1, q2, q3] (scalar first).
        
        Note:
            Sign continuity via QuaternionStateManager only
            (internal continuity is disabled to avoid double application).
            استمرارية الإشارة عبر QuaternionStateManager فقط.
        """
        # Delegate to unified quaternion_utils implementation.
        # Set apply_sign_continuity=False; continuity is handled below via QuaternionStateManager.
        q = _compute_velocity_aligned_quat(
            velocity_ned=velocity_ned,
            previous_quaternion=previous_quaternion,
            min_airspeed_m_s=min_airspeed_m_s,
            quat_norm_threshold=QUAT_NORM_THRESHOLD,
            vector_norm_threshold=VECTOR_NORM_THRESHOLD,
            apply_sign_continuity=False  # Disable internal sign continuity
        )
        
        # Apply sign continuity via QuaternionStateManager for unified state tracking
        # This ensures consistent sign continuity reference across all subsystems
        return self.quaternion_state_manager.apply_sign_continuity(q)

    def compute_launch_to_current_ned_rotation(
        self,
        C_ecef_to_ned_launch: np.ndarray,
        C_ecef_to_ned_current: np.ndarray
    ) -> np.ndarray:
        """
        Compute rotation matrix from launch NED to current local NED.

        C_launch_ned_to_current_ned = C_ecef_to_current_ned @ C_ecef_to_launch_ned.T
        """
        return C_ecef_to_ned_current @ C_ecef_to_ned_launch.T

    def compute_current_to_launch_ned_rotation(
        self,
        C_ecef_to_ned_launch: np.ndarray,
        C_ecef_to_ned_current: np.ndarray
    ) -> np.ndarray:
        """
        Compute rotation matrix from current local NED to launch NED.

        C_current_ned_to_launch_ned = C_ecef_to_launch_ned @ C_ecef_to_current_ned.T
        """
        return C_ecef_to_ned_launch @ C_ecef_to_ned_current.T

    def log_configuration(self):
        """
        Log the current frame configuration.
        """
        summary = self.get_configuration_summary()
        shared_manager_status = "YES (unified state)" if summary['using_shared_quaternion_state_manager'] else "NO (independent - may cause sign flips!)"
        logger.info(
            "==========================================================================\n"
            "Frame Consistency Manager Configuration (تكوين مدير اتساق الإطارات)\n"
            "==========================================================================\n"
            f"  long_range_mode: {summary['long_range_mode']}\n"
            f"  use_launch_fixed_ned: {summary['use_launch_fixed_ned']}\n"
            f"  include_earth_rate: {summary['include_earth_rate']}\n"
            f"  include_coriolis: {summary['include_coriolis']}\n"
            f"  include_centrifugal: {summary['include_centrifugal']}\n"
            f"  frame_mode: {summary['frame_mode']}\n"
            f"  launch_location: ({summary['launch_lat_deg']:.6f}°, {summary['launch_lon_deg']:.6f}°, {summary['launch_alt_m']:.1f}m)\n"
            f"  using_shared_quaternion_state_manager: {shared_manager_status}\n"
            "=========================================================================="
        )


def create_frame_manager_from_config(
    config: Dict[str, Any], 
    earth_model,
    long_range_mode: Optional[bool] = None,
    include_earth_rate: Optional[bool] = None,
    quaternion_state_manager: Optional[QuaternionStateManager] = None,
    require_shared_manager: bool = True,
    frame_configuration: Optional[FrameConfiguration] = None
) -> FrameConsistencyManager:
    """
    Create a FrameConsistencyManager from simulation configuration.


    Args:
        config: Simulation configuration dictionary.
        earth_model: EarthModel instance.
        long_range_mode: Optional override for long-range mode; if set, it overrides
            config['long_range']['enabled'].
        include_earth_rate: Optional override for Earth-rate inclusion; if set, it
            overrides config['long_range']['include_earth_rate'].
        quaternion_state_manager: Shared QuaternionStateManager for quaternion sign
            continuity across subsystems. Required by default.
        require_shared_manager: If True (default), raise FrameConsistencyError when
            no shared QuaternionStateManager is provided.
            QuaternionStateManager مشترك.
        frame_configuration: Optional shared FrameConfiguration passed directly to
            initialize(); has priority over individual frame parameters.
            FrameConfiguration مشترك اختياري يُمرر مباشرة إلى initialize() وله الأولوية على المعاملات الفردية.

    Returns:
        Initialized FrameConsistencyManager.
    """
    long_range_config = config.get('long_range', {})
    launch_config = config.get('launch', {})
    atm_config = config.get('atmosphere', {})
    
    # Use explicit long_range_mode if provided, otherwise read from config
    if long_range_mode is None:
        long_range_mode = long_range_config.get('enabled', False)
    use_launch_fixed_ned = long_range_config.get('use_launch_fixed_ned', True)
    include_coriolis = long_range_config.get('include_coriolis', True)
    include_centrifugal = long_range_config.get('include_centrifugal', True)
    
    if 'latitude' in launch_config:
        launch_lat_rad = np.radians(launch_config.get('latitude', 0.0))
        launch_lon_rad = np.radians(launch_config.get('longitude', 0.0))
        launch_alt_m = launch_config.get('altitude', 0.0)
    else:
        launch_lat_rad = atm_config.get('latitude_rad', 0.0)
        launch_lon_rad = atm_config.get('longitude_rad', 0.0)
        launch_alt_m = atm_config.get('launch_altitude_m', 0.0)
    
    long_range_earth_rate = long_range_config.get('include_earth_rate', None)
    # Use explicit include_earth_rate if provided, otherwise read from config
    if include_earth_rate is None:
        include_earth_rate = long_range_earth_rate if long_range_earth_rate is not None else long_range_mode
    
    manager = FrameConsistencyManager(
        earth_model,
        quaternion_state_manager=quaternion_state_manager,
        require_shared_manager=require_shared_manager
    )
    
    # Shared FrameConfiguration pattern:
    # if provided, pass it directly to initialize() so all subsystems share one config instance.
    manager.initialize(
        long_range_mode=long_range_mode,
        use_launch_fixed_ned=use_launch_fixed_ned,
        include_earth_rate=include_earth_rate,
        launch_lat_rad=launch_lat_rad,
        launch_lon_rad=launch_lon_rad,
        launch_alt_m=launch_alt_m,
        include_coriolis=include_coriolis,
        include_centrifugal=include_centrifugal,
        frame_configuration=frame_configuration
    )
    
    return manager
