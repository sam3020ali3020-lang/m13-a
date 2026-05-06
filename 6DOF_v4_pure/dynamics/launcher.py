#!/usr/bin/env python3
"""
Launch Rail Model for 6-DOF Rocket Simulation

Constrains the rocket to 1-DOF motion along the rail axis until CG exits
the effective rail length, including thrust/drag/gravity/friction projection
and hold-down behavior.
"""

import numpy as np
import logging
import warnings
from typing import Dict, Tuple, Optional
from dataclasses import dataclass

from dynamics.quaternion_utils import (
    DegenerateQuaternionError,
    QUAT_NORM_THRESHOLD as _QUAT_NORM_THRESHOLD,
    quaternion_to_rotation_matrix as _quaternion_to_rotation_matrix_utils,
)

# Import FrameConfiguration for type hints
# استيراد FrameConfiguration للتلميحات النوعية
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from dynamics.frame_manager import FrameConfiguration

logger = logging.getLogger(__name__)


@dataclass
class LauncherState:
    """
    State of the launch rail model.
    """
    on_rail: bool                    # Whether rocket is still on rail
    rail_distance: float             # Distance traveled along rail (m)
    rail_exit_time: Optional[float]  # Time when rocket left rail (s)
    rail_axis_ned: np.ndarray        # Rail axis unit vector in NED frame
    velocity_along_rail: float       # Velocity component along rail (m/s)


class LaunchRailModel:
    """
    Launch rail constraint model for rocket simulation.

    Constrains motion to 1-DOF along the rail axis until CG exits the
    effective rail length, with locked attitude, zero angular rate,
    friction, and hold-down behavior during rail phase.
    """
    
    def __init__(self, config: Dict,
                 frame_configuration: Optional['FrameConfiguration'] = None):
        """
        Initialize launch rail model from configuration.

        Args:
            config: Configuration dictionary with 'launcher' section.
            frame_configuration: Optional shared FrameConfiguration instance.
        """
        # Store shared FrameConfiguration for consistency with other subsystems
        self._frame_configuration = frame_configuration
        
        # WARNING: Without shared FrameConfiguration, subsystems may diverge.
        # Use one shared FrameConfiguration instance in production.
        if frame_configuration is None:
            warnings.warn(
                "LaunchRailModel created without shared FrameConfiguration. "
                "Frame parameters will be read from config dict, which may cause "
                "configuration divergence with other subsystems. "
                "For production use, pass a shared FrameConfiguration instance. "
                "(تحذير: تم إنشاء LaunchRailModel بدون FrameConfiguration مشترك. "
                "سيتم قراءة معاملات الإطار من قاموس التكوين، مما قد يسبب تباعد التكوين "
                "مع الأنظمة الفرعية الأخرى. للاستخدام الإنتاجي، مرر مثيل FrameConfiguration مشترك.)",
                UserWarning,
                stacklevel=2
            )
        
        launcher_config = config.get('launcher', {})
        
        self.enabled = launcher_config.get('enabled', False)
        self.rail_length = launcher_config.get('rail_length', 9.5)
        self.rail_effective_length = launcher_config.get('rail_effective_length', 4.85)
        self.rail_friction = launcher_config.get('rail_friction_coefficient', 0.1)
        self.launch_elevation_deg = launcher_config.get('launch_elevation', 85.0)
        self.launch_elevation_rad = np.radians(self.launch_elevation_deg)
        
        # Rail axis will be computed from initial quaternion
        self._rail_axis_ned: Optional[np.ndarray] = None
        self._rail_axis_ecef: Optional[np.ndarray] = None
        
        # State tracking (only updated on accepted integration steps)
        self._on_rail = True
        self._rail_distance = 0.0
        self._rail_exit_time: Optional[float] = None
        
        # Gravity constant
        self.g = 9.80665
        
        if self.enabled:
            logger.info(
                f"Launch rail model enabled: "
                f"length={self.rail_length}m, effective_length={self.rail_effective_length}m, "
                f"friction={self.rail_friction}, elevation={self.launch_elevation_deg}°"
            )
            logger.info(
                f"نموذج قضيب الإطلاق مفعّل: "
                f"الطول={self.rail_length}م، الطول الفعال={self.rail_effective_length}م، "
                f"الاحتكاك={self.rail_friction}، زاوية الارتفاع={self.launch_elevation_deg}°"
            )
            if frame_configuration is not None:
                logger.info(
                    f"Launch rail using shared FrameConfiguration "
                    f"(long_range_mode={frame_configuration.long_range_mode}) "
                    f"(قضيب الإطلاق يستخدم FrameConfiguration مشترك)"
                )
        
        # Validate configuration
        if self.enabled and self.rail_effective_length <= 0:
            logger.warning(
                f"rail_effective_length={self.rail_effective_length} <= 0, "
                f"rail constraint will be immediately released"
            )
    
    def initialize_rail_axis(self, initial_quaternion: np.ndarray,
                             C_ned_to_ecef: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Initialize rail axis from the initial attitude quaternion.

        Args:
            initial_quaternion: Initial attitude quaternion [q0, q1, q2, q3].
            C_ned_to_ecef: Optional NED-to-ECEF rotation matrix for long_range_mode.

        Returns:
            Rail axis unit vector in NED frame.
        """
        R = self._quaternion_to_rotation_matrix(initial_quaternion)
        
        # Body X-axis in NED frame (first column of R, since R transforms body→NED)
        self._rail_axis_ned = R[:, 0].copy()
        
        # Normalize to ensure unit vector
        norm = np.linalg.norm(self._rail_axis_ned)
        if norm > 1e-10:
            self._rail_axis_ned /= norm
        
        # For long_range_mode, also store rail axis in ECEF
        if C_ned_to_ecef is not None:
            self._rail_axis_ecef = C_ned_to_ecef @ self._rail_axis_ned
            norm_ecef = np.linalg.norm(self._rail_axis_ecef)
            if norm_ecef > 1e-10:
                self._rail_axis_ecef /= norm_ecef
        
        logger.debug(
            f"Rail axis initialized: NED={self._rail_axis_ned}, "
            f"elevation from vertical={np.degrees(np.arccos(-self._rail_axis_ned[2])):.1f}°"
        )
        
        return self._rail_axis_ned
    
    def get_rail_axis_ned(self, C_ecef_to_ned: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Get rail axis unit vector in NED frame.

        Args:
            C_ecef_to_ned: Optional ECEF-to-NED rotation matrix for long_range_mode.

        Returns:
            Rail axis unit vector in NED frame.
        """
        if self._rail_axis_ned is None:
            raise RuntimeError(
                "Rail axis not initialized. Call initialize_rail_axis(initial_quaternion) "
                "before using the launcher. This should be done automatically by "
                "Rocket6DOFSimulation when launcher.enabled=True. "
                "محور القضيب غير مُهيأ. استدعِ initialize_rail_axis(initial_quaternion) "
                "قبل استخدام المنصة. يجب أن يتم هذا تلقائياً بواسطة "
                "Rocket6DOFSimulation عندما launcher.enabled=True."
            )
        
        if C_ecef_to_ned is not None and self._rail_axis_ecef is not None:
            # Long-range mode: convert ECEF rail axis to current local NED
            rail_axis = C_ecef_to_ned @ self._rail_axis_ecef
            norm = np.linalg.norm(rail_axis)
            if norm > 1e-10:
                rail_axis /= norm
            return rail_axis
        
        return self._rail_axis_ned.copy()
    
    def compute_rail_distance(self, position_ned: np.ndarray,
                              rail_axis: np.ndarray) -> float:
        """
        Compute the distance traveled along the rail axis.
                
        Args:
            position_ned: Current position in NED frame [x, y, z] (m)
            rail_axis: Rail axis unit vector in NED frame
            
        Returns:
            Distance along rail axis (m)
        """
        return float(np.dot(position_ned, rail_axis))
    
    def is_on_rail(self, position_ned: np.ndarray,
                   rail_axis: np.ndarray) -> bool:
        """
        Check whether the rocket is still on the launch rail.

        Args:
            position_ned: Current position in NED frame [x, y, z] (m).
            rail_axis: Rail axis unit vector in NED frame.

        Returns:
            True if rocket is still on rail, otherwise False.
        """
        if not self.enabled:
            return False
        
        if self.rail_effective_length <= 0:
            return False
        
        distance = self.compute_rail_distance(position_ned, rail_axis)
        return distance < self.rail_effective_length
    
    def compute_constrained_dynamics(
        self,
        position_ned: np.ndarray,
        velocity_ned: np.ndarray,
        quaternion: np.ndarray,
        mass: float,
        thrust_body: np.ndarray,
        aero_forces_body: np.ndarray,
        R: np.ndarray,
        rail_axis: np.ndarray,
        gravity_ned: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, Dict]:
        """
        Compute constrained 1-DOF dynamics state derivatives for the launch rail phase.
        
        Args:
            position_ned: Position in NED frame [x, y, z] (m).
            velocity_ned: Velocity in NED frame [vx, vy, vz] (m/s).
            quaternion: Attitude quaternion [q0, q1, q2, q3].
            mass: Current mass (kg).
            thrust_body: Thrust vector in body frame [Fx, Fy, Fz] (N).
            aero_forces_body: Aerodynamic forces in body frame [Fx, Fy, Fz] (N).
            R: Rotation matrix body->NED.
            rail_axis: Rail axis unit vector in NED frame.
            gravity_ned: Gravity vector in NED frame [gx, gy, gz] (m/s^2).
            
        Returns:
            Tuple of (pos_dot, vel_dot, quat_dot, angular_accel, info_dict).
        """
        # Current velocity along rail
        v_along = float(np.dot(velocity_ned, rail_axis))
        
        # Transform forces to NED frame
        thrust_ned = R @ thrust_body
        aero_ned = R @ aero_forces_body
        
        # Project forces onto rail axis
        F_thrust_along = np.dot(thrust_ned, rail_axis)
        F_aero_along = np.dot(aero_ned, rail_axis)
        
        # Gravity force along rail (weight component)
        # gravity_ned is acceleration, so multiply by mass for force
        F_gravity_along = mass * np.dot(gravity_ned, rail_axis)
        
        # Friction force
        # Normal force magnitude: perpendicular component of weight
        # N = m * g * sqrt(1 - (down · rail_axis)^2)
        down_ned = np.array([0.0, 0.0, 1.0])  # Down direction in NED
        cos_angle = np.dot(down_ned, rail_axis)
        sin_angle_sq = 1.0 - cos_angle * cos_angle
        sin_angle = np.sqrt(max(0.0, sin_angle_sq))
        
        normal_force = mass * self.g * sin_angle
        
        # Friction opposes motion
        if abs(v_along) > 1e-6:
            F_friction = -self.rail_friction * normal_force * np.sign(v_along)
        else:
            # Static friction: oppose net force if stationary
            F_friction = 0.0
        
        # Net force along rail
        F_net_along = F_thrust_along + F_aero_along + F_gravity_along + F_friction
        
        # Acceleration along rail
        if mass > 0:
            accel_along = F_net_along / mass
        else:
            accel_along = 0.0
        
        # Hold-down: prevent backward rail motion.
        # If velocity <= 0 and net acceleration is backward, clamp both to zero.
        if v_along <= 0 and accel_along < 0:
            v_along = 0.0
            accel_along = 0.0
        
        # Constrained derivatives
        pos_dot = v_along * rail_axis
        vel_dot = accel_along * rail_axis
        quat_dot = np.zeros(4)  # Locked attitude
        
        angular_accel = np.zeros(3)  # No rotation on rail
        # Info for logging/debugging
        info = {
            'velocity_along_rail': v_along,
            'accel_along_rail': accel_along,
            'F_thrust_along': F_thrust_along,
            'F_aero_along': F_aero_along,
            'F_gravity_along': F_gravity_along,
            'F_friction': F_friction,
            'F_net_along': F_net_along,
            'normal_force': normal_force,
            'rail_distance': self.compute_rail_distance(position_ned, rail_axis),
        }
        
        return pos_dot, vel_dot, quat_dot, angular_accel, info
    
    def update_state(self, t: float, position_ned: np.ndarray,
                     rail_axis: np.ndarray) -> LauncherState:
        """
        Update launcher state on accepted integration steps only.

        Call only when update_state=True in _compute_dynamics to avoid
        integrator-dependent side effects.
        
        Args:
            t: Current time (s).
            position_ned: Position in NED frame [x, y, z] (m).
            rail_axis: Rail axis unit vector in NED frame.
            
        Returns:
            Current launcher state.
        """
        distance = self.compute_rail_distance(position_ned, rail_axis)
        was_on_rail = self._on_rail
        
        # Check for rail exit
        if self._on_rail and distance >= self.rail_effective_length:
            self._on_rail = False
            self._rail_exit_time = t
            logger.info(
                f"Rail exit at t={t:.4f}s, distance={distance:.3f}m "
                f"(effective_length={self.rail_effective_length}m)"
            )
            logger.info(
                f"خروج من القضيب عند t={t:.4f}ث، المسافة={distance:.3f}م "
                f"(الطول الفعال={self.rail_effective_length}م)"
            )
        
        self._rail_distance = distance
        
        # Compute velocity along rail for state
        v_along = 0.0  # Will be updated by caller if needed
        
        return LauncherState(
            on_rail=self._on_rail,
            rail_distance=self._rail_distance,
            rail_exit_time=self._rail_exit_time,
            rail_axis_ned=rail_axis.copy(),
            velocity_along_rail=v_along
        )
    
    def get_state(self) -> LauncherState:
        """
        Get current launcher state without updating.
                
        Returns:
            Current launcher state
        """
        return LauncherState(
            on_rail=self._on_rail,
            rail_distance=self._rail_distance,
            rail_exit_time=self._rail_exit_time,
            rail_axis_ned=self._rail_axis_ned.copy() if self._rail_axis_ned is not None else np.zeros(3),
            velocity_along_rail=0.0
        )
    
    def reset(self):
        """
        Reset launcher state for a new simulation.
        """
        self._on_rail = True
        self._rail_distance = 0.0
        self._rail_exit_time = None
        logger.debug("Launcher state reset")
    
    @property
    def on_rail(self) -> bool:
        """Whether rocket is currently on rail."""
        return self._on_rail
    
    @property
    def rail_exit_time(self) -> Optional[float]:
        """Time when rocket exited rail (None if still on rail)."""
        return self._rail_exit_time
    
    @property
    def is_initialized(self) -> bool:
        """
        Check if rail axis has been initialized.
                
        Returns:
            True if initialize_rail_axis() has been called successfully
        """
        return self._rail_axis_ned is not None
    
    @property
    def long_range_mode(self) -> bool:
        """
        Get long_range_mode from shared FrameConfiguration if available.
                
        Returns:
            True if long_range_mode is enabled, False otherwise
        """
        if self._frame_configuration is not None:
            return self._frame_configuration.long_range_mode
        return False
    
    @property
    def has_shared_frame_configuration(self) -> bool:
        """
        Check if launcher is using shared FrameConfiguration.
                
        Returns:
            True if shared FrameConfiguration is available
        """
        return self._frame_configuration is not None
    
    def _quaternion_to_rotation_matrix(self, q: np.ndarray) -> np.ndarray:
        """
        Convert quaternion to a body-to-NED rotation matrix.

        Delegates to quaternion_utils.quaternion_to_rotation_matrix to keep
        behavior consistent across the codebase.
        
        Args:
            q: Quaternion [q0, q1, q2, q3] (scalar-first convention).
            
        Returns:
            3x3 rotation matrix R such that v_ned = R @ v_body.
            
        Raises:
            DegenerateQuaternionError: If quaternion norm is below threshold or contains NaN/Inf.
        """
        return _quaternion_to_rotation_matrix_utils(q)


