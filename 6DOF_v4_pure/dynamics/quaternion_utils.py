#!/usr/bin/env python3
"""
Quaternion Utilities - وحدة أدوات الرباعي

Pure math quaternion functions (no simulation state dependency).
Convention: scalar-first [q0, q1, q2, q3] = [w, x, y, z].

Sign Continuity: q and -q represent the same rotation. Apply sign continuity
via QuaternionStateManager externally (recommended) or via
ensure_quaternion_sign_continuity() / apply_sign_continuity parameter.
"""

import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation
from typing import Tuple, Optional, Union, List
import logging

logger = logging.getLogger(__name__)

# ==========================================================================
# Numerical Thresholds - الثوابت العددية
# ==========================================================================
# These thresholds are the SINGLE SOURCE OF TRUTH for the entire codebase.
# All modules (rocket_6dof_sim.py, frame_manager.py, etc.) should import
# these constants from here to ensure consistency.
# ==========================================================================

# Threshold for quaternion normalization checks
DEFAULT_QUAT_NORM_THRESHOLD = 1e-10
QUAT_NORM_THRESHOLD = 1e-10  # Alias for compatibility / اسم بديل للتوافق

# Threshold for vector normalization (degenerate cases)
VECTOR_NORM_THRESHOLD = 1e-9

# Threshold for cross product checks (detecting near-parallel vectors)
# This is less strict than QUAT_NORM_THRESHOLD because cross products
# can be small even for non-degenerate cases when vectors are nearly parallel.
CROSS_PRODUCT_THRESHOLD = 1e-6


# ==========================================================================
# Exception Classes - فئات الاستثناءات
# ==========================================================================

class DegenerateQuaternionError(RuntimeError):
    """Degenerate quaternion (near-zero norm) detected - invalid attitude."""
    pass


def quaternion_to_euler(q: Union[np.ndarray, List[float]]) -> Tuple[float, float, float]:
    """
    Convert quaternion to Euler angles (roll, pitch, yaw).

    Args:
        q: Quaternion [q0, q1, q2, q3] (scalar first)

    Returns:
        Tuple (roll, pitch, yaw) in radians
    """
    q0, q1, q2, q3 = q

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (q0 * q1 + q2 * q3)
    cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (q0 * q2 - q3 * q1)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (q0 * q3 + q1 * q2)
    cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Convert Euler angles (rad) to quaternion [q0,q1,q2,q3] (scalar first)."""
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    q0 = cr * cp * cy + sr * sp * sy
    q1 = sr * cp * cy - cr * sp * sy
    q2 = cr * sp * cy + sr * cp * sy
    q3 = cr * cp * sy - sr * sp * cy

    return np.array([q0, q1, q2, q3])


def degrees_to_quaternion(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Convert Euler angles (degrees) to quaternion [q0,q1,q2,q3] (scalar first)."""
    roll_rad = np.radians(roll_deg)
    pitch_rad = np.radians(pitch_deg)
    yaw_rad = np.radians(yaw_deg)
    
    return euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)


def quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Multiply two quaternions q1 ⊗ q2 (scalar first)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])


def quaternion_slerp(
    q1: Union[np.ndarray, List[float]], 
    q2: Union[np.ndarray, List[float]], 
    t: float
) -> np.ndarray:
    """SLERP between two quaternions. t in [0,1]."""
    q1 = np.asarray(q1)
    q2 = np.asarray(q2)

    # Ensure shortest path (q and -q represent same rotation)
    dot = np.dot(q1, q2)
    if dot < 0:
        q2 = -q2
        dot = -dot

    # If quaternions are very close, use linear interpolation
    if dot > 0.9995:
        result = q1 + t * (q2 - q1)
        return result / np.linalg.norm(result)

    # SLERP formula
    theta_0 = np.arccos(np.clip(dot, -1.0, 1.0))
    theta = theta_0 * t

    sin_theta = np.sin(theta)
    sin_theta_0 = np.sin(theta_0)

    s1 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s2 = sin_theta / sin_theta_0

    result = s1 * q1 + s2 * q2
    return result / np.linalg.norm(result)


def ensure_quaternion_sign_continuity(
    q_new: np.ndarray, 
    q_previous: Optional[np.ndarray]
) -> np.ndarray:
    """Flip q_new sign if dot(q_new, q_previous) < 0 to avoid sign discontinuities."""
    if q_previous is not None and np.dot(q_new, q_previous) < 0:
        return -q_new
    return q_new


def rotation_matrix_to_quaternion_raw(
    R: np.ndarray,
    quat_norm_threshold: float = DEFAULT_QUAT_NORM_THRESHOLD
) -> np.ndarray:
    """Convert 3x3 rotation matrix to quaternion (scalar first). No sign continuity applied."""
    if R is None or not np.all(np.isfinite(R)):
        logger.warning(
            "Non-finite rotation matrix detected in rotation_matrix_to_quaternion_raw. "
            "Returning identity quaternion. "
            "تم اكتشاف مصفوفة دوران غير منتهية. إرجاع رباعي الهوية."
        )
        return np.array([1.0, 0.0, 0.0, 0.0])

    try:
        q_xyzw = ScipyRotation.from_matrix(R).as_quat()
        q = np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]])
    except Exception as exc:
        logger.warning(
            "SciPy rotation conversion failed in rotation_matrix_to_quaternion_raw: %s. "
            "Falling back to identity quaternion. "
            "فشل تحويل SciPy: %s. الرجوع إلى رباعي الهوية.",
            str(exc), str(exc)
        )
        return np.array([1.0, 0.0, 0.0, 0.0])

    q_norm = np.linalg.norm(q)
    if q_norm > quat_norm_threshold:
        q = q / q_norm
    
    return q


def rotation_matrix_to_quaternion(
    R: np.ndarray,
    quat_norm_threshold: float = DEFAULT_QUAT_NORM_THRESHOLD
) -> np.ndarray:
    """Alias for rotation_matrix_to_quaternion_raw(). Apply sign continuity externally."""
    return rotation_matrix_to_quaternion_raw(R, quat_norm_threshold)


def normalize_quaternion(
    q: np.ndarray,
    quat_norm_threshold: float = DEFAULT_QUAT_NORM_THRESHOLD
) -> np.ndarray:
    """Normalize quaternion to unit length. Raises DegenerateQuaternionError if norm < threshold."""
    q = np.asarray(q)
    q_norm = np.linalg.norm(q)
    
    if q_norm < quat_norm_threshold:
        raise DegenerateQuaternionError(
            f"Quaternion norm {q_norm} is below threshold {quat_norm_threshold}. "
            f"Cannot normalize degenerate quaternion. "
            f"معيار الرباعي {q_norm} تحت العتبة {quat_norm_threshold}. "
            f"لا يمكن تطبيع رباعي متدهور."
        )
    
    return q / q_norm


def quaternion_conjugate(q: np.ndarray) -> np.ndarray:
    """Return quaternion conjugate [q0, -q1, -q2, -q3] (= inverse for unit quaternions)."""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """Convert quaternion [q0,q1,q2,q3] to 3x3 rotation matrix (body→NED). Single source of truth.
    Raises DegenerateQuaternionError on NaN/Inf or near-zero norm."""
    q = np.asarray(q)
    
    # Check for NaN or Inf values - these indicate serious numerical issues
    if not np.all(np.isfinite(q)):
        raise DegenerateQuaternionError(
            f"Quaternion contains NaN or Inf values: {q}. "
            f"This indicates numerical instability in the simulation. "
            f"الرباعي يحتوي على قيم NaN أو Inf: {q}. "
            f"هذا يشير إلى عدم استقرار عددي في المحاكاة."
        )
    
    # Check for degenerate quaternion (near-zero norm)
    q_norm = np.linalg.norm(q)
    if q_norm < QUAT_NORM_THRESHOLD:
        raise DegenerateQuaternionError(
            f"Quaternion norm {q_norm} is below threshold {QUAT_NORM_THRESHOLD}. "
            f"Cannot convert degenerate quaternion to rotation matrix. "
            f"معيار الرباعي {q_norm} تحت العتبة {QUAT_NORM_THRESHOLD}. "
            f"لا يمكن تحويل رباعي متدهور إلى مصفوفة دوران."
        )
    
    # Normalize quaternion for numerical stability
    q0, q1, q2, q3 = q / q_norm
    
    # First row
    r00 = 1 - 2*(q2*q2 + q3*q3)
    r01 = 2*(q1*q2 - q0*q3)
    r02 = 2*(q1*q3 + q0*q2)
    
    # Second row
    r10 = 2*(q1*q2 + q0*q3)
    r11 = 1 - 2*(q1*q1 + q3*q3)
    r12 = 2*(q2*q3 - q0*q1)
    
    # Third row
    r20 = 2*(q1*q3 - q0*q2)
    r21 = 2*(q2*q3 + q0*q1)
    r22 = 1 - 2*(q1*q1 + q2*q2)
    
    return np.array([
        [r00, r01, r02],
        [r10, r11, r12],
        [r20, r21, r22]
    ])


def compute_velocity_aligned_quaternion(
    velocity_ned: np.ndarray,
    previous_quaternion: Optional[np.ndarray] = None,
    min_airspeed_m_s: Optional[float] = None,
    quat_norm_threshold: float = QUAT_NORM_THRESHOLD,
    vector_norm_threshold: float = VECTOR_NORM_THRESHOLD,
    apply_sign_continuity: bool = True
) -> np.ndarray:
    """Compute quaternion aligning body +X with velocity vector (velocity_aligned mode).
    Uses scipy Rotation for stable matrix→quaternion conversion.
    Set apply_sign_continuity=False when using QuaternionStateManager externally.
    Returns previous_quaternion or identity on invalid input."""
    # Early validation: check for NaN/Inf in input velocity
    if not np.all(np.isfinite(velocity_ned)):
        logger.warning(
            "Non-finite velocity_ned detected in compute_velocity_aligned_quaternion: %s. "
            "Returning previous/identity quaternion. "
            "تم اكتشاف velocity_ned غير منتهية: %s. إرجاع الرباعي السابق/الهوية.",
            velocity_ned, velocity_ned
        )
        if previous_quaternion is not None:
            return previous_quaternion.copy()
        return np.array([1.0, 0.0, 0.0, 0.0])

    V = np.linalg.norm(velocity_ned)

    # Use configurable threshold (default 50 m/s) to ensure rocket maintains launch attitude
    # during initial powered ascent when velocity direction is not yet stable.
    threshold = min_airspeed_m_s if min_airspeed_m_s is not None else 50.0
    if V < threshold:
        if previous_quaternion is not None:
            return previous_quaternion.copy()
        return np.array([1.0, 0.0, 0.0, 0.0])

    x_hat = velocity_ned / V

    # Build orthonormal basis with x_hat as first axis
    d_hat = np.array([0.0, 0.0, 1.0])  # Down direction

    cross_dx = np.cross(d_hat, x_hat)
    cross_norm = np.linalg.norm(cross_dx)

    # Handle degenerate cases where velocity is nearly vertical
    if cross_norm < CROSS_PRODUCT_THRESHOLD:
        n_hat = np.array([1.0, 0.0, 0.0])  # North direction
        cross_dx = np.cross(n_hat, x_hat)
        cross_norm = np.linalg.norm(cross_dx)

        if cross_norm < CROSS_PRODUCT_THRESHOLD:
            e_hat = np.array([0.0, 1.0, 0.0])  # East direction
            cross_dx = np.cross(e_hat, x_hat)
            cross_norm = np.linalg.norm(cross_dx)

    # If still degenerate, return previous quaternion or identity
    if cross_norm < vector_norm_threshold:
        if previous_quaternion is not None:
            return previous_quaternion.copy()
        return np.array([1.0, 0.0, 0.0, 0.0])

    y_hat = cross_dx / cross_norm
    z_hat = np.cross(x_hat, y_hat)

    # Gram-Schmidt re-orthonormalization to ensure numerical stability
    # This corrects any accumulated floating-point errors in the basis vectors
    y_hat = y_hat - x_hat * np.dot(x_hat, y_hat)
    y_norm = np.linalg.norm(y_hat)
    if y_norm > quat_norm_threshold:
        y_hat = y_hat / y_norm
    z_hat = np.cross(x_hat, y_hat)
    z_norm = np.linalg.norm(z_hat)
    if z_norm > quat_norm_threshold:
        z_hat = z_hat / z_norm

    # Build rotation matrix (body-to-NED)
    R = np.column_stack([x_hat, y_hat, z_hat])

    # Validate rotation matrix before conversion
    if not np.all(np.isfinite(R)):
        logger.warning(
            "Non-finite rotation matrix R detected in compute_velocity_aligned_quaternion. "
            "Returning previous/identity quaternion. "
            "تم اكتشاف مصفوفة دوران R غير منتهية. إرجاع الرباعي السابق/الهوية."
        )
        if previous_quaternion is not None:
            return previous_quaternion.copy()
        return np.array([1.0, 0.0, 0.0, 0.0])

    # Use SciPy's robust rotation matrix to quaternion conversion
    # This handles edge cases like trace ≈ -1 (180° rotation) properly
    try:
        q_xyzw = ScipyRotation.from_matrix(R).as_quat()
        # SciPy returns [x, y, z, w], convert to scalar-first [w, x, y, z]
        q = np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]])

        # Validate quaternion after SciPy conversion
        if not np.all(np.isfinite(q)):
            logger.warning(
                "Non-finite quaternion from SciPy conversion: %s. "
                "Returning previous/identity quaternion. "
                "رباعي غير منتهٍ من تحويل SciPy: %s. إرجاع الرباعي السابق/الهوية.",
                q, q
            )
            if previous_quaternion is not None:
                return previous_quaternion.copy()
            return np.array([1.0, 0.0, 0.0, 0.0])
    except Exception as e:
        # Fallback to identity if conversion fails (should not happen)
        logger.warning(
            "SciPy rotation conversion failed: %s. Returning previous/identity quaternion. "
            "فشل تحويل SciPy: %s. إرجاع الرباعي السابق/الهوية.",
            str(e), str(e)
        )
        if previous_quaternion is not None:
            return previous_quaternion.copy()
        return np.array([1.0, 0.0, 0.0, 0.0])

    # Normalize quaternion (should already be normalized, but cheap insurance)
    q_norm = np.linalg.norm(q)
    if q_norm > quat_norm_threshold:
        q = q / q_norm
    else:
        if previous_quaternion is not None:
            return previous_quaternion.copy()
        return np.array([1.0, 0.0, 0.0, 0.0])

    # Sign continuity with previous quaternion to avoid sudden flips
    # q and -q represent the same rotation, so choose the one closer to previous
    # Only apply if apply_sign_continuity=True (default) to avoid double application
    # when QuaternionStateManager is used externally for unified state tracking.
    if apply_sign_continuity and previous_quaternion is not None:
        if np.dot(q, previous_quaternion) < 0:
            q = -q

    return q


