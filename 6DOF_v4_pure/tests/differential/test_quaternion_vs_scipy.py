"""
Differential Tests — دوال الرباعي في `dynamics.quaternion_utils`
                     ضد SciPy `scipy.spatial.transform.Rotation`.

**الفكرة:** لدينا مرجعان يحلاّن نفس المسألة — اجعلهما يتنافسان.
أي انحراف = bug في أحدهما (وأغلب الاحتمال: في كودنا نحن).

----------------------------------------------------------------------
اتفاقيات التحويل
----------------------------------------------------------------------
كودنا       : scalar-first  [w, x, y, z]  (dynamics/quaternion_utils.py:6)
SciPy       : scalar-last   [x, y, z, w]  (Rotation.as_quat)
Euler order : 'xyz' extrinsic = Rz(yaw) · Ry(pitch) · Rx(roll)
              يُطابق تمامًا `euler_to_quaternion(roll, pitch, yaw)`
              (تم التحقق تحليليًا: q_z(y) ⊗ q_y(p) ⊗ q_x(r))
----------------------------------------------------------------------
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from hypothesis import given, settings
from hypothesis import strategies as st
from scipy.spatial.transform import Rotation as R

from strategies import (  # من tests/property/strategies.py عبر conftest
    euler_safe,
    quaternions,
    raw_quaternions,
    unit_vector3,
    vector3,
)

from dynamics.quaternion_utils import (
    euler_to_quaternion,
    normalize_quaternion,
    quaternion_conjugate,
    quaternion_multiply,
    quaternion_slerp,
    quaternion_to_euler,
    quaternion_to_rotation_matrix,
)


# ==========================================================================
# Helpers — تحويل convention
# ==========================================================================

def to_scipy(q_wxyz: np.ndarray) -> np.ndarray:
    """[w,x,y,z] → [x,y,z,w] (SciPy convention)."""
    return np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]])


def from_scipy(q_xyzw: np.ndarray) -> np.ndarray:
    """[x,y,z,w] → [w,x,y,z] (كودنا)."""
    return np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]])


def rotation_matrices_equivalent(R1: np.ndarray, R2: np.ndarray, tol: float = 1e-10) -> bool:
    """مقارنة مصفوفتَي دوران."""
    return bool(np.allclose(R1, R2, atol=tol, rtol=tol))


# ==========================================================================
# 1) quaternion_to_rotation_matrix vs Rotation.as_matrix
# ==========================================================================

@given(quaternions())
def test_rotation_matrix_matches_scipy(q):
    """R(q) من كودنا يُساوي R(q) من SciPy (ضمن دقّة float64)."""
    ours = quaternion_to_rotation_matrix(q)
    theirs = R.from_quat(to_scipy(q)).as_matrix()
    assert np.allclose(ours, theirs, atol=1e-12, rtol=1e-12), (
        f"Mismatch:\nours=\n{ours}\ntheirs=\n{theirs}"
    )


# ==========================================================================
# 2) Rotate vector: (R @ v) vs Rotation.apply(v)
# ==========================================================================

@given(quaternions(), vector3(min_value=-100.0, max_value=100.0))
def test_rotate_vector_matches_scipy(q, v):
    """تدوير متجه: كودنا (R@v) يتطابق مع SciPy `Rotation.apply`."""
    ours = quaternion_to_rotation_matrix(q) @ v
    theirs = R.from_quat(to_scipy(q)).apply(v)
    assert np.allclose(ours, theirs, atol=1e-10, rtol=1e-10)


# ==========================================================================
# 3) quaternion_multiply vs Rotation composition (r1 * r2)
# ==========================================================================

@given(quaternions(), quaternions())
def test_quaternion_multiply_matches_scipy_composition(q1, q2):
    """
    q1 ⊗ q2 (Hamilton) يُمثّل تركيب دورانَين.

    في SciPy: `r1 * r2` = الدوران الذي يُطبّق r2 ثم r1.
    في كودنا: `quaternion_multiply(q1, q2)` يُنتج دورانًا مماثلًا.

    نقارن مصفوفات الدوران (لأن q و -q نفس الدوران — scalar-first ≠ unique).
    """
    ours_q = quaternion_multiply(q1, q2)
    ours_R = quaternion_to_rotation_matrix(ours_q)

    r1 = R.from_quat(to_scipy(q1))
    r2 = R.from_quat(to_scipy(q2))
    theirs_R = (r1 * r2).as_matrix()

    assert rotation_matrices_equivalent(ours_R, theirs_R, tol=1e-10)


# ==========================================================================
# 4) quaternion_conjugate vs Rotation.inv() (للرباعي الوحدوي)
# ==========================================================================

@given(quaternions())
def test_conjugate_matches_scipy_inverse_for_unit_quaternion(q):
    """
    للرباعي الوحدوي: q* = q⁻¹.
    نقارن مصفوفات الدوران لأن SciPy قد تُعيد علامة معاكسة للرباعي.
    """
    ours_inv = quaternion_conjugate(q)
    ours_R = quaternion_to_rotation_matrix(ours_inv)

    theirs_R = R.from_quat(to_scipy(q)).inv().as_matrix()

    assert rotation_matrices_equivalent(ours_R, theirs_R, tol=1e-10)


# ==========================================================================
# 5) euler_to_quaternion vs Rotation.from_euler('xyz', extrinsic)
# ==========================================================================

@given(euler_safe(pitch_margin_deg=1.0))
def test_euler_to_quaternion_matches_scipy(rpy):
    """
    `euler_to_quaternion(r, p, y)` = `Rotation.from_euler('xyz', [r, p, y])`

    اتفاقية XYZ extrinsic: R = Rz(y) · Ry(p) · Rx(r)
    (تحقّقنا جبريًا أن هذا ما تبنيه دالتنا).
    نقارن مصفوفات الدوران لتجنّب غموض ±q.
    """
    roll, pitch, yaw = rpy
    ours_q = euler_to_quaternion(roll, pitch, yaw)
    ours_R = quaternion_to_rotation_matrix(ours_q)

    theirs_R = R.from_euler("xyz", [roll, pitch, yaw]).as_matrix()

    assert rotation_matrices_equivalent(ours_R, theirs_R, tol=1e-12)


# ==========================================================================
# 6) quaternion_to_euler vs Rotation.as_euler('xyz')
# ==========================================================================

@given(quaternions())
def test_quaternion_to_euler_matches_scipy(q):
    """
    كودنا: `quaternion_to_euler` يعيد (roll, pitch, yaw) لـ 'xyz' extrinsic.

    لتجنّب gimbal lock ambiguity (pitch ≈ ±π/2)، نقارن عبر إعادة البناء:
       Euler → rotation matrix ⇔ يجب أن تتطابق مع R(q) الأصلية.
    (المقارنة المباشرة للزوايا تفشل عند ±π = -π rollover.)
    """
    ours_rpy = quaternion_to_euler(q)  # (roll, pitch, yaw)
    ours_R_rebuilt = R.from_euler("xyz", list(ours_rpy)).as_matrix()

    q_scipy = R.from_quat(to_scipy(q))
    theirs_rpy = q_scipy.as_euler("xyz")
    theirs_R_rebuilt = R.from_euler("xyz", theirs_rpy).as_matrix()

    # كلاهما يجب أن يُعيد بناء نفس R الأصلية
    R_orig = quaternion_to_rotation_matrix(q)
    assert rotation_matrices_equivalent(ours_R_rebuilt, R_orig, tol=1e-9)
    assert rotation_matrices_equivalent(theirs_R_rebuilt, R_orig, tol=1e-9)


# ==========================================================================
# 7) SLERP vs scipy.spatial.transform.Slerp
# ==========================================================================

@given(
    quaternions(),
    quaternions(),
    st.floats(
        min_value=0.0, max_value=1.0,
        allow_nan=False, allow_infinity=False,
    ),
)
def test_slerp_matches_scipy_slerp(q1, q2, t):
    """
    كود SLERP لدينا يتطابق مع `scipy.spatial.transform.Slerp`.

    **ملاحظة خوارزمية** (اكتشفها differential test نفسه):
    كودنا في `dynamics/quaternion_utils.py:137-139` يتحوّل إلى **NLERP**
    (linear + normalize) عند `|dot| > 0.9995` لتجنّب قسمة على sin ≈ 0.
    SciPy يستخدم SLERP دقيقًا دائمًا.

    الفرق في منطقة NLERP متناهٍ في الصغر (O(1e-7)) للزوايا الصغيرة،
    لكنه ليس صفرًا. tolerance مُخفّف هنا ليقبله — مع الحفاظ على رصد
    أي انحراف أكبر (يشير إلى bug حقيقي، لا تقريب خوارزمي مقبول).
    """
    from scipy.spatial.transform import Slerp

    ours_q = quaternion_slerp(q1, q2, t)
    ours_R = quaternion_to_rotation_matrix(ours_q)

    key_rots = R.from_quat([to_scipy(q1), to_scipy(q2)])
    slerp = Slerp([0.0, 1.0], key_rots)
    theirs_R = slerp([t]).as_matrix()[0]

    # 1e-5 = يستوعب NLERP≠SLERP في الزوايا الصغيرة (O(1e-7))
    # مع رصد أي انحراف أكبر من ذلك
    assert rotation_matrices_equivalent(ours_R, theirs_R, tol=1e-5)


@given(
    quaternions(),
    quaternions(),
    st.floats(
        min_value=0.0, max_value=1.0,
        allow_nan=False, allow_infinity=False,
    ),
)
def test_slerp_far_angles_matches_scipy_strict(q1, q2, t):
    """
    للزوايا **الكبيرة** بين q1 و q2 (خارج منطقة NLERP): SLERP الدقيق.

    نشترط |dot(q1, q2)| < 0.999 (>~3.6° زاوية فاصلة) ⇒ خوارزمية SLERP
    كاملة في كلا الجانبَين ⇒ يجب أن تتطابق بدقّة float64.
    """
    from hypothesis import assume
    from scipy.spatial.transform import Slerp

    # استبعد الحالات القريبة جدًا (منطقة NLERP)
    dot = abs(float(np.dot(q1, q2)))
    assume(dot < 0.999)

    ours_q = quaternion_slerp(q1, q2, t)
    ours_R = quaternion_to_rotation_matrix(ours_q)

    key_rots = R.from_quat([to_scipy(q1), to_scipy(q2)])
    slerp = Slerp([0.0, 1.0], key_rots)
    theirs_R = slerp([t]).as_matrix()[0]

    # هنا SLERP الدقيق في كلا الجانبَين ⇒ tolerance صارم
    assert rotation_matrices_equivalent(ours_R, theirs_R, tol=1e-9)


# ==========================================================================
# 8) normalize_quaternion vs قسمة يدوية على norm
# ==========================================================================

@given(raw_quaternions())
def test_normalize_matches_manual_division(q):
    """
    `normalize_quaternion(q)` = q / ||q||.

    Sanity check — هذا أبسط differential test ممكن (المرجع: الرياضيات المباشرة).
    """
    ours = normalize_quaternion(q)
    manual = q / np.linalg.norm(q)
    assert np.allclose(ours, manual, atol=1e-14, rtol=1e-14)


# ==========================================================================
# 9) composite law — كودنا يحترم نفس algebra الذي تحترمه SciPy
#     مثال: R(q1 ⊗ q2) · v = R(q1) · (R(q2) · v) في كليهما
# ==========================================================================

@given(quaternions(), quaternions(), vector3(-10.0, 10.0))
def test_rotation_composition_same_in_both_frameworks(q1, q2, v):
    """تركيب دورانَين ثم تدوير متجه — نفس النتيجة في كودنا و SciPy."""
    # Our path: quaternion_multiply → R → apply
    ours_q = quaternion_multiply(q1, q2)
    ours_result = quaternion_to_rotation_matrix(ours_q) @ v

    # SciPy path: Rotation composition → apply
    r1 = R.from_quat(to_scipy(q1))
    r2 = R.from_quat(to_scipy(q2))
    theirs_result = (r1 * r2).apply(v)

    assert np.allclose(ours_result, theirs_result, atol=1e-10, rtol=1e-10)
