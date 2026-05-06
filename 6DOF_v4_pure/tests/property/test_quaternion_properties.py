"""
Property-based tests لقوانين جبر الرباعي (Hamilton quaternions).

القوانين المُختبَرة:
- conjugate/inverse: (q*)* = q ; q · q* = |q|² ; for unit: q · q⁻¹ = 1
- multiplication: associative ; not commutative (counter-check) ; reversal law
- rotation matrix: orthogonal + det = +1 ; R(q) = R(-q) ; preserves vector norm
- slerp: endpoints ; unit norm throughout ; identity at t=0.5 when q1=q2
- Euler round-trip: quaternion → Euler → quaternion gives same rotation

Convention: scalar-first [w, x, y, z] — see dynamics/quaternion_utils.py:6
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from hypothesis import assume, given, settings
from hypothesis import strategies as st

from .strategies import (
    euler_safe,
    quaternions,
    raw_quaternions,
    rotation_angles,
    unit_vector3,
    vector3,
)

from dynamics.quaternion_utils import (
    DegenerateQuaternionError,
    QUAT_NORM_THRESHOLD,
    euler_to_quaternion,
    normalize_quaternion,
    quaternion_conjugate,
    quaternion_multiply,
    quaternion_slerp,
    quaternion_to_euler,
    quaternion_to_rotation_matrix,
)


IDENTITY_Q = np.array([1.0, 0.0, 0.0, 0.0])


def _rotation_matrices_equal(R1: np.ndarray, R2: np.ndarray, tol: float = 1e-9) -> bool:
    """مقارنة مصفوفتي دوران — الأنسب من مقارنة الرباعيات (q و -q نفس الدوران)."""
    return bool(np.allclose(R1, R2, rtol=tol, atol=tol))


# ==========================================================================
# 1) Conjugate / Inverse
# ==========================================================================

@given(raw_quaternions())
def test_conjugate_is_involution(q):
    """(q*)* = q — تطبيق الاقتران مرّتين يُعيد الأصل."""
    assert np.allclose(quaternion_conjugate(quaternion_conjugate(q)), q, atol=0.0)


@given(raw_quaternions())
def test_conjugate_preserves_norm(q):
    """|q*| = |q|."""
    assert math.isclose(
        float(np.linalg.norm(quaternion_conjugate(q))),
        float(np.linalg.norm(q)),
        rel_tol=1e-14, abs_tol=1e-14,
    )


@given(raw_quaternions())
def test_q_times_conjugate_is_real_and_equals_norm_squared(q):
    """
    q ⊗ q* = (|q|², 0, 0, 0) — الجزء التخيّلي يُلغى.

    هذا القانون هو ما يجعل `conjugate == inverse` للرباعي الوحدوي.
    """
    product = quaternion_multiply(q, quaternion_conjugate(q))
    norm_sq = float(q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2)
    assert math.isclose(product[0], norm_sq, rel_tol=1e-10, abs_tol=1e-10)
    assert np.allclose(product[1:], [0.0, 0.0, 0.0], atol=1e-10)


@given(quaternions())
def test_unit_quaternion_times_conjugate_is_identity(q):
    """للرباعي الوحدوي: q ⊗ q* = (1, 0, 0, 0) — identity rotation."""
    product = quaternion_multiply(q, quaternion_conjugate(q))
    assert np.allclose(product, IDENTITY_Q, atol=1e-10)


# ==========================================================================
# 2) Multiplication laws
# ==========================================================================

@given(quaternions(), quaternions(), quaternions())
def test_quaternion_multiplication_is_associative(q1, q2, q3):
    """(q1 ⊗ q2) ⊗ q3 = q1 ⊗ (q2 ⊗ q3)."""
    lhs = quaternion_multiply(quaternion_multiply(q1, q2), q3)
    rhs = quaternion_multiply(q1, quaternion_multiply(q2, q3))
    assert np.allclose(lhs, rhs, atol=1e-12)


@given(quaternions())
def test_identity_is_multiplicative_identity(q):
    """Identity quaternion: 1 ⊗ q = q ⊗ 1 = q."""
    assert np.allclose(quaternion_multiply(IDENTITY_Q, q), q, atol=1e-14)
    assert np.allclose(quaternion_multiply(q, IDENTITY_Q), q, atol=1e-14)


@given(quaternions(), quaternions())
def test_multiplication_preserves_unit_norm(q1, q2):
    """|q1 ⊗ q2| = |q1| · |q2| — unit × unit = unit."""
    product = quaternion_multiply(q1, q2)
    assert math.isclose(
        float(np.linalg.norm(product)), 1.0, rel_tol=1e-10, abs_tol=1e-10
    )


@given(quaternions(), quaternions())
def test_conjugate_reversal_law(q1, q2):
    """(q1 ⊗ q2)* = q2* ⊗ q1* — قانون عكس الترتيب في الاقتران."""
    lhs = quaternion_conjugate(quaternion_multiply(q1, q2))
    rhs = quaternion_multiply(quaternion_conjugate(q2), quaternion_conjugate(q1))
    assert np.allclose(lhs, rhs, atol=1e-12)


@given(quaternions(), quaternions())
def test_quaternion_multiplication_cross_product_law(q1, q2):
    """
    قانون Hamilton الأساسي: (q1⊗q2) - (q2⊗q1) = (0, 2·(v1 × v2)).

    أقوى من counter-check عدم التبادل: هو قانون **موجب** يشرح
    *بالضبط* مقدار اللاتبادلية ويختبر صيغة الضرب المُستخدمة.
    """
    p12 = quaternion_multiply(q1, q2)
    p21 = quaternion_multiply(q2, q1)

    # الجزء الحقيقي دائمًا متساوٍ: w = w1·w2 - v1·v2 (symmetric in 1↔2)
    assert math.isclose(p12[0], p21[0], abs_tol=1e-12)

    # الجزء التخيّلي يختلف بقدر 2·(v1 × v2) بالضبط
    v1 = q1[1:]
    v2 = q2[1:]
    expected_diff = 2.0 * np.cross(v1, v2)
    actual_diff = p12[1:] - p21[1:]
    assert np.allclose(actual_diff, expected_diff, atol=1e-12)


# ==========================================================================
# 3) Rotation matrix properties
# ==========================================================================

@given(quaternions())
def test_rotation_matrix_is_orthogonal(q):
    """R(q) · R(q)ᵀ = I — خصيصة أساسية لكل دوران."""
    R = quaternion_to_rotation_matrix(q)
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-10)


@given(quaternions())
def test_rotation_matrix_has_determinant_one(q):
    """det(R) = +1 — proper rotation (بدون انعكاس)."""
    R = quaternion_to_rotation_matrix(q)
    assert math.isclose(float(np.linalg.det(R)), 1.0, abs_tol=1e-10)


@given(quaternions())
def test_q_and_negq_give_same_rotation_matrix(q):
    """
    q و -q يُمثّلان نفس الدوران (double cover of SO(3)).

    هذا القانون حيوي: سلوك sign continuity في الكود يعتمد عليه.
    """
    R_pos = quaternion_to_rotation_matrix(q)
    R_neg = quaternion_to_rotation_matrix(-q)
    assert _rotation_matrices_equal(R_pos, R_neg, tol=1e-12)


@given(quaternions(), vector3(-100, 100))
def test_rotation_preserves_vector_norm(q, v):
    """||R(q) · v|| = ||v|| — قانون isometry للدوران."""
    R = quaternion_to_rotation_matrix(q)
    rotated = R @ v
    assert math.isclose(
        float(np.linalg.norm(rotated)),
        float(np.linalg.norm(v)),
        rel_tol=1e-9, abs_tol=1e-9,
    )


@given(quaternions(), vector3(-50, 50), vector3(-50, 50))
def test_rotation_preserves_dot_product(q, u, v):
    """(R·u) · (R·v) = u · v — الزوايا بين المتجهات محفوظة."""
    R = quaternion_to_rotation_matrix(q)
    ru = R @ u
    rv = R @ v
    assert math.isclose(
        float(np.dot(u, v)),
        float(np.dot(ru, rv)),
        rel_tol=1e-9, abs_tol=1e-9,
    )


@given(quaternions(), quaternions(), vector3(-10, 10))
def test_rotation_composition_matches_quaternion_multiply(q1, q2, v):
    """
    R(q1 ⊗ q2) · v = R(q1) · (R(q2) · v)
    تركيب الدورانات عبر الرباعي = تركيبها عبر المصفوفات.
    """
    R12 = quaternion_to_rotation_matrix(quaternion_multiply(q1, q2))
    R1 = quaternion_to_rotation_matrix(q1)
    R2 = quaternion_to_rotation_matrix(q2)

    lhs = R12 @ v
    rhs = R1 @ (R2 @ v)
    assert np.allclose(lhs, rhs, rtol=1e-9, atol=1e-9)


@given(quaternions())
def test_rotation_of_inverse_is_matrix_transpose(q):
    """R(q*) = R(q)ᵀ — اقتران الرباعي ⇔ transpose للمصفوفة."""
    R = quaternion_to_rotation_matrix(q)
    R_conj = quaternion_to_rotation_matrix(quaternion_conjugate(q))
    assert np.allclose(R_conj, R.T, atol=1e-10)


# ==========================================================================
# 4) Normalization
# ==========================================================================

@given(raw_quaternions())
def test_normalize_produces_unit_quaternion(q):
    """|normalize(q)| = 1 (ضمن دقّة العائم)."""
    qn = normalize_quaternion(q)
    assert math.isclose(float(np.linalg.norm(qn)), 1.0, rel_tol=1e-14, abs_tol=1e-14)


@given(raw_quaternions())
def test_normalize_idempotent(q):
    """normalize(normalize(q)) = normalize(q) — التطبيع idempotent."""
    q1 = normalize_quaternion(q)
    q2 = normalize_quaternion(q1)
    assert np.allclose(q1, q2, atol=1e-14)


@given(raw_quaternions(), st.floats(min_value=0.01, max_value=100.0, allow_nan=False))
def test_normalize_is_scale_invariant(q, scale):
    """normalize(α·q) = ±normalize(q) للعامل α > 0 — علامة ثابتة."""
    n1 = normalize_quaternion(q)
    n2 = normalize_quaternion(scale * q)
    assert np.allclose(n1, n2, atol=1e-12)


def test_normalize_rejects_degenerate_quaternion():
    """quaternion قريب جدًا من الصفر ⇒ يرفع DegenerateQuaternionError."""
    tiny = np.array([1e-20, 0.0, 0.0, 0.0])
    with pytest.raises(DegenerateQuaternionError):
        normalize_quaternion(tiny)


# ==========================================================================
# 5) SLERP
# ==========================================================================

@given(quaternions(), quaternions())
def test_slerp_at_zero_returns_first_quaternion_rotation(q1, q2):
    """slerp(q1, q2, 0) يُمثّل نفس دوران q1 (قد يختلف إشارة بسبب shortest-path)."""
    result = quaternion_slerp(q1, q2, 0.0)
    R_result = quaternion_to_rotation_matrix(result)
    R_q1 = quaternion_to_rotation_matrix(q1)
    assert _rotation_matrices_equal(R_result, R_q1, tol=1e-9)


@given(quaternions(), quaternions())
def test_slerp_at_one_returns_second_quaternion_rotation(q1, q2):
    """slerp(q1, q2, 1) يُمثّل نفس دوران q2."""
    result = quaternion_slerp(q1, q2, 1.0)
    R_result = quaternion_to_rotation_matrix(result)
    R_q2 = quaternion_to_rotation_matrix(q2)
    assert _rotation_matrices_equal(R_result, R_q2, tol=1e-9)


@given(
    quaternions(),
    quaternions(),
    st.floats(min_value=0.0, max_value=1.0, allow_nan=False, allow_infinity=False),
)
def test_slerp_produces_unit_quaternion(q1, q2, t):
    """slerp يُنتج unit quaternion لأي t ∈ [0, 1]."""
    result = quaternion_slerp(q1, q2, t)
    assert math.isclose(
        float(np.linalg.norm(result)), 1.0, rel_tol=1e-9, abs_tol=1e-9
    )


@given(
    quaternions(),
    st.floats(min_value=0.0, max_value=1.0, allow_nan=False, allow_infinity=False),
)
def test_slerp_fixed_point(q, t):
    """slerp(q, q, t) = ±q — لا حركة بين نقطتَين متطابقتَين."""
    result = quaternion_slerp(q, q, t)
    R_result = quaternion_to_rotation_matrix(result)
    R_q = quaternion_to_rotation_matrix(q)
    assert _rotation_matrices_equal(R_result, R_q, tol=1e-9)


# ==========================================================================
# 6) Euler ↔ Quaternion round-trip
# ==========================================================================

@given(euler_safe(pitch_margin_deg=2.0))
def test_euler_quaternion_roundtrip_preserves_rotation(rpy):
    """
    Euler → quaternion → Euler → quaternion : نفس الدوران.

    نقارن مصفوفات الدوران (وليس الرباعيات مباشرة) لتجنّب ambiguity
    في التمثيل (q ≡ -q ، وأيضًا تمثيلات Euler متعددة لنفس الدوران).
    """
    roll, pitch, yaw = rpy
    q1 = euler_to_quaternion(roll, pitch, yaw)
    r2, p2, y2 = quaternion_to_euler(q1)
    q2 = euler_to_quaternion(r2, p2, y2)

    R1 = quaternion_to_rotation_matrix(q1)
    R2 = quaternion_to_rotation_matrix(q2)
    assert _rotation_matrices_equal(R1, R2, tol=1e-8)


@given(euler_safe())
def test_euler_to_quaternion_produces_unit_quaternion(rpy):
    """بما أن cos²+sin² = 1 فإن euler→quat يُنتج unit quaternion بالبناء."""
    roll, pitch, yaw = rpy
    q = euler_to_quaternion(roll, pitch, yaw)
    assert math.isclose(
        float(np.linalg.norm(q)), 1.0, rel_tol=1e-14, abs_tol=1e-14
    )


# ==========================================================================
# 7) Edge-case knowledge tests — توثيق سلوك الحواف (regression guards)
# ==========================================================================

def test_nan_quaternion_raises_on_rotation_matrix():
    """quaternion يحتوي NaN ⇒ يرفع DegenerateQuaternionError."""
    q_nan = np.array([1.0, float("nan"), 0.0, 0.0])
    with pytest.raises(DegenerateQuaternionError):
        quaternion_to_rotation_matrix(q_nan)


def test_inf_quaternion_raises_on_rotation_matrix():
    """quaternion يحتوي Inf ⇒ يرفع DegenerateQuaternionError."""
    q_inf = np.array([float("inf"), 0.0, 0.0, 0.0])
    with pytest.raises(DegenerateQuaternionError):
        quaternion_to_rotation_matrix(q_inf)


def test_zero_quaternion_raises_on_rotation_matrix():
    """quaternion صفري ⇒ يرفع DegenerateQuaternionError."""
    q_zero = np.zeros(4)
    with pytest.raises(DegenerateQuaternionError):
        quaternion_to_rotation_matrix(q_zero)
