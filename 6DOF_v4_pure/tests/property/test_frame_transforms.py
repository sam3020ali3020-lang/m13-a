"""
Property-based tests للـ frame transformations.

نختبر **القوانين** وليس عيّنات محددة:
1) `phone_to_frd`: isometry, involution, linearity, dot/cross preservation, det=+1
2) NED ↔ FUR: round-trip, orthogonality, norm preservation

----------------------------------------------------------------------
phone_to_frd — مرجع مباشر من C++:
    AndroidApp/app/src/main/cpp/native_sensor_reader.cpp:34-41
    Android: X=right, Y=forward, Z=up
    FRD:     X=forward, Y=right, Z=down
    => (x, y, z)  →  (y, x, -z)

نسخة Python هنا مطابقة bit-for-bit للسطور الأربعة في C++
ويجب أن تبقى كذلك (راجع `native_sensor_reader.cpp` إذا تغيّر هناك).
----------------------------------------------------------------------
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from .strategies import finite_floats, vector3

from dynamics.frame_manager import (
    C_FUR_TO_NED,
    C_NED_TO_FUR,
    transform_ned_to_fur,
)


# =========================================================================
# Python mirror of the C++ phone_to_frd (native_sensor_reader.cpp:37-41)
# =========================================================================
def phone_to_frd(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Mirror of static inline void phone_to_frd in native_sensor_reader.cpp."""
    return (y, x, -z)


# Matrix form — يُستخدم في خصائص الجبر الخطي
PHONE_TO_FRD_MATRIX = np.array(
    [
        [0.0, 1.0, 0.0],   # nx = y
        [1.0, 0.0, 0.0],   # ny = x
        [0.0, 0.0, -1.0],  # nz = -z
    ],
    dtype=np.float64,
)


# ==========================================================================
# 1) phone_to_frd — خصائص هندسية
# ==========================================================================

@given(finite_floats(), finite_floats(), finite_floats())
def test_phone_to_frd_preserves_norm(x, y, z):
    """Isometry: التحويل لا يُغيّر norm² (قانون فيزيائي — المستشعر يقرأ نفس القيمة)."""
    nx, ny, nz = phone_to_frd(x, y, z)
    original_sq = x * x + y * y + z * z
    transformed_sq = nx * nx + ny * ny + nz * nz
    # relative tolerance: norm² قد يكون كبيرًا (حتى 3×10⁴)
    assert math.isclose(original_sq, transformed_sq, rel_tol=1e-12, abs_tol=1e-12)


@given(finite_floats(), finite_floats(), finite_floats())
def test_phone_to_frd_is_involution(x, y, z):
    """
    تطبيق التحويل مرتين يُعيد المدخل الأصلي:
        (x, y, z) → (y, x, -z) → (x, y, z)
    """
    nx, ny, nz = phone_to_frd(x, y, z)
    rx, ry, rz = phone_to_frd(nx, ny, nz)
    assert rx == x and ry == y and rz == z


@given(vector3(), vector3(), finite_floats(-10, 10), finite_floats(-10, 10))
def test_phone_to_frd_is_linear(u, v, a, b):
    """Linearity: T(a·u + b·v) = a·T(u) + b·T(v)."""
    combined = a * u + b * v
    lhs = np.array(phone_to_frd(*combined))

    tu = np.array(phone_to_frd(*u))
    tv = np.array(phone_to_frd(*v))
    rhs = a * tu + b * tv

    assert np.allclose(lhs, rhs, rtol=1e-10, atol=1e-10)


@given(vector3(), vector3())
def test_phone_to_frd_preserves_dot_product(u, v):
    """Isometry ⇒ المحافظة على الضرب القياسي (الزوايا محفوظة)."""
    tu = np.array(phone_to_frd(*u))
    tv = np.array(phone_to_frd(*v))
    assert math.isclose(
        float(np.dot(u, v)),
        float(np.dot(tu, tv)),
        rel_tol=1e-10,
        abs_tol=1e-10,
    )


@given(vector3(-10, 10), vector3(-10, 10))
def test_phone_to_frd_is_proper_rotation_on_cross(u, v):
    """
    det = +1 ⇒ proper rotation ⇒ T(u × v) = T(u) × T(v).
    (لو كان det = -1 (انعكاس) لكان لدينا علامة ناقص.)
    """
    cross_before = np.cross(u, v)
    lhs = np.array(phone_to_frd(*cross_before))

    tu = np.array(phone_to_frd(*u))
    tv = np.array(phone_to_frd(*v))
    rhs = np.cross(tu, tv)

    assert np.allclose(lhs, rhs, rtol=1e-10, atol=1e-10)


def test_phone_to_frd_matrix_is_proper_rotation():
    """
    Sanity static: مصفوفة التحويل orthogonal و det = +1.

    ليست property-based (لا توجد مدخلات متغيّرة) لكنها تُحكم القيد:
    `phone_to_frd` يجب أن يبقى دورانًا صحيحًا حتى لو عدّله أحد في C++.
    """
    M = PHONE_TO_FRD_MATRIX
    # Orthogonality: M · Mᵀ = I
    assert np.allclose(M @ M.T, np.eye(3), atol=1e-12)
    # Proper rotation: det = +1
    assert math.isclose(float(np.linalg.det(M)), 1.0, abs_tol=1e-12)


@given(vector3())
def test_phone_to_frd_scalar_equals_matrix(v):
    """الدالة السكلارية == ضرب المصفوفة. يربط نسخة C++ بالجبر الخطي الرمزي."""
    scalar_result = np.array(phone_to_frd(*v))
    matrix_result = PHONE_TO_FRD_MATRIX @ v
    assert np.allclose(scalar_result, matrix_result, rtol=1e-12, atol=1e-12)


@given(vector3(-1e5, 1e5))
def test_phone_to_frd_zero_vector_maps_to_zero(v):
    """linear map ⇒ T(0) = 0 (تحقُّق ضمنيًّا عبر linearity — هنا صريح)."""
    zero = np.zeros(3)
    assert np.allclose(phone_to_frd(*zero), zero, atol=0.0)


# ==========================================================================
# 2) NED ↔ FUR (dynamics/frame_manager.py:51-58)
# ==========================================================================

@given(vector3())
def test_ned_to_fur_roundtrip(v):
    """T⁻¹(T(v)) = v — تمامًا (بدون خطأ عائم بسبب ±1 فقط)."""
    fur = transform_ned_to_fur(v)
    back = C_FUR_TO_NED @ fur
    assert np.allclose(back, v, rtol=1e-14, atol=1e-14)


@given(vector3())
def test_ned_to_fur_preserves_norm(v):
    """Isometry: FUR دوران ⇒ ||T(v)|| = ||v||."""
    fur = transform_ned_to_fur(v)
    assert math.isclose(
        float(np.linalg.norm(v)),
        float(np.linalg.norm(fur)),
        rel_tol=1e-12,
        abs_tol=1e-12,
    )


def test_ned_to_fur_matrix_is_proper_rotation():
    """C_NED_TO_FUR ∈ SO(3): orthogonal + det = +1."""
    M = C_NED_TO_FUR
    assert np.allclose(M @ M.T, np.eye(3), atol=1e-14)
    assert math.isclose(float(np.linalg.det(M)), 1.0, abs_tol=1e-14)


@given(vector3(), vector3())
def test_ned_to_fur_preserves_dot_product(u, v):
    """الزوايا والضرب القياسي محفوظان عبر التحويل."""
    tu = transform_ned_to_fur(u)
    tv = transform_ned_to_fur(v)
    assert math.isclose(float(np.dot(u, v)), float(np.dot(tu, tv)),
                        rel_tol=1e-12, abs_tol=1e-12)


# ==========================================================================
# 3) تفاعل بين phone_to_frd و NED↔FUR
#    كلاهما دوران ⇒ تركيبهما دوران ⇒ المحافظة على norm و dot
# ==========================================================================

@given(vector3(-50, 50))
def test_phone_to_frd_then_ned_to_fur_is_still_isometry(v):
    """
    حالة عملية: قراءة الهاتف → FRD (body) → (بافتراض ned=body) → FUR.
    يجب أن تبقى isometry عند تركيب أي دورانَين.
    """
    frd = np.array(phone_to_frd(*v))
    fur = transform_ned_to_fur(frd)
    assert math.isclose(
        float(np.linalg.norm(v)),
        float(np.linalg.norm(fur)),
        rel_tol=1e-12,
        abs_tol=1e-12,
    )
