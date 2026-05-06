"""
Hypothesis strategies مشتركة لاختبارات property-based.

تُولِّد:
- `finite_floats` : float في نطاق آمن (بدون NaN/Inf/overflow)
- `vector3`       : متجه 3D بأعداد منتهية
- `unit_vector3`  : متجه 3D بطول ≈ 1 (للاستخدام كمحور دوران)
- `quaternions`   : unit quaternions [w,x,y,z] (scalar-first)
- `raw_quaternions` : quaternions غير موحَّدة — لاختبار normalize
- `rotation_angles` : زوايا بالرادياند في (-π, π]
- `euler_safe`    : (roll, pitch, yaw) بعيدًا عن gimbal lock
"""

from __future__ import annotations

import math

import numpy as np
from hypothesis import strategies as st


# --------------------------------------------------------------------------
# Floats / vectors أساسية
# --------------------------------------------------------------------------

def finite_floats(min_value: float = -100.0, max_value: float = 100.0):
    """float منتهٍ بدون NaN/Inf في النطاق [min_value, max_value]."""
    return st.floats(
        min_value=min_value,
        max_value=max_value,
        allow_nan=False,
        allow_infinity=False,
        allow_subnormal=False,
    )


@st.composite
def vector3(draw, min_value: float = -100.0, max_value: float = 100.0):
    """متجه 3D: (x, y, z) — كلها أعداد منتهية."""
    x = draw(finite_floats(min_value, max_value))
    y = draw(finite_floats(min_value, max_value))
    z = draw(finite_floats(min_value, max_value))
    return np.array([x, y, z], dtype=np.float64)


@st.composite
def unit_vector3(draw):
    """
    متجه 3D بطول ≈ 1.

    نرفض المتجهات القريبة جدًا من الصفر لضمان استقرار عملية التطبيع.
    """
    v = draw(vector3(min_value=-1.0, max_value=1.0))
    n = float(np.linalg.norm(v))
    # norm تحت عتبة → ارفض العيّنة وحاول أخرى (Hypothesis سيولّد بدلها)
    from hypothesis import assume
    assume(n > 1e-3)
    return v / n


# --------------------------------------------------------------------------
# Quaternions (scalar-first [w, x, y, z])
# --------------------------------------------------------------------------

@st.composite
def raw_quaternions(draw, min_value: float = -10.0, max_value: float = 10.0):
    """
    Quaternion عشوائي (غير موحَّد) [w, x, y, z].

    يُستخدم لاختبار خصائص التطبيع والتعامل مع المدخلات غير الوحدوية.
    """
    w = draw(finite_floats(min_value, max_value))
    x = draw(finite_floats(min_value, max_value))
    y = draw(finite_floats(min_value, max_value))
    z = draw(finite_floats(min_value, max_value))
    q = np.array([w, x, y, z], dtype=np.float64)

    # ارفض quaternion صفري (لا يمكن تطبيعه)
    from hypothesis import assume
    assume(np.linalg.norm(q) > 1e-3)
    return q


@st.composite
def quaternions(draw):
    """
    Unit quaternion [w, x, y, z] (scalar-first) — يمثّل دورانًا صالحًا.

    البناء: (axis, angle) → quaternion. هذا يغطي جميع الدورانات بانتظام.
    """
    axis = draw(unit_vector3())
    # زاوية في [0, π] تكفي لتغطية كل الدورانات المختلفة (q و-q نفس الدوران)
    angle = draw(
        st.floats(
            min_value=0.0,
            max_value=math.pi,
            allow_nan=False,
            allow_infinity=False,
        )
    )
    half = angle * 0.5
    s = math.sin(half)
    return np.array(
        [math.cos(half), axis[0] * s, axis[1] * s, axis[2] * s],
        dtype=np.float64,
    )


# --------------------------------------------------------------------------
# Euler angles
# --------------------------------------------------------------------------

def rotation_angles(min_deg: float = -180.0, max_deg: float = 180.0):
    """زاوية دوران بالرادياند."""
    return st.floats(
        min_value=math.radians(min_deg),
        max_value=math.radians(max_deg),
        allow_nan=False,
        allow_infinity=False,
    )


@st.composite
def euler_safe(draw, pitch_margin_deg: float = 5.0):
    """
    (roll, pitch, yaw) بالرادياند، مع تجنّب gimbal lock عند |pitch| = π/2.

    pitch_margin_deg: هامش ابتعاد من ±90° لضمان round-trip مستقر.
    """
    margin = math.radians(pitch_margin_deg)
    roll = draw(rotation_angles())
    pitch = draw(
        st.floats(
            min_value=-math.pi / 2 + margin,
            max_value=math.pi / 2 - margin,
            allow_nan=False,
            allow_infinity=False,
        )
    )
    yaw = draw(rotation_angles())
    return roll, pitch, yaw
