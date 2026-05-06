"""
Differential Tests — Python MPC helpers vs C++ `validation/validate` binary.

الفكرة (مطلب المستخدم):
    نفس state → نفس output من Python و C++ runtime.
    أي انحراف = bug ترجمة خفي (أخطر نوع).

**البروتوكول:**
1. Hypothesis تُولّد حالة flight واحدة.
2. Python يحسب القيم المتوقّعة باستخدام `validation/generate_test_vectors.py`.
3. نكتب JSON مؤقّت مُطابقًا لـ schema الذي يقرأه `validate`.
4. نشغّل `./validate tmp.json` ونتحقّق أنّ stdout يحوي "PASS <name>".

**لماذا max_examples قليل؟** كل مثال = subprocess. default=30 كافٍ.
استخدم `HYPOTHESIS_PROFILE=diff_cpp` لـ 30 examples مضمونة أو `ci` لمزيد.

**الـ C++ binary:** يجب أن يكون مبنيًّا قبل التشغيل:
    cd 6DOF_v4_pure/validation && ./run_validation.sh
    (أو: g++ -std=c++17 -O2 -o validate validate_cpp.cpp -lm)
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import pytest
from hypothesis import assume, given, settings
from hypothesis import strategies as st

# --------------------------------------------------------------------------
# إعداد استيراد generate_test_vectors.py
# --------------------------------------------------------------------------
_PKG_ROOT = Path(__file__).resolve().parents[2]   # 6DOF_v4_pure/
_VAL_DIR = _PKG_ROOT / "validation"
_VALIDATE_BIN = _VAL_DIR / "validate"

if str(_VAL_DIR) not in sys.path:
    sys.path.insert(0, str(_VAL_DIR))

import generate_test_vectors as gtv  # noqa: E402


# --------------------------------------------------------------------------
# Config المرجعي (يُستعمل لكل اختبار)
# --------------------------------------------------------------------------
CFG = dict(
    burn_time=4.8,
    mass_full=39.5,
    mass_dry=27.0,
    thrust_plateau=1131.0,
    t_tail=1.0,
    target_x=3000.0,
    target_h=-1200.0,
    impact_angle_deg=-30.0,
    impact_blend_start=0.93,
    impact_blend_end=0.995,
    cruise_progress=0.65,
    gamma_natural_rad=0.23,
    H_SCALE=100.0,
)


# --------------------------------------------------------------------------
# Skip conditions
# --------------------------------------------------------------------------
pytestmark = pytest.mark.skipif(
    not _VALIDATE_BIN.exists(),
    reason=(
        f"C++ validator binary not built. "
        f"Run: cd {_VAL_DIR} && g++ -std=c++17 -O2 -o validate validate_cpp.cpp -lm"
    ),
)


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------
def run_cpp_validator(test_case: dict) -> tuple[bool, str]:
    """
    Run `validate` binary on a single-case JSON; return (pass, stdout).

    `test_case` يجب أن يحتوي مفاتيح 'name', 'inputs', 'expected'.
    """
    payload = {"config": CFG, "tests": [test_case]}
    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".json", delete=False, encoding="utf-8"
    ) as f:
        json.dump(payload, f)
        tmp_path = f.name

    try:
        result = subprocess.run(
            [str(_VALIDATE_BIN), tmp_path],
            capture_output=True,
            text=True,
            timeout=5,
        )
        stdout = result.stdout
        # يمرّ فقط إذا لم يفشل أي حقل
        # (`validate` exit code 0 = كل الحالات مرّت)
        return result.returncode == 0, stdout
    finally:
        os.unlink(tmp_path)


def python_expected_for_inputs(inp: dict) -> dict:
    """استدعاء نفس الدوال التي يستخدمها `generate_test_vectors.py` لحساب المتوقّع."""
    mass, thrust = gtv.compute_params(
        inp["t"], CFG["burn_time"], CFG["mass_full"], CFG["mass_dry"],
        CFG["thrust_plateau"], CFG["t_tail"],
    )

    gamma_ref, chi_ref, dx_safe, _, _ = gtv.compute_los(
        inp["x_pos"], inp["y_pos"], inp["altitude"],
        CFG["target_x"], CFG["target_h"],
        CFG["impact_angle_deg"], CFG["impact_blend_start"], CFG["impact_blend_end"],
        CFG["burn_time"], CFG["cruise_progress"], CFG["gamma_natural_rad"],
        inp["t"], inp["gamma_ref_prev"], inp["chi_ref_prev"], inp["dt"],
        inp["cruise_alt_set"], inp["cruise_alt_target"],
    )

    W, W_e = gtv.compute_weights(
        inp["t"], inp["gamma"], gamma_ref, inp["phi"], inp["alpha"],
        inp["q_rate"], inp["x_pos"], CFG["burn_time"], CFG["t_tail"],
        CFG["target_x"], CFG["cruise_progress"],
        inp["cruise_alt_set"], inp["cruise_alt_target"], CFG["H_SCALE"],
    )

    fins = gtv.compute_fin_mixing(inp["de"], inp["dr"], inp["da"])

    return {
        "mass": mass, "thrust": thrust,
        "gamma_ref": gamma_ref, "chi_ref": chi_ref, "dx_safe": dx_safe,
        "W": W, "W_e": W_e,
        "fins": fins,
    }


# --------------------------------------------------------------------------
# Strategies لحالات flight معقولة
# --------------------------------------------------------------------------
@st.composite
def flight_state(draw):
    """توليد حالة flight عشوائية ضمن نطاق واقعي."""
    return {
        "t": draw(st.floats(min_value=0.1, max_value=20.0, allow_nan=False)),
        "x_pos": draw(st.floats(min_value=0.0, max_value=2900.0, allow_nan=False)),
        "y_pos": draw(st.floats(min_value=-50.0, max_value=50.0, allow_nan=False)),
        "altitude": draw(st.floats(min_value=10.0, max_value=1000.0, allow_nan=False)),
        "gamma": draw(st.floats(min_value=-0.5, max_value=0.5, allow_nan=False)),
        "phi": draw(st.floats(min_value=-0.5, max_value=0.5, allow_nan=False)),
        "alpha": draw(st.floats(min_value=-0.1, max_value=0.1, allow_nan=False)),
        "q_rate": draw(st.floats(min_value=-5.0, max_value=5.0, allow_nan=False)),
        "gamma_ref_prev": draw(st.floats(min_value=-0.5, max_value=0.5, allow_nan=False)),
        "chi_ref_prev": draw(st.floats(min_value=-0.3, max_value=0.3, allow_nan=False)),
        "dt": 0.02,
        "cruise_alt_set": draw(st.booleans()),
        "cruise_alt_target": draw(st.floats(min_value=10.0, max_value=1200.0, allow_nan=False)),
        "de": draw(st.floats(min_value=-0.2, max_value=0.2, allow_nan=False)),
        "dr": draw(st.floats(min_value=-0.2, max_value=0.2, allow_nan=False)),
        "da": draw(st.floats(min_value=-0.2, max_value=0.2, allow_nan=False)),
    }


def build_test_case(inp: dict, name: str = "hyp", expected: dict | None = None) -> dict:
    """تكوين JSON entry لـ validator."""
    if expected is None:
        expected = python_expected_for_inputs(inp)
    return {"name": name, "inputs": inp, "expected": expected}


# ==========================================================================
# 1) compute_params — mass + thrust (أبسط differential test)
# ==========================================================================

@given(flight_state())
@settings(max_examples=30, deadline=None)
def test_compute_params_python_matches_cpp(inp):
    """
    `compute_params(t, ...)` في Python vs C++.

    تُحسب mass و thrust بدلالة الزمن فقط. اختبار بسيط لكنه يضمن عدم
    انزلاق أي تحويل وحدة (seconds، kg، N) بين التنفيذَين.
    """
    tc = build_test_case(inp, "params_diff")
    passed, stdout = run_cpp_validator(tc)

    # نبحث عن فشل في mass أو thrust فقط (نتجاهل فشل weights)
    param_fail = any(
        f"params_diff.{field}" in stdout and "FAIL" in line
        for line in stdout.split("\n")
        for field in ("mass", "thrust")
    )
    assert not param_fail, f"Python/C++ disagree on params:\n{stdout}"


# ==========================================================================
# 2) compute_los — guidance geometry
# ==========================================================================

@given(flight_state())
@settings(max_examples=30, deadline=None)
def test_compute_los_python_matches_cpp(inp):
    """
    `compute_los(...)` — gamma_ref, chi_ref, dx_safe.

    تحقّق أن هندسة LOS (حساب الزوايا، rate limiting، impact blend)
    تنفّذ بنفس الطريقة في Python و C++.
    """
    tc = build_test_case(inp, "los_diff")
    passed, stdout = run_cpp_validator(tc)

    los_fail = any(
        "FAIL" in line and f"los_diff.{field}" in line
        for line in stdout.split("\n")
        for field in ("gamma_ref", "chi_ref", "dx_safe")
    )
    assert not los_fail, f"Python/C++ disagree on LOS:\n{stdout}"


# ==========================================================================
# 3) compute_fin_mixing — linear combination (أدق اختبار)
# ==========================================================================

@given(flight_state())
@settings(max_examples=30, deadline=None)
def test_compute_fin_mixing_python_matches_cpp(inp):
    """
    `compute_fin_mixing(de, dr, da)` — خطي بسيط (4 فنs).

    هذا يجب أن يتطابق بدقّة ~1e-6 (tolerance C++).
    أي فشل هنا = bug خطير في bitwise conversion.
    """
    tc = build_test_case(inp, "fins_diff")
    passed, stdout = run_cpp_validator(tc)

    fin_fail = any(
        "FAIL" in line and "fins_diff.fin[" in line
        for line in stdout.split("\n")
    )
    assert not fin_fail, f"Python/C++ disagree on fin mixing:\n{stdout}"


# ==========================================================================
# 4) compute_weights — weight scheduling (12 x W, 9 x W_e)
# ==========================================================================

@given(flight_state())
@settings(max_examples=30, deadline=None)
def test_compute_weights_python_matches_cpp(inp):
    """
    `compute_weights(...)` — 12 x W, 9 x W_e يجب أن تتطابق بين Python و C++.

    ملاحظة تاريخية: كان هذا الاختبار موسومًا بـ xfail بسبب فرق 288 units
    في W[4] أثناء boost. السبب: `validate` binary قديم مبنيّ من نسخة
    سابقة من المصدر. بعد إعادة البناء من `validate_cpp.cpp` الحالي
    ⇒ 9/9 تمرّ. `run_validation.sh` يُعيد البناء تلقائيًا.
    """
    tc = build_test_case(inp, "weights_diff")
    passed, stdout = run_cpp_validator(tc)

    w_fail = any(
        "FAIL" in line and ("weights_diff.W[" in line or "weights_diff.W_e[" in line)
        for line in stdout.split("\n")
    )
    assert not w_fail, f"Python/C++ disagree on weights:\n{stdout}"


# ==========================================================================
# 5) End-to-end — كل الحقول معًا (strict batch test)
# ==========================================================================

def test_all_fixed_flight_points_still_valid():
    """
    Sanity: حالات الطيران الثابتة في `generate_test_vectors.py` لا تزال تُنتج
    نفس النتائج في Python و C++ (باستثناء الـ bug المُوثَّق في `compute_weights`).

    هذا ليس property-based لكنه يرفض أي تراجع في regression.
    """
    # Regenerate current test vectors
    data = gtv.generate_test_cases()
    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".json", delete=False, encoding="utf-8"
    ) as f:
        json.dump(data, f)
        tmp_path = f.name

    try:
        result = subprocess.run(
            [str(_VALIDATE_BIN), tmp_path],
            capture_output=True, text=True, timeout=10,
        )
    finally:
        os.unlink(tmp_path)

    # استخرج عدد الـ PASS/FAIL من آخر سطر "Results: X PASS, Y FAIL"
    pass_count = 0
    fail_count = 0
    for line in result.stdout.split("\n"):
        if line.startswith("Results:"):
            # e.g. "Results: 7 PASS, 2 FAIL (max error: 2.88e+02)"
            parts = line.split(",")
            pass_count = int(parts[0].split()[1])
            fail_count = int(parts[1].split()[0])
            break

    # يجب أن تمرّ جميع الحالات التسع بعد إعادة بناء validate من المصدر الحالي
    assert pass_count == 9 and fail_count == 0, (
        f"Expected 9 PASS, 0 FAIL. Got {pass_count} PASS, {fail_count} FAIL.\n"
        f"If FAIL appears: rebuild validate via `cd validation && "
        f"g++ -std=c++17 -O2 -o validate validate_cpp.cpp -lm`\n{result.stdout}"
    )
