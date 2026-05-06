"""
Pytest configuration for differential tests.

Differential testing = اختبار مرجعَين يحلاّن نفس المسألة ضد بعضهما
(Python vs SciPy, Python vs C++). Hypothesis تولّد حالات؛ والمرجعَان
يجب أن يتّفقا ضمن tolerance معقول.

يُضيف `6DOF_v4_pure/` إلى sys.path بالإضافة إلى `tests/property/`
لإعادة استخدام الـ strategies الموجودة هناك (مصدر واحد للحقيقة).

كما يُعيد بناء `validation/validate` تلقائيًا إذا كان المصدر أحدث
من الـ binary — يمنع إعادة ظهور bug الـ "stale binary" الذي أدّى
إلى 288-unit drift في W[4] في وقت سابق.
"""

import os
import subprocess
import sys

_HERE = os.path.abspath(os.path.dirname(__file__))
_PKG_ROOT = os.path.abspath(os.path.join(_HERE, "..", ".."))
_PROP_DIR = os.path.abspath(os.path.join(_HERE, "..", "property"))

for _path in (_PKG_ROOT, _PROP_DIR):
    if _path not in sys.path:
        sys.path.insert(0, _path)


# --------------------------------------------------------------------------
# أعد بناء `validate` إذا كان المصدر أحدث من binary (anti-stale-binary)
# --------------------------------------------------------------------------
def _ensure_validate_fresh() -> None:
    """Rebuild validation/validate if validate_cpp.cpp is newer."""
    val_dir = os.path.join(_PKG_ROOT, "validation")
    src = os.path.join(val_dir, "validate_cpp.cpp")
    exe = os.path.join(val_dir, "validate")

    if not os.path.isfile(src):
        return

    needs_build = (
        not os.path.isfile(exe)
        or os.path.getmtime(src) > os.path.getmtime(exe)
    )
    if not needs_build:
        return

    try:
        subprocess.run(
            ["g++", "-std=c++17", "-O2", "-o", exe, src, "-lm"],
            check=True, capture_output=True, timeout=30,
        )
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired, FileNotFoundError):
        # نتركه فاشلًا — pytest.mark.skipif سيتعامل معه
        pass


_ensure_validate_fresh()


# --------------------------------------------------------------------------
# Hypothesis profiles (نفس profile من property/)
# --------------------------------------------------------------------------
from hypothesis import HealthCheck, settings  # noqa: E402

if "ci" not in settings._profiles:
    settings.register_profile(
        "ci",
        max_examples=1000,
        deadline=None,
        suppress_health_check=[HealthCheck.too_slow],
    )

# Differential tests أبطأ (subprocess لـ C++) — profile مخصص
if "diff_cpp" not in settings._profiles:
    settings.register_profile(
        "diff_cpp",
        max_examples=30,       # قليل لأن كل مثال = تشغيل C++ binary
        deadline=None,
        suppress_health_check=[HealthCheck.too_slow, HealthCheck.data_too_large],
    )

settings.load_profile(os.environ.get("HYPOTHESIS_PROFILE", "default"))
