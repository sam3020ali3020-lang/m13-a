"""
Pytest configuration for property-based tests.

يضيف `6DOF_v4_pure/` إلى sys.path حتى تصبح `dynamics.quaternion_utils`
و `dynamics.frame_manager` قابلة للاستيراد بدون تثبيت الحزمة.

ويُسجّل profiles لـ Hypothesis:
- default : ~100 عيّنة/اختبار (سريع، للتطوير اليومي)
- ci      : 1000 عيّنة/اختبار، بدون deadline (للتشغيل المكثّف)
"""

import os
import sys

# tests/property/conftest.py → اصعد مستويين للوصول لجذر 6DOF_v4_pure
_HERE = os.path.abspath(os.path.dirname(__file__))
_PKG_ROOT = os.path.abspath(os.path.join(_HERE, "..", ".."))

if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)


# --------------------------------------------------------------------------
# Hypothesis profiles
# --------------------------------------------------------------------------
from hypothesis import settings, HealthCheck  # noqa: E402

settings.register_profile(
    "ci",
    max_examples=1000,
    deadline=None,
    suppress_health_check=[HealthCheck.too_slow],
)

# اختر profile عبر متغيّر البيئة HYPOTHESIS_PROFILE (افتراضي: default)
settings.load_profile(os.environ.get("HYPOTHESIS_PROFILE", "default"))
