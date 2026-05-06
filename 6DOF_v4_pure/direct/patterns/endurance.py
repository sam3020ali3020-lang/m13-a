"""Endurance / soak pattern.

يُشَغّل square-wave نَشِط لمدّة طويلة لكَشف degradation تَدريجيّ:
- thermal drift (المُحَرّك يَسخن → دقّة encoder تَنخفض؟ τ يَتغيّر؟)
- gear wear / lubrication breakdown
- memory leaks في driver/firmware
- bus-error accumulation
- backlash يَزداد بعد آلاف cycles

التَّحليل (لكلّ window 60s):
- average τ، delay، slew_max
- متوسّط backlash (إن قابلت ramps)
- fb_rate, error count
- seek/look درجَات إحصائيّة عبر الـ windows لكَشف drift

⚠️ هذا اختبار طويل (30 دقيقة افتراضياً). تَأكّد من:
- التبريد كافٍ
- لا تُرَكّبه على flight hardware
- إيقاف ctrl+C يُعيد إلى 0° آمن
"""
from __future__ import annotations

from typing import Any, Dict, List

from . import PatternSpec


def build(cfg: dict) -> PatternSpec:
    sub = cfg.get("endurance", {})
    duration_min = float(sub.get("duration_min", 30.0))
    amp_deg = float(sub.get("amp_deg", 8.0))
    period_s = float(sub.get("period_s", 1.0))
    pre_settle_s = float(sub.get("pre_settle_s", 0.5))
    window_s = float(sub.get("analysis_window_s", 60.0))

    if duration_min <= 0 or period_s <= 0 or amp_deg <= 0:
        raise ValueError("endurance: duration/period/amp يجب > 0")

    duration_s = duration_min * 60.0
    half = period_s / 2.0
    total = pre_settle_s + duration_s

    # Schedule: نَجزّئ المدّة إلى نَوافذ لتَحليل drift
    schedule: List[Dict[str, Any]] = []
    n_windows = max(1, int(duration_s / window_s))
    for w in range(n_windows):
        schedule.append({
            "window_idx": w,
            "t_window_start_s": pre_settle_s + w * window_s,
            "t_window_end_s": pre_settle_s + (w + 1) * window_s,
            "amp_deg": amp_deg, "period_s": period_s,
        })

    def cmd_fn(t_s: float) -> float:
        if t_s < pre_settle_s:
            return 0.0
        rel = t_s - pre_settle_s
        return amp_deg if (int(rel / half) % 2 == 0) else -amp_deg

    desc = (f"endurance: ±{amp_deg:.1f}° period={period_s:.1f}s "
            f"duration={duration_min:.0f}min ({total/60:.1f}min total)")
    return PatternSpec(
        duration_s=total, cmd_fn=cmd_fn, description=desc,
        schedule=schedule,
    )
