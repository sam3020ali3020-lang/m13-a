"""Bus-health pattern.

يَفحص صحّة CAN bus خلال نشاط طبيعيّ. الـ pattern نفسه = step بسيط
(لتوليد TX/RX حركة)، لكن الـ value يأتي من **التَّحليل** الّذي يَستخدم
الـ rx_log و bus counters لِحساب:

- TX رسائل/ثانية (المَطلوب: ≈ poll_rate × n_servos)
- RX frames/ثانية لكلّ servo
- lost-frame % (= 1 − rx_observed / rx_expected)
- inter-arrival jitter (std الـ Δt بين frames مُتتالية)
- error frames count (إن متوفّرة من الـ adapter)
- gap durations > 100ms (انقطاعات مَلحوظة)

المُخرَج: pass/fail لكلّ عتبة، + report مُفصّل.
"""
from __future__ import annotations

from typing import Any, Dict, List

from . import PatternSpec


def build(cfg: dict) -> PatternSpec:
    sub = cfg.get("bus_health", {})
    duration_s = float(sub.get("duration_s", 30.0))
    amp_deg = float(sub.get("amp_deg", 3.0))
    period_s = float(sub.get("period_s", 1.0))
    pre_settle_s = float(sub.get("pre_settle_s", 0.5))

    if duration_s <= 0 or period_s <= 0:
        raise ValueError("bus_health: duration/period يجب > 0")

    half = period_s / 2.0
    total = pre_settle_s + duration_s

    def cmd_fn(t_s: float) -> float:
        if t_s < pre_settle_s:
            return 0.0
        rel = t_s - pre_settle_s
        # square-wave ±amp لتَوليد cmd activity → سيرفو يَستجيب → RX traffic
        return amp_deg if (int(rel / half) % 2 == 0) else -amp_deg

    schedule: List[Dict[str, Any]] = [
        {"phase": "active",
         "t_start_s": pre_settle_s,
         "t_end_s": total,
         "amp_deg": amp_deg, "period_s": period_s},
    ]

    desc = (f"bus_health: ±{amp_deg:.1f}° period={period_s:.1f}s "
            f"duration={duration_s:.0f}s")
    return PatternSpec(
        duration_s=total, cmd_fn=cmd_fn, description=desc,
        schedule=schedule,
    )
