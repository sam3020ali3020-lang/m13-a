"""Hold-drift pattern.

يَفحص استقرار السيرفو عند مَواقع ثابتة لمدّة طويلة. لكلّ موقع:
1. step → cmd_target
2. settle قصير (دع dynamics تَخمد)
3. hold لِـ hold_s ثوانٍ
4. خلال الـ hold: قِس drift = (slope of fb over time)

يَكتشف:
- creep (gravitational sag)
- PID integral drift
- thermal drift خلال الاختبار
- encoder noise/quantization
- دوران غير مَطلوب (mechanical instability)
"""
from __future__ import annotations

from typing import Any, Dict, List

from . import PatternSpec


def build(cfg: dict) -> PatternSpec:
    sub = cfg.get("hold_drift", {})
    positions = sub.get("positions_deg", [-10.0, -5.0, 0.0, 5.0, 10.0])
    settle_s = float(sub.get("settle_s", 0.5))
    hold_s = float(sub.get("hold_s", 30.0))
    pre_settle_s = float(sub.get("pre_settle_s", 1.0))

    positions = [float(p) for p in positions]
    if not positions:
        raise ValueError("hold_drift.positions_deg فارغة")
    if hold_s <= 0:
        raise ValueError(f"hold_drift.hold_s يجب > 0، {hold_s}")

    # Timeline: [pre_settle | (step → settle → hold) for each pos]
    # cmd time series: t < pre_settle → 0; ثمّ بنود متتالية
    schedule: List[Dict[str, Any]] = []
    boundaries: List[tuple] = []   # (t_start, cmd)
    t_acc = pre_settle_s
    boundaries.append((0.0, 0.0))                    # initial
    for cmd in positions:
        boundaries.append((t_acc, cmd))              # step
        t_step = t_acc
        t_acc += settle_s + hold_s
        schedule.append({
            "t_step_s": t_step,
            "t_hold_start_s": t_step + settle_s,
            "t_hold_end_s": t_step + settle_s + hold_s,
            "cmd_target": cmd,
        })
    # نهاية: عودة 0
    boundaries.append((t_acc, 0.0))
    t_acc += settle_s
    total = t_acc

    def cmd_fn(t_s: float) -> float:
        cmd = 0.0
        for t_start, c in boundaries:
            if t_s >= t_start:
                cmd = c
            else:
                break
        return cmd

    desc = (f"hold_drift: {len(positions)} pos × hold={hold_s:.0f}s "
            f"({total:.1f}s total)")
    return PatternSpec(
        duration_s=total, cmd_fn=cmd_fn, description=desc,
        schedule=schedule,
    )
