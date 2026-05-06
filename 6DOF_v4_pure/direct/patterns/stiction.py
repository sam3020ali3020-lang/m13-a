"""Stiction (static friction) pattern.

يَختبر سلوك breakaway: عندما يَنتقل المُحَرّك من سَكون إلى حركة، يَجب
تَجاوُز static friction. نَختبر بـ slow-ramp بطيء جدّاً: 0° → 1° عبر 5s
ثمّ نُحلّل:
- متى بَدأ fb بالتَّحرّك؟
- هل ثَمّة "jump" أوّل (breakaway impulse)؟
- ما الفجوة بين cmd و fb في بداية الحركة؟

نُكَرّر بـ slopes مُختلفة + اتّجاهات + مَواقع.

يَكشف:
- breakaway torque
- gear stiction
- bearing wear
- PID dead-zone
"""
from __future__ import annotations

from typing import Any, Dict, List

from . import PatternSpec


def build(cfg: dict) -> PatternSpec:
    sub = cfg.get("stiction", {})
    ramp_amp_deg = float(sub.get("ramp_amp_deg", 1.0))
    ramp_dur_s = float(sub.get("ramp_dur_s", 5.0))   # 0.2°/s — بطيء جدّاً
    rest_s = float(sub.get("rest_s", 2.0))            # سَكون قبل كلّ ramp
    n_cycles = int(sub.get("n_cycles", 3))
    pre_settle_s = float(sub.get("pre_settle_s", 1.0))

    if ramp_amp_deg <= 0 or ramp_dur_s <= 0:
        raise ValueError("stiction: ramp_amp/dur يجب > 0")

    # Sequence: لكلّ cycle:  rest → ramp 0→+amp → rest → ramp +amp→0 → ...
    schedule: List[Dict[str, Any]] = []
    segments: List[tuple] = []   # (t_start, t_end, c_start, c_end)
    t_acc = pre_settle_s
    rate = ramp_amp_deg / ramp_dur_s
    for _ in range(n_cycles):
        # rest at 0
        segments.append((t_acc, t_acc + rest_s, 0.0, 0.0))
        t_acc += rest_s
        # ramp 0 → +amp
        schedule.append({
            "t_rest_start_s": t_acc - rest_s, "t_ramp_start_s": t_acc,
            "t_ramp_end_s": t_acc + ramp_dur_s,
            "cmd_start": 0.0, "cmd_end": +ramp_amp_deg,
            "rate_dps": +rate, "direction": "up_from_zero",
        })
        segments.append((t_acc, t_acc + ramp_dur_s, 0.0, +ramp_amp_deg))
        t_acc += ramp_dur_s
        # rest at +amp
        segments.append((t_acc, t_acc + rest_s, +ramp_amp_deg, +ramp_amp_deg))
        t_acc += rest_s
        # ramp +amp → 0
        schedule.append({
            "t_rest_start_s": t_acc - rest_s, "t_ramp_start_s": t_acc,
            "t_ramp_end_s": t_acc + ramp_dur_s,
            "cmd_start": +ramp_amp_deg, "cmd_end": 0.0,
            "rate_dps": -rate, "direction": "down_to_zero",
        })
        segments.append((t_acc, t_acc + ramp_dur_s, +ramp_amp_deg, 0.0))
        t_acc += ramp_dur_s
    total = t_acc

    def cmd_fn(t_s: float) -> float:
        for t_start, t_end, c_start, c_end in segments:
            if t_start <= t_s < t_end:
                if t_end > t_start:
                    frac = (t_s - t_start) / (t_end - t_start)
                else:
                    frac = 0.0
                return c_start + frac * (c_end - c_start)
        return 0.0

    desc = (f"stiction: ±{ramp_amp_deg:.2f}° rate={rate:.3f}°/s "
            f"×{n_cycles}c ({total:.1f}s)")
    return PatternSpec(
        duration_s=total, cmd_fn=cmd_fn, description=desc,
        schedule=schedule,
    )
