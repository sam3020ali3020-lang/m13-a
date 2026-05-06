"""Triangular ramp at fixed rate.

يقيس slew-rate الأقصى للسيرفو ونقطة saturation.

Period:  0 → +A → 0 → -A → 0  بمعدل rate_deg_per_s
Cycles:  تكرار دوري
"""

from __future__ import annotations

from . import PatternSpec


def build(cfg: dict) -> PatternSpec:
    sub = cfg.get("ramp", {})
    rate = float(sub.get("rate_deg_per_s", 30.0))
    amp = float(sub.get("amplitude_deg", 8.0))
    cycles = int(sub.get("cycles", 3))

    if rate <= 0 or amp <= 0 or cycles < 1:
        raise ValueError("ramp: rate/amp/cycles يجب > 0")

    leg_time = amp / rate                 # 0 → A time
    period = 4.0 * leg_time               # 0 → A → 0 → -A → 0
    total = cycles * period

    def cmd_fn(t_s: float) -> float:
        if t_s <= 0 or t_s >= total:
            return 0.0
        # حدّد المكان داخل الدورة
        tm = t_s % period
        if tm < leg_time:
            return rate * tm
        elif tm < 2 * leg_time:
            return amp - rate * (tm - leg_time)
        elif tm < 3 * leg_time:
            return -rate * (tm - 2 * leg_time)
        else:
            return -amp + rate * (tm - 3 * leg_time)

    desc = (f"ramp: ±{amp:.1f}° @ {rate:.0f}°/s, "
            f"{cycles} cycles ({total:.1f}s)")
    return PatternSpec(duration_s=total, cmd_fn=cmd_fn, description=desc)
