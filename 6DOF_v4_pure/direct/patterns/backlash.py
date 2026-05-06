"""Backlash / hysteresis detection.

يتحرك ببطء بخطوات صغيرة من -A إلى +A ثم عودة، مع انتظار dwell بين الخطوات.
الفارق بين fb عند صعود fb عند هبوط عند نفس cmd = hysteresis (backlash).
"""

from __future__ import annotations

from . import PatternSpec


def build(cfg: dict) -> PatternSpec:
    sub = cfg.get("backlash", {})
    amp = float(sub.get("amplitude_deg", 5.0))
    step = float(sub.get("step_deg", 0.1))
    dwell = float(sub.get("dwell_s", 0.10))
    cycles = int(sub.get("cycles", 2))

    if amp <= 0 or step <= 0 or dwell <= 0 or cycles < 1:
        raise ValueError("backlash: params يجب أن تكون موجبة")

    # بناء staircase: -A → +A → -A (×cycles)
    steps = []
    n = int(round(amp / step))
    for _ in range(cycles):
        # climb up
        for k in range(-n, n + 1):
            steps.append(k * step)
        # climb down
        for k in range(n, -n - 1, -1):
            steps.append(k * step)

    total = len(steps) * dwell

    def cmd_fn(t_s: float) -> float:
        if t_s <= 0:
            return steps[0]
        if t_s >= total:
            return 0.0
        idx = int(t_s / dwell)
        if idx >= len(steps):
            return 0.0
        return steps[idx]

    desc = (f"backlash: ±{amp:.1f}° step={step:.2f}° dwell={dwell:.2f}s "
            f"×{cycles}c ({total:.1f}s)")
    return PatternSpec(duration_s=total, cmd_fn=cmd_fn, description=desc)
