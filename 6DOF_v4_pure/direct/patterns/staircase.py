"""Staircase pattern.

يَختَبِر سُلوك السيرفو عَبر خَطَوات تَراكُميَّة صَغيرَة:
  0° → +step → +2*step → +3*step → ... → +max
  +max → +max-step → ... → 0
  0° → -step → -2*step → ... → -max
  -max → -max+step → ... → 0

كلّ خَطوَة هي زِيادَة/نُقصان قَدرُها ``step_deg`` فَقَط مِن الزاوِيَة الحاليَّة.
هذا يَكشِف:

- **stiction discrete**: السيرفو يَتَجاهَل خَطوات صَغيرَة (≤ deadband) أَحياناً.
- **PID dead-zone**: firmware يُعيد |error| < threshold كَأنَّه "وَصَلَ".
- **gear backlash مُتَقَطِّع**: لُعبَة تَختَلِف مَع المَوضِع/الاتِّجاه.
- **encoder hysteresis**: encoder لا يُسَجِّل تَغَيُّر صَغير بشَكل مُتَّسِق.

الفَرق عَن linearity:
- linearity يَبدأ مِن ``-max`` ويَمشي إلى ``+max`` ثُمَّ عَودَة (مَدى كامِل).
- staircase يَبدأ مِن 0، يَصعَد لـ +max، يَعود لِـ 0، يَنزِل لـ -max، يَعود.
  هذا يَجعَل كلّ "حُزمَة" مُستَقِلَّة، ويَكشِف stiction المَوضِعي بدِقَّة أَكبَر.

الفَرق عَن stiction:
- stiction يَستَخدِم ramp بَطيء (0.2°/s) لقِياس breakaway lag.
- staircase يَستَخدِم step فَوري + dwell، يَكشِف stalls تَراكُميَّة.
"""
from __future__ import annotations

from typing import Any, Dict, List

from . import PatternSpec


def build(cfg: dict) -> PatternSpec:
    sub = cfg.get("staircase", {})
    max_amp_deg = float(sub.get("max_amp_deg", 20.0))
    step_deg = float(sub.get("step_deg", 1.0))
    dwell_s = float(sub.get("dwell_s", 0.8))
    pre_settle_s = float(sub.get("pre_settle_s", 1.0))
    cycles = int(sub.get("cycles", 1))
    directions = str(sub.get("directions", "both")).lower()

    if max_amp_deg <= 0 or step_deg <= 0 or dwell_s <= 0:
        raise ValueError("staircase: max_amp/step/dwell يجب > 0")
    if directions not in ("up", "down", "both"):
        raise ValueError("staircase.directions: up | down | both")

    n_steps = int(round(max_amp_deg / step_deg))

    # نَبني تَسَلسُل القِيَم (cmd targets) لِكُل دَورَة كامِلَة:
    #   0 → +max → 0 → -max → 0
    full_seq: List[float] = []
    schedule: List[Dict[str, Any]] = []

    def add_leg(seq: List[float], leg_name: str, start: float, end: float):
        """Append cumulative steps from ``start`` to ``end`` بزِيادَة step_deg."""
        if end > start:
            sign = +1.0
        elif end < start:
            sign = -1.0
        else:
            return
        n = int(round(abs(end - start) / step_deg))
        for k in range(1, n + 1):
            v = start + sign * k * step_deg
            # تَجَنُّب الانحِراف العائِم
            if (sign > 0 and v > end) or (sign < 0 and v < end):
                v = end
            seq.append(v)

    # نَبني schedule مُتَزامِن مَع التَّسَلسُل
    t_acc = pre_settle_s
    for cyc in range(cycles):
        legs = []
        if directions in ("up", "both"):
            legs.append(("up_climb",   0.0, +max_amp_deg))
            legs.append(("up_descend", +max_amp_deg, 0.0))
        if directions in ("down", "both"):
            legs.append(("down_climb",   0.0, -max_amp_deg))
            legs.append(("down_descend", -max_amp_deg, 0.0))

        for leg_name, leg_start, leg_end in legs:
            seq_before = len(full_seq)
            add_leg(full_seq, leg_name, leg_start, leg_end)
            for i in range(seq_before, len(full_seq)):
                target = full_seq[i]
                # المَوضِع السابِق
                prev = full_seq[i - 1] if i > 0 else 0.0
                schedule.append({
                    "phase": leg_name,
                    "cycle": cyc,
                    "step_idx": i - seq_before,
                    "t_step_s": t_acc,
                    "t_dwell_end_s": t_acc + dwell_s,
                    "cmd_prev": prev,
                    "cmd_target": target,
                    "delta_deg": target - prev,
                })
                t_acc += dwell_s

    total = t_acc

    def cmd_fn(t_s: float) -> float:
        if t_s < pre_settle_s:
            return 0.0
        idx = int((t_s - pre_settle_s) / dwell_s)
        if idx < 0:
            return 0.0
        if idx >= len(full_seq):
            return full_seq[-1] if full_seq else 0.0
        return full_seq[idx]

    n_pts = len(full_seq)
    desc = (f"staircase: ±{max_amp_deg:.1f}° step={step_deg:.2f}° "
            f"dwell={dwell_s:.2f}s ×{cycles}c dir={directions} "
            f"({total:.1f}s, {n_pts} steps)")
    return PatternSpec(
        duration_s=total, cmd_fn=cmd_fn, description=desc,
        schedule=schedule,
    )
