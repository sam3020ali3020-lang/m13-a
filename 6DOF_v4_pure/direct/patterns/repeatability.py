"""Repeatability — N cycles of identical step.

يقيس الـ jitter في delay/overshoot/settling لـ step متطابق:
    +amp → -amp → +amp → -amp ...

Pattern:
    nصف-period 1: cmd = +amp لـ half_period_s
    نصف-period 2: cmd = -amp لـ half_period_s
    × n_cycles مرة (كامل period = 2 × half_period_s)

Output schedule لكل edge: {cycle_idx, edge_idx, t_edge, direction, ...}
يُستهلك من analysis لإنتاج histogram + stats.
"""

from __future__ import annotations

from typing import Any, Dict, List

from . import PatternSpec


def build(cfg: dict) -> PatternSpec:
    sub = cfg.get("repeatability", {})

    amp = float(sub.get("step_amplitude_deg", 5.0))
    half_period_s = float(sub.get("half_period_s", 1.0))
    n_cycles = int(sub.get("n_cycles", 50))
    pre_settle_s = float(sub.get("pre_settle_s", 0.5))

    if amp <= 0:
        raise ValueError("repeatability.step_amplitude_deg يجب > 0")
    if half_period_s < 0.2:
        raise ValueError("half_period_s يجب ≥ 0.2s (وقت كافٍ للوصول والاستقرار)")
    if n_cycles < 5:
        raise ValueError("n_cycles يجب ≥ 5 لقياس معنوي")

    period_s = 2.0 * half_period_s
    total_duration = pre_settle_s + n_cycles * period_s

    schedule: List[Dict[str, Any]] = []
    edge_idx = 0
    for cycle in range(n_cycles):
        t_pos = pre_settle_s + cycle * period_s
        t_neg = t_pos + half_period_s
        schedule.append({
            "edge_idx": edge_idx,
            "cycle_idx": cycle,
            "t_edge_s": round(t_pos, 4),
            "direction": "up",
            "cmd_from": -amp if cycle > 0 else 0.0,
            "cmd_to": +amp,
        })
        edge_idx += 1
        schedule.append({
            "edge_idx": edge_idx,
            "cycle_idx": cycle,
            "t_edge_s": round(t_neg, 4),
            "direction": "down",
            "cmd_from": +amp,
            "cmd_to": -amp,
        })
        edge_idx += 1

    def cmd_fn(t_s: float) -> float:
        if t_s < pre_settle_s:
            return 0.0
        rel = t_s - pre_settle_s
        if rel >= n_cycles * period_s:
            return -amp  # نهاية الـ pattern: استقرار على آخر قيمة
        # داخل الدورة:
        in_cycle = rel % period_s
        if in_cycle < half_period_s:
            return +amp
        return -amp

    desc = (
        f"repeatability: ±{amp:.1f}° period={period_s:.1f}s "
        f"×{n_cycles}cycles ({total_duration:.1f}s, {len(schedule)} edges)"
    )

    return PatternSpec(
        duration_s=total_duration,
        cmd_fn=cmd_fn,
        description=desc,
        schedule=schedule,
    )
