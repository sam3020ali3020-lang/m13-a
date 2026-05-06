"""Linearity pattern.

يَفحص خطّيّة cmd → fb عبر كامل النطاق (±max_amp_deg).
سلسلة step-and-hold بطيئة بـ step_deg درجات في كلّ مرحلة، dwell ثابت لكلّ
نقطة. التَّحليل يَفت linear regression cmd vs fb ويُخرج slope, intercept,
R², max_residual.

أمثلة استخدامات:
- كَشف خطأ في scale factor (delta_scale أو units_per_deg)
- كَشف non-linearity في gear/encoder
- كَشف dead-band واسع (تَجمّع نقاط حول 0)
- مُقارنة linearity بين السيرفوهات (وحدة-إلى-وحدة)
"""
from __future__ import annotations

from typing import Any, Dict, List

from . import PatternSpec


def build(cfg: dict) -> PatternSpec:
    sub = cfg.get("linearity", {})
    max_amp = float(sub.get("max_amp_deg", 18.0))     # أقلّ من angle_limit
    step_deg = float(sub.get("step_deg", 1.0))
    dwell_s = float(sub.get("dwell_s", 0.6))
    pre_settle_s = float(sub.get("pre_settle_s", 0.5))
    cycles = int(sub.get("cycles", 1))                 # ↑↓ ذهاب وعودة

    if max_amp <= 0 or step_deg <= 0 or dwell_s <= 0:
        raise ValueError("linearity: max_amp/step/dwell يجب > 0")

    # نُولّد نقاط ascending ثمّ descending
    n_steps = int(round(max_amp / step_deg))
    pts_up = [-max_amp + i * step_deg for i in range(2 * n_steps + 1)]
    # روابط nan لتَجنّب float drift
    if abs(pts_up[-1] - max_amp) > 1e-6:
        pts_up.append(max_amp)
    pts_down = list(reversed(pts_up))

    full_seq: List[float] = []
    for _ in range(cycles):
        full_seq += pts_up + pts_down

    # Schedule: list of (t_dwell_start, cmd_target) per point
    schedule: List[Dict[str, Any]] = []
    t_acc = pre_settle_s
    for cmd in full_seq:
        schedule.append({"t_dwell_start_s": t_acc, "cmd_target": cmd})
        t_acc += dwell_s
    total = t_acc

    def cmd_fn(t_s: float) -> float:
        if t_s < pre_settle_s:
            return 0.0
        idx = int((t_s - pre_settle_s) / dwell_s)
        if idx < 0 or idx >= len(full_seq):
            return full_seq[-1]
        return full_seq[idx]

    desc = (f"linearity: ±{max_amp:.1f}° step={step_deg:.2f}° "
            f"dwell={dwell_s:.2f}s ×{cycles}c "
            f"({total:.1f}s, {len(full_seq)} points)")
    return PatternSpec(
        duration_s=total, cmd_fn=cmd_fn, description=desc,
        schedule=schedule,
    )
