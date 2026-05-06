"""Firmware audit pattern.

يَختَبِر سُلوك firmware لكلّ السيرفوهات عَلى نَفس قائِمَة الأَوامِر بالضَّبط،
ويَكشِف اختِلافات داخِليَّة في firmware (حُدود مُتَفاوِتَة، saturation، ABORT).

الهَدَف: التَّمييز بَين خَلَل في البروتوكول (يُؤَثِّر عَلى الكلّ) وخَلَل في
firmware سيرفو واحِد (يَظهَر في سُلوك مُتَفاوِت بَين السيرفوهات).

نُرسِل نَفس cmd لجَميع السيرفوهات في وَقت مُتَزامِن، نَنتَظِر للاستِقرار،
ثُمَّ نُسَجِّل fb. التَّحليل يُقارِن بَين السيرفوهات ويَكشِف:
  - أَقصى fb مُحَقَّق (saturation positive/negative)
  - الفَرق بَين cmd و fb لكلّ سيرفو
  - الـ servos الَّتي تَتَّبِع cmd vs الَّتي لا تَتَّبِع
  - تَناظُر/عَدَم تَناظُر المَدى لكلّ سيرفو

التَّسَلسُل (افتِراضي):
  0 → +5 → +10 → +13 → +14 → +15 → +18 → +20 → +25 → +30 →
  0 → -5 → -10 → -13 → -14 → -15 → -18 → -20 → -25 → -30 → 0
"""
from __future__ import annotations

from typing import Any, Dict, List

from . import PatternSpec


def build(cfg: dict) -> PatternSpec:
    sub = cfg.get("firmware_audit", {})
    pos_values = sub.get(
        "pos_values_deg",
        [5.0, 10.0, 13.0, 14.0, 14.5, 15.0, 16.0, 18.0, 20.0, 25.0, 30.0],
    )
    neg_values = sub.get(
        "neg_values_deg",
        [-5.0, -10.0, -13.0, -14.0, -14.5, -15.0, -16.0, -18.0, -20.0, -25.0,
         -30.0],
    )
    dwell_s = float(sub.get("dwell_s", 1.5))
    pre_settle_s = float(sub.get("pre_settle_s", 1.0))
    return_to_zero = bool(sub.get("return_to_zero_between", True))

    pos_values = [float(v) for v in pos_values]
    neg_values = [float(v) for v in neg_values]

    # سَلسَلَة الأَوامِر:
    #   0 → pos1 → 0 → pos2 → 0 → ... → pos_N → 0 →
    #   neg1 → 0 → neg2 → 0 → ... → neg_N → 0
    # إذا return_to_zero=False:
    #   0 → pos1 → pos2 → ... → pos_N → 0 → neg1 → neg2 → ... → neg_N → 0
    cmd_seq: List[float] = []
    schedule: List[Dict[str, Any]] = []

    def append_step(cmd: float, phase: str, group_idx: int):
        cmd_seq.append(cmd)
        schedule.append({
            "phase": phase,
            "step_idx": group_idx,
            "cmd_target": cmd,
            "t_step_s": pre_settle_s + (len(cmd_seq) - 1) * dwell_s,
            "t_dwell_end_s": pre_settle_s + len(cmd_seq) * dwell_s,
        })

    # المَوجَب
    for i, v in enumerate(pos_values):
        append_step(v, "pos_test", i)
        if return_to_zero:
            append_step(0.0, "zero_between", i)

    # عَودَة لـ 0 إذا لَم تَكُن العَودَة بَعد كلّ خَطوَة
    if not return_to_zero and pos_values:
        append_step(0.0, "zero_between", len(pos_values))

    # السالِب
    for i, v in enumerate(neg_values):
        append_step(v, "neg_test", i)
        if return_to_zero:
            append_step(0.0, "zero_between", i)

    # عَودَة نِهائيَّة
    if not return_to_zero and neg_values:
        append_step(0.0, "zero_final", 0)

    total = pre_settle_s + len(cmd_seq) * dwell_s + 0.3

    def cmd_fn(t_s: float) -> float:
        if t_s < pre_settle_s:
            return 0.0
        idx = int((t_s - pre_settle_s) / dwell_s)
        if idx < 0:
            return 0.0
        if idx >= len(cmd_seq):
            return 0.0
        return cmd_seq[idx]

    desc = (f"firmware_audit: pos={len(pos_values)}, neg={len(neg_values)}, "
            f"dwell={dwell_s}s, return_to_zero={return_to_zero} "
            f"({total:.1f}s, {len(cmd_seq)} steps)")
    return PatternSpec(
        duration_s=total, cmd_fn=cmd_fn, description=desc,
        schedule=schedule,
    )
