"""End-stop verification pattern.

يَفحص سلوك السيرفو عند الحدود الزاويّة المُعلَنة (delta_max / delta_min).
الاختبار يَتمّ بـ slow ramp إلى الحدّ، hold، ثمّ ramp عَكسيّ.

⚠️ الـ angle_limit_deg في safety.* يَفرض clamp في الـ runner — هذا الـ pattern
يَستخدم الحدّ الفعليّ (= angle_limit_deg) ولا يَتجاوزه. الهدف ليس كَسْر
السيرفو، بل التَّحقّق من:
- هل fb يَصِل فعلاً إلى الحدّ المَطلوب؟ (أو أقلّ بسبب hard-stop ميكانيكيّ)
- هل ثَمّة bounce/oscillation عند ضَرب الحدّ؟
- هل الـ commanded vs achieved gap متّسق بين السيرفوهات؟

التَّحليل:
- max_fb_pos / min_fb_neg
- gap من الحدّ المَطلوب
- post-hit oscillation amplitude (std خلال آخر 0.5s من الـ hold)
"""
from __future__ import annotations

from typing import Any, Dict, List

from . import PatternSpec


def build(cfg: dict) -> PatternSpec:
    sub = cfg.get("end_stop", {})
    target_deg = float(sub.get("target_deg", 18.0))   # ضمن angle_limit
    ramp_rate_dps = float(sub.get("ramp_rate_dps", 30.0))   # slow approach
    hold_s = float(sub.get("hold_s", 2.0))
    pre_settle_s = float(sub.get("pre_settle_s", 0.5))

    if target_deg <= 0 or ramp_rate_dps <= 0 or hold_s <= 0:
        raise ValueError("end_stop: target/ramp/hold يجب > 0")

    ramp_dur = target_deg / ramp_rate_dps   # واحد ramp 0→target
    # تَسلسل: 0 → +target → 0 → -target → 0
    # كلّ ramp مدّته ramp_dur، كلّ hold مدّته hold_s
    t_pos_start = pre_settle_s
    t_pos_top = t_pos_start + ramp_dur
    t_pos_end = t_pos_top + hold_s
    t_back0_start = t_pos_end
    t_back0_end = t_back0_start + ramp_dur
    t_neg_start = t_back0_end
    t_neg_bot = t_neg_start + ramp_dur
    t_neg_end = t_neg_bot + hold_s
    t_final_start = t_neg_end
    t_final_end = t_final_start + ramp_dur
    total = t_final_end + 0.3

    schedule: List[Dict[str, Any]] = [
        {"phase": "pos_ramp",  "t_start_s": t_pos_start, "t_end_s": t_pos_top,
         "cmd_target": target_deg},
        {"phase": "pos_hold",  "t_start_s": t_pos_top,   "t_end_s": t_pos_end,
         "cmd_target": target_deg},
        {"phase": "neg_ramp",  "t_start_s": t_neg_start, "t_end_s": t_neg_bot,
         "cmd_target": -target_deg},
        {"phase": "neg_hold",  "t_start_s": t_neg_bot,   "t_end_s": t_neg_end,
         "cmd_target": -target_deg},
    ]

    def cmd_fn(t_s: float) -> float:
        if t_s < t_pos_start:
            return 0.0
        if t_s < t_pos_top:        # ramp 0 → +target
            return ramp_rate_dps * (t_s - t_pos_start)
        if t_s < t_pos_end:        # hold +target
            return target_deg
        if t_s < t_back0_end:      # ramp +target → 0
            return target_deg - ramp_rate_dps * (t_s - t_back0_start)
        if t_s < t_neg_bot:        # ramp 0 → -target
            return -ramp_rate_dps * (t_s - t_neg_start)
        if t_s < t_neg_end:        # hold -target
            return -target_deg
        if t_s < t_final_end:      # ramp -target → 0
            return -target_deg + ramp_rate_dps * (t_s - t_final_start)
        return 0.0

    desc = (f"end_stop: ramp@{ramp_rate_dps:.0f}°/s to ±{target_deg:.1f}° "
            f"hold={hold_s:.1f}s ({total:.1f}s)")
    return PatternSpec(
        duration_s=total, cmd_fn=cmd_fn, description=desc,
        schedule=schedule,
    )
