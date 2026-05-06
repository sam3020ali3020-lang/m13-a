"""Rate-limit verification pattern.

يَطلب step كبير سَريع لتَشبيع rate-limit للسيرفو ثمّ يَقيس peak slew rate
الفعليّ. يُكَرّر بـ amplitudes مُتزايدة لِرَسم منحنى slew_max vs amplitude:
- عند amp صغير: slew محدود بـ τ (= amp/τ تقريبياً)
- عند amp كبير: slew يَصِل إلى rate_max (saturation)

التَّحليل يُخرِج:
- peak slew لكلّ amplitude
- amplitude_threshold عند بداية التَّشبّع
- rate_max الفعليّ المُقدَّر (asymptote)
- مُقارنة مع rated 492°/s (XQ-BLS8145C)

⚠️ Step كبير سَريع قد يَتَجاوز angle_limit_deg المَضبوط في safety —
الـ runner سيَقصّ. تَأكّد من angle_limit_deg ≥ max_amp_deg.
"""
from __future__ import annotations

from typing import Any, Dict, List

from . import PatternSpec


def build(cfg: dict) -> PatternSpec:
    sub = cfg.get("rate_limit_verify", {})
    amplitudes = sub.get("amplitudes_deg", [2.0, 5.0, 10.0, 15.0, 18.0])
    dwell_s = float(sub.get("dwell_s", 0.8))
    repeats = int(sub.get("repeats", 2))
    pre_settle_s = float(sub.get("pre_settle_s", 0.5))

    amplitudes = [float(a) for a in amplitudes]
    if not amplitudes:
        raise ValueError("rate_limit_verify.amplitudes_deg فارغة")
    if dwell_s <= 0:
        raise ValueError(f"dwell_s يجب > 0، {dwell_s}")

    # لكلّ amp: 0 → +amp → -amp → 0  (ضِعفان step كَبيران)
    # كرّر repeats × |amplitudes| مرّات
    boundaries: List[tuple] = [(0.0, 0.0)]
    schedule: List[Dict[str, Any]] = []
    t_acc = pre_settle_s
    for _ in range(repeats):
        for amp in amplitudes:
            # step UP
            boundaries.append((t_acc, +amp))
            schedule.append({
                "t_edge_s": t_acc, "amp_deg": amp, "direction": "up",
                "cmd_from": 0.0, "cmd_to": +amp,
            })
            t_acc += dwell_s
            # step DOWN
            boundaries.append((t_acc, -amp))
            schedule.append({
                "t_edge_s": t_acc, "amp_deg": amp, "direction": "down",
                "cmd_from": +amp, "cmd_to": -amp,
            })
            t_acc += dwell_s
            # عودة 0
            boundaries.append((t_acc, 0.0))
            t_acc += dwell_s
    total = t_acc

    def cmd_fn(t_s: float) -> float:
        cmd = 0.0
        for t_start, c in boundaries:
            if t_s >= t_start:
                cmd = c
            else:
                break
        return cmd

    desc = (f"rate_limit_verify: {len(amplitudes)} amps × {repeats} reps "
            f"({total:.1f}s, {len(schedule)} edges)")
    return PatternSpec(
        duration_s=total, cmd_fn=cmd_fn, description=desc,
        schedule=schedule,
    )
