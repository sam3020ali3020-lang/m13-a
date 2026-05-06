"""Dead-band pattern.

يَختبر أصغر تَغيير في الأمر يُسَبّب استجابة مَحسوسة في fb.
نَبدأ من 0° ثمّ نَعطي steps صغيرة جدّاً مُتزايدة:
  ±0.025°, ±0.05°, ±0.10°, ±0.15°, ±0.20°, ±0.30°, ±0.50°, ±1.0°.

التَّحليل:
- لكلّ amplitude: قِس Δfb = mean(fb after) − mean(fb before)
- ابحَث عن أصغر amp يُعطي |Δfb| > noise_threshold (مَثلاً 2× std الـ idle)
- النتيجة: dead_band_deg = أصغر cmd مَعنويّ

يَكشف:
- encoder resolution (1/18 = 0.0556°)
- PID dead-zone في firmware
- gear/coupling backlash dominance
"""
from __future__ import annotations

from typing import Any, Dict, List

from . import PatternSpec


def build(cfg: dict) -> PatternSpec:
    sub = cfg.get("dead_band", {})
    amplitudes = sub.get(
        "amplitudes_deg",
        [0.025, 0.05, 0.10, 0.15, 0.20, 0.30, 0.50, 1.0],
    )
    dwell_s = float(sub.get("dwell_s", 1.0))
    repeats = int(sub.get("repeats", 3))
    pre_settle_s = float(sub.get("pre_settle_s", 1.0))

    amplitudes = [float(a) for a in amplitudes]
    if not amplitudes:
        raise ValueError("dead_band.amplitudes_deg فارغة")

    # Sequence: لكلّ amp و كلّ rep:  0 → +amp → 0 → -amp → 0
    boundaries: List[tuple] = [(0.0, 0.0)]
    schedule: List[Dict[str, Any]] = []
    t_acc = pre_settle_s
    for amp in amplitudes:
        for rep in range(repeats):
            # idle window قبل الـ +step (للحساب baseline)
            schedule.append({
                "phase": "baseline_pos", "amp_deg": amp, "rep": rep,
                "direction": "up",
                "t_baseline_start_s": t_acc - 0.5,
                "t_baseline_end_s": t_acc,
                "t_step_s": t_acc,
                "t_dwell_end_s": t_acc + dwell_s,
                "cmd_target": +amp,
            })
            boundaries.append((t_acc, +amp))
            t_acc += dwell_s
            # عودة 0
            boundaries.append((t_acc, 0.0))
            schedule.append({
                "phase": "baseline_neg", "amp_deg": amp, "rep": rep,
                "direction": "down",
                "t_baseline_start_s": t_acc - 0.3,
                "t_baseline_end_s": t_acc,
                "t_step_s": t_acc,
                "t_dwell_end_s": t_acc + dwell_s,
                "cmd_target": -amp,
            })
            t_acc += dwell_s
            # -step
            boundaries.append((t_acc, -amp))
            t_acc += dwell_s
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

    desc = (f"dead_band: {len(amplitudes)} amps × {repeats} reps "
            f"({total:.1f}s)")
    return PatternSpec(
        duration_s=total, cmd_fn=cmd_fn, description=desc,
        schedule=schedule,
    )
