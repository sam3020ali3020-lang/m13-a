"""Cold-start pattern.

يَختبر استجابة السيرفو بعد فترة سَكون طويلة (idle). الهدف: كَشف "cold-USB
transient" أو cold-MCU effects الّتي قد تُؤخّر/تُغيّر الاستجابة الأُولى.

التَّسلسل:
1. idle طويل (لا أوامر) لمدّة `idle_s` ثوانٍ — ⚠️ هذا الـ pattern يُرسل
   0° خلال هذه الفترة (لا يَستطيع الـ runner التَوقّف)، لكنّ السيرفو
   ساكن فعليّاً.
2. ثلاث steps مُتَتالية بفاصل قصير لقياس:
   - step #1: استجابة "cold" — أوّل تَحريك بعد الـ idle
   - step #2: استجابة "warm" — السيرفو دافئ
   - step #3: استجابة "hot" — للتَأكيد

التَّحليل (Phase-1 generic step metrics):
- مُقارنة delay/τ بين step#1 vs #2 vs #3
- لو delay#1 > delay#2 بـ 20%+ → cold-start يَستحقّ تَعديل warm-up

ملاحظة: warmup_exercise في safety يُلغي cold-start فعلياً. لاختبار حقيقيّ
يَنبغي ضَبْط warmup_exercise_s = 0 في safety.
"""
from __future__ import annotations

from typing import Any, Dict, List

from . import PatternSpec


def build(cfg: dict) -> PatternSpec:
    sub = cfg.get("cold_start", {})
    idle_s = float(sub.get("idle_s", 30.0))           # سَكون 30s
    step_amp_deg = float(sub.get("step_amp_deg", 5.0))
    step_dwell_s = float(sub.get("step_dwell_s", 1.0))
    n_steps = int(sub.get("n_steps", 3))
    pre_settle_s = float(sub.get("pre_settle_s", 0.5))

    if idle_s < 5:
        raise ValueError(f"cold_start.idle_s يجب ≥ 5، {idle_s}")

    # Sequence:
    #   pre_settle (0°) → idle (0°) → [+amp dwell, -amp dwell, +amp dwell, ...] × n_steps
    schedule: List[Dict[str, Any]] = []
    boundaries: List[tuple] = [(0.0, 0.0)]
    t_acc = pre_settle_s + idle_s

    for i in range(n_steps):
        cmd_target = +step_amp_deg if (i % 2 == 0) else -step_amp_deg
        boundaries.append((t_acc, cmd_target))
        schedule.append({
            "step_idx": i,
            "t_edge_s": t_acc,
            "cmd_to": cmd_target,
            "cmd_from": 0.0 if i == 0 else (
                -step_amp_deg if (i % 2 == 0) else +step_amp_deg
            ),
            "label": "cold" if i == 0 else ("warm" if i == 1 else "hot"),
        })
        t_acc += step_dwell_s
        # عودة 0
        boundaries.append((t_acc, 0.0))
        t_acc += step_dwell_s
    total = t_acc

    def cmd_fn(t_s: float) -> float:
        cmd = 0.0
        for t_start, c in boundaries:
            if t_s >= t_start:
                cmd = c
            else:
                break
        return cmd

    desc = (f"cold_start: idle={idle_s:.0f}s + {n_steps} steps@±{step_amp_deg}° "
            f"({total:.1f}s)")
    return PatternSpec(
        duration_s=total, cmd_fn=cmd_fn, description=desc,
        schedule=schedule,
    )
