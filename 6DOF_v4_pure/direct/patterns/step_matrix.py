"""Step matrix — sweep step responses across (amplitude × offset × direction × repeat).

يفحص اعتماد delay/τ/overshoot على:
  - حجم الخطوة (amplitudes_deg)
  - نقطة العمل (offsets_deg)
  - الاتجاه (up / down / both)
  - التكرار (repeats — للحصول على mean ± std لكل خلية)

شكل كل خلية واحدة:
    settle_s  @ offset
    dwell_s   @ (offset + amp)         ← edge UP
    dwell_s   @ offset                  ← edge DOWN (back)

الخلايا تُولَّد بترتيب thoughtful: نُجمّع بنفس offset لتقليل الانتقالات الكبيرة.

Schedule (يُحفَظ في pattern.schedule لاستهلاك في analysis):
    [{cell_id, t_start, t_end, amp_deg, offset_deg, direction, repeat_idx,
      cmd_initial, cmd_target, cmd_return}, ...]
"""

from __future__ import annotations

from typing import Any, Dict, List

from . import PatternSpec


def build(cfg: dict) -> PatternSpec:
    sub = cfg.get("step_matrix", {})

    amps = [float(x) for x in sub.get("amplitudes_deg", [1, 3, 5, 8, 10])]
    offsets = [float(x) for x in sub.get("offsets_deg", [0])]
    directions_arg = str(sub.get("directions", "both")).lower()
    if directions_arg == "up":
        directions = ["up"]
    elif directions_arg == "down":
        directions = ["down"]
    elif directions_arg == "both":
        directions = ["up", "down"]
    else:
        raise ValueError(
            f"step_matrix.directions يجب 'up'/'down'/'both'، وجد '{directions_arg}'"
        )

    repeats = int(sub.get("repeats", 3))
    settle_s = float(sub.get("settle_s", 0.5))
    dwell_s = float(sub.get("dwell_s", 0.6))

    if repeats < 1:
        raise ValueError("step_matrix.repeats يجب ≥ 1")
    if settle_s < 0.1 or dwell_s < 0.1:
        raise ValueError("settle_s و dwell_s يجب ≥ 0.1s")
    if not amps or not offsets:
        raise ValueError("step_matrix يحتاج amplitudes_deg و offsets_deg غير فارغة")

    # ── ولّد timeline ─────────────────────────────────────────────
    # كل cell = settle (offset) → dwell (target) → dwell (back to offset)
    # مدّة cell = settle_s + 2*dwell_s
    cell_duration = settle_s + 2.0 * dwell_s

    timeline: List[Dict[str, Any]] = []  # سيمتلئ بـ {t_start, cmd}
    schedule: List[Dict[str, Any]] = []

    t = 0.0
    cell_id = 0
    # ترتيب مُحَسَّن: لكل offset → لكل amp → لكل direction → كرّر
    for offset in offsets:
        for amp in amps:
            for direction in directions:
                target = offset + amp if direction == "up" else offset - amp
                for rep in range(repeats):
                    t_settle_start = t
                    t_edge_up = t + settle_s
                    t_edge_down = t + settle_s + dwell_s
                    t_end = t + cell_duration

                    # ابني timeline (segments بالأمر بدون hold)
                    timeline.append((t_settle_start, offset))
                    timeline.append((t_edge_up, target))
                    timeline.append((t_edge_down, offset))

                    schedule.append({
                        "cell_id": cell_id,
                        "t_start": round(t_settle_start, 4),
                        "t_edge_up_s": round(t_edge_up, 4),
                        "t_edge_down_s": round(t_edge_down, 4),
                        "t_end": round(t_end, 4),
                        "amp_deg": amp,
                        "offset_deg": offset,
                        "direction": direction,
                        "repeat_idx": rep,
                        "cmd_initial": offset,
                        "cmd_target": target,
                    })
                    cell_id += 1
                    t = t_end

    total_duration = t

    # حُل cmd_fn(t_s) عن طريق last-event-before-t (timeline مرتّب)
    # نستخدم إيجاد أحدث (t_start <= t_s)
    timeline_arr = timeline  # list of (t_start, cmd)
    final_cmd = timeline_arr[-1][1]

    def cmd_fn(t_s: float) -> float:
        if t_s >= total_duration:
            return final_cmd
        # binary search via linear scan (timeline حجمه لطيف)
        cmd = timeline_arr[0][1]
        for t_start, c in timeline_arr:
            if t_s >= t_start:
                cmd = c
            else:
                break
        return cmd

    desc = (
        f"step_matrix: amps={amps}° offsets={offsets}° "
        f"dirs={directions} ×{repeats}reps  "
        f"({len(schedule)} cells, {total_duration:.1f}s)"
    )

    return PatternSpec(
        duration_s=total_duration,
        cmd_fn=cmd_fn,
        description=desc,
        schedule=schedule,
    )
