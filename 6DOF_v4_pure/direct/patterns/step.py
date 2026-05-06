"""Step response pattern.

Sequence format:  list of [dt_s, angle_deg].
الأمر = angle_deg خلال نافذة dt_s التي تسبقه عند كل لحظة.
"""

from __future__ import annotations

from . import PatternSpec


def build(cfg: dict) -> PatternSpec:
    sub = cfg.get("step", {})
    seq = sub.get("sequence", [])
    if not seq:
        raise ValueError("step.sequence فارغة")

    # بناء قائمة (t_start, angle) بعد التراكم
    timeline = []
    t_acc = 0.0
    for pair in seq:
        dt, ang = float(pair[0]), float(pair[1])
        if dt <= 0:
            raise ValueError(f"step.sequence dt يجب > 0، وجد {dt}")
        timeline.append((t_acc, float(ang)))
        t_acc += dt
    total = t_acc

    # holds after last segment
    final_angle = float(seq[-1][1])

    def cmd_fn(t_s: float) -> float:
        if t_s >= total:
            return final_angle
        # binary-search بسيط
        angle = timeline[0][1]
        for t_start, ang in timeline:
            if t_s >= t_start:
                angle = ang
            else:
                break
        return angle

    steps_desc = " → ".join(f"{a:+.1f}°" for _, a in timeline)
    desc = f"step: {steps_desc}  (total {total:.2f}s)"
    return PatternSpec(duration_s=total, cmd_fn=cmd_fn, description=desc)
