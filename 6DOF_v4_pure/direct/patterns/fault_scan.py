"""fault_scan — راصِد انوماليّات سيرفو/CAN باستِمرار.

نَمَط القِيادَة: موجَة مُربَّعَة بَطيئَة ±amp_deg بـ period_s. الـ pattern
نَفسه بَسيط؛ القيمَة المُضافَة في **التحليل** الذي يَكتَشِف العديد من
أَنواع الأَعطال:

| Anomaly                       | كَيف يُكشَف                                    |
|--------------------------------|-----------------------------------------------|
| **fb gap** (CAN frame loss)   | Δt_arrival > gap_max_ms                       |
| **fb jump** (encoder glitch)  | |Δfb| > jump_max_deg في عَيِّنَتَين مُتَتاليَتَين  |
| **stale fb**                  | لا fresh frame لـ stale_max_ms                |
| **sign mismatch**             | sign(cmd) ≠ sign(fb) بَعد debounce_s          |
| **saturation**                | |fb| ≥ angle_limit بَينَما |cmd| < angle_limit |
| **dead servo**                | std(fb) < 0.05° بَينَما cmd يَتَغَيَّر          |
| **excess overshoot**          | overshoot > os_max_pct على edge               |
| **slow recovery**             | t_settle > settle_max_ms                      |

المُخرَج: PASS لو كلّ الأنوماليّات تَحت الحُدود، وإلّا FAIL مع تَفاصيل
لكلّ نوع و سيرفو.
"""

from __future__ import annotations

from typing import Any, Dict, List

from . import PatternSpec


def build(cfg: dict) -> PatternSpec:
    sub = cfg.get("fault_scan", {})

    duration_s = float(sub.get("duration_s", 30.0))
    amp_deg = float(sub.get("amp_deg", 3.0))
    period_s = float(sub.get("period_s", 1.0))
    pre_settle_s = float(sub.get("pre_settle_s", 0.5))

    if duration_s < 5.0:
        raise ValueError("fault_scan.duration_s ≥ 5.0")
    if amp_deg <= 0:
        raise ValueError("fault_scan.amp_deg يَجِب > 0")
    if period_s < 0.4:
        raise ValueError("fault_scan.period_s ≥ 0.4 (لإعطاء وَقت settle)")

    half = period_s / 2.0
    n_edges_full = int(duration_s / half)
    total_duration = pre_settle_s + duration_s

    # ─── schedule: كلّ edge مَوقوت لمُساعَدَة التحليل في عَزل
    # نَوافِذ "transition" (يُتَوَقَّع جيتّر أو Jump) عن "steady" (نَوافِذ
    # هُدوء يَجِب ألّا تَحوي gaps أَو jumps).
    schedule: List[Dict[str, Any]] = []
    for k in range(n_edges_full):
        t_edge = pre_settle_s + k * half
        direction = "up" if (k % 2) == 0 else "down"
        cmd_to = +amp_deg if direction == "up" else -amp_deg
        schedule.append({
            "phase": "fault_scan_edge",
            "edge_idx": k,
            "t_edge_s": round(t_edge, 4),
            "direction": direction,
            "cmd_to": cmd_to,
            # نافِذَة "steady" بَعد الـ edge بـ 100ms حَتَّى الـ edge التالي
            "t_steady_start_s": round(t_edge + 0.10, 4),
            "t_steady_end_s": round(t_edge + half, 4),
        })

    def cmd_fn(t_s: float) -> float:
        if t_s < pre_settle_s:
            return 0.0
        rel = t_s - pre_settle_s
        if rel >= duration_s:
            return 0.0
        # موجة مُربَّعة:
        return +amp_deg if (rel % period_s) < half else -amp_deg

    desc = (f"fault_scan: square ±{amp_deg:.1f}° period={period_s:.1f}s "
            f"for {duration_s:.1f}s ({n_edges_full} edges)")

    return PatternSpec(
        duration_s=total_duration,
        cmd_fn=cmd_fn,
        description=desc,
        schedule=schedule,
    )
