"""wiring_audit — كَشف خَلط أَسلاك السيرفو بِبَصمَة تَرَدُّديَّة فَريدَة.

كلّ سيرفو يَأخُذ موجة جَيبيَّة بتَرَدُّد فَريد في وَقت واحد. التحليل يُحَلِّل
طَيف fb لكلّ سيرفو ويَتَحَقَّق:

* قِمَّة الطَيف عِند التَرَدُّد المُتَوَقَّع لذلك السيرفو ✓
* لا قِمم قَويَّة عِند تَرَدُّدات السيرفوهات الأُخرى ✓

إذا كان servo[node_X] يَستَجيب لِبَصمَة servo[node_Y] → الأَسلاك مَخلوطَة
أَو الـ node IDs مُحَدَّدَة خَطأ في الـ firmware.

اختِيار التَرَدُّدات: تَباعُد كاف لِفَصلِها في FFT و كلّها أقَلّ من
servo bandwidth المُتَوَقَّع (~10-15Hz). الـ defaults تُعطي 4 سيرفوهات بـ
1.5/2.5/3.5/5.5Hz و كلّها قابِلَة للتَتَبُّع بسَهولَة.

Output schedule:
    [{"slot": 0, "freq_hz": 1.5, "amp_deg": 4.0}, ...]
"""

from __future__ import annotations

from math import pi, sin
from typing import Any, Dict, List, Sequence

from . import PatternSpec


# تَرَدُّدات افتِراضيَّة لكلّ سيرفو — مُتَباعِدَة لِسهولَة الفَصل في FFT.
# لا تُطابِق نِسَب صَحيحَة (مَنع cross-talk عِند harmonics).
DEFAULT_FREQS_HZ = [1.5, 2.5, 3.5, 5.5]


def build(cfg: dict) -> PatternSpec:
    sub = cfg.get("wiring_audit", {})

    duration_s = float(sub.get("duration_s", 8.0))
    amp_deg = float(sub.get("amp_deg", 4.0))
    pre_settle_s = float(sub.get("pre_settle_s", 0.5))

    freqs_cfg = sub.get("freqs_hz", None)
    if freqs_cfg is None:
        freqs = list(DEFAULT_FREQS_HZ)
    else:
        freqs = [float(x) for x in freqs_cfg]
        if len(freqs) < 1:
            raise ValueError("wiring_audit.freqs_hz فارِغَة")

    # ─── validations ────────────────────────────────────────────────────
    if duration_s < 4.0:
        raise ValueError("wiring_audit.duration_s ≥ 4.0 (لاستِخراج FFT جيد)")
    if amp_deg <= 0:
        raise ValueError("wiring_audit.amp_deg يَجِب > 0")
    # تَأكَّد كلّ التَرَدُّدات ضِمن النِطاق (≥ 2 cycles in window)
    f_min_required = 2.0 / duration_s
    for f in freqs:
        if f <= 0:
            raise ValueError("wiring_audit: التَرَدُّد يَجِب > 0")
        if f < f_min_required:
            raise ValueError(
                f"wiring_audit: التَرَدُّد {f}Hz صَغير جِدّاً لـ "
                f"duration={duration_s}s (≥ {f_min_required:.2f}Hz)")
    # تَأكَّد لا تَكرار
    if len(set(freqs)) != len(freqs):
        raise ValueError("wiring_audit.freqs_hz: قِيَم مُكَرَّرَة")
    # تَأكَّد تَباعُد ≥ 0.5Hz بَين أَيَّ تَرَدُّدَين (لتَجَنُّب FFT bin overlap)
    fs_sorted = sorted(freqs)
    for i in range(1, len(fs_sorted)):
        if fs_sorted[i] - fs_sorted[i - 1] < 0.5:
            raise ValueError(
                f"wiring_audit: تَرَدُّدات قَريبَة جِدّاً "
                f"({fs_sorted[i-1]:.2f} و {fs_sorted[i]:.2f}Hz) — يَجِب "
                f"تَباعُد ≥ 0.5Hz لفَصل FFT bins")

    total_duration = pre_settle_s + duration_s

    # ─── schedule ───────────────────────────────────────────────────────
    schedule: List[Dict[str, Any]] = []
    for slot, f in enumerate(freqs):
        schedule.append({
            "phase": "wiring_audit",
            "slot_in_target": slot,
            "expected_freq_hz": float(f),
            "amp_deg": float(amp_deg),
            "t_start_s": round(pre_settle_s, 4),
            "t_end_s": round(total_duration, 4),
        })

    # ─── command function (per-servo: كلّ slot يأخُذ تَرَدُّده) ─────────
    omega = [2.0 * pi * f for f in freqs]

    def cmd_fn_multi(t_s: float, target: Sequence[int],
                     n: int) -> List[float]:
        out = [0.0] * n
        if t_s < pre_settle_s:
            return out
        rel = t_s - pre_settle_s
        if rel >= duration_s:
            return out
        # كلّ slot في target يَأخُذ تَرَدُّده. السيرفوهات خارِج target = 0°.
        for slot, servo_idx in enumerate(target):
            if slot >= len(omega):
                break  # لا يَكفي تَرَدُّدات للسيرفوهات الإضافيَّة
            si = int(servo_idx)
            if 0 <= si < n:
                out[si] = amp_deg * sin(omega[slot] * rel)
        return out

    # cmd_fn fallback — slot 0 فقط (تَقريب)
    def cmd_fn(t_s: float) -> float:
        if t_s < pre_settle_s or t_s - pre_settle_s >= duration_s:
            return 0.0
        return amp_deg * sin(omega[0] * (t_s - pre_settle_s))

    desc = (f"wiring_audit: {len(freqs)} unique freqs "
            f"({', '.join(f'{f:.1f}Hz' for f in freqs)}) "
            f"×{amp_deg:.1f}° for {duration_s:.1f}s")

    return PatternSpec(
        duration_s=total_duration,
        cmd_fn=cmd_fn,
        cmd_fn_multi=cmd_fn_multi,
        description=desc,
        schedule=schedule,
    )
