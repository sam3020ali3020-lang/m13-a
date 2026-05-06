"""Nokia Tune (Gran Vals) — نغمة نوكيا المشهورة لتشخيص السيرفوهات.

كل الدفّات تتحرّك بنفس اللحن في نفس الوقت.
أيّ سيرفو يخالف الباقي يكون واضحاً فوراً بالعين.

النوتات (RTTTL: d=4,o=5,b=225):
  E5 D5 F#4 G#4 | C#5 B4 D4 E4 | B4 A4 C#4 E4 | A4
  درري رن درري رن درري ري رن

وضعان:
  mode: melody  — كل نوتة = زاوية ثابتة مع staccato (فصل بين النوتات).
                   اللحن مرئي بالعين.
  mode: buzz    — كل نوتة = اهتزاز بتردد مختلف (موجة مربّعة).
                   السيرفوهات تصدر صوت اللحن فعلياً من الاهتزاز.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List

from . import PatternSpec

# ─── Nokia Tune melody (RTTTL verified) ────────────────────────────────────
# (midi_note, duration_in_eighth_notes)
# 8th=1, quarter=2, half=4
_MELODY = [
    (76, 1),   # E5  - 8th   ─┐
    (74, 1),   # D5  - 8th    │ "درري"
    (66, 2),   # F#4 - qtr    │ "رن"
    (68, 2),   # G#4 - qtr   ─┘ "رن"
    (73, 1),   # C#5 - 8th   ─┐
    (71, 1),   # B4  - 8th    │ "درري"
    (62, 2),   # D4  - qtr    │ "رن"
    (64, 2),   # E4  - qtr   ─┘ "رن"
    (71, 1),   # B4  - 8th   ─┐
    (69, 1),   # A4  - 8th    │ "درري"
    (61, 2),   # C#4 - qtr    │ "ري"
    (64, 2),   # E4  - qtr    │ "رن"
    (69, 4),   # A4  - half  ─┘ (ending)
]

_MIDI_MIN = min(n for n, _ in _MELODY)  # 61
_MIDI_MAX = max(n for n, _ in _MELODY)  # 76


def _midi_to_angle(midi: int, amp: float) -> float:
    """خريطة خطية: MIDI [61..76] → angle [-amp..+amp]."""
    return -amp + (midi - _MIDI_MIN) / (_MIDI_MAX - _MIDI_MIN) * 2.0 * amp


def _midi_to_freq(midi: int, base_hz: float) -> float:
    """MIDI → تردد اهتزاز (equal temperament من base_hz عند MIDI_MIN)."""
    return base_hz * (2.0 ** ((midi - _MIDI_MIN) / 12.0))


# ═══════════════════════════════════════════════════════════════════════════
#  Mode: melody — position steps with staccato articulation
# ═══════════════════════════════════════════════════════════════════════════

def _build_melody(sub: dict) -> PatternSpec:
    amp = float(sub.get("amplitude_deg", 8.0))
    eighth_s = float(sub.get("eighth_note_s", 0.25))
    repeats = int(sub.get("repeats", 2))
    gap_s = float(sub.get("gap_between_repeats_s", 0.6))

    # بناء timeline لتكرار واحد: (t_start, angle, duration)
    single_notes = []
    t_acc = 0.0
    for midi, dur_eighths in _MELODY:
        angle = _midi_to_angle(midi, amp)
        dur = dur_eighths * eighth_s
        single_notes.append((t_acc, angle, dur))
        t_acc += dur
    single_dur = t_acc

    total = repeats * single_dur + max(0, repeats - 1) * gap_s

    schedule: List[Dict[str, Any]] = []
    for rep in range(repeats):
        base_t = rep * (single_dur + gap_s)
        for i, (t_rel, angle, dur) in enumerate(single_notes):
            schedule.append({
                "repeat": rep, "note_idx": i,
                "t_start_s": round(base_t + t_rel, 4),
                "t_end_s": round(base_t + t_rel + dur, 4),
                "angle_deg": round(angle, 2),
            })

    def cmd_fn(t_s: float) -> float:
        if t_s < 0 or t_s >= total:
            return 0.0
        cycle_dur = single_dur + gap_s
        rep_idx = int(t_s / cycle_dur)
        if rep_idx >= repeats:
            return 0.0
        t_in = t_s - rep_idx * cycle_dur
        if t_in >= single_dur:
            return 0.0
        # ابحث عن النوتة — خطوات مباشرة بدون فجوات
        for t_start, ang, dur in reversed(single_notes):
            if t_in >= t_start:
                return ang
        return 0.0

    desc = (f"nokia_tune[melody]: ±{amp:.1f}° 8th={eighth_s:.2f}s "
            f"×{repeats} ({total:.1f}s) — "
            f"درري رن درري رن درري ري رن")
    return PatternSpec(
        duration_s=total, cmd_fn=cmd_fn,
        description=desc, schedule=schedule,
    )


# ═══════════════════════════════════════════════════════════════════════════
#  Mode: buzz — servo vibration at note-proportional frequencies
# ═══════════════════════════════════════════════════════════════════════════

def _build_buzz(sub: dict) -> PatternSpec:
    amp = float(sub.get("buzz_amplitude_deg", 2.0))
    eighth_s = float(sub.get("eighth_note_s", 0.25))
    repeats = int(sub.get("repeats", 2))
    gap_s = float(sub.get("gap_between_repeats_s", 0.6))
    base_hz = float(sub.get("base_freq_hz", 12.0))
    # offset_amp: إضافة offset ثابت لكل نوتة (يُظهر اللحن بصرياً أيضاً)
    offset_amp = float(sub.get("offset_amplitude_deg", 4.0))
    # wave: square أو sine
    wave = str(sub.get("wave", "square")).lower()

    single_notes = []   # (t_start, freq_hz, offset, total_dur)
    t_acc = 0.0
    for midi, dur_eighths in _MELODY:
        freq = _midi_to_freq(midi, base_hz)
        offset = _midi_to_angle(midi, offset_amp) if offset_amp > 0 else 0.0
        total_dur = dur_eighths * eighth_s
        single_notes.append((t_acc, freq, offset, total_dur))
        t_acc += total_dur
    single_dur = t_acc

    total = repeats * single_dur + max(0, repeats - 1) * gap_s

    schedule: List[Dict[str, Any]] = []
    for rep in range(repeats):
        base_t = rep * (single_dur + gap_s)
        for i, (t_rel, freq, offset, td) in enumerate(single_notes):
            schedule.append({
                "repeat": rep, "note_idx": i,
                "t_start_s": round(base_t + t_rel, 4),
                "t_end_s": round(base_t + t_rel + td, 4),
                "freq_hz": round(freq, 1),
                "offset_deg": round(offset, 2),
            })

    def cmd_fn(t_s: float) -> float:
        if t_s < 0 or t_s >= total:
            return 0.0
        cycle_dur = single_dur + gap_s
        rep_idx = int(t_s / cycle_dur)
        if rep_idx >= repeats:
            return 0.0
        t_in = t_s - rep_idx * cycle_dur
        if t_in >= single_dur:
            return 0.0
        # ابحث عن النوتة الحالية
        freq, offset = base_hz, 0.0
        for t_start, f, off, td in reversed(single_notes):
            if t_in >= t_start:
                freq, offset = f, off
                break
        # اهتزاز
        if wave == "sine":
            osc = amp * math.sin(2.0 * math.pi * freq * t_s)
        else:
            # square wave — أقوى صوت ميكانيكي
            osc = amp if (t_s * freq) % 1.0 < 0.5 else -amp
        return offset + osc

    wave_label = "□" if wave == "square" else "∿"
    desc = (f"nokia_tune[buzz{wave_label}]: ±{amp:.1f}° vibration "
            f"f={base_hz:.0f}-{_midi_to_freq(_MIDI_MAX, base_hz):.0f}Hz "
            f"offset±{offset_amp:.1f}° ×{repeats} ({total:.1f}s)")
    return PatternSpec(
        duration_s=total, cmd_fn=cmd_fn,
        description=desc, schedule=schedule,
    )


# ═══════════════════════════════════════════════════════════════════════════
#  Factory
# ═══════════════════════════════════════════════════════════════════════════

def build(cfg: dict) -> PatternSpec:
    sub = cfg.get("nokia_tune", {})
    mode = str(sub.get("mode", "melody")).lower()
    if mode == "buzz":
        return _build_buzz(sub)
    return _build_melody(sub)
