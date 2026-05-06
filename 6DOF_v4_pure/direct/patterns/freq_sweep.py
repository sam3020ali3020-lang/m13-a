"""Frequency sweep (chirp) — sinusoidal sweep من f_start إلى f_end.

Linear:  freq(t) = f0 + (f1 - f0) * t/T
Log:     freq(t) = f0 * (f1/f0)**(t/T)

الزاوية = amplitude * sin(2π ∫ freq dt)

يستخدم لاستخراج Bode plot عبر FFT في direct_analysis.
"""

from __future__ import annotations

import math

from . import PatternSpec


def build(cfg: dict) -> PatternSpec:
    sub = cfg.get("freq_sweep", {})
    amp = float(sub.get("amplitude_deg", 3.0))
    f0 = float(sub.get("f_start_hz", 0.1))
    f1 = float(sub.get("f_end_hz", 10.0))
    T = float(sub.get("duration_s", 30.0))
    kind = str(sub.get("sweep_type", "log")).lower()

    if amp <= 0 or T <= 0 or f0 <= 0 or f1 <= 0:
        raise ValueError("amplitude/duration/f_start/f_end يجب > 0")

    # phase closed-form:
    # linear: φ(t) = 2π (f0·t + (f1-f0)·t²/(2T))
    # log:    φ(t) = 2π f0 · T/ln(r) · (r**(t/T) - 1),  r = f1/f0
    if kind == "linear":
        def phase(t):
            return 2.0 * math.pi * (f0 * t + (f1 - f0) * t * t / (2.0 * T))
    elif kind == "log":
        r = f1 / f0
        if abs(math.log(r)) < 1e-9:
            # degenerate: constant freq
            def phase(t):
                return 2.0 * math.pi * f0 * t
        else:
            coeff = 2.0 * math.pi * f0 * T / math.log(r)
            def phase(t):
                return coeff * ((r ** (t / T)) - 1.0)
    else:
        raise ValueError(f"sweep_type غير معروف: {kind}")

    def cmd_fn(t_s: float) -> float:
        if t_s <= 0:
            return 0.0
        if t_s >= T:
            return 0.0   # غلق آمن على صفر
        return amp * math.sin(phase(t_s))

    desc = (f"chirp {kind}: {f0:.2f}→{f1:.2f} Hz, "
            f"amp={amp:.1f}°, {T:.1f}s")
    return PatternSpec(duration_s=T, cmd_fn=cmd_fn, description=desc)
