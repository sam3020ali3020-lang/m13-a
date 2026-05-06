"""Signal generators for servo characterization tests.

All generators return a numpy array of shape (N,) with command-angle (deg)
samples at the requested control rate `fs_hz`.
"""

from __future__ import annotations

import numpy as np


def constant(value_deg: float, duration_s: float, fs_hz: float) -> np.ndarray:
    n = int(round(duration_s * fs_hz))
    return np.full(n, value_deg, dtype=float)


def step(start_deg: float, target_deg: float, t_step_s: float,
         duration_s: float, fs_hz: float) -> np.ndarray:
    n = int(round(duration_s * fs_hz))
    t = np.arange(n) / fs_hz
    out = np.where(t < t_step_s, start_deg, target_deg)
    return out.astype(float)


def staircase(levels_deg: list[float], hold_s: float, fs_hz: float) -> np.ndarray:
    n_per = int(round(hold_s * fs_hz))
    chunks = [np.full(n_per, lvl, dtype=float) for lvl in levels_deg]
    return np.concatenate(chunks) if chunks else np.zeros(0)


def square_wave(freq_hz: float, amplitude_deg: float,
                duration_s: float, fs_hz: float) -> np.ndarray:
    n = int(round(duration_s * fs_hz))
    t = np.arange(n) / fs_hz
    return amplitude_deg * np.sign(np.sin(2 * np.pi * freq_hz * t))


def linear_chirp(f_start_hz: float, f_end_hz: float, amplitude_deg: float,
                 duration_s: float, fs_hz: float) -> np.ndarray:
    """Linearly varying instantaneous frequency from f_start -> f_end."""
    n = int(round(duration_s * fs_hz))
    t = np.arange(n) / fs_hz
    k = (f_end_hz - f_start_hz) / max(duration_s, 1e-9)
    phase = 2 * np.pi * (f_start_hz * t + 0.5 * k * t * t)
    return amplitude_deg * np.sin(phase)


def log_chirp(f_start_hz: float, f_end_hz: float, amplitude_deg: float,
              duration_s: float, fs_hz: float) -> np.ndarray:
    """Exponentially varying frequency — better for Bode plots (equal log spacing)."""
    n = int(round(duration_s * fs_hz))
    t = np.arange(n) / fs_hz
    f_start = max(f_start_hz, 1e-3)
    L = np.log(f_end_hz / f_start) / max(duration_s, 1e-9)
    phase = 2 * np.pi * f_start * (np.exp(L * t) - 1) / L
    return amplitude_deg * np.sin(phase)


def slow_triangle(amplitude_deg: float, period_s: float,
                  duration_s: float, fs_hz: float) -> np.ndarray:
    """Slow up-then-down triangle for hysteresis measurement."""
    n = int(round(duration_s * fs_hz))
    t = np.arange(n) / fs_hz
    half = period_s / 2.0
    phase = (t % period_s) / half  # 0..2
    tri = np.where(phase <= 1, phase, 2 - phase)  # 0..1..0
    return amplitude_deg * (2 * tri - 1)


def slew_burst(amplitude_deg: float, duration_s: float, fs_hz: float) -> np.ndarray:
    """Hard +A -> -A reversals to expose slew-rate ceiling."""
    pattern = [+amplitude_deg, -amplitude_deg]
    n_per_seg = int(round(0.5 * fs_hz))   # 0.5 s per direction
    segs = []
    n_total = int(round(duration_s * fs_hz))
    i = 0
    while sum(len(s) for s in segs) < n_total:
        segs.append(np.full(n_per_seg, pattern[i % 2], dtype=float))
        i += 1
    out = np.concatenate(segs)[:n_total]
    return out


def prbs(amplitude_deg: float, n_bits: int, period_samples: int,
         duration_s: float, fs_hz: float, seed: int = 0xACE1) -> np.ndarray:
    """Pseudo-random binary sequence for system identification.

    n_bits: register length (e.g. 9 -> sequence period 511)
    period_samples: how many samples each bit is held for (sets dominant bandwidth)
    """
    n = int(round(duration_s * fs_hz))
    state = seed & ((1 << n_bits) - 1) or 1
    out = np.empty(n, dtype=float)
    bit_idx = 0
    sample_in_bit = 0
    current = +amplitude_deg if (state & 1) else -amplitude_deg
    for k in range(n):
        if sample_in_bit >= period_samples:
            # Galois LFSR feedback (taps depend on n_bits — use standard maximum-length)
            # For n_bits=9: taps at bits 5,9
            taps = {7: 0x60, 8: 0xB8, 9: 0x110, 10: 0x240, 11: 0x500}.get(n_bits, 0x110)
            lsb = state & 1
            state >>= 1
            if lsb:
                state ^= taps
            current = +amplitude_deg if (state & 1) else -amplitude_deg
            sample_in_bit = 0
            bit_idx += 1
        out[k] = current
        sample_in_bit += 1
    return out
