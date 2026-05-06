"""Post-processing & system identification for servo tests.

Reads the long-format CSVs produced by tester.py and fits:
  - First-order plus pure-delay model:  τ * dy/dt + y = K * u(t - Td)
  - Static gain, deadband, hysteresis, slew rate, noise floor
  - Frequency response (Bode) from chirp data via cross-spectral estimation
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
import pandas as pd
from scipy.optimize import least_squares
from scipy.signal import csd, welch


# ---------------------------------------------------------------------- IO

def load_test_csv(path: str, n_servos: int = 4) -> dict:
    """Load a long-format test CSV. Returns dict with per-slot cmd/fb arrays."""
    df = pd.read_csv(path)
    df["t_s"] = df["t_rel_ns"].astype(float) / 1e9

    cmd_df = df[df["kind"] == "CMD"]
    fb_df = df[df["kind"] == "FB"]

    out: dict = {"t_cmd": None, "fs_hz": None, "cmd": {}, "fb": {}, "fb_t": {}}
    for slot in range(n_servos):
        c = cmd_df[cmd_df["slot"] == slot].sort_values("t_s")
        if len(c) == 0:
            continue
        if out["t_cmd"] is None:
            out["t_cmd"] = c["t_s"].to_numpy()
            dt = np.median(np.diff(out["t_cmd"]))
            out["fs_hz"] = float(1.0 / dt) if dt > 0 else 0.0
        out["cmd"][slot] = c["value_deg"].to_numpy()

        f = fb_df[fb_df["slot"] == slot].sort_values("t_s")
        out["fb_t"][slot] = f["t_s"].to_numpy()
        out["fb"][slot] = f["value_deg"].to_numpy()
    return out


def resample_fb_to_cmd_grid(t_cmd: np.ndarray, t_fb: np.ndarray,
                             y_fb: np.ndarray) -> np.ndarray:
    """Linear interpolation of feedback samples onto command-time grid."""
    if len(t_fb) == 0:
        return np.zeros_like(t_cmd)
    return np.interp(t_cmd, t_fb, y_fb, left=y_fb[0], right=y_fb[-1])


# ---------------------------------------------------------------------- FOPDT model

def simulate_fopdt(u: np.ndarray, fs_hz: float, K: float, tau_s: float,
                   delay_s: float, y0: float = 0.0) -> np.ndarray:
    """Simulate y for τ ẏ + y = K · u(t - Td) with ZOH input."""
    n = len(u)
    dt = 1.0 / fs_hz
    delay_samples = int(round(max(delay_s, 0.0) * fs_hz))
    y = np.empty(n, dtype=float)
    y[0] = y0
    if tau_s <= 0:
        u_d = np.concatenate([np.full(delay_samples, u[0]), u])[:n]
        return K * u_d
    a = np.exp(-dt / tau_s)
    for k in range(1, n):
        idx = k - delay_samples
        u_eff = u[idx] if idx >= 0 else u[0]
        y[k] = a * y[k - 1] + (1 - a) * K * u_eff
    return y


@dataclass
class FOPDTFit:
    K: float
    tau_s: float
    delay_s: float
    rmse_deg: float
    n_samples: int


def fit_fopdt(u: np.ndarray, y: np.ndarray, fs_hz: float,
              tau_bounds: tuple[float, float] = (0.001, 0.5),
              delay_bounds: tuple[float, float] = (0.0, 0.5),
              K_bounds: tuple[float, float] = (0.5, 1.5)) -> FOPDTFit:
    """Joint fit of (K, τ, Td) by least-squares."""
    def residuals(params):
        log_tau, td, K = params
        tau = float(np.exp(log_tau))
        y_sim = simulate_fopdt(u, fs_hz, K, tau, td, y0=y[0])
        return y_sim - y

    x0 = [np.log(0.04), 0.10, 1.0]
    lo = [np.log(tau_bounds[0]), delay_bounds[0], K_bounds[0]]
    hi = [np.log(tau_bounds[1]), delay_bounds[1], K_bounds[1]]
    res = least_squares(residuals, x0, bounds=(lo, hi), method="trf",
                        max_nfev=400, xtol=1e-9, ftol=1e-9)
    log_tau, td, K = res.x
    tau = float(np.exp(log_tau))
    rmse = float(np.sqrt(np.mean(res.fun ** 2)))
    return FOPDTFit(K=float(K), tau_s=tau, delay_s=float(td),
                    rmse_deg=rmse, n_samples=len(u))


# ---------------------------------------------------------------------- step metrics

@dataclass
class StepMetrics:
    amplitude_deg: float
    rise_time_10_90_ms: float
    settling_time_5pct_ms: float
    overshoot_pct: float
    steady_state_error_deg: float
    delay_to_first_motion_ms: float


def step_metrics(t: np.ndarray, y: np.ndarray, t_step: float,
                 amplitude_deg: float, motion_threshold_deg: float = 0.5) -> StepMetrics:
    y0 = float(np.mean(y[t < t_step])) if np.any(t < t_step) else float(y[0])
    yf = float(np.mean(y[t > t[-1] - 0.2])) if t[-1] > 0.2 else float(y[-1])
    A = yf - y0
    if A == 0:
        A = amplitude_deg
    yn = (y - y0) / A

    motion_idx = np.where(np.abs(y - y0) > motion_threshold_deg)[0]
    delay_ms = float(((t[motion_idx[0]] - t_step) * 1000.0)) if len(motion_idx) else float("nan")

    try:
        i10 = np.where(yn >= 0.10)[0][0]
        i90 = np.where(yn >= 0.90)[0][0]
        rise_ms = float((t[i90] - t[i10]) * 1000.0)
    except IndexError:
        rise_ms = float("nan")

    band = 0.05 * abs(A)
    after = np.where(t >= t_step)[0]
    settling_ms = float("nan")
    if len(after):
        for j in after[::-1]:
            if abs(y[j] - yf) > band:
                settling_ms = float((t[j] - t_step) * 1000.0)
                break

    if A > 0:
        peak = float(np.max(y[t >= t_step])) if len(after) else y0
        overshoot = max(0.0, (peak - yf) / abs(A) * 100.0)
    else:
        peak = float(np.min(y[t >= t_step])) if len(after) else y0
        overshoot = max(0.0, (yf - peak) / abs(A) * 100.0)

    sse = float(yf - (y0 + amplitude_deg))
    return StepMetrics(amplitude_deg=amplitude_deg, rise_time_10_90_ms=rise_ms,
                       settling_time_5pct_ms=settling_ms, overshoot_pct=float(overshoot),
                       steady_state_error_deg=sse, delay_to_first_motion_ms=delay_ms)


# ---------------------------------------------------------------------- nonlinearity

def estimate_deadband(cmd: np.ndarray, fb: np.ndarray) -> float:
    """Smallest |cmd| level whose feedback exceeds 50% of cmd magnitude."""
    abs_levels = np.unique(np.round(np.abs(cmd[cmd != 0]), 2))
    abs_levels.sort()
    for lvl in abs_levels:
        mask = np.isclose(np.abs(cmd), lvl, atol=1e-3)
        if not np.any(mask):
            continue
        median_fb = float(np.median(np.abs(fb[mask])))
        if median_fb > 0.5 * lvl:
            return float(lvl)
    return float(abs_levels[-1]) if len(abs_levels) else float("nan")


def estimate_hysteresis(cmd: np.ndarray, fb: np.ndarray) -> float:
    """Triangle: max fb difference between up-going and down-going branch."""
    d = np.diff(cmd, prepend=cmd[0])
    bins = np.linspace(np.min(cmd), np.max(cmd), 21)
    hyst = []
    for i in range(len(bins) - 1):
        mask = (cmd >= bins[i]) & (cmd < bins[i + 1])
        up = mask & (d > 0)
        dn = mask & (d < 0)
        if np.any(up) and np.any(dn):
            hyst.append(abs(float(np.median(fb[up]) - np.median(fb[dn]))))
    return float(np.max(hyst)) if hyst else 0.0


def estimate_slew_rate(t: np.ndarray, fb: np.ndarray) -> float:
    if len(fb) < 5:
        return float("nan")
    dt = np.diff(t)
    dy = np.diff(fb)
    rate = np.abs(dy / np.maximum(dt, 1e-9))
    win = 5
    if len(rate) >= win:
        kernel = np.ones(win) / win
        smooth = np.convolve(rate, kernel, mode="valid")
        return float(np.max(smooth))
    return float(np.max(rate))


def estimate_noise_floor(fb: np.ndarray) -> float:
    if len(fb) < 8:
        return float("nan")
    return float(np.std(fb - np.mean(fb)))


# ---------------------------------------------------------------------- frequency response

@dataclass
class BodePoint:
    f_hz: float
    mag_db: float
    phase_deg: float
    coherence: float


def estimate_bode(u: np.ndarray, y: np.ndarray, fs_hz: float,
                  nperseg: Optional[int] = None) -> list[BodePoint]:
    n = min(len(u), len(y))
    if n < 64:
        return []
    if nperseg is None:
        nperseg = min(1024, n // 4)
    f, Suu = welch(u[:n], fs=fs_hz, nperseg=nperseg)
    _, Syy = welch(y[:n], fs=fs_hz, nperseg=nperseg)
    _, Suy = csd(u[:n], y[:n], fs=fs_hz, nperseg=nperseg)
    H = Suy / np.maximum(Suu, 1e-20)
    coh = np.abs(Suy) ** 2 / np.maximum(Suu * Syy, 1e-20)
    pts = []
    for fk, hk, ck in zip(f, H, coh):
        if fk <= 0 or ck < 0.5:
            continue
        mag_db = 20 * np.log10(np.maximum(np.abs(hk), 1e-12))
        phase_deg = np.degrees(np.angle(hk))
        pts.append(BodePoint(f_hz=float(fk), mag_db=float(mag_db),
                             phase_deg=float(phase_deg), coherence=float(ck)))
    return pts


def bandwidth_minus3db(bode_pts: list[BodePoint]) -> float:
    """Find -3 dB point (relative to low-freq mag). Returns Hz or NaN."""
    if len(bode_pts) < 4:
        return float("nan")
    pts = sorted(bode_pts, key=lambda p: p.f_hz)
    mag0 = float(np.median([p.mag_db for p in pts[:5]]))
    target = mag0 - 3.0
    for p in pts:
        if p.mag_db <= target:
            return float(p.f_hz)
    return float("nan")
