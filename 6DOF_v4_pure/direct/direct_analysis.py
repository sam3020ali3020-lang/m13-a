#!/usr/bin/env python3
"""/direct post-processing — fits, metrics, and Plotly HTML plots.

Consumes CSV from ``direct_runner.py`` and writes alongside it:
  - ``<stem>.metrics.txt``  — key-value metrics (delay, τ, OS, slew, backlash, BW)
  - ``<stem>.plot.html``    — interactive Plotly figure (time + optional bode/hysteresis)

Standalone use:
    python3 direct_analysis.py results/direct_step_YYYYMMDD_HHMMSS.csv
"""

from __future__ import annotations

import argparse
import re
import sys
from html import escape as html_escape
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import pandas as pd


# ─── Step-response metrics ──────────────────────────────────────────────────

def _find_step_edges(cmd: np.ndarray, threshold: float = 0.5) -> List[int]:
    d = np.abs(np.diff(cmd))
    return list(np.where(d > threshold)[0] + 1)


def _step_metrics(t: np.ndarray, cmd: np.ndarray, fb: np.ndarray,
                  edge_i: int, window_s: float = 1.5,
                  t_fb: Optional[np.ndarray] = None) -> Optional[Dict]:
    """
    قياسات step response.

    `t`     — وقت إرسال أمر loop (cmd time). يُستخدم لتحديد لحظة الـ edge.
    `t_fb`  — (اختياري) وقت وصول الـ fb الحقيقي (من CAN). إن مُرِّر يُستخدم
               لقياس delay/τ/settling بدقّة أعلى. خلاف ذلك يُستعمل `t`.
    """
    t0_abs = float(t[edge_i])
    # محور الزمن لقياسات fb: arrival time إن توفّر، وإلا loop time.
    t_for_fb = t_fb if t_fb is not None else t
    mask = (t_for_fb >= t0_abs) & (t_for_fb <= t0_abs + window_s)
    if mask.sum() < 5:
        return None
    ts = t_for_fb[mask]
    ys = fb[mask]
    y_init = float(fb[max(0, edge_i - 5):edge_i].mean()) if edge_i > 5 \
        else float(fb[edge_i])
    y_final = float(cmd[edge_i + 1]) if edge_i + 1 < len(cmd) \
        else float(cmd[edge_i])
    step = y_final - y_init
    if abs(step) < 1e-6:
        return None

    # transport delay: أول خروج خارج 5% من حجم الخطوة
    deadband = 0.05 * abs(step)
    moved = np.abs(ys - y_init) > deadband
    if not np.any(moved):
        return None
    i_move = int(np.argmax(moved))
    t_delay = float(ts[i_move] - t0_abs)

    # τ عند 63.2%
    target = y_init + 0.632 * step
    if step > 0:
        reached = ys >= target
    else:
        reached = ys <= target
    tau = float("nan")
    if np.any(reached):
        i63 = int(np.argmax(reached))
        tau = float(ts[i63] - ts[i_move])

    # overshoot
    peak = float(np.max(ys) if step > 0 else np.min(ys))
    overshoot = (peak - y_final) / step * 100.0

    # settling (±2%)
    tol = 0.02 * abs(step)
    settled = np.abs(ys - y_final) <= tol
    t_settle = float("nan")
    for i in range(len(ys)):
        if np.all(settled[i:]):
            t_settle = float(ts[i] - t0_abs)
            break

    return {
        "t_edge_s": t0_abs,
        "step_size_deg": step,
        "transport_delay_ms": t_delay * 1000.0,
        "tau_ms": (tau * 1000.0) if not np.isnan(tau) else float("nan"),
        "overshoot_pct": overshoot,
        "settling_ms": (t_settle * 1000.0) if not np.isnan(t_settle) else float("nan"),
        "final_err_deg": float(ys[-1] - y_final),
    }


def _slew_max(t: np.ndarray, fb: np.ndarray) -> float:
    """أقصى معدّل تغيّر (°/s) — robust ضد outliers بدون قتل saturation peaks.

    العتبة السابقة كانت median filter طول 7 سامبل، يَقتل peaks قصيرة حين
    يكون fb sampling خشناً (50Hz). عند step 25° وسرعة 490°/s، transit يستغرق
    ~5 سامبل فقط — median-7 يَطرحها.

    الحلّ: نستخدم p99 على raw rates بدون median. هذا يُزيل outlier واحد
    (noise spike) ويَحفظ saturation region لو استمرّت ≥1% من الـ samples.
    """
    if len(fb) < 3:
        return 0.0
    dy = np.diff(fb)
    dt = np.diff(t)
    # fb sampling 50Hz → dt طبيعي = 20ms. jitter يجعله أحياناً < 1ms →
    # rate خيالي. نَستبعد الـ pairs ذات dt قليل جدّاً.
    # servo rate حقيقي عند dt = 20ms (سامبل كامل واحد بين كلّ قياسين).
    min_dt = 0.010  # 10ms — نصف فترة أخذ العيّنة
    valid = dt >= min_dt
    if not np.any(valid):
        return 0.0
    rate = np.abs(dy[valid] / dt[valid])
    if len(rate) == 0:
        return 0.0
    # outlier rejection: استبعد أعلى 2 قيمتين (spike + glitch) ثمّ نأخذ الثالث.
    if len(rate) < 5:
        return float(rate.max())
    rate_sorted = np.sort(rate)[::-1]  # descending
    return float(rate_sorted[2])


# ─── Bode (chirp analysis) ──────────────────────────────────────────────────

def _bode(t: np.ndarray, cmd: np.ndarray, fb: np.ndarray,
          fs: float = 200.0) -> Optional[Dict]:
    if len(t) < 64 or t[-1] - t[0] < 1.0:
        return None
    n = int((t[-1] - t[0]) * fs)
    if n < 64:
        return None
    grid = t[0] + np.arange(n) / fs
    c = np.interp(grid, t, cmd)
    y = np.interp(grid, t, fb)
    c -= c.mean()
    y -= y.mean()
    w = np.hanning(n)
    C = np.fft.rfft(c * w)
    Y = np.fft.rfft(y * w)
    freq = np.fft.rfftfreq(n, d=1.0 / fs)
    mask = np.abs(C) > 0.01 * np.max(np.abs(C))
    mask[0] = False
    if not np.any(mask):
        return None
    H = Y[mask] / C[mask]
    mag_db = 20.0 * np.log10(np.abs(H) + 1e-12)
    phase_deg = np.unwrap(np.angle(H)) * 180.0 / np.pi
    f = freq[mask]
    # bandwidth = أول تردد تنخفض فيه magnitude بـ 3dB عن max
    bw = float("nan")
    peak_db = float(mag_db.max())
    below = np.where(mag_db <= peak_db - 3.0)[0]
    if len(below) > 0:
        bw = float(f[below[0]])
    return {
        "freq_hz": f, "mag_db": mag_db, "phase_deg": phase_deg,
        "bandwidth_hz": bw,
    }


# ─── Hysteresis / backlash ──────────────────────────────────────────────────

def _hysteresis(cmd: np.ndarray, fb: np.ndarray, n_bins: int = 40) -> Dict:
    if len(cmd) < 20:
        return {"backlash_deg": 0.0}
    dc = np.diff(cmd, prepend=cmd[0])
    up = dc > 1e-4
    dn = dc < -1e-4
    if not (np.any(up) and np.any(dn)):
        return {"backlash_deg": 0.0}
    lo, hi = float(cmd.min()), float(cmd.max())
    if hi - lo < 1e-3:
        return {"backlash_deg": 0.0}
    bins = np.linspace(lo, hi, n_bins + 1)
    idx = np.clip(np.digitize(cmd, bins) - 1, 0, n_bins - 1)
    up_fb = np.full(n_bins, np.nan)
    dn_fb = np.full(n_bins, np.nan)
    for b in range(n_bins):
        s_u = (idx == b) & up
        s_d = (idx == b) & dn
        if np.any(s_u):
            up_fb[b] = float(np.mean(fb[s_u]))
        if np.any(s_d):
            dn_fb[b] = float(np.mean(fb[s_d]))
    diff = up_fb - dn_fb
    diff = diff[~np.isnan(diff)]
    return {
        "backlash_deg": float(np.max(np.abs(diff))) if len(diff) else 0.0,
        "bin_centers": 0.5 * (bins[:-1] + bins[1:]),
        "up_fb": up_fb, "dn_fb": dn_fb,
    }


# ─── Plot builder ───────────────────────────────────────────────────────────

def _build_plot(servo_idx: int, node_id: str, t: np.ndarray,
                cmd: np.ndarray, fb: np.ndarray,
                bode: Optional[Dict], hys: Optional[Dict]):
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    rows = 1 + (1 if bode else 0) + (1 if hys else 0)
    titles = [f"servo#{servo_idx} ({node_id}) — time response"]
    if bode:
        titles.append("Bode — magnitude (dB) + phase (°)")
    if hys:
        titles.append("Hysteresis — fb vs cmd (up vs down)")
    specs = [[{"secondary_y": False}]]
    if bode:
        specs.append([{"secondary_y": True}])
    if hys:
        specs.append([{"secondary_y": False}])

    fig = make_subplots(rows=rows, cols=1, subplot_titles=titles,
                        vertical_spacing=0.10, specs=specs)

    # time
    fig.add_trace(go.Scatter(x=t, y=cmd, name="cmd",
                             line=dict(color="#d62728", width=1.5)), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=fb, name="fb",
                             line=dict(color="#1f77b4", width=1.5)), row=1, col=1)
    fig.update_xaxes(title_text="time [s]", row=1, col=1)
    fig.update_yaxes(title_text="angle [deg]", row=1, col=1)

    next_row = 2
    if bode:
        fig.add_trace(go.Scatter(
            x=bode["freq_hz"], y=bode["mag_db"], name="|H| dB",
            line=dict(color="#2ca02c")), row=next_row, col=1, secondary_y=False)
        fig.add_trace(go.Scatter(
            x=bode["freq_hz"], y=bode["phase_deg"], name="phase °",
            line=dict(color="#ff7f0e", dash="dot")),
            row=next_row, col=1, secondary_y=True)
        fig.update_xaxes(type="log", title_text="freq [Hz]",
                         row=next_row, col=1)
        fig.update_yaxes(title_text="magnitude [dB]",
                         row=next_row, col=1, secondary_y=False)
        fig.update_yaxes(title_text="phase [°]",
                         row=next_row, col=1, secondary_y=True)
        next_row += 1

    if hys:
        xs = hys["bin_centers"]
        fig.add_trace(go.Scatter(x=xs, y=hys["up_fb"], name="cmd↑ fb",
                                 line=dict(color="#9467bd")),
                      row=next_row, col=1)
        fig.add_trace(go.Scatter(x=xs, y=hys["dn_fb"], name="cmd↓ fb",
                                 line=dict(color="#8c564b", dash="dash")),
                      row=next_row, col=1)
        fig.update_xaxes(title_text="cmd [deg]", row=next_row, col=1)
        fig.update_yaxes(title_text="fb [deg]", row=next_row, col=1)

    fig.update_layout(
        height=320 * rows, width=1100,
        template="plotly_white",
        title=f"/direct — servo#{servo_idx} ({node_id})",
        hovermode="x unified",
    )
    return fig


# ─── Phase 1: per-cell breakdowns ────────────────────────────────────────────

def _edge_metrics(t: np.ndarray, fb: np.ndarray,
                  t0: float, y_init: float, y_final: float,
                  window_end: float) -> Optional[Dict[str, float]]:
    """
    قياسات edge واحد بمعطيات معروفة (لا حاجة للكشف).

    `t`           — مصفوفة الزمن (preferably t_fb_arrival_s)
    `fb`          — مصفوفة الـ fb
    `t0`          — لحظة إرسال الأمر (cmd edge)
    `y_init`      — قيمة fb المتوقعة قبل الـ edge
    `y_final`     — قيمة fb المستهدفة بعد الـ edge
    `window_end`  — حدّ النافذة الأعلى للزمن
    """
    mask = (t >= t0) & (t <= window_end)
    if mask.sum() < 3:
        return None
    ts = t[mask]
    ys = fb[mask]
    step = y_final - y_init
    if abs(step) < 1e-6:
        return None

    # transport delay: أول خروج خارج 5% من حجم الخطوة من y_init
    deadband = 0.05 * abs(step)
    moved = np.abs(ys - y_init) > deadband
    if not np.any(moved):
        return None
    i_move = int(np.argmax(moved))
    t_delay = float(ts[i_move] - t0)

    # τ عند 63.2%
    target = y_init + 0.632 * step
    if step > 0:
        reached = ys >= target
    else:
        reached = ys <= target
    tau = float("nan")
    if np.any(reached):
        i63 = int(np.argmax(reached))
        if i63 > i_move:
            tau = float(ts[i63] - ts[i_move])

    # overshoot (من y_final)
    if step > 0:
        peak = float(np.max(ys))
        overshoot_pct = (peak - y_final) / step * 100.0
    else:
        peak = float(np.min(ys))
        overshoot_pct = (peak - y_final) / step * 100.0

    # settling (±2% من حجم الخطوة)
    tol = 0.02 * abs(step)
    settled = np.abs(ys - y_final) <= tol
    t_settle = float("nan")
    for i in range(len(ys)):
        if np.all(settled[i:]):
            t_settle = float(ts[i] - t0)
            break

    return {
        "delay_ms": t_delay * 1000.0,
        "tau_ms": (tau * 1000.0) if not np.isnan(tau) else float("nan"),
        "overshoot_pct": overshoot_pct,
        "settling_ms": (t_settle * 1000.0) if not np.isnan(t_settle) else float("nan"),
        "final_err_deg": float(ys[-1] - y_final),
        "samples": int(mask.sum()),
    }


def _summarize(values: List[float], name: str) -> str:
    """mean ± std + min/p50/p95/max."""
    arr = np.array([v for v in values if not np.isnan(v)], dtype=float)
    if len(arr) == 0:
        return f"{name}: (no data)"
    mean = float(np.mean(arr))
    std = float(np.std(arr, ddof=1)) if len(arr) > 1 else 0.0
    return (f"{name}: mean={mean:7.2f}  std={std:6.2f}  "
            f"min={float(np.min(arr)):7.2f}  "
            f"p50={float(np.percentile(arr, 50)):7.2f}  "
            f"p95={float(np.percentile(arr, 95)):7.2f}  "
            f"max={float(np.max(arr)):7.2f}  N={len(arr)}")


def _matrix_breakdown(df: "pd.DataFrame", schedule: List[Dict],
                      pattern_name: str, csv_path: Path,
                      cfg: Optional[dict] = None):
    """Dispatch to per-pattern breakdown logic.

    Returns (lines, csv_path) — text lines + optional per-cell CSV path.
    """
    if pattern_name == "step_matrix":
        return _step_matrix_breakdown(df, schedule, csv_path)
    if pattern_name == "repeatability":
        return _repeatability_breakdown(df, schedule, csv_path)
    if pattern_name == "multi_servo":
        return _multi_servo_breakdown(df, schedule, csv_path)
    # ── Tier-1 validation ──
    if pattern_name == "linearity":
        return _linearity_breakdown(df, schedule, csv_path)
    if pattern_name == "hold_drift":
        return _hold_drift_breakdown(df, schedule, csv_path)
    if pattern_name == "rate_limit_verify":
        return _rate_limit_verify_breakdown(df, schedule, csv_path)
    # ── Tier-2 validation ──
    if pattern_name == "dead_band":
        return _dead_band_breakdown(df, schedule, csv_path)
    if pattern_name == "stiction":
        return _stiction_breakdown(df, schedule, csv_path)
    if pattern_name == "cold_start":
        return _cold_start_breakdown(df, schedule, csv_path)
    if pattern_name == "endurance":
        return _endurance_breakdown(df, schedule, csv_path)
    # ── Tier-3 cumulative-step diagnostics ──
    if pattern_name == "staircase":
        return _staircase_breakdown(df, schedule, csv_path)
    if pattern_name == "mech_limits":
        return _mech_limits_breakdown(df, schedule, csv_path)
    if pattern_name == "firmware_audit":
        return _firmware_audit_breakdown(df, schedule, csv_path)
    # ── Tier-4 fault-detection / pre-flight integrity ──
    if pattern_name == "preflight_check":
        return _preflight_check_breakdown(df, schedule, csv_path, cfg)
    if pattern_name == "wiring_audit":
        return _wiring_audit_breakdown(df, schedule, csv_path)
    if pattern_name == "fault_scan":
        return _fault_scan_breakdown(df, schedule, csv_path, cfg)
    return ([], None)


# ─── Tier-1 validation analyzers ────────────────────────────────────────────

def _linearity_breakdown(df, schedule: List[Dict], csv_path: Path):
    """Per-servo linear regression cmd → fb across the dwell points.

    لكلّ نقطة في schedule نأخذ متوسّط fb في آخر 60% من نافذة dwell (نتجنّب
    transient الـ step). ثمّ نَفت linear regression cmd vs fb_steady →
    نُخرج slope, intercept, R², max_residual.
    """
    lines: List[str] = []
    lines.append("─── linearity breakdown ──────────────────────────────")

    if not schedule:
        lines.append("  (no schedule)")
        return (lines, None)

    # Estimate dwell duration from schedule spacing
    if len(schedule) >= 2:
        dwell_s = float(schedule[1]["t_dwell_start_s"]
                         - schedule[0]["t_dwell_start_s"])
    else:
        dwell_s = 0.6
    settle_frac = 0.4   # ignore first 40% of dwell, average over last 60%

    # Auto-detect schedule ↔ CSV time offset (warm-up + safety pre-pad)
    target_servos = sorted(df["servo_idx"].unique())
    pat_t0 = 0.0
    if target_servos:
        first_cmd = float(schedule[0]["cmd_target"])
        first_t = float(schedule[0]["t_dwell_start_s"])
        for s in target_servos:
            off = _detect_pattern_offset(df, int(s), first_cmd, first_t)
            if off is not None:
                pat_t0 = off
                break
        lines.append(f"  pattern alignment: csv_t = sched_t + {pat_t0:.3f}s")

    rows: List[Dict] = []
    for sidx in sorted(df["servo_idx"].unique()):
        arrs = _per_servo_arrays(df, int(sidx))
        if arrs is None:
            continue
        t, _cmd, fb = arrs
        cmds: List[float] = []
        fbs: List[float] = []
        for ev in schedule:
            t0 = float(ev["t_dwell_start_s"]) + pat_t0 + settle_frac * dwell_s
            t1 = float(ev["t_dwell_start_s"]) + pat_t0 + dwell_s
            mask = (t >= t0) & (t <= t1)
            if mask.sum() < 2:
                continue
            cmds.append(float(ev["cmd_target"]))
            fbs.append(float(np.mean(fb[mask])))
        if len(cmds) < 3:
            continue
        cmd_arr = np.asarray(cmds)
        fb_arr = np.asarray(fbs)
        # Fit fb = slope * cmd + intercept
        slope, intercept = np.polyfit(cmd_arr, fb_arr, 1)
        pred = slope * cmd_arr + intercept
        residuals = fb_arr - pred
        ss_res = float(np.sum(residuals ** 2))
        ss_tot = float(np.sum((fb_arr - np.mean(fb_arr)) ** 2))
        r2 = 1.0 - ss_res / max(ss_tot, 1e-12)
        max_dev = float(np.max(np.abs(residuals)))

        verdict = (
            "OK"
            if (abs(slope - 1.0) < 0.05 and abs(intercept) < 0.5
                and r2 > 0.999 and max_dev < 0.5)
            else "REVIEW"
        )
        lines.append(f"  servo#{sidx}: slope={slope:+.4f}  "
                     f"intercept={intercept:+.3f}°  R²={r2:.5f}  "
                     f"max_dev={max_dev:.3f}°  N={len(cmds)}  [{verdict}]")
        rows.append({
            "servo_idx": int(sidx), "slope": slope, "intercept_deg": intercept,
            "r2": r2, "max_dev_deg": max_dev, "n_points": len(cmds),
            "verdict": verdict,
        })

    out_csv: Optional[Path] = None
    if rows:
        try:
            import pandas as pd
            out_csv = csv_path.with_suffix("").parent / (
                csv_path.stem + ".linearity.csv"
            )
            pd.DataFrame(rows).to_csv(out_csv, index=False)
        except Exception:
            out_csv = None
    return (lines, out_csv)


def _hold_drift_breakdown(df, schedule: List[Dict], csv_path: Path):
    """Per-position drift slope (deg/s) over the hold window per servo.

    لكلّ موقع في schedule نَستخرج fb داخل [t_hold_start, t_hold_end]
    ونَفت linear regression fb(t). slope = drift_deg_per_s.
    """
    lines: List[str] = []
    lines.append("─── hold_drift breakdown ─────────────────────────────")
    if not schedule:
        lines.append("  (no schedule)")
        return (lines, None)

    # Auto-detect schedule ↔ CSV offset
    target_servos = sorted(df["servo_idx"].unique())
    pat_t0 = 0.0
    if target_servos:
        first_cmd = float(schedule[0]["cmd_target"])
        first_t = float(schedule[0]["t_step_s"])
        for s in target_servos:
            off = _detect_pattern_offset(df, int(s), first_cmd, first_t)
            if off is not None:
                pat_t0 = off
                break
        lines.append(f"  pattern alignment: csv_t = sched_t + {pat_t0:.3f}s")

    rows: List[Dict] = []
    for sidx in sorted(df["servo_idx"].unique()):
        arrs = _per_servo_arrays(df, int(sidx))
        if arrs is None:
            continue
        t, _cmd, fb = arrs
        for ev in schedule:
            t0 = float(ev["t_hold_start_s"]) + pat_t0
            t1 = float(ev["t_hold_end_s"]) + pat_t0
            target = float(ev["cmd_target"])
            mask = (t >= t0) & (t <= t1)
            n = int(mask.sum())
            if n < 5:
                continue
            ts = t[mask] - t0
            ys = fb[mask]
            slope, intercept = np.polyfit(ts, ys, 1)
            mean_fb = float(np.mean(ys))
            std_fb = float(np.std(ys, ddof=1))
            err = mean_fb - target
            verdict = "OK" if abs(slope) < 0.02 and std_fb < 0.10 else "REVIEW"
            lines.append(
                f"  servo#{sidx} @cmd={target:+6.1f}°: "
                f"mean_fb={mean_fb:+7.3f}° err={err:+.3f}° "
                f"drift={slope*1000:+.2f}m°/s std={std_fb:.3f}° "
                f"N={n}  [{verdict}]"
            )
            rows.append({
                "servo_idx": int(sidx), "cmd_target_deg": target,
                "mean_fb_deg": mean_fb, "err_deg": err,
                "drift_deg_per_s": slope, "drift_mdeg_per_s": slope * 1000,
                "std_deg": std_fb, "n_samples": n, "verdict": verdict,
            })

    out_csv: Optional[Path] = None
    if rows:
        try:
            import pandas as pd
            out_csv = csv_path.with_suffix("").parent / (
                csv_path.stem + ".hold_drift.csv"
            )
            pd.DataFrame(rows).to_csv(out_csv, index=False)
        except Exception:
            out_csv = None
    return (lines, out_csv)


def _rate_limit_verify_breakdown(df, schedule: List[Dict], csv_path: Path):
    """Peak slew rate per (servo, amplitude). Detect saturation onset.

    لكلّ edge في schedule نأخذ fb في النافذة بعد الـ edge ونَحسب أقصى
    |Δfb/Δt| (مع outlier rejection). نَجمع per-amplitude المتوسّط ونُلاحظ
    عند أيّ amplitude تَصِل القيمة إلى plateau (rate saturation).
    """
    lines: List[str] = []
    lines.append("─── rate_limit_verify breakdown ──────────────────────")
    if not schedule:
        lines.append("  (no schedule)")
        return (lines, None)

    # Auto-detect schedule ↔ CSV offset
    target_servos = sorted(df["servo_idx"].unique())
    pat_t0 = 0.0
    if target_servos:
        first_cmd = float(schedule[0]["cmd_to"])
        first_t = float(schedule[0]["t_edge_s"])
        for s in target_servos:
            off = _detect_pattern_offset(df, int(s), first_cmd, first_t)
            if off is not None:
                pat_t0 = off
                break
        lines.append(f"  pattern alignment: csv_t = sched_t + {pat_t0:.3f}s")

    # window بعد الـ edge — يَكفي 200ms لرَصْد الـ peak slew
    win_s = 0.20
    rows: List[Dict] = []
    for sidx in sorted(df["servo_idx"].unique()):
        arrs = _per_servo_arrays(df, int(sidx))
        if arrs is None:
            continue
        t, _cmd, fb = arrs
        per_amp: Dict[float, List[float]] = {}
        for ev in schedule:
            t_edge = float(ev["t_edge_s"]) + pat_t0
            amp = float(ev["amp_deg"])
            mask = (t >= t_edge) & (t <= t_edge + win_s)
            if mask.sum() < 3:
                continue
            ts = t[mask]
            ys = fb[mask]
            # إزالة duplicates
            tu, idxu = np.unique(ts, return_index=True)
            yu = ys[idxu]
            if len(tu) < 3:
                continue
            dy = np.diff(yu)
            dt = np.diff(tu)
            with np.errstate(divide="ignore", invalid="ignore"):
                slew = np.abs(dy / np.where(dt > 1e-6, dt, np.nan))
            slew = slew[np.isfinite(slew)]
            if len(slew) == 0:
                continue
            # outlier rejection: 95th percentile كحدّ أعلى
            peak = float(np.percentile(slew, 95))
            per_amp.setdefault(amp, []).append(peak)
        for amp in sorted(per_amp):
            vals = per_amp[amp]
            mean_pk = float(np.mean(vals))
            std_pk = float(np.std(vals, ddof=1)) if len(vals) > 1 else 0.0
            lines.append(
                f"  servo#{sidx} amp={amp:5.1f}°: "
                f"peak_slew mean={mean_pk:6.1f}°/s std={std_pk:5.1f}°/s "
                f"N={len(vals)}"
            )
            rows.append({
                "servo_idx": int(sidx), "amp_deg": amp,
                "peak_slew_mean_dps": mean_pk, "peak_slew_std_dps": std_pk,
                "n_edges": len(vals),
            })

    out_csv: Optional[Path] = None
    if rows:
        try:
            import pandas as pd
            out_csv = csv_path.with_suffix("").parent / (
                csv_path.stem + ".rate_limit.csv"
            )
            pd.DataFrame(rows).to_csv(out_csv, index=False)
        except Exception:
            out_csv = None
    return (lines, out_csv)


# ─── Tier-2 validation analyzers ────────────────────────────────────────────

def _dead_band_breakdown(df, schedule: List[Dict], csv_path: Path):
    """يَكتشف أصغر amplitude يَجعل |Δfb| > noise_threshold.

    لكلّ schedule entry: قِس fb_pre (داخل t_baseline window) و fb_post (في
    آخر نصف dwell). Δfb = mean(post) − mean(pre). نُجَمّع per-amplitude.
    threshold = 2 × std(idle) ≈ 0.05° (أو 1 encoder bin).
    """
    lines: List[str] = []
    lines.append("─── dead_band breakdown ──────────────────────────────")
    if not schedule:
        lines.append("  (no schedule)")
        return (lines, None)

    target_servos = sorted(df["servo_idx"].unique())
    pat_t0 = 0.0
    if target_servos:
        first_cmd = float(schedule[0]["cmd_target"])
        first_t = float(schedule[0]["t_step_s"])
        for s in target_servos:
            off = _detect_pattern_offset(df, int(s), first_cmd, first_t)
            if off is not None:
                pat_t0 = off
                break
        lines.append(f"  pattern alignment: csv_t = sched_t + {pat_t0:.3f}s")

    rows: List[Dict] = []
    for sidx in target_servos:
        arrs = _per_servo_arrays(df, int(sidx))
        if arrs is None:
            continue
        t, _cmd, fb = arrs
        per_amp: Dict[float, List[float]] = {}
        for ev in schedule:
            t_pre0 = float(ev["t_baseline_start_s"]) + pat_t0
            t_pre1 = float(ev["t_baseline_end_s"]) + pat_t0
            t_post0 = float(ev["t_step_s"]) + pat_t0 + 0.5 * (
                float(ev["t_dwell_end_s"]) - float(ev["t_step_s"]))
            t_post1 = float(ev["t_dwell_end_s"]) + pat_t0
            mask_pre = (t >= t_pre0) & (t <= t_pre1)
            mask_post = (t >= t_post0) & (t <= t_post1)
            if mask_pre.sum() < 2 or mask_post.sum() < 2:
                continue
            pre = float(np.mean(fb[mask_pre]))
            post = float(np.mean(fb[mask_post]))
            d_fb = post - pre
            amp = float(ev["amp_deg"])
            d_signed = d_fb if ev["direction"] == "up" else -d_fb
            per_amp.setdefault(amp, []).append(d_signed)
        if not per_amp:
            continue
        # threshold from smallest-amp std (proxy for noise)
        smallest = min(per_amp)
        noise = float(np.std(per_amp[smallest], ddof=1)) if len(
            per_amp[smallest]) > 1 else 0.05
        threshold = max(0.05, 2.0 * noise)
        dead_amp = None
        for amp in sorted(per_amp):
            mean_d = float(np.mean(per_amp[amp]))
            std_d = float(np.std(per_amp[amp], ddof=1)) if len(
                per_amp[amp]) > 1 else 0.0
            verdict = "ABOVE" if mean_d > threshold else "in-band"
            lines.append(f"  servo#{sidx} amp={amp:.3f}°: "
                         f"Δfb_mean={mean_d:+.4f}°  std={std_d:.4f}°  "
                         f"N={len(per_amp[amp])}  [{verdict}]")
            if dead_amp is None and mean_d > threshold:
                dead_amp = amp
            rows.append({
                "servo_idx": int(sidx), "amp_deg": amp,
                "delta_fb_mean": mean_d, "delta_fb_std": std_d,
                "n": len(per_amp[amp]), "verdict": verdict,
            })
        if dead_amp is not None:
            lines.append(f"  → servo#{sidx} dead_band ≈ {dead_amp:.3f}° "
                         f"(threshold={threshold:.3f}°)")
        else:
            lines.append(f"  → servo#{sidx} dead_band > {max(per_amp):.3f}° "
                         f"(غير مَكشوف ضمن النطاق)")

    out_csv: Optional[Path] = None
    if rows:
        try:
            import pandas as pd
            out_csv = csv_path.with_suffix("").parent / (
                csv_path.stem + ".dead_band.csv")
            pd.DataFrame(rows).to_csv(out_csv, index=False)
        except Exception:
            out_csv = None
    return (lines, out_csv)


def _stiction_breakdown(df, schedule: List[Dict], csv_path: Path):
    """يَكشف breakaway: متى يَبدأ fb بالتَحرّك بعد بدء الـ ramp.

    لكلّ ramp في schedule: نَأخذ fb داخل النافذة، نَحسب lag أوّل تَحرّك > 0.05°
    من قيمة الـ baseline. lag الكَبير = stiction أكبر.
    """
    lines: List[str] = []
    lines.append("─── stiction breakdown ───────────────────────────────")
    if not schedule:
        lines.append("  (no schedule)")
        return (lines, None)

    target_servos = sorted(df["servo_idx"].unique())
    pat_t0 = 0.0
    if target_servos:
        first_cmd = float(schedule[0]["cmd_end"])
        first_t = float(schedule[0]["t_ramp_start_s"])
        for s in target_servos:
            off = _detect_pattern_offset(df, int(s), first_cmd, first_t)
            if off is not None:
                pat_t0 = off
                break
        lines.append(f"  pattern alignment: csv_t = sched_t + {pat_t0:.3f}s")

    rows: List[Dict] = []
    for sidx in target_servos:
        arrs = _per_servo_arrays(df, int(sidx))
        if arrs is None:
            continue
        t, _cmd, fb = arrs
        for ev in schedule:
            t_ramp_start = float(ev["t_ramp_start_s"]) + pat_t0
            t_ramp_end = float(ev["t_ramp_end_s"]) + pat_t0
            c_start = float(ev["cmd_start"])
            c_end = float(ev["cmd_end"])
            mask = (t >= t_ramp_start - 0.5) & (t <= t_ramp_end + 0.5)
            if mask.sum() < 5:
                continue
            ts = t[mask] - t_ramp_start
            ys = fb[mask]
            # baseline = fb قَبل ramp start
            pre_mask = ts < 0
            if pre_mask.sum() < 2:
                base = ys[0]
            else:
                base = float(np.mean(ys[pre_mask]))
            # find first sample where |y - base| > 0.05° (= 1 encoder bin tolerance)
            in_ramp = ts >= 0
            if in_ramp.sum() < 2:
                continue
            ts_r = ts[in_ramp]
            ys_r = ys[in_ramp]
            move_thresh = 0.05
            move_idx = np.where(np.abs(ys_r - base) > move_thresh)[0]
            if len(move_idx) == 0:
                lag = float("nan")
            else:
                lag = float(ts_r[move_idx[0]])
            direction = ev["direction"]
            lines.append(
                f"  servo#{sidx} {direction:18s} c={c_start:+.2f}→{c_end:+.2f}°  "
                f"breakaway_lag={lag*1000 if not np.isnan(lag) else float('nan'):.1f}ms"
            )
            rows.append({
                "servo_idx": int(sidx), "direction": direction,
                "cmd_start": c_start, "cmd_end": c_end,
                "breakaway_lag_s": lag,
            })

    out_csv: Optional[Path] = None
    if rows:
        try:
            import pandas as pd
            out_csv = csv_path.with_suffix("").parent / (
                csv_path.stem + ".stiction.csv")
            pd.DataFrame(rows).to_csv(out_csv, index=False)
        except Exception:
            out_csv = None
    return (lines, out_csv)


def _staircase_breakdown(df, schedule: List[Dict], csv_path: Path):
    """يَكشف stalls المُتَقَطِّعَة في خَطَوات تَراكُميَّة.

    لكلّ خَطوَة في schedule:
      fb_pre  = mean(fb في آخر 0.2s قَبل t_step_s)        # قَبل القَفزَة
      fb_post = mean(fb في آخر 60% من dwell)               # بَعد الاستِقرار
      Δfb_obs = fb_post − fb_pre
      Δcmd    = cmd_target − cmd_prev                       # = step_deg ±

    خَطوَة "stalled" إذا |Δfb_obs| < 0.5 × |Δcmd| (أي السيرفو لَم يَتَحَرَّك إلّا
    أَقَلّ مِن نِصف ما طُلِب).

    لكلّ سيرفو نُخرِج:
      - عَدَد الـ stalls
      - عَدَد الخَطَوات الكَلّيَّة
      - مَوقِع stalls (zone: مُوجَب/سالِب) ومُتَوَسِّط cmd عِندَها
      - أَقصى |error_steady_state| (الفَرق بَين cmd_target و fb_post)
    """
    lines: List[str] = []
    lines.append("─── staircase breakdown ──────────────────────────────")
    if not schedule:
        lines.append("  (no schedule)")
        return (lines, None)

    target_servos = sorted(df["servo_idx"].unique())
    pat_t0 = 0.0
    if target_servos:
        first_cmd = float(schedule[0]["cmd_target"])
        first_t = float(schedule[0]["t_step_s"])
        for s in target_servos:
            off = _detect_pattern_offset(df, int(s), first_cmd, first_t)
            if off is not None:
                pat_t0 = off
                break
        lines.append(f"  pattern alignment: csv_t = sched_t + {pat_t0:.3f}s")

    # نِسبَة العَتبَة لِاعتِبار الخَطوَة "stalled"
    STALL_FRAC = 0.5
    # عَتبَة دَنيا (لِتَجَنُّب false positives على خَطَوات صَغيرَة جِدّاً)
    STALL_MIN_DEG = 0.2

    rows: List[Dict] = []
    for sidx in target_servos:
        arrs = _per_servo_arrays(df, int(sidx))
        if arrs is None:
            continue
        t, _cmd, fb = arrs

        n_total = 0
        n_stall = 0
        max_ss_err = 0.0
        worst_stall: Optional[Dict] = None
        # نُجَمِّع stalls حَسَب الـ phase
        per_phase: Dict[str, int] = {}

        for ev in schedule:
            t_step = float(ev["t_step_s"]) + pat_t0
            t_dwell_end = float(ev["t_dwell_end_s"]) + pat_t0
            cmd_target = float(ev["cmd_target"])
            delta_cmd = float(ev["delta_deg"])
            phase = str(ev.get("phase", ""))

            # نافِذَة fb_pre: 0.2s قَبل t_step
            mask_pre = (t >= t_step - 0.2) & (t < t_step)
            # نافِذَة fb_post: آخر 60% مِن dwell
            t_post0 = t_step + 0.4 * (t_dwell_end - t_step)
            mask_post = (t >= t_post0) & (t <= t_dwell_end)
            if mask_pre.sum() < 2 or mask_post.sum() < 2:
                continue

            fb_pre = float(np.mean(fb[mask_pre]))
            fb_post = float(np.mean(fb[mask_post]))
            d_fb = fb_post - fb_pre
            ss_err = fb_post - cmd_target

            n_total += 1
            stalled = False
            if abs(delta_cmd) >= STALL_MIN_DEG:
                # الإشارَة يَجِب أن تُطابِق
                same_dir = (d_fb * delta_cmd) > 0
                small_motion = abs(d_fb) < STALL_FRAC * abs(delta_cmd)
                stalled = (not same_dir) or small_motion
                if stalled:
                    n_stall += 1
                    per_phase[phase] = per_phase.get(phase, 0) + 1
                    if (worst_stall is None or
                            abs(ss_err) > abs(worst_stall["ss_err"])):
                        worst_stall = {
                            "phase": phase, "cmd_target": cmd_target,
                            "cmd_prev": float(ev["cmd_prev"]),
                            "delta_cmd": delta_cmd, "d_fb": d_fb,
                            "ss_err": ss_err,
                        }

            if abs(ss_err) > abs(max_ss_err):
                max_ss_err = ss_err

            rows.append({
                "servo_idx": int(sidx),
                "phase": phase,
                "cycle": int(ev.get("cycle", 0)),
                "step_idx": int(ev.get("step_idx", 0)),
                "cmd_prev": float(ev["cmd_prev"]),
                "cmd_target": cmd_target,
                "delta_cmd_deg": delta_cmd,
                "fb_pre": fb_pre,
                "fb_post": fb_post,
                "d_fb_deg": d_fb,
                "ss_err_deg": ss_err,
                "stalled": stalled,
            })

        # طَباعَة مُلَخَّص السيرفو
        stall_pct = (100.0 * n_stall / n_total) if n_total else 0.0
        verdict = "✅ نظيف" if n_stall == 0 else (
            "⚠️ stalls قليلة" if stall_pct < 5.0 else "🔴 stalls كثيرة")
        lines.append(
            f"  servo#{sidx}: stalls={n_stall}/{n_total} "
            f"({stall_pct:.1f}%)  max_ss_err={max_ss_err:+.3f}°  [{verdict}]"
        )
        if per_phase:
            phs = ", ".join(f"{k}:{v}" for k, v in sorted(per_phase.items()))
            lines.append(f"      stalls per phase: {phs}")
        if worst_stall is not None:
            lines.append(
                f"      worst stall: {worst_stall['phase']} "
                f"cmd {worst_stall['cmd_prev']:+.1f}→{worst_stall['cmd_target']:+.1f}°  "
                f"Δcmd={worst_stall['delta_cmd']:+.2f}°  "
                f"Δfb={worst_stall['d_fb']:+.3f}°  "
                f"ss_err={worst_stall['ss_err']:+.3f}°"
            )

    out_csv: Optional[Path] = None
    if rows:
        try:
            import pandas as pd
            out_csv = csv_path.with_suffix("").parent / (
                csv_path.stem + ".staircase.csv")
            pd.DataFrame(rows).to_csv(out_csv, index=False)
        except Exception:
            out_csv = None
    return (lines, out_csv)


def _mech_limits_breakdown(df, schedule: List[Dict], csv_path: Path):
    """يَستَخرِج الحُدود الميكانيكيَّة الفِعليَّة لكلّ سيرفو.

    في كلّ leg (pos_climb / neg_climb):
      - نَتَتَبَّع fb_post عِند كلّ cmd_target مُتَزايِد
      - الحَدّ الفِعلي = أَقصى fb_post وَصَل إليه السيرفو
      - إذا fb_post لَم يَتَقَدَّم عَن الخَطوَة السابِقَة بـ MIN_PROGRESS،
        نَعتَبِرُه ضَرَبَ الحَدّ

    نُخرِج لكلّ سيرفو:
      pos_limit   = أَقصى fb مُوجَب مُحَقَّق
      neg_limit   = أَقصى fb سالِب مُحَقَّق (= أَصغَر قيمة fb)
      pos_cmd     = cmd الَّذي حَقَّق pos_limit
      neg_cmd     = cmd الَّذي حَقَّق neg_limit
      travel      = pos_limit - neg_limit
      midpoint    = (pos_limit + neg_limit) / 2  (= zero offset)
      asymmetry   = pos_limit - |neg_limit|
    """
    lines: List[str] = []
    lines.append("─── mech_limits breakdown ────────────────────────────")
    if not schedule:
        lines.append("  (no schedule)")
        return (lines, None)

    # عَتبَة التَّقَدُّم الدَّنيا لاعتِبار السيرفو "لا يَزال يَتَحَرَّك"
    MIN_PROGRESS_DEG = 1.0

    target_servos = sorted(df["servo_idx"].unique())
    pat_t0 = 0.0
    if target_servos:
        first_cmd = float(schedule[0]["cmd_target"])
        first_t = float(schedule[0]["t_step_s"])
        for s in target_servos:
            off = _detect_pattern_offset(df, int(s), first_cmd, first_t)
            if off is not None:
                pat_t0 = off
                break
        lines.append(f"  pattern alignment: csv_t = sched_t + {pat_t0:.3f}s")

    rows: List[Dict] = []
    summary_rows: List[Dict] = []
    for sidx in target_servos:
        arrs = _per_servo_arrays(df, int(sidx))
        if arrs is None:
            continue
        t, _cmd, fb = arrs

        pos_limit = 0.0
        pos_cmd_at_limit = 0.0
        neg_limit = 0.0
        neg_cmd_at_limit = 0.0

        # نُعالِج فَقَط "climb" phases (عَن الصِّفر باتِّجاه ±max)
        last_fb_per_dir: Dict[str, float] = {"pos": 0.0, "neg": 0.0}
        hit_limit: Dict[str, bool] = {"pos": False, "neg": False}

        for ev in schedule:
            phase = str(ev.get("phase", ""))
            direction = str(ev.get("direction", ""))
            t_step = float(ev["t_step_s"]) + pat_t0
            t_dwell_end = float(ev["t_dwell_end_s"]) + pat_t0
            cmd_target = float(ev["cmd_target"])

            # نافِذَة fb_post: آخِر 60% مِن dwell
            t_post0 = t_step + 0.4 * (t_dwell_end - t_step)
            mask_post = (t >= t_post0) & (t <= t_dwell_end)
            if mask_post.sum() < 2:
                continue
            fb_post = float(np.mean(fb[mask_post]))

            is_climb = phase.endswith("_climb")
            if is_climb and direction in ("pos", "neg"):
                prev_fb = last_fb_per_dir[direction]
                delta_fb = fb_post - prev_fb
                is_progress = (
                    (direction == "pos" and delta_fb > MIN_PROGRESS_DEG) or
                    (direction == "neg" and delta_fb < -MIN_PROGRESS_DEG)
                )
                if is_progress and not hit_limit[direction]:
                    if direction == "pos":
                        pos_limit = fb_post
                        pos_cmd_at_limit = cmd_target
                    else:
                        neg_limit = fb_post
                        neg_cmd_at_limit = cmd_target
                else:
                    # وَصَلنا الحَدّ في هذا الاتِّجاه
                    hit_limit[direction] = True
                last_fb_per_dir[direction] = fb_post

            rows.append({
                "servo_idx": int(sidx),
                "phase": phase,
                "direction": direction,
                "step_idx": int(ev.get("step_idx", 0)),
                "cmd_target": cmd_target,
                "fb_post": fb_post,
            })

        # مُلَخَّص السيرفو
        travel = pos_limit - neg_limit
        midpoint = (pos_limit + neg_limit) / 2.0
        asymmetry = pos_limit - abs(neg_limit)
        # تَقييم بَصَري
        offset_verdict = "✅" if abs(midpoint) < 1.0 else (
            "⚠️" if abs(midpoint) < 3.0 else "🔴")
        asym_verdict = "✅" if abs(asymmetry) < 2.0 else (
            "⚠️" if abs(asymmetry) < 5.0 else "🔴")

        lines.append(
            f"  servo#{sidx}:  pos={pos_limit:+6.2f}° (cmd{pos_cmd_at_limit:+.0f})  "
            f"neg={neg_limit:+6.2f}° (cmd{neg_cmd_at_limit:+.0f})"
        )
        lines.append(
            f"            travel={travel:6.2f}°  "
            f"midpoint={midpoint:+.2f}° {offset_verdict}  "
            f"asymmetry={asymmetry:+.2f}° {asym_verdict}"
        )

        summary_rows.append({
            "servo_idx": int(sidx),
            "pos_limit_deg": pos_limit,
            "pos_cmd_at_limit": pos_cmd_at_limit,
            "neg_limit_deg": neg_limit,
            "neg_cmd_at_limit": neg_cmd_at_limit,
            "total_travel_deg": travel,
            "midpoint_deg": midpoint,
            "asymmetry_deg": asymmetry,
        })

    # مُلَخَّص نِهائي بَصَري
    if summary_rows:
        lines.append("")
        lines.append("  ─── النَّتائج النِّهائيَّة ───")
        lines.append(
            "  servo    pos       neg     travel   midpoint  asymmetry"
        )
        for r in summary_rows:
            lines.append(
                f"  #{r['servo_idx']}    {r['pos_limit_deg']:+6.2f}°  "
                f"{r['neg_limit_deg']:+6.2f}°  {r['total_travel_deg']:6.2f}°  "
                f"{r['midpoint_deg']:+7.2f}°  {r['asymmetry_deg']:+7.2f}°"
            )

    out_csv: Optional[Path] = None
    if rows:
        try:
            import pandas as pd
            out_csv = csv_path.with_suffix("").parent / (
                csv_path.stem + ".mech_limits.csv")
            pd.DataFrame(rows).to_csv(out_csv, index=False)
            # مُلَخَّص مُنفَصِل
            summary_csv = csv_path.with_suffix("").parent / (
                csv_path.stem + ".mech_limits_summary.csv")
            pd.DataFrame(summary_rows).to_csv(summary_csv, index=False)
        except Exception:
            out_csv = None
    return (lines, out_csv)


def _firmware_audit_breakdown(df, schedule: List[Dict], csv_path: Path):
    """يُقارِن سُلوك firmware لكلّ السيرفوهات لِنَفس قائِمَة الأَوامِر.

    لكلّ خَطوَة pos_test/neg_test في schedule:
      - يَستَخرِج fb_post (آخِر 60% مِن dwell) لكلّ سيرفو
      - يَحسِب err = fb_post - cmd_target
      - يُحَدِّد إن كان السيرفو "تابِع" (|err| < 1°) أَم "مَحدود" (|err| ≥ 1°)

    يُخرِج جَدوَل مُقارَنَة بَين السيرفوهات + saturation per direction.
    """
    lines: List[str] = []
    lines.append("─── firmware_audit breakdown ─────────────────────────")
    if not schedule:
        lines.append("  (no schedule)")
        return (lines, None)

    target_servos = sorted(df["servo_idx"].unique())
    pat_t0 = 0.0
    if target_servos:
        # نَأخذ أَوَّل خَطوَة pos_test أَو أَيّ غَير zero_between
        first_active = next(
            (e for e in schedule if not e["phase"].startswith("zero")),
            schedule[0],
        )
        first_cmd = float(first_active["cmd_target"])
        first_t = float(first_active["t_step_s"])
        for s in target_servos:
            off = _detect_pattern_offset(df, int(s), first_cmd, first_t)
            if off is not None:
                pat_t0 = off
                break
        lines.append(f"  pattern alignment: csv_t = sched_t + {pat_t0:.3f}s")

    # نَجمَع نَتائج لكلّ سيرفو لكلّ خَطوَة test
    rows: List[Dict] = []
    per_servo_max_pos: Dict[int, float] = {s: 0.0 for s in target_servos}
    per_servo_max_neg: Dict[int, float] = {s: 0.0 for s in target_servos}
    per_servo_first_sat_pos: Dict[int, Optional[float]] = {
        s: None for s in target_servos}
    per_servo_first_sat_neg: Dict[int, Optional[float]] = {
        s: None for s in target_servos}

    test_events = [e for e in schedule
                   if e["phase"] in ("pos_test", "neg_test")]

    for sidx in target_servos:
        arrs = _per_servo_arrays(df, int(sidx))
        if arrs is None:
            continue
        t, _cmd, fb = arrs

        for ev in test_events:
            t_step = float(ev["t_step_s"]) + pat_t0
            t_dwell_end = float(ev["t_dwell_end_s"]) + pat_t0
            cmd_target = float(ev["cmd_target"])
            phase = ev["phase"]

            # نافِذَة fb_post: آخِر 60% مِن dwell
            t_post0 = t_step + 0.4 * (t_dwell_end - t_step)
            mask_post = (t >= t_post0) & (t <= t_dwell_end)
            if mask_post.sum() < 2:
                continue
            fb_post = float(np.mean(fb[mask_post]))
            err = fb_post - cmd_target

            # تَتَبَّع أَقصى fb وأَوَّل saturation
            if phase == "pos_test":
                if fb_post > per_servo_max_pos[sidx]:
                    per_servo_max_pos[sidx] = fb_post
                if (per_servo_first_sat_pos[sidx] is None and
                        abs(err) > 1.0 and cmd_target > 0):
                    per_servo_first_sat_pos[sidx] = cmd_target
            elif phase == "neg_test":
                if fb_post < per_servo_max_neg[sidx]:
                    per_servo_max_neg[sidx] = fb_post
                if (per_servo_first_sat_neg[sidx] is None and
                        abs(err) > 1.0 and cmd_target < 0):
                    per_servo_first_sat_neg[sidx] = cmd_target

            rows.append({
                "servo_idx": int(sidx),
                "phase": phase,
                "cmd_target": cmd_target,
                "fb_post": fb_post,
                "error": err,
                "stuck": abs(err) > 1.0,
            })

    # طِباعَة جَدوَل المُقارَنَة لكلّ cmd
    lines.append("")
    lines.append("  جَدوَل المُقارَنَة (cmd → fb لكلّ سيرفو):")
    header = f"  {'cmd':>+8s}"
    for s in target_servos:
        header += f"  {'srv'+str(s+1):>10s}"
    lines.append(header)
    lines.append("  " + "─" * (8 + 12 * len(target_servos)))

    # نُجَمِّع per-cmd per-servo
    by_cmd: Dict[float, Dict[int, float]] = {}
    for r in rows:
        cmd = r["cmd_target"]
        by_cmd.setdefault(cmd, {})[r["servo_idx"]] = r["fb_post"]

    for cmd in sorted(by_cmd.keys()):
        line = f"  {cmd:>+8.2f}"
        for s in target_servos:
            fb = by_cmd[cmd].get(s, float("nan"))
            err = fb - cmd
            mark = "✅" if abs(err) < 1.0 else "❌"
            line += f"  {fb:>+8.2f}{mark}"
        lines.append(line)

    # مُلَخَّص saturation لكلّ سيرفو
    lines.append("")
    lines.append("  ─── saturation و حُدود لكلّ سيرفو ───")
    lines.append(f"  {'servo':>6s}  {'max_pos':>10s}  {'sat@pos':>10s}  "
                 f"{'max_neg':>10s}  {'sat@neg':>10s}  asymmetry")
    for s in target_servos:
        max_p = per_servo_max_pos[s]
        max_n = per_servo_max_neg[s]
        sat_p = per_servo_first_sat_pos[s]
        sat_n = per_servo_first_sat_neg[s]
        asym = max_p - abs(max_n)
        sat_p_str = f"{sat_p:+.1f}" if sat_p is not None else "—"
        sat_n_str = f"{sat_n:+.1f}" if sat_n is not None else "—"
        verdict = "✅" if abs(asym) < 3.0 else "🔴"
        lines.append(
            f"  #{s+1:>5d}  {max_p:>+10.2f}  {sat_p_str:>10s}  "
            f"{max_n:>+10.2f}  {sat_n_str:>10s}  {asym:>+7.2f}° {verdict}"
        )

    # تَشخيص نِهائي
    lines.append("")
    lines.append("  ─── التَّشخيص النِّهائي ───")
    pos_maxes = [per_servo_max_pos[s] for s in target_servos]
    neg_maxes = [per_servo_max_neg[s] for s in target_servos]
    if pos_maxes and neg_maxes:
        pos_spread = max(pos_maxes) - min(pos_maxes)
        neg_spread = max(neg_maxes) - min(neg_maxes)

        if pos_spread > 3.0 or neg_spread > 3.0:
            lines.append(
                f"  🔴 تَفاوُت بَين السيرفوهات (pos_spread={pos_spread:.2f}°, "
                f"neg_spread={neg_spread:.2f}°)"
            )
            lines.append(
                "      → التَّفسير الأَرجَح: firmware السيرفوهات لَه "
                "حُدود داخِليَّة مُختَلِفَة، البروتوكول سَليم."
            )
        else:
            lines.append("  ✅ كلّ السيرفوهات تَتَصَرَّف بشَكل مُتَقارِب")

    out_csv: Optional[Path] = None
    if rows:
        try:
            import pandas as pd
            out_csv = csv_path.with_suffix("").parent / (
                csv_path.stem + ".firmware_audit.csv")
            pd.DataFrame(rows).to_csv(out_csv, index=False)
        except Exception:
            out_csv = None
    return (lines, out_csv)


def _cold_start_breakdown(df, schedule: List[Dict], csv_path: Path):
    """يُقارن delay/τ بين أوّل step (cold) والـ 2/3 (warm/hot) لكلّ سيرفو."""
    lines: List[str] = []
    lines.append("─── cold_start breakdown ─────────────────────────────")
    if not schedule:
        lines.append("  (no schedule)")
        return (lines, None)

    target_servos = sorted(df["servo_idx"].unique())
    pat_t0 = 0.0
    if target_servos:
        first_cmd = float(schedule[0]["cmd_to"])
        first_t = float(schedule[0]["t_edge_s"])
        for s in target_servos:
            off = _detect_pattern_offset(df, int(s), first_cmd, first_t)
            if off is not None:
                pat_t0 = off
                break
        lines.append(f"  pattern alignment: csv_t = sched_t + {pat_t0:.3f}s")

    rows: List[Dict] = []
    for sidx in target_servos:
        arrs = _per_servo_arrays(df, int(sidx))
        if arrs is None:
            continue
        t, _cmd, fb = arrs
        per_step: List[Dict] = []
        for ev in schedule:
            t_edge = float(ev["t_edge_s"]) + pat_t0
            cmd_to = float(ev["cmd_to"])
            cmd_from = float(ev["cmd_from"])
            label = str(ev.get("label", ""))
            # نأخذ نافذة 1s بعد الـ edge
            mask = (t >= t_edge) & (t <= t_edge + 1.0)
            if mask.sum() < 5:
                per_step.append({"label": label, "delay_ms": float("nan"),
                                  "tau_ms": float("nan"), "OS_pct": float("nan")})
                continue
            ts = t[mask] - t_edge
            ys = fb[mask]
            step = cmd_to - cmd_from
            # delay: أوّل خروج خارج 5% deadband
            deadband = 0.05 * abs(step)
            moved = np.abs(ys - cmd_from) > deadband
            if not np.any(moved):
                delay_ms = float("nan")
                tau_ms = float("nan")
            else:
                delay_ms = float(ts[int(np.argmax(moved))]) * 1000.0
                # τ: الوصول لـ 63.2%
                target = cmd_from + 0.632 * step
                if step > 0:
                    reached = ys >= target
                else:
                    reached = ys <= target
                if np.any(reached):
                    tau_ms = (float(ts[int(np.argmax(reached))]) -
                              float(ts[int(np.argmax(moved))])) * 1000.0
                else:
                    tau_ms = float("nan")
            # overshoot
            if step != 0:
                peak = float(np.max(ys) if step > 0 else np.min(ys))
                os_pct = (peak - cmd_to) / step * 100.0
            else:
                os_pct = float("nan")
            per_step.append({"label": label, "delay_ms": delay_ms,
                              "tau_ms": tau_ms, "OS_pct": os_pct})
        # Compare cold vs warm
        if len(per_step) >= 2:
            cold = per_step[0]
            warm = per_step[1]
            d_delay = cold["delay_ms"] - warm["delay_ms"]
            d_tau = cold["tau_ms"] - warm["tau_ms"]
            for ps in per_step:
                lines.append(
                    f"  servo#{sidx} [{ps['label']:4s}]: "
                    f"delay={ps['delay_ms']:6.1f}ms  τ={ps['tau_ms']:6.1f}ms  "
                    f"OS={ps['OS_pct']:5.1f}%"
                )
            verdict = "OK" if abs(d_delay) < 20 and abs(d_tau) < 20 else "REVIEW"
            lines.append(f"  → servo#{sidx} cold-vs-warm Δdelay={d_delay:+.1f}ms "
                         f"Δτ={d_tau:+.1f}ms [{verdict}]")
            rows.append({
                "servo_idx": int(sidx),
                "delay_cold_ms": cold["delay_ms"], "delay_warm_ms": warm["delay_ms"],
                "tau_cold_ms": cold["tau_ms"], "tau_warm_ms": warm["tau_ms"],
                "delta_delay_ms": d_delay, "delta_tau_ms": d_tau,
                "verdict": verdict,
            })

    out_csv: Optional[Path] = None
    if rows:
        try:
            import pandas as pd
            out_csv = csv_path.with_suffix("").parent / (
                csv_path.stem + ".cold_start.csv")
            pd.DataFrame(rows).to_csv(out_csv, index=False)
        except Exception:
            out_csv = None
    return (lines, out_csv)


def _endurance_breakdown(df, schedule: List[Dict], csv_path: Path):
    """يَحسب slew_max و std fb لكلّ نافذة (60s) لكَشف drift عبر الزمن."""
    lines: List[str] = []
    lines.append("─── endurance breakdown ──────────────────────────────")
    if not schedule:
        lines.append("  (no schedule)")
        return (lines, None)

    target_servos = sorted(df["servo_idx"].unique())
    pat_t0 = 0.0
    if target_servos:
        # endurance يَستخدم square wave — أوّل half-period يَنتقل من 0 → +amp
        amp = float(schedule[0]["amp_deg"])
        first_t = float(schedule[0]["t_window_start_s"])
        for s in target_servos:
            off = _detect_pattern_offset(df, int(s), amp, first_t)
            if off is not None:
                pat_t0 = off
                break
        lines.append(f"  pattern alignment: csv_t = sched_t + {pat_t0:.3f}s")

    rows: List[Dict] = []
    for sidx in target_servos:
        arrs = _per_servo_arrays(df, int(sidx))
        if arrs is None:
            continue
        t, _cmd, fb = arrs
        for ev in schedule:
            t0 = float(ev["t_window_start_s"]) + pat_t0
            t1 = float(ev["t_window_end_s"]) + pat_t0
            mask = (t >= t0) & (t <= t1)
            if mask.sum() < 50:
                continue
            ts = t[mask]
            ys = fb[mask]
            tu, idxu = np.unique(ts, return_index=True)
            yu = ys[idxu]
            if len(tu) < 5:
                continue
            slew = np.max(np.abs(np.diff(yu) / np.maximum(np.diff(tu), 1e-6)))
            fb_std = float(np.std(yu, ddof=1))
            fb_mean = float(np.mean(yu))
            rows.append({
                "servo_idx": int(sidx), "window_idx": int(ev["window_idx"]),
                "fb_mean": fb_mean, "fb_std": fb_std,
                "peak_slew_dps": float(slew), "n_samples": int(mask.sum()),
            })

    # Print summary per servo: first window vs last window
    for sidx in target_servos:
        wins = [r for r in rows if r["servo_idx"] == sidx]
        if len(wins) >= 2:
            first = wins[0]
            last = wins[-1]
            d_slew = last["peak_slew_dps"] - first["peak_slew_dps"]
            d_std = last["fb_std"] - first["fb_std"]
            verdict = "OK" if (abs(d_slew) < 50 and abs(d_std) < 0.1) else "DRIFT"
            lines.append(f"  servo#{sidx}: {len(wins)} windows  "
                         f"Δslew={d_slew:+.1f}°/s  Δstd={d_std:+.3f}°  [{verdict}]")
        elif len(wins) == 1:
            lines.append(f"  servo#{sidx}: single window — duration too short")

    out_csv: Optional[Path] = None
    if rows:
        try:
            import pandas as pd
            out_csv = csv_path.with_suffix("").parent / (
                csv_path.stem + ".endurance.csv")
            pd.DataFrame(rows).to_csv(out_csv, index=False)
        except Exception:
            out_csv = None
    return (lines, out_csv)


# ─── Tier-4: fault-detection / pre-flight integrity ───────────────────────

def _preflight_check_breakdown(df, schedule: List[Dict], csv_path: Path,
                               cfg: Optional[dict] = None):
    """مَرحَلَة-بمَرحَلَة GO/NO-GO verdict مع per-servo PASS/FAIL.

    يَطبَع تَفصيل لكلّ مَرحَلَة (online/zero/direction/wiring/travel/step/
    recovery) ولكلّ سيرفو، ثمّ verdict نِهائي واحِد (GO أَو NO-GO).
    Thresholds تُؤخَذ من ``cfg.pattern.preflight_check`` (مع defaults).
    """
    lines: List[str] = []
    lines.append("─── preflight_check breakdown ─────────────────────────")
    if not schedule:
        lines.append("  (no schedule)")
        return (lines, None)

    pf = (cfg or {}).get("pattern", {}).get("preflight_check", {})
    rate_min_hz = float(pf.get("rate_min_hz", 30.0))
    zero_amp_tol = float(pf.get("zero_amp_tol_deg", 0.30))
    zero_std_tol = float(pf.get("zero_std_tol_deg", 0.20))
    direction_amp = float(pf.get("direction_amp_deg", 2.0))
    direction_min_frac = float(pf.get("direction_min_frac", 0.5))
    wiring_amp = float(pf.get("wiring_amp_deg", 5.0))
    wiring_active_min = float(
        pf.get("wiring_active_min_frac", 0.5)) * wiring_amp
    witness_tol = float(pf.get("wiring_witness_tol_deg", 1.0))
    travel_amp_default = float(pf.get("travel_amp_deg", 8.0))
    travel_tol = float(pf.get("travel_tol_deg", 1.5))
    step_amp = float(pf.get("step_amp_deg", 5.0))
    delay_max_ms = float(pf.get("step_delay_max_ms", 50.0))
    tau_max_ms = float(pf.get("step_tau_max_ms", 80.0))
    os_max_pct = float(pf.get("step_os_max_pct", 30.0))
    recovery_tol = float(pf.get("recovery_tol_deg", 0.5))

    target_servos = sorted(int(s) for s in df["servo_idx"].unique())

    # ─── auto-align CSV t to schedule t (relative-to-pattern-start) ──
    pat_t0 = 0.0
    direction_phase = next(
        (p for p in schedule if p.get("phase") == "direction_sign"), None)
    if direction_phase is not None:
        sched_pos_t = float(direction_phase["t_pos_s"])
        for s in target_servos:
            off = _detect_pattern_offset(df, s, +direction_amp, sched_pos_t)
            if off is not None:
                pat_t0 = off
                break

    lines.append(f"  alignment: csv_t = sched_t + {pat_t0:.3f}s")
    lines.append(f"  thresholds: rate≥{rate_min_hz:.0f}Hz  "
                 f"|zero|<{zero_amp_tol:.2f}°  std<{zero_std_tol:.2f}°  "
                 f"witness<{witness_tol:.2f}°")
    lines.append(f"              travel±{travel_tol:.1f}°  "
                 f"step delay<{delay_max_ms:.0f}ms  τ<{tau_max_ms:.0f}ms  "
                 f"OS<{os_max_pct:.0f}%")

    arrs = {s: _per_servo_arrays(df, s) for s in target_servos}
    per_servo_failures: Dict[int, List[str]] = {s: [] for s in target_servos}
    rows: List[Dict] = []

    for entry in schedule:
        phase = entry["phase"]
        ts = float(entry["t_start_s"]) + pat_t0
        te = float(entry["t_end_s"]) + pat_t0

        # ─── 1) online_check ────────────────────────────────────────
        if phase == "online_check":
            for s in target_servos:
                a = arrs[s]
                if a is None:
                    per_servo_failures[s].append("online: no fb arrays")
                    rows.append({"servo": s, "phase": phase,
                                 "metric": "fb_rate_hz", "value": 0.0,
                                 "threshold": rate_min_hz, "verdict": "FAIL"})
                    continue
                t, _, _ = a
                mask = (t >= ts) & (t < te)
                n_in = int(mask.sum())
                dur = max(te - ts, 1e-6)
                rate = n_in / dur
                ok = rate >= rate_min_hz
                if not ok:
                    per_servo_failures[s].append(
                        f"online: rate={rate:.0f}Hz<{rate_min_hz:.0f}")
                rows.append({"servo": s, "phase": phase,
                             "metric": "fb_rate_hz", "value": round(rate, 1),
                             "threshold": rate_min_hz,
                             "verdict": "PASS" if ok else "FAIL"})

        # ─── 2) zero_stab ───────────────────────────────────────────
        elif phase == "zero_stab":
            for s in target_servos:
                a = arrs[s]
                if a is None:
                    continue
                t, _, fb = a
                mask = (t >= ts) & (t < te)
                if mask.sum() < 3:
                    per_servo_failures[s].append("zero_stab: insufficient")
                    continue
                fb_w = fb[mask]
                m = float(np.mean(fb_w))
                sd = float(np.std(fb_w))
                ok_m = abs(m) < zero_amp_tol
                ok_sd = sd < zero_std_tol
                if not ok_m:
                    per_servo_failures[s].append(
                        f"zero: |mean|={abs(m):.2f}°>{zero_amp_tol:.2f}")
                if not ok_sd:
                    per_servo_failures[s].append(
                        f"zero: std={sd:.2f}°>{zero_std_tol:.2f}")
                rows.append({"servo": s, "phase": phase,
                             "metric": "mean_deg", "value": round(m, 3),
                             "threshold": zero_amp_tol,
                             "verdict": "PASS" if ok_m else "FAIL"})
                rows.append({"servo": s, "phase": phase,
                             "metric": "std_deg", "value": round(sd, 3),
                             "threshold": zero_std_tol,
                             "verdict": "PASS" if ok_sd else "FAIL"})

        # ─── 3) direction_sign ──────────────────────────────────────
        elif phase == "direction_sign":
            t_mid = float(entry["t_neg_s"]) + pat_t0
            for s in target_servos:
                a = arrs[s]
                if a is None:
                    continue
                t, _, fb = a
                mp = (t >= ts + 0.20) & (t < t_mid - 0.05)
                mn = (t >= t_mid + 0.20) & (t < te - 0.05)
                fb_pos = float(np.mean(fb[mp])) if mp.sum() >= 2 else 0.0
                fb_neg = float(np.mean(fb[mn])) if mn.sum() >= 2 else 0.0
                thr = direction_amp * direction_min_frac
                ok_pos = fb_pos > thr
                ok_neg = fb_neg < -thr
                if not ok_pos:
                    per_servo_failures[s].append(
                        f"dir(+): fb={fb_pos:+.2f}° (need>{thr:+.2f})")
                if not ok_neg:
                    per_servo_failures[s].append(
                        f"dir(-): fb={fb_neg:+.2f}° (need<{-thr:+.2f})")
                rows.append({"servo": s, "phase": phase,
                             "metric": "fb_pos", "value": round(fb_pos, 3),
                             "threshold": thr,
                             "verdict": "PASS" if ok_pos else "FAIL"})
                rows.append({"servo": s, "phase": phase,
                             "metric": "fb_neg", "value": round(fb_neg, 3),
                             "threshold": -thr,
                             "verdict": "PASS" if ok_neg else "FAIL"})

        # ─── 4) wiring_isolation ────────────────────────────────────
        elif phase == "wiring_isolation":
            slot = int(entry.get("slot_in_target", 0))
            # active servo: الذي |cmd|>50% في النافذة
            active_servo = None
            for s in target_servos:
                g = df[df["servo_idx"] == s]
                m_cmd = (g["t_s"] >= ts) & (g["t_s"] < te)
                if m_cmd.sum() == 0:
                    continue
                cmd_max = float(g.loc[m_cmd, "cmd_deg"].abs().max())
                if cmd_max > wiring_amp * 0.5:
                    active_servo = s
                    break
            if active_servo is None:
                lines.append(f"  wiring slot {slot}: cannot identify active")
                continue

            # active يَجِب أَن يَصِل ≥ wiring_active_min (آخر 50% من نافِذَة)
            a = arrs[active_servo]
            if a is not None:
                t, _, fb = a
                t_check = ts + (te - ts) * 0.5
                mask = (t >= t_check) & (t < te)
                if mask.sum() >= 2:
                    fb_max = float(np.max(fb[mask]))
                    ok = fb_max >= wiring_active_min
                    if not ok:
                        per_servo_failures[active_servo].append(
                            f"wiring slot{slot}(active): "
                            f"max fb={fb_max:.2f}°<{wiring_active_min:.2f}")
                    rows.append({"servo": active_servo, "phase": phase,
                                 "metric": f"slot{slot}_active_max",
                                 "value": round(fb_max, 3),
                                 "threshold": wiring_active_min,
                                 "verdict": "PASS" if ok else "FAIL"})

            # witnesses لا يَجِب أَن يَتَحَرَّكوا
            for w in target_servos:
                if w == active_servo:
                    continue
                a = arrs[w]
                if a is None:
                    continue
                t, _, fb = a
                mask = (t >= ts + 0.20) & (t < te)
                if mask.sum() < 2:
                    continue
                fb_max = float(np.max(np.abs(fb[mask])))
                ok = fb_max < witness_tol
                if not ok:
                    per_servo_failures[w].append(
                        f"wiring slot{slot}: witness moved "
                        f"max|fb|={fb_max:.2f}°>{witness_tol:.2f}")
                rows.append({"servo": w, "phase": phase,
                             "metric": f"slot{slot}_witness_maxabs",
                             "value": round(fb_max, 3),
                             "threshold": witness_tol,
                             "verdict": "PASS" if ok else "FAIL"})

        # ─── 5) travel_check ────────────────────────────────────────
        elif phase == "travel_check":
            expected = float(entry.get("cmd_amp_deg", travel_amp_default))
            for s in target_servos:
                a = arrs[s]
                if a is None:
                    continue
                t, _, fb = a
                mask = (t >= ts) & (t < te)
                if mask.sum() < 5:
                    continue
                fb_w = fb[mask]
                fb_max = float(np.max(fb_w))
                fb_min = float(np.min(fb_w))
                ok_pos = fb_max >= expected - travel_tol
                ok_neg = fb_min <= -expected + travel_tol
                if not ok_pos:
                    per_servo_failures[s].append(
                        f"travel(+): max={fb_max:+.2f}°"
                        f"<{expected-travel_tol:+.2f}")
                if not ok_neg:
                    per_servo_failures[s].append(
                        f"travel(-): min={fb_min:+.2f}°"
                        f">{-expected+travel_tol:+.2f}")
                rows.append({"servo": s, "phase": phase,
                             "metric": "fb_max", "value": round(fb_max, 3),
                             "threshold": expected - travel_tol,
                             "verdict": "PASS" if ok_pos else "FAIL"})
                rows.append({"servo": s, "phase": phase,
                             "metric": "fb_min", "value": round(fb_min, 3),
                             "threshold": -expected + travel_tol,
                             "verdict": "PASS" if ok_neg else "FAIL"})

        # ─── 6) step_response ───────────────────────────────────────
        elif phase == "step_response":
            for s in target_servos:
                a = arrs[s]
                if a is None:
                    continue
                t, _, fb = a
                pre = (t >= ts - 0.10) & (t < ts)
                post = (t >= ts) & (t < te)
                if pre.sum() < 2 or post.sum() < 5:
                    per_servo_failures[s].append("step: insufficient")
                    continue
                y_init = float(np.mean(fb[pre]))
                m = _edge_metrics(t, fb, ts, y_init, step_amp, te)
                if m is None:
                    per_servo_failures[s].append("step: edge metrics failed")
                    continue
                d_ms = m.get("delay_ms", float("nan"))
                tau_ms_v = m.get("tau_ms", float("nan"))
                os_v = m.get("overshoot_pct", float("nan"))
                ok_d = (not np.isnan(d_ms)) and d_ms <= delay_max_ms
                ok_t = (not np.isnan(tau_ms_v)) and tau_ms_v <= tau_max_ms
                ok_o = (not np.isnan(os_v)) and abs(os_v) <= os_max_pct
                if not ok_d:
                    per_servo_failures[s].append(
                        f"step: delay={d_ms:.0f}ms>{delay_max_ms:.0f}")
                if not ok_t:
                    per_servo_failures[s].append(
                        f"step: τ={tau_ms_v:.0f}ms>{tau_max_ms:.0f}")
                if not ok_o:
                    per_servo_failures[s].append(
                        f"step: OS={os_v:+.1f}%>{os_max_pct:.0f}%")
                rows.append({"servo": s, "phase": phase, "metric": "delay_ms",
                             "value": round(d_ms, 1) if not np.isnan(d_ms)
                             else None, "threshold": delay_max_ms,
                             "verdict": "PASS" if ok_d else "FAIL"})
                rows.append({"servo": s, "phase": phase, "metric": "tau_ms",
                             "value": round(tau_ms_v, 1)
                             if not np.isnan(tau_ms_v) else None,
                             "threshold": tau_max_ms,
                             "verdict": "PASS" if ok_t else "FAIL"})
                rows.append({"servo": s, "phase": phase,
                             "metric": "overshoot_pct",
                             "value": round(os_v, 1)
                             if not np.isnan(os_v) else None,
                             "threshold": os_max_pct,
                             "verdict": "PASS" if ok_o else "FAIL"})

        # ─── 7) recovery ────────────────────────────────────────────
        elif phase == "recovery":
            for s in target_servos:
                a = arrs[s]
                if a is None:
                    continue
                t, _, fb = a
                mask = (t >= te - 0.30) & (t < te)
                if mask.sum() < 2:
                    continue
                m = float(np.mean(fb[mask]))
                ok = abs(m) < recovery_tol
                if not ok:
                    per_servo_failures[s].append(
                        f"recovery: |fb|={abs(m):.2f}°>{recovery_tol:.2f}")
                rows.append({"servo": s, "phase": phase,
                             "metric": "final_fb", "value": round(m, 3),
                             "threshold": recovery_tol,
                             "verdict": "PASS" if ok else "FAIL"})

    # ─── per-servo summary + overall verdict ────────────────────────
    lines.append("")
    lines.append("  Per-servo verdict:")
    overall_pass = True
    for s in target_servos:
        fails = per_servo_failures[s]
        if not fails:
            lines.append(f"    servo#{s}: PASS")
        else:
            overall_pass = False
            lines.append(f"    servo#{s}: FAIL ({len(fails)} issue(s))")
            for f in fails[:8]:
                lines.append(f"      - {f}")
            if len(fails) > 8:
                lines.append(f"      ... +{len(fails)-8} more")

    lines.append("")
    if overall_pass:
        lines.append("  ╔══════════════════════════════════════════╗")
        lines.append("  ║   PREFLIGHT VERDICT:  GO  (all servos)   ║")
        lines.append("  ╚══════════════════════════════════════════╝")
    else:
        lines.append("  ╔══════════════════════════════════════════╗")
        lines.append("  ║   PREFLIGHT VERDICT:  NO-GO              ║")
        lines.append("  ╚══════════════════════════════════════════╝")

    out_csv: Optional[Path] = None
    if rows:
        try:
            out_csv = csv_path.with_suffix("").parent / (
                csv_path.stem + ".preflight_check.csv")
            pd.DataFrame(rows).to_csv(out_csv, index=False)
        except Exception:
            out_csv = None
    return (lines, out_csv)


def _wiring_audit_breakdown(df, schedule: List[Dict], csv_path: Path):
    """يَكشف خَلط الأَسلاك بمُقارَنَة طَيف cmd vs fb لكلّ سيرفو.

    لكلّ سيرفو يُحَدِّد القِمَّة الطَيفيَّة في cmd (الـ freq المُرسَل) و fb
    (الـ freq المُستَقبَل). إذا اخْتَلَفا → خَلط أَسلاك (السيرفو مُتَّصِل
    بسلك سيرفو آخَر) أَو خَطأ في node ID.
    """
    lines: List[str] = []
    lines.append("─── wiring_audit breakdown ─────────────────────────")
    if not schedule:
        lines.append("  (no schedule)")
        return (lines, None)

    expected_freqs = sorted({float(e["expected_freq_hz"]) for e in schedule
                              if "expected_freq_hz" in e})
    if not expected_freqs:
        lines.append("  (no expected_freq_hz in schedule)")
        return (lines, None)
    target_servos = sorted(int(s) for s in df["servo_idx"].unique())

    f_max_search = max(expected_freqs) * 1.5

    def _dominant_freq(t_arr: np.ndarray, sig: np.ndarray):
        if len(t_arr) < 30 or float(t_arr[-1] - t_arr[0]) < 2.0:
            return None, 0.0
        fs = 100.0
        n = int((t_arr[-1] - t_arr[0]) * fs)
        if n < 64:
            return None, 0.0
        grid = float(t_arr[0]) + np.arange(n) / fs
        s_int = np.interp(grid, t_arr, sig)
        s_int = s_int - np.mean(s_int)
        w = np.hanning(n)
        S = np.fft.rfft(s_int * w)
        freq = np.fft.rfftfreq(n, d=1.0 / fs)
        mag = np.abs(S)
        sel = (freq > 0.3) & (freq < f_max_search)
        if not np.any(sel):
            return None, 0.0
        peak_idx = int(np.argmax(mag[sel]))
        return float(freq[sel][peak_idx]), float(mag[sel][peak_idx])

    cmd_freq_per_servo: Dict[int, Optional[float]] = {}
    fb_freq_per_servo: Dict[int, Optional[float]] = {}
    fb_mag_per_servo: Dict[int, float] = {}

    for s in target_servos:
        g = df[df["servo_idx"] == s].sort_values("t_s")
        t_cmd = g["t_s"].to_numpy(dtype=float)
        cmd = g["cmd_deg"].to_numpy(dtype=float)
        f_c, _ = _dominant_freq(t_cmd, cmd)
        cmd_freq_per_servo[s] = f_c
        a = _per_servo_arrays(df, s)
        if a is None:
            fb_freq_per_servo[s] = None
            fb_mag_per_servo[s] = 0.0
            continue
        t_fb, _, fb = a
        f_f, mag_f = _dominant_freq(t_fb, fb)
        fb_freq_per_servo[s] = f_f
        fb_mag_per_servo[s] = mag_f

    lines.append(f"  expected freqs: " +
                 ", ".join(f"{f:.2f}Hz" for f in expected_freqs))
    lines.append(f"  servos in CSV : {target_servos}")
    lines.append("")

    rows: List[Dict] = []
    n_fail_swap = 0
    n_fail_silent = 0
    n_fail_other = 0
    for s in target_servos:
        cmd_f = cmd_freq_per_servo.get(s)
        fb_f = fb_freq_per_servo.get(s)
        fb_m = fb_mag_per_servo.get(s, 0.0)
        if cmd_f is None or fb_f is None:
            verdict = "FAIL_NO_DATA"
            lines.append(f"  servo#{s}: cmd_f={cmd_f}  fb_f={fb_f}  "
                         f"[{verdict}]")
            n_fail_silent += 1
            rows.append({"servo": s, "cmd_freq_hz": cmd_f,
                         "fb_freq_hz": fb_f, "fb_mag": fb_m,
                         "verdict": verdict})
            continue
        delta = abs(fb_f - cmd_f)
        if delta < 0.5:
            verdict = "PASS"
            lines.append(f"  servo#{s}: cmd_f={cmd_f:.2f}Hz  "
                         f"fb_f={fb_f:.2f}Hz  Δ={delta:.2f}Hz  [{verdict}]")
        else:
            best_match = None
            for f_other in expected_freqs:
                if abs(f_other - cmd_f) < 0.3:
                    continue
                if abs(fb_f - f_other) < 0.5:
                    best_match = f_other
                    break
            if best_match is not None:
                verdict = "FAIL_SWAPPED"
                lines.append(f"  servo#{s}: cmd_f={cmd_f:.2f}Hz  "
                             f"fb_f={fb_f:.2f}Hz  → swapped with "
                             f"{best_match:.2f}Hz  [{verdict}]")
                n_fail_swap += 1
            else:
                verdict = "FAIL_NO_TRACK"
                lines.append(f"  servo#{s}: cmd_f={cmd_f:.2f}Hz  "
                             f"fb_f={fb_f:.2f}Hz  → no clear match  "
                             f"[{verdict}]")
                n_fail_other += 1
        rows.append({"servo": s, "cmd_freq_hz": round(cmd_f, 3),
                     "fb_freq_hz": round(fb_f, 3),
                     "fb_mag": round(fb_m, 1), "verdict": verdict})

    lines.append("")
    if n_fail_swap == 0 and n_fail_silent == 0 and n_fail_other == 0:
        lines.append("  WIRING OK — no servos swapped, all freqs tracked")
    else:
        lines.append(f"  WIRING FAULT — {n_fail_swap} swapped, "
                     f"{n_fail_silent} silent, "
                     f"{n_fail_other} no_track")

    out_csv: Optional[Path] = None
    if rows:
        try:
            out_csv = csv_path.with_suffix("").parent / (
                csv_path.stem + ".wiring_audit.csv")
            pd.DataFrame(rows).to_csv(out_csv, index=False)
        except Exception:
            out_csv = None
    return (lines, out_csv)


def _fault_scan_breakdown(df, schedule: List[Dict], csv_path: Path,
                          cfg: Optional[dict] = None):
    """يَرصِد انوماليّات: gaps, jumps, saturation, sign_mismatch, overshoot.

    لكلّ سيرفو: PASS لو كلّ المُؤَشِّرات تَحت الحُدود. وإلّا تَفصيل.
    """
    lines: List[str] = []
    lines.append("─── fault_scan breakdown ─────────────────────────")
    if not schedule:
        lines.append("  (no schedule)")
        return (lines, None)

    fs_cfg = (cfg or {}).get("pattern", {}).get("fault_scan", {})
    gap_max_ms = float(fs_cfg.get("gap_max_ms", 50.0))
    jump_max_deg = float(fs_cfg.get("jump_max_deg", 2.0))
    os_max_pct = float(fs_cfg.get("os_max_pct", 30.0))
    sign_tol_count = int(fs_cfg.get("sign_mismatch_max", 3))
    angle_limit = float(
        (cfg or {}).get("xqpower", {}).get("angle_limit_deg", 10.0))

    target_servos = sorted(int(s) for s in df["servo_idx"].unique())

    rows: List[Dict] = []
    overall_anomalies = 0
    for s in target_servos:
        a = _per_servo_arrays(df, s)
        if a is None:
            lines.append(f"  servo#{s}: insufficient fb")
            overall_anomalies += 1
            continue
        t, cmd, fb = a

        # 1) max gap بين fresh fb arrivals
        if len(t) >= 2:
            dt = np.diff(t) * 1000.0
            max_gap_ms = float(np.max(dt))
            mean_gap_ms = float(np.mean(dt))
        else:
            max_gap_ms = float("inf")
            mean_gap_ms = float("inf")
        gap_ok = max_gap_ms <= gap_max_ms

        # 2) jumps في steady regions (نَستَثني نَوافِذ ±100ms حَول edges)
        edges = _find_step_edges(cmd, threshold=0.5)
        jump_max = 0.0
        if len(fb) >= 2:
            dfb = np.abs(np.diff(fb))
            t_mid = 0.5 * (t[1:] + t[:-1])
            in_steady = np.ones(len(dfb), dtype=bool)
            for ei in edges:
                if ei < len(t):
                    te_abs = float(t[ei])
                    around = np.abs(t_mid - te_abs) < 0.10
                    in_steady &= ~around
            if in_steady.any():
                jump_max = float(np.max(dfb[in_steady]))
        jump_ok = jump_max <= jump_max_deg

        # 3) saturation events (|fb| > angle_limit)
        sat_count = int(np.sum(np.abs(fb) > angle_limit))
        sat_ok = sat_count == 0

        # 4) sign mismatch in steady (|cmd|>1 و |fb|>1 → نَفس الإشارة)
        # نَستَثني ±150ms بَعد كلّ edge للسماح بالـ transient.
        excluded = np.zeros(len(t), dtype=bool)
        for ei in edges:
            if ei < len(t):
                te_abs = float(t[ei])
                excluded |= ((t >= te_abs) & (t < te_abs + 0.15))
        sig_mask = (~excluded) & (np.abs(cmd) > 1.0) & (np.abs(fb) > 1.0)
        sign_mismatches = int(np.sum(
            sig_mask & ((cmd > 0) ^ (fb > 0))))
        sign_ok = sign_mismatches < sign_tol_count

        # 5) overshoot لأَوَّل 8 edges
        os_max = 0.0
        os_count = 0
        for ei in edges[:8]:
            m = _step_metrics(t, cmd, fb, ei, window_s=0.8, t_fb=t)
            if m is None:
                continue
            os = m.get("overshoot_pct", 0.0)
            if not np.isnan(os):
                os_max = max(os_max, abs(os))
                os_count += 1
        os_ok = os_max <= os_max_pct

        anomalies = sum(0 if x else 1 for x in
                        (gap_ok, jump_ok, sat_ok, sign_ok, os_ok))
        overall_anomalies += anomalies

        verdict = "OK" if anomalies == 0 else f"⚠ {anomalies} anomaly"
        lines.append(f"  servo#{s}: [{verdict}]")
        lines.append(f"    fb gap     : max={max_gap_ms:6.1f}ms  "
                     f"mean={mean_gap_ms:6.1f}ms  "
                     f"[{'OK' if gap_ok else 'FAIL'}]")
        lines.append(f"    fb jump    : max={jump_max:5.2f}°  "
                     f"[{'OK' if jump_ok else 'FAIL'}]")
        lines.append(f"    saturation : {sat_count} samples (>"
                     f"{angle_limit:.1f}°)  "
                     f"[{'OK' if sat_ok else 'FAIL'}]")
        lines.append(f"    sign mism. : {sign_mismatches} samples  "
                     f"[{'OK' if sign_ok else 'FAIL'}]")
        lines.append(f"    overshoot  : max={os_max:5.1f}% "
                     f"({os_count} edges)  "
                     f"[{'OK' if os_ok else 'FAIL'}]")

        rows.append({
            "servo": s,
            "max_gap_ms": round(max_gap_ms, 1),
            "mean_gap_ms": round(mean_gap_ms, 1),
            "max_jump_deg": round(jump_max, 2),
            "saturation_count": sat_count,
            "sign_mismatches": sign_mismatches,
            "max_overshoot_pct": round(os_max, 1),
            "anomalies": anomalies,
            "verdict": "OK" if anomalies == 0 else "FAIL",
        })

    lines.append("")
    if overall_anomalies == 0:
        lines.append("  NO FAULTS DETECTED — bus + servos clean")
    else:
        lines.append(f"  {overall_anomalies} anomalies across all servos")

    out_csv: Optional[Path] = None
    if rows:
        try:
            out_csv = csv_path.with_suffix("").parent / (
                csv_path.stem + ".fault_scan.csv")
            pd.DataFrame(rows).to_csv(out_csv, index=False)
        except Exception:
            out_csv = None
    return (lines, out_csv)


def _per_servo_arrays(df: "pd.DataFrame", servo_idx: int):
    """يستخرج (t, cmd, fb) لـ سيرفو واحد.

    t: الوقت الأنسب لقياس delay/edge — ``t_fb_arrival_s`` (وقت وصول
    فعليّ للـ CAN frame). للنوافذ التي لا يوجد بها fb طازج، الـ rows
    الـ stale (t_arrival لم يتغيّر) تُحذف لتجنّب نتائج زائفة.
    """
    g = df[df["servo_idx"] == servo_idx].sort_values("t_s").reset_index(drop=True)
    g_fb = g.dropna(subset=["fb_deg"]).reset_index(drop=True)
    if len(g_fb) < 5:
        return None
    if "t_fb_arrival_s" in g_fb.columns:
        t_arr = g_fb["t_fb_arrival_s"].to_numpy(dtype=float)
        nan_mask = np.isnan(t_arr)
        t_loop = g_fb["t_s"].to_numpy(dtype=float)
        t_arr[nan_mask] = t_loop[nan_mask]
        # احذف الـ rows المكرّرة (fb لم يتجدّد): نحتفظ بأوّل ظهور لكل arrival
        # (الـ runner يعيد forward-fill لـ fb بين الـ frames).
        diff = np.diff(t_arr, prepend=t_arr[0] - 1.0)
        fresh = diff > 0
        if fresh.sum() < 5:
            # كل المعلومات stale — استخدم t_s للنوافذ وإلّا التحليل يفشل
            t_arr = t_loop
        else:
            t_arr = t_arr[fresh]
            g_fb = g_fb.iloc[fresh].reset_index(drop=True)
    else:
        t_arr = g_fb["t_s"].to_numpy(dtype=float)
    fb = g_fb["fb_deg"].to_numpy(dtype=float)
    cmd = g_fb["cmd_deg"].to_numpy(dtype=float)
    return t_arr, cmd, fb


def _measure_y_init(t: np.ndarray, fb: np.ndarray, t0: float,
                    pre_window_s: float = 0.10,
                    fallback: float = 0.0) -> float:
    """يُقاس y_init من mean(fb) في window قبل t0 مباشرة."""
    pre_mask = (t >= t0 - pre_window_s) & (t < t0)
    if pre_mask.sum() >= 1:
        return float(np.mean(fb[pre_mask]))
    return fallback


def _detect_pattern_offset(df: "pd.DataFrame", servo_idx: int,
                           expected_first_target: float,
                           schedule_first_edge_t: float = 0.0
                           ) -> Optional[float]:
    """
    Auto-detect الفرق بين t_s في CSV و schedule (relative-to-pattern-start).

    Returns offset_s بحيث: csv_t_s ≈ schedule_t + offset_s.

    الطريقة: يبحث عن أوّل cmd transition في الـ CSV إلى ``expected_first_target``
    (= schedule[0].cmd_target/cmd_to). يرجع: csv_t_at_first_edge -
    schedule_first_edge_t.
    """
    g = df[df["servo_idx"] == servo_idx].sort_values("t_s").reset_index(drop=True)
    if len(g) < 2:
        return None
    cmd = g["cmd_deg"].to_numpy(dtype=float)
    t_s = g["t_s"].to_numpy(dtype=float)
    for i in range(1, len(cmd)):
        if abs(cmd[i] - expected_first_target) < 0.01 and \
           abs(cmd[i - 1] - expected_first_target) > 0.01:
            return float(t_s[i]) - float(schedule_first_edge_t)
    return None


def _step_matrix_breakdown(df, schedule, csv_path: Path):
    lines: List[str] = []
    lines.append("─── step_matrix breakdown ─────────────────────────────")

    # نختار target servo: الذي تغيّر cmd له (أكبر range)
    cmd_ranges = []
    for sidx in sorted(df["servo_idx"].unique()):
        g = df[df["servo_idx"] == sidx]
        cmd_ranges.append((sidx, float(g["cmd_deg"].max() - g["cmd_deg"].min())))
    target_idx = max(cmd_ranges, key=lambda x: x[1])[0]
    lines.append(f"  target servo: #{target_idx}")

    arrs = _per_servo_arrays(df, int(target_idx))
    if arrs is None:
        lines.append("  (insufficient fb)")
        return (lines, None)
    t, cmd, fb = arrs

    # auto-align: schedule t نسبيّ لبداية الـ pattern؛ CSV نسبيّ لـ t0
    expected_first_target = float(schedule[0]["cmd_target"])
    sched_first_t = float(schedule[0]["t_edge_up_s"])
    pat_t0 = _detect_pattern_offset(df, int(target_idx),
                                    expected_first_target, sched_first_t)
    if pat_t0 is None:
        lines.append("  WARN: لم نجد أوّل cmd transition في CSV — قد يفشل التحليل")
        pat_t0 = 0.0
    else:
        lines.append(f"  pattern alignment: csv_t = sched_t + {pat_t0:.3f}s "
                     f"(first edge @ csv {pat_t0 + sched_first_t:.3f}s)")

    # حلّل كل cell من schedule
    rows = []  # per-cell metrics
    for cell in schedule:
        t0 = float(cell["t_edge_up_s"]) + pat_t0
        win_end = float(cell["t_edge_down_s"]) + pat_t0
        # y_init قياسي: mean(fb) في 100ms قبل t0 (أكثر دقّة من schedule)
        y_init_meas = _measure_y_init(t, fb, t0, pre_window_s=0.10,
                                      fallback=float(cell["cmd_initial"]))
        y_final = float(cell["cmd_target"])
        m = _edge_metrics(t, fb, t0, y_init_meas, y_final, win_end)
        if m is None:
            continue
        rows.append({
            "cell_id": int(cell["cell_id"]),
            "amp_deg": float(cell["amp_deg"]),
            "offset_deg": float(cell["offset_deg"]),
            "direction": str(cell["direction"]),
            "repeat_idx": int(cell["repeat_idx"]),
            **m,
        })

    if not rows:
        lines.append("  (no cells matched fb data)")
        return (lines, None)

    # اكتب per-cell CSV
    out_csv = csv_path.with_suffix(".cells.csv")
    cell_df = pd.DataFrame(rows)
    cell_df.to_csv(out_csv, index=False, float_format="%.4f")

    # ملخّص: mean±std لكل (amp, offset, direction)
    lines.append("")
    lines.append("  per (amp, offset, direction):")
    grouped = cell_df.groupby(["amp_deg", "offset_deg", "direction"])
    for (amp, off, dirn), g in grouped:
        delays = g["delay_ms"].to_list()
        overshoots = g["overshoot_pct"].to_list()
        mean_d = float(np.nanmean(delays))
        std_d = float(np.nanstd(delays, ddof=1)) if len(delays) > 1 else 0.0
        mean_os = float(np.nanmean(overshoots))
        std_os = float(np.nanstd(overshoots, ddof=1)) if len(overshoots) > 1 else 0.0
        lines.append(
            f"    amp={amp:+5.1f}° off={off:+5.1f}° dir={dirn:<4s} N={len(g):2d}  "
            f"delay={mean_d:6.2f}±{std_d:5.2f}ms  "
            f"OS={mean_os:6.2f}±{std_os:5.2f}%"
        )

    # تحليل اعتماد delay على amp
    lines.append("")
    lines.append("  delay vs amplitude (linearity):")
    by_amp = cell_df.groupby("amp_deg")["delay_ms"].mean().sort_index()
    if len(by_amp) >= 3:
        amps = by_amp.index.to_numpy(dtype=float)
        delays = by_amp.to_numpy(dtype=float)
        slope, intercept = np.polyfit(amps, delays, 1)
        lines.append(f"    delay(amp) ≈ {slope:+.2f} ms/° × amp + {intercept:.2f} ms")
        lines.append(f"    {'OK: independent of amp' if abs(slope) < 1.0 else 'WARN: amp-dependent delay'}")

    # direction asymmetry
    by_dir = cell_df.groupby("direction")["delay_ms"].mean()
    if "up" in by_dir.index and "down" in by_dir.index:
        diff = float(by_dir["up"] - by_dir["down"])
        lines.append(f"  direction asymmetry: |up - down| = {abs(diff):.2f}ms "
                     f"(up={by_dir['up']:.1f}ms, down={by_dir['down']:.1f}ms)")

    return (lines, out_csv)


def _aggregate_tau_fit(t: np.ndarray, fb: np.ndarray, schedule: List[Dict],
                       pat_t0: float, pre_window_s: float = 0.10
                       ) -> Optional[Dict]:
    """يُجمّع كلّ transitions الـ repeatability في fit موحّد لـ (delay, τ).

    لكلّ transition: نُنشئ (t_rel, y_norm) حيث:
        t_rel  = t - t_edge_s
        y_norm = (fb - y_init) / (y_final - y_init)
    ثمّ نَفت first-order: y_norm(t) = 1 - exp(-(t - delay) / τ) for t > delay.

    قبل الـ delay (t < delay): y_norm = 0.
    تُجَمَّع كلّ النقاط من 100 transition في pool واحد → fit مستقرّ
    حتّى لو كان fb sampling خشِناً (50Hz). كلّ transition تُسهم بـ ~10 نقاط
    → 1000 نقطة في الـ fit الإجمالي.
    """
    try:
        from scipy.optimize import curve_fit
    except ImportError:
        return None

    t_pool = []
    y_pool = []
    for ev in schedule:
        # حدّد مفاتيح الـ schedule بمرونة (repeatability vs step_matrix)
        t_edge = float(ev.get("t_edge_s",
                              ev.get("t_edge_up_s", 0.0))) + pat_t0
        # نهاية النافذة: حتى الـ edge التالي
        # (لكن schedule هنا يأتي من الـ caller — لا نملك next مباشرة)
        # سنستخدم نافذة 1.0s ثابتة بعد الـ edge.
        win_end = t_edge + 1.0

        y_init = _measure_y_init(t, fb, t_edge, pre_window_s=pre_window_s,
                                  fallback=float(ev.get("cmd_from",
                                                        ev.get("cmd_initial",
                                                               0.0))))
        y_final = float(ev.get("cmd_to", ev.get("cmd_target", 0.0)))
        step = y_final - y_init
        if abs(step) < 1e-3:
            continue
        mask = (t >= t_edge) & (t <= win_end)
        if mask.sum() < 3:
            continue
        ts = t[mask] - t_edge
        ys = (fb[mask] - y_init) / step
        t_pool.append(ts)
        y_pool.append(ys)
    if not t_pool:
        return None
    t_arr = np.concatenate(t_pool)
    y_arr = np.concatenate(y_pool)

    # نموذج first-order مع pure delay: y(t) = 0 إن t<delay وإلّا 1-exp(-(t-d)/τ)
    def model(t, delay, tau):
        out = np.where(t < delay, 0.0,
                       1.0 - np.exp(-np.maximum(t - delay, 0.0) / max(tau, 1e-4)))
        return out

    # تقديرات أوّليّة: delay من أوّل نقطة بعد العتبة، τ من الـ rise
    # نستخدم median لكلّ bin زمني صغير لإستخراج tendencies
    try:
        # initial guess
        p0 = [0.05, 0.030]   # 50ms delay, 30ms τ
        bounds = ([0.0, 1e-3], [0.5, 0.5])  # delay∈[0..500ms], τ∈[1..500ms]
        popt, pcov = curve_fit(model, t_arr, y_arr, p0=p0, bounds=bounds,
                                maxfev=5000)
        delay_s, tau_s = float(popt[0]), float(popt[1])
        perr = np.sqrt(np.diag(pcov))
        delay_se_s, tau_se_s = float(perr[0]), float(perr[1])
        # R²
        y_pred = model(t_arr, *popt)
        ss_res = float(np.sum((y_arr - y_pred) ** 2))
        ss_tot = float(np.sum((y_arr - np.mean(y_arr)) ** 2))
        r2 = 1.0 - ss_res / max(ss_tot, 1e-12)
    except Exception:
        return None

    return {
        "delay_ms": delay_s * 1000.0,
        "delay_se_ms": delay_se_s * 1000.0,
        "tau_ms": tau_s * 1000.0,
        "tau_se_ms": tau_se_s * 1000.0,
        "r2": r2,
        "n_points": int(len(t_arr)),
        "n_transitions": int(len(t_pool)),
    }


def _repeatability_breakdown(df, schedule, csv_path: Path):
    lines: List[str] = []
    lines.append("─── repeatability breakdown ──────────────────────────")

    # target servo (أعلى cmd range)
    cmd_ranges = []
    for sidx in sorted(df["servo_idx"].unique()):
        g = df[df["servo_idx"] == sidx]
        cmd_ranges.append((sidx, float(g["cmd_deg"].max() - g["cmd_deg"].min())))
    target_idx = max(cmd_ranges, key=lambda x: x[1])[0]
    lines.append(f"  target servo: #{target_idx}")

    arrs = _per_servo_arrays(df, int(target_idx))
    if arrs is None:
        lines.append("  (insufficient fb)")
        return (lines, None)
    t, cmd, fb = arrs

    expected_first_target = float(schedule[0]["cmd_to"])
    sched_first_t = float(schedule[0]["t_edge_s"])
    pat_t0 = _detect_pattern_offset(df, int(target_idx),
                                    expected_first_target, sched_first_t)
    if pat_t0 is None:
        lines.append("  WARN: لم نجد أوّل cmd transition — قد يفشل التحليل")
        pat_t0 = 0.0
    else:
        lines.append(f"  pattern alignment: csv_t = sched_t + {pat_t0:.3f}s "
                     f"(edge_0 @ csv {pat_t0 + sched_first_t:.3f}s)")

    rows = []
    for i, ev in enumerate(schedule):
        t0 = float(ev["t_edge_s"]) + pat_t0
        y_init_meas = _measure_y_init(t, fb, t0, pre_window_s=0.10,
                                       fallback=float(ev["cmd_from"]))
        y_final = float(ev["cmd_to"])
        # نافذة: حتى الـ edge التالي (أو نهاية الـ test)
        if i + 1 < len(schedule):
            win_end = float(schedule[i + 1]["t_edge_s"]) + pat_t0
        else:
            win_end = t0 + 1.5  # تقدير
        m = _edge_metrics(t, fb, t0, y_init_meas, y_final, win_end)
        if m is None:
            continue
        rows.append({
            "edge_idx": int(ev["edge_idx"]),
            "cycle_idx": int(ev["cycle_idx"]),
            "direction": str(ev["direction"]),
            **m,
        })

    if not rows:
        lines.append("  (no edges matched fb data)")
        return (lines, None)

    out_csv = csv_path.with_suffix(".cells.csv")
    cell_df = pd.DataFrame(rows)
    cell_df.to_csv(out_csv, index=False, float_format="%.4f")

    # stats
    lines.append("")
    for d in ["up", "down", "all"]:
        g = cell_df if d == "all" else cell_df[cell_df["direction"] == d]
        if len(g) == 0:
            continue
        lines.append(f"  [{d:<4s}] N={len(g)}")
        lines.append("    " + _summarize(g["delay_ms"].to_list(), "delay (ms)"))
        lines.append("    " + _summarize(g["overshoot_pct"].to_list(), "OS    (%) "))
        lines.append("    " + _summarize(g["settling_ms"].to_list(), "settle(ms)"))

    # drift: linear regression delay vs cycle
    if len(cell_df) >= 5:
        ci = cell_df["cycle_idx"].to_numpy(dtype=float)
        dl = cell_df["delay_ms"].to_numpy(dtype=float)
        valid = ~np.isnan(dl)
        if valid.sum() >= 5:
            slope, _ = np.polyfit(ci[valid], dl[valid], 1)
            verdict = "OK" if abs(slope) < 0.2 else "WARN drift"
            lines.append(f"  delay drift over cycles: {slope:+.3f} ms/cycle [{verdict}]")

    # aggregate first-order fit (delay + τ) من 100 transition مجتمعة
    fit = _aggregate_tau_fit(t, fb, schedule, pat_t0)
    if fit is not None:
        lines.append("")
        lines.append("  ─── aggregate first-order fit (pooled transitions) ───")
        lines.append(
            f"    pure delay: {fit['delay_ms']:7.2f} ± {fit['delay_se_ms']:.2f} ms"
        )
        lines.append(
            f"    τ:          {fit['tau_ms']:7.2f} ± {fit['tau_se_ms']:.2f} ms"
        )
        lines.append(
            f"    fit R²={fit['r2']:.4f}  (N={fit['n_points']} samples "
            f"from {fit['n_transitions']} transitions)"
        )

    return (lines, out_csv)


def _multi_servo_breakdown(df, schedule, csv_path: Path):
    lines: List[str] = []
    lines.append("─── multi_servo breakdown ───────────────────────────")

    # detect mode من schedule
    if not schedule:
        return (lines, None)
    sample = schedule[0]
    if "applied_servos" in sample and sample["applied_servos"] == "all":
        mode = "synchronous"
    elif "applied_servos" in sample and sample["applied_servos"] == "target_only":
        mode = "single_with_witnesses"
    elif "active_position_in_target" in sample:
        mode = "cascaded"
    else:
        mode = "unknown"
    lines.append(f"  detected mode: {mode}")

    servo_ids = sorted(df["servo_idx"].unique().tolist())
    arrs_by_servo = {}
    for s in servo_ids:
        a = _per_servo_arrays(df, int(s))
        if a is not None:
            arrs_by_servo[int(s)] = a

    # auto-align باستخدام أوّل سيرفو يُظهر cmd transition للقيمة المتوقّعة
    pat_t0 = None
    if "cmd_to" in sample and "t_edge_s" in sample:
        expected_first_target = float(sample["cmd_to"])
        sched_first_t = float(sample["t_edge_s"])
        for s in servo_ids:
            off = _detect_pattern_offset(df, s, expected_first_target,
                                          sched_first_t)
            if off is not None:
                pat_t0 = off
                break
    elif "t_active_start_s" in sample:
        # cascaded: أوّل سيرفو يأخذ +amp في slot 0
        expected_first_target = float(sample.get("cmd_active", 0.0))
        sched_first_t = float(sample["t_active_start_s"])
        for s in servo_ids:
            off = _detect_pattern_offset(df, s, expected_first_target,
                                          sched_first_t)
            if off is not None:
                pat_t0 = off
                break
    if pat_t0 is None:
        pat_t0 = 0.0
    lines.append(f"  pattern alignment: csv_t = sched_t + {pat_t0:.3f}s")

    if mode == "synchronous":
        # لكل edge: delay لكل سيرفو (كلهم تلقّوا الأمر) → phase spread
        spreads = []
        for ev in schedule:
            t0 = float(ev["t_edge_s"]) + pat_t0
            y_target = float(ev["cmd_to"])
            servo_delays = []
            for s, (t, cmd, fb) in arrs_by_servo.items():
                # y_init قياسي لكل سيرفو على حدة
                y_init_meas = _measure_y_init(t, fb, t0,
                                              pre_window_s=0.10,
                                              fallback=-y_target)
                m = _edge_metrics(t, fb, t0, y_init_meas, y_target, t0 + 1.0)
                if m is not None:
                    servo_delays.append((s, m["delay_ms"]))
            if len(servo_delays) >= 2:
                ds = [d for _, d in servo_delays if not np.isnan(d)]
                if len(ds) >= 2:
                    spread = max(ds) - min(ds)
                    spreads.append(spread)
        if spreads:
            lines.append(f"  phase spread (max-min delay across servos):")
            lines.append("    " + _summarize(spreads, "spread (ms)"))

    elif mode == "single_with_witnesses":
        # cross-talk: max |fb| من السيرفوهات غير المستهدفة أثناء الـ pattern
        # الـ target غير معروف من schedule هنا — نستدلّ من cmd range
        cmd_ranges = [(s, float(df[df.servo_idx == s].cmd_deg.max() -
                                 df[df.servo_idx == s].cmd_deg.min()))
                      for s in servo_ids]
        target_servo = max(cmd_ranges, key=lambda x: x[1])[0]
        witnesses = [s for s in servo_ids if s != target_servo]
        lines.append(f"  target: servo#{target_servo}, witnesses: {witnesses}")
        for w in witnesses:
            if w not in arrs_by_servo:
                continue
            _, _, fb = arrs_by_servo[w]
            if len(fb) == 0:
                continue
            mn = float(np.min(fb))
            mx = float(np.max(fb))
            lines.append(f"    witness servo#{w}: fb range [{mn:+.3f}, "
                         f"{mx:+.3f}]°  max|fb|={max(abs(mn), abs(mx)):.3f}°")

    elif mode == "cascaded":
        lines.append("  cascaded analysis:")
        windows = [ev for ev in schedule]
        n_stalled = 0
        for ev in windows[:8]:
            t0 = float(ev["t_active_start_s"]) + pat_t0
            t1 = float(ev["t_return_end_s"]) + pat_t0
            mask_window = []
            stalled_servos = []
            for s in servo_ids:
                if s not in arrs_by_servo:
                    continue
                t, _, fb = arrs_by_servo[s]
                sel = (t >= t0) & (t <= t1)
                if sel.sum() >= 1:
                    mask_window.append((s, float(np.max(np.abs(fb[sel])))))
                else:
                    # تحقّق هل fb وصل أصلاً في هذه النافذة (raw)
                    g_raw = df[df["servo_idx"] == s]
                    if "t_fb_arrival_s" in g_raw.columns:
                        ta = g_raw["t_fb_arrival_s"].dropna()
                        if len(ta) and ta.max() < t0:
                            stalled_servos.append(s)
            mask_window.sort(key=lambda x: -x[1])
            top = " ".join(f"s#{s}={v:.2f}" for s, v in mask_window[:4])
            tag = f"pos={ev['active_position_in_target']}"
            if not mask_window and stalled_servos:
                lines.append(f"    window@{t0:.1f}s..{t1:.1f}s {tag}: "
                             f"⚠ NO FRESH FB (stalled — last fb @ "
                             f"{df['t_fb_arrival_s'].max():.2f}s)")
                n_stalled += 1
            else:
                lines.append(f"    window@{t0:.1f}s..{t1:.1f}s {tag}: "
                             f"max|fb| → {top}")
        if n_stalled:
            lines.append(f"  ⚠ {n_stalled}/{len(windows)} windows had no "
                         f"fresh feedback — bus or USB stalled.")

    return (lines, None)


# ─── Entry point ────────────────────────────────────────────────────────────

def analyze(csv_path: Path, pattern_name: str = "",
            pattern_desc: str = "", cfg: Optional[dict] = None,
            schedule: Optional[List[Dict]] = None,
            output_html: bool = True) -> Path:
    """تحليل CSV من /direct.

    schedule: قائمة dicts تصف الـ cells/edges داخل الـ pattern (إن توفّرت).
              تُستخدم لـ patterns مثل step_matrix و repeatability لإنتاج
              per-cell breakdown.
    """
    csv_path = Path(csv_path)
    df = pd.read_csv(csv_path)
    need = {"t_s", "servo_idx", "cmd_deg", "fb_deg"}
    if not need.issubset(df.columns):
        raise ValueError(f"CSV missing cols: {need - set(df.columns)}")

    lines: List[str] = []
    lines.append("═══════════════════════════════════════════════════════")
    lines.append(f"  /direct analysis — {pattern_name or '(unknown)'}")
    if pattern_desc:
        lines.append(f"  {pattern_desc}")
    lines.append(f"  csv: {csv_path.name}")
    lines.append("═══════════════════════════════════════════════════════")

    try:
        import plotly  # noqa: F401
        plotly_ok = True
    except ImportError:
        plotly_ok = False
        output_html = False

    figs = []

    # هل CSV جديد (يحتوي t_fb_arrival_s) أم قديم؟
    # نظام الزمن:
    #   t (cmd time) = t_s — loop time. ندري متى تم إرسال الـ cmd.
    #   t_fb (arrival time) = t_fb_arrival_s — متى وصل CAN frame فعلياً.
    # عند توفّر t_fb_arrival_s نقيس delay/τ/settling بدقّة أعلى (نزيل
    # forward-fill latency من القياس).
    use_arrival = "t_fb_arrival_s" in df.columns
    if use_arrival:
        lines.append("  (using t_fb_arrival_s for fb timing — high-accuracy)")
    else:
        lines.append("  (legacy CSV — fb timing = loop time, includes forward-fill latency)")

    for servo_idx, g in df.groupby("servo_idx"):
        g = g.sort_values("t_s").reset_index(drop=True)
        g_fb = g.dropna(subset=["fb_deg"]).reset_index(drop=True)
        node_id = str(g["node_id"].iloc[0]) if "node_id" in g.columns else "?"
        if len(g_fb) < 5:
            lines.append(f"\nservo#{servo_idx} ({node_id}): insufficient fb "
                         f"({len(g_fb)} samples)")
            continue
        t = g_fb["t_s"].to_numpy(dtype=float)
        cmd = g_fb["cmd_deg"].to_numpy(dtype=float)
        fb = g_fb["fb_deg"].to_numpy(dtype=float)
        # arrival time لكل عيّنة لها fb. NaN يُملأ بـ t_s (forward-fill fallback)
        # ليبقى الـ array بنفس الطول والترتيب.
        t_fb = None
        if use_arrival and "t_fb_arrival_s" in g_fb.columns:
            t_fb_raw = g_fb["t_fb_arrival_s"].to_numpy(dtype=float)
            # ايام NaN: استخدم t (loop) بدلها — لا يضرّ القياس لأن delay عند
            # تلك العيّنات سيكون قريباً من loop-time قياس قديم.
            nan_mask = np.isnan(t_fb_raw)
            t_fb = t_fb_raw.copy()
            t_fb[nan_mask] = t[nan_mask]

        lines.append("")
        lines.append(f"── servo#{servo_idx} (node {node_id}) "
                     f"─────────────────────────")
        lines.append(f"  samples: {len(g_fb)}   "
                     f"cmd∈[{cmd.min():+.2f},{cmd.max():+.2f}]°   "
                     f"fb∈[{fb.min():+.2f},{fb.max():+.2f}]°")
        # slew_max يَحتاج timestamps حقيقيّة لـ fb arrivals مع إزالة المكرّرات،
        # وإلا dt = loop period (5ms) ولا يوجد تغيّر في fb بين معظم العيّنات.
        # نستخدم _per_servo_arrays الذي يقوم بـ dedup لـ t_fb_arrival_s.
        slew_arrs = _per_servo_arrays(df, int(servo_idx))
        slew_val = _slew_max(slew_arrs[0], slew_arrs[2]) if slew_arrs else 0.0
        lines.append(f"  slew_max: {slew_val:.1f} °/s")

        # step metrics if edges detected
        edges = _find_step_edges(cmd, threshold=0.5)
        if edges:
            lines.append(f"  step edges: {len(edges)}")
            agg_td, agg_tau, agg_os = [], [], []
            for e in edges[:8]:
                m = _step_metrics(t, cmd, fb, e, t_fb=t_fb)
                if m is None:
                    continue
                lines.append(
                    f"    edge@{m['t_edge_s']:.2f}s Δ={m['step_size_deg']:+.1f}°  "
                    f"delay={m['transport_delay_ms']:.1f}ms  "
                    f"τ={m['tau_ms']:.1f}ms  "
                    f"OS={m['overshoot_pct']:.1f}%  "
                    f"t_settle={m['settling_ms']:.0f}ms  "
                    f"err={m['final_err_deg']:+.3f}°")
                if not np.isnan(m["transport_delay_ms"]):
                    agg_td.append(m["transport_delay_ms"])
                if not np.isnan(m["tau_ms"]):
                    agg_tau.append(m["tau_ms"])
                agg_os.append(m["overshoot_pct"])
            if agg_td:
                lines.append(f"  avg transport_delay={np.mean(agg_td):.2f}ms  "
                             f"avg τ={np.mean(agg_tau):.2f}ms  "
                             f"avg OS={np.mean(agg_os):.2f}%")

        bode = _bode(t, cmd, fb) if pattern_name == "freq_sweep" else None
        if bode is not None:
            lines.append(f"  bandwidth (-3dB): {bode['bandwidth_hz']:.2f} Hz")

        hys = _hysteresis(cmd, fb) if pattern_name in ("backlash", "ramp") else None
        if hys is not None and "backlash_deg" in hys:
            lines.append(f"  backlash: {hys['backlash_deg']:.3f}°")

        if output_html and plotly_ok:
            figs.append(_build_plot(int(servo_idx), node_id, t, cmd, fb,
                                    bode if bode is not None else None,
                                    hys if hys is not None and
                                    "bin_centers" in hys else None))

    # ─── Phase 1: per-cell breakdown (إن توفّرت schedule) ─────────────
    schedule_csv_path: Optional[Path] = None
    if schedule and pattern_name in (
        "step_matrix", "repeatability", "multi_servo",
        "linearity", "hold_drift", "rate_limit_verify",
        "dead_band", "stiction", "cold_start", "endurance",
        "staircase", "mech_limits", "firmware_audit",
        "preflight_check", "wiring_audit", "fault_scan",
    ):
        breakdown_lines, schedule_csv_path = _matrix_breakdown(
            df=df,
            schedule=schedule,
            pattern_name=pattern_name,
            csv_path=csv_path,
            cfg=cfg,
        )
        if breakdown_lines:
            lines.append("")
            lines.extend(breakdown_lines)

    # ─── write outputs ───────────────────────────────────────────────────
    stem = csv_path.with_suffix("")
    metrics_path = Path(f"{stem}.metrics.txt")
    metrics_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"[analysis] metrics: {metrics_path}")
    if schedule_csv_path is not None:
        print(f"[analysis] per-cell CSV: {schedule_csv_path}")
    for ln in lines:
        print(ln)

    if output_html and figs:
        html_path = Path(f"{stem}.plot.html")
        # Build a self-contained HTML page with:
        #  1) a structured numerical-metrics table parsed from `lines`
        #  2) the raw textual report as a <pre> block (full fidelity)
        #  3) all Plotly figures concatenated below.
        table_html = _direct_metrics_table_from_lines(lines)
        _raw_report_text = "\n".join(lines)
        pre_html = (
            '<details class="card"><summary style="cursor:pointer;'
            'font-weight:700">Raw textual report</summary>'
            '<pre style="white-space:pre-wrap;font-size:.85rem;'
            'background:#f7f7f9;padding:10px;border-radius:4px">'
            f'{html_escape(_raw_report_text)}</pre></details>'
        )
        fig_parts = []
        for i, fig in enumerate(figs):
            fig_parts.append(fig.to_html(full_html=False,
                                         include_plotlyjs=("cdn" if i == 0 else False)))
        figs_html = '\n<hr/>\n'.join(fig_parts)
        page = (
            '<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8">'
            f'<title>/direct — {csv_path.name}</title>'
            '<style>body{font-family:-apple-system,Segoe UI,Roboto,sans-serif;'
            'margin:16px;background:#fafafa;color:#222}'
            '.card{background:#fff;border:1px solid #ddd;border-radius:6px;'
            'padding:12px;margin-bottom:16px;box-shadow:0 1px 2px rgba(0,0,0,.04)}'
            'table{border-collapse:collapse;width:100%}'
            'th,td{padding:6px 10px;border-bottom:1px solid #eee;text-align:left}'
            'th{background:#f3f4f6;font-weight:600}'
            '.num{text-align:right;font-variant-numeric:tabular-nums;font-weight:600}'
            '.unit{color:#888;font-size:.85rem}'
            '.cat{background:#f0f4f8;font-weight:700}'
            '</style></head><body>'
            f'<h1>/direct — {html_escape(csv_path.name)}</h1>'
            f'{table_html}{pre_html}'
            f'<div class="card">{figs_html}</div></body></html>'
        )
        html_path.write_text(page, encoding="utf-8")
        print(f"[analysis] plot:    {html_path}")
        return html_path
    return metrics_path


# ─── Numerical metrics table parser (from the textual `lines`) ──────────
_NUM_RE = re.compile(r'^([A-Za-z][\w \-/().·²³µ%°]+?):\s*([+-]?[\d.]+(?:[eE][+-]?\d+)?)\s*([A-Za-zµ°/%·²³]*)\s*$')

def _direct_metrics_table_from_lines(lines):
    """Parse "key: value [unit]" patterns from the textual report and
    render them as an HTML table. Header lines (starting with ─, =, or
    not containing ':') become category separators."""
    rows = []
    for raw in lines:
        s = raw.rstrip()
        if not s.strip():
            continue
        stripped = s.strip()
        # Section heading: lines starting with non-alphanumeric markers
        if stripped[0] in ('─', '=', '#', '*', '▶') or stripped.startswith('---'):
            label = stripped.lstrip('─=#*▶- ').strip()
            if label:
                rows.append((label, None, None))
            continue
        # Subheading like "[servo 0 — node 5]"
        if stripped.startswith('[') and stripped.endswith(']'):
            rows.append((stripped[1:-1], None, None))
            continue
        # "key: value unit" or "key: value"
        m = _NUM_RE.match(stripped)
        if m:
            label, val, unit = m.group(1).strip(), m.group(2), m.group(3) or ""
            rows.append((label, val, unit))
            continue
        # Lines with " : " but composite — show as plain text row
        if ':' in stripped:
            k, _, v = stripped.partition(':')
            if k.strip() and v.strip():
                rows.append((k.strip(), v.strip(), ""))
                continue
    if not rows:
        return ""
    body = ""
    for label, val, unit in rows:
        if val is None:
            body += (f'<tr class="cat"><td colspan="3" style="padding:8px 6px">'
                     f'■ {html_escape(label)}</td></tr>')
        else:
            body += (f'<tr><td style="font-family:ui-monospace,monospace;'
                     f'font-size:.85rem">{html_escape(label)}</td>'
                     f'<td class="num">{html_escape(val)}</td>'
                     f'<td class="unit">{html_escape(unit)}</td></tr>')
    return ('<div class="card"><h2 style="margin-top:0">📊 Parsed Numerical Metrics</h2>'
            '<table><thead><tr><th>Metric</th><th style="text-align:right">Value</th>'
            f'<th>Unit</th></tr></thead><tbody>{body}</tbody></table></div>')


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("csv", type=Path, help="direct_*.csv from direct_runner")
    p.add_argument("--pattern", default="", help="pattern name hint")
    args = p.parse_args()
    try:
        analyze(args.csv, pattern_name=args.pattern)
        return 0
    except Exception as e:
        print(f"[analysis] FAIL: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
