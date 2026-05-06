#!/usr/bin/env python3
"""/lab post-processing — full flight + servo tracking analysis.

Consumes CSVs produced by ``lab_runner.py``:
  - ``lab_can_YYYYMMDD_HHMMSS.csv``  — CAN traffic (cmd + fb rows)
  - ``lab_sim_YYYYMMDD_HHMMSS.csv``  — sim flight trace (same schema as /sitl)

Strategy:
  1. Delegates flight analysis to ``sitl_analysis.analyze_sitl_csv`` —
     produces the full professional HTML report (trajectory, attitude,
     aero, forces, control, actuators, scoring, diagnostics).
  2. Prepends a `/lab`-specific **Servo tracking quality** section with
     per-servo tracking, delay, backlash, timing health (built from CAN
     log — unique to /lab).
  3. Merges both into a single ``<stem>.report.html`` in lab/results/.

Standalone use:
    python3 lab_analysis.py                               # latest CAN log
    python3 lab_analysis.py <path/to/lab_can_*.csv>       # specific file
    python3 lab_analysis.py --no-open                     # skip browser open
"""

from __future__ import annotations

import argparse
import sys
import webbrowser
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import pandas as pd

try:
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots
except ImportError:
    print("ERROR: plotly is required — run: pip install plotly", file=sys.stderr)
    sys.exit(1)

# Make /sitl importable so we can reuse sitl_analysis (full flight report).
_LAB_DIR = Path(__file__).resolve().parent
_SITL_DIR = _LAB_DIR.parent / "sitl"
if str(_SITL_DIR) not in sys.path:
    sys.path.insert(0, str(_SITL_DIR))

try:
    import sitl_analysis as _sitl  # type: ignore
    _SITL_AVAILABLE = True
except Exception as _e:
    print(f"[lab-analysis] WARN: sitl_analysis unavailable ({_e}); "
          f"flight section will be skipped.", file=sys.stderr)
    _sitl = None
    _SITL_AVAILABLE = False


# ─────────────────────────────────────────────────────────────────────────────
#  Data loading
# ─────────────────────────────────────────────────────────────────────────────

def _latest_can_csv(results_dir: Path) -> Path:
    files = sorted(results_dir.glob("lab_can_*.csv"))
    if not files:
        raise FileNotFoundError(f"no lab_can_*.csv in {results_dir}")
    return files[-1]


def _sibling_sim_csv(can_csv: Path) -> Optional[Path]:
    """Try to find the sim CSV with matching timestamp.

    Tries lab_sim_*.csv first (current naming), then lab_bridge_*.csv
    (legacy). Returns None if neither exists.
    """
    ts = can_csv.stem.replace("lab_can_", "")
    for prefix in ("lab_sim_", "lab_bridge_"):
        cand = can_csv.parent / f"{prefix}{ts}.csv"
        if cand.exists():
            return cand
    return None


def _load_can(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    need = {"t_s", "kind", "servo_idx", "value_deg"}
    miss = need - set(df.columns)
    if miss:
        raise ValueError(f"{path.name} missing columns: {miss}")
    df = df.sort_values("t_s").reset_index(drop=True)
    return df


# ─────────────────────────────────────────────────────────────────────────────
#  Per-servo resampling (cmd / fb → common uniform grid)
# ─────────────────────────────────────────────────────────────────────────────

def _resample_cmd_fb(df_servo: pd.DataFrame, fs: float = 200.0
                     ) -> Optional[Dict[str, np.ndarray]]:
    """Return cmd & fb on common uniform time grid (wall-clock seconds).

    Uses zero-order-hold (step) for cmd (since cmd is issued discretely) and
    linear interpolation for fb. Returns None if not enough samples.
    """
    cmd = df_servo[df_servo["kind"] == "cmd"][["t_s", "value_deg"]].dropna()
    fb = df_servo[df_servo["kind"] == "fb"][["t_s", "value_deg"]].dropna()
    if len(cmd) < 5 or len(fb) < 5:
        return None

    t0 = max(cmd["t_s"].iloc[0], fb["t_s"].iloc[0])
    t1 = min(cmd["t_s"].iloc[-1], fb["t_s"].iloc[-1])
    if t1 - t0 < 0.1:
        return None

    n = int((t1 - t0) * fs)
    grid = t0 + np.arange(n) / fs

    # cmd: zero-order-hold (searchsorted → last cmd at or before grid[i])
    t_cmd = cmd["t_s"].to_numpy()
    v_cmd = cmd["value_deg"].to_numpy()
    idx = np.searchsorted(t_cmd, grid, side="right") - 1
    idx = np.clip(idx, 0, len(v_cmd) - 1)
    cmd_u = v_cmd[idx]

    # fb: linear interp
    fb_u = np.interp(grid, fb["t_s"].to_numpy(), fb["value_deg"].to_numpy())

    return {"t": grid, "cmd": cmd_u, "fb": fb_u,
            "n_cmd_raw": len(cmd), "n_fb_raw": len(fb)}


# ─────────────────────────────────────────────────────────────────────────────
#  Metrics
# ─────────────────────────────────────────────────────────────────────────────

def _tracking_metrics(t: np.ndarray, cmd: np.ndarray, fb: np.ndarray
                      ) -> Dict[str, float]:
    err = fb - cmd
    rmse = float(np.sqrt(np.mean(err**2)))
    max_abs = float(np.max(np.abs(err)))
    mean = float(np.mean(err))
    return {
        "rmse_deg": rmse,
        "max_abs_err_deg": max_abs,
        "bias_deg": mean,
    }


def _effective_delay_ms(t: np.ndarray, cmd: np.ndarray, fb: np.ndarray,
                        max_lag_s: float = 0.3) -> float:
    """Estimate transport delay via cross-correlation argmax.

    Positive value = fb lags cmd by this many milliseconds.
    """
    if len(t) < 20:
        return float("nan")
    dt = float(np.median(np.diff(t)))
    if dt <= 0:
        return float("nan")
    c = cmd - cmd.mean()
    y = fb - fb.mean()
    # normalize
    c_norm = np.std(c)
    y_norm = np.std(y)
    if c_norm < 1e-6 or y_norm < 1e-6:
        return float("nan")
    # full correlation; positive lag = fb shifted right = fb lags cmd
    max_lag = int(max_lag_s / dt)
    lags = np.arange(-max_lag, max_lag + 1)
    xc = np.empty(len(lags))
    for i, lag in enumerate(lags):
        if lag >= 0:
            xc[i] = np.dot(c[:len(c) - lag], y[lag:]) / (len(c) - lag)
        else:
            xc[i] = np.dot(c[-lag:], y[:len(y) + lag]) / (len(c) + lag)
    xc /= (c_norm * y_norm)
    best = lags[int(np.argmax(xc))]
    return float(best * dt * 1000.0)


def _slew_max_deg_per_s(t: np.ndarray, fb: np.ndarray) -> float:
    if len(fb) < 3:
        return 0.0
    dy = np.diff(fb)
    dt = np.diff(t)
    dt[dt <= 0] = np.inf
    rate = np.abs(dy / dt)
    # median filter (window 5) to suppress single-sample glitches
    k = 2
    if len(rate) > 2 * k + 1:
        filt = np.empty_like(rate)
        for i in range(len(rate)):
            lo, hi = max(0, i - k), min(len(rate), i + k + 1)
            filt[i] = np.median(rate[lo:hi])
        rate = filt
    return float(rate.max())


def _backlash_deg(cmd: np.ndarray, fb: np.ndarray, n_bins: int = 32) -> float:
    if len(cmd) < 20:
        return 0.0
    dc = np.diff(cmd, prepend=cmd[0])
    up = dc > 1e-4
    dn = dc < -1e-4
    if not (np.any(up) and np.any(dn)):
        return 0.0
    lo, hi = float(cmd.min()), float(cmd.max())
    if hi - lo < 1e-3:
        return 0.0
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
    return float(np.max(np.abs(diff))) if len(diff) else 0.0


def _fb_health(df_servo: pd.DataFrame) -> Dict[str, float]:
    fb = df_servo[df_servo["kind"] == "fb"]["t_s"].to_numpy()
    if len(fb) < 5:
        return {"fb_rate_hz": 0.0, "fb_gap_max_ms": float("nan"),
                "fb_gap_p99_ms": float("nan")}
    d = np.diff(fb)
    d = d[d > 0]
    if len(d) == 0:
        return {"fb_rate_hz": 0.0, "fb_gap_max_ms": float("nan"),
                "fb_gap_p99_ms": float("nan")}
    return {
        "fb_rate_hz": 1.0 / float(np.median(d)),
        "fb_gap_max_ms": float(d.max() * 1000.0),
        "fb_gap_p99_ms": float(np.percentile(d, 99) * 1000.0),
    }


# ─────────────────────────────────────────────────────────────────────────────
#  Plot builders
# ─────────────────────────────────────────────────────────────────────────────

_COLORS_CMD = "#d62728"
_COLORS_FB = "#1f77b4"
_COLORS_ERR = "#ff7f0e"


def _plot_servo_tracking(res: Dict, meta: Dict) -> go.Figure:
    t = res["t"] - res["t"][0]
    cmd, fb = res["cmd"], res["fb"]
    err = fb - cmd

    fig = make_subplots(
        rows=2, cols=1, shared_xaxes=True,
        row_heights=[0.65, 0.35],
        vertical_spacing=0.06,
        subplot_titles=[
            f"servo#{meta['idx']} (node 0x{meta['node_id']:02X}) — "
            f"cmd vs fb  |  RMSE={meta['rmse']:.3f}°  "
            f"delay={meta['delay_ms']:.1f}ms",
            "tracking error (fb − cmd)",
        ],
    )
    fig.add_trace(go.Scatter(x=t, y=cmd, name="cmd",
                             line=dict(color=_COLORS_CMD, width=1.4)),
                  row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=fb, name="fb",
                             line=dict(color=_COLORS_FB, width=1.4)),
                  row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=err, name="err",
                             line=dict(color=_COLORS_ERR, width=1.0),
                             showlegend=False),
                  row=2, col=1)
    fig.add_hline(y=0, row=2, col=1, line=dict(color="#888", width=0.5))

    fig.update_xaxes(title_text="time [s]", row=2, col=1)
    fig.update_yaxes(title_text="angle [°]", row=1, col=1)
    fig.update_yaxes(title_text="err [°]", row=2, col=1)
    fig.update_layout(
        height=420, width=1100, template="plotly_white",
        hovermode="x unified",
        margin=dict(l=60, r=30, t=60, b=50),
    )
    return fig


def _plot_hysteresis(res: Dict, meta: Dict) -> go.Figure:
    cmd, fb = res["cmd"], res["fb"]
    dc = np.diff(cmd, prepend=cmd[0])
    up = dc > 1e-4
    dn = dc < -1e-4
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=cmd[up], y=fb[up], name="cmd↑",
                             mode="markers",
                             marker=dict(color="#9467bd", size=3, opacity=0.5)))
    fig.add_trace(go.Scatter(x=cmd[dn], y=fb[dn], name="cmd↓",
                             mode="markers",
                             marker=dict(color="#8c564b", size=3, opacity=0.5)))
    lo, hi = float(cmd.min()), float(cmd.max())
    fig.add_trace(go.Scatter(x=[lo, hi], y=[lo, hi], name="ideal 1:1",
                             line=dict(color="#888", width=1, dash="dot"),
                             showlegend=True))
    fig.update_layout(
        title=f"servo#{meta['idx']} hysteresis  |  "
              f"backlash={meta['backlash']:.3f}°",
        xaxis_title="cmd [°]", yaxis_title="fb [°]",
        height=380, width=560,
        template="plotly_white",
        margin=dict(l=60, r=30, t=50, b=50),
    )
    return fig


def _plot_overview(per_servo: List[Dict]) -> go.Figure:
    """Small bar charts summarizing per-servo metrics."""
    idx = [m["idx"] for m in per_servo]
    names = [f"#{i}" for i in idx]
    fig = make_subplots(
        rows=1, cols=4,
        subplot_titles=("RMSE [°]", "max |err| [°]",
                        "delay [ms]", "backlash [°]"),
        horizontal_spacing=0.08,
    )
    fig.add_trace(go.Bar(x=names, y=[m["rmse"] for m in per_servo],
                         marker_color="#1f77b4"), row=1, col=1)
    fig.add_trace(go.Bar(x=names, y=[m["max_err"] for m in per_servo],
                         marker_color="#d62728"), row=1, col=2)
    fig.add_trace(go.Bar(x=names, y=[m["delay_ms"] for m in per_servo],
                         marker_color="#2ca02c"), row=1, col=3)
    fig.add_trace(go.Bar(x=names, y=[m["backlash"] for m in per_servo],
                         marker_color="#9467bd"), row=1, col=4)
    fig.update_layout(
        height=260, width=1100, template="plotly_white",
        showlegend=False,
        margin=dict(l=40, r=30, t=50, b=40),
    )
    return fig


def _plot_flight_summary(sim_df: pd.DataFrame) -> Optional[go.Figure]:
    """Compact flight trace (altitude + attitude + servos) from sim CSV."""
    if sim_df is None or len(sim_df) < 10:
        return None
    t_col = "time" if "time" in sim_df.columns else "time_s"
    if t_col not in sim_df.columns:
        return None
    t = sim_df[t_col].to_numpy()

    fig = make_subplots(
        rows=3, cols=1, shared_xaxes=True,
        row_heights=[0.33, 0.33, 0.34],
        vertical_spacing=0.06,
        subplot_titles=("altitude [m]", "attitude rates [°/s]",
                        "fins [°]"),
    )
    if "altitude" in sim_df.columns:
        fig.add_trace(go.Scatter(x=t, y=sim_df["altitude"], name="alt",
                                 line=dict(color="#1f77b4")), row=1, col=1)
    for col, color, label in [
        ("omega_x", "#d62728", "ωx"), ("omega_y", "#2ca02c", "ωy"),
        ("omega_z", "#9467bd", "ωz"),
    ]:
        if col in sim_df.columns:
            fig.add_trace(go.Scatter(x=t, y=np.degrees(sim_df[col]),
                                     name=label,
                                     line=dict(color=color, width=1.2)),
                          row=2, col=1)
    for i, color in enumerate(["#1f77b4", "#d62728", "#2ca02c", "#9467bd"]):
        col = f"fin_cmd_{i+1}"
        if col in sim_df.columns:
            fig.add_trace(go.Scatter(x=t, y=np.degrees(sim_df[col]),
                                     name=f"fin{i+1}",
                                     line=dict(color=color, width=1.0)),
                          row=3, col=1)
    fig.update_xaxes(title_text="time [s]", row=3, col=1)
    fig.update_layout(
        height=620, width=1100, template="plotly_white",
        hovermode="x unified",
        margin=dict(l=60, r=30, t=50, b=50),
    )
    return fig


# ─────────────────────────────────────────────────────────────────────────────
#  HTML report assembly
# ─────────────────────────────────────────────────────────────────────────────

_SERVO_SECTION_CSS = """
<style>
  .lab-sec { background: var(--card-bg, #fff); padding: 14px 18px;
             border-radius: 8px; margin: 12px 0;
             box-shadow: 0 1px 3px rgba(0,0,0,0.08); }
  .lab-sec h2 { font-size: 17px; color: #1f4e79; margin: 0 0 12px 0; }
  table.lab-metrics { border-collapse: collapse; margin: 8px 0;
                      font-size: 13px; width: 100%; }
  table.lab-metrics th, table.lab-metrics td {
      border: 1px solid #ccc; padding: 5px 10px; text-align: right; }
  table.lab-metrics th { background: #e8f0fa; color: #1f4e79; }
  .lab-ok   { color: #2a7a2a; font-weight: 600; }
  .lab-warn { color: #c67700; font-weight: 600; }
  .lab-fail { color: #b71c1c; font-weight: 600; }
  .lab-hint { color: #666; font-size: 12px; margin: 4px 0 10px 0; }
</style>
"""

_HTML_HEAD_FALLBACK = """<!DOCTYPE html>
<html lang="en"><head>
<meta charset="utf-8">
<title>/lab Report — {title}</title>
<style>
 body {{ font-family: -apple-system, 'Segoe UI', sans-serif;
        max-width: 1200px; margin: 24px auto; padding: 0 16px;
        background: #fafafa; color: #222; }}
 h1 {{ font-size: 22px; border-bottom: 2px solid #1f77b4; padding-bottom: 8px; }}
 h2 {{ font-size: 17px; color: #1f4e79; margin-top: 28px; }}
 .lab-sec {{ background: white; padding: 14px 18px; border-radius: 6px;
             box-shadow: 0 1px 3px rgba(0,0,0,0.06); margin-bottom: 20px; }}
 table.lab-metrics {{ border-collapse: collapse; margin: 12px 0; font-size: 13px; }}
 table.lab-metrics th, table.lab-metrics td {{ border: 1px solid #ccc;
    padding: 5px 10px; text-align: right; }}
 table.lab-metrics th {{ background: #e8f0fa; }}
 .lab-ok   {{ color: #2a7a2a; font-weight: 600; }}
 .lab-warn {{ color: #c67700; font-weight: 600; }}
 .lab-fail {{ color: #b71c1c; font-weight: 600; }}
 .lab-hint {{ color: #666; font-size: 12px; margin: 4px 0 10px 0; }}
</style></head><body>
<h1>/lab analysis report</h1>
<div class="lab-hint">source: <code>{src}</code></div>
"""

_HTML_TAIL_FALLBACK = "</body></html>\n"


def _write_standalone_html(html_path: Path, can_csv: Path,
                           lab_html: str) -> None:
    """Fallback: write a minimal standalone HTML when sim CSV is missing."""
    html = (
        _HTML_HEAD_FALLBACK.format(title=can_csv.stem, src=str(can_csv))
        + lab_html
        + _HTML_TAIL_FALLBACK
    )
    html_path.write_text(html, encoding="utf-8")


def _grade(value: float, warn: float, fail: float, lower_is_better: bool = True
           ) -> str:
    if np.isnan(value):
        return '<span class="lab-warn">—</span>'
    if lower_is_better:
        if value >= fail:
            cls = "lab-fail"
        elif value >= warn:
            cls = "lab-warn"
        else:
            cls = "lab-ok"
    else:
        if value <= fail:
            cls = "lab-fail"
        elif value <= warn:
            cls = "lab-warn"
        else:
            cls = "lab-ok"
    return f'<span class="{cls}">{value:.3f}</span>'


def _render_summary_table(per_servo: List[Dict]) -> str:
    rows = []
    for m in per_servo:
        rows.append(
            "<tr>"
            f"<td>#{m['idx']}</td>"
            f"<td>0x{m['node_id']:02X}</td>"
            f"<td>{m['n_cmd']}</td>"
            f"<td>{m['n_fb']}</td>"
            f"<td>{m['fb_rate']:.1f}</td>"
            f"<td>{m['fb_gap_p99']:.1f}</td>"
            f"<td>{m['cmd_min']:+.2f} … {m['cmd_max']:+.2f}</td>"
            f"<td>{_grade(m['rmse'], 0.5, 2.0)}</td>"
            f"<td>{_grade(m['max_err'], 1.0, 3.0)}</td>"
            f"<td>{_grade(m['delay_ms'], 30.0, 80.0)}</td>"
            f"<td>{_grade(m['backlash'], 0.3, 1.0)}</td>"
            f"<td>{m['slew_max']:.0f}</td>"
            "</tr>"
        )
    return (
        '<table class="lab-metrics"><thead><tr>'
        "<th>servo</th><th>node</th><th>N cmd</th><th>N fb</th>"
        "<th>fb rate [Hz]</th><th>fb gap p99 [ms]</th>"
        "<th>cmd range [°]</th>"
        "<th>RMSE [°]</th><th>max|err| [°]</th>"
        "<th>delay [ms]</th><th>backlash [°]</th>"
        "<th>slew [°/s]</th>"
        "</tr></thead><tbody>"
        + "".join(rows)
        + "</tbody></table>"
    )


# ─────────────────────────────────────────────────────────────────────────────
#  Main analyze
# ─────────────────────────────────────────────────────────────────────────────

def analyze(can_csv: Path, sim_csv: Optional[Path] = None,
            open_browser: bool = True) -> Path:
    can_csv = Path(can_csv)
    df = _load_can(can_csv)
    print(f"[lab-analysis] loaded {can_csv.name}: {len(df)} rows")

    if sim_csv is None:
        sim_csv = _sibling_sim_csv(can_csv)
    sim_df = None
    if sim_csv and sim_csv.exists():
        try:
            sim_df = pd.read_csv(sim_csv)
            print(f"[lab-analysis] sim CSV: {sim_csv.name} "
                  f"({len(sim_df)} rows)")
        except Exception as e:
            print(f"[lab-analysis] sim CSV read failed: {e}")

    per_servo: List[Dict] = []
    servo_figs: List[go.Figure] = []
    hyst_figs: List[go.Figure] = []
    text_lines: List[str] = []

    servo_indices = sorted(df["servo_idx"].dropna().unique().astype(int).tolist())
    for sid in servo_indices:
        dfs = df[df["servo_idx"] == sid]
        node_id = int(dfs["node_id"].iloc[0]) if "node_id" in dfs.columns \
            else (sid + 1)
        res = _resample_cmd_fb(dfs)
        n_cmd_raw = int((dfs["kind"] == "cmd").sum())
        n_fb_raw = int((dfs["kind"] == "fb").sum())
        health = _fb_health(dfs)
        if res is None:
            text_lines.append(f"servo#{sid}: insufficient data "
                              f"(cmd={n_cmd_raw}, fb={n_fb_raw}) — SKIP")
            continue

        t, cmd, fb = res["t"], res["cmd"], res["fb"]
        trk = _tracking_metrics(t, cmd, fb)
        delay_ms = _effective_delay_ms(t, cmd, fb)
        slew = _slew_max_deg_per_s(t, fb)
        bl = _backlash_deg(cmd, fb)

        meta = {
            "idx": sid,
            "node_id": node_id,
            "n_cmd": n_cmd_raw,
            "n_fb": n_fb_raw,
            "fb_rate": health["fb_rate_hz"],
            "fb_gap_p99": health["fb_gap_p99_ms"],
            "cmd_min": float(cmd.min()),
            "cmd_max": float(cmd.max()),
            "rmse": trk["rmse_deg"],
            "max_err": trk["max_abs_err_deg"],
            "bias": trk["bias_deg"],
            "delay_ms": delay_ms,
            "backlash": bl,
            "slew_max": slew,
        }
        per_servo.append(meta)

        servo_figs.append(_plot_servo_tracking(res, meta))
        hyst_figs.append(_plot_hysteresis(res, meta))

        text_lines.append(
            f"servo#{sid} (0x{node_id:02X})  "
            f"N(cmd/fb)={n_cmd_raw}/{n_fb_raw}  "
            f"fb_rate={health['fb_rate_hz']:5.1f}Hz  "
            f"gap_p99={health['fb_gap_p99_ms']:5.1f}ms"
        )
        text_lines.append(
            f"   cmd range: [{cmd.min():+6.2f}, {cmd.max():+6.2f}]°  "
            f"RMSE={trk['rmse_deg']:.3f}°  "
            f"max|err|={trk['max_abs_err_deg']:.3f}°  "
            f"bias={trk['bias_deg']:+.3f}°"
        )
        text_lines.append(
            f"   delay={delay_ms:+.1f}ms  "
            f"backlash={bl:.3f}°  slew_max={slew:.0f}°/s"
        )
        text_lines.append("")

    # ─── Build the /lab-specific CAN tracking section (HTML fragment) ────
    # Whether standalone or injected into sitl_analysis's full flight
    # report, this fragment is self-contained (uses .lab-* CSS classes).
    include_plotlyjs_here = not _SITL_AVAILABLE or sim_csv is None \
        or not Path(sim_csv).exists()
    lab_parts: List[str] = [_SERVO_SECTION_CSS]
    if per_servo:
        lab_parts.append('<div class="lab-sec"><h2>'
                         '🛠 Servo Tracking — CAN (real hardware)</h2>')
        lab_parts.append(
            '<div class="lab-hint">Color coding: '
            '<span class="lab-ok">OK</span> · '
            '<span class="lab-warn">WARN</span> · '
            '<span class="lab-fail">FAIL</span>. '
            'Thresholds — RMSE&lt;0.5°/2°, delay&lt;30/80ms, '
            'backlash&lt;0.3°/1°.</div>')
        lab_parts.append(_render_summary_table(per_servo))
        lab_parts.append(_plot_overview(per_servo).to_html(
            full_html=False,
            include_plotlyjs=("cdn" if include_plotlyjs_here else False)))
        include_plotlyjs_here = False  # already included above
        lab_parts.append("</div>")

    if servo_figs:
        lab_parts.append('<div class="lab-sec"><h2>'
                         'Per-servo time-series (cmd / fb / error)</h2>')
        for fig in servo_figs:
            lab_parts.append(fig.to_html(
                full_html=False,
                include_plotlyjs=("cdn" if include_plotlyjs_here else False)))
            include_plotlyjs_here = False
        lab_parts.append("</div>")

    if hyst_figs:
        lab_parts.append('<div class="lab-sec"><h2>'
                         'Hysteresis (backlash)</h2>')
        lab_parts.append('<div style="display:flex;flex-wrap:wrap;gap:12px">')
        for fig in hyst_figs:
            lab_parts.append('<div>' + fig.to_html(
                full_html=False,
                include_plotlyjs=("cdn" if include_plotlyjs_here else False)
            ) + '</div>')
            include_plotlyjs_here = False
        lab_parts.append("</div></div>")

    lab_html = "\n".join(lab_parts)

    # ─── Assemble final report ───────────────────────────────────────────
    # If /sitl is available and a sim CSV was produced, use the full
    # sitl_analysis flight report (Overview / Trajectory / 3D / Attitude /
    # Aero / Forces / Control / Phase Portrait) and inject the /lab
    # servo-tracking section near the top.  Otherwise, fall back to a
    # standalone minimal /lab HTML.
    stem = can_csv.with_suffix("")
    html_path = Path(f"{stem}.report.html")
    metrics_path = Path(f"{stem}.metrics.txt")

    if _SITL_AVAILABLE and sim_csv is not None and Path(sim_csv).exists():
        try:
            df_sim = _sitl.load_sitl_csv(Path(sim_csv))
            metrics = _sitl.extract_metrics(df_sim, Path(sim_csv))
            scores = _sitl.score_run(metrics)
            diags = _sitl.diagnose(df_sim, metrics)
            recs = _sitl.recommend(metrics, scores, diags)
            full_html = _sitl.generate_html(
                df_sim, metrics, scores, diags, recs, html_path=None)

            # Inject the /lab section just after the first <h1> (title).
            marker = "</h1>"
            pos = full_html.find(marker)
            if pos > 0:
                inject_at = pos + len(marker)
                full_html = (
                    full_html[:inject_at]
                    + '\n<div style="color:#666;font-size:.85rem;'
                      'margin:6px 0 10px 0">+ CAN-bus hardware-in-the-loop '
                      f'trace: <code>{can_csv.name}</code></div>\n'
                    + lab_html
                    + full_html[inject_at:]
                )
            else:
                full_html = full_html.replace(
                    "</body>", lab_html + "\n</body>")

            # Retitle from "M130 SITL Flight Analysis" → "/lab Report"
            full_html = full_html.replace(
                "M130 SITL Flight Analysis",
                "M130 /lab Report — Hardware-in-the-loop")

            html_path.write_text(full_html, encoding="utf-8")
            print_console = True
            try:
                _sitl.print_console_summary(metrics, scores)
            except Exception:
                print_console = False
        except Exception as e:
            print(f"[lab-analysis] flight merge failed ({e}); "
                  f"writing standalone report.")
            _write_standalone_html(html_path, can_csv, lab_html)
    else:
        _write_standalone_html(html_path, can_csv, lab_html)

    # ─── metrics.txt ─────────────────────────────────────────────────────
    header = [
        "═══════════════════════════════════════════════════════",
        f"  /lab analysis — {can_csv.name}",
        "═══════════════════════════════════════════════════════",
        "",
    ]
    metrics_path.write_text(
        "\n".join(header + text_lines) + "\n", encoding="utf-8")

    # ─── console output ──────────────────────────────────────────────────
    for ln in header + text_lines:
        print(ln)
    print(f"[lab-analysis] metrics: {metrics_path}")
    print(f"[lab-analysis] report : {html_path}")

    if open_browser:
        url = f"file://{html_path.resolve()}"
        try:
            opened = webbrowser.open(url)
        except Exception as e:
            opened = False
            print(f"[lab-analysis] webbrowser error: {e}")
        if not opened:
            print(f"[lab-analysis] could NOT open browser automatically.")
            print(f"               open this URL manually:")
            print(f"                 {url}")

    return html_path


# ─────────────────────────────────────────────────────────────────────────────
#  CLI
# ─────────────────────────────────────────────────────────────────────────────

def main() -> int:
    p = argparse.ArgumentParser(
        description="/lab post-processing (tracking, delay, backlash).")
    p.add_argument("csv", type=Path, nargs="?",
                   help="lab_can_*.csv (default: latest)")
    p.add_argument("--sim-csv", type=Path, default=None,
                   help="explicit sim CSV (default: auto-match timestamp)")
    p.add_argument("--no-open", action="store_true",
                   help="don't open report in browser")
    args = p.parse_args()

    results_dir = Path(__file__).resolve().parent / "results"
    csv = args.csv or _latest_can_csv(results_dir)

    try:
        analyze(csv, sim_csv=args.sim_csv, open_browser=not args.no_open)
        return 0
    except Exception as e:
        print(f"[lab-analysis] FAIL: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
