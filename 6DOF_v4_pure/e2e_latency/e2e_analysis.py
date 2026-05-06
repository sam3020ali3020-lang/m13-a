#!/usr/bin/env python3
"""
e2e_analysis.py — End-to-end latency analysis from MAVLink streams
=====================================================================

Computes latency for each pipeline stage from captured CSV files:
  - L_sensor:   IMU sample → vehicle_attitude (HIGHRES_IMU → ATTITUDE)
  - L_mpc:      MPC solve time (from RktGNC.mpc_solve_us)
  - L_actuator: fin command → physical servo position match (SERVO_OUTPUT_RAW → SRV_FB)
  - L_total:    L_sensor + L_mpc + L_actuator

All timestamps are in PX4 HRT (μs) so direct subtraction works.

Usage:
    from e2e_analysis import analyze_directory
    metrics = analyze_directory(Path("results/20260503_180000"))

Or as CLI:
    python3 e2e_analysis.py results/20260503_180000/
"""

from __future__ import annotations

import argparse
import csv
import json
import logging
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger("e2e_analysis")


# ============================================================================
# CSV loaders
# ============================================================================

def _load_csv(path: Path) -> Optional[np.ndarray]:
    """Load CSV with header into structured numpy array. Return None if missing."""
    if not path.exists():
        return None
    try:
        return np.genfromtxt(path, delimiter=",", names=True, dtype=None,
                             encoding="utf-8")
    except (OSError, ValueError) as e:
        logger.warning(f"Failed to load {path}: {e}")
        return None


# ============================================================================
# Latency stage 1: L_sensor (IMU sample → ATTITUDE published)
# ============================================================================

def compute_l_sensor(imu_data: np.ndarray, att_data: np.ndarray) -> Dict:
    """For each ATTITUDE sample, find the most recent HIGHRES_IMU sample
    with time_usec <= attitude time, and compute the delta.

    Returns dict with mean/p50/p99/min/max and the array of latencies (ms).
    """
    if imu_data is None or len(imu_data) < 2 or att_data is None or len(att_data) < 2:
        return {"error": "insufficient data", "latencies_ms": []}

    # IMU: t_boot_us (PX4 HRT μs, 64-bit, from HIGHRES_IMU.time_usec)
    imu_t = np.asarray(imu_data["t_boot_us"], dtype=np.int64)

    # ATTITUDE: t_boot_ms (PX4 HRT ms, 32-bit) — convert to μs
    # The CSV header from save_attitude_csv has "t_boot_ms"
    att_t = np.asarray(att_data["t_boot_ms"], dtype=np.int64) * 1000

    # Sort IMU times for searchsorted (already chronological in practice but be safe)
    imu_t_sorted = np.sort(imu_t)

    # For each att_t, find rightmost imu_t <= att_t
    idx = np.searchsorted(imu_t_sorted, att_t, side="right") - 1
    valid = idx >= 0
    if not np.any(valid):
        return {"error": "no IMU before any ATTITUDE", "latencies_ms": []}

    latencies_us = att_t[valid] - imu_t_sorted[idx[valid]]
    # Discard impossibly large gaps (e.g., wrap-around or stream restart): > 5 sec
    latencies_us = latencies_us[(latencies_us >= 0) & (latencies_us < 5_000_000)]
    latencies_ms = latencies_us / 1000.0

    if len(latencies_ms) == 0:
        return {"error": "no valid pairs", "latencies_ms": []}

    return _stats(latencies_ms, "ms")


# ============================================================================
# Latency stage 2: L_mpc (MPC solve time, direct from RktGNC)
# ============================================================================

def compute_l_mpc(gnc_data: np.ndarray) -> Dict:
    """Pull mpc_solve_us directly from RktGNC stream (data[47] decoded
    by e2e_reader)."""
    if gnc_data is None or len(gnc_data) < 5:
        return {"error": "no RktGNC data", "latencies_ms": []}

    if "mpc_solve_us" not in gnc_data.dtype.names:
        return {"error": "mpc_solve_us field missing in CSV", "latencies_ms": []}

    mpc_us = np.asarray(gnc_data["mpc_solve_us"], dtype=np.float64)
    # Filter out zeros (= MPC didn't run yet) and obvious garbage
    mpc_us = mpc_us[(mpc_us > 0) & (mpc_us < 1_000_000)]  # < 1s sanity

    if len(mpc_us) < 5:
        return {"error": "insufficient MPC solve samples (MPC may be idle)",
                "latencies_ms": []}

    return _stats(mpc_us / 1000.0, "ms")


# ============================================================================
# Latency stage 3: L_actuator (fin cmd → physical servo fb match)
# ============================================================================

def compute_l_actuator(srv_fb_data: np.ndarray,
                        tolerance_deg: float = 0.5,
                        min_step_deg: float = 1.0) -> Dict:
    """For each significant cmd change in SRV_FB stream, measure how long
    until fb tracks the new cmd within tolerance_deg.

    Args:
        srv_fb_data: structured array from servo_fb.csv
        tolerance_deg: |fb - cmd| < tol → "arrived"
        min_step_deg: minimum cmd change magnitude to consider a "step"
                      (filters out noise/idle)
    """
    if srv_fb_data is None or len(srv_fb_data) < 10:
        return {"error": "insufficient SRV_FB data", "latencies_ms": []}

    # Use servo 0 as primary (fin1) — same logic applies to others
    t_us = np.asarray(srv_fb_data["time_usec"], dtype=np.int64)
    cmd = np.asarray(srv_fb_data["cmd0"], dtype=np.float64)
    fb  = np.asarray(srv_fb_data["fb0"], dtype=np.float64)

    if len(t_us) < 10:
        return {"error": "too few SRV_FB samples", "latencies_ms": []}

    # Detect "command edges": when |cmd[i] - cmd[i-1]| > min_step_deg
    cmd_diff = np.abs(np.diff(cmd))
    edge_idx = np.where(cmd_diff > min_step_deg)[0]  # index BEFORE the edge

    if len(edge_idx) < 1:
        return {"error": "no significant fin movement detected — MPC may be idle",
                "latencies_ms": [], "edges_detected": 0}

    latencies_us = []
    skipped_no_settle = 0

    for ei in edge_idx:
        target = cmd[ei + 1]   # new cmd after the edge
        t_cmd = t_us[ei + 1]
        # search forward from ei+1 for first sample where |fb - target| < tol
        for j in range(ei + 1, min(len(t_us), ei + 200)):
            if abs(fb[j] - target) < tolerance_deg:
                latencies_us.append(t_us[j] - t_cmd)
                break
        else:
            skipped_no_settle += 1

    if len(latencies_us) < 1:
        return {"error": "no fb arrived within tolerance",
                "latencies_ms": [], "edges_detected": len(edge_idx)}

    latencies_ms = np.asarray(latencies_us) / 1000.0
    # Sanity: discard >2s (pathological)
    latencies_ms = latencies_ms[(latencies_ms >= 0) & (latencies_ms < 2000)]

    result = _stats(latencies_ms, "ms")
    result["edges_detected"] = int(len(edge_idx))
    result["edges_settled"] = int(len(latencies_ms))
    result["skipped_no_settle"] = int(skipped_no_settle)
    return result


# ============================================================================
# Stream health check
# ============================================================================

def compute_stream_rates(imu_data, att_data, gnc_data, srv_fb_data,
                          servo_raw_data) -> Dict:
    """Compute observed Hz for each stream from t_wall_s span."""
    rates = {}

    def _rate(arr, t_field="t_wall_s"):
        if arr is None or len(arr) < 5:
            return 0.0
        try:
            t = np.asarray(arr[t_field], dtype=np.float64)
            duration = float(t[-1] - t[0])
            if duration <= 0:
                return 0.0
            return float(len(arr) / duration)
        except (KeyError, ValueError):
            return 0.0

    rates["imu_hz"] = round(_rate(imu_data), 2)
    rates["attitude_hz"] = round(_rate(att_data), 2)
    rates["rktgnc_hz"] = round(_rate(gnc_data), 2)
    rates["srv_fb_hz"] = round(_rate(srv_fb_data), 2)
    rates["servo_raw_hz"] = round(_rate(servo_raw_data), 2)

    rates["imu_samples"] = int(len(imu_data)) if imu_data is not None else 0
    rates["attitude_samples"] = int(len(att_data)) if att_data is not None else 0
    rates["rktgnc_samples"] = int(len(gnc_data)) if gnc_data is not None else 0
    rates["srv_fb_samples"] = int(len(srv_fb_data)) if srv_fb_data is not None else 0
    rates["servo_raw_samples"] = int(len(servo_raw_data)) if servo_raw_data is not None else 0

    return rates


# ============================================================================
# Stats helper
# ============================================================================

def _stats(values: np.ndarray, unit: str = "ms") -> Dict:
    """Compute distribution stats for a 1-D array of latency values."""
    arr = np.asarray(values, dtype=np.float64)
    arr = arr[~np.isnan(arr)]
    if len(arr) == 0:
        return {"count": 0, "unit": unit}

    return {
        "count": int(len(arr)),
        "unit": unit,
        "min": round(float(np.min(arr)), 3),
        "p50": round(float(np.percentile(arr, 50)), 3),
        "mean": round(float(np.mean(arr)), 3),
        "p90": round(float(np.percentile(arr, 90)), 3),
        "p99": round(float(np.percentile(arr, 99)), 3),
        "max": round(float(np.max(arr)), 3),
        "stddev": round(float(np.std(arr)), 3),
    }


# ============================================================================
# Main analysis function
# ============================================================================

def analyze_directory(result_dir: Path,
                       thresholds: Optional[dict] = None) -> Dict:
    """Run full analysis on a results directory and return metrics dict."""
    result_dir = Path(result_dir)
    if not result_dir.exists():
        return {"error": f"Directory not found: {result_dir}"}

    # Load CSVs (may be missing if corresponding stream was empty)
    imu_data       = _load_csv(result_dir / "imu.csv")
    att_data       = _load_csv(result_dir / "attitude.csv")
    gnc_data       = _load_csv(result_dir / "gnc.csv")
    srv_fb_data    = _load_csv(result_dir / "servo_fb.csv")
    servo_raw_data = _load_csv(result_dir / "servo_cmd.csv")

    metrics = {
        "result_dir": str(result_dir),
        "streams": compute_stream_rates(imu_data, att_data, gnc_data,
                                          srv_fb_data, servo_raw_data),
        "l_sensor_ms":   compute_l_sensor(imu_data, att_data),
        "l_mpc_ms":      compute_l_mpc(gnc_data),
        "l_actuator_ms": compute_l_actuator(srv_fb_data),
    }

    # Estimated total = sum of p50s (using p50 as central tendency)
    parts = []
    for key in ("l_sensor_ms", "l_mpc_ms", "l_actuator_ms"):
        m = metrics[key]
        if isinstance(m, dict) and "p50" in m:
            parts.append((key, m["p50"], m.get("p99", m["p50"])))

    if parts:
        l_total_p50 = sum(p[1] for p in parts)
        l_total_p99 = sum(p[2] for p in parts)
        metrics["l_total_estimate_ms"] = {
            "p50": round(l_total_p50, 3),
            "p99": round(l_total_p99, 3),
            "components": [p[0] for p in parts],
            "note": "sum of per-stage stats; not a directly observed event",
        }
    else:
        metrics["l_total_estimate_ms"] = {"error": "no stage measurable"}

    # Apply thresholds → PASS/FAIL
    if thresholds:
        metrics["pass_fail"] = _apply_thresholds(metrics, thresholds)

    return metrics


def _apply_thresholds(metrics: Dict, thresh: Dict) -> Dict:
    """Compare metrics to thresholds and produce PASS/FAIL list."""
    failures = []
    checks = []

    def _check(name, actual, max_val, unit="ms"):
        if actual is None:
            checks.append(f"[SKIP] {name}: no data")
            return
        ok = actual <= max_val
        status = "PASS" if ok else "FAIL"
        checks.append(f"[{status}] {name}: {actual:.2f} {unit} (max {max_val})")
        if not ok:
            failures.append(f"{name} {actual:.2f} > {max_val} {unit}")

    # L_sensor
    ls = metrics.get("l_sensor_ms", {})
    th = thresh.get("l_sensor", {})
    if isinstance(ls, dict) and "p50" in ls:
        _check("l_sensor.p50", ls.get("p50"), th.get("p50_ms_max", 30))
        _check("l_sensor.p99", ls.get("p99"), th.get("p99_ms_max", 60))
        if ls.get("count", 0) < th.get("samples_min", 100):
            failures.append(f"l_sensor samples {ls.get('count')} < {th.get('samples_min', 100)}")

    # L_mpc
    lm = metrics.get("l_mpc_ms", {})
    th = thresh.get("l_mpc", {})
    if isinstance(lm, dict) and "p50" in lm:
        _check("l_mpc.p50", lm.get("p50"), th.get("p50_ms_max", 10))
        _check("l_mpc.p99", lm.get("p99"), th.get("p99_ms_max", 25))

    # L_actuator
    la = metrics.get("l_actuator_ms", {})
    th = thresh.get("l_actuator", {})
    if isinstance(la, dict) and "p50" in la:
        _check("l_actuator.p50", la.get("p50"), th.get("p50_ms_max", 80))
        _check("l_actuator.p99", la.get("p99"), th.get("p99_ms_max", 150))

    # L_total
    lt = metrics.get("l_total_estimate_ms", {})
    th = thresh.get("l_total", {})
    if isinstance(lt, dict) and "p50" in lt:
        _check("l_total.p50", lt.get("p50"), th.get("p50_ms_max", 120))
        _check("l_total.p99", lt.get("p99"), th.get("p99_ms_max", 250))

    # Stream health
    rates = metrics.get("streams", {})
    sh = thresh.get("streams", {})
    for stream_key, min_key in [
        ("imu_hz", "imu_min_rate_hz"),
        ("attitude_hz", "attitude_min_rate_hz"),
        ("rktgnc_hz", "debug_array_min_rate_hz"),
        ("servo_raw_hz", "servo_raw_min_rate_hz"),
    ]:
        if stream_key in rates and min_key in sh:
            actual = rates[stream_key]
            min_val = sh[min_key]
            if actual < min_val:
                failures.append(f"stream {stream_key}={actual} Hz < {min_val} Hz")

    return {
        "passed": len(failures) == 0,
        "failures": failures,
        "checks": checks,
    }


# ============================================================================
# Reporting
# ============================================================================

def format_report(metrics: Dict) -> str:
    """Pretty-print metrics for human reading."""
    lines = ["=" * 70, "  E2E LATENCY REPORT", "=" * 70]

    # Streams
    streams = metrics.get("streams", {})
    lines.append("\n### STREAM HEALTH ###")
    lines.append(f"  IMU (HIGHRES_IMU)         {streams.get('imu_hz', 0):>6.1f} Hz  "
                 f"({streams.get('imu_samples', 0)} samples)")
    lines.append(f"  ATTITUDE                  {streams.get('attitude_hz', 0):>6.1f} Hz  "
                 f"({streams.get('attitude_samples', 0)} samples)")
    lines.append(f"  RktGNC (DEBUG_FLOAT_ARRAY id=2)  "
                 f"{streams.get('rktgnc_hz', 0):>6.1f} Hz  "
                 f"({streams.get('rktgnc_samples', 0)} samples)")
    lines.append(f"  SRV_FB (DEBUG_FLOAT_ARRAY id=1)  "
                 f"{streams.get('srv_fb_hz', 0):>6.1f} Hz  "
                 f"({streams.get('srv_fb_samples', 0)} samples)")
    lines.append(f"  SERVO_OUTPUT_RAW          {streams.get('servo_raw_hz', 0):>6.1f} Hz  "
                 f"({streams.get('servo_raw_samples', 0)} samples)")

    # Latency stages
    def _fmt_stage(name, m, comment=""):
        if not isinstance(m, dict) or "p50" not in m:
            err = m.get("error", "n/a") if isinstance(m, dict) else "n/a"
            return f"  {name:<22} ERROR: {err}"
        return (f"  {name:<22} "
                f"min={m['min']:>6.2f}  p50={m['p50']:>6.2f}  "
                f"mean={m['mean']:>6.2f}  p90={m['p90']:>6.2f}  "
                f"p99={m['p99']:>6.2f}  max={m['max']:>6.2f}  "
                f"(n={m['count']}) ms"
                + (f"\n      {comment}" if comment else ""))

    lines.append("\n### LATENCY STAGES (ms) ###")
    lines.append(_fmt_stage("L_sensor (IMU→ATT)",
                            metrics.get("l_sensor_ms", {}),
                            "phone IMU sample → vehicle_attitude published"))
    lines.append(_fmt_stage("L_mpc (solve time)",
                            metrics.get("l_mpc_ms", {}),
                            "MPC compute time per cycle (from RktGNC)"))
    la = metrics.get("l_actuator_ms", {})
    actuator_extra = ""
    if isinstance(la, dict) and "edges_detected" in la:
        actuator_extra = (f"edges_detected={la['edges_detected']}, "
                          f"settled={la.get('edges_settled', 0)}")
    lines.append(_fmt_stage("L_actuator (CMD→FB)",
                            la, actuator_extra))

    # Total
    lt = metrics.get("l_total_estimate_ms", {})
    lines.append("\n### TOTAL TRANSPORT DELAY (estimated) ###")
    if "p50" in lt:
        lines.append(f"  L_total = sum(L_sensor + L_mpc + L_actuator)")
        lines.append(f"           p50 ≈ {lt['p50']:.2f} ms,  p99 ≈ {lt['p99']:.2f} ms")
        lines.append(f"  Note:    {lt.get('note', '')}")
    else:
        lines.append(f"  ERROR: {lt.get('error', 'n/a')}")

    # PASS/FAIL
    if "pass_fail" in metrics:
        pf = metrics["pass_fail"]
        lines.append("\n### PASS / FAIL ###")
        for c in pf.get("checks", []):
            lines.append(f"  {c}")
        verdict = "✅ GO" if pf.get("passed") else "❌ NO-GO"
        lines.append(f"\n  VERDICT: {verdict}")
        if pf.get("failures"):
            lines.append("\n  Failures:")
            for f in pf["failures"]:
                lines.append(f"    - {f}")

    # MPC tuning hint
    if "p99" in lt:
        lines.append("\n### MPC TUNING HINT ###")
        p99 = lt["p99"]
        if p99 < 80:
            hint = "RKT_MPC_SVO_DLY = 0.100f (current floor) is sufficient"
        elif p99 < 120:
            hint = "Consider RKT_MPC_SVO_DLY = 0.150f"
        elif p99 < 180:
            hint = "Consider RKT_MPC_SVO_DLY = 0.200f"
        else:
            hint = "Latency >180ms — investigate thermal throttling or pipeline"
        lines.append(f"  L_total p99 = {p99:.1f} ms → {hint}")

    lines.append("\n" + "=" * 70)
    return "\n".join(lines)


# ============================================================================
# Optional plotting
# ============================================================================

def maybe_plot(result_dir: Path, metrics: Dict):
    """Generate interactive Plotly HTML if plotly is installed."""
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError:
        logger.info("plotly not installed — skipping HTML plot")
        return None

    imu = _load_csv(result_dir / "imu.csv")
    att = _load_csv(result_dir / "attitude.csv")
    gnc = _load_csv(result_dir / "gnc.csv")
    fb  = _load_csv(result_dir / "servo_fb.csv")

    fig = make_subplots(rows=4, cols=1,
                        subplot_titles=("Phone IMU |gyro|",
                                        "Phone IMU |accel|",
                                        "MPC solve time (μs) — RktGNC",
                                        "Servo cmd vs feedback (deg)"),
                        shared_xaxes=False, vertical_spacing=0.08)

    # Row 1: gyro magnitude over time
    if imu is not None and len(imu) > 5:
        try:
            gx = np.asarray(imu["gx"], dtype=np.float64)
            gy = np.asarray(imu["gy"], dtype=np.float64)
            gz = np.asarray(imu["gz"], dtype=np.float64)
            mag = np.sqrt(gx*gx + gy*gy + gz*gz)
            t = np.asarray(imu["t_wall_s"], dtype=np.float64)
            fig.add_trace(go.Scatter(x=t, y=mag, name="|gyro|", mode="lines",
                                      line=dict(width=1)), row=1, col=1)
        except (KeyError, ValueError):
            pass

    # Row 2: accel magnitude
    if imu is not None and len(imu) > 5:
        try:
            ax = np.asarray(imu["ax"], dtype=np.float64)
            ay = np.asarray(imu["ay"], dtype=np.float64)
            az = np.asarray(imu["az"], dtype=np.float64)
            amag = np.sqrt(ax*ax + ay*ay + az*az)
            t = np.asarray(imu["t_wall_s"], dtype=np.float64)
            fig.add_trace(go.Scatter(x=t, y=amag, name="|accel|", mode="lines",
                                      line=dict(width=1, color="orange")),
                           row=2, col=1)
        except (KeyError, ValueError):
            pass

    # Row 3: MPC solve us
    if gnc is not None and len(gnc) > 5:
        try:
            t = np.asarray(gnc["t_wall_s"], dtype=np.float64)
            mpc = np.asarray(gnc["mpc_solve_us"], dtype=np.float64)
            fig.add_trace(go.Scatter(x=t, y=mpc, name="mpc_solve_us",
                                      mode="lines+markers",
                                      line=dict(width=1, color="purple")),
                           row=3, col=1)
        except (KeyError, ValueError):
            pass

    # Row 4: servo cmd vs fb (servo 0)
    if fb is not None and len(fb) > 5:
        try:
            t = np.asarray(fb["t_wall_s"], dtype=np.float64)
            cmd = np.asarray(fb["cmd0"], dtype=np.float64)
            fb0 = np.asarray(fb["fb0"], dtype=np.float64)
            fig.add_trace(go.Scatter(x=t, y=cmd, name="cmd0", mode="lines",
                                      line=dict(width=1, color="blue")),
                           row=4, col=1)
            fig.add_trace(go.Scatter(x=t, y=fb0, name="fb0", mode="lines",
                                      line=dict(width=1, color="red", dash="dot")),
                           row=4, col=1)
        except (KeyError, ValueError):
            pass

    fig.update_layout(
        height=1000,
        title=f"E2E Latency — {result_dir.name}",
        showlegend=True,
    )
    fig.update_xaxes(title_text="t_wall (s)", row=4, col=1)

    out = result_dir / "latency_plot.html"
    # Wrap figure with a numerical-metrics table at the top.
    fig_html = fig.to_html(full_html=False, include_plotlyjs="cdn")
    table_html = _metrics_table_html(metrics)
    page = (
        '<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8">'
        f'<title>E2E Latency — {result_dir.name}</title>'
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
        '.pass{color:#0a8a0a}.fail{color:#c00}'
        '</style></head><body>'
        f'<h1>E2E Latency — {result_dir.name}</h1>'
        f'{table_html}<div class="card">{fig_html}</div></body></html>'
    )
    out.write_text(page, encoding="utf-8")
    logger.info(f"Plot saved: {out}")
    return out


def _fmt_num(v):
    if v is None: return "—"
    if isinstance(v, bool): return "true" if v else "false"
    if isinstance(v, (int, np.integer)): return f"{int(v):,}"
    if isinstance(v, float):
        if not np.isfinite(v): return "NaN" if np.isnan(v) else ("+∞" if v > 0 else "−∞")
        av = abs(v)
        if av == 0: return "0"
        if av >= 1e6 or av < 1e-3: return f"{v:.3e}"
        if av >= 100: return f"{v:.1f}"
        if av >= 1: return f"{v:.3f}"
        return f"{v:.4f}"
    return str(v)


def _metrics_table_html(metrics: Dict) -> str:
    """Flatten nested metrics dict and render as a categorized HTML table."""
    rows = []
    # Stream health
    streams = metrics.get("streams", {})
    if streams:
        rows.append(('Stream Health', None, None, None))
        for k, v in streams.items():
            unit = "Hz" if k.endswith("_hz") else ("samples" if k.endswith("_samples") else "")
            rows.append((k, _fmt_num(v), unit, None))
    # Latency stages
    for stage_key, stage_label in [
        ("l_sensor_ms", "L_sensor (IMU→ATT) [ms]"),
        ("l_mpc_ms", "L_mpc (solve) [ms]"),
        ("l_actuator_ms", "L_actuator (CMD→FB) [ms]"),
        ("l_total_estimate_ms", "L_total (estimated) [ms]"),
    ]:
        st = metrics.get(stage_key, {})
        if not isinstance(st, dict) or "p50" not in st:
            err = st.get("error", "n/a") if isinstance(st, dict) else "n/a"
            rows.append((stage_label, None, None, None))
            rows.append((f"{stage_key}.error", str(err), "", None))
            continue
        rows.append((stage_label, None, None, None))
        for sub in ("min", "p50", "mean", "p90", "p99", "max", "count"):
            if sub in st:
                u = "ms" if sub != "count" else "samples"
                rows.append((f"{stage_key}.{sub}", _fmt_num(st[sub]), u, None))
        for extra in ("edges_detected", "edges_settled", "note"):
            if extra in st:
                rows.append((f"{stage_key}.{extra}", _fmt_num(st[extra]) if isinstance(st[extra], (int, float)) else str(st[extra]), "", None))
    # PASS/FAIL
    pf = metrics.get("pass_fail")
    if isinstance(pf, dict):
        rows.append(("Pass/Fail Checks", None, None, None))
        verdict = "PASS" if pf.get("passed") else "FAIL"
        rows.append(("verdict", verdict, "", "pass" if pf.get("passed") else "fail"))
        for c in pf.get("checks", []):
            rows.append((f"check[{len(rows)}]", str(c), "", None))
        for f_ in pf.get("failures", []):
            rows.append(("failure", str(f_), "", "fail"))
    # Render
    body = ""
    for label, val, unit, cls in rows:
        if val is None:
            body += f'<tr class="cat"><td colspan="3">■ {label}</td></tr>'
        else:
            cls_attr = f' class="{cls}"' if cls else ""
            body += (f'<tr><td style="font-family:ui-monospace,monospace;font-size:.85rem">{label}</td>'
                     f'<td class="num"{cls_attr}>{val}</td>'
                     f'<td class="unit">{unit}</td></tr>')
    return ('<div class="card"><h2 style="margin-top:0">📊 Numerical Metrics</h2>'
            '<table><thead><tr><th>Metric</th><th style="text-align:right">Value</th>'
            f'<th>Unit</th></tr></thead><tbody>{body}</tbody></table></div>')


# ============================================================================
# CLI
# ============================================================================

def main():
    p = argparse.ArgumentParser(description="Analyze E2E latency from results directory")
    p.add_argument("result_dir", type=Path,
                   help="Path to e2e_latency results directory (containing CSVs)")
    p.add_argument("--config", type=Path, default=None,
                   help="Optional e2e_config.yaml for thresholds")
    p.add_argument("--plot", action="store_true",
                   help="Generate Plotly HTML plot")
    args = p.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s")

    thresholds = None
    if args.config and args.config.exists():
        import yaml
        with open(args.config) as f:
            cfg = yaml.safe_load(f)
            thresholds = cfg.get("thresholds")

    metrics = analyze_directory(args.result_dir, thresholds=thresholds)

    # Save JSON
    json_path = args.result_dir / "latency.metrics.json"
    with open(json_path, "w") as f:
        json.dump(metrics, f, indent=2)
    print(f"Metrics → {json_path}")

    # Save report
    report = format_report(metrics)
    report_path = args.result_dir / "latency_report.txt"
    with open(report_path, "w") as f:
        f.write(report)
    print(report)

    if args.plot:
        maybe_plot(args.result_dir, metrics)


if __name__ == "__main__":
    main()
