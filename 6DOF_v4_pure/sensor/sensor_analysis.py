#!/usr/bin/env python3
"""
sensor_analysis.py — تحليل بيانات الحساسات وإنتاج الرسوم البيانية
==================================================================

يقبل مجلد نتائج من sensor_runner.py ويُنتج:
  - Allan deviation plots (HTML تفاعلي + PNG)
  - Noise time-series plots
  - FFT / PSD plots (vibration)
  - Temperature drift plots
  - GPS scatter plot
  - ملخّص HTML شامل

الاستخدام:
    python3 sensor_analysis.py results/20260502_210000/
    python3 sensor_analysis.py results/20260502_210000/ --format png
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

logger = logging.getLogger("sensor_analysis")

_SCRIPT_DIR = Path(__file__).resolve().parent


# ============================================================================
# Allan Deviation (standalone — can be used without sensor_runner)
# ============================================================================

def allan_deviation(data: np.ndarray, fs: float,
                    tau_min: float = 0.005, tau_max: float = 1000.0,
                    n_points: int = 100) -> tuple:
    """Overlapping Allan deviation.

    Args:
        data: 1D time-series (e.g. gyro_z in rad/s)
        fs: sample rate in Hz
        tau_min/tau_max: range of cluster times
        n_points: number of logarithmically-spaced τ values

    Returns:
        (taus, adevs) numpy arrays
    """
    N = len(data)
    max_m = N // 2
    min_m = max(1, int(tau_min * fs))
    max_m = min(max_m, int(tau_max * fs))

    if max_m <= min_m:
        return np.array([]), np.array([])

    ms = np.unique(np.logspace(np.log10(min_m), np.log10(max_m),
                               n_points).astype(int))
    ms = ms[(ms >= 1) & (ms <= max_m)]

    taus = ms / fs
    adevs = np.zeros(len(ms))

    cumsum = np.cumsum(data)
    cumsum = np.insert(cumsum, 0, 0)

    for i, m in enumerate(ms):
        n_full = len(data) - 2 * m
        if n_full < 1:
            adevs[i] = np.nan
            continue
        s0 = cumsum[:n_full]
        s1 = cumsum[m:m + n_full]
        s2 = cumsum[2 * m:2 * m + n_full]
        diff = s2 - 2 * s1 + s0
        adevs[i] = np.sqrt(np.mean(diff**2) / (2.0 * (m**2)))

    valid = ~np.isnan(adevs)
    return taus[valid], adevs[valid]


def extract_allan_params(taus: np.ndarray, adevs: np.ndarray) -> dict:
    """Extract key IMU parameters from Allan deviation curve.

    Returns dict with:
        - arw: Angle Random Walk (rad/√s for gyro, m/s/√s for accel)
        - bi: Bias Instability
        - bi_tau: τ at minimum Allan deviation
        - rrw: Rate Random Walk (estimated from slope)
    """
    if len(taus) == 0:
        return {}

    # Bias instability = minimum of Allan deviation
    bi_idx = np.argmin(adevs)
    bi = float(adevs[bi_idx])
    bi_tau = float(taus[bi_idx])

    # ARW = Allan deviation at τ=1s (from -1/2 slope region)
    # σ(τ) = ARW / √τ → at τ=1, σ = ARW
    arw = None
    # Find the τ=1 point, or interpolate
    if taus[0] <= 1.0 <= taus[-1]:
        arw = float(np.interp(1.0, taus, adevs))
    elif taus[0] > 1.0:
        # Extrapolate from first point using -1/2 slope
        arw = float(adevs[0] * np.sqrt(taus[0]))

    # RRW = estimated from +1/2 slope region (long τ)
    # σ(τ) = RRW × √(τ/3) → find slope in log-log
    rrw = None
    if len(taus) > bi_idx + 5:
        # Use points after BI minimum
        long_taus = taus[bi_idx + 3:]
        long_adevs = adevs[bi_idx + 3:]
        if len(long_taus) >= 3:
            log_slope = np.polyfit(np.log10(long_taus), np.log10(long_adevs), 1)
            if log_slope[0] > 0.3:  # +1/2 slope region
                # RRW at τ=3
                rrw = float(long_adevs[-1] * np.sqrt(3.0 / long_taus[-1]))

    return {
        "bi": bi,
        "bi_tau": bi_tau,
        "arw": arw,
        "rrw": rrw,
    }


# ============================================================================
# Plot generators (Plotly HTML)
# ============================================================================

def plot_allan_html(results: dict, output_path: Path, title: str = "Allan Deviation"):
    """Generate interactive Plotly HTML for Allan deviation curves."""
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError:
        logger.warning("plotly not installed — skipping Allan plot")
        return

    fig = make_subplots(rows=1, cols=2,
                        subplot_titles=("Gyroscope", "Accelerometer"),
                        horizontal_spacing=0.12)

    colors = {"x": "#e74c3c", "y": "#2ecc71", "z": "#3498db"}

    for axis_name in ["gx", "gy", "gz"]:
        if axis_name not in results:
            continue
        r = results[axis_name]
        taus = np.array(r["taus"])
        adevs = np.array(r["adevs"])
        if len(taus) == 0:
            continue

        params = extract_allan_params(taus, adevs)
        label = axis_name.upper()
        if params.get("bi"):
            bi_dph = params["bi"] * 3600 * np.degrees(1.0)
            label += f" (BI={bi_dph:.1f} °/hr)"

        fig.add_trace(go.Scatter(
            x=taus, y=adevs, mode="lines",
            name=label,
            line=dict(color=colors.get(axis_name[-1], "#666")),
        ), row=1, col=1)

        # Mark BI point
        if params.get("bi"):
            fig.add_trace(go.Scatter(
                x=[params["bi_tau"]], y=[params["bi"]],
                mode="markers", marker=dict(size=10, symbol="star"),
                name=f"{axis_name} BI",
                showlegend=False,
            ), row=1, col=1)

    for axis_name in ["ax", "ay", "az"]:
        if axis_name not in results:
            continue
        r = results[axis_name]
        taus = np.array(r["taus"])
        adevs = np.array(r["adevs"])
        if len(taus) == 0:
            continue

        params = extract_allan_params(taus, adevs)
        label = axis_name.upper()
        if params.get("bi"):
            bi_mg = params["bi"] / 9.81 * 1000
            label += f" (BI={bi_mg:.2f} mg)"

        fig.add_trace(go.Scatter(
            x=taus, y=adevs, mode="lines",
            name=label,
            line=dict(color=colors.get(axis_name[-1], "#666")),
        ), row=1, col=2)

        if params.get("bi"):
            fig.add_trace(go.Scatter(
                x=[params["bi_tau"]], y=[params["bi"]],
                mode="markers", marker=dict(size=10, symbol="star"),
                name=f"{axis_name} BI",
                showlegend=False,
            ), row=1, col=2)

    fig.update_xaxes(type="log", title_text="Cluster Time τ (s)", row=1, col=1)
    fig.update_xaxes(type="log", title_text="Cluster Time τ (s)", row=1, col=2)
    fig.update_yaxes(type="log", title_text="Allan Deviation (rad/s)", row=1, col=1)
    fig.update_yaxes(type="log", title_text="Allan Deviation (m/s²)", row=1, col=2)
    fig.update_layout(title=title, height=500, width=1100,
                      template="plotly_white")
    fig.write_html(str(output_path))
    logger.info(f"Allan plot saved: {output_path}")


def plot_static_html(imu_csv: Path, output_path: Path):
    """Generate time-series noise plot for static test."""
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError:
        logger.warning("plotly not installed — skipping static plot")
        return

    data = np.genfromtxt(imu_csv, delimiter=",", names=True, dtype=None,
                         encoding="utf-8")
    if len(data) < 10:
        return

    t = data["t_wall_s"].astype(float)
    t = t - t[0]  # relative time

    fig = make_subplots(rows=3, cols=2,
                        subplot_titles=("Accelerometer", "Gyroscope",
                                        "Accel Histogram", "Gyro Histogram",
                                        "Magnetometer", "Temperature"),
                        vertical_spacing=0.08)

    # Accel time series
    for col, color, label in [("ax", "#e74c3c", "X"),
                               ("ay", "#2ecc71", "Y"),
                               ("az", "#3498db", "Z")]:
        fig.add_trace(go.Scatter(
            x=t, y=data[col].astype(float), mode="lines",
            name=f"Accel {label}", line=dict(width=0.5, color=color),
        ), row=1, col=1)

    # Gyro time series
    for col, color, label in [("gx", "#e74c3c", "X"),
                               ("gy", "#2ecc71", "Y"),
                               ("gz", "#3498db", "Z")]:
        fig.add_trace(go.Scatter(
            x=t, y=data[col].astype(float), mode="lines",
            name=f"Gyro {label}", line=dict(width=0.5, color=color),
        ), row=1, col=2)

    # Accel histogram
    for col, color in [("ax", "#e74c3c"), ("ay", "#2ecc71"), ("az", "#3498db")]:
        vals = data[col].astype(float)
        fig.add_trace(go.Histogram(
            x=vals - np.mean(vals), nbinsx=100, opacity=0.6,
            name=f"{col} dist", marker_color=color, showlegend=False,
        ), row=2, col=1)

    # Gyro histogram
    for col, color in [("gx", "#e74c3c"), ("gy", "#2ecc71"), ("gz", "#3498db")]:
        vals = data[col].astype(float)
        fig.add_trace(go.Histogram(
            x=vals - np.mean(vals), nbinsx=100, opacity=0.6,
            name=f"{col} dist", marker_color=color, showlegend=False,
        ), row=2, col=2)

    # Mag
    for col, color, label in [("mx", "#e74c3c", "X"),
                               ("my", "#2ecc71", "Y"),
                               ("mz", "#3498db", "Z")]:
        fig.add_trace(go.Scatter(
            x=t, y=data[col].astype(float), mode="lines",
            name=f"Mag {label}", line=dict(width=0.5, color=color),
            showlegend=False,
        ), row=3, col=1)

    # Temperature
    fig.add_trace(go.Scatter(
        x=t, y=data["temperature"].astype(float), mode="lines",
        name="Temperature", line=dict(color="#e67e22"),
        showlegend=False,
    ), row=3, col=2)

    fig.update_layout(title="Static Sensor Data", height=900, width=1100,
                      template="plotly_white")
    fig.update_xaxes(title_text="Time (s)", row=3, col=1)
    fig.update_xaxes(title_text="Time (s)", row=3, col=2)
    fig.update_yaxes(title_text="m/s²", row=1, col=1)
    fig.update_yaxes(title_text="rad/s", row=1, col=2)
    fig.update_yaxes(title_text="Gauss", row=3, col=1)
    fig.update_yaxes(title_text="°C", row=3, col=2)

    fig.write_html(str(output_path))
    logger.info(f"Static plot saved: {output_path}")


def plot_vibration_html(imu_csv: Path, output_path: Path):
    """Generate FFT/PSD plot for vibration test."""
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
        from scipy import signal as sig
    except ImportError:
        logger.warning("plotly/scipy not installed — skipping vibration plot")
        return

    data = np.genfromtxt(imu_csv, delimiter=",", names=True, dtype=None,
                         encoding="utf-8")
    if len(data) < 100:
        return

    t = data["t_wall_s"].astype(float)
    dt = np.median(np.diff(t))
    fs = 1.0 / dt if dt > 0 else 200.0
    nperseg = min(1024, len(data) // 4)

    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Accel PSD", "Gyro PSD",
                                        "Accel Time Domain", "Gyro Time Domain"),
                        vertical_spacing=0.12)

    colors = {"x": "#e74c3c", "y": "#2ecc71", "z": "#3498db"}

    # Accel PSD
    for col, axis in [("ax", "x"), ("ay", "y"), ("az", "z")]:
        signal_data = data[col].astype(float)
        freqs, psd = sig.welch(signal_data, fs=fs, nperseg=nperseg)
        fig.add_trace(go.Scatter(
            x=freqs, y=10 * np.log10(psd + 1e-20), mode="lines",
            name=f"Accel {axis.upper()}", line=dict(color=colors[axis]),
        ), row=1, col=1)

    # Gyro PSD
    for col, axis in [("gx", "x"), ("gy", "y"), ("gz", "z")]:
        signal_data = data[col].astype(float)
        freqs, psd = sig.welch(signal_data, fs=fs, nperseg=nperseg)
        fig.add_trace(go.Scatter(
            x=freqs, y=10 * np.log10(psd + 1e-20), mode="lines",
            name=f"Gyro {axis.upper()}", line=dict(color=colors[axis]),
        ), row=1, col=2)

    # Time domain
    t_rel = t - t[0]
    for col, axis in [("ax", "x"), ("ay", "y"), ("az", "z")]:
        fig.add_trace(go.Scatter(
            x=t_rel, y=data[col].astype(float), mode="lines",
            name=f"a{axis}", line=dict(width=0.5, color=colors[axis]),
            showlegend=False,
        ), row=2, col=1)

    for col, axis in [("gx", "x"), ("gy", "y"), ("gz", "z")]:
        fig.add_trace(go.Scatter(
            x=t_rel, y=data[col].astype(float), mode="lines",
            name=f"g{axis}", line=dict(width=0.5, color=colors[axis]),
            showlegend=False,
        ), row=2, col=2)

    fig.update_xaxes(title_text="Frequency (Hz)", row=1, col=1)
    fig.update_xaxes(title_text="Frequency (Hz)", row=1, col=2)
    fig.update_xaxes(title_text="Time (s)", row=2, col=1)
    fig.update_xaxes(title_text="Time (s)", row=2, col=2)
    fig.update_yaxes(title_text="PSD (dB)", row=1, col=1)
    fig.update_yaxes(title_text="PSD (dB)", row=1, col=2)
    fig.update_yaxes(title_text="m/s²", row=2, col=1)
    fig.update_yaxes(title_text="rad/s", row=2, col=2)

    fig.update_layout(title="Vibration Analysis", height=700, width=1100,
                      template="plotly_white")
    fig.write_html(str(output_path))
    logger.info(f"Vibration plot saved: {output_path}")


def plot_gps_html(gps_csv: Path, output_path: Path):
    """Generate GPS scatter plot."""
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError:
        logger.warning("plotly not installed — skipping GPS plot")
        return

    data = np.genfromtxt(gps_csv, delimiter=",", names=True, dtype=None,
                         encoding="utf-8")
    if len(data) < 5:
        return

    lat = data["lat_e7"].astype(float) / 1e7
    lon = data["lon_e7"].astype(float) / 1e7
    t = data["t_wall_s"].astype(float) - data["t_wall_s"].astype(float)[0]
    sats = data["satellites"].astype(int)
    eph = data["eph_cm"].astype(float) / 100.0

    lat_mean = np.mean(lat)
    lon_mean = np.mean(lon)
    dlat_m = (lat - lat_mean) * 111320.0
    dlon_m = (lon - lon_mean) * 111320.0 * np.cos(np.radians(lat_mean))

    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Position Scatter (m from mean)",
                                        "HDOP over Time",
                                        "Satellites over Time",
                                        "Position Error (m)"),
                        vertical_spacing=0.12)

    # Scatter plot
    fig.add_trace(go.Scatter(
        x=dlon_m, y=dlat_m, mode="markers",
        marker=dict(size=4, color=t, colorscale="Viridis",
                    colorbar=dict(title="Time (s)", x=0.45)),
        name="Position",
    ), row=1, col=1)

    # HDOP
    fig.add_trace(go.Scatter(
        x=t, y=eph, mode="lines",
        name="HDOP", line=dict(color="#e74c3c"),
    ), row=1, col=2)

    # Satellites
    fig.add_trace(go.Scatter(
        x=t, y=sats, mode="lines",
        name="Satellites", line=dict(color="#3498db"),
    ), row=2, col=1)

    # Distance from mean
    dist = np.sqrt(dlat_m**2 + dlon_m**2)
    fig.add_trace(go.Scatter(
        x=t, y=dist, mode="lines",
        name="Error (m)", line=dict(color="#2ecc71"),
    ), row=2, col=2)

    fig.update_xaxes(title_text="East (m)", row=1, col=1)
    fig.update_yaxes(title_text="North (m)", row=1, col=1)
    fig.update_xaxes(title_text="Time (s)", row=1, col=2)
    fig.update_xaxes(title_text="Time (s)", row=2, col=1)
    fig.update_xaxes(title_text="Time (s)", row=2, col=2)
    # Equal aspect ratio for scatter
    fig.update_yaxes(scaleanchor="x", scaleratio=1, row=1, col=1)

    fig.update_layout(title="GPS Performance", height=700, width=1100,
                      template="plotly_white")
    fig.write_html(str(output_path))
    logger.info(f"GPS plot saved: {output_path}")


def plot_temperature_html(imu_csv: Path, output_path: Path):
    """Generate temperature drift plot."""
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError:
        logger.warning("plotly not installed — skipping temperature plot")
        return

    data = np.genfromtxt(imu_csv, delimiter=",", names=True, dtype=None,
                         encoding="utf-8")
    if len(data) < 100:
        return

    t = data["t_wall_s"].astype(float) - data["t_wall_s"].astype(float)[0]
    temp = data["temperature"].astype(float)

    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Gyro Bias vs Temperature",
                                        "Accel Bias vs Temperature",
                                        "Temperature over Time",
                                        "Gyro Bias over Time"),
                        vertical_spacing=0.12)

    # Compute rolling mean (30s windows)
    window = max(1, int(30.0 / np.median(np.diff(t))))
    colors = {"x": "#e74c3c", "y": "#2ecc71", "z": "#3498db"}

    # Gyro bias vs temp
    for col, axis in [("gx", "x"), ("gy", "y"), ("gz", "z")]:
        vals = data[col].astype(float)
        # Rolling mean
        if len(vals) > window:
            rm = np.convolve(vals, np.ones(window)/window, mode="valid")
            rm_t = np.convolve(temp, np.ones(window)/window, mode="valid")
            fig.add_trace(go.Scatter(
                x=rm_t, y=rm, mode="markers",
                marker=dict(size=2, color=colors[axis]),
                name=f"Gyro {axis.upper()}",
            ), row=1, col=1)

    # Accel bias vs temp
    for col, axis in [("ax", "x"), ("ay", "y"), ("az", "z")]:
        vals = data[col].astype(float)
        if len(vals) > window:
            rm = np.convolve(vals, np.ones(window)/window, mode="valid")
            rm_t = np.convolve(temp, np.ones(window)/window, mode="valid")
            fig.add_trace(go.Scatter(
                x=rm_t, y=rm, mode="markers",
                marker=dict(size=2, color=colors[axis]),
                name=f"Accel {axis.upper()}",
            ), row=1, col=2)

    # Temperature over time
    fig.add_trace(go.Scatter(
        x=t, y=temp, mode="lines",
        name="Temperature", line=dict(color="#e67e22"),
        showlegend=False,
    ), row=2, col=1)

    # Gyro bias over time
    for col, axis in [("gx", "x"), ("gy", "y"), ("gz", "z")]:
        vals = data[col].astype(float)
        if len(vals) > window:
            rm = np.convolve(vals, np.ones(window)/window, mode="valid")
            rm_time = t[:len(rm)]
            fig.add_trace(go.Scatter(
                x=rm_time, y=rm, mode="lines",
                name=f"g{axis} (rolling)", line=dict(color=colors[axis]),
                showlegend=False,
            ), row=2, col=2)

    fig.update_xaxes(title_text="Temperature (°C)", row=1, col=1)
    fig.update_xaxes(title_text="Temperature (°C)", row=1, col=2)
    fig.update_xaxes(title_text="Time (s)", row=2, col=1)
    fig.update_xaxes(title_text="Time (s)", row=2, col=2)
    fig.update_yaxes(title_text="rad/s", row=1, col=1)
    fig.update_yaxes(title_text="m/s²", row=1, col=2)
    fig.update_yaxes(title_text="°C", row=2, col=1)
    fig.update_yaxes(title_text="rad/s", row=2, col=2)

    fig.update_layout(title="Temperature Drift Analysis", height=700, width=1100,
                      template="plotly_white")
    fig.write_html(str(output_path))
    logger.info(f"Temperature plot saved: {output_path}")


# ============================================================================
# Config suggestion — update 6dof_config_advanced.yaml
# ============================================================================

def suggest_config_updates(result_dir: Path) -> dict:
    """Based on measured sensor data, suggest updates for 6dof_config_advanced.yaml."""
    suggestions = {}

    # Static metrics
    static_json = result_dir / "static" / "static.metrics.json"
    if static_json.exists():
        with open(static_json) as f:
            m = json.load(f)

        accel_noise = m.get("accel_noise_rms_ms2", None)
        gyro_noise = m.get("gyro_noise_rms_rads", None)
        gyro_mean = m.get("gyro_mean_xyz_rads", None)
        accel_mean = m.get("accel_mean_xyz_ms2", None)

        if accel_noise:
            suggestions["estimation.sensors.accel_noise_std"] = round(accel_noise, 3)
            suggestions["bridge.noise.accel_std"] = round(accel_noise, 3)
        if gyro_noise:
            suggestions["estimation.sensors.gyro_noise_std"] = round(gyro_noise, 4)
            suggestions["bridge.noise.gyro_std"] = round(gyro_noise, 4)
        if gyro_mean:
            suggestions["error_injection.sig_gyro_bias_x"] = round(abs(gyro_mean[0]) * 2, 4)
            suggestions["error_injection.sig_gyro_bias_y"] = round(abs(gyro_mean[1]) * 2, 4)
            suggestions["error_injection.sig_gyro_bias_z"] = round(abs(gyro_mean[2]) * 2, 4)
        if accel_mean:
            # Remove gravity from Z
            accel_bias = list(accel_mean)
            accel_bias[2] = accel_bias[2] + 9.81  # FRD: az ≈ -9.81 when flat
            suggestions["error_injection.sig_accel_bias_x"] = round(abs(accel_bias[0]) * 2, 3)
            suggestions["error_injection.sig_accel_bias_y"] = round(abs(accel_bias[1]) * 2, 3)
            suggestions["error_injection.sig_accel_bias_z"] = round(abs(accel_bias[2]) * 2, 3)

    # Allan metrics
    allan_json = result_dir / "allan" / "allan.metrics.json"
    if allan_json.exists():
        with open(allan_json) as f:
            m = json.load(f)

        gyro_bi = m.get("gyro_bias_instability_rads", None)
        accel_bi = m.get("accel_bias_instability_ms2", None)
        if gyro_bi:
            suggestions["estimation.sensors.gyro_bias_std"] = round(gyro_bi * 3, 5)
        if accel_bi:
            suggestions["estimation.sensors.accel_bias_std"] = round(accel_bi * 3, 4)

    return suggestions


# ============================================================================
# Generate summary HTML
# ============================================================================

def generate_summary_html(result_dir: Path):
    """Generate a summary HTML file combining all test results."""
    all_metrics_path = result_dir / "all_metrics.json"
    go_nogo_path = result_dir / "GO_NOGO.txt"

    if not all_metrics_path.exists():
        logger.warning("No all_metrics.json found — run sensor_runner first")
        return

    with open(all_metrics_path) as f:
        all_metrics = json.load(f)

    verdict = "?"
    if go_nogo_path.exists():
        with open(go_nogo_path) as f:
            first_line = f.readline().strip()
            verdict = first_line.replace("VERDICT:", "").strip()

    # Suggestions
    suggestions = suggest_config_updates(result_dir)

    html = f"""<!DOCTYPE html>
<html dir="rtl" lang="ar">
<head>
    <meta charset="UTF-8">
    <title>Sensor Test Report</title>
    <style>
        body {{ font-family: 'Segoe UI', sans-serif; margin: 40px; background: #f5f5f5; direction: ltr; }}
        .container {{ max-width: 1200px; margin: 0 auto; }}
        h1 {{ color: #2c3e50; border-bottom: 3px solid #3498db; padding-bottom: 10px; }}
        h2 {{ color: #34495e; margin-top: 30px; }}
        .verdict {{ font-size: 2em; padding: 20px; text-align: center; border-radius: 10px;
                    margin: 20px 0; font-weight: bold; }}
        .verdict.go {{ background: #d4edda; color: #155724; border: 2px solid #28a745; }}
        .verdict.nogo {{ background: #f8d7da; color: #721c24; border: 2px solid #dc3545; }}
        .test-card {{ background: white; border-radius: 8px; padding: 20px; margin: 15px 0;
                      box-shadow: 0 2px 4px rgba(0,0,0,0.1); }}
        .pass {{ color: #28a745; font-weight: bold; }}
        .fail {{ color: #dc3545; font-weight: bold; }}
        .skip {{ color: #6c757d; font-weight: bold; }}
        table {{ border-collapse: collapse; width: 100%; }}
        th, td {{ border: 1px solid #dee2e6; padding: 8px 12px; text-align: left; }}
        th {{ background: #f8f9fa; }}
        .suggestion {{ background: #fff3cd; border: 1px solid #ffc107; border-radius: 5px;
                       padding: 15px; margin: 10px 0; }}
        pre {{ background: #f8f9fa; padding: 10px; border-radius: 5px; overflow-x: auto; }}
    </style>
</head>
<body>
<div class="container">
    <h1>/sensor — Sensor Test Report</h1>
    <p>Generated: {result_dir.name}</p>

    <div class="verdict {'go' if verdict == 'GO' else 'nogo'}">
        {'&#10004;' if verdict == 'GO' else '&#10008;'} {verdict}
    </div>
"""

    # Test results
    for test_name, test_data in all_metrics.items():
        passed = test_data.get("passed")
        failures = test_data.get("failures", [])
        metrics = test_data.get("metrics", {})

        status_class = "pass" if passed else ("fail" if passed is False else "skip")
        status_text = "PASS" if passed else ("FAIL" if passed is False else "SKIP")

        html += f"""
    <div class="test-card">
        <h2>{test_name} <span class="{status_class}">[{status_text}]</span></h2>
"""
        if failures:
            html += "<ul>\n"
            for f in failures:
                html += f"  <li class='fail'>{f}</li>\n"
            html += "</ul>\n"

        # Metrics table — flatten nested dicts (one level) so every number
        # is visible. Skips heavy raw arrays (allan_results) and long lists.
        def _flatten(d, prefix=""):
            for k, v in d.items():
                if k == "allan_results":
                    continue
                key = f"{prefix}{k}" if not prefix else f"{prefix}.{k}"
                if isinstance(v, dict):
                    yield from _flatten(v, key)
                elif isinstance(v, list) and len(v) > 6:
                    yield (key, f"[{len(v)} items]")
                else:
                    if isinstance(v, float):
                        yield (key, f"{v:.6g}")
                    elif isinstance(v, list):
                        yield (key, "[" + ", ".join(
                            f"{x:.4g}" if isinstance(x, float) else str(x)
                            for x in v) + "]")
                    else:
                        yield (key, str(v))
        flat_pairs = list(_flatten(metrics))
        if flat_pairs:
            html += '<table><tr><th>Metric</th><th style="text-align:right">Value</th></tr>\n'
            for k, v in flat_pairs:
                html += (f'<tr><td style="font-family:ui-monospace,monospace;'
                         f'font-size:.85rem">{k}</td>'
                         f'<td style="text-align:right;font-variant-numeric:tabular-nums;'
                         f'font-weight:600">{v}</td></tr>\n')
            html += "</table>\n"

        html += "    </div>\n"

    # Config suggestions
    if suggestions:
        html += """
    <h2>Config Update Suggestions</h2>
    <div class="suggestion">
        <p>Based on measured sensor data, consider updating
           <code>6dof_config_advanced.yaml</code>:</p>
        <pre>
"""
        for key, val in suggestions.items():
            html += f"  {key}: {val}\n"
        html += """</pre>
    </div>
"""

    # Embedded plots
    html += """
    <h2>Plots</h2>
    <p>Open the individual HTML plot files for interactive charts:</p>
    <ul>
"""
    for plot_file in sorted(result_dir.rglob("*.plot.html")):
        html += f"    <li><a href='{plot_file.relative_to(result_dir)}'>{plot_file.name}</a></li>\n"
    html += """    </ul>
"""

    html += """
</div>
</body>
</html>
"""

    summary_path = result_dir / "summary.html"
    with open(summary_path, "w", encoding="utf-8") as f:
        f.write(html)
    logger.info(f"Summary HTML saved: {summary_path}")


# ============================================================================
# Analyze existing results
# ============================================================================

def analyze_results(result_dir: Path):
    """Run analysis on existing CSV results and generate plots."""

    # Allan plots
    allan_json = result_dir / "allan" / "allan.metrics.json"
    if allan_json.exists():
        with open(allan_json) as f:
            m = json.load(f)
        if "allan_results" in m:
            plot_allan_html(m["allan_results"],
                            result_dir / "allan" / "allan.plot.html")

    # Static plots
    static_csv = result_dir / "static" / "sensor_imu.csv"
    if static_csv.exists():
        plot_static_html(static_csv, result_dir / "static" / "static.plot.html")

    # Vibration plots
    vib_csv = result_dir / "vibration" / "sensor_imu.csv"
    if vib_csv.exists():
        plot_vibration_html(vib_csv, result_dir / "vibration" / "vibration.plot.html")

    # GPS plots
    gps_csv = result_dir / "gps" / "sensor_gps.csv"
    if gps_csv.exists():
        plot_gps_html(gps_csv, result_dir / "gps" / "gps.plot.html")

    # Temperature plots
    temp_csv = result_dir / "temperature" / "sensor_imu.csv"
    if temp_csv.exists():
        plot_temperature_html(temp_csv, result_dir / "temperature" / "temperature.plot.html")

    # Config suggestions
    suggestions = suggest_config_updates(result_dir)
    if suggestions:
        print("\n  Config update suggestions (6dof_config_advanced.yaml):")
        for key, val in suggestions.items():
            print(f"    {key}: {val}")
        with open(result_dir / "config_suggestions.json", "w") as f:
            json.dump(suggestions, f, indent=2)

    # Summary HTML
    generate_summary_html(result_dir)

    print(f"\n  Analysis complete → {result_dir}/summary.html")


# ============================================================================
# CLI
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Sensor data analysis — Allan variance, noise, plots")
    parser.add_argument("result_dir", type=str,
                        help="Path to results directory from sensor_runner")
    parser.add_argument("--format", choices=["html", "png"], default="html",
                        help="Plot format (default: html)")
    parser.add_argument("-v", "--verbose", action="store_true")

    args = parser.parse_args()

    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=level,
                        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s")

    result_dir = Path(args.result_dir)
    if not result_dir.exists():
        print(f"Error: {result_dir} does not exist")
        sys.exit(1)

    analyze_results(result_dir)


if __name__ == "__main__":
    main()
