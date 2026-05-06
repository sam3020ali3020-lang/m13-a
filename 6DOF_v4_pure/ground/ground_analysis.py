#!/usr/bin/env python3
"""
ground_analysis.py — تحليل نتائج الاختبار الأرضي
==================================================

يُنتج:
  - مخطط EKF2 (innovations, flags, accuracy)
  - مخطط MPC/MHE timing (histogram + time series)
  - مخطط CPU + حرارة
  - مخطط Attitude drift
  - مقارنة مع PIL (اختياري)
  - تقرير HTML شامل

الاستخدام:
    python3 ground_analysis.py results/20260502_213000/
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path

import numpy as np

logger = logging.getLogger("ground_analysis")


# ============================================================================
# Plot generators
# ============================================================================

def plot_ekf2_html(result_dir: Path):
    """EKF2 innovation ratios, flags, and accuracy over time."""
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError:
        logger.warning("plotly not installed — skipping EKF2 plot")
        return

    csv_path = result_dir / "ground_estimator.csv"
    if not csv_path.exists():
        return

    data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None,
                         encoding="utf-8")
    if len(data) < 3:
        return

    t = data["t_wall_s"].astype(float)
    t = t - t[0]

    fig = make_subplots(rows=3, cols=2,
                        subplot_titles=(
                            "Innovation Ratios (< 1.0 = healthy)",
                            "Position Accuracy (m)",
                            "EKF2 Flags",
                            "Velocity Ratio Detail",
                            "Horizontal Position Ratio",
                            "Magnetometer Ratio",
                        ),
                        vertical_spacing=0.08)

    # Innovation ratios
    for col, color, label in [
        ("vel_ratio", "#e74c3c", "Velocity"),
        ("pos_horiz_ratio", "#2ecc71", "Pos Horiz"),
        ("pos_vert_ratio", "#3498db", "Pos Vert"),
        ("mag_ratio", "#9b59b6", "Mag"),
    ]:
        fig.add_trace(go.Scatter(
            x=t, y=data[col].astype(float), mode="lines",
            name=label, line=dict(width=1),
        ), row=1, col=1)
    # Threshold line at 1.0
    fig.add_hline(y=1.0, line_dash="dash", line_color="red",
                  annotation_text="Threshold", row=1, col=1)

    # Position accuracy
    fig.add_trace(go.Scatter(
        x=t, y=data["pos_horiz_accuracy_m"].astype(float), mode="lines",
        name="Horiz Acc", line=dict(color="#e74c3c"),
    ), row=1, col=2)
    fig.add_trace(go.Scatter(
        x=t, y=data["pos_vert_accuracy_m"].astype(float), mode="lines",
        name="Vert Acc", line=dict(color="#3498db"),
    ), row=1, col=2)

    # Flags as integer
    flags = []
    for f_str in data["flags"]:
        try:
            flags.append(int(str(f_str), 16))
        except (ValueError, TypeError):
            flags.append(0)
    fig.add_trace(go.Scatter(
        x=t, y=flags, mode="lines",
        name="Flags", line=dict(color="#e67e22"),
    ), row=2, col=1)

    # Individual ratio details
    fig.add_trace(go.Scatter(
        x=t, y=data["vel_ratio"].astype(float), mode="lines",
        name="Vel", line=dict(color="#e74c3c"), showlegend=False,
    ), row=2, col=2)

    fig.add_trace(go.Scatter(
        x=t, y=data["pos_horiz_ratio"].astype(float), mode="lines",
        name="Pos H", line=dict(color="#2ecc71"), showlegend=False,
    ), row=3, col=1)

    fig.add_trace(go.Scatter(
        x=t, y=data["mag_ratio"].astype(float), mode="lines",
        name="Mag", line=dict(color="#9b59b6"), showlegend=False,
    ), row=3, col=2)

    fig.update_layout(title="EKF2 Health", height=900, width=1100,
                      template="plotly_white")
    for r in range(1, 4):
        for c in range(1, 3):
            fig.update_xaxes(title_text="Time (s)", row=r, col=c)

    out_path = result_dir / "ekf2.plot.html"
    fig.write_html(str(out_path))
    logger.info(f"EKF2 plot saved: {out_path}")


def plot_timing_html(result_dir: Path):
    """MPC/MHE/cycle timing histograms and time series."""
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError:
        logger.warning("plotly not installed — skipping timing plot")
        return

    csv_path = result_dir / "ground_timing.csv"
    if not csv_path.exists():
        return

    data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None,
                         encoding="utf-8")
    if len(data) < 10:
        return

    t = data["t_wall_s"].astype(float)
    t = t - t[0]
    mpc_ms = data["mpc_solve_us"].astype(float) / 1000.0
    mhe_ms = data["mhe_solve_us"].astype(float) / 1000.0
    cycle_ms = data["cycle_us"].astype(float) / 1000.0

    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=(
                            "MPC Solve Time (ms)",
                            "MPC Histogram",
                            "MHE + Cycle Time (ms)",
                            "Cycle Histogram",
                        ),
                        vertical_spacing=0.12)

    # MPC time series
    fig.add_trace(go.Scatter(
        x=t, y=mpc_ms, mode="lines",
        name="MPC", line=dict(color="#e74c3c", width=0.8),
    ), row=1, col=1)
    # p99 line
    p99 = float(np.percentile(mpc_ms, 99))
    fig.add_hline(y=p99, line_dash="dash", line_color="orange",
                  annotation_text=f"p99={p99:.1f}ms", row=1, col=1)

    # MPC histogram
    fig.add_trace(go.Histogram(
        x=mpc_ms, nbinsx=60, name="MPC dist",
        marker_color="#e74c3c", opacity=0.7,
    ), row=1, col=2)

    # MHE + Cycle time series
    fig.add_trace(go.Scatter(
        x=t, y=mhe_ms, mode="lines",
        name="MHE", line=dict(color="#3498db", width=0.8),
    ), row=2, col=1)
    fig.add_trace(go.Scatter(
        x=t, y=cycle_ms, mode="lines",
        name="Cycle", line=dict(color="#2ecc71", width=0.8),
    ), row=2, col=1)

    # Cycle histogram
    fig.add_trace(go.Histogram(
        x=cycle_ms, nbinsx=60, name="Cycle dist",
        marker_color="#2ecc71", opacity=0.7,
    ), row=2, col=2)

    fig.update_layout(title="MPC/MHE Timing — Ground Test", height=700, width=1100,
                      template="plotly_white")
    fig.update_xaxes(title_text="Time (s)", row=1, col=1)
    fig.update_xaxes(title_text="Time (s)", row=2, col=1)
    fig.update_xaxes(title_text="ms", row=1, col=2)
    fig.update_xaxes(title_text="ms", row=2, col=2)
    fig.update_yaxes(title_text="ms", row=1, col=1)
    fig.update_yaxes(title_text="ms", row=2, col=1)

    out_path = result_dir / "timing.plot.html"
    fig.write_html(str(out_path))
    logger.info(f"Timing plot saved: {out_path}")


def plot_system_html(result_dir: Path):
    """CPU load and temperature over time."""
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError:
        logger.warning("plotly not installed — skipping system plot")
        return

    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=(
                            "CPU Load (%)",
                            "Temperature (°C)",
                            "Attitude (deg) — should be ~0",
                            "Yaw Drift",
                        ),
                        vertical_spacing=0.12)

    # CPU Load
    sys_csv = result_dir / "ground_sys_status.csv"
    if sys_csv.exists():
        data = np.genfromtxt(sys_csv, delimiter=",", names=True, dtype=None,
                             encoding="utf-8")
        if len(data) > 3:
            t = data["t_wall_s"].astype(float)
            t = t - t[0]
            cpu = data["cpu_load_permille"].astype(float) / 10.0
            fig.add_trace(go.Scatter(
                x=t, y=cpu, mode="lines",
                name="CPU %", line=dict(color="#e74c3c"),
            ), row=1, col=1)

    # Temperature from IMU
    imu_csv = result_dir / "ground_imu.csv"
    if imu_csv.exists():
        data = np.genfromtxt(imu_csv, delimiter=",", names=True, dtype=None,
                             encoding="utf-8")
        if len(data) > 10:
            t = data["t_wall_s"].astype(float)
            t = t - t[0]
            temp = data["temperature"].astype(float)
            if np.any(temp != 0):
                fig.add_trace(go.Scatter(
                    x=t, y=temp, mode="lines",
                    name="Temperature", line=dict(color="#e67e22"),
                ), row=1, col=2)

    # Attitude
    att_csv = result_dir / "ground_attitude.csv"
    if att_csv.exists():
        data = np.genfromtxt(att_csv, delimiter=",", names=True, dtype=None,
                             encoding="utf-8")
        if len(data) > 10:
            t = data["t_wall_s"].astype(float)
            t = t - t[0]
            roll = np.degrees(data["roll_rad"].astype(float))
            pitch = np.degrees(data["pitch_rad"].astype(float))
            yaw = np.degrees(data["yaw_rad"].astype(float))

            fig.add_trace(go.Scatter(
                x=t, y=roll, mode="lines",
                name="Roll", line=dict(color="#e74c3c", width=0.8),
            ), row=2, col=1)
            fig.add_trace(go.Scatter(
                x=t, y=pitch, mode="lines",
                name="Pitch", line=dict(color="#2ecc71", width=0.8),
            ), row=2, col=1)

            # Yaw
            fig.add_trace(go.Scatter(
                x=t, y=yaw, mode="lines",
                name="Yaw", line=dict(color="#3498db"),
            ), row=2, col=2)

    fig.update_layout(title="System Health — Ground Test", height=700, width=1100,
                      template="plotly_white")
    for r in range(1, 3):
        for c in range(1, 3):
            fig.update_xaxes(title_text="Time (s)", row=r, col=c)

    out_path = result_dir / "system.plot.html"
    fig.write_html(str(out_path))
    logger.info(f"System plot saved: {out_path}")


def plot_pil_comparison_html(result_dir: Path, pil_csv: Path):
    """Side-by-side MPC timing comparison: ground vs PIL."""
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError:
        return

    ground_csv = result_dir / "ground_timing.csv"
    if not ground_csv.exists() or not pil_csv.exists():
        return

    g = np.genfromtxt(ground_csv, delimiter=",", names=True, dtype=None,
                      encoding="utf-8")
    p = np.genfromtxt(pil_csv, delimiter=",", names=True, dtype=None,
                      encoding="utf-8")

    fig = make_subplots(rows=1, cols=2,
                        subplot_titles=("MPC Solve Time Distribution",
                                        "Cumulative Distribution"))

    g_mpc = g["mpc_solve_us"].astype(float) / 1000.0
    p_mpc = None
    for col in ["mpc_solve_us", "mpc_us"]:
        if col in p.dtype.names:
            p_mpc = p[col].astype(float) / 1000.0
            break
    if p_mpc is None:
        return
    p_mpc = p_mpc[p_mpc > 0]

    fig.add_trace(go.Histogram(
        x=g_mpc, nbinsx=50, name="Ground (real sensors)",
        marker_color="#e74c3c", opacity=0.6,
    ), row=1, col=1)
    fig.add_trace(go.Histogram(
        x=p_mpc, nbinsx=50, name="PIL (simulated sensors)",
        marker_color="#3498db", opacity=0.6,
    ), row=1, col=1)

    # CDF
    for vals, name, color in [(g_mpc, "Ground", "#e74c3c"),
                               (p_mpc, "PIL", "#3498db")]:
        sorted_v = np.sort(vals)
        cdf = np.arange(1, len(sorted_v) + 1) / len(sorted_v) * 100
        fig.add_trace(go.Scatter(
            x=sorted_v, y=cdf, mode="lines",
            name=f"{name} CDF", line=dict(color=color),
        ), row=1, col=2)

    fig.update_xaxes(title_text="MPC Solve (ms)", row=1, col=1)
    fig.update_xaxes(title_text="MPC Solve (ms)", row=1, col=2)
    fig.update_yaxes(title_text="Count", row=1, col=1)
    fig.update_yaxes(title_text="Percentile (%)", row=1, col=2)
    fig.update_layout(title="Ground vs PIL — MPC Timing", height=400, width=1100,
                      template="plotly_white", barmode="overlay")

    out_path = result_dir / "pil_comparison.plot.html"
    fig.write_html(str(out_path))
    logger.info(f"PIL comparison plot saved: {out_path}")


# ============================================================================
# HTML summary
# ============================================================================

def generate_summary_html(result_dir: Path):
    """Generate comprehensive HTML report."""
    metrics_path = result_dir / "ground_metrics.json"
    if not metrics_path.exists():
        return

    with open(metrics_path) as f:
        metrics = json.load(f)

    go_nogo_path = result_dir / "GO_NOGO.txt"
    verdict = "?"
    if go_nogo_path.exists():
        with open(go_nogo_path) as f:
            verdict = f.readline().strip().replace("VERDICT:", "").strip()

    html = f"""<!DOCTYPE html>
<html lang="ar">
<head>
    <meta charset="UTF-8">
    <title>/ground — Ground Integration Test Report</title>
    <style>
        body {{ font-family: 'Segoe UI', sans-serif; margin: 40px; background: #f5f5f5; }}
        .container {{ max-width: 1200px; margin: 0 auto; }}
        h1 {{ color: #2c3e50; border-bottom: 3px solid #e67e22; padding-bottom: 10px; }}
        h2 {{ color: #34495e; margin-top: 30px; }}
        .verdict {{ font-size: 2em; padding: 20px; text-align: center; border-radius: 10px;
                    margin: 20px 0; font-weight: bold; }}
        .verdict.go {{ background: #d4edda; color: #155724; border: 2px solid #28a745; }}
        .verdict.nogo {{ background: #f8d7da; color: #721c24; border: 2px solid #dc3545; }}
        .card {{ background: white; border-radius: 8px; padding: 20px; margin: 15px 0;
                 box-shadow: 0 2px 4px rgba(0,0,0,0.1); }}
        .pass {{ color: #28a745; font-weight: bold; }}
        .fail {{ color: #dc3545; font-weight: bold; }}
        table {{ border-collapse: collapse; width: 100%; margin: 10px 0; }}
        th, td {{ border: 1px solid #dee2e6; padding: 8px 12px; text-align: left; }}
        th {{ background: #f8f9fa; }}
        .metric-grid {{ display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 10px; }}
        .metric-box {{ background: #f8f9fa; padding: 12px; border-radius: 5px; text-align: center; }}
        .metric-box .value {{ font-size: 1.5em; font-weight: bold; color: #2c3e50; }}
        .metric-box .label {{ font-size: 0.85em; color: #7f8c8d; }}
    </style>
</head>
<body>
<div class="container">
    <h1>/ground — Ground Integration Test Report</h1>
    <p>Timestamp: {result_dir.name}</p>
    <p>حساسات حقيقية + EKF2 + MPC — الهاتف ثابت على الأرض</p>

    <div class="verdict {'go' if verdict == 'GO' else 'nogo'}">
        {'&#10004;' if verdict == 'GO' else '&#10008;'} {verdict}
    </div>
"""

    for section_name, section_data in metrics.items():
        passed = section_data.get("passed")
        status = "PASS" if passed else ("FAIL" if passed is False else "NO DATA")
        status_class = "pass" if passed else "fail"

        html += f"""
    <div class="card">
        <h2>{section_name} <span class="{status_class}">[{status}]</span></h2>
"""
        # Failures
        for fail in section_data.get("failures", []):
            html += f"<p class='fail'>&#10008; {fail}</p>\n"

        # Metrics table — flatten nested dicts so every numeric value is shown.
        def _flatten(d, prefix=""):
            for k, v in d.items():
                if k in ("passed", "failures", "error", "flags_decoded"):
                    continue
                key = f"{prefix}{k}" if not prefix else f"{prefix}.{k}"
                if isinstance(v, dict):
                    yield from _flatten(v, key)
                elif isinstance(v, list):
                    if len(v) <= 6:
                        yield (key, "[" + ", ".join(
                            f"{x:.4g}" if isinstance(x, float) else str(x)
                            for x in v) + "]")
                    else:
                        yield (key, f"[{len(v)} items]")
                else:
                    if isinstance(v, float):
                        yield (key, f"{v:.4g}")
                    else:
                        yield (key, str(v))
        flat_pairs = list(_flatten(section_data))
        if flat_pairs:
            html += '<table><tr><th>Metric</th><th style="text-align:right">Value</th></tr>\n'
            for k, v in flat_pairs:
                html += (f'<tr><td style="font-family:ui-monospace,monospace;'
                         f'font-size:.85rem">{k}</td>'
                         f'<td style="text-align:right;font-variant-numeric:tabular-nums;'
                         f'font-weight:600">{v}</td></tr>\n')
            html += "</table>\n"

        if section_data.get("flags_decoded"):
            html += f"<p>EKF2 flags: {', '.join(section_data['flags_decoded'])}</p>\n"
        if section_data.get("error"):
            html += f"<p class='fail'>Error: {section_data['error']}</p>\n"

        html += "    </div>\n"

    # Plot links
    html += """
    <h2>Interactive Plots</h2>
    <ul>
"""
    for plot_file in sorted(result_dir.glob("*.plot.html")):
        html += f"    <li><a href='{plot_file.name}'>{plot_file.stem}</a></li>\n"
    html += """    </ul>
"""

    html += """
    <h2>مقارنة مع الاختبارات الأخرى</h2>
    <table>
    <tr><th>الاختبار</th><th>الحساسات</th><th>EKF2</th><th>MPC</th><th>ماذا يكشف</th></tr>
    <tr><td>/sensor</td><td>✅ حقيقية</td><td>❌</td><td>❌</td><td>جودة الحساسات الخام</td></tr>
    <tr><td><strong>/ground</strong></td><td><strong>✅ حقيقية</strong></td><td><strong>✅ حقيقي</strong></td><td><strong>✅ حقيقي</strong></td><td><strong>هل النظام يعمل ككل</strong></td></tr>
    <tr><td>/pil</td><td>❌ وهمية</td><td>❌</td><td>✅ حقيقي</td><td>أداء المعالج فقط</td></tr>
    <tr><td>/hil</td><td>❌ وهمية</td><td>✅ حقيقي</td><td>✅ حقيقي</td><td>طيران محاكى كامل</td></tr>
    </table>
</div>
</body>
</html>
"""

    summary_path = result_dir / "summary.html"
    with open(summary_path, "w", encoding="utf-8") as f:
        f.write(html)
    logger.info(f"Summary HTML: {summary_path}")


# ============================================================================
# Main entry point
# ============================================================================

def analyze_results(result_dir: Path, pil_csv: Path = None):
    """Run all analysis on existing results."""
    print(f"\n  Analyzing: {result_dir}")

    plot_ekf2_html(result_dir)
    plot_timing_html(result_dir)
    plot_system_html(result_dir)

    if pil_csv and pil_csv.exists():
        plot_pil_comparison_html(result_dir, pil_csv)

    generate_summary_html(result_dir)
    print(f"  Done → {result_dir}/summary.html\n")


def main():
    parser = argparse.ArgumentParser(
        description="Ground test analysis — EKF2, timing, system plots")
    parser.add_argument("result_dir", type=str)
    parser.add_argument("--pil-csv", type=str, default=None)
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=level,
                        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s")

    result_dir = Path(args.result_dir)
    if not result_dir.exists():
        print(f"Error: {result_dir} does not exist")
        sys.exit(1)

    pil_csv = Path(args.pil_csv) if args.pil_csv else None
    analyze_results(result_dir, pil_csv)


if __name__ == "__main__":
    main()
