#!/usr/bin/env python3
"""
════════════════════════════════════════════════════════════════════════
  M130 SITL vs Standalone — Side-by-Side Comparison Report
  مقارنة SITL مع المحاكاة المستقلة — تقرير HTML تفاعلي
════════════════════════════════════════════════════════════════════════

Usage:
    python compare_sitl_vs_standalone.py
    python compare_sitl_vs_standalone.py --sitl <csv> --standalone <csv>
    python compare_sitl_vs_standalone.py --no-open
"""

import sys
import os
import argparse
import webbrowser
from pathlib import Path
from datetime import datetime
from html import escape as html_escape

import numpy as np
import pandas as pd
import yaml

import plotly.graph_objects as go
from plotly.subplots import make_subplots
import plotly.io as pio

_SCRIPT_DIR = Path(__file__).resolve().parent
_SIM_DIR = _SCRIPT_DIR.parent
_RESULTS_DIR = _SCRIPT_DIR / "results"
_STANDALONE_DIR = _SIM_DIR / "results"
_CONFIG_PATH = _SIM_DIR / "config" / "6dof_config_advanced.yaml"

try:
    with open(_CONFIG_PATH, "r") as _f:
        _cfg = yaml.safe_load(_f)
    TARGET_RANGE_M = float(_cfg.get("target", {}).get("range_m", 2600.0))
    LAUNCH_ALT_M = float(_cfg.get("launch", {}).get("altitude", 1200.0))
except Exception:
    TARGET_RANGE_M = 2600.0
    LAUNCH_ALT_M = 1200.0

# ══════════════════════════════════════════════════════════════════════
#  Unified Loader — normalize both CSV formats to common columns
# ══════════════════════════════════════════════════════════════════════

def _safe_col(df, col, default=0):
    return df[col] if col in df.columns else default


def _load_standalone(path):
    df = pd.read_csv(path)
    out = pd.DataFrame()
    out["time"] = df["time_s"]
    out["speed"] = df["velocity_total_m_s"]
    out["altitude"] = df["altitude_m"] - LAUNCH_ALT_M
    out["ground_range"] = df["ground_range_m"]
    out["alpha_deg"] = df["alpha_deg"]
    out["beta_deg"] = df["beta_deg"]
    out["mach"] = df["mach"]
    out["q_dynamic_Pa"] = _safe_col(df, "q_dynamic_Pa")
    out["pitch_deg"] = df["pitch_deg"]
    out["yaw_deg"] = _safe_col(df, "yaw_deg")
    out["roll_deg"] = _safe_col(df, "roll_deg")
    out["omega_x_deg"] = _safe_col(df, "omega_x_deg_s")
    out["omega_y_deg"] = _safe_col(df, "omega_y_deg_s")
    out["omega_z_deg"] = _safe_col(df, "omega_z_deg_s")
    out["mass"] = df["mass_kg"]
    out["force_x"] = _safe_col(df, "force_x_N")
    out["force_y"] = _safe_col(df, "force_y_N")
    out["force_z"] = _safe_col(df, "force_z_N")
    out["thrust_x"] = _safe_col(df, "thrust_x_N")
    out["moment_x"] = _safe_col(df, "moment_x_Nm")
    out["moment_y"] = _safe_col(df, "moment_y_Nm")
    out["moment_z"] = _safe_col(df, "moment_z_Nm")
    # Individual fins
    for i in range(1, 5):
        out[f"fin_{i}_deg"] = np.degrees(_safe_col(df, f"fin_{i}_rad"))
    # Virtual commands
    if "delta_pitch_rad" in df.columns:
        out["fin_pitch_deg"] = np.degrees(df["delta_pitch_rad"])
        out["fin_yaw_deg"] = np.degrees(df["delta_yaw_rad"])
        out["fin_roll_deg"] = np.degrees(_safe_col(df, "delta_roll_rad"))
    else:
        out["fin_pitch_deg"] = (out["fin_1_deg"] + out["fin_2_deg"] - out["fin_3_deg"] - out["fin_4_deg"]) / 4
        out["fin_yaw_deg"] = (out["fin_1_deg"] - out["fin_2_deg"] + out["fin_3_deg"] - out["fin_4_deg"]) / 4
        out["fin_roll_deg"] = (out["fin_1_deg"] + out["fin_2_deg"] + out["fin_3_deg"] + out["fin_4_deg"]) / 4
    # G-load
    if "acceleration_body_x_g" in df.columns:
        out["g_total"] = np.sqrt(df["acceleration_body_x_g"]**2 +
                                  df["acceleration_body_y_g"]**2 +
                                  df["acceleration_body_z_g"]**2)
        out["g_axial"] = df["acceleration_body_x_g"]
    else:
        out["g_total"] = 0
        out["g_axial"] = 0
    # Energy
    out["KE_kJ"] = 0.5 * out["mass"] * out["speed"]**2 / 1000
    out["PE_kJ"] = out["mass"] * 9.81 * out["altitude"] / 1000
    out["TE_kJ"] = out["KE_kJ"] + out["PE_kJ"]
    # Aero coefficients (standalone only)
    out["CN_total"] = _safe_col(df, "CN_total")
    out["CM_total"] = _safe_col(df, "CM_total")
    out["static_margin"] = _safe_col(df, "static_margin_cal")
    # MPC diagnostics (standalone only)
    out["mpc_gamma_ref"] = _safe_col(df, "mpc_gamma_ref_deg")
    out["mpc_solve_ms"] = _safe_col(df, "mpc_solve_time_ms")
    out["mpc_delta_e_deg"] = np.degrees(_safe_col(df, "mpc_delta_e_rad"))
    # Lat/Lon
    out["lat"] = _safe_col(df, "latitude_deg")
    out["lon"] = _safe_col(df, "longitude_deg")
    # Flight phase
    out["phase"] = _safe_col(df, "flight_phase", "UNKNOWN")
    return out


def _load_sitl(path):
    df = pd.read_csv(path)
    out = pd.DataFrame()
    out["time"] = df["time"]
    out["speed"] = np.sqrt(df["vel_x"]**2 + df["vel_y"]**2 + df["vel_z"]**2)
    out["altitude"] = df["altitude"]
    out["ground_range"] = df["ground_range"]
    out["alpha_deg"] = np.degrees(df["alpha"])
    out["beta_deg"] = np.degrees(df["beta"])
    out["mach"] = df["mach"]
    # Dynamic pressure approximation
    rho = 1.225 * (1 - 2.25577e-5 * (out["altitude"] + LAUNCH_ALT_M))**4.25588
    out["q_dynamic_Pa"] = 0.5 * rho * out["speed"]**2
    # Euler from quaternion
    q0, q1, q2, q3 = df["q0"], df["q1"], df["q2"], df["q3"]
    out["pitch_deg"] = np.degrees(np.arcsin(np.clip(2*(q0*q2 - q3*q1), -1, 1)))
    out["yaw_deg"] = np.degrees(np.arctan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2)))
    out["roll_deg"] = np.degrees(np.arctan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2)))
    out["omega_x_deg"] = np.degrees(df["omega_x"])
    out["omega_y_deg"] = np.degrees(df["omega_y"])
    out["omega_z_deg"] = np.degrees(df["omega_z"])
    out["mass"] = df["mass"]
    out["force_x"] = df["force_x"]
    out["force_y"] = df["force_y"]
    out["force_z"] = df["force_z"]
    out["thrust_x"] = 0  # not in SITL CSV
    out["moment_x"] = 0
    out["moment_y"] = 0
    out["moment_z"] = 0
    # Individual fins (commanded)
    for i in range(1, 5):
        out[f"fin_{i}_deg"] = np.degrees(df[f"fin_cmd_{i}"])
    # Actual fins
    for i in range(1, 5):
        out[f"fin_act_{i}_deg"] = np.degrees(df[f"fin_act_{i}"])
    # Virtual commands
    c1, c2, c3, c4 = out["fin_1_deg"], out["fin_2_deg"], out["fin_3_deg"], out["fin_4_deg"]
    out["fin_pitch_deg"] = (c1 + c2 - c3 - c4) / 4
    out["fin_yaw_deg"] = (c1 - c2 + c3 - c4) / 4
    out["fin_roll_deg"] = (c1 + c2 + c3 + c4) / 4
    # G-load
    out["g_total"] = np.sqrt(df["accel_x"]**2 + df["accel_y"]**2 + df["accel_z"]**2) / 9.80665
    out["g_axial"] = df["accel_x"] / 9.80665
    # Energy
    out["KE_kJ"] = 0.5 * out["mass"] * out["speed"]**2 / 1000
    out["PE_kJ"] = out["mass"] * 9.81 * out["altitude"] / 1000
    out["TE_kJ"] = out["KE_kJ"] + out["PE_kJ"]
    # Aero coefficients not available in SITL
    out["CN_total"] = 0
    out["CM_total"] = 0
    out["static_margin"] = np.nan
    out["mpc_gamma_ref"] = np.nan
    out["mpc_solve_ms"] = np.nan
    out["mpc_delta_e_deg"] = np.nan
    out["lat"] = df["lat"]
    out["lon"] = df["lon"]
    out["phase"] = "SITL"
    return out


# ══════════════════════════════════════════════════════════════════════
#  Metrics
# ══════════════════════════════════════════════════════════════════════

def _metrics(df, label):
    m = {"label": label}
    m["flight_time"] = df["time"].iloc[-1]
    m["n_steps"] = len(df)
    m["dt"] = np.median(np.diff(df["time"].values[:100]))
    m["range"] = df["ground_range"].iloc[-1]
    m["range_err"] = df["ground_range"].iloc[-1] - TARGET_RANGE_M
    m["range_err_pct"] = (df["ground_range"].iloc[-1] / TARGET_RANGE_M - 1) * 100
    m["peak_alt"] = df["altitude"].max()
    m["peak_alt_time"] = df.loc[df["altitude"].idxmax(), "time"]
    m["max_speed"] = df["speed"].max()
    m["final_speed"] = df["speed"].iloc[-1]
    m["max_mach"] = df["mach"].max()
    m["max_alpha"] = df["alpha_deg"].abs().max()
    m["mean_alpha"] = df["alpha_deg"].abs().mean()
    m["max_beta"] = df["beta_deg"].abs().max()
    m["mean_beta"] = df["beta_deg"].abs().mean()
    m["max_g"] = df["g_total"].max()
    m["max_g_axial"] = df["g_axial"].max() if "g_axial" in df.columns else 0
    m["max_q_Pa"] = df["q_dynamic_Pa"].max() if isinstance(df["q_dynamic_Pa"], pd.Series) else 0
    # Attitude
    m["max_pitch"] = df["pitch_deg"].max()
    m["min_pitch"] = df["pitch_deg"].min()
    m["max_roll"] = df["roll_deg"].abs().max() if isinstance(df["roll_deg"], pd.Series) else 0
    m["max_yaw"] = df["yaw_deg"].abs().max() if isinstance(df["yaw_deg"], pd.Series) else 0
    # Angular rates
    m["max_omega_x"] = df["omega_x_deg"].abs().max()
    m["max_omega_y"] = df["omega_y_deg"].abs().max()
    m["max_omega_z"] = df["omega_z_deg"].abs().max()
    m["omega_total_max"] = np.sqrt(df["omega_x_deg"]**2 + df["omega_y_deg"]**2 + df["omega_z_deg"]**2).max()
    # Control
    m["max_fin_pitch"] = df["fin_pitch_deg"].abs().max()
    m["max_fin_yaw"] = df["fin_yaw_deg"].abs().max()
    m["max_fin_roll"] = df["fin_roll_deg"].abs().max() if "fin_roll_deg" in df.columns else 0
    m["mean_fin_pitch"] = df["fin_pitch_deg"].mean()
    for i in range(1, 5):
        col = f"fin_{i}_deg"
        if col in df.columns:
            m[f"max_fin{i}"] = df[col].abs().max()
    # Mass
    m["mass_i"] = df["mass"].iloc[0]
    m["mass_f"] = df["mass"].iloc[-1]
    m["propellant"] = df["mass"].iloc[0] - df["mass"].iloc[-1]
    # Forces
    m["max_force_x"] = df["force_x"].abs().max() if isinstance(df["force_x"], pd.Series) else 0
    m["max_force_z"] = df["force_z"].abs().max() if isinstance(df["force_z"], pd.Series) else 0
    m["max_thrust"] = df["thrust_x"].max() if isinstance(df["thrust_x"], pd.Series) else 0
    # Stability
    n30 = max(1, int(len(df) * 0.3))
    m["pitch_std"] = df["pitch_deg"].iloc[-n30:].std()
    m["alpha_std"] = df["alpha_deg"].iloc[-n30:].std()
    # Energy
    m["max_KE"] = df["KE_kJ"].max()
    m["max_PE"] = df["PE_kJ"].max()
    m["max_TE"] = df["TE_kJ"].max()
    return m


# ══════════════════════════════════════════════════════════════════════
#  Comparison Figures
# ══════════════════════════════════════════════════════════════════════

C_STANDALONE = "#2196f3"
C_SITL = "#ff5722"


def _overlay(fig, ds, dl, col, row, coln, name_s="Standalone", name_l="SITL", show_legend=True):
    fig.add_trace(go.Scatter(x=ds["time"], y=ds[col], name=name_s,
                             line=dict(color=C_STANDALONE, width=2),
                             legendgroup="standalone", showlegend=show_legend), row=row, col=coln)
    fig.add_trace(go.Scatter(x=dl["time"], y=dl[col], name=name_l,
                             line=dict(color=C_SITL, width=2, dash="dot"),
                             legendgroup="sitl", showlegend=show_legend), row=row, col=coln)


def fig_trajectory(ds, dl):
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Altitude AGL", "Ground Range",
                                        "Trajectory Profile", "Speed"))
    _overlay(fig, ds, dl, "altitude", 1, 1)
    _overlay(fig, ds, dl, "ground_range", 1, 2, show_legend=False)
    # Profile: range vs alt
    fig.add_trace(go.Scatter(x=ds["ground_range"], y=ds["altitude"], name="Standalone",
                             line=dict(color=C_STANDALONE, width=2),
                             legendgroup="standalone", showlegend=False), row=2, col=1)
    fig.add_trace(go.Scatter(x=dl["ground_range"], y=dl["altitude"], name="SITL",
                             line=dict(color=C_SITL, width=2, dash="dot"),
                             legendgroup="sitl", showlegend=False), row=2, col=1)
    _overlay(fig, ds, dl, "speed", 2, 2, show_legend=False)
    fig.add_hline(y=TARGET_RANGE_M, line_dash="dash", line_color="gray", opacity=0.5, row=1, col=2,
                  annotation_text=f"Target {TARGET_RANGE_M:.0f}m")
    fig.update_xaxes(title_text="Time (s)", row=1, col=1)
    fig.update_xaxes(title_text="Time (s)", row=1, col=2)
    fig.update_xaxes(title_text="Range (m)", row=2, col=1)
    fig.update_xaxes(title_text="Time (s)", row=2, col=2)
    fig.update_yaxes(title_text="Alt (m)", row=1, col=1)
    fig.update_yaxes(title_text="Range (m)", row=1, col=2)
    fig.update_yaxes(title_text="Alt (m)", row=2, col=1)
    fig.update_yaxes(title_text="Speed (m/s)", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white",
                      legend=dict(orientation="h", yanchor="bottom", y=1.02))
    return fig


def fig_attitude(ds, dl):
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Pitch", "α (AoA)",
                                        "Pitch Rate q", "β (Sideslip)"))
    _overlay(fig, ds, dl, "pitch_deg", 1, 1)
    _overlay(fig, ds, dl, "alpha_deg", 1, 2, show_legend=False)
    _overlay(fig, ds, dl, "omega_y_deg", 2, 1, show_legend=False)
    _overlay(fig, ds, dl, "beta_deg", 2, 2, show_legend=False)
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="°", row=1, col=1)
    fig.update_yaxes(title_text="°", row=1, col=2)
    fig.update_yaxes(title_text="°/s", row=2, col=1)
    fig.update_yaxes(title_text="°", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white")
    return fig


def fig_aero(ds, dl):
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Mach", "G-Load",
                                        "Mass", "Fin Pitch Command"))
    _overlay(fig, ds, dl, "mach", 1, 1)
    _overlay(fig, ds, dl, "g_total", 1, 2, show_legend=False)
    _overlay(fig, ds, dl, "mass", 2, 1, show_legend=False)
    _overlay(fig, ds, dl, "fin_pitch_deg", 2, 2, show_legend=False)
    fig.add_hline(y=1.0, line_dash="dash", line_color="red", opacity=0.4, row=1, col=1,
                  annotation_text="Mach 1")
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="Mach", row=1, col=1)
    fig.update_yaxes(title_text="G", row=1, col=2)
    fig.update_yaxes(title_text="kg", row=2, col=1)
    fig.update_yaxes(title_text="°", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white")
    return fig


def fig_diff(ds, dl):
    """Difference plots: SITL − Standalone (interpolated to common time)."""
    t_max = min(ds["time"].iloc[-1], dl["time"].iloc[-1])
    t_common = np.linspace(0, t_max, 500)

    def interp(df, col):
        return np.interp(t_common, df["time"].values, df[col].values)

    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Δ Altitude (SITL − Standalone)",
                                        "Δ Ground Range",
                                        "Δ Speed",
                                        "Δ Pitch"))
    diffs = {
        "altitude": interp(dl, "altitude") - interp(ds, "altitude"),
        "ground_range": interp(dl, "ground_range") - interp(ds, "ground_range"),
        "speed": interp(dl, "speed") - interp(ds, "speed"),
        "pitch_deg": interp(dl, "pitch_deg") - interp(ds, "pitch_deg"),
    }

    positions = [("altitude", 1, 1, "m"), ("ground_range", 1, 2, "m"),
                 ("speed", 2, 1, "m/s"), ("pitch_deg", 2, 2, "°")]

    for col, r, c, unit in positions:
        d = diffs[col]
        color = np.where(d >= 0, C_SITL, C_STANDALONE)
        fig.add_trace(go.Scatter(x=t_common, y=d, mode="lines",
                                 line=dict(color="#666", width=1.5),
                                 fill="tozeroy", fillcolor="rgba(255,87,34,0.15)",
                                 name=f"Δ {col}", showlegend=False), row=r, col=c)
        fig.add_hline(y=0, line_dash="dot", line_color="gray", opacity=0.5, row=r, col=c)
        # Annotate max difference
        idx_max = np.argmax(np.abs(d))
        fig.add_annotation(x=t_common[idx_max], y=d[idx_max],
                           text=f"max Δ={d[idx_max]:+.1f}{unit}",
                           showarrow=True, arrowhead=2, font=dict(size=10),
                           row=r, col=c)

    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="Δ m", row=1, col=1)
    fig.update_yaxes(title_text="Δ m", row=1, col=2)
    fig.update_yaxes(title_text="Δ m/s", row=2, col=1)
    fig.update_yaxes(title_text="Δ °", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white",
                      title="Differences (SITL − Standalone)")
    return fig


def fig_control_compare(ds, dl):
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Fin Pitch (virtual)", "Fin Yaw (virtual)",
                                        "Fin Roll (virtual)", "Individual Fins (Standalone vs SITL)"))
    _overlay(fig, ds, dl, "fin_pitch_deg", 1, 1)
    _overlay(fig, ds, dl, "fin_yaw_deg", 1, 2, show_legend=False)
    _overlay(fig, ds, dl, "fin_roll_deg", 2, 1, show_legend=False)
    # Individual fins
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]
    for i in range(1, 5):
        col = f"fin_{i}_deg"
        if col in ds.columns:
            fig.add_trace(go.Scatter(x=ds["time"], y=ds[col], name=f"S-Fin{i}",
                                     line=dict(color=colors[i-1], width=1),
                                     legendgroup=f"fin{i}", showlegend=True), row=2, col=2)
        if col in dl.columns:
            fig.add_trace(go.Scatter(x=dl["time"], y=dl[col], name=f"L-Fin{i}",
                                     line=dict(color=colors[i-1], width=1, dash="dot"),
                                     legendgroup=f"fin{i}", showlegend=False), row=2, col=2)
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="°")
    fig.update_layout(height=700, template="plotly_white")
    return fig


def fig_forces_energy(ds, dl):
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Axial Force F_x", "Normal Force F_z",
                                        "Kinetic Energy", "Total Energy"))
    _overlay(fig, ds, dl, "force_x", 1, 1)
    _overlay(fig, ds, dl, "force_z", 1, 2, show_legend=False)
    _overlay(fig, ds, dl, "KE_kJ", 2, 1, show_legend=False)
    _overlay(fig, ds, dl, "TE_kJ", 2, 2, show_legend=False)
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="N", row=1, col=1)
    fig.update_yaxes(title_text="N", row=1, col=2)
    fig.update_yaxes(title_text="kJ", row=2, col=1)
    fig.update_yaxes(title_text="kJ", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white")
    return fig


def fig_rates_gload(ds, dl):
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Pitch Rate q", "Yaw Rate r",
                                        "Roll Rate p", "G-Load"))
    _overlay(fig, ds, dl, "omega_y_deg", 1, 1)
    _overlay(fig, ds, dl, "omega_z_deg", 1, 2, show_legend=False)
    _overlay(fig, ds, dl, "omega_x_deg", 2, 1, show_legend=False)
    _overlay(fig, ds, dl, "g_total", 2, 2, show_legend=False)
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="°/s", row=1, col=1)
    fig.update_yaxes(title_text="°/s", row=1, col=2)
    fig.update_yaxes(title_text="°/s", row=2, col=1)
    fig.update_yaxes(title_text="G", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white")
    return fig


def fig_dynamic_pressure(ds, dl):
    fig = make_subplots(rows=1, cols=2,
                        subplot_titles=("Dynamic Pressure", "Mach vs α"))
    _overlay(fig, ds, dl, "q_dynamic_Pa", 1, 1)
    # Mach vs alpha scatter
    fig.add_trace(go.Scatter(x=ds["mach"], y=ds["alpha_deg"], mode="markers",
                             name="Standalone", marker=dict(size=3, color=C_STANDALONE, opacity=0.5),
                             legendgroup="standalone", showlegend=False), row=1, col=2)
    fig.add_trace(go.Scatter(x=dl["mach"], y=dl["alpha_deg"], mode="markers",
                             name="SITL", marker=dict(size=3, color=C_SITL, opacity=0.5),
                             legendgroup="sitl", showlegend=False), row=1, col=2)
    fig.update_xaxes(title_text="Time (s)", row=1, col=1)
    fig.update_xaxes(title_text="Mach", row=1, col=2)
    fig.update_yaxes(title_text="Pa", row=1, col=1)
    fig.update_yaxes(title_text="α (°)", row=1, col=2)
    fig.update_layout(height=400, template="plotly_white")
    return fig


# ══════════════════════════════════════════════════════════════════════
#  HTML Report
# ══════════════════════════════════════════════════════════════════════

_CSS = """\
:root{--bg:#f8f9fa;--card:#fff;--border:#e0e0e0;--text:#222;--text-secondary:#666;
--accent:#1565c0;--pass:#4caf50;--warn:#ff9800;--fail:#e53935;--th-bg:#f0f4f8;
--hover:rgba(21,101,192,.04);--standalone:#2196f3;--sitl:#ff5722}
[data-theme="dark"]{--bg:#121212;--card:#1e1e1e;--border:#333;--text:#e0e0e0;
--text-secondary:#aaa;--accent:#42a5f5;--th-bg:#2a2a2a;--hover:rgba(66,165,245,.08)}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,sans-serif;background:var(--bg);
color:var(--text);line-height:1.6;padding:20px;transition:background .3s,color .3s}
.container{max-width:1400px;margin:0 auto}
h1{font-size:1.8rem;border-bottom:3px solid var(--accent);padding-bottom:8px;margin-bottom:16px}
h2{font-size:1.3rem;color:var(--accent);margin:24px 0 12px;border-left:4px solid var(--accent);padding-left:10px}
.grid{display:grid;gap:16px}.grid-2{grid-template-columns:1fr 1fr}
.grid-3{grid-template-columns:1fr 1fr 1fr}
.card{background:var(--card);border:1px solid var(--border);border-radius:8px;
padding:16px;box-shadow:0 1px 3px rgba(0,0,0,.08)}
table{width:100%;border-collapse:collapse;font-size:.85rem}
th{background:var(--th-bg);padding:8px 12px;text-align:left;border-bottom:2px solid var(--border);font-weight:600}
td{padding:6px 12px;border-bottom:1px solid var(--border)}tr:hover{background:var(--hover)}
.better{color:var(--pass);font-weight:700}.worse{color:var(--fail);font-weight:700}
.same{color:var(--text-secondary)}
.tag{display:inline-block;padding:1px 8px;border-radius:10px;font-size:.7rem;font-weight:700;color:#fff}
.tag-s{background:var(--standalone)}.tag-l{background:var(--sitl)}
.tabs{display:flex;gap:4px;border-bottom:2px solid var(--border);flex-wrap:wrap}
.tab-btn{padding:8px 20px;border:none;background:none;cursor:pointer;font-size:.9rem;
font-weight:600;border-bottom:3px solid transparent;color:var(--text-secondary);transition:.2s}
.tab-btn:hover{color:var(--accent)}.tab-btn.active{color:var(--accent);border-bottom-color:var(--accent)}
.tab-panel{display:none;padding:16px 0}.tab-panel.active{display:block}
.toolbar{display:flex;align-items:center;gap:12px;margin-bottom:16px;flex-wrap:wrap}
.theme-toggle{background:var(--card);border:1px solid var(--border);border-radius:20px;
padding:6px 14px;cursor:pointer;font-size:.85rem;color:var(--text)}
@media(max-width:900px){.grid-2,.grid-3{grid-template-columns:1fr}}
"""

_JS = """\
function openTab(evt,tabId){
  document.querySelectorAll('.tab-panel').forEach(p=>p.classList.remove('active'));
  document.querySelectorAll('.tab-btn').forEach(b=>b.classList.remove('active'));
  document.getElementById(tabId).classList.add('active');
  evt.currentTarget.classList.add('active');
  document.getElementById(tabId).querySelectorAll('.js-plotly-plot').forEach(p=>Plotly.Plots.resize(p));
}
(function(){
  var saved=localStorage.getItem('m130_cmp_theme')||'light';
  document.documentElement.setAttribute('data-theme',saved);
  window.addEventListener('DOMContentLoaded',function(){
    var btn=document.getElementById('theme-toggle');
    if(!btn)return;
    btn.textContent=saved==='dark'?'\\u2600 Light':'\\u263e Dark';
    btn.addEventListener('click',function(){
      var cur=document.documentElement.getAttribute('data-theme')||'light';
      var next=cur==='dark'?'light':'dark';
      document.documentElement.setAttribute('data-theme',next);
      localStorage.setItem('m130_cmp_theme',next);
      btn.textContent=next==='dark'?'\\u2600 Light':'\\u263e Dark';
      var bg=next==='dark'?'#1e1e1e':'#fff',fg=next==='dark'?'#e0e0e0':'#444';
      document.querySelectorAll('.js-plotly-plot').forEach(function(p){
        Plotly.relayout(p,{'paper_bgcolor':bg,'plot_bgcolor':bg,'font.color':fg});
      });
    });
  });
})();
"""

plotly_cdn = '<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>'


def _plotly_div(fig):
    return pio.to_html(fig, full_html=False, include_plotlyjs=False,
                       config={"responsive": True, "displayModeBar": True})


def _compare_row(label, val_s, val_l, unit="", fmt=".1f", lower_better=False):
    """Build a table row with color-coded winner."""
    vs = f"{val_s:{fmt}}"
    vl = f"{val_l:{fmt}}"
    diff = val_l - val_s
    diff_str = f"{diff:+{fmt}}"

    if abs(diff) < 0.01 * max(abs(val_s), abs(val_l), 1):
        cls_s, cls_l = "same", "same"
        winner = "—"
    elif (diff < 0) != lower_better:
        cls_s, cls_l = "better", "worse"
        winner = '<span class="tag tag-s">Standalone</span>'
    else:
        cls_s, cls_l = "worse", "better"
        winner = '<span class="tag tag-l">SITL</span>'

    return (f'<tr><td><b>{label}</b></td>'
            f'<td class="{cls_s}">{vs}{unit}</td>'
            f'<td class="{cls_l}">{vl}{unit}</td>'
            f'<td>{diff_str}{unit}</td><td>{winner}</td></tr>')


def generate_report(ds, dl, ms, ml, html_path):
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    # Comparison table — grouped sections
    def _section(title):
        return f'<tr><td colspan="5" style="background:var(--accent);color:#fff;font-weight:700;padding:8px 12px">{title}</td></tr>'

    rows = ""
    # ── General ──
    rows += _section("General")
    rows += _compare_row("Flight Time", ms["flight_time"], ml["flight_time"], "s")
    rows += _compare_row("Steps", ms["n_steps"], ml["n_steps"], "", ".0f")
    rows += _compare_row("dt", ms["dt"], ml["dt"], "s", ".4f")

    # ── Range & Impact ──
    rows += _section("Range & Impact")
    rows += _compare_row("Ground Range", ms["range"], ml["range"], "m", ".0f")
    rows += _compare_row("|Range Error|", abs(ms["range_err"]), abs(ml["range_err"]), "m", ".0f", lower_better=True)
    rows += _compare_row("Range Error %", abs(ms["range_err_pct"]), abs(ml["range_err_pct"]), "%", lower_better=True)
    rows += _compare_row("Final Speed", ms["final_speed"], ml["final_speed"], " m/s", ".0f")

    # ── Trajectory ──
    rows += _section("Trajectory")
    rows += _compare_row("Peak Altitude AGL", ms["peak_alt"], ml["peak_alt"], "m", ".0f")
    rows += _compare_row("Peak Alt Time", ms["peak_alt_time"], ml["peak_alt_time"], "s")
    rows += _compare_row("Max Speed", ms["max_speed"], ml["max_speed"], " m/s", ".0f")
    rows += _compare_row("Max Mach", ms["max_mach"], ml["max_mach"], "", ".3f")
    rows += _compare_row("Max q (dynamic pressure)", ms["max_q_Pa"], ml["max_q_Pa"], " Pa", ".0f")

    # ── Aerodynamics ──
    rows += _section("Aerodynamics")
    rows += _compare_row("Max |α|", ms["max_alpha"], ml["max_alpha"], "°", lower_better=True)
    rows += _compare_row("Mean |α|", ms["mean_alpha"], ml["mean_alpha"], "°", ".2f", lower_better=True)
    rows += _compare_row("Max |β|", ms["max_beta"], ml["max_beta"], "°", ".2f", lower_better=True)
    rows += _compare_row("Mean |β|", ms["mean_beta"], ml["mean_beta"], "°", ".3f", lower_better=True)

    # ── Attitude ──
    rows += _section("Attitude")
    rows += _compare_row("Max Pitch", ms["max_pitch"], ml["max_pitch"], "°")
    rows += _compare_row("Min Pitch", ms["min_pitch"], ml["min_pitch"], "°")
    rows += _compare_row("Max |Roll|", ms["max_roll"], ml["max_roll"], "°", ".2f", lower_better=True)
    rows += _compare_row("Max |Yaw|", ms["max_yaw"], ml["max_yaw"], "°", ".2f")

    # ── Angular Rates ──
    rows += _section("Angular Rates")
    rows += _compare_row("Max |p| (roll rate)", ms["max_omega_x"], ml["max_omega_x"], "°/s", ".2f", lower_better=True)
    rows += _compare_row("Max |q| (pitch rate)", ms["max_omega_y"], ml["max_omega_y"], "°/s", ".1f", lower_better=True)
    rows += _compare_row("Max |r| (yaw rate)", ms["max_omega_z"], ml["max_omega_z"], "°/s", ".2f", lower_better=True)
    rows += _compare_row("Max ω_total", ms["omega_total_max"], ml["omega_total_max"], "°/s", ".1f", lower_better=True)

    # ── Stability ──
    rows += _section("Stability")
    rows += _compare_row("Pitch σ (last 30%)", ms["pitch_std"], ml["pitch_std"], "°", ".2f", lower_better=True)
    rows += _compare_row("α σ (last 30%)", ms["alpha_std"], ml["alpha_std"], "°", ".2f", lower_better=True)

    # ── Loads ──
    rows += _section("Loads")
    rows += _compare_row("Max G (total)", ms["max_g"], ml["max_g"], "g")
    rows += _compare_row("Max G (axial)", ms["max_g_axial"], ml["max_g_axial"], "g")
    rows += _compare_row("Max |F_x|", ms["max_force_x"], ml["max_force_x"], " N", ".0f")
    rows += _compare_row("Max |F_z|", ms["max_force_z"], ml["max_force_z"], " N", ".0f")
    rows += _compare_row("Max Thrust", ms["max_thrust"], ml["max_thrust"], " N", ".0f")

    # ── Control ──
    rows += _section("Control (Fins)")
    rows += _compare_row("Max Fin Pitch (virtual)", ms["max_fin_pitch"], ml["max_fin_pitch"], "°")
    rows += _compare_row("Max Fin Yaw (virtual)", ms["max_fin_yaw"], ml["max_fin_yaw"], "°", ".2f")
    rows += _compare_row("Max Fin Roll (virtual)", ms["max_fin_roll"], ml["max_fin_roll"], "°", ".3f")
    rows += _compare_row("Mean Fin Pitch", ms["mean_fin_pitch"], ml["mean_fin_pitch"], "°", ".3f")
    for i in range(1, 5):
        k = f"max_fin{i}"
        if k in ms and k in ml:
            rows += _compare_row(f"Max |Fin {i}|", ms[k], ml[k], "°")

    # ── Mass & Propulsion ──
    rows += _section("Mass & Propulsion")
    rows += _compare_row("Initial Mass", ms["mass_i"], ml["mass_i"], " kg", ".3f")
    rows += _compare_row("Final Mass", ms["mass_f"], ml["mass_f"], " kg", ".3f")
    rows += _compare_row("Propellant Burned", ms["propellant"], ml["propellant"], " kg", ".3f")

    # ── Energy ──
    rows += _section("Energy")
    rows += _compare_row("Max KE", ms["max_KE"], ml["max_KE"], " kJ", ".1f")
    rows += _compare_row("Max PE", ms["max_PE"], ml["max_PE"], " kJ", ".1f")
    rows += _compare_row("Max Total Energy", ms["max_TE"], ml["max_TE"], " kJ", ".1f")

    table_html = (
        '<div class="card"><table>'
        '<tr><th>Metric</th>'
        '<th><span class="tag tag-s">Standalone</span></th>'
        '<th><span class="tag tag-l">SITL</span></th>'
        '<th>Δ (SITL−Standalone)</th><th>Winner</th></tr>'
        f'{rows}</table></div>'
    )

    # Figures
    f_traj = fig_trajectory(ds, dl)
    f_att = fig_attitude(ds, dl)
    f_aero = fig_aero(ds, dl)
    f_diff = fig_diff(ds, dl)
    f_ctrl = fig_control_compare(ds, dl)
    f_forces = fig_forces_energy(ds, dl)
    f_rates = fig_rates_gload(ds, dl)
    f_qpres = fig_dynamic_pressure(ds, dl)

    tabs = [("Comparison Table", "tab-table"), ("Trajectory", "tab-traj"),
            ("Attitude", "tab-att"), ("Angular Rates", "tab-rates"),
            ("Aero & Mass", "tab-aero"), ("Forces & Energy", "tab-forces"),
            ("Control & Fins", "tab-ctrl"), ("Dynamic Pressure", "tab-qpres"),
            ("Differences Δ", "tab-diff")]

    tabs_header = '<div class="tabs">'
    for i, (lbl, tid) in enumerate(tabs):
        a = " active" if i == 0 else ""
        tabs_header += f'<button class="tab-btn{a}" onclick="openTab(event,\'{tid}\')">{lbl}</button>'
    tabs_header += '</div>'

    tab_table = f'<div id="tab-table" class="tab-panel active"><h2>Full Metrics Comparison</h2>{table_html}</div>'
    tab_traj = f'<div id="tab-traj" class="tab-panel"><h2>Trajectory Overlay</h2>{_plotly_div(f_traj)}</div>'
    tab_att = f'<div id="tab-att" class="tab-panel"><h2>Attitude Overlay</h2>{_plotly_div(f_att)}</div>'
    tab_rates = f'<div id="tab-rates" class="tab-panel"><h2>Angular Rates & G-Load</h2>{_plotly_div(f_rates)}</div>'
    tab_aero = f'<div id="tab-aero" class="tab-panel"><h2>Aero & Mass</h2>{_plotly_div(f_aero)}</div>'
    tab_forces = f'<div id="tab-forces" class="tab-panel"><h2>Forces & Energy</h2>{_plotly_div(f_forces)}</div>'
    tab_ctrl = f'<div id="tab-ctrl" class="tab-panel"><h2>Control & Individual Fins</h2>{_plotly_div(f_ctrl)}</div>'
    tab_qpres = f'<div id="tab-qpres" class="tab-panel"><h2>Dynamic Pressure & Mach-α</h2>{_plotly_div(f_qpres)}</div>'
    tab_diff = f'<div id="tab-diff" class="tab-panel"><h2>Time-Series Differences (SITL − Standalone)</h2>{_plotly_div(f_diff)}</div>'

    toolbar = ('<div class="toolbar">'
               '<button id="theme-toggle" class="theme-toggle">&#9790; Dark</button>'
               f'<span style="font-size:.8rem;color:var(--text-secondary)">Target: {TARGET_RANGE_M:.0f}m</span>'
               '</div>')

    html = f"""<!DOCTYPE html>
<html lang="en"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>M130 SITL vs Standalone Comparison</title>
{plotly_cdn}<style>{_CSS}</style></head>
<body><div class="container">
<h1>M130 — SITL vs Standalone Comparison</h1>
{toolbar}
<div style="color:var(--text-secondary);font-size:.85rem;margin-bottom:16px">
  <span class="tag tag-s">Standalone</span> {html_escape(ms['label'])} &nbsp;|&nbsp;
  <span class="tag tag-l">SITL</span> {html_escape(ml['label'])} &nbsp;|&nbsp; Generated: {now}
</div>
<div class="tab-container">{tabs_header}
{tab_table}{tab_traj}{tab_att}{tab_rates}{tab_aero}{tab_forces}{tab_ctrl}{tab_qpres}{tab_diff}
</div></div>
<script>{_JS}</script></body></html>"""

    Path(html_path).write_text(html, encoding="utf-8")
    return html_path


# ══════════════════════════════════════════════════════════════════════
#  CLI
# ══════════════════════════════════════════════════════════════════════

def _find_latest(directory, pattern):
    files = sorted(directory.glob(pattern), key=lambda p: p.stat().st_mtime, reverse=True)
    return files[0] if files else None


def main():
    parser = argparse.ArgumentParser(description="M130 SITL vs Standalone comparison report")
    parser.add_argument("--sitl", type=str, default=None, help="SITL CSV path")
    parser.add_argument("--standalone", type=str, default=None, help="Standalone CSV path")
    parser.add_argument("--no-open", action="store_true", help="Don't open browser")
    args = parser.parse_args()

    sitl_csv = Path(args.sitl) if args.sitl else _find_latest(_RESULTS_DIR, "sitl_*.csv")
    standalone_csv = Path(args.standalone) if args.standalone else _find_latest(_STANDALONE_DIR, "*_log.csv")

    if not sitl_csv or not sitl_csv.exists():
        print(f"ERROR: No SITL CSV found in {_RESULTS_DIR}")
        sys.exit(1)
    if not standalone_csv or not standalone_csv.exists():
        print(f"ERROR: No standalone CSV found in {_STANDALONE_DIR}")
        sys.exit(1)

    print(f"  Standalone: {standalone_csv.name}")
    print(f"  SITL:       {sitl_csv.name}")
    print(f"  Target:     {TARGET_RANGE_M:.0f}m")

    ds = _load_standalone(standalone_csv)
    dl = _load_sitl(sitl_csv)
    ms = _metrics(ds, standalone_csv.name)
    ml = _metrics(dl, sitl_csv.name)

    out_dir = _RESULTS_DIR / "plots"
    out_dir.mkdir(parents=True, exist_ok=True)
    html_path = out_dir / "sitl_vs_standalone.html"

    generate_report(ds, dl, ms, ml, str(html_path))

    # Console summary
    print()
    print("═" * 60)
    print("  SITL vs Standalone Comparison")
    print("═" * 60)
    print(f"  {'Metric':<25} {'Standalone':>12} {'SITL':>12} {'Δ':>10}")
    print(f"  {'─'*25} {'─'*12} {'─'*12} {'─'*10}")
    rows_data = [
        ("Range", "range", "m", ".0f"), ("Range Error", "range_err", "m", "+.0f"),
        ("Peak Alt", "peak_alt", "m", ".0f"), ("Max Mach", "max_mach", "", ".3f"),
        ("Max α", "max_alpha", "°", ".1f"), ("Max β", "max_beta", "°", ".2f"),
        ("Pitch σ", "pitch_std", "°", ".2f"), ("Max Fin", "max_fin_pitch", "°", ".1f"),
    ]
    for lbl, key, u, fmt in rows_data:
        vs, vl = ms[key], ml[key]
        s1 = format(vs, fmt)
        s2 = format(vl, fmt)
        sd = format(vl - vs, "+" + fmt.lstrip("+"))
        print(f"  {lbl:<25} {s1:>10}{u:>2} {s2:>10}{u:>2} {sd:>8}{u}")
    print("═" * 60)
    print(f"  HTML → {html_path}")

    if not args.no_open:
        webbrowser.open(f"file://{html_path.resolve()}")


if __name__ == "__main__":
    main()
