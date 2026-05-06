#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════
  M130 6-DOF Simulation — Interactive HTML Analysis Engine
  محرك تحليل تفاعلي لنتائج محاكاة صاروخ M130 بستّ درجات حرية
═══════════════════════════════════════════════════════════════════════════════

Generates a professional self-contained HTML report with:
  - Smart scoring system (PASS/WARN/FAIL)
  - Auto-diagnostics & anomaly detection
  - Phase-aware analysis
  - Interactive Plotly charts (zoom, hover, pan)
  - Smart recommendations
  - Multi-run comparison & statistics

Usage:
    python results/advanced_analysis.py                    # latest run
    python results/advanced_analysis.py --all              # all runs
    python results/advanced_analysis.py --latest 10        # newest 10
    python results/advanced_analysis.py --file <csv_path>  # specific file
    python results/advanced_analysis.py --no-open          # don't open browser
"""

import sys, os, math, argparse, webbrowser, json
from pathlib import Path
from datetime import datetime
from collections import OrderedDict
from html import escape as html_escape

import numpy as np
import pandas as pd
import yaml
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import plotly.io as pio

# ─── Mission Configuration (loaded from YAML config) ─────────────────────────
_CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "6dof_config_advanced.yaml"
try:
    with open(_CONFIG_PATH, "r") as _f:
        _cfg = yaml.safe_load(_f)
    TARGET_RANGE_M   = float(_cfg["target"]["range_m"])
    TARGET_ALT_M     = float(_cfg["target"]["altitude"])
    LAUNCH_ALT_M     = float(_cfg["launch"]["altitude"])
except Exception:
    TARGET_RANGE_M   = 2900.0
    TARGET_ALT_M     = 1200.0
    LAUNCH_ALT_M     = 1200.0

# Scoring weights (configurable from YAML, must sum to 100)
_sw = _cfg.get("scoring_weights", {}) if '_cfg' in dir() else {}
SCORE_WEIGHTS = {
    "range":        _sw.get("range", 30),
    "impact_angle": _sw.get("impact_angle", 15),
    "stability":    _sw.get("stability", 20),
    "aoa":          _sw.get("aoa", 15),
    "sideslip":     _sw.get("sideslip", 10),
    "g_load":       _sw.get("g_load", 10),
}
CRUISE_GAMMA_DEG = -1.0

# Performance thresholds
THRESH = {
    "range_error_good":     50,    # m
    "range_error_warn":     150,   # m
    "impact_gamma_min":     -80,   # deg (too steep)
    "impact_gamma_max":     -20,   # deg (too shallow)
    "max_alpha_warn":       15,    # deg
    "max_alpha_fail":       25,    # deg
    "max_beta_warn":        9,     # deg (4 m/s crosswind on slow rocket yields β≈8°; was 5°)
    "max_beta_fail":        15,    # deg (raised proportionally from 10°)
    "pitch_std_warn":       3,     # deg
    "pitch_std_fail":       8,     # deg
    "max_g_warn":           15,    # g
    "max_g_fail":           25,    # g
    "omega_warn":           50,    # deg/s
    "omega_fail":           100,   # deg/s
}

PHASE_COLORS = OrderedDict([
    # القديمة (للتوافق)
    ("ARMED",     "#9e9e9e"), ("LAUNCH",   "#ff9800"), ("BOOST",    "#f44336"),
    ("COAST",     "#2196f3"), ("CRUISE",   "#4caf50"), ("TERMINAL", "#9c27b0"),
    ("BALLISTIC", "#795548"),
    # الجديدة (FlightPhase enum) — مُطابِقة لما يَكتبه السيم في CSV
    ("PREFLIGHT",         "#607d8b"),
    ("POWERED_ASCENT",    "#f44336"),  # = BOOST
    ("BURNOUT",           "#e91e63"),
    ("COAST_ASCENT",      "#2196f3"),  # = COAST
    ("NEAR_SPACE_ASCENT", "#3f51b5"),
    ("EXOATM_COAST",      "#673ab7"),
    ("APOGEE",            "#ffeb3b"),
    ("DESCENT",           "#9c27b0"),
    ("RECOVERY",          "#4caf50"),
])

# خَريطة phase aliases: الاسم الجديد → الاسم القديم المُكافئ.
# تَستَخدمها _score_phases وتَحقّقات impact_phase للتوافق.
PHASE_ALIASES = {
    "POWERED_ASCENT": "BOOST",
    "BURNOUT":        "BOOST",
    "COAST_ASCENT":   "COAST",
    "APOGEE":         "COAST",
    "DESCENT":        "TERMINAL",
}

# ─── Data Loading ────────────────────────────────────────────────────────────

def _flight_path_angle(vx, vy, vz, speed):
    """
    γ = arctan(-v_down / v_horizontal) — يَتَطلّب NED frame.
    في long_range mode، velocity_x/y/z هي ECEF (خاطئة لـ γ).
    استخدم vel_ned_* عند المُتَوَفر — هذه الدالة تَبقى للوضع short-range.
    """
    vh = np.sqrt(vx**2 + vy**2)
    return np.where(speed > 5.0, np.degrees(np.arctan2(-vz, vh)), 0.0)

def _load_csv(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    # Fix safety_violations: may contain string '[]' instead of numeric
    if "safety_violations" in df.columns:
        df["safety_violations"] = pd.to_numeric(df["safety_violations"].astype(str).str.strip("[] "), errors="coerce").fillna(0)
    df["alt_agl_m"]           = df["altitude_m"] - LAUNCH_ALT_M
    # γ يَجب أن يُحسَب من NED (vertical = -down, horizontal = sqrt(N²+E²)).
    # في long_range mode، velocity_x/y/z هي ECEF — استخدام vel_ned_* الصحيح.
    if "vel_ned_north_m_s" in df.columns and "vel_ned_down_m_s" in df.columns:
        df["gamma_deg"] = _flight_path_angle(
            df["vel_ned_north_m_s"], df["vel_ned_east_m_s"],
            df["vel_ned_down_m_s"], df["velocity_total_m_s"]
        )
        df["speed_horizontal_m_s"] = np.sqrt(df["vel_ned_north_m_s"]**2 + df["vel_ned_east_m_s"]**2)
    else:
        df["gamma_deg"] = _flight_path_angle(
            df["velocity_x_m_s"], df["velocity_y_m_s"],
            df["velocity_z_m_s"], df["velocity_total_m_s"]
        )
        df["speed_horizontal_m_s"] = np.sqrt(df["velocity_x_m_s"]**2 + df["velocity_y_m_s"]**2)
    df["range_error_m"]       = df["ground_range_m"] - TARGET_RANGE_M
    df["omega_total_deg_s"]   = np.sqrt(df["omega_x_deg_s"]**2 + df["omega_y_deg_s"]**2 + df["omega_z_deg_s"]**2)
    df["g_total"]             = np.sqrt(df["acceleration_body_x_g"]**2 + df["acceleration_body_y_g"]**2 + df["acceleration_body_z_g"]**2)
    df["thrust_total_N"]      = np.sqrt(df["thrust_x_N"]**2 + df["thrust_y_N"]**2 + df["thrust_z_N"]**2)
    df["moment_total_Nm"]     = np.sqrt(df["moment_x_Nm"]**2 + df["moment_y_Nm"]**2 + df["moment_z_Nm"]**2)
    df["KE_kJ"] = 0.5 * df["mass_kg"] * df["velocity_total_m_s"]**2 / 1000
    df["PE_kJ"] = df["mass_kg"] * 9.81 * df["alt_agl_m"] / 1000
    df["total_energy_kJ"] = df["KE_kJ"] + df["PE_kJ"]
    # Derived columns: vertical speed = -vel_down في NED (لا velocity_z في ECEF!).
    if "vel_ned_down_m_s" in df.columns:
        df["speed_vertical_m_s"] = -df["vel_ned_down_m_s"]  # +ve = climbing
    else:
        df["speed_vertical_m_s"] = -df["velocity_z_m_s"]  # positive = climbing
    df["thrust_lateral_N"] = np.sqrt(df["thrust_y_N"]**2 + df["thrust_z_N"]**2)
    df["force_normal_N"] = df["force_z_N"]
    df["force_lateral_N"] = df["force_y_N"]
    df["CY_total"] = df.get("CY_total", df.get("CY_base", 0)) 
    df["Cn_total"] = df.get("Cn_total", df.get("Cn_base", 0))
    # Static margin from aero-only coefficients (total minus control)
    _CN_aero = df["CN_total"] - df["CN_control"]
    _CM_aero = df["CM_total"] - df["CM_control"]
    with np.errstate(divide='ignore', invalid='ignore'):
        _sm_raw = np.where(np.abs(_CN_aero) > 0.05, -_CM_aero / _CN_aero, np.nan)
    df["static_margin_cal"] = np.clip(_sm_raw, -2.0, 2.0)
    # MPC tracking error (if MPC columns available)
    # χ (chi) = atan2(ve, vn) NED heading للسرعة — ليس yaw الجسم!
    # مع sideslip أو رياح، yaw_deg ≠ chi (يَفترقان بـ β + wind drift).
    if "vel_ned_north_m_s" in df.columns and "vel_ned_east_m_s" in df.columns:
        df["chi_deg"] = np.degrees(np.arctan2(df["vel_ned_east_m_s"], df["vel_ned_north_m_s"]))
    else:
        df["chi_deg"] = df["yaw_deg"]  # fallback (إذا لا NED، يَستوي)
    if "mpc_gamma_ref_deg" in df.columns:
        df["tracking_error_gamma_deg"] = df["gamma_deg"] - df["mpc_gamma_ref_deg"]
    if "mpc_chi_ref_deg" in df.columns:
        # حلّ wrap-around: error بين -180 و +180
        err_chi = df["chi_deg"] - df["mpc_chi_ref_deg"]
        df["tracking_error_chi_deg"] = ((err_chi + 180.0) % 360.0) - 180.0
    # Fin deflections (if available)
    if "delta_pitch_rad" in df.columns:
        df["delta_pitch_deg"] = np.degrees(df["delta_pitch_rad"])
        df["delta_yaw_deg"] = np.degrees(df["delta_yaw_rad"])
        df["delta_roll_deg"] = np.degrees(df["delta_roll_rad"])
    # Energy dissipation rate (dE/dt)
    dt = np.gradient(df["time_s"].values)
    dt = np.where(dt > 0, dt, 1e-6)
    df["dE_dt_kW"] = np.gradient(df["total_energy_kJ"].values) / dt  # kJ/s = kW
    # Actuator commanded → degrees (if available)
    for j in range(1, 5):
        col = f"actuator_cmd_fin{j}_rad"
        if col in df.columns:
            df[f"actuator_cmd_fin{j}_deg"] = np.degrees(df[col])
    # Actuator lag = commanded - actual (rad)
    if "actuator_cmd_fin1_rad" in df.columns and "fin_1_rad" in df.columns:
        for j in range(1, 5):
            df[f"actuator_lag_fin{j}_deg"] = np.degrees(df[f"actuator_cmd_fin{j}_rad"] - df[f"fin_{j}_rad"])
    # Angular acceleration degrees
    if "angular_accel_x_rad_s2" in df.columns:
        df["angular_accel_x_deg_s2"] = np.degrees(df["angular_accel_x_rad_s2"])
        df["angular_accel_y_deg_s2"] = np.degrees(df["angular_accel_y_rad_s2"])
        df["angular_accel_z_deg_s2"] = np.degrees(df["angular_accel_z_rad_s2"])
    # MPC virtual commands → degrees
    if "mpc_delta_e_rad" in df.columns:
        df["mpc_delta_e_deg"] = np.degrees(df["mpc_delta_e_rad"])
        df["mpc_delta_r_deg"] = np.degrees(df["mpc_delta_r_rad"])
        df["mpc_delta_a_deg"] = np.degrees(df["mpc_delta_a_rad"])
    # Detect conditional data presence flags
    df.attrs["has_mhe"] = "mhe_quality" in df.columns and df["mhe_quality"].abs().max() > 0
    df.attrs["has_sensor"] = "accel_meas_x" in df.columns
    df.attrs["has_lla"] = "latitude_deg" in df.columns
    df.attrs["has_actuator_cmd"] = "actuator_cmd_fin1_rad" in df.columns
    df.attrs["has_fins"]         = "fin_1_rad" in df.columns
    df.attrs["has_mpc_diag"] = ("mpc_solve_time_ms" in df.columns and
                                (df["mpc_solve_time_ms"].abs().max() > 0 or
                                 ("mpc_delta_e_rad" in df.columns and df["mpc_delta_e_rad"].abs().max() > 0.001)))
    df.attrs["has_angular_accel"] = "angular_accel_x_rad_s2" in df.columns
    # Detailed MHE state vector columns (exported by rocket_6dof_sim.py)
    df.attrs["has_mhe_states"] = "mhe_V_m_s" in df.columns and df["mhe_V_m_s"].abs().max() > 0
    if df.attrs["has_mhe_states"]:
        # Derived MHE quantities for analysis
        df["mhe_gamma_deg"] = np.degrees(df["mhe_gamma_rad"])
        df["mhe_chi_deg"]   = np.degrees(df["mhe_chi_rad"])
        df["mhe_alpha_deg"] = np.degrees(df["mhe_alpha_rad"])
        df["mhe_beta_deg"]  = np.degrees(df["mhe_beta_rad"])
        df["mhe_phi_deg"]   = np.degrees(df["mhe_phi_rad"])
        df["mhe_p_deg_s"]   = np.degrees(df["mhe_p_rad_s"])
        df["mhe_q_deg_s"]   = np.degrees(df["mhe_q_rad_s"])
        df["mhe_r_deg_s"]   = np.degrees(df["mhe_r_rad_s"])
        df["mhe_alt_m"]     = df["mhe_h_scaled"] * 100.0 + LAUNCH_ALT_M
        df["mhe_xg_m"]      = df["mhe_xg_scaled"] * 1000.0
        df["mhe_yg_m"]      = df["mhe_yg_scaled"] * 1000.0
        # Estimation errors vs truth (only where MHE is active: quality > 0)
        mhe_active = df["mhe_quality"] > 0
        df["mhe_alpha_error_deg"] = np.where(mhe_active, df["mhe_alpha_deg"] - np.degrees(df["alpha_rad"]), np.nan)
        df["mhe_beta_error_deg"]  = np.where(mhe_active, df["mhe_beta_deg"]  - np.degrees(df["beta_rad"]),  np.nan)
        df["mhe_V_error_m_s"]     = np.where(mhe_active, df["mhe_V_m_s"] - df["airspeed_m_s"], np.nan)
        df["mhe_gamma_error_deg"] = np.where(mhe_active, df["mhe_gamma_deg"] - df["gamma_deg"], np.nan)
    return df


def _extract_run_metrics(df: pd.DataFrame, filepath: Path) -> dict:
    m = {}
    t = df["time_s"].values
    m["file"]       = filepath.name
    m["timestamp"]  = filepath.name.split("_log.csv")[0].split("CFD68_")[-1] if "CFD68_" in filepath.name else filepath.stem
    m["flight_time_s"] = t[-1]
    m["n_steps"]    = len(df)

    last = df.iloc[-1]
    m["impact_range_m"]   = last["ground_range_m"]
    m["range_error_m"]    = last["ground_range_m"] - TARGET_RANGE_M
    m["range_error_pct"]  = (last["ground_range_m"] / TARGET_RANGE_M - 1) * 100
    m["impact_speed_mps"] = last["velocity_total_m_s"]
    m["impact_gamma_deg"] = last.get("gamma_deg", np.nan)
    m["impact_pitch_deg"] = last["pitch_deg"]
    m["impact_alt_agl_m"] = last.get("alt_agl_m", last["altitude_m"] - LAUNCH_ALT_M)
    m["impact_phase"]     = last["flight_phase"]
    m["impact_mass_kg"]   = last["mass_kg"]
    # Cross-range: من lat/lon وليس ECEF (position_*_m في ECEF, ليست NED!).
    # نحسب الإزاحة الشمالية والشرقية ثم نُسقطها على bearing.
    if "latitude_deg" in df.columns and "longitude_deg" in df.columns:
        # ملاحظة: العمود اسمه "_deg" لكن الوحدات قد تكون rad فعلياً
        lat0 = float(df["latitude_deg"].iloc[0])
        lat_end = float(last["latitude_deg"])
        lon0 = float(df["longitude_deg"].iloc[0])
        lon_end = float(last["longitude_deg"])
        # تلقائي: إذا |lat| < π/2 ≈ 1.57 → القيم في rad
        if abs(lat0) < 1.6:
            lat0 = np.degrees(lat0); lat_end = np.degrees(lat_end)
            lon0 = np.degrees(lon0); lon_end = np.degrees(lon_end)
        # تحويل إلى Δnorth و Δeast (متر)
        d_north = (lat_end - lat0) * 111320.0
        d_east = (lon_end - lon0) * 111320.0 * np.cos(np.radians(lat0))
        # bearing من config (default 0° = الشمال)
        try:
            bearing_rad = np.radians(float(_cfg["target"].get("bearing_deg", 0.0)))
        except Exception:
            bearing_rad = 0.0
        # downrange = إسقاط على اتجاه الـ bearing
        # cross-range = إسقاط على المحور العمودي على bearing (right-hand+)
        downrange = d_north * np.cos(bearing_rad) + d_east * np.sin(bearing_rad)
        crossrange = -d_north * np.sin(bearing_rad) + d_east * np.cos(bearing_rad)
        m["cross_range_error_m"] = float(crossrange)
        # تحديث range_error_m من lat/lon (أكثر دقة من ground_range_m المُحفوظ)
        m["range_error_m_geodetic"] = float(downrange - TARGET_RANGE_M)
    else:
        m["cross_range_error_m"] = 0.0

    alt_agl = df["alt_agl_m"].values
    idx_peak = np.argmax(alt_agl)
    m["peak_alt_agl_m"]  = alt_agl[idx_peak]
    m["peak_alt_time_s"] = t[idx_peak]

    spd = df["velocity_total_m_s"].values
    m["max_speed_mps"]    = np.max(spd)
    m["max_speed_time_s"] = t[np.argmax(spd)]

    mach = df["mach"].values
    m["max_mach"]         = np.max(mach)
    m["max_mach_time_s"]  = t[np.argmax(mach)]

    q_dyn = df["q_dynamic_Pa"].values
    idx_maxq = np.argmax(q_dyn)
    m["max_q_Pa"]         = q_dyn[idx_maxq]
    m["max_q_time_s"]     = t[idx_maxq]

    g = df["g_total"].values
    m["max_g"]            = np.max(g)
    m["max_g_time_s"]     = t[np.argmax(g)]

    # AoA/β في أول 1.5s (post-rail transient + wind buildup) عديمة المعنى:
    # القضيب يحرر الصاروخ بسرعة منخفضة، فالأمواج الأولى لا تعكس أداء MPC.
    # نأخذ الذروات بعد استقرار الديناميكا (vel>50m/s).
    inflight = df["velocity_total_m_s"].values > 50.0
    if inflight.any():
        m["max_alpha_deg"] = float(np.max(np.abs(df["alpha_deg"].values[inflight])))
        m["max_beta_deg"]  = float(np.max(np.abs(df["beta_deg"].values[inflight])))
    else:
        m["max_alpha_deg"] = float(np.max(np.abs(df["alpha_deg"].values)))
        m["max_beta_deg"]  = float(np.max(np.abs(df["beta_deg"].values)))
    m["max_pitch_rate_dps"] = np.max(np.abs(df["omega_y_deg_s"].values))
    m["max_yaw_rate_dps"]   = np.max(np.abs(df["omega_z_deg_s"].values))
    m["max_roll_rate_dps"]  = np.max(np.abs(df["omega_x_deg_s"].values))
    m["max_omega_deg_s"]    = np.max(df["omega_total_deg_s"].values)
    m["max_thrust_N"]       = np.max(df["thrust_total_N"].values)
    m["initial_mass_kg"]    = df["mass_kg"].iloc[0]
    m["mass_consumed_kg"]   = df["mass_kg"].iloc[0] - df["mass_kg"].iloc[-1]
    m["max_airspeed_mps"]   = np.max(df["airspeed_m_s"].values)
    m["max_CN_delta"]       = np.max(np.abs(df["CN_delta"].values))
    m["max_CM_delta"]       = np.max(np.abs(df["CM_delta"].values))
    m["max_CY_control"]     = np.max(np.abs(df["CY_control"].values))
    m["max_Cn_control"]     = np.max(np.abs(df["Cn_control"].values))
    m["max_force_y_N"]      = np.max(np.abs(df["force_y_N"].values))
    m["max_force_z_N"]      = np.max(np.abs(df["force_z_N"].values))
    sm = df["static_margin_cal"].dropna().values
    m["mean_static_margin"] = float(np.nanmean(sm)) if len(sm) > 0 else 0.0
    m["min_static_margin"]  = float(np.nanmin(sm)) if len(sm) > 0 else 0.0

    # Phase durations
    phase_col = df["flight_phase"]
    for ph in phase_col.unique():
        ph_times = t[phase_col == ph]
        if len(ph_times) > 0:
            m[f"phase_{ph}_dur_s"] = ph_times[-1] - ph_times[0]

    # Stability
    n30 = max(1, int(0.3 * len(df)))
    m["pitch_std_last30pct"]   = np.std(df["pitch_deg"].values[-n30:])
    m["pitch_range_last30pct"] = np.ptp(df["pitch_deg"].values[-n30:])
    m["yaw_drift_deg"]         = df["yaw_deg"].values[-1] - df["yaw_deg"].values[-n30]

    # MPC tracking RMSE in last 30% (best stability metric when MPC reference
    # is available — measures how well the rocket follows the PLAN rather
    # than how constant its pitch is. A proper dive maneuver has large
    # σ_pitch but near-zero tracking error).
    if "tracking_error_gamma_deg" in df.columns:
        tail_err = df["tracking_error_gamma_deg"].values[-n30:]
        # Filter NaN/Inf from pre-MPC phase (MPC disabled before launch)
        tail_err = tail_err[np.isfinite(tail_err)]
        if len(tail_err) > 0:
            m["tracking_rmse_gamma_last30pct"] = float(
                np.sqrt(np.mean(tail_err ** 2)))
    if "tracking_error_chi_deg" in df.columns:
        tail_err = df["tracking_error_chi_deg"].values[-n30:]
        tail_err = tail_err[np.isfinite(tail_err)]
        if len(tail_err) > 0:
            m["tracking_rmse_chi_last30pct"] = float(
                np.sqrt(np.mean(tail_err ** 2)))

    # Energy
    m["max_KE_kJ"] = np.max(df["KE_kJ"].values)
    m["impact_KE_kJ"] = df["KE_kJ"].iloc[-1]

    # MPC solver diagnostics
    if df.attrs.get("has_mpc_diag", False):
        mpc_active = df["mpc_solve_time_ms"] > 0
        if mpc_active.any():
            mpc_sub = df[mpc_active]
            m["mpc_mean_solve_ms"] = float(mpc_sub["mpc_solve_time_ms"].mean())
            m["mpc_max_solve_ms"] = float(mpc_sub["mpc_solve_time_ms"].max())
            m["mpc_failures"] = int((mpc_sub["mpc_solver_status"] > 1).sum())
            m["mpc_mean_sqp_iters"] = float(mpc_sub["mpc_sqp_iterations"].mean())

    # Actuator analysis
    if df.attrs.get("has_actuator_cmd", False):
        max_lag = 0
        for j in range(1, 5):
            col = f"actuator_lag_fin{j}_deg"
            if col in df.columns:
                max_lag = max(max_lag, df[col].abs().max())
        m["max_actuator_lag_deg"] = float(max_lag)

    # Fin authority
    if "fin_authority" in df.columns:
        m["min_fin_authority"] = float(df["fin_authority"].min())
        m["mean_fin_authority"] = float(df["fin_authority"].mean())

    # Safety violations
    if "safety_violations" in df.columns:
        sv = df["safety_violations"].astype(float)
        m["total_safety_violations"] = int(sv.sum())

    return m


# ─── Smart Scoring & Diagnostics ────────────────────────────────────────────

def _score_run(m: dict) -> dict:
    scores = {}
    W = SCORE_WEIGHTS

    # Range accuracy — يجب أن يَستخدم خطأ الضربة الكلّي (downrange + cross).
    # السابق: range_error_m فقط → يُخفي cross-range miss (مثلاً wind drift).
    w = W["range"]
    dr_err = m["range_error_m"]
    cr_err = m.get("cross_range_error_m", 0.0)
    miss_total = float(np.sqrt(dr_err**2 + cr_err**2))  # total impact miss
    if miss_total < THRESH["range_error_good"]:
        s, v = w, "PASS"
    elif miss_total < THRESH["range_error_warn"]:
        s = w * (1 - (miss_total - THRESH["range_error_good"]) /
                  (THRESH["range_error_warn"] - THRESH["range_error_good"]))
        v = "WARN"
    else:
        s = max(0, (w * 0.25) * (1 - miss_total / 1000))
        v = "FAIL"
    scores["range"] = {"score": round(s, 1), "verdict": v,
                       "detail": f"miss = {miss_total:.0f}m  (DR {dr_err:+.0f}m, CR {cr_err:+.0f}m)"}

    # Trajectory classification — flat missions (low peak alt) have rapid
    # pitch transitions that are PHYSICALLY CORRECT but produce large σ_pitch
    # and shallow impact γ. Steep missions (high apogee) expect steady tails.
    peak_alt = m.get("peak_alt_agl_m", m.get("peak_alt_m", 9999.0))
    is_flat = peak_alt < 500.0
    traj_type = "flat" if is_flat else "steep"

    # Impact angle — adaptive
    w = W["impact_angle"]
    gamma = m["impact_gamma_deg"]
    if is_flat:
        # Flat trajectory: accept shallow γ down to -3°
        if gamma < -3:
            s, v = w, "PASS"
        elif gamma < 0:
            s, v = round(w * 0.8, 1), "PASS"
        else:
            s, v = round(w * 0.4, 1), "WARN"
    else:
        # Steep trajectory: expect steep dive
        if THRESH["impact_gamma_min"] < gamma < THRESH["impact_gamma_max"]:
            s, v = w, "PASS"
        elif -90 < gamma < -10:
            s, v = round(w * 0.53, 1), "WARN"
        else:
            s, v = 0, "FAIL"
    scores["impact_angle"] = {"score": round(s, 1), "verdict": v,
                              "detail": f"γ = {gamma:.1f}° ({traj_type} trajectory)"}

    # Stability — trajectory-aware scoring:
    #   Flat trajectory: σ_pitch with RELAXED thresholds (8°/15°) because
    #     the rocket must ballistically dive to hit the target — rapid pitch
    #     transition is physically correct, not a control failure.
    #     NB: tracking_rmse vs LOS γ_ref is meaningless here (rocket MUST
    #     diverge from LOS to account for gravity+drag).
    #   Steep trajectory: prefer tracking_rmse (rocket should follow LOS).
    #     Fallback to σ_pitch with tight thresholds (3°/8°).
    w = W["stability"]
    pstd = m["pitch_std_last30pct"]
    trk_rmse = m.get("tracking_rmse_gamma_last30pct")

    if is_flat:
        # Flat trajectory: σ_pitch with relaxed thresholds
        warn_th, fail_th = 8.0, 15.0
        if pstd < warn_th:
            s, v = w, "PASS"
        elif pstd < fail_th:
            s = w * (1 - (pstd - warn_th) / (fail_th - warn_th))
            v = "WARN"
        else:
            s, v = 0, "FAIL"
        detail = f"pitch σ = {pstd:.2f}° (flat traj.; warn≥{warn_th:.0f}°)"
    elif trk_rmse is not None and trk_rmse > 0:
        # Steep trajectory with MPC data: tracking RMSE is meaningful
        if trk_rmse < 3.0:
            s, v = w, "PASS"
        elif trk_rmse < 8.0:
            s = w * (1 - (trk_rmse - 3.0) / 5.0)
            v = "WARN"
        else:
            s, v = 0, "FAIL"
        detail = f"γ tracking RMSE = {trk_rmse:.2f}° (steep traj., last 30%)"
    else:
        # Steep trajectory, no MPC reference: tight σ_pitch thresholds
        if pstd < THRESH["pitch_std_warn"]:
            s, v = w, "PASS"
        elif pstd < THRESH["pitch_std_fail"]:
            s = w * (1 - (pstd - THRESH["pitch_std_warn"]) / (THRESH["pitch_std_fail"] - THRESH["pitch_std_warn"]))
            v = "WARN"
        else:
            s, v = 0, "FAIL"
        detail = f"pitch σ = {pstd:.2f}° (steep traj.)"
    scores["stability"] = {"score": round(s, 1), "verdict": v, "detail": detail}

    # AoA
    w = W["aoa"]
    alpha = m["max_alpha_deg"]
    if alpha < THRESH["max_alpha_warn"]:
        s, v = w, "PASS"
    elif alpha < THRESH["max_alpha_fail"]:
        s, v = round(w * 0.5, 1), "WARN"
    else:
        s, v = 0, "FAIL"
    scores["aoa"] = {"score": round(s, 1), "verdict": v, "detail": f"max |α| = {alpha:.1f}°"}

    # Sideslip
    w = W["sideslip"]
    beta = m["max_beta_deg"]
    if beta < THRESH["max_beta_warn"]:
        s, v = w, "PASS"
    elif beta < THRESH["max_beta_fail"]:
        s, v = round(w * 0.5, 1), "WARN"
    else:
        s, v = 0, "FAIL"
    scores["sideslip"] = {"score": round(s, 1), "verdict": v, "detail": f"max |β| = {beta:.1f}°"}

    # G-load
    w = W["g_load"]
    g = m["max_g"]
    if g < THRESH["max_g_warn"]:
        s, v = w, "PASS"
    elif g < THRESH["max_g_fail"]:
        s, v = round(w * 0.5, 1), "WARN"
    else:
        s, v = 0, "FAIL"
    scores["g_load"] = {"score": round(s, 1), "verdict": v, "detail": f"max G = {g:.1f}"}

    total = sum(c["score"] for c in scores.values())
    overall = "PASS" if total >= 80 else ("WARN" if total >= 50 else "FAIL")
    scores["_total"] = round(total, 1)
    scores["_overall"] = overall
    return scores


def _diagnose(df: pd.DataFrame, m: dict) -> list:
    diags = []
    def _add(level, title, detail):
        diags.append({"level": level, "title": title, "detail": detail})

    abs_err = abs(m["range_error_m"])
    if abs_err > THRESH["range_error_warn"]:
        direction = "SHORT" if m["range_error_m"] < 0 else "LONG"
        _add("error", f"Range {direction} by {abs_err:.0f}m",
             f"Impact at {m['impact_range_m']:.0f}m vs target {TARGET_RANGE_M:.0f}m ({m['range_error_pct']:+.1f}%)")
    elif abs_err > THRESH["range_error_good"]:
        _add("warning", f"Range error {m['range_error_m']:+.0f}m",
             f"Within warning band ({THRESH['range_error_good']}-{THRESH['range_error_warn']}m)")

    if m["max_alpha_deg"] > THRESH["max_alpha_fail"]:
        _add("error", f"Extreme AoA: {m['max_alpha_deg']:.1f}°",
             "Risk of flow separation and loss of control. Check pitch program / CG location.")
    elif m["max_alpha_deg"] > THRESH["max_alpha_warn"]:
        _add("warning", f"High AoA: {m['max_alpha_deg']:.1f}°",
             "Approaching aerodynamic limits. Monitor CN linearity.")

    if m["max_beta_deg"] > THRESH["max_beta_fail"]:
        _add("error", f"Excessive sideslip: {m['max_beta_deg']:.1f}°",
             "Yaw channel instability or asymmetric disturbance.")
    elif m["max_beta_deg"] > THRESH["max_beta_warn"]:
        _add("warning", f"Sideslip β = {m['max_beta_deg']:.1f}°",
             "Check yaw damping and wind disturbance rejection.")

    # Pitch oscillation detection
    n40 = max(1, int(0.4 * len(df)))
    pitch_tail = df["omega_y_deg_s"].values[-n40:]
    zero_crossings = np.sum(np.diff(np.sign(pitch_tail)) != 0)
    flight_dur_tail = df["time_s"].values[-1] - df["time_s"].values[-n40]
    if flight_dur_tail > 0.5:
        osc_freq = zero_crossings / (2 * flight_dur_tail)
        if osc_freq > 3 and m["pitch_std_last30pct"] > THRESH["pitch_std_warn"]:
            _add("warning", f"Pitch oscillation detected: ~{osc_freq:.1f} Hz",
                 f"Sustained oscillation with σ={m['pitch_std_last30pct']:.2f}°. Possible limit cycle.")

    if m["max_omega_deg_s"] > THRESH["omega_fail"]:
        _add("error", f"Extreme angular rate: {m['max_omega_deg_s']:.0f} °/s",
             "Possible tumbling or control divergence.")
    elif m["max_omega_deg_s"] > THRESH["omega_warn"]:
        _add("warning", f"High angular rate: {m['max_omega_deg_s']:.0f} °/s", "Check damping gains.")

    if m["max_g"] > THRESH["max_g_fail"]:
        _add("error", f"G-load exceeded structural limit: {m['max_g']:.1f} g",
             "Reduce pitch maneuver aggressiveness.")
    elif m["max_g"] > THRESH["max_g_warn"]:
        _add("warning", f"High G-load: {m['max_g']:.1f} g", "Approaching structural limits.")

    gamma = m["impact_gamma_deg"]
    if gamma > -10:
        _add("error", f"Very shallow impact: γ = {gamma:.1f}°",
             "Rocket nearly horizontal at impact. Ineffective warhead function.")
    elif gamma > THRESH["impact_gamma_max"]:
        _add("warning", f"Shallow impact angle: γ = {gamma:.1f}°",
             "Consider steeper terminal dive for better penetration.")
    elif gamma < THRESH["impact_gamma_min"]:
        _add("warning", f"Very steep impact: γ = {gamma:.1f}°",
             "Nearly vertical — may reduce ground-range accuracy.")

    # boost duration: تَجمع POWERED_ASCENT + BOOST (الاسمين القديم والجديد)
    boost_dur = m.get("phase_POWERED_ASCENT_dur_s", 0) + m.get("phase_BOOST_dur_s", 0)
    if boost_dur < 0.5 and m["max_thrust_N"] > 100:
        _add("warning", f"Very short boost phase: {boost_dur:.2f}s",
             "Check propellant mass or burn rate.")

    # Static margin check
    sm = df["static_margin_cal"].dropna()
    if len(sm) > 0 and sm.min() < 0:
        _add("error", f"Statically unstable: SM = {sm.min():.2f} cal",
             "Negative static margin detected — CP ahead of CG.")
    elif len(sm) > 0 and sm.min() < 0.5:
        _add("warning", f"Low static margin: {sm.min():.2f} cal",
             "Marginal stability — sensitive to CG shifts.")

    # Lateral force check
    if m["max_force_y_N"] > 500:
        _add("warning", f"High lateral force: {m['max_force_y_N']:.0f} N",
             "Significant side force — check yaw trim and wind.")

    # Accept old + new phase names + aliased equivalents.
    _terminal_phases = ("TERMINAL", "BALLISTIC", "DESCENT", "RECOVERY")
    if m["impact_phase"] not in _terminal_phases:
        _add("warning", f"Impact during {m['impact_phase']} phase",
             "Simulation may have ended prematurely or ground collision during boost/cruise.")

    # MPC solver diagnostics
    if m.get("mpc_failures", 0) > 0:
        _add("warning", f"MPC solver failures: {m['mpc_failures']}",
             f"Mean solve: {m.get('mpc_mean_solve_ms', 0):.2f} ms, max: {m.get('mpc_max_solve_ms', 0):.2f} ms. "
             "Solver not converging — check MPC formulation or horizon.")
    if m.get("mpc_max_solve_ms", 0) > 10:
        _add("warning", f"Slow MPC solve: max {m['mpc_max_solve_ms']:.1f} ms",
             "May cause control delays. Consider reducing prediction horizon.")

    # Actuator lag
    if m.get("max_actuator_lag_deg", 0) > 5.0:
        _add("error", f"Severe actuator lag: {m['max_actuator_lag_deg']:.1f}°",
             "Actuator saturated — control authority severely degraded.")
    elif m.get("max_actuator_lag_deg", 0) > 2.0:
        _add("warning", f"Large actuator lag: {m['max_actuator_lag_deg']:.1f}°",
             "Commands not tracked — actuator rate limit or backlash may be too restrictive.")

    # Fin authority
    if m.get("min_fin_authority", 1.0) < 0.2:
        _add("error", f"Low fin authority: min = {m['min_fin_authority']:.2f}",
             "Fins nearly ineffective — check airspeed and dynamic pressure.")
    elif m.get("min_fin_authority", 1.0) < 0.5:
        _add("warning", f"Reduced fin authority: min = {m['min_fin_authority']:.2f}",
             "Fin effectiveness below 50% — maneuverability compromised.")

    # Safety violations
    if m.get("total_safety_violations", 0) > 0:
        _add("error", f"Safety violations: {m['total_safety_violations']}",
             "One or more safety limits exceeded during flight. Check structural loads and flight envelope.")

    if not diags:
        _add("info", "No anomalies detected", "All parameters within nominal bounds.")
    return diags


def _recommend(m: dict, scores: dict, diags: list) -> list:
    recs = []
    if m["range_error_m"] < -THRESH["range_error_warn"]:
        recs.append("Range short → Try lower cruise_gamma_deg for a flatter trajectory, or increase propellant.")
    if m["range_error_m"] > THRESH["range_error_warn"]:
        recs.append("Range long → Increase cruise_gamma_deg or start terminal dive earlier.")
    if m["max_alpha_deg"] > THRESH["max_alpha_warn"]:
        recs.append("High AoA → Reduce pitch rate command or check CG-CP margin.")
    if m["max_beta_deg"] > THRESH["max_beta_warn"]:
        recs.append("Sideslip → Increase yaw damping gain or check fin alignment.")
    if m["pitch_std_last30pct"] > THRESH["pitch_std_warn"]:
        recs.append("Pitch oscillation → Tune MPC weights or add rate feedback.")
    if m["max_g"] > THRESH["max_g_warn"]:
        recs.append("G-load → Reduce maneuver aggressiveness in pitch program.")
    if m["impact_gamma_deg"] > THRESH["impact_gamma_max"]:
        recs.append("Shallow impact → Start terminal dive earlier or steeper dive angle.")
    # Servo delay compensation check
    max_roll = m.get("max_roll_deg", 0)
    pitch_std = m.get("pitch_std_last30pct", 0)
    max_alpha = m.get("max_alpha_deg", 0)
    delay_issues = sum([max_roll > 30, pitch_std > 5, max_alpha > 15])
    if delay_issues >= 2:
        recs.append("⚠ SERVO DELAY: Under-compensated — increase delay_steps in "
                    "rocket_properties.yaml (lookahead_stage auto-computes). "
                    "For HIL/flight: set RKT_MPC_SVO_DLY = measured delay in seconds "
                    "(e.g. 0.080 for 80ms → lookahead_stage=4)")
    if not recs:
        recs.append("Performance nominal — no changes recommended.")
    return recs


# ─── Phase Analysis ──────────────────────────────────────────────────────────

def _analyze_phases(df: pd.DataFrame) -> list:
    phases = []
    phase_col = df["flight_phase"].values
    t = df["time_s"].values
    for ph in OrderedDict.fromkeys(phase_col):
        mask = phase_col == ph
        sub = df[mask]
        if len(sub) == 0:
            continue
        p = {
            "name": ph, "color": PHASE_COLORS.get(ph, "#333"),
            "t_start": t[mask][0], "t_end": t[mask][-1],
            "duration": t[mask][-1] - t[mask][0],
            "alt_start": sub["alt_agl_m"].iloc[0], "alt_end": sub["alt_agl_m"].iloc[-1],
            "speed_start": sub["velocity_total_m_s"].iloc[0], "speed_end": sub["velocity_total_m_s"].iloc[-1],
            "max_speed": sub["velocity_total_m_s"].max(),
            "range_start": sub["ground_range_m"].iloc[0], "range_end": sub["ground_range_m"].iloc[-1],
            "max_alpha": sub["alpha_deg"].abs().max(),
            "max_g": sub["g_total"].max(),
            "max_q": sub["q_dynamic_Pa"].max(),
            "pitch_start": sub["pitch_deg"].iloc[0], "pitch_end": sub["pitch_deg"].iloc[-1],
        }
        phases.append(p)
    return phases


def _score_phases(df: pd.DataFrame, phases: list) -> list:
    """Score each flight phase individually on relevant criteria."""
    scored = []
    for p in phases:
        ph = p["name"]
        mask = df["flight_phase"] == ph
        sub = df[mask]
        if len(sub) < 2:
            continue
        s = {"name": ph, "color": p["color"], "checks": []}

        alpha_max = sub["alpha_deg"].abs().max()
        beta_max = sub["beta_deg"].abs().max()
        g_max = sub["g_total"].max()
        omega_max = sub["omega_total_deg_s"].max()
        pitch_std = sub["pitch_deg"].std()

        # AoA check
        if alpha_max < THRESH["max_alpha_warn"]:
            s["checks"].append(("AoA", "PASS", f"|α| max = {alpha_max:.1f}°"))
        elif alpha_max < THRESH["max_alpha_fail"]:
            s["checks"].append(("AoA", "WARN", f"|α| max = {alpha_max:.1f}°"))
        else:
            s["checks"].append(("AoA", "FAIL", f"|α| max = {alpha_max:.1f}°"))

        # Sideslip
        if beta_max < THRESH["max_beta_warn"]:
            s["checks"].append(("Sideslip", "PASS", f"|β| max = {beta_max:.1f}°"))
        elif beta_max < THRESH["max_beta_fail"]:
            s["checks"].append(("Sideslip", "WARN", f"|β| max = {beta_max:.1f}°"))
        else:
            s["checks"].append(("Sideslip", "FAIL", f"|β| max = {beta_max:.1f}°"))

        # G-load
        if g_max < THRESH["max_g_warn"]:
            s["checks"].append(("G-Load", "PASS", f"max = {g_max:.1f} g"))
        elif g_max < THRESH["max_g_fail"]:
            s["checks"].append(("G-Load", "WARN", f"max = {g_max:.1f} g"))
        else:
            s["checks"].append(("G-Load", "FAIL", f"max = {g_max:.1f} g"))

        # Angular rate
        if omega_max < THRESH["omega_warn"]:
            s["checks"].append(("Ang. Rate", "PASS", f"max = {omega_max:.0f} °/s"))
        elif omega_max < THRESH["omega_fail"]:
            s["checks"].append(("Ang. Rate", "WARN", f"max = {omega_max:.0f} °/s"))
        else:
            s["checks"].append(("Ang. Rate", "FAIL", f"max = {omega_max:.0f} °/s"))

        # Pitch stability
        if pitch_std < THRESH["pitch_std_warn"]:
            s["checks"].append(("Pitch σ", "PASS", f"σ = {pitch_std:.2f}°"))
        elif pitch_std < THRESH["pitch_std_fail"]:
            s["checks"].append(("Pitch σ", "WARN", f"σ = {pitch_std:.2f}°"))
        else:
            s["checks"].append(("Pitch σ", "FAIL", f"σ = {pitch_std:.2f}°"))

        # Phase-specific checks (مع alias map للأسماء الجديدة)
        ph_canonical = PHASE_ALIASES.get(ph, ph)
        if ph_canonical == "BOOST" or ph == "POWERED_ASCENT":
            speed_gain = sub["velocity_total_m_s"].iloc[-1] - sub["velocity_total_m_s"].iloc[0]
            s["checks"].append(("ΔV", "PASS" if speed_gain > 50 else "WARN", f"ΔV = {speed_gain:.0f} m/s"))
        elif ph_canonical == "CRUISE":
            alt_var = sub["alt_agl_m"].std()
            s["checks"].append(("Alt Hold", "PASS" if alt_var < 20 else ("WARN" if alt_var < 50 else "FAIL"), f"σ_alt = {alt_var:.1f} m"))
            gamma_std = sub["gamma_deg"].std() if "gamma_deg" in sub.columns else 0
            s["checks"].append(("γ Hold", "PASS" if gamma_std < 3 else ("WARN" if gamma_std < 8 else "FAIL"), f"σ_γ = {gamma_std:.1f}°"))
        elif ph_canonical == "TERMINAL" or ph == "TERMINAL":
            final_gamma = sub["gamma_deg"].iloc[-1] if "gamma_deg" in sub.columns else 0
            s["checks"].append(("Dive γ", "PASS" if final_gamma < -30 else "WARN", f"γ_end = {final_gamma:.1f}°"))

        # Overall verdict for phase
        verdicts = [c[1] for c in s["checks"]]
        if "FAIL" in verdicts:
            s["overall"] = "FAIL"
        elif "WARN" in verdicts:
            s["overall"] = "WARN"
        else:
            s["overall"] = "PASS"
        scored.append(s)
    return scored


def _phase_score_html(phase_scores):
    """Render phase scoring as HTML cards."""
    if not phase_scores:
        return '<div class="card">No phase data available</div>'
    h = '<div class="grid grid-2" style="gap:12px">'
    for ps in phase_scores:
        badge = f'<span class="badge {ps["overall"].lower()}">{ps["overall"]}</span>'
        rows = ""
        for check_name, verdict, detail in ps["checks"]:
            v_cls = verdict.lower()
            rows += f'<tr><td>{check_name}</td><td><span class="badge {v_cls}" style="font-size:.65rem">{verdict}</span></td><td style="font-size:.8rem;color:#555">{detail}</td></tr>'
        h += f'<div class="card"><h3 style="margin-bottom:8px"><span style="color:{ps["color"]}">■</span> {ps["name"]} {badge}</h3><table style="font-size:.85rem"><tr><th>Check</th><th>Status</th><th>Detail</th></tr>{rows}</table></div>'
    h += '</div>'
    return h

def _fig_trajectory(df):
    t = df["time_s"]
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Altitude AGL vs Time", "Ground Range vs Time",
                                        "Trajectory Profile", "Speed vs Time"),
                        vertical_spacing=0.12, horizontal_spacing=0.08)
    for ph in OrderedDict.fromkeys(df["flight_phase"]):
        mask = df["flight_phase"] == ph
        c = PHASE_COLORS.get(ph, "#333")
        fig.add_trace(go.Scatter(x=t[mask], y=df["alt_agl_m"][mask], mode="lines",
                                 name=ph, line=dict(color=c, width=2), legendgroup=ph), row=1, col=1)
        fig.add_trace(go.Scatter(x=t[mask], y=df["ground_range_m"][mask], mode="lines",
                                 line=dict(color=c, width=2), legendgroup=ph, showlegend=False), row=1, col=2)
        fig.add_trace(go.Scatter(x=df["ground_range_m"][mask], y=df["alt_agl_m"][mask],
                                 mode="lines", line=dict(color=c, width=2), legendgroup=ph, showlegend=False), row=2, col=1)
        fig.add_trace(go.Scatter(x=t[mask], y=df["velocity_total_m_s"][mask], mode="lines",
                                 line=dict(color=c, width=2), legendgroup=ph, showlegend=False), row=2, col=2)
    fig.add_hline(y=0, line_dash="dot", line_color="gray", opacity=0.4, row=1, col=1)
    fig.add_hline(y=TARGET_RANGE_M, line_dash="dash", line_color="red", opacity=0.5, row=1, col=2,
                  annotation_text=f"Target {TARGET_RANGE_M:.0f}m")
    fig.add_vline(x=TARGET_RANGE_M, line_dash="dash", line_color="red", opacity=0.4, row=2, col=1)
    fig.update_xaxes(title_text="Time (s)", row=1, col=1); fig.update_xaxes(title_text="Time (s)", row=1, col=2)
    fig.update_xaxes(title_text="Ground Range (m)", row=2, col=1); fig.update_xaxes(title_text="Time (s)", row=2, col=2)
    fig.update_yaxes(title_text="Alt AGL (m)", row=1, col=1); fig.update_yaxes(title_text="Range (m)", row=1, col=2)
    fig.update_yaxes(title_text="Alt AGL (m)", row=2, col=1); fig.update_yaxes(title_text="Speed (m/s)", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white", legend=dict(orientation="h", yanchor="bottom", y=1.02))
    return fig

def _fig_attitude(df):
    t = df["time_s"]
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Euler Angles", "Flight Path Angle γ",
                                        "α & β (AoA / Sideslip)", "Angular Rates"),
                        vertical_spacing=0.12, horizontal_spacing=0.08)
    fig.add_trace(go.Scatter(x=t, y=df["pitch_deg"], name="Pitch", line=dict(color="#1f77b4")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["yaw_deg"], name="Yaw", line=dict(color="#d62728")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["roll_deg"], name="Roll", line=dict(color="#2ca02c")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["gamma_deg"], name="γ", line=dict(color="#17becf"), showlegend=False), row=1, col=2)
    fig.add_hline(y=CRUISE_GAMMA_DEG, line_dash="dash", line_color="orange", opacity=0.6, row=1, col=2,
                  annotation_text=f"Cruise {CRUISE_GAMMA_DEG}°")
    fig.add_hline(y=0, line_dash="dot", line_color="gray", opacity=0.3, row=1, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["alpha_deg"], name="α", line=dict(color="#1f77b4"), showlegend=False), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["beta_deg"], name="β", line=dict(color="#d62728"), showlegend=False), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["omega_y_deg_s"], name="q (pitch)", line=dict(color="#1f77b4"), showlegend=False), row=2, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["omega_z_deg_s"], name="r (yaw)", line=dict(color="#d62728"), showlegend=False), row=2, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["omega_x_deg_s"], name="p (roll)", line=dict(color="#2ca02c"), showlegend=False), row=2, col=2)
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="Angle (deg)", row=1, col=1); fig.update_yaxes(title_text="γ (deg)", row=1, col=2)
    fig.update_yaxes(title_text="Angle (deg)", row=2, col=1); fig.update_yaxes(title_text="Rate (deg/s)", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white")
    return fig

def _fig_aero_forces(df):
    t = df["time_s"]
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Dynamic Pressure & Mach", "Thrust & Axial Force",
                                        "Aero Force Coefficients", "Moments"),
                        vertical_spacing=0.12, horizontal_spacing=0.08)
    fig.add_trace(go.Scatter(x=t, y=df["q_dynamic_Pa"]/1000, name="q (kPa)", line=dict(color="#ff7f0e")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["mach"]*100, name="Mach×100", line=dict(color="#d62728", dash="dash")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["thrust_total_N"], name="Thrust", line=dict(color="#d62728")), row=1, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["force_x_N"], name="Force X", line=dict(color="#1f77b4", dash="dot")), row=1, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["CN_total"], name="CN total", line=dict(color="#1f77b4")), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["CN_control"], name="CN ctrl", line=dict(color="#d62728", dash="dash")), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["CY_total"], name="CY total", line=dict(color="#2ca02c")), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["moment_y_Nm"], name="M_pitch", line=dict(color="#1f77b4")), row=2, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["moment_z_Nm"], name="M_yaw", line=dict(color="#d62728")), row=2, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["moment_x_Nm"], name="M_roll", line=dict(color="#2ca02c")), row=2, col=2)
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="kPa / Mach×100", row=1, col=1); fig.update_yaxes(title_text="Force (N)", row=1, col=2)
    fig.update_yaxes(title_text="Coefficient", row=2, col=1); fig.update_yaxes(title_text="Moment (N·m)", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white")
    return fig

def _fig_gload_energy(df):
    t = df["time_s"]
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("G-Load (Body)", "Mass & CG",
                                        "Energy Budget", "FUR Acceleration"),
                        vertical_spacing=0.12, horizontal_spacing=0.08)
    fig.add_trace(go.Scatter(x=t, y=df["acceleration_body_x_g"], name="Ax (axial)", line=dict(color="#d62728")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["acceleration_body_y_g"], name="Ay (normal)", line=dict(color="#1f77b4")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["acceleration_body_z_g"], name="Az (lat)", line=dict(color="#2ca02c")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["g_total"], name="Total", line=dict(color="black", dash="dash")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["mass_kg"], name="Mass (kg)", line=dict(color="#333")), row=1, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["xbc_m"]*100, name="xbc (cm)", line=dict(color="#17becf", dash="dash")), row=1, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["KE_kJ"], name="KE", line=dict(color="#d62728")), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["PE_kJ"], name="PE", line=dict(color="#1f77b4")), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["total_energy_kJ"], name="Total", line=dict(color="black", dash="dash")), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["acceleration_fur_x_m_s2"], name="A_fwd", line=dict(color="#d62728")), row=2, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["acceleration_fur_y_m_s2"], name="A_up", line=dict(color="#1f77b4")), row=2, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["acceleration_fur_z_m_s2"], name="A_right", line=dict(color="#2ca02c")), row=2, col=2)
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="G", row=1, col=1); fig.update_yaxes(title_text="kg / cm", row=1, col=2)
    fig.update_yaxes(title_text="Energy (kJ)", row=2, col=1); fig.update_yaxes(title_text="Accel (m/s²)", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white")
    return fig

def _fig_cm_stability(df):
    t = df["time_s"]
    fig = make_subplots(rows=1, cols=2,
                        subplot_titles=("Moment Coefficients", "Aero Moments"),
                        horizontal_spacing=0.08)
    fig.add_trace(go.Scatter(x=t, y=df["CM_total"], name="CM total", line=dict(color="#1f77b4")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["CM_control"], name="CM ctrl", line=dict(color="#d62728", dash="dash")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["Cn_total"], name="Cn total", line=dict(color="#2ca02c")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["Cn_control"], name="Cn ctrl", line=dict(color="#ff7f0e", dash="dash")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["M_pitch_aero"], name="M pitch aero", line=dict(color="#1f77b4")), row=1, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["M_yaw_aero"], name="M yaw aero", line=dict(color="#d62728")), row=1, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["M_roll_aero"], name="M roll aero", line=dict(color="#2ca02c")), row=1, col=2)
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="Coefficient", row=1, col=1); fig.update_yaxes(title_text="Moment (N·m)", row=1, col=2)
    fig.update_layout(height=400, template="plotly_white")
    return fig

def _fig_control_effectiveness(df):
    t = df["time_s"]
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Fin Effectiveness (CN_δ, CM_δ)", "Yaw Control (CY_ctrl, Cn_ctrl)",
                                        "Total Coefficients (base+control)", "Static Margin"),
                        vertical_spacing=0.12, horizontal_spacing=0.08)
    fig.add_trace(go.Scatter(x=t, y=df["CN_delta"], name="CN_δ", line=dict(color="#1f77b4")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["CM_delta"], name="CM_δ", line=dict(color="#d62728")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["CY_control"], name="CY ctrl", line=dict(color="#2ca02c")), row=1, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["Cn_control"], name="Cn ctrl", line=dict(color="#ff7f0e")), row=1, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["CN_total"], name="CN total", line=dict(color="#1f77b4")), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["CM_total"], name="CM total", line=dict(color="#d62728")), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["CY_total"], name="CY total", line=dict(color="#2ca02c", dash="dash")), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["Cn_total"], name="Cn total", line=dict(color="#ff7f0e", dash="dash")), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["static_margin_cal"], name="SM (cal)", line=dict(color="#9c27b0", width=2)), row=2, col=2)
    fig.add_hline(y=0, line_dash="dot", line_color="gray", opacity=0.4, row=2, col=2)
    fig.add_hline(y=0.5, line_dash="dash", line_color="orange", opacity=0.5, row=2, col=2,
                  annotation_text="Min recommended")
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="Coefficient", row=1, col=1); fig.update_yaxes(title_text="Coefficient", row=1, col=2)
    fig.update_yaxes(title_text="Coefficient", row=2, col=1); fig.update_yaxes(title_text="Calibers", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white")
    return fig

def _fig_forces_3axis(df):
    t = df["time_s"]
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Aero Forces (3-axis)", "Thrust Vector Components",
                                        "Airspeed vs Ground Speed", "Mach vs Mach_aero"),
                        vertical_spacing=0.12, horizontal_spacing=0.08)
    fig.add_trace(go.Scatter(x=t, y=df["force_x_N"], name="Fx (axial)", line=dict(color="#d62728")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["force_y_N"], name="Fy (lateral)", line=dict(color="#2ca02c")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["force_z_N"], name="Fz (normal)", line=dict(color="#1f77b4")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["thrust_x_N"], name="Tx", line=dict(color="#d62728")), row=1, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["thrust_y_N"], name="Ty", line=dict(color="#2ca02c")), row=1, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["thrust_z_N"], name="Tz", line=dict(color="#1f77b4")), row=1, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["airspeed_m_s"], name="Airspeed", line=dict(color="#1f77b4")), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["velocity_total_m_s"], name="Ground speed", line=dict(color="#d62728", dash="dash")), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["mach"], name="Mach", line=dict(color="#1f77b4")), row=2, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["mach_aero"], name="Mach aero", line=dict(color="#d62728", dash="dash")), row=2, col=2)
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="Force (N)", row=1, col=1); fig.update_yaxes(title_text="Force (N)", row=1, col=2)
    fig.update_yaxes(title_text="Speed (m/s)", row=2, col=1); fig.update_yaxes(title_text="Mach", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white")
    return fig

def _fig_3d_trajectory(df):
    fig = go.Figure()
    # في long_range mode، position_x/y_m هي ECEF (أَرقام بالملايين، غير قابلة للقراءة).
    # نَحوّل إلى local NED من lat/lon (إذا متاحة) أو نُسقِط ECEF إلى local origin.
    if "latitude_deg" in df.columns and "longitude_deg" in df.columns:
        lat_arr = df["latitude_deg"].values.astype(float)
        lon_arr = df["longitude_deg"].values.astype(float)
        # auto-detect: إذا |lat| < 1.6 → الأرجح radians (CSV قديم) → تَحويل.
        if abs(lat_arr[0]) < 1.6:
            lat_arr = np.degrees(lat_arr); lon_arr = np.degrees(lon_arr)
        lat0 = float(lat_arr[0]); lon0 = float(lon_arr[0])
        x_north = (lat_arr - lat0) * 111320.0
        y_east = (lon_arr - lon0) * 111320.0 * np.cos(np.radians(lat0))
    else:
        # fallback: subtract initial ECEF position (يُعطي local ECEF — ليس NED حقيقي)
        x_north = df["position_x_m"].values - df["position_x_m"].iloc[0]
        y_east = df["position_y_m"].values - df["position_y_m"].iloc[0]
    # Z: AGL أكثر وضوحاً من MSL (يَبدأ من 0 بدل 1200)
    z_alt = df["alt_agl_m"].values if "alt_agl_m" in df.columns else df["altitude_m"].values

    # تَرتيب المحاور للخريطة الطبيعية:
    #   x = East (يميناً عند زيادة)
    #   y = North (للأمام عند زيادة)
    #   z = Altitude
    for ph in OrderedDict.fromkeys(df["flight_phase"]):
        mask = (df["flight_phase"] == ph).values
        c = PHASE_COLORS.get(ph, "#333")
        fig.add_trace(go.Scatter3d(
            x=y_east[mask], y=x_north[mask], z=z_alt[mask],
            mode="lines", name=ph, line=dict(color=c, width=4), legendgroup=ph))

    # علامات الإطلاق والاصطدام
    fig.add_trace(go.Scatter3d(
        x=[y_east[0]], y=[x_north[0]], z=[z_alt[0]],
        mode="markers", marker=dict(size=8, color="green", symbol="diamond"),
        name="Launch", showlegend=True))
    fig.add_trace(go.Scatter3d(
        x=[y_east[-1]], y=[x_north[-1]], z=[z_alt[-1]],
        mode="markers", marker=dict(size=8, color="red", symbol="x"),
        name="Impact", showlegend=True))
    # نُسقط على المستويات لرؤية أوضح
    fig.add_trace(go.Scatter3d(
        x=y_east, y=x_north, z=np.zeros_like(z_alt),
        mode="lines", line=dict(color="#999", width=1, dash="dash"),
        name="Ground track", showlegend=True, opacity=0.6))

    fig.update_layout(
        height=650, template="plotly_white",
        scene=dict(
            xaxis_title="East (m)  →", yaxis_title="North (m)  ↑", zaxis_title="Altitude AGL (m)",
            aspectmode="auto",
            # camera من الجنوب-الغرب-العالي → East يميناً، North للأمام
            camera=dict(eye=dict(x=-1.5, y=-1.5, z=0.8)),
        ),
        legend=dict(orientation="h", yanchor="bottom", y=1.02))
    return fig

def _fig_phase_portrait(df):
    """Phase portrait: omega vs angle for pitch and yaw — reveals limit cycles.
    Uses gradient coloring by time, direction arrows, and start/end markers."""
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=(
                            "Pitch Phase Portrait (q vs θ)",
                            "Yaw Phase Portrait (r vs ψ)",
                            "Pitch: Time-Colored Trajectory",
                            "Yaw: Time-Colored Trajectory",
                        ),
                        vertical_spacing=0.14, horizontal_spacing=0.10)

    phases = list(OrderedDict.fromkeys(df["flight_phase"]))
    t_arr = df["time_s"].values

    # ── Row 1: Phase-colored trajectories with direction arrows ──
    for ph in phases:
        mask = df["flight_phase"] == ph
        c = PHASE_COLORS.get(ph, "#333")
        idx = np.where(mask)[0]
        if len(idx) < 2:
            continue

        # Pitch phase portrait
        px, py = df["pitch_deg"].values[idx], df["omega_y_deg_s"].values[idx]
        fig.add_trace(go.Scatter(
            x=px, y=py, mode="lines", name=ph, legendgroup=ph,
            line=dict(color=c, width=2, shape="spline", smoothing=0.8),
            opacity=0.85,
        ), row=1, col=1)

        # Yaw phase portrait
        yx_vals, yy = df["yaw_deg"].values[idx], df["omega_z_deg_s"].values[idx]
        fig.add_trace(go.Scatter(
            x=yx_vals, y=yy, mode="lines", legendgroup=ph, showlegend=False,
            line=dict(color=c, width=2, shape="spline", smoothing=0.8),
            opacity=0.85,
        ), row=1, col=2)

        # Direction arrows (every ~15% of phase segment)
        n_pts = len(idx)
        arrow_step = max(1, n_pts // 7)
        arrow_idx = list(range(arrow_step, n_pts - 1, arrow_step))
        for ai in arrow_idx:
            for col_i, (xv, yv) in enumerate([(px, py), (yx_vals, yy)], 1):
                dx = xv[ai] - xv[ai - 1]
                dy = yv[ai] - yv[ai - 1]
                norm = np.sqrt(dx**2 + dy**2)
                if norm < 1e-9:
                    continue
                fig.add_annotation(
                    x=xv[ai], y=yv[ai], ax=xv[ai] - dx / norm * 12,
                    ay=yv[ai] - dy / norm * 12,
                    xref=f"x{col_i}", yref=f"y{col_i}",
                    axref=f"x{col_i}", ayref=f"y{col_i}",
                    showarrow=True,
                    arrowhead=2, arrowsize=1.5, arrowwidth=1.8,
                    arrowcolor=c, opacity=0.7,
                )

    # Start / end markers on row 1
    for col_i, (xk, yk) in enumerate([("pitch_deg", "omega_y_deg_s"), ("yaw_deg", "omega_z_deg_s")], 1):
        fig.add_trace(go.Scatter(
            x=[df[xk].iloc[0]], y=[df[yk].iloc[0]], mode="markers",
            marker=dict(symbol="circle", size=10, color="#2ca02c", line=dict(width=2, color="white")),
            name="Start" if col_i == 1 else None, showlegend=(col_i == 1),
            legendgroup="markers",
        ), row=1, col=col_i)
        fig.add_trace(go.Scatter(
            x=[df[xk].iloc[-1]], y=[df[yk].iloc[-1]], mode="markers",
            marker=dict(symbol="x", size=11, color="#d62728", line=dict(width=2)),
            name="End" if col_i == 1 else None, showlegend=(col_i == 1),
            legendgroup="markers",
        ), row=1, col=col_i)

    # ── Row 2: Time-colored continuous trajectory (gradient) ──
    for col_i, (xk, yk, ax_label) in enumerate([
        ("pitch_deg", "omega_y_deg_s", "Pitch"),
        ("yaw_deg", "omega_z_deg_s", "Yaw"),
    ], 1):
        xv = df[xk].values
        yv = df[yk].values
        # Draw segments colored by time
        fig.add_trace(go.Scatter(
            x=xv, y=yv, mode="lines+markers",
            marker=dict(
                size=3, color=t_arr, colorscale="Viridis", showscale=(col_i == 2),
                colorbar=dict(title="Time (s)", x=1.02, len=0.4, y=0.18) if col_i == 2 else None,
            ),
            line=dict(color="rgba(150,150,150,0.3)", width=1),
            showlegend=False, hovertemplate=f"{ax_label}: %{{x:.1f}}°<br>Rate: %{{y:.1f}}°/s<br>t=%{{marker.color:.2f}}s<extra></extra>",
        ), row=2, col=col_i)
        # Start / end
        fig.add_trace(go.Scatter(
            x=[xv[0]], y=[yv[0]], mode="markers",
            marker=dict(symbol="circle", size=10, color="#2ca02c", line=dict(width=2, color="white")),
            showlegend=False,
        ), row=2, col=col_i)
        fig.add_trace(go.Scatter(
            x=[xv[-1]], y=[yv[-1]], mode="markers",
            marker=dict(symbol="x", size=11, color="#d62728", line=dict(width=2)),
            showlegend=False,
        ), row=2, col=col_i)

    # Axis labels
    fig.update_xaxes(title_text="Pitch θ (deg)", row=1, col=1)
    fig.update_xaxes(title_text="Yaw ψ (deg)", row=1, col=2)
    fig.update_xaxes(title_text="Pitch θ (deg)", row=2, col=1)
    fig.update_xaxes(title_text="Yaw ψ (deg)", row=2, col=2)
    fig.update_yaxes(title_text="q – Pitch Rate (deg/s)", row=1, col=1)
    fig.update_yaxes(title_text="r – Yaw Rate (deg/s)", row=1, col=2)
    fig.update_yaxes(title_text="q – Pitch Rate (deg/s)", row=2, col=1)
    fig.update_yaxes(title_text="r – Yaw Rate (deg/s)", row=2, col=2)

    # Grid + crosshair at origin
    for r in [1, 2]:
        for c_i in [1, 2]:
            fig.add_hline(y=0, line_dash="dot", line_color="gray", opacity=0.3, row=r, col=c_i)
            fig.add_vline(x=0, line_dash="dot", line_color="gray", opacity=0.3, row=r, col=c_i)

    fig.update_layout(
        height=800, template="plotly_white",
        legend=dict(orientation="h", yanchor="bottom", y=1.02, font=dict(size=11)),
    )
    return fig

def _fig_energy_dissipation(df):
    """Energy budget with dissipation rate — shows where drag wastes energy."""
    t = df["time_s"]
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Energy Components", "Energy Dissipation Rate (dE/dt)",
                                        "Specific Energy (E/m)", "Cumulative Energy Loss"),
                        vertical_spacing=0.12, horizontal_spacing=0.08)
    fig.add_trace(go.Scatter(x=t, y=df["KE_kJ"], name="KE", line=dict(color="#d62728")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["PE_kJ"], name="PE", line=dict(color="#1f77b4")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["total_energy_kJ"], name="Total", line=dict(color="black", width=2)), row=1, col=1)
    # dE/dt
    fig.add_trace(go.Scatter(x=t, y=df["dE_dt_kW"], name="dE/dt", line=dict(color="#ff7f0e"), showlegend=False), row=1, col=2)
    fig.add_hline(y=0, line_dash="dot", line_color="gray", opacity=0.4, row=1, col=2)
    # Specific energy
    specific_e = df["total_energy_kJ"] / df["mass_kg"] * 1000  # J/kg
    fig.add_trace(go.Scatter(x=t, y=specific_e, name="Es", line=dict(color="#9c27b0"), showlegend=False), row=2, col=1)
    # Cumulative energy loss (integral of negative dE/dt)
    de_dt = df["dE_dt_kW"].values
    dt_arr = np.gradient(df["time_s"].values)
    loss = np.cumsum(np.where(de_dt < 0, -de_dt * dt_arr, 0))
    fig.add_trace(go.Scatter(x=t, y=loss, name="Cumul. loss", line=dict(color="#d62728"), showlegend=False), row=2, col=2)
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="Energy (kJ)", row=1, col=1); fig.update_yaxes(title_text="Power (kW)", row=1, col=2)
    fig.update_yaxes(title_text="Es (J/kg)", row=2, col=1); fig.update_yaxes(title_text="Energy Lost (kJ)", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white")
    return fig

def _fig_velocity_components(df):
    """All velocity components in NED + horizontal/vertical."""
    t = df["time_s"]
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("NED Velocity Components", "Horizontal & Vertical Speed",
                                        "Velocity FUR Frame", "Position FUR Frame"),
                        vertical_spacing=0.12, horizontal_spacing=0.08)
    # NED Velocity: نَستَخدم vel_ned_* (الصحيح). velocity_x/y/z هي ECEF في long_range.
    if "vel_ned_north_m_s" in df.columns:
        vn, ve, vd = df["vel_ned_north_m_s"], df["vel_ned_east_m_s"], df["vel_ned_down_m_s"]
    else:
        vn, ve, vd = df["velocity_x_m_s"], df["velocity_y_m_s"], df["velocity_z_m_s"]
    fig.add_trace(go.Scatter(x=t, y=vn, name="Vn (North)", line=dict(color="#d62728")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=ve, name="Ve (East)", line=dict(color="#2ca02c")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=vd, name="Vd (Down)", line=dict(color="#1f77b4")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["speed_horizontal_m_s"], name="V_horiz", line=dict(color="#ff7f0e")), row=1, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["speed_vertical_m_s"], name="V_vert", line=dict(color="#9c27b0")), row=1, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["velocity_total_m_s"], name="V_total", line=dict(color="black", dash="dash")), row=1, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["velocity_fur_x_m_s"], name="V_fwd", line=dict(color="#d62728")), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["velocity_fur_y_m_s"], name="V_up", line=dict(color="#1f77b4")), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["velocity_fur_z_m_s"], name="V_right", line=dict(color="#2ca02c")), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["position_fur_x_m"], name="X_fwd", line=dict(color="#d62728")), row=2, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["position_fur_y_m"], name="Y_up", line=dict(color="#1f77b4")), row=2, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["position_fur_z_m"], name="Z_right", line=dict(color="#2ca02c")), row=2, col=2)
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="Vel (m/s)", row=1, col=1); fig.update_yaxes(title_text="Speed (m/s)", row=1, col=2)
    fig.update_yaxes(title_text="Vel (m/s)", row=2, col=1); fig.update_yaxes(title_text="Position (m)", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white")
    return fig

def _fig_fft_spectrum(df):
    """FFT frequency spectrum for pitch and yaw — reveals dominant oscillation frequencies."""
    from scipy.fft import rfft, rfftfreq
    t = df["time_s"].values
    dt_mean = np.mean(np.diff(t))
    n = len(t)
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Pitch Angle Spectrum", "Pitch Rate Spectrum",
                                        "Yaw Angle Spectrum", "Yaw Rate Spectrum"),
                        vertical_spacing=0.12, horizontal_spacing=0.08)
    signals = [("pitch_deg", 1, 1), ("omega_y_deg_s", 1, 2),
               ("yaw_deg", 2, 1), ("omega_z_deg_s", 2, 2)]
    colors = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e"]
    freq = rfftfreq(n, dt_mean)
    for (col_name, r, c), clr in zip(signals, colors):
        sig = df[col_name].values - np.mean(df[col_name].values)  # remove DC
        amp = 2.0 / n * np.abs(rfft(sig))
        # Skip DC component (freq=0), limit to meaningful range
        fig.add_trace(go.Scatter(x=freq[1:], y=amp[1:], mode="lines",
                                 line=dict(color=clr, width=1.5), showlegend=False), row=r, col=c)
    fig.update_xaxes(title_text="Frequency (Hz)", range=[0, min(50, 0.5/dt_mean)])
    fig.update_yaxes(title_text="Amplitude (deg)", row=1, col=1); fig.update_yaxes(title_text="Amplitude (deg/s)", row=1, col=2)
    fig.update_yaxes(title_text="Amplitude (deg)", row=2, col=1); fig.update_yaxes(title_text="Amplitude (deg/s)", row=2, col=2)
    fig.update_layout(height=650, template="plotly_white")
    return fig

def _fig_tracking_error(df):
    """MPC tracking error: actual vs reference for gamma and chi, plus fin deflections."""
    t = df["time_s"]
    has_mpc = "mpc_gamma_ref_deg" in df.columns and df["mpc_gamma_ref_deg"].abs().max() > 0.01
    has_fins = "delta_pitch_deg" in df.columns
    n_rows = 2 + (1 if has_fins else 0)
    subplot_titles = [
        "Flight Path Angle: Actual vs Reference",
        "Tracking Error (γ and χ)",
        "Heading: Actual vs Reference",
        "Wind Components",
    ]
    if has_fins:
        subplot_titles += ["Fin Deflections (virtual axes)", "Fin Deflections (4 fins)"]

    fig = make_subplots(
        rows=n_rows,
        cols=2,
        subplot_titles=subplot_titles,
        vertical_spacing=0.10,
        horizontal_spacing=0.08,
    )
    if has_mpc:
        # gamma actual vs ref
        fig.add_trace(go.Scatter(x=t, y=df["gamma_deg"], name="γ actual", line=dict(color="#1f77b4")), row=1, col=1)
        fig.add_trace(go.Scatter(x=t, y=df["mpc_gamma_ref_deg"], name="γ ref", line=dict(color="#d62728", dash="dash")), row=1, col=1)
        # tracking error
        fig.add_trace(go.Scatter(x=t, y=df["tracking_error_gamma_deg"], name="γ error", line=dict(color="#ff7f0e")), row=1, col=2)
        if "tracking_error_chi_deg" in df.columns:
            fig.add_trace(go.Scatter(x=t, y=df["tracking_error_chi_deg"], name="χ error", line=dict(color="#9c27b0")), row=1, col=2)
        fig.add_hline(y=0, line_dash="dot", line_color="gray", opacity=0.4, row=1, col=2)
        # chi actual (heading السرعة، ليس yaw الجسم) vs ref
        chi_actual = df["chi_deg"] if "chi_deg" in df.columns else df["yaw_deg"]
        fig.add_trace(go.Scatter(x=t, y=chi_actual, name="χ actual", line=dict(color="#1f77b4"), showlegend=False), row=2, col=1)
        fig.add_trace(go.Scatter(x=t, y=df["mpc_chi_ref_deg"], name="χ ref", line=dict(color="#d62728", dash="dash"), showlegend=False), row=2, col=1)
    else:
        fig.add_trace(go.Scatter(x=t, y=df["gamma_deg"], name="γ", line=dict(color="#1f77b4")), row=1, col=1)
        fig.add_annotation(text="MPC reference data not available", xref="x domain", yref="y domain",
                          x=0.5, y=0.5, showarrow=False, font=dict(size=14, color="gray"), row=1, col=2)
        fig.add_trace(go.Scatter(x=t, y=df["yaw_deg"], name="Yaw", line=dict(color="#1f77b4"), showlegend=False), row=2, col=1)
    # Wind
    if "wind_north_m_s" in df.columns:
        fig.add_trace(go.Scatter(x=t, y=df["wind_north_m_s"], name="W_north", line=dict(color="#d62728")), row=2, col=2)
        fig.add_trace(go.Scatter(x=t, y=df["wind_east_m_s"], name="W_east", line=dict(color="#2ca02c")), row=2, col=2)
        fig.add_trace(go.Scatter(x=t, y=df["wind_down_m_s"], name="W_down", line=dict(color="#1f77b4")), row=2, col=2)
    # Fins
    if has_fins:
        fig.add_trace(go.Scatter(x=t, y=df["delta_pitch_deg"], name="δ_pitch", line=dict(color="#1f77b4")), row=3, col=1)
        fig.add_trace(go.Scatter(x=t, y=df["delta_yaw_deg"], name="δ_yaw", line=dict(color="#d62728")), row=3, col=1)
        fig.add_trace(go.Scatter(x=t, y=df["delta_roll_deg"], name="δ_roll", line=dict(color="#2ca02c")), row=3, col=1)
        if "fin_1_rad" in df.columns:
            for j, clr in enumerate(["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e"]):
                fig.add_trace(go.Scatter(x=t, y=np.degrees(df[f"fin_{j+1}_rad"]),
                             name=f"Fin {j+1}", line=dict(color=clr, width=1)), row=3, col=2)
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="γ (deg)", row=1, col=1); fig.update_yaxes(title_text="Error (deg)", row=1, col=2)
    fig.update_yaxes(title_text="χ / Yaw (deg)", row=2, col=1); fig.update_yaxes(title_text="Wind (m/s)", row=2, col=2)
    if has_fins:
        fig.update_yaxes(title_text="δ (deg)", row=3, col=1); fig.update_yaxes(title_text="δ (deg)", row=3, col=2)
    fig.update_layout(height=350 * n_rows, template="plotly_white")
    return fig


# ─── New Analysis Charts (previously missing data) ──────────────────────────

def _fig_mpc_diagnostics(df):
    """MPC solver diagnostics: solve time, status, SQP iterations, virtual commands."""
    t = df["time_s"]
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("MPC Solve Time", "MPC Solver Status",
                                        "SQP Iterations", "MPC Virtual Commands"),
                        vertical_spacing=0.12, horizontal_spacing=0.08)
    fig.add_trace(go.Scatter(x=t, y=df["mpc_solve_time_ms"], name="Solve time",
                             line=dict(color="#1f77b4", width=1)), row=1, col=1)
    mean_st = df["mpc_solve_time_ms"].mean()
    fig.add_hline(y=mean_st, line_dash="dash", line_color="orange", opacity=0.6, row=1, col=1,
                  annotation_text=f"Mean {mean_st:.2f} ms")
    fig.add_trace(go.Scatter(x=t, y=df["mpc_solver_status"], name="Status",
                             mode="markers", marker=dict(size=3, color=df["mpc_solver_status"],
                             colorscale=[[0,"#4caf50"],[0.5,"#ff9800"],[1,"#f44336"]])), row=1, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["mpc_sqp_iterations"], name="SQP iters",
                             line=dict(color="#9c27b0", width=1)), row=2, col=1)
    if "mpc_delta_e_deg" in df.columns:
        fig.add_trace(go.Scatter(x=t, y=df["mpc_delta_e_deg"], name="δe (pitch)",
                                 line=dict(color="#1f77b4")), row=2, col=2)
        fig.add_trace(go.Scatter(x=t, y=df["mpc_delta_r_deg"], name="δr (yaw)",
                                 line=dict(color="#d62728")), row=2, col=2)
        fig.add_trace(go.Scatter(x=t, y=df["mpc_delta_a_deg"], name="δa (roll)",
                                 line=dict(color="#2ca02c")), row=2, col=2)
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="ms", row=1, col=1); fig.update_yaxes(title_text="Status (0=OK)", row=1, col=2)
    fig.update_yaxes(title_text="Iterations", row=2, col=1); fig.update_yaxes(title_text="Deflection (deg)", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white")
    return fig


def _fig_actuator_analysis(df):
    """Actuator: commanded vs actual, lag, fin authority, safety violations."""
    t = df["time_s"]
    has_cmd = df.attrs.get("has_actuator_cmd", False)
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=(
                            "Commanded vs Actual Fins" if has_cmd else "Fin Deflections",
                            "Actuator Lag (cmd - actual)" if has_cmd else "Fin Authority",
                            "Fin Authority", "Safety Violations"),
                        vertical_spacing=0.12, horizontal_spacing=0.08)
    fin_colors = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e"]
    if has_cmd:
        for j, clr in enumerate(fin_colors):
            fig.add_trace(go.Scatter(x=t, y=np.degrees(df[f"fin_{j+1}_rad"]),
                         name=f"Fin{j+1} actual", line=dict(color=clr, width=1.5)), row=1, col=1)
            fig.add_trace(go.Scatter(x=t, y=df[f"actuator_cmd_fin{j+1}_deg"],
                         name=f"Fin{j+1} cmd", line=dict(color=clr, width=1, dash="dash"),
                         showlegend=False), row=1, col=1)
        for j, clr in enumerate(fin_colors):
            fig.add_trace(go.Scatter(x=t, y=df[f"actuator_lag_fin{j+1}_deg"],
                         name=f"Lag {j+1}", line=dict(color=clr, width=1), showlegend=False), row=1, col=2)
        fig.add_hline(y=0, line_dash="dot", line_color="gray", opacity=0.4, row=1, col=2)
    else:
        for j, clr in enumerate(fin_colors):
            if f"fin_{j+1}_rad" in df.columns:
                fig.add_trace(go.Scatter(x=t, y=np.degrees(df[f"fin_{j+1}_rad"]),
                             name=f"Fin {j+1}", line=dict(color=clr, width=1.5)), row=1, col=1)
    if "fin_authority" in df.columns:
        fig.add_trace(go.Scatter(x=t, y=df["fin_authority"], name="Authority",
                                 line=dict(color="#9c27b0", width=2), showlegend=False), row=2, col=1)
        fig.add_hline(y=0.5, line_dash="dash", line_color="orange", opacity=0.5, row=2, col=1,
                      annotation_text="50% authority")
    if "safety_violations" in df.columns:
        fig.add_trace(go.Scatter(x=t, y=df["safety_violations"].astype(float), name="Violations",
                                 mode="markers+lines", line=dict(color="#f44336", width=1),
                                 marker=dict(size=3), showlegend=False), row=2, col=2)
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="Deflection (deg)", row=1, col=1)
    fig.update_yaxes(title_text="Lag (deg)" if has_cmd else "Authority", row=1, col=2)
    fig.update_yaxes(title_text="Authority (0-1)", row=2, col=1)
    fig.update_yaxes(title_text="Violations", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white")
    return fig


def _fig_angular_dynamics(df):
    """Angular acceleration and total moments (after CG correction)."""
    t = df["time_s"]
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Angular Acceleration (deg/s²)",
                                        "Angular Acceleration (rad/s²)",
                                        "Total Moments (after CG correction)",
                                        "Aero Moments vs Total Moments (pitch)"),
                        vertical_spacing=0.12, horizontal_spacing=0.08)
    if "angular_accel_x_deg_s2" in df.columns:
        fig.add_trace(go.Scatter(x=t, y=df["angular_accel_x_deg_s2"], name="α̈ roll",
                                 line=dict(color="#2ca02c")), row=1, col=1)
        fig.add_trace(go.Scatter(x=t, y=df["angular_accel_y_deg_s2"], name="α̈ pitch",
                                 line=dict(color="#1f77b4")), row=1, col=1)
        fig.add_trace(go.Scatter(x=t, y=df["angular_accel_z_deg_s2"], name="α̈ yaw",
                                 line=dict(color="#d62728")), row=1, col=1)
    if "angular_accel_x_rad_s2" in df.columns:
        fig.add_trace(go.Scatter(x=t, y=df["angular_accel_x_rad_s2"], name="roll",
                                 line=dict(color="#2ca02c"), showlegend=False), row=1, col=2)
        fig.add_trace(go.Scatter(x=t, y=df["angular_accel_y_rad_s2"], name="pitch",
                                 line=dict(color="#1f77b4"), showlegend=False), row=1, col=2)
        fig.add_trace(go.Scatter(x=t, y=df["angular_accel_z_rad_s2"], name="yaw",
                                 line=dict(color="#d62728"), showlegend=False), row=1, col=2)
    if "total_moment_x_Nm" in df.columns:
        fig.add_trace(go.Scatter(x=t, y=df["total_moment_x_Nm"], name="M_roll total",
                                 line=dict(color="#2ca02c")), row=2, col=1)
        fig.add_trace(go.Scatter(x=t, y=df["total_moment_y_Nm"], name="M_pitch total",
                                 line=dict(color="#1f77b4")), row=2, col=1)
        fig.add_trace(go.Scatter(x=t, y=df["total_moment_z_Nm"], name="M_yaw total",
                                 line=dict(color="#d62728")), row=2, col=1)
    # Compare aero vs total for pitch
    if "M_pitch_aero" in df.columns and "total_moment_y_Nm" in df.columns:
        fig.add_trace(go.Scatter(x=t, y=df["M_pitch_aero"], name="M_pitch aero",
                                 line=dict(color="#1f77b4", dash="dash"), showlegend=False), row=2, col=2)
        fig.add_trace(go.Scatter(x=t, y=df["total_moment_y_Nm"], name="M_pitch total (CG)",
                                 line=dict(color="#d62728"), showlegend=False), row=2, col=2)
        fig.add_trace(go.Scatter(x=t, y=df["M_yaw_aero"], name="M_yaw aero",
                                 line=dict(color="#2ca02c", dash="dash"), showlegend=False), row=2, col=2)
        fig.add_trace(go.Scatter(x=t, y=df["total_moment_z_Nm"], name="M_yaw total (CG)",
                                 line=dict(color="#ff7f0e"), showlegend=False), row=2, col=2)
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="deg/s²", row=1, col=1); fig.update_yaxes(title_text="rad/s²", row=1, col=2)
    fig.update_yaxes(title_text="Moment (N·m)", row=2, col=1); fig.update_yaxes(title_text="Moment (N·m)", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white")
    return fig


def _fig_mass_propulsion(df):
    """Propellant fraction, CG position, CA, speed of sound, stage number."""
    t = df["time_s"]
    has_multi_stage = "stage_number" in df.columns and df["stage_number"].max() > 1
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Propellant Fraction & CG Position",
                                        "Axial Force Coefficient (CA)",
                                        "Speed of Sound & Mach",
                                        "Stage Number" if has_multi_stage else "Mass Breakdown"),
                        vertical_spacing=0.12, horizontal_spacing=0.08,
                        specs=[[{"secondary_y": True}, {}], [{}, {}]])
    if "propellant_fraction" in df.columns:
        fig.add_trace(go.Scatter(x=t, y=df["propellant_fraction"], name="Propellant fraction",
                                 line=dict(color="#d62728", width=2)), row=1, col=1, secondary_y=False)
    if "cg_position_m" in df.columns:
        fig.add_trace(go.Scatter(x=t, y=df["cg_position_m"], name="CG position (m)",
                                 line=dict(color="#1f77b4", width=2, dash="dash")), row=1, col=1, secondary_y=True)
    if "CA" in df.columns:
        fig.add_trace(go.Scatter(x=t, y=df["CA"], name="CA",
                                 line=dict(color="#ff7f0e", width=2), showlegend=False), row=1, col=2)
    if "speed_of_sound_m_s" in df.columns:
        fig.add_trace(go.Scatter(x=t, y=df["speed_of_sound_m_s"], name="Speed of sound",
                                 line=dict(color="#1f77b4")), row=2, col=1)
        fig.add_trace(go.Scatter(x=t, y=df["mach"] * 340, name="V = Mach × 340",
                                 line=dict(color="#d62728", dash="dot")), row=2, col=1)
    if has_multi_stage:
        fig.add_trace(go.Scatter(x=t, y=df["stage_number"], name="Stage",
                                 mode="lines+markers", line=dict(color="#9c27b0", width=2),
                                 marker=dict(size=3), showlegend=False), row=2, col=2)
    else:
        fig.add_trace(go.Scatter(x=t, y=df["mass_kg"], name="Total mass",
                                 line=dict(color="#333", width=2), showlegend=False), row=2, col=2)
        if "propellant_fraction" in df.columns:
            prop_mass = df["mass_kg"].iloc[0] * df["propellant_fraction"]
            fig.add_trace(go.Scatter(x=t, y=prop_mass, name="Propellant mass est.",
                                     line=dict(color="#d62728", dash="dash"), showlegend=False), row=2, col=2)
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="Fraction (0-1)", row=1, col=1, secondary_y=False)
    fig.update_yaxes(title_text="CG (m)", row=1, col=1, secondary_y=True)
    fig.update_yaxes(title_text="CA", row=1, col=2)
    fig.update_yaxes(title_text="Speed (m/s)", row=2, col=1)
    fig.update_yaxes(title_text="Stage #" if has_multi_stage else "Mass (kg)", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white")
    return fig


def _fig_velocity_frames(df):
    """Velocity in different frames: launch-fixed NED, local NED, air-relative NED."""
    t = df["time_s"]
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Vel Launch-Fixed NED (m/s)",
                                        "Vel Local NED (m/s)",
                                        "Vel Air-Relative NED (m/s)",
                                        "Wind Effect (Ground - Air)"),
                        vertical_spacing=0.12, horizontal_spacing=0.08)
    ned_labels = [("north", "#d62728"), ("east", "#2ca02c"), ("down", "#1f77b4")]
    for i, (ax, clr) in enumerate(ned_labels):
        col = f"vel_ned_launch_{ax}_m_s"
        if col in df.columns:
            fig.add_trace(go.Scatter(x=t, y=df[col], name=f"V_{ax} (launch)",
                                     line=dict(color=clr)), row=1, col=1)
    for i, (ax, clr) in enumerate(ned_labels):
        col = f"vel_ned_{ax}_m_s"
        if col in df.columns:
            fig.add_trace(go.Scatter(x=t, y=df[col], name=f"V_{ax} (local)",
                                     line=dict(color=clr), showlegend=False), row=1, col=2)
    for i, (ax, clr) in enumerate(ned_labels):
        col = f"vel_aero_{ax}_m_s"
        if col in df.columns:
            fig.add_trace(go.Scatter(x=t, y=df[col], name=f"V_{ax} (aero)",
                                     line=dict(color=clr), showlegend=False), row=2, col=1)
    # Wind effect = ground vel - air vel
    if "vel_ned_north_m_s" in df.columns and "vel_aero_north_m_s" in df.columns:
        for ax, clr in ned_labels:
            wind_eff = df[f"vel_ned_{ax}_m_s"] - df[f"vel_aero_{ax}_m_s"]
            fig.add_trace(go.Scatter(x=t, y=wind_eff, name=f"Δ_{ax}",
                                     line=dict(color=clr), showlegend=False), row=2, col=2)
    fig.update_xaxes(title_text="Time (s)")
    for r in [1, 2]:
        for c in [1, 2]:
            fig.update_yaxes(title_text="Vel (m/s)", row=r, col=c)
    fig.update_layout(height=700, template="plotly_white")
    return fig


def _fig_sensor_mhe(df):
    """Sensor measurements, biases, and MHE quality (conditional)."""
    t = df["time_s"]
    has_sensor = df.attrs.get("has_sensor", False)
    has_mhe = df.attrs.get("has_mhe", False)
    n_rows = (1 if has_sensor else 0) + (1 if has_mhe else 0) + 1
    n_rows = max(2, n_rows)
    titles = []
    if has_sensor:
        titles.extend(["Accelerometer (measured)", "Gyroscope (measured)"])
    if has_mhe:
        titles.extend(["MHE Estimation Quality", "MHE Solve Time"])
    if has_sensor:
        titles.extend(["Accel Bias", "Gyro Bias"])
    while len(titles) < 4:
        titles.append("")
    n_rows = (len(titles) + 1) // 2
    fig = make_subplots(rows=n_rows, cols=2, subplot_titles=titles,
                        vertical_spacing=0.12, horizontal_spacing=0.08)
    row_idx = 1
    if has_sensor:
        fig.add_trace(go.Scatter(x=t, y=df["accel_meas_x"], name="ax", line=dict(color="#d62728")), row=row_idx, col=1)
        fig.add_trace(go.Scatter(x=t, y=df["accel_meas_y"], name="ay", line=dict(color="#2ca02c")), row=row_idx, col=1)
        fig.add_trace(go.Scatter(x=t, y=df["accel_meas_z"], name="az", line=dict(color="#1f77b4")), row=row_idx, col=1)
        fig.add_trace(go.Scatter(x=t, y=df["gyro_meas_x"], name="gx", line=dict(color="#d62728"), showlegend=False), row=row_idx, col=2)
        fig.add_trace(go.Scatter(x=t, y=df["gyro_meas_y"], name="gy", line=dict(color="#2ca02c"), showlegend=False), row=row_idx, col=2)
        fig.add_trace(go.Scatter(x=t, y=df["gyro_meas_z"], name="gz", line=dict(color="#1f77b4"), showlegend=False), row=row_idx, col=2)
        fig.update_yaxes(title_text="Accel (m/s²)", row=row_idx, col=1)
        fig.update_yaxes(title_text="Gyro (rad/s)", row=row_idx, col=2)
        row_idx += 1
    if has_mhe:
        fig.add_trace(go.Scatter(x=t, y=df["mhe_quality"], name="Quality",
                                 line=dict(color="#9c27b0", width=2), showlegend=False), row=row_idx, col=1)
        fig.add_hline(y=1.0, line_dash="dash", line_color="green", opacity=0.5, row=row_idx, col=1,
                      annotation_text="Ideal")
        fig.add_trace(go.Scatter(x=t, y=df["mhe_solve_ms"], name="Solve ms",
                                 line=dict(color="#ff7f0e", width=1), showlegend=False), row=row_idx, col=2)
        fig.update_yaxes(title_text="Quality", row=row_idx, col=1)
        fig.update_yaxes(title_text="ms", row=row_idx, col=2)
        row_idx += 1
    if has_sensor:
        fig.add_trace(go.Scatter(x=t, y=df["accel_bias_x"], name="bias ax", line=dict(color="#d62728", dash="dash"), showlegend=False), row=row_idx, col=1)
        fig.add_trace(go.Scatter(x=t, y=df["accel_bias_y"], name="bias ay", line=dict(color="#2ca02c", dash="dash"), showlegend=False), row=row_idx, col=1)
        fig.add_trace(go.Scatter(x=t, y=df["accel_bias_z"], name="bias az", line=dict(color="#1f77b4", dash="dash"), showlegend=False), row=row_idx, col=1)
        fig.add_trace(go.Scatter(x=t, y=df["gyro_bias_x"], name="bias gx", line=dict(color="#d62728", dash="dash"), showlegend=False), row=row_idx, col=2)
        fig.add_trace(go.Scatter(x=t, y=df["gyro_bias_y"], name="bias gy", line=dict(color="#2ca02c", dash="dash"), showlegend=False), row=row_idx, col=2)
        fig.add_trace(go.Scatter(x=t, y=df["gyro_bias_z"], name="bias gz", line=dict(color="#1f77b4", dash="dash"), showlegend=False), row=row_idx, col=2)
        fig.update_yaxes(title_text="Bias (m/s²)", row=row_idx, col=1)
        fig.update_yaxes(title_text="Bias (rad/s)", row=row_idx, col=2)
    fig.update_xaxes(title_text="Time (s)")
    fig.update_layout(height=350 * n_rows, template="plotly_white")
    return fig


def _fig_mhe_detail(df):
    """Detailed MHE state vector analysis: estimated states vs truth + estimation errors."""
    t = df["time_s"]
    mhe_active = df["mhe_quality"] > 0

    fig = make_subplots(
        rows=5, cols=2,
        subplot_titles=(
            "Airspeed: MHE vs Truth",    "Flight Path Angle γ: MHE vs Truth",
            "AoA α: MHE vs Truth",       "Sideslip β: MHE vs Truth",
            "Gyro Bias Estimates",        "Wind Estimates (N, E)",
            "Angular Rates: MHE vs Truth","Roll Angle φ: MHE vs Truth",
            "MHE Estimation Errors",      "MHE Quality & Solve Time",
        ),
        vertical_spacing=0.07, horizontal_spacing=0.08,
    )

    # ── Row 1: Airspeed & gamma ──
    fig.add_trace(go.Scatter(x=t, y=df["airspeed_m_s"], name="V truth",
                             line=dict(color="#1f77b4", width=2)), row=1, col=1)
    fig.add_trace(go.Scatter(x=t[mhe_active], y=df["mhe_V_m_s"][mhe_active], name="V MHE",
                             line=dict(color="#ff7f0e", width=1.5, dash="dash")), row=1, col=1)
    fig.update_yaxes(title_text="m/s", row=1, col=1)

    fig.add_trace(go.Scatter(x=t, y=df["gamma_deg"], name="γ truth",
                             line=dict(color="#1f77b4", width=2), showlegend=False), row=1, col=2)
    fig.add_trace(go.Scatter(x=t[mhe_active], y=df["mhe_gamma_deg"][mhe_active], name="γ MHE",
                             line=dict(color="#ff7f0e", width=1.5, dash="dash"), showlegend=False), row=1, col=2)
    fig.update_yaxes(title_text="deg", row=1, col=2)

    # ── Row 2: AoA & sideslip ──
    fig.add_trace(go.Scatter(x=t, y=np.degrees(df["alpha_rad"]), name="α truth",
                             line=dict(color="#2ca02c", width=2), showlegend=False), row=2, col=1)
    fig.add_trace(go.Scatter(x=t[mhe_active], y=df["mhe_alpha_deg"][mhe_active], name="α MHE",
                             line=dict(color="#d62728", width=1.5, dash="dash"), showlegend=False), row=2, col=1)
    fig.update_yaxes(title_text="deg", row=2, col=1)

    fig.add_trace(go.Scatter(x=t, y=np.degrees(df["beta_rad"]), name="β truth",
                             line=dict(color="#2ca02c", width=2), showlegend=False), row=2, col=2)
    fig.add_trace(go.Scatter(x=t[mhe_active], y=df["mhe_beta_deg"][mhe_active], name="β MHE",
                             line=dict(color="#d62728", width=1.5, dash="dash"), showlegend=False), row=2, col=2)
    fig.update_yaxes(title_text="deg", row=2, col=2)

    # ── Row 3: Gyro bias & wind ──
    fig.add_trace(go.Scatter(x=t[mhe_active], y=np.degrees(df["mhe_bgx_rad_s"][mhe_active]),
                             name="b_gx", line=dict(color="#d62728")), row=3, col=1)
    fig.add_trace(go.Scatter(x=t[mhe_active], y=np.degrees(df["mhe_bgy_rad_s"][mhe_active]),
                             name="b_gy", line=dict(color="#2ca02c")), row=3, col=1)
    fig.add_trace(go.Scatter(x=t[mhe_active], y=np.degrees(df["mhe_bgz_rad_s"][mhe_active]),
                             name="b_gz", line=dict(color="#1f77b4")), row=3, col=1)
    fig.add_hline(y=0, line_dash="dot", line_color="black", opacity=0.3, row=3, col=1)
    fig.update_yaxes(title_text="deg/s", row=3, col=1)

    fig.add_trace(go.Scatter(x=t[mhe_active], y=df["mhe_wn_m_s"][mhe_active],
                             name="Wind N", line=dict(color="#9467bd")), row=3, col=2)
    fig.add_trace(go.Scatter(x=t[mhe_active], y=df["mhe_we_m_s"][mhe_active],
                             name="Wind E", line=dict(color="#8c564b")), row=3, col=2)
    if "wind_north_m_s" in df.columns:
        fig.add_trace(go.Scatter(x=t, y=df["wind_north_m_s"], name="Wind N truth",
                                 line=dict(color="#9467bd", dash="dot"), showlegend=True), row=3, col=2)
        fig.add_trace(go.Scatter(x=t, y=df["wind_east_m_s"], name="Wind E truth",
                                 line=dict(color="#8c564b", dash="dot"), showlegend=True), row=3, col=2)
    fig.update_yaxes(title_text="m/s", row=3, col=2)

    # ── Row 4: Angular rates & roll angle ──
    fig.add_trace(go.Scatter(x=t, y=df["omega_x_deg_s"], name="p truth",
                             line=dict(color="#d62728", width=1.5), showlegend=False), row=4, col=1)
    fig.add_trace(go.Scatter(x=t[mhe_active], y=df["mhe_p_deg_s"][mhe_active], name="p MHE",
                             line=dict(color="#d62728", dash="dash", width=1.5), showlegend=False), row=4, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["omega_y_deg_s"], name="q truth",
                             line=dict(color="#2ca02c", width=1.5), showlegend=False), row=4, col=1)
    fig.add_trace(go.Scatter(x=t[mhe_active], y=df["mhe_q_deg_s"][mhe_active], name="q MHE",
                             line=dict(color="#2ca02c", dash="dash", width=1.5), showlegend=False), row=4, col=1)
    fig.update_yaxes(title_text="deg/s", row=4, col=1)

    if "roll_deg" in df.columns:
        fig.add_trace(go.Scatter(x=t, y=df["roll_deg"], name="φ truth",
                                 line=dict(color="#1f77b4", width=2), showlegend=False), row=4, col=2)
    fig.add_trace(go.Scatter(x=t[mhe_active], y=df["mhe_phi_deg"][mhe_active], name="φ MHE",
                             line=dict(color="#ff7f0e", width=1.5, dash="dash"), showlegend=False), row=4, col=2)
    fig.update_yaxes(title_text="deg", row=4, col=2)

    # ── Row 5: Estimation errors & solver quality ──
    fig.add_trace(go.Scatter(x=t, y=df["mhe_alpha_error_deg"], name="err α",
                             line=dict(color="#d62728")), row=5, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["mhe_beta_error_deg"], name="err β",
                             line=dict(color="#2ca02c")), row=5, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["mhe_V_error_m_s"], name="err V (m/s)",
                             line=dict(color="#1f77b4")), row=5, col=1)
    fig.add_hline(y=0, line_dash="dot", line_color="black", opacity=0.4, row=5, col=1)
    fig.update_yaxes(title_text="Error (deg / m/s)", row=5, col=1)

    fig.add_trace(go.Scatter(x=t, y=df["mhe_quality"], name="Quality",
                             line=dict(color="#9c27b0", width=2), fill="tozeroy",
                             fillcolor="rgba(156,39,176,0.12)", showlegend=False), row=5, col=2)
    fig.add_hline(y=1.0, line_dash="dash", line_color="green", opacity=0.6,
                  annotation_text="Ideal", row=5, col=2)
    ax2 = go.Scatter(x=t, y=df["mhe_solve_ms"], name="Solve ms",
                     line=dict(color="#ff7f0e", width=1), yaxis="y10", showlegend=True)
    fig.add_trace(ax2, row=5, col=2)
    fig.update_yaxes(title_text="Quality / ms", row=5, col=2)

    fig.update_xaxes(title_text="Time (s)")
    fig.update_layout(
        height=330 * 5,
        template="plotly_white",
        title_text="MHE Detailed State Estimation Analysis",
        legend=dict(orientation="h", yanchor="bottom", y=1.01, xanchor="right", x=1),
    )
    return fig


def _mhe_consistency_stats(df):
    """Compute MHE estimator consistency metrics: RMSE, bias, std, dropout rate."""
    mhe_active = (df["mhe_quality"] > 0).values
    n_total = len(df)
    n_active = int(mhe_active.sum())
    n_dropout = n_total - n_active
    results = {
        "n_active":    n_active,
        "n_dropout":   n_dropout,
        "dropout_pct": n_dropout / n_total * 100 if n_total > 0 else 0.0,
        "active_pct":  n_active  / n_total * 100 if n_total > 0 else 0.0,
    }
    error_cols = {
        "alpha": "mhe_alpha_error_deg",
        "beta":  "mhe_beta_error_deg",
        "V":     "mhe_V_error_m_s",
        "gamma": "mhe_gamma_error_deg",
    }
    for key, col in error_cols.items():
        if col in df.columns:
            e = df[col].values[mhe_active]
            e = e[np.isfinite(e)]
            if len(e) > 0:
                results[f"rmse_{key}"]    = float(np.sqrt(np.mean(e**2)))
                results[f"bias_{key}"]    = float(np.mean(e))
                results[f"std_{key}"]     = float(np.std(e))
                results[f"max_abs_{key}"] = float(np.max(np.abs(e)))
            else:
                for s in ("rmse", "bias", "std", "max_abs"):
                    results[f"{s}_{key}"] = float("nan")
    # Simple consistency verdict: |bias| < 0.5 * RMSE for both alpha and V
    def _is_consistent(key):
        bias = results.get(f"bias_{key}", float("nan"))
        rmse = results.get(f"rmse_{key}", float("nan"))
        if np.isnan(rmse) or rmse < 1e-9:
            return None
        return abs(bias) < 0.5 * rmse
    ca = _is_consistent("alpha")
    cv = _is_consistent("V")
    if ca is None or cv is None:
        results["overall_consistent"] = None
    else:
        results["overall_consistent"] = ca and cv
    return results


def _fig_mhe_consistency(df):
    """MHE estimator consistency analysis with 6 subplots:
      Row 1 — Running RMSE (rolling window) | MHE Active/Dropout timeline
      Row 2 — Error histogram α+β with Gaussian fit | Error histogram V+γ
      Row 3 — Pseudo-NIS (χ² consistency test)     | α error by flight phase
    """
    from scipy import stats as _stats
    from scipy.stats import chi2 as _chi2

    t          = df["time_s"].values
    mhe_active = (df["mhe_quality"] > 0).values
    window     = max(10, len(df) // 50)

    error_cols = [
        ("mhe_alpha_error_deg", "α error",  "#d62728"),
        ("mhe_beta_error_deg",  "β error",  "#2ca02c"),
        ("mhe_V_error_m_s",     "V error",  "#1f77b4"),
        ("mhe_gamma_error_deg", "γ error",  "#9c27b0"),
    ]

    fig = make_subplots(
        rows=3, cols=2,
        subplot_titles=(
            "Running RMSE (50-step rolling window)",
            "MHE Active / Dropout Map",
            "Aerodynamic Angle Errors — Histogram + Gaussian Fit",
            "Speed & Flight Path Angle Errors — Histogram",
            "Pseudo-NIS (χ² Consistency Test)",
            "α Estimation Error by Flight Phase",
        ),
        vertical_spacing=0.10, horizontal_spacing=0.08,
        row_heights=[0.28, 0.36, 0.36],
    )

    # ── Row 1 col 1: Running RMSE ────────────────────────────────────────────
    for col, label, color in error_cols:
        if col not in df.columns:
            continue
        e = df[col].values.astype(float).copy()
        e[~mhe_active] = np.nan
        rmse_run = np.sqrt(
            pd.Series(e ** 2).rolling(window, min_periods=3).mean().values
        )
        fig.add_trace(go.Scatter(
            x=t, y=rmse_run, name=f"{label} RMSE",
            line=dict(color=color, width=1.8),
            hovertemplate=f"{label}: %{{y:.3f}}<br>t=%{{x:.2f}}s<extra></extra>",
        ), row=1, col=1)
    fig.add_hline(y=0, line_dash="dot", line_color="gray", opacity=0.3, row=1, col=1)
    fig.update_yaxes(title_text="RMSE", row=1, col=1)
    fig.update_xaxes(title_text="Time (s)", row=1, col=1)

    # ── Row 1 col 2: Dropout Map ──────────────────────────────────────────────
    quality = df["mhe_quality"].values.astype(float)
    fig.add_trace(go.Scatter(
        x=t, y=quality, fill="tozeroy",
        line=dict(color="#4caf50", width=1.5),
        fillcolor="rgba(76,175,80,0.20)",
        name="MHE Quality",
        hovertemplate="Quality=%{y:.3f} | t=%{x:.2f}s<extra></extra>",
        showlegend=False,
    ), row=1, col=2)
    # Shade dropout intervals in red
    in_dropout = False
    d_start = None
    for ti, qi in zip(t, quality):
        if not in_dropout and qi == 0:
            in_dropout = True
            d_start = ti
        elif in_dropout and qi > 0:
            fig.add_vrect(x0=d_start, x1=ti,
                          fillcolor="rgba(244,67,54,0.18)", layer="below",
                          line_width=0, row=1, col=2)
            in_dropout = False
    if in_dropout and d_start is not None:
        fig.add_vrect(x0=d_start, x1=t[-1],
                      fillcolor="rgba(244,67,54,0.18)", layer="below",
                      line_width=0, row=1, col=2)
    fig.add_hline(y=1.0, line_dash="dash", line_color="#4caf50", opacity=0.6,
                  annotation_text="Ideal=1.0", row=1, col=2)
    fig.update_yaxes(title_text="Quality", row=1, col=2)
    fig.update_xaxes(title_text="Time (s)", row=1, col=2)

    # ── Row 2: Error histograms + Gaussian fit ────────────────────────────────
    for plot_col, pairs in [
        (1, [error_cols[0], error_cols[1]]),   # α, β
        (2, [error_cols[2], error_cols[3]]),   # V, γ
    ]:
        for col, label, color in pairs:
            if col not in df.columns:
                continue
            e_raw  = df[col].values.astype(float)
            e_mask = mhe_active & np.isfinite(e_raw)
            e_clean = e_raw[e_mask]
            if len(e_clean) < 10:
                continue
            fig.add_trace(go.Histogram(
                x=e_clean, name=label,
                histnorm="probability density",
                marker_color=color, opacity=0.40, nbinsx=40,
                hovertemplate=f"{label}: %{{x:.3f}}<br>density=%{{y:.4f}}<extra></extra>",
            ), row=2, col=plot_col)
            # Gaussian PDF overlay
            mu, sigma = _stats.norm.fit(e_clean)
            x_fit = np.linspace(e_clean.min(), e_clean.max(), 300)
            y_fit = _stats.norm.pdf(x_fit, mu, sigma)
            fig.add_trace(go.Scatter(
                x=x_fit, y=y_fit,
                name=f"N({mu:+.2f}, {sigma:.2f})",
                line=dict(color=color, width=2, dash="dash"),
                hovertemplate="x=%{x:.3f}<br>PDF=%{y:.4f}<extra>Gauss fit</extra>",
            ), row=2, col=plot_col)
            # Bias line
            fig.add_vline(x=mu, line_dash="dash", line_color=color, opacity=0.65,
                          annotation_text=f"μ={mu:+.2f}", row=2, col=plot_col)
        fig.add_vline(x=0, line_dash="dot", line_color="gray", opacity=0.4, row=2, col=plot_col)
        fig.update_xaxes(title_text="Error", row=2, col=plot_col)
        fig.update_yaxes(title_text="Density", row=2, col=plot_col)

    # ── Row 3 col 1: Pseudo-NIS ───────────────────────────────────────────────
    # NIS_i = Σ (e_j / σ_j)² → should follow χ²(n) if estimator is consistent
    nis_parts = []
    n_nis = 0
    for col, _, _ in error_cols:
        if col not in df.columns:
            continue
        e = df[col].values.astype(float).copy()
        e[~mhe_active] = np.nan
        e_clean_all = e[mhe_active & np.isfinite(e)]
        if len(e_clean_all) < 5:
            continue
        sigma = np.std(e_clean_all)
        if sigma < 1e-9:
            continue
        nis_parts.append(e ** 2 / sigma ** 2)
        n_nis += 1
    if nis_parts:
        nis_raw = np.nansum(np.column_stack(nis_parts), axis=1)
        nis_raw[~mhe_active] = np.nan
        nis_smooth = pd.Series(nis_raw).rolling(window, min_periods=3).mean().values
        chi2_95 = _chi2.ppf(0.95, df=n_nis)
        chi2_05 = _chi2.ppf(0.05, df=n_nis)
        fig.add_trace(go.Scatter(
            x=t, y=nis_smooth,
            name=f"NIS (χ²({n_nis}))",
            line=dict(color="#ff7f0e", width=2),
            fill="tozeroy", fillcolor="rgba(255,127,14,0.08)",
            hovertemplate="NIS=%{y:.2f} | t=%{x:.2f}s<extra></extra>",
            showlegend=False,
        ), row=3, col=1)
        fig.add_hline(y=n_nis, line_dash="solid", line_color="#4caf50", opacity=0.7,
                      annotation_text=f"E[χ²]={n_nis} (consistent)", row=3, col=1)
        fig.add_hline(y=chi2_95, line_dash="dash", line_color="#f44336", opacity=0.6,
                      annotation_text=f"95%={chi2_95:.1f}", row=3, col=1)
        fig.add_hline(y=chi2_05, line_dash="dot", line_color="#2196f3", opacity=0.6,
                      annotation_text=f"5%={chi2_05:.1f}", row=3, col=1)
        fig.update_yaxes(title_text="NIS", row=3, col=1)
        fig.update_xaxes(title_text="Time (s)", row=3, col=1)

    # ── Row 3 col 2: α error by flight phase (boxplot) ───────────────────────
    alpha_col = "mhe_alpha_error_deg"
    if alpha_col in df.columns:
        for ph in OrderedDict.fromkeys(df["flight_phase"]):
            mask_ph = (df["flight_phase"] == ph).values & mhe_active
            e = df[alpha_col].values[mask_ph].astype(float)
            e = e[np.isfinite(e)]
            if len(e) < 3:
                continue
            fig.add_trace(go.Box(
                y=e, name=ph,
                marker_color=PHASE_COLORS.get(ph, "#333"),
                boxmean="sd",
                showlegend=False,
                hovertemplate=f"Phase={ph}<br>α err=%{{y:.2f}}°<extra></extra>",
            ), row=3, col=2)
        fig.add_hline(y=0, line_dash="dot", line_color="gray", opacity=0.4, row=3, col=2)
        fig.update_yaxes(title_text="α error (°)", row=3, col=2)

    fig.update_layout(
        height=980,
        template="plotly_white",
        title_text="MHE Estimator Consistency Analysis",
        legend=dict(orientation="h", yanchor="bottom", y=1.01,
                    xanchor="right", x=1, font=dict(size=10)),
        barmode="overlay",
    )
    return fig


def _control_allocation_stats(df):
    """Per-fin stats: RMS, max deflection, saturation rate; mixing fidelity."""
    FIN_SAT_DEG = 20.0   # fin travel limit (degrees) — adjust if different
    stats = {"fin_sat_threshold_deg": FIN_SAT_DEG}
    for j in range(1, 5):
        col = f"fin_{j}_rad"
        if col not in df.columns:
            continue
        d = np.degrees(df[col].values)
        stats[f"fin{j}_max_deg"]  = float(np.max(np.abs(d)))
        stats[f"fin{j}_rms_deg"]  = float(np.sqrt(np.mean(d**2)))
        stats[f"fin{j}_sat_pct"]  = float(np.mean(np.abs(d) > 0.8 * FIN_SAT_DEG) * 100)
    # Mixing fidelity: Pearson r between MPC virtual cmd and reconstructed virtual
    for virt, actual in [("mpc_delta_e_deg", "delta_pitch_deg"),
                          ("mpc_delta_r_deg", "delta_yaw_deg"),
                          ("mpc_delta_a_deg", "delta_roll_deg")]:
        if virt in df.columns and actual in df.columns:
            v = df[virt].values
            a = df[actual].values
            mask = np.isfinite(v) & np.isfinite(a)
            if mask.sum() > 10:
                r = float(np.corrcoef(v[mask], a[mask])[0, 1])
                key = virt.replace("mpc_delta_", "").replace("_deg", "")
                stats[f"mixing_r_{key}"] = r
    return stats


def _fig_control_allocation(df):
    """Control Allocation Analysis — 3 × 2 subplots:
      Row 1 — Fin deflection histograms (balance check)  | RMS utilization per phase
      Row 2 — Pitch mixer fidelity scatter               | Virtual commands vs reconstructed
      Row 3 — Fin saturation timeline                    | Control authority budget (stacked)
    """
    t           = df["time_s"].values
    fin_colors  = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e"]
    FIN_SAT_DEG = 20.0
    has_fins    = df.attrs.get("has_fins", False)
    has_virt    = "mpc_delta_e_deg" in df.columns and df["mpc_delta_e_deg"].abs().max() > 0.01
    has_recon   = "delta_pitch_deg" in df.columns

    n_rows = 3
    fig = make_subplots(
        rows=n_rows, cols=2,
        subplot_titles=(
            "Fin Deflection Distributions (Balance Check)",
            "RMS Utilization per Flight Phase",
            "Pitch Mixer Fidelity: MPC δe vs Actual δ_pitch",
            "Virtual Commands: MPC vs Reconstructed",
            "Fin Saturation Timeline",
            "Control Authority Budget",
        ),
        vertical_spacing=0.10, horizontal_spacing=0.10,
    )

    # ── Row 1 col 1: Fin deflection histograms ────────────────────────────────
    if has_fins:
        for j, clr in enumerate(fin_colors):
            col = f"fin_{j+1}_rad"
            if col not in df.columns:
                continue
            d = np.degrees(df[col].values)
            fig.add_trace(go.Histogram(
                x=d, name=f"Fin {j+1}",
                histnorm="probability density",
                marker_color=clr, opacity=0.45, nbinsx=50,
                hovertemplate=f"Fin {j+1}: %{{x:.2f}}°<extra></extra>",
            ), row=1, col=1)
        for lim, clr, dash in [(FIN_SAT_DEG, "#f44336", "dash"), (-FIN_SAT_DEG, "#f44336", "dash")]:
            fig.add_vline(x=lim, line_dash=dash, line_color=clr, opacity=0.6, row=1, col=1)
        fig.add_vline(x=0, line_dash="dot", line_color="gray", opacity=0.4, row=1, col=1)
    fig.update_xaxes(title_text="Deflection (°)", row=1, col=1)
    fig.update_yaxes(title_text="Density", row=1, col=1)

    # ── Row 1 col 2: RMS per phase grouped bar ────────────────────────────────
    if has_fins:
        phases_order = list(OrderedDict.fromkeys(df["flight_phase"]))
        for j, clr in enumerate(fin_colors):
            col = f"fin_{j+1}_rad"
            if col not in df.columns:
                continue
            rms_vals = []
            for ph in phases_order:
                mask = df["flight_phase"] == ph
                d = np.degrees(df[col].values[mask])
                rms_vals.append(float(np.sqrt(np.mean(d**2))) if len(d) > 0 else 0.0)
            fig.add_trace(go.Bar(
                x=phases_order, y=rms_vals, name=f"Fin {j+1}",
                marker_color=clr, opacity=0.8,
                showlegend=False,
                hovertemplate=f"Fin {j+1} RMS: %{{y:.2f}}°<br>Phase: %{{x}}<extra></extra>",
            ), row=1, col=2)
    fig.update_xaxes(title_text="Phase", row=1, col=2)
    fig.update_yaxes(title_text="RMS Deflection (°)", row=1, col=2)

    # ── Row 2 col 1: Pitch mixer fidelity scatter ─────────────────────────────
    if has_virt and has_recon:
        ve = df["mpc_delta_e_deg"].values
        dp = df["delta_pitch_deg"].values
        mask = np.isfinite(ve) & np.isfinite(dp)
        # Color by time
        fig.add_trace(go.Scatter(
            x=ve[mask], y=dp[mask], mode="markers",
            marker=dict(size=3, color=t[mask], colorscale="Viridis",
                        showscale=True,
                        colorbar=dict(title="Time (s)", x=0.48, len=0.28, y=0.57)),
            name="(δe, δ_pitch)",
            hovertemplate="MPC δe=%{x:.2f}°<br>δ_pitch=%{y:.2f}°<br>t=%{marker.color:.2f}s<extra></extra>",
            showlegend=False,
        ), row=2, col=1)
        # Identity line
        lim = max(float(np.max(np.abs(ve[mask]))), float(np.max(np.abs(dp[mask])))) * 1.05
        fig.add_trace(go.Scatter(
            x=[-lim, lim], y=[-lim, lim], mode="lines",
            line=dict(color="gray", dash="dash", width=1),
            showlegend=False,
        ), row=2, col=1)
        # Compute Pearson r for annotation
        r = float(np.corrcoef(ve[mask], dp[mask])[0, 1]) if mask.sum() > 5 else float("nan")
        fig.add_annotation(
            xref="x domain", yref="y domain", x=0.05, y=0.95,
            text=f"r = {r:.3f}", showarrow=False,
            font=dict(size=12, color="#333"),
            bgcolor="rgba(255,255,255,0.7)", row=2, col=1,
        )
    else:
        fig.add_annotation(text="MPC virtual commands not available",
                           xref="x domain", yref="y domain", x=0.5, y=0.5,
                           showarrow=False, font=dict(size=13, color="gray"), row=2, col=1)
    fig.update_xaxes(title_text="MPC δe (°)", row=2, col=1)
    fig.update_yaxes(title_text="δ_pitch actual (°)", row=2, col=1)

    # ── Row 2 col 2: Virtual vs reconstructed time series ────────────────────
    virt_pairs = [
        ("mpc_delta_e_deg", "delta_pitch_deg", "#1f77b4", "Pitch"),
        ("mpc_delta_r_deg", "delta_yaw_deg",   "#d62728", "Yaw"),
        ("mpc_delta_a_deg", "delta_roll_deg",  "#2ca02c", "Roll"),
    ]
    for virt_col, recon_col, clr, label in virt_pairs:
        if virt_col in df.columns and df[virt_col].abs().max() > 0.01:
            fig.add_trace(go.Scatter(
                x=t, y=df[virt_col], name=f"MPC {label}",
                line=dict(color=clr, width=1.8),
                hovertemplate=f"MPC {label}: %{{y:.2f}}°<extra></extra>",
            ), row=2, col=2)
        if recon_col in df.columns:
            fig.add_trace(go.Scatter(
                x=t, y=df[recon_col], name=f"Recon {label}",
                line=dict(color=clr, width=1.2, dash="dot"),
                showlegend=True,
                hovertemplate=f"Recon {label}: %{{y:.2f}}°<extra></extra>",
            ), row=2, col=2)
    fig.update_xaxes(title_text="Time (s)", row=2, col=2)
    fig.update_yaxes(title_text="δ (°)", row=2, col=2)

    # ── Row 3 col 1: Fin saturation timeline ──────────────────────────────────
    if has_fins:
        for j, clr in enumerate(fin_colors):
            col = f"fin_{j+1}_rad"
            if col not in df.columns:
                continue
            d_abs = np.abs(np.degrees(df[col].values))
            fig.add_trace(go.Scatter(
                x=t, y=d_abs, name=f"|Fin {j+1}|",
                line=dict(color=clr, width=1.2),
                showlegend=False,
                hovertemplate=f"|Fin {j+1}|=%{{y:.2f}}°<extra></extra>",
            ), row=3, col=1)
        fig.add_hline(y=FIN_SAT_DEG, line_dash="dash", line_color="#f44336", opacity=0.7,
                      annotation_text=f"Sat. limit {FIN_SAT_DEG}°", row=3, col=1)
        fig.add_hline(y=0.8 * FIN_SAT_DEG, line_dash="dot", line_color="#ff9800", opacity=0.6,
                      annotation_text="80% limit", row=3, col=1)
    fig.update_xaxes(title_text="Time (s)", row=3, col=1)
    fig.update_yaxes(title_text="|Deflection| (°)", row=3, col=1)

    # ── Row 3 col 2: Control authority budget (stacked area) ─────────────────
    budget_items = [
        ("delta_pitch_deg", "|δ_pitch|", "#1f77b4"),
        ("delta_yaw_deg",   "|δ_yaw|",   "#d62728"),
        ("delta_roll_deg",  "|δ_roll|",  "#2ca02c"),
    ]
    any_budget = False
    for col, label, clr in budget_items:
        if col not in df.columns:
            continue
        d_abs = np.abs(df[col].values).astype(float)
        fig.add_trace(go.Scatter(
            x=t, y=d_abs,
            name=label, mode="none",
            stackgroup="budget",
            fillcolor=clr,
            hovertemplate=f"{label}: %{{y:.2f}}°<extra></extra>",
        ), row=3, col=2)
        any_budget = True
    if any_budget:
        fig.add_hline(y=FIN_SAT_DEG, line_dash="dash", line_color="#f44336", opacity=0.5,
                      annotation_text=f"Max authority {FIN_SAT_DEG}°", row=3, col=2)
    fig.update_xaxes(title_text="Time (s)", row=3, col=2)
    fig.update_yaxes(title_text="Authority used (°)", row=3, col=2)

    fig.update_layout(
        height=1000,
        template="plotly_white",
        title_text="Control Allocation Analysis",
        legend=dict(orientation="h", yanchor="bottom", y=1.01,
                    xanchor="right", x=1, font=dict(size=10)),
        barmode="group",
    )
    return fig


def _fig_geo_track(df):
    """Geographic track: lat/lon ground track and LLA altitude."""
    t = df["time_s"]
    fig = make_subplots(rows=1, cols=2,
                        subplot_titles=("Ground Track (Lat/Lon)", "Altitude (LLA) vs Time"),
                        horizontal_spacing=0.08)
    for ph in OrderedDict.fromkeys(df["flight_phase"]):
        mask = df["flight_phase"] == ph
        c = PHASE_COLORS.get(ph, "#333")
        fig.add_trace(go.Scatter(x=df["longitude_deg"][mask], y=df["latitude_deg"][mask],
                                 mode="lines", name=ph, line=dict(color=c, width=2),
                                 legendgroup=ph), row=1, col=1)
        fig.add_trace(go.Scatter(x=t[mask], y=df["altitude_lla_m"][mask],
                                 mode="lines", line=dict(color=c, width=2),
                                 legendgroup=ph, showlegend=False), row=1, col=2)
    # Mark launch and impact
    fig.add_trace(go.Scatter(x=[df["longitude_deg"].iloc[0]], y=[df["latitude_deg"].iloc[0]],
                             mode="markers", marker=dict(symbol="star", size=12, color="green"),
                             name="Launch"), row=1, col=1)
    fig.add_trace(go.Scatter(x=[df["longitude_deg"].iloc[-1]], y=[df["latitude_deg"].iloc[-1]],
                             mode="markers", marker=dict(symbol="x", size=12, color="red"),
                             name="Impact"), row=1, col=1)
    fig.update_xaxes(title_text="Longitude (°)", row=1, col=1); fig.update_xaxes(title_text="Time (s)", row=1, col=2)
    fig.update_yaxes(title_text="Latitude (°)", row=1, col=1); fig.update_yaxes(title_text="Altitude (m)", row=1, col=2)
    fig.update_layout(height=450, template="plotly_white",
                      yaxis1=dict(scaleanchor="x1"))
    return fig


# ─── Multi-Run Charts ────────────────────────────────────────────────────────

def _fig_multi_scatter(mdf):
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Range Error vs Flight Time", "Impact Speed vs Impact γ",
                                        "Range Error Distribution", "Impact Speed Distribution"),
                        vertical_spacing=0.14, horizontal_spacing=0.08)
    colors = ["green" if abs(e) < 50 else ("orange" if abs(e) < 150 else "red") for e in mdf["range_error_m"]]
    fig.add_trace(go.Scatter(x=mdf["flight_time_s"], y=mdf["range_error_m"], mode="markers",
                             marker=dict(color=colors, size=7, line=dict(width=0.5, color="black")),
                             text=mdf["timestamp"], name="Runs"), row=1, col=1)
    fig.add_hline(y=0, line_dash="dot", opacity=0.4, row=1, col=1)
    fig.add_trace(go.Scatter(x=mdf["impact_speed_mps"], y=mdf["impact_gamma_deg"], mode="markers",
                             marker=dict(color=colors, size=7, line=dict(width=0.5, color="black")),
                             text=mdf["timestamp"], showlegend=False), row=1, col=2)
    fig.add_trace(go.Histogram(x=mdf["range_error_m"], nbinsx=30, marker_color="steelblue", showlegend=False), row=2, col=1)
    fig.add_vline(x=0, line_dash="dash", line_color="red", row=2, col=1)
    fig.add_trace(go.Histogram(x=mdf["impact_speed_mps"], nbinsx=30, marker_color="salmon", showlegend=False), row=2, col=2)
    fig.update_xaxes(title_text="Flight Time (s)", row=1, col=1); fig.update_yaxes(title_text="Range Error (m)", row=1, col=1)
    fig.update_xaxes(title_text="Impact Speed (m/s)", row=1, col=2); fig.update_yaxes(title_text="Impact γ (deg)", row=1, col=2)
    fig.update_xaxes(title_text="Range Error (m)", row=2, col=1); fig.update_xaxes(title_text="Impact Speed (m/s)", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white")
    return fig

def _fig_multi_trends(mdf):
    mdf_s = mdf.sort_values("timestamp").reset_index(drop=True)
    run_idx = np.arange(len(mdf_s))
    fig = make_subplots(rows=2, cols=3,
                        subplot_titles=("Range Error", "Impact Speed", "Peak Alt",
                                        "Max |α|", "Pitch σ (last 30%)", "Max G"),
                        vertical_spacing=0.14, horizontal_spacing=0.06)
    cols_data = [("range_error_m","#4363d8"),("impact_speed_mps","#e6194b"),("peak_alt_agl_m","#3cb44b"),
                 ("max_alpha_deg","#f58231"),("pitch_std_last30pct","#911eb4"),("max_g","#dc143c")]
    for i, (col, clr) in enumerate(cols_data):
        r, c = divmod(i, 3)
        vals = mdf_s[col].values
        fig.add_trace(go.Scatter(x=run_idx, y=vals, mode="markers+lines",
                                 marker=dict(size=4, color=clr), line=dict(width=0.8, color=clr),
                                 showlegend=False), row=r+1, col=c+1)
        if len(vals) >= 5:
            w = min(10, len(vals)//2)
            roll = pd.Series(vals).rolling(w, min_periods=1).mean()
            fig.add_trace(go.Scatter(x=run_idx, y=roll, mode="lines",
                                     line=dict(width=2, color="black", dash="dash"),
                                     name=f"Avg({w})", showlegend=(i==0)), row=r+1, col=c+1)
    fig.update_xaxes(title_text="Run #")
    fig.update_layout(height=600, template="plotly_white")
    return fig

def _fig_multi_overlay(all_dfs, max_n=30):
    n = min(len(all_dfs), max_n)
    fig = make_subplots(rows=1, cols=2, subplot_titles=("Altitude Profiles", "Range vs Altitude"), horizontal_spacing=0.08)
    cscale = [f"hsl({int(h)},70%,50%)" for h in np.linspace(0, 300, n)]
    for i, df in enumerate(all_dfs[:n]):
        fig.add_trace(go.Scatter(x=df["time_s"], y=df["alt_agl_m"], mode="lines",
                                 line=dict(width=1, color=cscale[i]), opacity=0.6, showlegend=False), row=1, col=1)
        fig.add_trace(go.Scatter(x=df["ground_range_m"], y=df["alt_agl_m"], mode="lines",
                                 line=dict(width=1, color=cscale[i]), opacity=0.6, showlegend=False), row=1, col=2)
    fig.add_vline(x=TARGET_RANGE_M, line_dash="dash", line_color="red", opacity=0.4, row=1, col=2)
    fig.update_xaxes(title_text="Time (s)", row=1, col=1); fig.update_xaxes(title_text="Range (m)", row=1, col=2)
    fig.update_yaxes(title_text="Alt AGL (m)")
    fig.update_layout(height=400, template="plotly_white")
    return fig

def _fig_correlation(mdf):
    cols = ["range_error_m","impact_speed_mps","impact_gamma_deg","peak_alt_agl_m",
            "max_mach","max_g","max_alpha_deg","pitch_std_last30pct","flight_time_s"]
    cols = [c for c in cols if c in mdf.columns]
    corr = mdf[cols].corr()
    fig = go.Figure(data=go.Heatmap(z=corr.values, x=cols, y=cols, colorscale="RdBu_r",
                                     zmin=-1, zmax=1, text=np.round(corr.values, 2), texttemplate="%{text}"))
    fig.update_layout(height=550, width=700, template="plotly_white", title="Metric Correlation")
    return fig


def _compute_error_ellipse(range_errors, cross_errors, confidence=0.50):
    """Return (x, y) boundary of the confidence-level error ellipse via covariance eigendecomposition."""
    from scipy.stats import chi2 as _chi2
    data = np.column_stack([range_errors, cross_errors])
    mean = data.mean(axis=0)
    cov = np.cov(data.T)
    scale = np.sqrt(_chi2.ppf(confidence, df=2))
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    order = eigenvalues.argsort()[::-1]
    eigenvalues = eigenvalues[order]
    eigenvectors = eigenvectors[:, order]
    theta = np.linspace(0, 2.0 * np.pi, 200)
    unit = np.column_stack([np.cos(theta), np.sin(theta)])
    scaled = unit * (np.sqrt(np.maximum(eigenvalues, 0)) * scale)
    rotated = (eigenvectors @ scaled.T).T + mean
    return rotated[:, 0], rotated[:, 1]


def _fig_dispersion_2d(mdf):
    """2D impact dispersion scatter with 50%/90% CEP radial circles and covariance ellipses."""
    range_err = mdf["range_error_m"].values
    cross_err = (mdf["cross_range_error_m"].fillna(0).values
                 if "cross_range_error_m" in mdf.columns else np.zeros(len(mdf)))
    dist_2d = np.sqrt(range_err**2 + cross_err**2)
    cep_50  = float(np.percentile(dist_2d, 50))
    cep_90  = float(np.percentile(dist_2d, 90))
    pt_colors = ["#4caf50" if d < 50 else ("#ff9800" if d < 150 else "#f44336") for d in dist_2d]
    labels = mdf["timestamp"].values if "timestamp" in mdf.columns else [str(i) for i in range(len(mdf))]

    fig = go.Figure()

    # Impact scatter
    fig.add_trace(go.Scatter(
        x=range_err, y=cross_err, mode="markers",
        marker=dict(color=pt_colors, size=7, line=dict(width=0.5, color="rgba(0,0,0,0.35)")),
        text=labels, name="Impact Points",
        hovertemplate="Range err: %{x:.0f} m<br>Cross err: %{y:.0f} m<br>Run: %{text}<extra></extra>",
    ))

    # Target crosshair
    fig.add_trace(go.Scatter(
        x=[0], y=[0], mode="markers",
        marker=dict(symbol="x", size=16, color="red", line=dict(width=3, color="red")),
        name="Target (0, 0)",
    ))

    # Radial CEP circles
    theta_c = np.linspace(0, 2 * np.pi, 300)
    for r, color, dash, label in [
        (cep_50, "#2196f3", "solid", f"CEP₅₀ = {cep_50:.0f} m"),
        (cep_90, "#f57c00", "dash",  f"CEP₉₀ = {cep_90:.0f} m"),
    ]:
        fig.add_trace(go.Scatter(
            x=r * np.cos(theta_c), y=r * np.sin(theta_c),
            mode="lines", line=dict(color=color, width=1.5, dash=dash),
            name=label,
        ))

    # Covariance ellipses
    if len(mdf) >= 4:
        try:
            for conf, color, dash, label in [
                (0.50, "#1565c0", "solid", "50% Ellipse"),
                (0.90, "#b71c1c", "dash",  "90% Ellipse"),
            ]:
                ex, ey = _compute_error_ellipse(range_err, cross_err, confidence=conf)
                fig.add_trace(go.Scatter(
                    x=np.append(ex, ex[0]), y=np.append(ey, ey[0]),
                    mode="lines", line=dict(color=color, width=2.5, dash=dash),
                    name=label,
                ))
        except Exception:
            pass

    max_ax = max(float(np.max(np.abs(range_err))), float(np.max(np.abs(cross_err))), cep_90, 1.0) * 1.3
    fig.update_layout(
        height=580, template="plotly_white",
        title=f"2D Impact Dispersion  —  CEP₅₀ = {cep_50:.0f} m  |  CEP₉₀ = {cep_90:.0f} m  |  n = {len(mdf)} runs",
        xaxis=dict(title="Range Error (m)  [+ = Overshoot]",
                   zeroline=True, zerolinecolor="#9e9e9e", zerolinewidth=1,
                   range=[-max_ax, max_ax]),
        yaxis=dict(title="Cross-Range Error (m)  [+ = Right of target line]",
                   zeroline=True, zerolinecolor="#9e9e9e", zerolinewidth=1,
                   scaleanchor="x", range=[-max_ax, max_ax]),
        legend=dict(orientation="h", yanchor="bottom", y=1.02),
    )
    return fig


# ─── HTML Builder ────────────────────────────────────────────────────────────

_CSS = """
:root{--pass:#4caf50;--warn:#ff9800;--fail:#f44336;--bg:#fafafa;--card:#fff;--border:#e0e0e0;--text:#212121;--text-secondary:#666;--text-muted:#999;--accent:#1976d2;--hover:#f0f7ff;--th-bg:#f5f5f5;--diag-error-bg:#ffebee;--diag-warn-bg:#fff3e0;--diag-info-bg:#e3f2fd;--rec-bg:#e8f5e9;--bar-track:#eee}
[data-theme="dark"]{--bg:#121212;--card:#1e1e1e;--border:#333;--text:#e0e0e0;--text-secondary:#aaa;--text-muted:#777;--accent:#64b5f6;--hover:#263238;--th-bg:#2a2a2a;--diag-error-bg:#2c1518;--diag-warn-bg:#2c2415;--diag-info-bg:#152535;--rec-bg:#152c18;--bar-track:#333}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,-apple-system,sans-serif;background:var(--bg);color:var(--text);line-height:1.6;padding:20px;transition:background .3s,color .3s}
.container{max-width:1400px;margin:0 auto}
h1{font-size:1.8rem;border-bottom:3px solid var(--accent);padding-bottom:8px;margin-bottom:16px}
h2{font-size:1.3rem;color:var(--accent);margin:24px 0 12px;border-left:4px solid var(--accent);padding-left:10px}
.grid{display:grid;gap:16px}.grid-2{grid-template-columns:1fr 1fr}.grid-3{grid-template-columns:1fr 1fr 1fr}.grid-4{grid-template-columns:repeat(4,1fr)}
.card{background:var(--card);border:1px solid var(--border);border-radius:8px;padding:16px;box-shadow:0 1px 3px rgba(0,0,0,.08);transition:background .3s,border-color .3s}
.score-ring{width:120px;height:120px;border-radius:50%;display:flex;align-items:center;justify-content:center;font-size:2rem;font-weight:700;margin:0 auto 8px;border:6px solid}
.score-ring.pass{border-color:var(--pass);color:var(--pass)}.score-ring.warn{border-color:var(--warn);color:var(--warn)}.score-ring.fail{border-color:var(--fail);color:var(--fail)}
.badge{display:inline-block;padding:2px 10px;border-radius:12px;font-size:.75rem;font-weight:700;color:#fff;text-transform:uppercase}
.badge.pass{background:var(--pass)}.badge.warn{background:var(--warn)}.badge.fail{background:var(--fail)}.badge.info{background:#2196f3}
.metric-box{text-align:center;padding:12px}.metric-box .value{font-size:1.6rem;font-weight:700;color:var(--accent)}.metric-box .label{font-size:.75rem;color:var(--text-secondary);text-transform:uppercase;letter-spacing:.5px}.metric-box .sub{font-size:.7rem;color:var(--text-muted)}
table{width:100%;border-collapse:collapse;font-size:.85rem}th{background:var(--th-bg);padding:8px 12px;text-align:left;border-bottom:2px solid var(--border);font-weight:600}td{padding:6px 12px;border-bottom:1px solid var(--border)}tr:hover{background:var(--hover)}
.diag{padding:10px 14px;border-radius:6px;margin-bottom:8px;border-left:4px solid}
.diag.error{background:var(--diag-error-bg);border-color:var(--fail)}.diag.warning{background:var(--diag-warn-bg);border-color:var(--warn)}.diag.info{background:var(--diag-info-bg);border-color:#2196f3}
.diag .dtitle{font-weight:700;font-size:.9rem}.diag .ddetail{font-size:.8rem;color:var(--text-secondary);margin-top:2px}
.rec{padding:8px 14px;background:var(--rec-bg);border-radius:6px;margin-bottom:6px;font-size:.85rem;border-left:3px solid var(--pass)}
.phase-bar{display:flex;height:32px;border-radius:6px;overflow:hidden;margin:8px 0}
.phase-seg{display:flex;align-items:center;justify-content:center;color:#fff;font-size:.7rem;font-weight:700;min-width:30px}
.chart-container{margin:12px 0}
.tabs{display:flex;gap:4px;border-bottom:2px solid var(--border);flex-wrap:wrap}.tab-btn{padding:8px 20px;border:none;background:none;cursor:pointer;font-size:.9rem;font-weight:600;border-bottom:3px solid transparent;color:var(--text-secondary);transition:.2s}.tab-btn:hover{color:var(--accent)}.tab-btn.active{color:var(--accent);border-bottom-color:var(--accent)}
.tab-panel{display:none;padding:16px 0}.tab-panel.active{display:block}
.toolbar{display:flex;align-items:center;gap:12px;margin-bottom:16px;flex-wrap:wrap}
.theme-toggle{background:var(--card);border:1px solid var(--border);border-radius:20px;padding:6px 14px;cursor:pointer;font-size:.85rem;color:var(--text);transition:.2s;display:flex;align-items:center;gap:6px}.theme-toggle:hover{border-color:var(--accent)}
.phase-filter{background:var(--card);border:1px solid var(--border);border-radius:6px;padding:6px 12px;font-size:.85rem;color:var(--text);cursor:pointer}
.phase-filter option{background:var(--card);color:var(--text)}
@media(max-width:900px){.grid-2,.grid-3,.grid-4{grid-template-columns:1fr}}
"""

_JS = """
function openTab(evt,tabId){
  document.querySelectorAll('.tab-panel').forEach(p=>p.classList.remove('active'));
  document.querySelectorAll('.tab-btn').forEach(b=>b.classList.remove('active'));
  document.getElementById(tabId).classList.add('active');
  evt.currentTarget.classList.add('active');
  var el=document.getElementById(tabId);
  el.querySelectorAll('.js-plotly-plot').forEach(function(p){Plotly.Plots.resize(p);});
  // re-apply phase filter after tab switch
  if(window._activePhaseFilter&&window._activePhaseFilter!=='all')
    setTimeout(function(){_applyPhaseFilter(window._activePhaseFilter);},200);
}

// ── Dark Mode ──────────────────────────────────────────────────────────────
(function(){
  var saved=localStorage.getItem('m130_theme')||'light';
  document.documentElement.setAttribute('data-theme',saved);
  window.addEventListener('DOMContentLoaded',function(){
    var btn=document.getElementById('theme-toggle');
    if(!btn)return;
    _updateToggleLabel(btn,saved);
    btn.addEventListener('click',function(){
      var cur=document.documentElement.getAttribute('data-theme')||'light';
      var next=cur==='dark'?'light':'dark';
      document.documentElement.setAttribute('data-theme',next);
      localStorage.setItem('m130_theme',next);
      _updateToggleLabel(btn,next);
      var bg=next==='dark'?'#1e1e1e':'#fff';
      var fg=next==='dark'?'#e0e0e0':'#444';
      document.querySelectorAll('.js-plotly-plot').forEach(function(p){
        Plotly.relayout(p,{'paper_bgcolor':bg,'plot_bgcolor':bg,'font.color':fg});
      });
    });
  });
  function _updateToggleLabel(btn,theme){btn.textContent=theme==='dark'?'\u2600 Light Mode':'\u263e Dark Mode';}
})();

// ── Phase Filter ──────────────────────────────────────────────────────────
function filterPhase(sel){
  window._activePhaseFilter=sel.value;
  _applyPhaseFilter(sel.value);
}
function _applyPhaseFilter(phase){
  document.querySelectorAll('.js-plotly-plot').forEach(function(p){
    var data=p.data;
    if(!data||!data.length)return;
    var hasPhase=data.some(function(t){return t.customdata&&t.customdata.length&&typeof t.customdata[0]==='string';});
    if(!hasPhase)return;
    var update={visible:data.map(function(t){
      if(!t.customdata||!t.customdata.length)return true;
      if(phase==='all')return true;
      return t.customdata.some(function(cd){return Array.isArray(cd)?cd[0]===phase:cd===phase;});
    })};
    Plotly.restyle(p,update);
  });
}
window.addEventListener('DOMContentLoaded',function(){window._activePhaseFilter='all';});
"""

def _metric_card(label, value, sub="", color="var(--accent)"):
    return f'<div class="metric-box"><div class="value" style="color:{color}">{value}</div><div class="label">{label}</div><div class="sub">{sub}</div></div>'

def _badge(verdict):
    return f'<span class="badge {verdict.lower()}">{verdict}</span>'

def _score_html(scores):
    total = scores["_total"]
    ring_cls = scores["_overall"].lower()
    cats = {"range":"Range Accuracy","impact_angle":"Impact Angle","stability":"Stability","aoa":"AoA Margin","sideslip":"Sideslip","g_load":"G-Load"}
    cat_max = {"range":40,"impact_angle":15,"stability":15,"aoa":10,"sideslip":10,"g_load":10}
    rows = ""
    for key, label in cats.items():
        c = scores[key]
        pct = c["score"] / cat_max[key] * 100
        bar_color = {"PASS":"var(--pass)","WARN":"var(--warn)","FAIL":"var(--fail)"}[c["verdict"]]
        rows += f'<tr><td>{label}</td><td>{_badge(c["verdict"])}</td><td><div style="background:var(--bar-track);border-radius:4px;height:14px"><div style="background:{bar_color};height:100%;width:{pct:.0f}%;border-radius:4px"></div></div></td><td style="text-align:right;font-weight:600">{c["score"]}/{cat_max[key]}</td><td style="font-size:.8rem;color:var(--text-secondary)">{c["detail"]}</td></tr>'
    return f'<div class="card" style="text-align:center"><div class="score-ring {ring_cls}">{total:.0f}</div><div style="font-size:1.1rem;font-weight:700;margin-bottom:4px">{_badge(scores["_overall"])} Overall Score</div><table style="margin-top:12px"><tr><th>Category</th><th>Status</th><th style="width:120px">Score</th><th></th><th>Detail</th></tr>{rows}</table></div>'

def _diag_html(diags):
    return "".join(f'<div class="diag {d["level"]}"><div class="dtitle">{html_escape(d["title"])}</div><div class="ddetail">{html_escape(d["detail"])}</div></div>' for d in diags)

def _rec_html(recs):
    return "".join(f'<div class="rec">{html_escape(r)}</div>' for r in recs)

def _phase_bar_html(phases, total_time):
    h = '<div class="phase-bar">'
    for p in phases:
        pct = max(2, p["duration"] / total_time * 100) if total_time > 0 else 10
        h += f'<div class="phase-seg" style="width:{pct:.1f}%;background:{p["color"]}" title="{p["name"]}: {p["duration"]:.2f}s">{p["name"]}<br>{p["duration"]:.1f}s</div>'
    return h + '</div>'

def _phase_table_html(phases):
    h = '<table><tr><th>Phase</th><th>Duration</th><th>Alt Start→End</th><th>Speed Start→End</th><th>Range Start→End</th><th>Max |α|</th><th>Max G</th><th>Max q</th></tr>'
    for p in phases:
        h += f'<tr><td><span style="color:{p["color"]};font-weight:700">■ {p["name"]}</span></td><td>{p["duration"]:.2f}s</td><td>{p["alt_start"]:.0f}→{p["alt_end"]:.0f}m</td><td>{p["speed_start"]:.0f}→{p["speed_end"]:.0f} m/s</td><td>{p["range_start"]:.0f}→{p["range_end"]:.0f}m</td><td>{p["max_alpha"]:.1f}°</td><td>{p["max_g"]:.1f}</td><td>{p["max_q"]/1000:.1f} kPa</td></tr>'
    return h + '</table>'

def _plotly_div(fig, div_id=None):
    config = {"responsive": True, "displayModeBar": True, "modeBarButtonsToRemove": ["lasso2d", "select2d"]}
    html = pio.to_html(fig, full_html=False, include_plotlyjs=False, config=config, div_id=div_id)
    return f'<div class="chart-container">{html}</div>'

# ─── Numerical Metrics Table ────────────────────────────────────────────────
# Self-contained renderer that converts the metrics dict into a categorized,
# searchable HTML table. Keeps every numeric value visible in addition to the
# Plotly charts. Group keys by prefix; auto-infer units from key suffix.

_UNIT_SUFFIXES = (
    ("_mps2","m/s²"), ("_deg_s","°/s"), ("_deg","°"), ("_rad_s","rad/s"),
    ("_rad","rad"), ("_mps","m/s"), ("_ms","ms"), ("_us","µs"), ("_ns","ns"),
    ("_hz","Hz"), ("_khz","kHz"), ("_kpa","kPa"), ("_pa","Pa"),
    ("_nm","N·m"), ("_kg","kg"), ("_kpa","kPa"), ("_pct","%"),
    ("_percent","%"), ("_m2","m²"), ("_m3","m³"), ("_m","m"), ("_s","s"),
    ("_n","N"), ("_g","g"), ("_count",""), ("_ratio",""),
)

_CATEGORIES = (
    ("file","Run Info"), ("timestamp","Run Info"),
    ("impact_","Impact"), ("range_","Impact"), ("flight_time","Impact"),
    ("flight_dur","Impact"),
    ("peak_","Trajectory"), ("apogee","Trajectory"), ("max_alt","Trajectory"),
    ("ground_range","Trajectory"),
    ("max_mach","Aerodynamics"), ("max_q","Aerodynamics"),
    ("max_alpha","Aerodynamics"), ("max_beta","Aerodynamics"),
    ("max_g","Loads"), ("max_omega","Loads"), ("max_roll","Loads"),
    ("max_speed","Loads"),
    ("pitch_","Stability"), ("yaw_","Stability"), ("tracking_","Stability"),
    ("mpc_","MPC"), ("mhe_","MHE"), ("ekf2_","EKF2"),
    ("sensor_","Sensors"), ("servo_","Servos"),
    ("can_","CAN Bus"), ("fb_","Feedback"), ("cmd_","Commands"),
    ("latency_","Latency"), ("delay_","Latency"),
    ("dt_","Timing"), ("solve_","Timing"), ("cycle_","Timing"),
    ("temp","Thermal"), ("thermal","Thermal"),
)

def _infer_unit(key):
    k = key.lower()
    for sfx, u in _UNIT_SUFFIXES:
        if k.endswith(sfx):
            return u
    return ""

def _category_for(key):
    k = key.lower()
    for prefix, cat in _CATEGORIES:
        if k.startswith(prefix) or k == prefix.rstrip("_"):
            return cat
    return "Other"

def _fmt_value(v):
    if v is None:
        return "—"
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (int, np.integer)):
        return f"{int(v):,}"
    if isinstance(v, (float, np.floating)):
        if not np.isfinite(v):
            return "NaN" if np.isnan(v) else ("+∞" if v > 0 else "−∞")
        av = abs(v)
        if av == 0:
            return "0"
        if av >= 1e6 or av < 1e-3:
            return f"{v:.3e}"
        if av >= 100:
            return f"{v:.1f}"
        if av >= 1:
            return f"{v:.3f}"
        return f"{v:.4f}"
    if isinstance(v, (list, tuple, np.ndarray)):
        arr = list(v)
        if len(arr) > 6:
            head = ", ".join(_fmt_value(x) for x in arr[:6])
            return f"[{head}, … ×{len(arr)-6} more]"
        return "[" + ", ".join(_fmt_value(x) for x in arr) + "]"
    if isinstance(v, dict):
        kvs = ", ".join(f"{k}={_fmt_value(val)}" for k, val in list(v.items())[:4])
        more = f" … +{len(v)-4}" if len(v) > 4 else ""
        return f"{{{kvs}{more}}}"
    return html_escape(str(v))

def _metrics_table_html(m, scores=None, title="All Numerical Metrics"):
    """Render the metrics dict as one categorized, searchable HTML table."""
    # Group
    groups = OrderedDict()
    for k, v in m.items():
        if k.startswith("_"):
            continue
        cat = _category_for(k)
        groups.setdefault(cat, []).append((k, v))
    # Preferred display order
    order = ["Run Info", "Impact", "Trajectory", "Aerodynamics", "Loads",
             "Stability", "MPC", "MHE", "EKF2", "Sensors", "Servos",
             "CAN Bus", "Feedback", "Commands", "Latency", "Timing",
             "Thermal", "Other"]
    ordered_groups = OrderedDict((c, groups[c]) for c in order if c in groups)
    for c in groups:
        if c not in ordered_groups:
            ordered_groups[c] = groups[c]

    # Score summary table (if provided)
    score_rows = ""
    if scores:
        for k, v in scores.items():
            if k.startswith("_"):
                continue
            score_rows += (
                f'<tr><td><b>{html_escape(k)}</b></td>'
                f'<td><span class="badge {v["verdict"].lower()}">{v["verdict"]}</span></td>'
                f'<td style="text-align:right">{v["score"]:.1f}</td>'
                f'<td style="font-size:.85rem;color:var(--text-secondary,#666)">{html_escape(str(v.get("detail","")))}</td></tr>'
            )
        if "_total" in scores:
            score_rows += (
                f'<tr style="background:var(--surface-2,#f5f5f5);font-weight:700">'
                f'<td>TOTAL</td>'
                f'<td><span class="badge {scores["_overall"].lower()}">{scores["_overall"]}</span></td>'
                f'<td style="text-align:right">{scores["_total"]:.1f}</td><td>/100</td></tr>'
            )
    score_card = ""
    if score_rows:
        score_card = (
            '<div class="card" style="margin-bottom:16px">'
            '<h3 style="margin-bottom:8px">Score Breakdown</h3>'
            '<table><tr><th>Category</th><th>Verdict</th><th style="text-align:right">Score</th><th>Detail</th></tr>'
            f'{score_rows}</table></div>'
        )

    # Searchable metrics table
    rows_html = ""
    for cat, items in ordered_groups.items():
        rows_html += (
            f'<tr style="background:var(--surface-2,#fafafa)"><td colspan="3" '
            f'style="font-weight:700;padding:8px 6px;border-top:2px solid var(--border,#ddd)">'
            f'■ {html_escape(cat)} '
            f'<span style="color:var(--text-secondary,#888);font-weight:400;font-size:.85rem">'
            f'({len(items)})</span></td></tr>'
        )
        for k, v in sorted(items, key=lambda kv: kv[0]):
            unit = _infer_unit(k)
            rows_html += (
                f'<tr class="metric-row">'
                f'<td style="font-family:ui-monospace,monospace;font-size:.85rem">{html_escape(k)}</td>'
                f'<td style="text-align:right;font-variant-numeric:tabular-nums;font-weight:600">{_fmt_value(v)}</td>'
                f'<td style="color:var(--text-secondary,#666);font-size:.85rem">{html_escape(unit)}</td>'
                f'</tr>'
            )
    table_html = (
        '<div class="card">'
        f'<h3 style="margin-bottom:8px">{html_escape(title)} '
        f'<span style="color:var(--text-secondary,#888);font-weight:400;font-size:.85rem">'
        f'({sum(len(v) for v in ordered_groups.values())} keys, {len(ordered_groups)} groups)</span></h3>'
        '<input type="text" placeholder="🔎 Filter metric name…" '
        'oninput="(function(e){const q=e.target.value.toLowerCase();'
        'document.querySelectorAll(\'#tab-numbers .metric-row\').forEach(r=>{'
        'r.style.display=r.children[0].textContent.toLowerCase().includes(q)?\'\':\'none\';});})(event)" '
        'style="width:100%;padding:8px;margin-bottom:12px;border:1px solid var(--border,#ddd);border-radius:4px">'
        '<table style="width:100%"><thead><tr>'
        '<th style="text-align:left">Metric</th>'
        '<th style="text-align:right">Value</th>'
        '<th style="text-align:left">Unit</th>'
        '</tr></thead><tbody>'
        f'{rows_html}</tbody></table></div>'
    )
    return score_card + table_html

def _multi_run_stats_html(mdf):
    n = len(mdf)
    def _row(label, col, unit="", fmt=".1f"):
        if col not in mdf.columns: return ""
        v = mdf[col].dropna()
        if len(v) == 0: return ""
        return f"<tr><td>{label}</td><td>{v.mean():{fmt}} ± {v.std():{fmt}} {unit}</td><td>{v.min():{fmt}}</td><td>{v.max():{fmt}}</td><td>{v.median():{fmt}}</td></tr>"
    cross_err_s = mdf["cross_range_error_m"].fillna(0) if "cross_range_error_m" in mdf.columns else pd.Series(np.zeros(n))
    dist_2d = np.sqrt(mdf["range_error_m"]**2 + cross_err_s**2)
    cep_50 = float(np.percentile(dist_2d, 50))
    cep_90 = float(np.percentile(dist_2d, 90))
    good = (dist_2d < 50).sum()
    ok   = ((dist_2d >= 50) & (dist_2d < 150)).sum()
    bad  = (dist_2d >= 150).sum()
    cls_html = f'<div class="grid grid-3" style="margin-bottom:16px">{_metric_card("≤ 50 m (GOOD)", f"{good} ({good/n*100:.0f}%)", "2D dist ≤ 50 m", "#4caf50")}{_metric_card("50–150 m (WARN)", f"{ok} ({ok/n*100:.0f}%)", "2D dist 50–150 m", "#ff9800")}{_metric_card("> 150 m (FAIL)", f"{bad} ({bad/n*100:.0f}%)", "2D dist > 150 m", "#f44336")}</div>'
    tbl = f'<table><tr><th>Metric</th><th>Mean ± σ</th><th>Min</th><th>Max</th><th>Median</th></tr>{_row("Range Error","range_error_m","m")}{_row("Impact Range","impact_range_m","m")}{_row("Impact Speed","impact_speed_mps","m/s")}{_row("Impact γ","impact_gamma_deg","°")}{_row("Flight Time","flight_time_s","s",".2f")}{_row("Peak Alt AGL","peak_alt_agl_m","m")}{_row("Max Mach","max_mach","",".3f")}{_row("Max G","max_g","g")}{_row("Max |α|","max_alpha_deg","°")}{_row("Pitch σ(30%)","pitch_std_last30pct","°",".2f")}</table>'
    sorted_by = mdf.sort_values("range_error_m", key=abs)
    best5 = '<table><tr><th>#</th><th>Timestamp</th><th>Range Error</th><th>Speed</th><th>γ</th></tr>'
    for i, (_, r) in enumerate(sorted_by.head(5).iterrows()):
        best5 += f'<tr><td>{i+1}</td><td>{r["timestamp"]}</td><td>{r["range_error_m"]:+.0f}m</td><td>{r["impact_speed_mps"]:.0f}</td><td>{r["impact_gamma_deg"]:.1f}°</td></tr>'
    best5 += '</table>'
    worst5 = '<table><tr><th>#</th><th>Timestamp</th><th>Range Error</th><th>Speed</th><th>γ</th></tr>'
    for i, (_, r) in enumerate(sorted_by.tail(5).iterrows()):
        worst5 += f'<tr><td>{i+1}</td><td>{r["timestamp"]}</td><td>{r["range_error_m"]:+.0f}m</td><td>{r["impact_speed_mps"]:.0f}</td><td>{r["impact_gamma_deg"]:.1f}°</td></tr>'
    worst5 += '</table>'
    cep_cards = (f'<div class="grid grid-3" style="margin-bottom:12px">'
                 f'<div class="card">{_metric_card("CEP₅₀ (2D)", f"{cep_50:.0f} m", "50% hits within this radius", "#1565c0")}</div>'
                 f'<div class="card">{_metric_card("CEP₉₀ (2D)", f"{cep_90:.0f} m", "90% hits within this radius", "#0288d1")}</div>'
                 f'<div class="card">{_metric_card("Max Miss (2D)", f"{float(dist_2d.max()):.0f} m", "Worst single shot", "#f44336")}</div>'
                 f'</div>')
    return f'{cls_html}{cep_cards}<div class="grid grid-2"><div class="card"><h3 style="margin-bottom:8px">Summary Statistics ({n} runs)</h3>{tbl}</div></div><div class="grid grid-2" style="margin-top:16px"><div class="card"><h3 style="margin-bottom:8px">Best 5</h3>{best5}</div><div class="card"><h3 style="margin-bottom:8px">Worst 5</h3>{worst5}</div></div>'


def generate_html_report(df, metrics, scores, diags, recs, phases,
                         all_metrics=None, all_dfs=None, html_path=None):
    fig_traj = _fig_trajectory(df)
    fig_att = _fig_attitude(df)
    fig_aero = _fig_aero_forces(df)
    fig_gl = _fig_gload_energy(df)
    fig_cm = _fig_cm_stability(df)
    fig_ctrl = _fig_control_effectiveness(df)
    fig_forces = _fig_forces_3axis(df)
    fig_3d = _fig_3d_trajectory(df)
    fig_phase_portrait = _fig_phase_portrait(df)
    fig_energy_diss = _fig_energy_dissipation(df)
    fig_vel_comp = _fig_velocity_components(df)
    fig_fft = _fig_fft_spectrum(df)
    fig_tracking = _fig_tracking_error(df)
    # New figures for previously missing data
    fig_mpc_diag = _fig_mpc_diagnostics(df) if df.attrs.get("has_mpc_diag", False) else None
    fig_actuator = _fig_actuator_analysis(df)
    fig_ang_dyn = _fig_angular_dynamics(df) if df.attrs.get("has_angular_accel", False) else None
    fig_mass_prop = _fig_mass_propulsion(df)
    fig_vel_frames = _fig_velocity_frames(df)
    fig_sensor_mhe = _fig_sensor_mhe(df) if (df.attrs.get("has_sensor", False) or df.attrs.get("has_mhe", False)) else None
    fig_mhe_detail       = _fig_mhe_detail(df)       if df.attrs.get("has_mhe_states", False) else None
    fig_mhe_consistency  = _fig_mhe_consistency(df)  if df.attrs.get("has_mhe_states", False) else None
    mhe_cs               = _mhe_consistency_stats(df) if df.attrs.get("has_mhe_states", False) else None
    _has_ctrl_alloc = df.attrs.get("has_fins", False)
    fig_ctrl_alloc       = _fig_control_allocation(df) if _has_ctrl_alloc else None
    ctrl_alloc_stats     = _control_allocation_stats(df) if _has_ctrl_alloc else None
    fig_geo = _fig_geo_track(df) if df.attrs.get("has_lla", False) else None
    phase_scores = _score_phases(df, phases)
    plotly_cdn = '<script src="https://cdn.plot.ly/plotly-2.35.0.min.js"></script>'
    m = metrics
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    err_color = "#4caf50" if abs(m["range_error_m"]) < 50 else ("#ff9800" if abs(m["range_error_m"]) < 150 else "#f44336")

    key_cards = f"""<div class="grid grid-4">
    {_metric_card('Ground Range', f'{m["impact_range_m"]:.0f} m', f'Error: {m["range_error_m"]:+.0f}m ({m["range_error_pct"]:+.1f}%)', err_color)}
    {_metric_card('Impact Speed', f'{m["impact_speed_mps"]:.0f} m/s', f'Max: {m["max_speed_mps"]:.0f} m/s')}
    {_metric_card('Impact Angle', f'{m["impact_gamma_deg"]:.1f}°', f'Pitch: {m["impact_pitch_deg"]:.1f}°')}
    {_metric_card('Flight Time', f'{m["flight_time_s"]:.2f} s', f'{m["n_steps"]} points')}
    </div><div class="grid grid-4" style="margin-top:8px">
    {_metric_card('Peak Alt', f'{m["peak_alt_agl_m"]:.0f} m', f'@{m["peak_alt_time_s"]:.1f}s')}
    {_metric_card('Max Mach', f'{m["max_mach"]:.3f}', f'@{m["max_mach_time_s"]:.1f}s')}
    {_metric_card('Max G', f'{m["max_g"]:.1f} g', f'@{m["max_g_time_s"]:.1f}s')}
    {_metric_card('Max q', f'{m["max_q_Pa"]/1000:.1f} kPa', f'@{m["max_q_time_s"]:.1f}s')}
    </div><div class="grid grid-4" style="margin-top:8px">
    {_metric_card('Max Airspeed', f'{m["max_airspeed_mps"]:.0f} m/s', 'Air-relative')}
    {_metric_card('Static Margin', f'{m["mean_static_margin"]:.1f} cal', f'Min: {m["min_static_margin"]:.1f} cal')}
    {_metric_card('Max |Fy|', f'{m["max_force_y_N"]:.0f} N', 'Lateral force')}
    {_metric_card('Max |Fz|', f'{m["max_force_z_N"]:.0f} N', 'Normal force')}
    </div>"""

    # Build tabs header — always-visible tabs + conditional tabs
    tabs_header = ('<div class="tabs">'
        '<button class="tab-btn active" onclick="openTab(event,\'tab-overview\')">Overview</button>'
        '<button class="tab-btn" onclick="openTab(event,\'tab-numbers\')">📊 Numbers</button>'
        '<button class="tab-btn" onclick="openTab(event,\'tab-traj\')">Trajectory</button>'
        '<button class="tab-btn" onclick="openTab(event,\'tab-3d\')">3D View</button>'
        '<button class="tab-btn" onclick="openTab(event,\'tab-att\')">Attitude</button>'
        '<button class="tab-btn" onclick="openTab(event,\'tab-aero\')">Aero & Forces</button>'
        '<button class="tab-btn" onclick="openTab(event,\'tab-forces\')">Forces Detail</button>'
        '<button class="tab-btn" onclick="openTab(event,\'tab-control\')">Control</button>'
        '<button class="tab-btn" onclick="openTab(event,\'tab-actuator\')">Actuator</button>'
        '<button class="tab-btn" onclick="openTab(event,\'tab-struct\')">G-Load & Energy</button>'
        '<button class="tab-btn" onclick="openTab(event,\'tab-energy\')">Energy</button>'
        '<button class="tab-btn" onclick="openTab(event,\'tab-vel\')">Velocity</button>'
        '<button class="tab-btn" onclick="openTab(event,\'tab-velframes\')">Vel Frames</button>'
        '<button class="tab-btn" onclick="openTab(event,\'tab-mass\')">Mass & Propulsion</button>'
        '<button class="tab-btn" onclick="openTab(event,\'tab-phases\')">Phases</button>'
        '<button class="tab-btn" onclick="openTab(event,\'tab-stab\')">Stability</button>'
        '<button class="tab-btn" onclick="openTab(event,\'tab-portrait\')">Phase Portrait</button>'
        '<button class="tab-btn" onclick="openTab(event,\'tab-fft\')">FFT Spectrum</button>'
        '<button class="tab-btn" onclick="openTab(event,\'tab-tracking\')">Tracking</button>'
    )
    if fig_mpc_diag is not None:
        tabs_header += '<button class="tab-btn" onclick="openTab(event,\'tab-mpcdiag\')">MPC Diag</button>'
    if fig_ang_dyn is not None:
        tabs_header += '<button class="tab-btn" onclick="openTab(event,\'tab-angdyn\')">Angular Dyn</button>'
    if fig_sensor_mhe is not None:
        tabs_header += '<button class="tab-btn" onclick="openTab(event,\'tab-sensormhe\')">Sensor/MHE</button>'
    if fig_mhe_detail is not None:
        tabs_header += '<button class="tab-btn" onclick="openTab(event,\'tab-mhedetail\')">MHE Detail</button>'
    if fig_mhe_consistency is not None:
        tabs_header += '<button class="tab-btn" onclick="openTab(event,\'tab-mheconsistency\')">MHE Consistency</button>'
    if fig_ctrl_alloc is not None:
        tabs_header += '<button class="tab-btn" onclick="openTab(event,\'tab-ctrlalloc\')">Control Allocation</button>'
    if fig_geo is not None:
        tabs_header += '<button class="tab-btn" onclick="openTab(event,\'tab-geo\')">Geo Track</button>'
    if all_metrics and len(all_metrics) > 1:
        tabs_header += '<button class="tab-btn" onclick="openTab(event,\'tab-multi\')">Multi-Run</button>'
    tabs_header += '</div>'

    tab_overview = f'<div id="tab-overview" class="tab-panel active"><h2>Performance Score</h2>{_score_html(scores)}<div class="grid grid-2" style="margin-top:16px"><div class="card"><h3 style="margin-bottom:8px">Diagnostics</h3>{_diag_html(diags)}</div><div class="card"><h3 style="margin-bottom:8px">Recommendations</h3>{_rec_html(recs)}</div></div></div>'
    tab_numbers = f'<div id="tab-numbers" class="tab-panel"><h2>📊 All Numerical Metrics</h2>{_metrics_table_html(m, scores)}</div>'
    tab_traj = f'<div id="tab-traj" class="tab-panel">{_plotly_div(fig_traj)}</div>'
    tab_att = f'<div id="tab-att" class="tab-panel">{_plotly_div(fig_att)}</div>'
    tab_aero = f'<div id="tab-aero" class="tab-panel">{_plotly_div(fig_aero)}</div>'
    tab_struct = f'<div id="tab-struct" class="tab-panel">{_plotly_div(fig_gl)}</div>'
    tab_phases = f'<div id="tab-phases" class="tab-panel"><h2>Flight Phase Timeline</h2>{_phase_bar_html(phases, m["flight_time_s"])}<div class="card" style="margin-top:12px">{_phase_table_html(phases)}</div><h2>Phase-wise Scoring</h2>{_phase_score_html(phase_scores)}</div>'
    tab_stab = f'<div id="tab-stab" class="tab-panel"><h2>Stability & Moment Coefficients</h2>{_plotly_div(fig_cm)}</div>'
    tab_control = f'<div id="tab-control" class="tab-panel"><h2>Control Effectiveness & Static Margin</h2>{_plotly_div(fig_ctrl)}</div>'
    tab_forces = f'<div id="tab-forces" class="tab-panel"><h2>Forces & Airspeed Detail</h2>{_plotly_div(fig_forces)}</div>'
    tab_3d = f'<div id="tab-3d" class="tab-panel"><h2>3D Trajectory</h2>{_plotly_div(fig_3d)}</div>'
    tab_portrait = f'<div id="tab-portrait" class="tab-panel"><h2>Phase Portrait (Limit Cycle Detection)</h2><p style="color:#666;font-size:.85rem;margin-bottom:12px"><b>Row 1:</b> Phase-colored trajectories with direction arrows. Green ● = start, Red ✕ = end. Closed loops = limit cycle. Spirals → origin = stable. Spirals outward = unstable.<br><b>Row 2:</b> Time-gradient coloring (dark=early, bright=late) shows trajectory evolution.</p>{_plotly_div(fig_phase_portrait)}</div>'
    tab_energy = f'<div id="tab-energy" class="tab-panel"><h2>Energy Analysis & Dissipation</h2>{_plotly_div(fig_energy_diss)}</div>'
    tab_vel = f'<div id="tab-vel" class="tab-panel"><h2>Velocity Components & FUR Frame</h2>{_plotly_div(fig_vel_comp)}</div>'
    tab_fft = f'<div id="tab-fft" class="tab-panel"><h2>Oscillation Frequency Spectrum (FFT)</h2><p style="color:#666;font-size:.85rem;margin-bottom:12px">Dominant peaks indicate sustained oscillation frequencies. Compare with structural resonance limits.</p>{_plotly_div(fig_fft)}</div>'
    tab_tracking = f'<div id="tab-tracking" class="tab-panel"><h2>MPC Tracking Error & Control</h2><p style="color:#666;font-size:.85rem;margin-bottom:12px">Shows actual vs MPC reference, tracking error, wind disturbance, and fin deflections.</p>{_plotly_div(fig_tracking)}</div>'

    # New always-visible tabs
    tab_actuator = f'<div id="tab-actuator" class="tab-panel"><h2>Actuator Analysis</h2><p style="color:#666;font-size:.85rem;margin-bottom:12px">Commanded vs actual fin deflections, actuator lag, fin authority, and safety violations.</p>{_plotly_div(fig_actuator)}</div>'
    tab_velframes = f'<div id="tab-velframes" class="tab-panel"><h2>Velocity Reference Frames</h2><p style="color:#666;font-size:.85rem;margin-bottom:12px">Velocity in launch-fixed NED, local NED, and air-relative NED. Wind effect = ground − air velocity.</p>{_plotly_div(fig_vel_frames)}</div>'
    tab_mass = f'<div id="tab-mass" class="tab-panel"><h2>Mass & Propulsion Detail</h2><p style="color:#666;font-size:.85rem;margin-bottom:12px">Propellant fraction, CG position, axial force coefficient (CA), speed of sound, and staging.</p>{_plotly_div(fig_mass_prop)}</div>'

    # Conditional tabs
    tab_mpcdiag = ""
    if fig_mpc_diag is not None:
        mpc_info = f'Mean solve: {m.get("mpc_mean_solve_ms", 0):.2f} ms | Max: {m.get("mpc_max_solve_ms", 0):.2f} ms | Failures: {m.get("mpc_failures", 0)} | Mean SQP iters: {m.get("mpc_mean_sqp_iters", 0):.1f}'
        tab_mpcdiag = f'<div id="tab-mpcdiag" class="tab-panel"><h2>MPC Solver Diagnostics</h2><div class="card" style="margin-bottom:12px"><p style="font-size:.9rem">{mpc_info}</p></div>{_plotly_div(fig_mpc_diag)}</div>'

    tab_angdyn = ""
    if fig_ang_dyn is not None:
        tab_angdyn = f'<div id="tab-angdyn" class="tab-panel"><h2>Angular Dynamics</h2><p style="color:#666;font-size:.85rem;margin-bottom:12px">Angular acceleration (from Euler equation) and total moments after CG offset correction. Compare raw aero moments vs corrected total moments.</p>{_plotly_div(fig_ang_dyn)}</div>'

    tab_sensormhe = ""
    if fig_sensor_mhe is not None:
        tab_sensormhe = f'<div id="tab-sensormhe" class="tab-panel"><h2>Sensor Data & MHE Estimation</h2><p style="color:#666;font-size:.85rem;margin-bottom:12px">IMU measurements (accelerometer/gyroscope) with biases, and MHE estimation quality and solver performance.</p>{_plotly_div(fig_sensor_mhe)}</div>'

    tab_mhedetail = ""
    if fig_mhe_detail is not None:
        tab_mhedetail = (
            '<div id="tab-mhedetail" class="tab-panel">'
            '<h2>MHE Detailed State Estimation</h2>'
            '<p style="color:#666;font-size:.85rem;margin-bottom:12px">'
            'Full MHE estimated state vector vs truth: airspeed (V), flight path angle (γ), AoA (α), sideslip (β), '
            'roll angle (φ), angular rates (p,q,r), gyro bias estimates (b_gx, b_gy, b_gz), '
            'wind estimates (N, E), and estimation errors. Dashed lines = MHE estimates; solid = truth from 6-DOF.'
            '</p>'
            f'{_plotly_div(fig_mhe_detail)}</div>'
        )

    tab_geo = ""
    if fig_geo is not None:
        tab_geo = f'<div id="tab-geo" class="tab-panel"><h2>Geographic Track (LLA)</h2><p style="color:#666;font-size:.85rem;margin-bottom:12px">Ground track in latitude/longitude and geodetic altitude from WGS84 model.</p>{_plotly_div(fig_geo)}</div>'

    tab_mheconsistency = ""
    if fig_mhe_consistency is not None and mhe_cs is not None:
        st = mhe_cs
        # Consistency verdict badge
        c_val = st.get("overall_consistent")
        if c_val is True:
            c_badge = '<span class="badge pass">CONSISTENT</span>'
            c_detail = "Estimator bias < 0.5 × RMSE for α and V — estimator appears consistent."
        elif c_val is False:
            c_badge = '<span class="badge warn">BIASED</span>'
            c_detail = "Estimator bias ≥ 0.5 × RMSE — possible systematic error or model mismatch."
        else:
            c_badge = '<span class="badge info">N/A</span>'
            c_detail = "Insufficient MHE-active data for consistency verdict."
        drop_color = "#4caf50" if st["dropout_pct"] < 5 else ("#ff9800" if st["dropout_pct"] < 20 else "#f44336")
        active_pct  = f'{st["active_pct"]:.1f}%'
        n_steps_str = f'{st["n_active"]} / {st["n_active"] + st["n_dropout"]} steps'
        drop_str    = f'{st["dropout_pct"]:.1f}%'
        drop_sub    = f'{st["n_dropout"]} steps'
        rmse_a_str  = f'{st.get("rmse_alpha",  float("nan")):.3f}°'
        bias_a_str  = f'bias={st.get("bias_alpha",  0):+.3f}°'
        rmse_V_str  = f'{st.get("rmse_V",      float("nan")):.2f} m/s'
        bias_V_str  = f'bias={st.get("bias_V",      0):+.3f} m/s'
        rmse_b_str  = f'{st.get("rmse_beta",   float("nan")):.3f}°'
        bias_b_str  = f'bias={st.get("bias_beta",   0):+.3f}°'
        rmse_g_str  = f'{st.get("rmse_gamma",  float("nan")):.3f}°'
        bias_g_str  = f'bias={st.get("bias_gamma",  0):+.3f}°'
        maxa_str    = f'{st.get("max_abs_alpha", float("nan")):.2f}°'
        maxV_str    = f'{st.get("max_abs_V",     float("nan")):.2f} m/s'
        summary_cards = (
            '<div class="grid grid-4" style="margin-bottom:16px">'
            + f'<div class="card">{_metric_card("MHE Active",    active_pct,  n_steps_str)}</div>'
            + f'<div class="card">{_metric_card("Dropout Rate",  drop_str,    drop_sub, drop_color)}</div>'
            + f'<div class="card">{_metric_card("RMSE α",        rmse_a_str,  bias_a_str)}</div>'
            + f'<div class="card">{_metric_card("RMSE V",        rmse_V_str,  bias_V_str)}</div>'
            + '</div>'
            + '<div class="grid grid-4" style="margin-bottom:16px">'
            + f'<div class="card">{_metric_card("RMSE β",        rmse_b_str,  bias_b_str)}</div>'
            + f'<div class="card">{_metric_card("RMSE γ",        rmse_g_str,  bias_g_str)}</div>'
            + f'<div class="card">{_metric_card("Max |α err|",   maxa_str)}</div>'
            + f'<div class="card">{_metric_card("Max |V err|",   maxV_str)}</div>'
            + '</div>'
        )
        verdict_card = f'<div class="card" style="margin-bottom:16px"><h3 style="margin-bottom:8px">Consistency Verdict {c_badge}</h3><p style="font-size:.85rem;color:#555">{c_detail}</p><p style="font-size:.8rem;color:#999;margin-top:6px">NIS (Normalized Innovation Squared): plot should fluctuate around χ²(n) mean (= number of estimated states). Values consistently above the 95% line indicate over-confidence (P too small); values below the 5% line indicate under-confidence (P too large).</p></div>'
        tab_mheconsistency = (
            '<div id="tab-mheconsistency" class="tab-panel">'
            '<h2>MHE Estimator Consistency Analysis</h2>'
            f'{verdict_card}{summary_cards}'
            f'{_plotly_div(fig_mhe_consistency)}'
            '</div>'
        )

    tab_ctrlalloc = ""
    if fig_ctrl_alloc is not None and ctrl_alloc_stats is not None:
        st = ctrl_alloc_stats
        FIN_SAT = st.get("fin_sat_threshold_deg", 20.0)
        fin_cards = ""
        for j in range(1, 5):
            rms  = st.get(f"fin{j}_rms_deg",  float("nan"))
            mxd  = st.get(f"fin{j}_max_deg",  float("nan"))
            satp = st.get(f"fin{j}_sat_pct",  float("nan"))
            sat_color = "#4caf50" if satp < 1 else ("#ff9800" if satp < 5 else "#f44336")
            fin_cards += (
                f'<div class="card">'
                + _metric_card(f"Fin {j}", f"{rms:.2f}°",
                               f"max={mxd:.1f}° | sat={satp:.1f}%", sat_color)
                + '</div>'
            )
        fid_rows = ""
        for axis_key, label in [("e", "Pitch (δe)"), ("r", "Yaw (δr)"), ("a", "Roll (δa)")]:
            r = st.get(f"mixing_r_{axis_key}")
            if r is None:
                continue
            verdict = "PASS" if abs(r) > 0.9 else ("WARN" if abs(r) > 0.7 else "FAIL")
            fid_rows += (f'<tr><td>{label}</td><td>{r:.4f}</td>'
                         f'<td>{_badge(verdict)}</td></tr>')
        fid_html = ""
        if fid_rows:
            fid_html = (
                '<div class="card" style="margin-bottom:16px">'
                '<h3 style="margin-bottom:8px">Mixer Fidelity (Pearson r: MPC virtual vs reconstructed)</h3>'
                '<table><tr><th>Axis</th><th>r</th><th>Verdict</th></tr>'
                f'{fid_rows}</table>'
                '<p style="font-size:.8rem;color:#999;margin-top:6px">'
                'r &gt; 0.9 → mixer faithfully reproduces MPC commands. '
                'r &lt; 0.7 → saturation or mixing errors are degrading control.</p>'
                '</div>'
            )
        tab_ctrlalloc = (
            '<div id="tab-ctrlalloc" class="tab-panel">'
            '<h2>Control Allocation Analysis</h2>'
            f'<div class="grid grid-4" style="margin-bottom:16px">{fin_cards}</div>'
            f'{fid_html}'
            f'{_plotly_div(fig_ctrl_alloc)}'
            '</div>'
        )

    tab_multi = ""
    if all_metrics and len(all_metrics) > 1:
        mdf = pd.DataFrame(all_metrics)
        tab_multi = f'<div id="tab-multi" class="tab-panel"><h2>Multi-Run Analysis ({len(mdf)} runs)</h2>{_multi_run_stats_html(mdf)}<h2 style="margin-top:20px">2D Impact Dispersion</h2>{_plotly_div(_fig_dispersion_2d(mdf))}<h2 style="margin-top:20px">Scatter Comparison</h2>{_plotly_div(_fig_multi_scatter(mdf))}<h2>Trends Over Runs</h2>{_plotly_div(_fig_multi_trends(mdf))}'
        if all_dfs and len(all_dfs) > 1:
            tab_multi += f'<h2>Trajectory Overlay</h2>{_plotly_div(_fig_multi_overlay(all_dfs))}'
        if len(mdf) >= 5:
            tab_multi += f'<h2>Correlation Matrix</h2>{_plotly_div(_fig_correlation(mdf))}'
        tab_multi += '</div>'

    # Build phase filter options from detected phases
    phase_opts = '<option value="all">All Phases</option>' + "".join(
        f'<option value="{html_escape(p["name"])}">{html_escape(p["name"])}</option>'
        for p in phases
    )
    toolbar = (
        '<div class="toolbar">'
        '<button id="theme-toggle" class="theme-toggle">☾ Dark Mode</button>'
        f'<label style="font-size:.85rem;color:var(--text-secondary)">Phase Filter:</label>'
        f'<select class="phase-filter" onchange="filterPhase(this)">{phase_opts}</select>'
        '</div>'
    )

    html = f"""<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>M130 Analysis — {m['timestamp']}</title>{plotly_cdn}<style>{_CSS}</style></head>
<body><div class="container"><h1>M130 6-DOF Simulation Analysis</h1>
{toolbar}<div style="color:var(--text-secondary);font-size:.85rem;margin-bottom:16px">File: {html_escape(m['file'])} | Generated: {now}</div>
{key_cards}<div class="tab-container">{tabs_header}{tab_overview}{tab_numbers}{tab_traj}{tab_3d}{tab_att}{tab_aero}{tab_forces}{tab_control}{tab_actuator}{tab_struct}{tab_energy}{tab_vel}{tab_velframes}{tab_mass}{tab_phases}{tab_stab}{tab_portrait}{tab_fft}{tab_tracking}{tab_mpcdiag}{tab_angdyn}{tab_sensormhe}{tab_mhedetail}{tab_mheconsistency}{tab_ctrlalloc}{tab_geo}{tab_multi}</div></div>
<script>{_JS}</script></body></html>"""

    if html_path:
        Path(html_path).write_text(html, encoding="utf-8")
    return html


# ─── Console Summary ─────────────────────────────────────────────────────────

def print_console_summary(m, scores):
    overall = scores["_overall"]
    total = scores["_total"]
    tag = {"PASS": "✅", "WARN": "⚠️", "FAIL": "❌"}[overall]
    print()
    print("═" * 58)
    print(f"  M130 Analysis  {tag} {overall} ({total:.0f}/100)")
    print("═" * 58)
    print(f"  Range:     {m['impact_range_m']:.0f}m  (err {m['range_error_m']:+.0f}m = {m['range_error_pct']:+.1f}%)")
    print(f"  Speed:     {m['impact_speed_mps']:.0f} m/s   γ: {m['impact_gamma_deg']:.1f}°   Pitch: {m['impact_pitch_deg']:.1f}°")
    print(f"  Peak Alt:  {m['peak_alt_agl_m']:.0f}m   Max Mach: {m['max_mach']:.3f}   Max G: {m['max_g']:.1f}")
    print(f"  Time:      {m['flight_time_s']:.2f}s   Max |α|: {m['max_alpha_deg']:.1f}°   σ_pitch: {m['pitch_std_last30pct']:.2f}°")
    # Servo delay compensation check
    max_roll = m.get("max_roll_deg", 0)
    pitch_std = m.get("pitch_std_last30pct", 0)
    max_alpha = m.get("max_alpha_deg", 0)
    delay_issues = sum([max_roll > 30, pitch_std > 5, max_alpha > 15])
    if delay_issues >= 2:
        print(f"  ⚠ SERVO DELAY: Under-compensated (roll={max_roll:.0f}°, σ_pitch={pitch_std:.1f}°, |α|={max_alpha:.1f}°)")
        print(f"    → Increase delay_steps in rocket_properties.yaml")
        print(f"    → For HIL/flight: set RKT_MPC_SVO_DLY = measured delay (e.g. 0.080 for 80ms)")
    elif overall == "PASS":
        print(f"  ✅ Servo delay OK — for HIL/flight: set RKT_MPC_SVO_DLY = measured servo delay")
    print("═" * 58)


# ─── Entry Points ────────────────────────────────────────────────────────────

def analyze_csv(csv_path, all_csvs=None, open_browser=True):
    """Main entry point. Returns (metrics, scores, html_path)."""
    csv_path = Path(csv_path)
    df = _load_csv(csv_path)
    metrics = _extract_run_metrics(df, csv_path)
    scores = _score_run(metrics)
    diags = _diagnose(df, metrics)
    recs = _recommend(metrics, scores, diags)
    phases = _analyze_phases(df)

    all_metrics, all_dfs = None, None
    if all_csvs and len(all_csvs) > 1:
        all_dfs, all_metrics = [], []
        for p in all_csvs:
            try:
                d = _load_csv(Path(p))
                all_dfs.append(d)
                all_metrics.append(_extract_run_metrics(d, Path(p)))
            except Exception:
                pass

    out_dir = csv_path.parent / "plots"
    out_dir.mkdir(parents=True, exist_ok=True)
    html_path = out_dir / f"analysis_{metrics['timestamp']}.html"

    generate_html_report(df, metrics, scores, diags, recs, phases,
                         all_metrics=all_metrics, all_dfs=all_dfs, html_path=str(html_path))
    print_console_summary(metrics, scores)
    print(f"  HTML → {html_path}")

    if open_browser:
        webbrowser.open(f"file://{html_path.resolve()}")
    return metrics, scores, str(html_path)


def discover_csv_files(results_dir: Path) -> list:
    return sorted(results_dir.glob("*_log.csv"), key=lambda p: p.stat().st_mtime, reverse=True)


def main():
    parser = argparse.ArgumentParser(description="M130 6-DOF Interactive HTML Analysis")
    parser.add_argument("--all", action="store_true", help="Analyze ALL CSV runs")
    parser.add_argument("--latest", type=int, default=None, help="Analyze newest N runs")
    parser.add_argument("--file", type=str, default=None, help="Analyze specific CSV")
    parser.add_argument("--no-open", action="store_true", help="Don't open browser")
    args = parser.parse_args()

    results_dir = Path(__file__).parent
    csv_files = [Path(args.file)] if args.file else discover_csv_files(results_dir)
    if not csv_files:
        print("ERROR: no *_log.csv files found")
        sys.exit(1)
    if not args.all and args.latest is None and args.file is None:
        csv_files = csv_files[:1]
    elif args.latest:
        csv_files = csv_files[:args.latest]

    print(f"  Analyzing {len(csv_files)} CSV file(s)...")
    all_csvs = csv_files if len(csv_files) > 1 else None
    analyze_csv(csv_files[0], all_csvs=all_csvs, open_browser=not args.no_open)


if __name__ == "__main__":
    main()
