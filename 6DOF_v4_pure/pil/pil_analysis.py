#!/usr/bin/env python3
"""
════════════════════════════════════════════════════════════════════════
  M130 PIL Flight Analysis — Interactive HTML Report
  تحليل نتائج طيران PIL (Processor-In-the-Loop) — تقرير HTML تفاعلي
════════════════════════════════════════════════════════════════════════

Generates a self-contained professional HTML report for PIL flights.
Similar to sitl_analysis.py but for PIL data:
  - PIL CSV has no accel_x/y/z (derived from velocity gradient instead)
  - Additional Timing tab if pil_timing.csv has samples (MHE/MPC/cycle us)
  - PIL-specific diagnostics (ARM64 realtime deadline, MPC timeouts)

Usage:
    python pil_analysis.py                        # latest PIL CSV
    python pil_analysis.py --file <csv_path>      # specific file
    python pil_analysis.py --no-open              # don't open browser
"""

import sys
import argparse
import webbrowser
from pathlib import Path
from datetime import datetime
from html import escape as html_escape

import numpy as np
import pandas as pd
import yaml

try:
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots
    import plotly.io as pio
except ImportError:
    print("ERROR: plotly is required.  pip install plotly")
    sys.exit(1)

# ─── Paths ────────────────────────────────────────────────────────────
_SCRIPT_DIR = Path(__file__).resolve().parent
_SIM_DIR = _SCRIPT_DIR.parent
_RESULTS_DIR = _SCRIPT_DIR / "results"
_CONFIG_PATH = _SIM_DIR / "config" / "6dof_config_advanced.yaml"
_PIL_CONFIG_PATH = _SCRIPT_DIR / "pil_config.yaml"

# ─── Mission Config ───────────────────────────────────────────────────
try:
    with open(_CONFIG_PATH, "r") as _f:
        _cfg = yaml.safe_load(_f)
    TARGET_RANGE_M = float(_cfg.get("target", {}).get("range_m", 2900.0))
    LAUNCH_ALT_M = float(_cfg.get("launch", {}).get("altitude", 1200.0))
    _BEARING_DEG = float(_cfg.get("target", {}).get("bearing_deg", 0.0))
except Exception:
    TARGET_RANGE_M = 2900.0
    LAUNCH_ALT_M = 1200.0
    _BEARING_DEG = 0.0

# ─── PIL Timing Deadline ──────────────────────────────────────────────
try:
    with open(_PIL_CONFIG_PATH, "r") as _f:
        _pil_cfg = yaml.safe_load(_f)
    TIMING_DEADLINE_US = int(_pil_cfg.get("timing", {}).get("deadline_us", 20000))
    TIMING_WARN_US = int(_pil_cfg.get("timing", {}).get("warn_us", 10000))
except Exception:
    TIMING_DEADLINE_US = 20000
    TIMING_WARN_US = 10000


# ══════════════════════════════════════════════════════════════════════
#  Data Loading & Derived Quantities
# ══════════════════════════════════════════════════════════════════════

def load_pil_csv(path: Path) -> pd.DataFrame:
    """Load PIL bridge CSV and compute derived columns.

    Unlike SITL, PIL CSV lacks `accel_x/y/z`.  We derive accelerations
    from velocity gradients.
    """
    df = pd.read_csv(path)
    df.rename(columns={"time": "time_s"}, inplace=True)

    # Speed: |v| في أي frame ثابت (rotation-invariant).
    df["speed_total"] = np.sqrt(df["vel_x"]**2 + df["vel_y"]**2 + df["vel_z"]**2)
    # speed_horizontal: في long_range mode vel هي ECEF → sqrt(vx²+vy²) خاطئ.
    # نَحسبها لاحقاً من |v|² - climb_rate² (frame-independent).

    # Angles in degrees
    df["alpha_deg"] = np.degrees(df["alpha"])
    df["beta_deg"] = np.degrees(df["beta"])

    # Euler angles from quaternion
    q0, q1, q2, q3 = df["q0"], df["q1"], df["q2"], df["q3"]
    df["roll_deg"] = np.degrees(np.arctan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2)))
    df["pitch_deg"] = np.degrees(np.arcsin(np.clip(2*(q0*q2 - q3*q1), -1, 1)))
    df["yaw_deg"] = np.degrees(np.arctan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2)))

    # Flight path angle: يَستَخدم |v| و climb_rate (frame-independent).
    # إصلاح بغت long_range: vel_x/y/z في ECEF → sqrt(vx²+vy²) ليس v_h الحقيقي.
    # الصحيح: v_h = sqrt(|v|² - v_vertical²) حيث v_vertical = dalt/dt.
    with np.errstate(divide='ignore', invalid='ignore'):
        dt = np.gradient(df["time_s"].values)
        dt = np.where(dt > 0, dt, 1e-6)
        dalt = np.gradient(df["altitude"].values)
        climb_rate = dalt / dt
        v_h_correct = np.sqrt(np.maximum(df["speed_total"]**2 - climb_rate**2, 0.0))
        df["speed_horizontal"] = v_h_correct
        df["gamma_deg"] = np.degrees(np.arctan2(climb_rate, np.maximum(v_h_correct, 0.1)))

    # Angular rates in deg/s
    df["omega_x_deg"] = np.degrees(df["omega_x"])
    df["omega_y_deg"] = np.degrees(df["omega_y"])
    df["omega_z_deg"] = np.degrees(df["omega_z"])
    df["omega_total_deg"] = np.sqrt(df["omega_x_deg"]**2 + df["omega_y_deg"]**2 + df["omega_z_deg"]**2)

    # Fin deflections in degrees
    for i in range(1, 5):
        df[f"fin_cmd_{i}_deg"] = np.degrees(df[f"fin_cmd_{i}"])
        df[f"fin_act_{i}_deg"] = np.degrees(df[f"fin_act_{i}"])
        df[f"fin_lag_{i}_deg"] = df[f"fin_cmd_{i}_deg"] - df[f"fin_act_{i}_deg"]

    # PIL CSV lacks accel_* — derive from velocity gradient
    if "accel_x" not in df.columns:
        dt_grad = np.gradient(df["time_s"].values)
        dt_grad = np.where(dt_grad > 0, dt_grad, 1e-6)
        df["accel_x"] = np.gradient(df["vel_x"].values) / dt_grad
        df["accel_y"] = np.gradient(df["vel_y"].values) / dt_grad
        df["accel_z"] = np.gradient(df["vel_z"].values) / dt_grad

    # G-load from (derived) body accelerations
    df["g_total"] = np.sqrt(df["accel_x"]**2 + df["accel_y"]**2 + df["accel_z"]**2) / 9.80665
    df["g_axial"] = df["accel_x"] / 9.80665

    # Force magnitude
    df["force_total"] = np.sqrt(df["force_x"]**2 + df["force_y"]**2 + df["force_z"]**2)

    # Energy
    df["KE_kJ"] = 0.5 * df["mass"] * df["speed_total"]**2 / 1000
    df["PE_kJ"] = df["mass"] * 9.81 * df["altitude"] / 1000
    df["total_energy_kJ"] = df["KE_kJ"] + df["PE_kJ"]

    # Alt AGL (subtract launch altitude)
    df["alt_agl_m"] = df["altitude"] - LAUNCH_ALT_M

    # Flight phase detection
    df["flight_phase"] = "COAST"
    dm = np.gradient(df["mass"].values)
    df.loc[dm < -0.01, "flight_phase"] = "BOOST"
    if df["alt_agl_m"].max() > 0:
        df.loc[(df["alt_agl_m"] < df["alt_agl_m"].max() * 0.3) &
               (df["time_s"] > df["time_s"].iloc[-1] * 0.7), "flight_phase"] = "TERMINAL"

    return df


def load_pil_timing(csv_path: Path):
    """Load pil_timing.csv if it has samples.  Returns None if empty/missing."""
    # Derive timestamped timing filename from flight CSV name:
    #   pil_flight_20260501_120455.csv → pil_timing_20260501_120455.csv
    stem = csv_path.stem  # e.g. "pil_flight_20260501_120455"
    ts_suffix = stem.replace("pil_flight", "")  # e.g. "_20260501_120455"
    timing_path = csv_path.parent / f"pil_timing{ts_suffix}.csv"
    if not timing_path.exists():
        # Fallback to non-timestamped name (standalone CLI usage)
        timing_path = csv_path.parent / "pil_timing.csv"
    if not timing_path.exists():
        return None
    try:
        tdf = pd.read_csv(timing_path)
    except Exception:
        return None
    if len(tdf) == 0:
        return None
    return tdf


# ══════════════════════════════════════════════════════════════════════
#  Metrics Extraction
# ══════════════════════════════════════════════════════════════════════

def extract_metrics(df: pd.DataFrame, filepath: Path, timing_df=None) -> dict:
    m = {}
    t = df["time_s"].values
    last = df.iloc[-1]

    m["file"] = filepath.name
    m["timestamp"] = filepath.stem
    m["flight_time_s"] = t[-1]
    m["n_steps"] = len(df)

    m["impact_range_m"] = last["ground_range"]
    m["range_error_m"] = last["ground_range"] - TARGET_RANGE_M
    m["range_error_pct"] = (last["ground_range"] / TARGET_RANGE_M - 1) * 100 if TARGET_RANGE_M > 0 else 0

    # Cross-range: من lat/lon (إذا متوفر) — يَكشف انجراف الرياح
    # PIL CSV يَملك lat/lon. النتيجة تُسقَط على bearing axis.
    if "lat" in df.columns and "lon" in df.columns:
        lat0 = float(df["lat"].iloc[0])
        lat_end = float(last["lat"])
        lon0 = float(df["lon"].iloc[0])
        lon_end = float(last["lon"])
        if abs(lat0) < 1.6:
            lat0 = np.degrees(lat0); lat_end = np.degrees(lat_end)
            lon0 = np.degrees(lon0); lon_end = np.degrees(lon_end)
        d_north = (lat_end - lat0) * 111320.0
        d_east = (lon_end - lon0) * 111320.0 * np.cos(np.radians(lat0))
        try:
            bearing_rad = np.radians(float(_BEARING_DEG))
        except (NameError, TypeError):
            bearing_rad = 0.0
        crossrange = -d_north * np.sin(bearing_rad) + d_east * np.cos(bearing_rad)
        m["cross_range_error_m"] = float(crossrange)
    else:
        m["cross_range_error_m"] = 0.0
    m["total_miss_m"] = float(np.sqrt(m["range_error_m"]**2 + m["cross_range_error_m"]**2))

    m["peak_alt_m"] = df["alt_agl_m"].max()
    m["peak_alt_time_s"] = df.loc[df["alt_agl_m"].idxmax(), "time_s"]
    m["impact_speed_mps"] = last["speed_total"]
    m["max_speed_mps"] = df["speed_total"].max()
    m["max_mach"] = df["mach"].max()

    # AoA/β عند vel<30 m/s (لحظة الانطلاق) = atan(wind/0) → vacuous.
    # نأخذ الذروات أثناء الطيران الفعلي فقط.
    # Use 50 m/s threshold (not 30) to skip the rail-departure transient
    # where the rocket is still on the launcher and α/β arctan2 produce
    # large spurious peaks from tiny velocity components combined with
    # crosswind. By ~50 m/s the rocket has cleared the rail and aerodynamic
    # alpha/beta become physically meaningful.
    _inflight = df["speed_total"].values > 50.0
    if _inflight.any():
        m["max_alpha_deg"] = float(df["alpha_deg"].abs().values[_inflight].max())
        m["max_beta_deg"] = float(df["beta_deg"].abs().values[_inflight].max())
    else:
        m["max_alpha_deg"] = float(df["alpha_deg"].abs().max())
        m["max_beta_deg"] = float(df["beta_deg"].abs().max())
    m["max_g"] = df["g_total"].max()
    m["max_axial_g"] = df["g_axial"].max()

    m["max_omega_deg_s"] = df["omega_total_deg"].max()
    m["max_roll_deg"] = df["roll_deg"].abs().max()

    m["max_fin_cmd_deg"] = max(df[f"fin_cmd_{i}_deg"].abs().max() for i in range(1, 5))
    m["max_fin_act_deg"] = max(df[f"fin_act_{i}_deg"].abs().max() for i in range(1, 5))

    m["mass_initial"] = df["mass"].iloc[0]
    m["mass_final"] = df["mass"].iloc[-1]
    m["propellant_kg"] = df["mass"].iloc[0] - df["mass"].iloc[-1]

    n30 = max(1, int(len(df) * 0.3))
    m["pitch_std_last30pct"] = df["pitch_deg"].iloc[-n30:].std()

    m["impact_gamma_deg"] = df["gamma_deg"].iloc[-1]
    m["impact_pitch_deg"] = last["pitch_deg"]

    # Max |alpha| during stable flight: exclude terminal tumble (last 5%)
    # AND pre-launch + rail-departure (speed < 50 m/s) where alpha/beta are
    # arctan2 of tiny components and produce spurious peaks.
    t_cutoff = t[-1] * 0.95
    stable = df[(df["time_s"] < t_cutoff) & (df["speed_total"] > 50.0)]
    m["max_alpha_flight_deg"] = stable["alpha_deg"].abs().max() if len(stable) else m["max_alpha_deg"]
    m["max_beta_flight_deg"] = stable["beta_deg"].abs().max() if len(stable) else m["max_beta_deg"]

    # Fin saturation during stable flight
    sat_frac = 0.0
    if len(stable):
        n_stable = len(stable)
        for i in range(1, 5):
            f_sat = (stable[f"fin_cmd_{i}_deg"].abs() > 19.5).sum() / n_stable
            sat_frac = max(sat_frac, f_sat)
    m["fin_saturation_pct"] = sat_frac * 100

    # Timing metrics (PIL-specific)
    if timing_df is not None and len(timing_df) > 0:
        m["timing_samples"] = len(timing_df)
        for col, label in [("mpc_us", "mpc"), ("mhe_us", "mhe"), ("cycle_us", "cycle")]:
            if col in timing_df.columns:
                m[f"{label}_us_avg"] = float(timing_df[col].mean())
                m[f"{label}_us_p99"] = float(timing_df[col].quantile(0.99))
                m[f"{label}_us_max"] = float(timing_df[col].max())
                m[f"{label}_over_deadline_pct"] = float((timing_df[col] > TIMING_DEADLINE_US).mean() * 100)
    else:
        m["timing_samples"] = 0

    return m


# ══════════════════════════════════════════════════════════════════════
#  Scoring (same weights as SITL for comparability)
# ══════════════════════════════════════════════════════════════════════

SCORE_WEIGHTS = {"range": 30, "impact_angle": 15, "stability": 20, "aoa": 15, "sideslip": 10, "g_load": 10}
THRESH = {
    "range_error_good":  50,
    "range_error_warn":  200,
    "max_alpha_warn":    15,
    "max_alpha_fail":    25,
    "max_beta_warn":     5,
    "max_beta_fail":     10,
    "pitch_std_warn":    3,
    "pitch_std_fail":    8,
    "max_g_warn":        15,
    "max_g_fail":        25,
}


def score_run(m: dict) -> dict:
    scores = {}
    W = SCORE_WEIGHTS

    # Range — يَستَخدم خطأ الضربة الكلي (downrange + cross-range)
    w = W["range"]
    miss_total = m.get("total_miss_m", abs(m["range_error_m"]))
    dr = m["range_error_m"]
    cr = m.get("cross_range_error_m", 0.0)
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
                        "detail": f"miss = {miss_total:.0f}m  (DR {dr:+.0f}, CR {cr:+.0f})"}

    # Impact angle (adaptive)
    w = W["impact_angle"]
    gamma = m["impact_gamma_deg"]
    is_flat = m["peak_alt_m"] < 500
    if is_flat:
        if gamma < -3:
            s, v = w, "PASS"
        elif gamma < 0:
            s, v = round(w * 0.8, 1), "PASS"
        else:
            s, v = round(w * 0.4, 1), "WARN"
    else:
        if -80 < gamma < -20:
            s, v = w, "PASS"
        elif -90 < gamma < -10:
            s, v = round(w * 0.53, 1), "WARN"
        else:
            s, v = 0, "FAIL"
    traj_type = "flat" if is_flat else "steep"
    scores["impact_angle"] = {"score": round(s, 1), "verdict": v,
                               "detail": f"γ = {gamma:.1f}° ({traj_type} trajectory)"}

    # Stability — adaptive thresholds based on trajectory type.
    # Flat missions transition pitch 60°+ in <15s — high σ is physically
    # correct, not a control failure. Steep missions stabilize in terminal
    # dive, so tight thresholds apply.
    w = W["stability"]
    pstd = m["pitch_std_last30pct"]
    if is_flat:
        warn_th, fail_th = 8.0, 15.0
        detail_suffix = f" (flat traj.; warn≥{warn_th:.0f}°)"
    else:
        warn_th, fail_th = THRESH["pitch_std_warn"], THRESH["pitch_std_fail"]
        detail_suffix = " (steep traj.)"
    if pstd < warn_th:
        s, v = w, "PASS"
    elif pstd < fail_th:
        s = w * (1 - (pstd - warn_th) / (fail_th - warn_th))
        v = "WARN"
    else:
        s, v = 0, "FAIL"
    scores["stability"] = {"score": round(s, 1), "verdict": v,
                            "detail": f"pitch σ = {pstd:.2f}°{detail_suffix}"}

    # AoA — use during-flight value (not terminal)
    w = W["aoa"]
    alpha = m.get("max_alpha_flight_deg", m["max_alpha_deg"])
    if alpha < THRESH["max_alpha_warn"]:
        s, v = w, "PASS"
    elif alpha < THRESH["max_alpha_fail"]:
        s, v = round(w * 0.5, 1), "WARN"
    else:
        s, v = 0, "FAIL"
    scores["aoa"] = {"score": round(s, 1), "verdict": v,
                      "detail": f"max |α| (flight) = {alpha:.1f}°"}

    # Sideslip — use during-flight value (not rail-departure transient)
    w = W["sideslip"]
    beta = m.get("max_beta_flight_deg", m["max_beta_deg"])
    if beta < THRESH["max_beta_warn"]:
        s, v = w, "PASS"
    elif beta < THRESH["max_beta_fail"]:
        s, v = round(w * 0.5, 1), "WARN"
    else:
        s, v = 0, "FAIL"
    scores["sideslip"] = {"score": round(s, 1), "verdict": v,
                           "detail": f"max |β| = {beta:.1f}°"}

    # G-load
    w = W["g_load"]
    g = m["max_g"]
    if g < THRESH["max_g_warn"]:
        s, v = w, "PASS"
    elif g < THRESH["max_g_fail"]:
        s, v = round(w * 0.5, 1), "WARN"
    else:
        s, v = 0, "FAIL"
    scores["g_load"] = {"score": round(s, 1), "verdict": v,
                         "detail": f"max G = {g:.1f}"}

    total = sum(c["score"] for c in scores.values())
    overall = "PASS" if total >= 80 else ("WARN" if total >= 50 else "FAIL")
    scores["_total"] = round(total, 1)
    scores["_overall"] = overall

    checks = [(k, v["verdict"], v["detail"]) for k, v in scores.items() if not k.startswith("_")]
    return {"checks": checks,
            "details": {k: v["score"] for k, v in scores.items() if not k.startswith("_")},
            "overall": overall, "total": round(total, 1)}


# ══════════════════════════════════════════════════════════════════════
#  Diagnostics & Recommendations (PIL-specific)
# ══════════════════════════════════════════════════════════════════════

def diagnose(df, m):
    diags = []
    if m["flight_time_s"] < 5:
        diags.append(("error", "Very Short Flight",
                      f"Only {m['flight_time_s']:.1f}s — possible early crash or sim failure"))
    if m.get("max_alpha_flight_deg", m["max_alpha_deg"]) > 15:
        diags.append(("warning", "High Angle of Attack",
                      f"|α| reached {m['max_alpha_flight_deg']:.1f}° during flight "
                      f"— may indicate MPC solver timeouts on ARM64"))
    if m["max_omega_deg_s"] > 200:
        diags.append(("warning", "High Angular Rate",
                      f"ω reached {m['max_omega_deg_s']:.0f}°/s — tumbling risk"))
    if m["max_fin_cmd_deg"] < 0.1:
        diags.append(("warning", "No Control Commands",
                      "PX4 rocket_mpc sent near-zero fin commands — check ARM state"))
    if m["fin_saturation_pct"] > 3:
        diags.append(("warning", "Fin Saturation",
                      f"Fins at max deflection {m['fin_saturation_pct']:.1f}% of flight "
                      f"— solver may be computing unstable controls"))
    if m["propellant_kg"] < 0.1:
        diags.append(("info", "No Propellant Burned",
                      "Mass barely changed — check thrust model"))

    # PIL timing diagnostics
    if m["timing_samples"] == 0:
        diags.append(("info", "No Timing Data",
                      "pil_timing.csv is empty — 'adb reverse tcp:5760 tcp:5760' "
                      "may not be set, or MAVLink DEBUG_VECT stream is not enabled"))
    else:
        over_pct = m.get("mpc_over_deadline_pct", 0)
        if over_pct > 50:
            diags.append(("error", "MPC Deadline Violations",
                          f"mpc_us exceeded {TIMING_DEADLINE_US}us in {over_pct:.0f}% of cycles "
                          f"— ARM64 cannot meet realtime budget"))
        elif over_pct > 10:
            diags.append(("warning", "Occasional MPC Timeouts",
                          f"mpc_us exceeded deadline in {over_pct:.0f}% of cycles"))

    # ── Servo delay compensation diagnostics (PIL) ────────────────────
    max_roll = m.get("max_roll_deg", 0)
    pitch_std = m.get("pitch_std_last30pct", 0)
    max_alpha = m.get("max_alpha_flight_deg", m.get("max_alpha_deg", 0))

    delay_issues = 0
    delay_hints = []
    if max_roll > 30:
        delay_issues += 1
        delay_hints.append(f"roll reached {max_roll:.0f}° (should be <10°)")
    if pitch_std > 5:
        delay_issues += 1
        delay_hints.append(f"pitch σ={pitch_std:.1f}° in terminal (should be <4°)")
    if max_alpha > 15:
        delay_issues += 1
        delay_hints.append(f"|α| reached {max_alpha:.1f}° (should be <12°)")

    if delay_issues >= 2:
        diags.append(("error", "Servo Delay Under-Compensated",
                      "MPC commands arrive too late for servo — increase RKT_MPC_SVO_DLY. "
                      "Symptoms: " + "; ".join(delay_hints)))
    elif delay_issues == 1:
        diags.append(("warning", "Possible Servo Delay Mismatch",
                      "One symptom of under-compensated delay: " + delay_hints[0]))

    return diags


def recommend(m, scores, diags):
    recs = []
    if m["max_fin_cmd_deg"] < 0.5:
        recs.append("Verify PX4 is armed and rocket_mpc module is running — fin commands are near-zero")
    if abs(m["range_error_pct"]) > 15:
        recs.append(f"Range error is {m['range_error_pct']:+.1f}% — tune MPC guidance gains or verify thrust curve")
    if m.get("max_alpha_flight_deg", 0) > 12:
        recs.append("High in-flight AoA — consider lowering qp_solver_cond_N further or reducing N_horizon")
    if m["pitch_std_last30pct"] > 5:
        recs.append("Pitch oscillation in terminal phase — check damping gains and actuator bandwidth")
    if m["fin_saturation_pct"] > 3:
        recs.append("Fins saturating — likely MPC solve timeouts produce suboptimal controls; "
                    "check qp_solver_cond_N is set (should be ~10, not None)")

    # ── Servo delay compensation recommendations (PIL) ────────────────
    max_roll = m.get("max_roll_deg", 0)
    pitch_std = m.get("pitch_std_last30pct", 0)
    max_alpha = m.get("max_alpha_flight_deg", m.get("max_alpha_deg", 0))

    delay_issues = sum([
        max_roll > 30,
        pitch_std > 5,
        max_alpha > 15,
    ])

    if delay_issues >= 2:
        recs.append(
            "⚠ SERVO DELAY: Increase RKT_MPC_SVO_DLY in QGC "
            "(currently under-compensated). "
            "Rule: RKT_MPC_SVO_DLY = measured servo delay in seconds. "
            "Example: 0.080 for 80ms → lookahead_stage=4"
        )
    elif delay_issues == 0 and scores.get("overall") == "PASS":
        recs.append(
            "✅ Servo delay compensation OK — RKT_MPC_SVO_DLY is well-tuned. "
            "Use same value for real flight."
        )

    if m["timing_samples"] == 0:
        recs.append("Enable timing capture: ensure 'adb reverse tcp:5760 tcp:5760' and PX4 publishes "
                    "DEBUG_VECT 'TIMING' with fields mhe_us/mpc_us/cycle_us")
    if not diags and scores["overall"] == "PASS":
        recs.append("PIL flight nominal — no issues detected")
    return recs


# ══════════════════════════════════════════════════════════════════════
#  Plotly Figures
# ══════════════════════════════════════════════════════════════════════

def _fig_trajectory(df):
    t = df["time_s"]
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Altitude AGL vs Time", "Ground Range vs Time",
                                        "Trajectory Profile", "Speed vs Time"),
                        vertical_spacing=0.12, horizontal_spacing=0.08)
    fig.add_trace(go.Scatter(x=t, y=df["alt_agl_m"], mode="lines",
                             name="Altitude", line=dict(color="#1f77b4", width=2)), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["ground_range"], mode="lines",
                             name="Range", line=dict(color="#ff7f0e", width=2)), row=1, col=2)
    fig.add_trace(go.Scatter(x=df["ground_range"], y=df["alt_agl_m"], mode="lines",
                             name="Profile", line=dict(color="#2ca02c", width=2)), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["speed_total"], mode="lines",
                             name="Speed", line=dict(color="#d62728", width=2)), row=2, col=2)
    fig.add_hline(y=TARGET_RANGE_M, line_dash="dash", line_color="red", opacity=0.5, row=1, col=2,
                  annotation_text=f"Target {TARGET_RANGE_M:.0f}m")
    fig.add_vline(x=TARGET_RANGE_M, line_dash="dash", line_color="red", opacity=0.4, row=2, col=1)
    fig.update_xaxes(title_text="Time (s)", row=1, col=1)
    fig.update_xaxes(title_text="Time (s)", row=1, col=2)
    fig.update_xaxes(title_text="Ground Range (m)", row=2, col=1)
    fig.update_xaxes(title_text="Time (s)", row=2, col=2)
    fig.update_yaxes(title_text="Altitude AGL (m)", row=1, col=1)
    fig.update_yaxes(title_text="Range (m)", row=1, col=2)
    fig.update_yaxes(title_text="Altitude AGL (m)", row=2, col=1)
    fig.update_yaxes(title_text="Speed (m/s)", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white",
                      legend=dict(orientation="h", yanchor="bottom", y=1.02))
    return fig


def _fig_attitude(df):
    t = df["time_s"]
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Euler Angles", "Flight Path γ",
                                        "α & β (AoA / Sideslip)", "Angular Rates"),
                        vertical_spacing=0.12, horizontal_spacing=0.08)
    fig.add_trace(go.Scatter(x=t, y=df["pitch_deg"], name="Pitch", line=dict(color="#1f77b4")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["yaw_deg"], name="Yaw", line=dict(color="#d62728")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["roll_deg"], name="Roll", line=dict(color="#2ca02c")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["gamma_deg"], name="γ",
                             line=dict(color="#17becf"), showlegend=False), row=1, col=2)
    fig.add_hline(y=0, line_dash="dot", line_color="gray", opacity=0.3, row=1, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["alpha_deg"], name="α",
                             line=dict(color="#1f77b4"), showlegend=False), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["beta_deg"], name="β",
                             line=dict(color="#d62728"), showlegend=False), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["omega_y_deg"], name="q (pitch)",
                             line=dict(color="#1f77b4"), showlegend=False), row=2, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["omega_z_deg"], name="r (yaw)",
                             line=dict(color="#d62728"), showlegend=False), row=2, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["omega_x_deg"], name="p (roll)",
                             line=dict(color="#2ca02c"), showlegend=False), row=2, col=2)
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="Angle (°)", row=1, col=1)
    fig.update_yaxes(title_text="γ (°)", row=1, col=2)
    fig.update_yaxes(title_text="Angle (°)", row=2, col=1)
    fig.update_yaxes(title_text="Rate (°/s)", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white")
    return fig


def _fig_aero(df):
    t = df["time_s"]
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Mach Number", "G-Load",
                                        "α vs Mach", "Mass & Propellant"),
                        vertical_spacing=0.12, horizontal_spacing=0.08)
    fig.add_trace(go.Scatter(x=t, y=df["mach"], name="Mach",
                             line=dict(color="#9467bd", width=2)), row=1, col=1)
    fig.add_hline(y=1.0, line_dash="dash", line_color="red", opacity=0.4, row=1, col=1,
                  annotation_text="Mach 1")
    fig.add_trace(go.Scatter(x=t, y=df["g_total"], name="G-Load",
                             line=dict(color="#e74c3c", width=2)), row=1, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["g_axial"], name="Axial G",
                             line=dict(color="#ff7f0e", width=1, dash="dot")), row=1, col=2)
    fig.add_trace(go.Scatter(x=df["mach"], y=df["alpha_deg"], mode="markers",
                             name="α vs Mach",
                             marker=dict(size=3, color=t, colorscale="Viridis",
                                         colorbar=dict(title="Time (s)")),
                             showlegend=False), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["mass"], name="Mass",
                             line=dict(color="#8c564b", width=2)), row=2, col=2)
    fig.update_xaxes(title_text="Time (s)", row=1, col=1)
    fig.update_xaxes(title_text="Time (s)", row=1, col=2)
    fig.update_xaxes(title_text="Mach", row=2, col=1)
    fig.update_xaxes(title_text="Time (s)", row=2, col=2)
    fig.update_yaxes(title_text="Mach", row=1, col=1)
    fig.update_yaxes(title_text="G", row=1, col=2)
    fig.update_yaxes(title_text="α (°)", row=2, col=1)
    fig.update_yaxes(title_text="Mass (kg)", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white")
    return fig


def _fig_forces(df):
    t = df["time_s"]
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Body Forces (3-axis)", "Body Accelerations (derived)",
                                        "Energy Budget", "Force Magnitude"),
                        vertical_spacing=0.12, horizontal_spacing=0.08)
    fig.add_trace(go.Scatter(x=t, y=df["force_x"], name="F_x (axial)",
                             line=dict(color="#1f77b4")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["force_y"], name="F_y",
                             line=dict(color="#2ca02c")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["force_z"], name="F_z",
                             line=dict(color="#d62728")), row=1, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["accel_x"], name="a_x",
                             line=dict(color="#1f77b4"), showlegend=False), row=1, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["accel_y"], name="a_y",
                             line=dict(color="#2ca02c"), showlegend=False), row=1, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["accel_z"], name="a_z",
                             line=dict(color="#d62728"), showlegend=False), row=1, col=2)
    fig.add_trace(go.Scatter(x=t, y=df["KE_kJ"], name="Kinetic",
                             line=dict(color="#ff7f0e"), showlegend=False), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["PE_kJ"], name="Potential",
                             line=dict(color="#2ca02c"), showlegend=False), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["total_energy_kJ"], name="Total",
                             line=dict(color="#1f77b4", width=2), showlegend=False), row=2, col=1)
    fig.add_trace(go.Scatter(x=t, y=df["force_total"], name="|F|",
                             line=dict(color="#9467bd"), showlegend=False), row=2, col=2)
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="Force (N)", row=1, col=1)
    fig.update_yaxes(title_text="Accel (m/s²)", row=1, col=2)
    fig.update_yaxes(title_text="Energy (kJ)", row=2, col=1)
    fig.update_yaxes(title_text="Force (N)", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white")
    return fig


def _fig_control(df):
    t = df["time_s"]
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("Fin Commands (PX4 → Bridge)",
                                        "Fin Actual (Actuator Model)",
                                        "Actuator Lag (Cmd − Actual)",
                                        "Command Envelope"),
                        vertical_spacing=0.12, horizontal_spacing=0.08)
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]
    for i in range(1, 5):
        fig.add_trace(go.Scatter(x=t, y=df[f"fin_cmd_{i}_deg"], name=f"Fin {i} cmd",
                                 line=dict(color=colors[i-1], width=1.5)), row=1, col=1)
        fig.add_trace(go.Scatter(x=t, y=df[f"fin_act_{i}_deg"], name=f"Fin {i} act",
                                 line=dict(color=colors[i-1], width=1.5), showlegend=False), row=1, col=2)
        fig.add_trace(go.Scatter(x=t, y=df[f"fin_lag_{i}_deg"], name=f"Fin {i} lag",
                                 line=dict(color=colors[i-1], width=1), showlegend=False), row=2, col=1)
    all_cmd = np.concatenate([df[f"fin_cmd_{i}_deg"].values for i in range(1, 5)])
    all_t = np.tile(t.values, 4)
    fig.add_trace(go.Scatter(x=all_t, y=all_cmd, mode="markers",
                             marker=dict(size=2, color="#1f77b4", opacity=0.3),
                             name="All cmds", showlegend=False), row=2, col=2)
    fig.add_hline(y=20, line_dash="dash", line_color="red", opacity=0.5, row=2, col=2,
                  annotation_text="±20° limit")
    fig.add_hline(y=-20, line_dash="dash", line_color="red", opacity=0.5, row=2, col=2)
    fig.update_xaxes(title_text="Time (s)")
    fig.update_yaxes(title_text="Deflection (°)", row=1, col=1)
    fig.update_yaxes(title_text="Deflection (°)", row=1, col=2)
    fig.update_yaxes(title_text="Lag (°)", row=2, col=1)
    fig.update_yaxes(title_text="Deflection (°)", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white",
                      legend=dict(orientation="h", yanchor="bottom", y=1.02))
    return fig


def _fig_3d_trajectory(df):
    PHASE_COLORS = {
        "BOOST": "#f44336", "COAST": "#2196f3", "TERMINAL": "#9c27b0",
        "ARMED": "#9e9e9e", "LAUNCH": "#ff9800", "CRUISE": "#4caf50",
        "BALLISTIC": "#795548",
    }
    has_pos = "pos_x" in df.columns and "pos_y" in df.columns
    fig = go.Figure()
    if has_pos and "flight_phase" in df.columns:
        from collections import OrderedDict
        for ph in OrderedDict.fromkeys(df["flight_phase"]):
            mask = df["flight_phase"] == ph
            c = PHASE_COLORS.get(ph, "#333")
            fig.add_trace(go.Scatter3d(
                x=df["pos_x"][mask], y=df["pos_y"][mask], z=df["alt_agl_m"][mask],
                mode="lines", name=ph, line=dict(color=c, width=5), legendgroup=ph,
                text=[f"t={t:.1f}s<br>Alt={a:.0f}m<br>V={v:.0f}m/s"
                      for t, a, v in zip(df["time_s"][mask], df["alt_agl_m"][mask],
                                         df["speed_total"][mask])],
                hoverinfo="text"
            ))
        fig.update_layout(
            scene=dict(xaxis_title="X North (m)", yaxis_title="Y East (m)",
                       zaxis_title="Altitude (m)", aspectmode="data"),
            height=650, template="plotly_white",
            title="3D Trajectory (colored by flight phase)",
            legend=dict(orientation="h", yanchor="bottom", y=1.02)
        )
    else:
        fig.add_trace(go.Scatter3d(
            x=df["ground_range"].values, y=np.zeros(len(df)), z=df["alt_agl_m"].values,
            mode="lines",
            line=dict(color=df["speed_total"].values, colorscale="Turbo", width=5,
                      colorbar=dict(title="Speed (m/s)", x=1.05)),
            text=[f"t={t:.1f}s<br>Alt={a:.0f}m<br>V={v:.0f}m/s<br>M={mach:.3f}"
                  for t, a, v, mach in zip(df["time_s"], df["alt_agl_m"],
                                           df["speed_total"], df["mach"])],
            hoverinfo="text"
        ))
        fig.update_layout(
            scene=dict(xaxis_title="Range (m)", yaxis_title="Cross-range (m)",
                       zaxis_title="Altitude AGL (m)", aspectmode="data"),
            height=600, template="plotly_white", title="3D Trajectory (colored by speed)"
        )
    return fig


def _fig_geo(df):
    if "lat" not in df.columns:
        return None
    fig = go.Figure(go.Scattermap(
        lat=df["lat"], lon=df["lon"],
        mode="lines+markers",
        marker=dict(size=4, color=df["time_s"], colorscale="Viridis",
                    colorbar=dict(title="Time (s)")),
        line=dict(width=2, color="#d62728"),
        text=[f"t={t:.1f}s Alt={a:.0f}m" for t, a in zip(df["time_s"], df["alt_msl"])],
        hoverinfo="text"
    ))
    center_lat = df["lat"].mean()
    center_lon = df["lon"].mean()
    fig.update_layout(
        map=dict(style="open-street-map",
                 center=dict(lat=center_lat, lon=center_lon), zoom=12),
        height=500, margin=dict(l=0, r=0, t=30, b=0), title="Ground Track"
    )
    return fig


def _fig_phase_portrait(df):
    t = df["time_s"]
    fig = make_subplots(rows=1, cols=2,
                        subplot_titles=("Pitch Phase Portrait (θ vs q)",
                                        "α Phase Portrait (α vs q)"))
    fig.add_trace(go.Scatter(
        x=df["pitch_deg"], y=df["omega_y_deg"], mode="markers",
        marker=dict(size=3, color=t, colorscale="Viridis",
                    colorbar=dict(title="t (s)", x=0.45)),
        name="θ-q", showlegend=False), row=1, col=1)
    fig.add_trace(go.Scatter(
        x=df["alpha_deg"], y=df["omega_y_deg"], mode="markers",
        marker=dict(size=3, color=t, colorscale="Plasma"),
        name="α-q", showlegend=False), row=1, col=2)
    fig.update_xaxes(title_text="Pitch (°)", row=1, col=1)
    fig.update_xaxes(title_text="α (°)", row=1, col=2)
    fig.update_yaxes(title_text="q (°/s)")
    fig.update_layout(height=450, template="plotly_white")
    return fig


def _fig_timing(timing_df):
    """PIL-specific: MPC / MHE / cycle timing vs deadline."""
    if timing_df is None or len(timing_df) == 0:
        return None
    t = timing_df["t_sim"] if "t_sim" in timing_df.columns else np.arange(len(timing_df))
    fig = make_subplots(rows=2, cols=2,
                        subplot_titles=("MPC solve time (µs)", "MHE solve time (µs)",
                                        "Full cycle time (µs)", "Distribution (MPC)"),
                        vertical_spacing=0.14, horizontal_spacing=0.08)
    deadline = TIMING_DEADLINE_US
    warn = TIMING_WARN_US

    if "mpc_us" in timing_df.columns:
        fig.add_trace(go.Scatter(x=t, y=timing_df["mpc_us"], name="mpc",
                                 line=dict(color="#e74c3c", width=1)), row=1, col=1)
        fig.add_hline(y=deadline, line_dash="dash", line_color="red", opacity=0.6, row=1, col=1,
                      annotation_text=f"deadline {deadline}µs")
        fig.add_hline(y=warn, line_dash="dot", line_color="orange", opacity=0.5, row=1, col=1,
                      annotation_text=f"warn {warn}µs")
        fig.add_trace(go.Histogram(x=timing_df["mpc_us"], nbinsx=50, name="mpc dist",
                                   marker_color="#e74c3c", showlegend=False), row=2, col=2)
    if "mhe_us" in timing_df.columns:
        fig.add_trace(go.Scatter(x=t, y=timing_df["mhe_us"], name="mhe",
                                 line=dict(color="#2ca02c", width=1), showlegend=False), row=1, col=2)
        fig.add_hline(y=deadline, line_dash="dash", line_color="red", opacity=0.4, row=1, col=2)
    if "cycle_us" in timing_df.columns:
        fig.add_trace(go.Scatter(x=t, y=timing_df["cycle_us"], name="cycle",
                                 line=dict(color="#1f77b4", width=1), showlegend=False), row=2, col=1)
        fig.add_hline(y=deadline, line_dash="dash", line_color="red", opacity=0.6, row=2, col=1,
                      annotation_text=f"deadline {deadline}µs")

    fig.update_xaxes(title_text="t_sim (s)", row=1, col=1)
    fig.update_xaxes(title_text="t_sim (s)", row=1, col=2)
    fig.update_xaxes(title_text="t_sim (s)", row=2, col=1)
    fig.update_xaxes(title_text="mpc_us", row=2, col=2)
    fig.update_yaxes(title_text="µs", row=1, col=1)
    fig.update_yaxes(title_text="µs", row=1, col=2)
    fig.update_yaxes(title_text="µs", row=2, col=1)
    fig.update_yaxes(title_text="count", row=2, col=2)
    fig.update_layout(height=700, template="plotly_white")
    return fig


# ══════════════════════════════════════════════════════════════════════
#  HTML Generation
# ══════════════════════════════════════════════════════════════════════

_CSS = """\
:root{--bg:#f8f9fa;--card:#fff;--border:#e0e0e0;--text:#222;--text-secondary:#666;
--text-muted:#999;--accent:#6a1b9a;--pass:#4caf50;--warn:#ff9800;--fail:#e53935;
--th-bg:#f0f4f8;--hover:rgba(106,27,154,.04);--diag-error-bg:#fbe9e7;
--diag-warn-bg:#fff3e0;--diag-info-bg:#e3f2fd;--rec-bg:#e8f5e9}
[data-theme="dark"]{--bg:#121212;--card:#1e1e1e;--border:#333;--text:#e0e0e0;
--text-secondary:#aaa;--text-muted:#777;--accent:#ba68c8;--th-bg:#2a2a2a;
--hover:rgba(186,104,200,.08);--diag-error-bg:#3e1a1a;--diag-warn-bg:#3e2e1a;
--diag-info-bg:#1a2e3e;--rec-bg:#1a3e1a}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,-apple-system,sans-serif;background:var(--bg);
color:var(--text);line-height:1.6;padding:20px;transition:background .3s,color .3s}
.container{max-width:1400px;margin:0 auto}
h1{font-size:1.8rem;border-bottom:3px solid var(--accent);padding-bottom:8px;margin-bottom:16px}
h2{font-size:1.3rem;color:var(--accent);margin:24px 0 12px;border-left:4px solid var(--accent);padding-left:10px}
.grid{display:grid;gap:16px}.grid-2{grid-template-columns:1fr 1fr}
.grid-3{grid-template-columns:1fr 1fr 1fr}.grid-4{grid-template-columns:repeat(4,1fr)}
.card{background:var(--card);border:1px solid var(--border);border-radius:8px;
padding:16px;box-shadow:0 1px 3px rgba(0,0,0,.08);transition:background .3s,border-color .3s}
.score-ring{width:120px;height:120px;border-radius:50%;display:flex;align-items:center;
justify-content:center;font-size:2rem;font-weight:700;margin:0 auto 8px;border:6px solid}
.score-ring.pass{border-color:var(--pass);color:var(--pass)}
.score-ring.warn{border-color:var(--warn);color:var(--warn)}
.score-ring.fail{border-color:var(--fail);color:var(--fail)}
.badge{display:inline-block;padding:2px 10px;border-radius:12px;font-size:.75rem;
font-weight:700;color:#fff;text-transform:uppercase}
.badge.pass{background:var(--pass)}.badge.warn{background:var(--warn)}.badge.fail{background:var(--fail)}
.badge.info{background:#2196f3}
.metric-box{text-align:center;padding:12px}
.metric-box .value{font-size:1.6rem;font-weight:700;color:var(--accent)}
.metric-box .label{font-size:.75rem;color:var(--text-secondary);text-transform:uppercase;letter-spacing:.5px}
.metric-box .sub{font-size:.7rem;color:var(--text-muted)}
table{width:100%;border-collapse:collapse;font-size:.85rem}
th{background:var(--th-bg);padding:8px 12px;text-align:left;border-bottom:2px solid var(--border);font-weight:600}
td{padding:6px 12px;border-bottom:1px solid var(--border)}tr:hover{background:var(--hover)}
.diag{padding:10px 14px;border-radius:6px;margin-bottom:8px;border-left:4px solid}
.diag.error{background:var(--diag-error-bg);border-color:var(--fail)}
.diag.warning{background:var(--diag-warn-bg);border-color:var(--warn)}
.diag.info{background:var(--diag-info-bg);border-color:#2196f3}
.diag .dtitle{font-weight:700;font-size:.9rem}
.diag .ddetail{font-size:.8rem;color:var(--text-secondary);margin-top:2px}
.rec{padding:8px 14px;background:var(--rec-bg);border-radius:6px;margin-bottom:6px;
font-size:.85rem;border-left:3px solid var(--pass)}
.tabs{display:flex;gap:4px;border-bottom:2px solid var(--border);flex-wrap:wrap}
.tab-btn{padding:8px 20px;border:none;background:none;cursor:pointer;font-size:.9rem;
font-weight:600;border-bottom:3px solid transparent;color:var(--text-secondary);transition:.2s}
.tab-btn:hover{color:var(--accent)}.tab-btn.active{color:var(--accent);border-bottom-color:var(--accent)}
.tab-panel{display:none;padding:16px 0}.tab-panel.active{display:block}
.toolbar{display:flex;align-items:center;gap:12px;margin-bottom:16px;flex-wrap:wrap}
.theme-toggle{background:var(--card);border:1px solid var(--border);border-radius:20px;
padding:6px 14px;cursor:pointer;font-size:.85rem;color:var(--text);transition:.2s;
display:flex;align-items:center;gap:6px}.theme-toggle:hover{border-color:var(--accent)}
.pil-banner{background:linear-gradient(90deg,#6a1b9a,#8e24aa);color:#fff;padding:6px 14px;
border-radius:20px;font-size:.8rem;font-weight:700}
@media(max-width:900px){.grid-2,.grid-3,.grid-4{grid-template-columns:1fr}}
"""

_JS = """\
function openTab(evt,tabId){
  document.querySelectorAll('.tab-panel').forEach(p=>p.classList.remove('active'));
  document.querySelectorAll('.tab-btn').forEach(b=>b.classList.remove('active'));
  document.getElementById(tabId).classList.add('active');
  evt.currentTarget.classList.add('active');
  var el=document.getElementById(tabId);
  el.querySelectorAll('.js-plotly-plot').forEach(function(p){Plotly.Plots.resize(p);});
}
(function(){
  var saved=localStorage.getItem('m130_pil_theme')||'light';
  document.documentElement.setAttribute('data-theme',saved);
  window.addEventListener('DOMContentLoaded',function(){
    var btn=document.getElementById('theme-toggle');
    if(!btn)return;
    _updateToggleLabel(btn,saved);
    btn.addEventListener('click',function(){
      var cur=document.documentElement.getAttribute('data-theme')||'light';
      var next=cur==='dark'?'light':'dark';
      document.documentElement.setAttribute('data-theme',next);
      localStorage.setItem('m130_pil_theme',next);
      _updateToggleLabel(btn,next);
      var bg=next==='dark'?'#1e1e1e':'#fff';
      var fg=next==='dark'?'#e0e0e0':'#444';
      document.querySelectorAll('.js-plotly-plot').forEach(function(p){
        Plotly.relayout(p,{'paper_bgcolor':bg,'plot_bgcolor':bg,'font.color':fg});
      });
    });
  });
  function _updateToggleLabel(btn,theme){
    btn.textContent=theme==='dark'?'\\u2600 Light Mode':'\\u263e Dark Mode';
  }
})();
"""

plotly_cdn = '<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>'


def _metric_card(label, value, sub="", color="var(--accent)"):
    return (f'<div class="metric-box"><div class="value" style="color:{color}">{value}</div>'
            f'<div class="label">{label}</div><div class="sub">{sub}</div></div>')


# ─── Numerical Metrics Table (self-contained) ────────────────────────────
_UNIT_SUFFIXES = (
    ("_mps2","m/s²"),("_deg_s","°/s"),("_deg","°"),("_rad_s","rad/s"),
    ("_rad","rad"),("_mps","m/s"),("_ms","ms"),("_us","µs"),("_hz","Hz"),
    ("_kpa","kPa"),("_pa","Pa"),("_nm","N·m"),("_kg","kg"),("_pct","%"),
    ("_m2","m²"),("_m","m"),("_s","s"),("_n","N"),("_g","g"),("_count",""),
)
_CATEGORIES_TBL = (
    ("file","Run Info"),("timestamp","Run Info"),
    ("impact_","Impact"),("range_","Impact"),("flight_time","Impact"),
    ("peak_","Trajectory"),("apogee","Trajectory"),("max_alt","Trajectory"),
    ("ground_range","Trajectory"),("max_mach","Aerodynamics"),
    ("max_q","Aerodynamics"),("max_alpha","Aerodynamics"),("max_beta","Aerodynamics"),
    ("max_g","Loads"),("max_omega","Loads"),("max_roll","Loads"),
    ("max_speed","Loads"),("pitch_","Stability"),("yaw_","Stability"),
    ("tracking_","Stability"),("mpc_","MPC"),("mhe_","MHE"),
    ("ekf2_","EKF2"),("sensor_","Sensors"),("servo_","Servos"),
    ("can_","CAN"),("fb_","Feedback"),("cmd_","Commands"),
    ("fin_","Actuators"),("mass_","Mass"),("dt_","Timing"),
    ("solve_","Timing"),("timing_","Timing"),("latency_","Latency"),
)
def _infer_unit(key):
    k = key.lower()
    for sfx, u in _UNIT_SUFFIXES:
        if k.endswith(sfx): return u
    return ""
def _category_for(key):
    k = key.lower()
    for prefix, cat in _CATEGORIES_TBL:
        if k.startswith(prefix) or k == prefix.rstrip("_"): return cat
    return "Other"
def _fmt_value(v):
    if v is None: return "—"
    if isinstance(v, bool): return "true" if v else "false"
    if isinstance(v, (int, np.integer)): return f"{int(v):,}"
    if isinstance(v, (float, np.floating)):
        if not np.isfinite(v):
            return "NaN" if np.isnan(v) else ("+∞" if v > 0 else "−∞")
        av = abs(v)
        if av == 0: return "0"
        if av >= 1e6 or av < 1e-3: return f"{v:.3e}"
        if av >= 100: return f"{v:.1f}"
        if av >= 1: return f"{v:.3f}"
        return f"{v:.4f}"
    if isinstance(v, (list, tuple, np.ndarray)):
        arr = list(v)
        if len(arr) > 6:
            return "[" + ", ".join(_fmt_value(x) for x in arr[:6]) + f", … ×{len(arr)-6}]"
        return "[" + ", ".join(_fmt_value(x) for x in arr) + "]"
    if isinstance(v, dict):
        kvs = ", ".join(f"{k}={_fmt_value(val)}" for k, val in list(v.items())[:4])
        more = f" … +{len(v)-4}" if len(v) > 4 else ""
        return f"{{{kvs}{more}}}"
    return html_escape(str(v))

def _metrics_table_html(m, scores=None, title="All Numerical Metrics"):
    from collections import OrderedDict
    groups = OrderedDict()
    for k, v in m.items():
        if k.startswith("_"): continue
        groups.setdefault(_category_for(k), []).append((k, v))
    order = ["Run Info","Impact","Trajectory","Aerodynamics","Loads",
             "Stability","MPC","MHE","EKF2","Sensors","Servos","CAN",
             "Feedback","Commands","Actuators","Mass","Timing","Latency","Other"]
    g = OrderedDict((c, groups[c]) for c in order if c in groups)
    for c in groups:
        if c not in g: g[c] = groups[c]
    score_card = ""
    if scores:
        rows = ""
        for k, v in scores.items():
            if k in ("total","overall") or k.startswith("_"): continue
            if isinstance(v, dict) and "verdict" in v:
                rows += (f'<tr><td><b>{html_escape(k)}</b></td>'
                         f'<td><span class="badge {v["verdict"].lower()}">{v["verdict"]}</span></td>'
                         f'<td style="text-align:right">{v["score"]:.1f}</td>'
                         f'<td style="font-size:.85rem;color:var(--text-secondary,#666)">{html_escape(str(v.get("detail","")))}</td></tr>')
        if "total" in scores:
            ov = scores.get("overall","")
            rows += (f'<tr style="background:var(--surface-2,#f5f5f5);font-weight:700">'
                     f'<td>TOTAL</td><td><span class="badge {ov.lower()}">{ov}</span></td>'
                     f'<td style="text-align:right">{scores["total"]:.1f}</td><td>/100</td></tr>')
        if rows:
            score_card = ('<div class="card" style="margin-bottom:16px">'
                          '<h3 style="margin-bottom:8px">Score Breakdown</h3>'
                          '<table><tr><th>Category</th><th>Verdict</th><th style="text-align:right">Score</th><th>Detail</th></tr>'
                          f'{rows}</table></div>')
    rows_html = ""
    for cat, items in g.items():
        rows_html += (f'<tr style="background:var(--surface-2,#fafafa)"><td colspan="3" '
                      f'style="font-weight:700;padding:8px 6px;border-top:2px solid var(--border,#ddd)">'
                      f'■ {html_escape(cat)} <span style="color:var(--text-secondary,#888);font-weight:400;font-size:.85rem">({len(items)})</span></td></tr>')
        for k, v in sorted(items, key=lambda kv: kv[0]):
            rows_html += (f'<tr class="metric-row">'
                          f'<td style="font-family:ui-monospace,monospace;font-size:.85rem">{html_escape(k)}</td>'
                          f'<td style="text-align:right;font-variant-numeric:tabular-nums;font-weight:600">{_fmt_value(v)}</td>'
                          f'<td style="color:var(--text-secondary,#666);font-size:.85rem">{html_escape(_infer_unit(k))}</td></tr>')
    table_html = ('<div class="card">'
                  f'<h3 style="margin-bottom:8px">{html_escape(title)} '
                  f'<span style="color:var(--text-secondary,#888);font-weight:400;font-size:.85rem">({sum(len(v) for v in g.values())} keys)</span></h3>'
                  '<input type="text" placeholder="🔎 Filter metric name…" '
                  'oninput="(function(e){const q=e.target.value.toLowerCase();'
                  'document.querySelectorAll(\'#tab-numbers .metric-row\').forEach(r=>{'
                  'r.style.display=r.children[0].textContent.toLowerCase().includes(q)?\'\':\'none\';});})(event)" '
                  'style="width:100%;padding:8px;margin-bottom:12px;border:1px solid var(--border,#ddd);border-radius:4px">'
                  '<table style="width:100%"><thead><tr><th style="text-align:left">Metric</th>'
                  '<th style="text-align:right">Value</th><th style="text-align:left">Unit</th>'
                  f'</tr></thead><tbody>{rows_html}</tbody></table></div>')
    return score_card + table_html


def _plotly_div(fig):
    if fig is None:
        return ""
    config = {"responsive": True, "displayModeBar": True,
              "modeBarButtonsToRemove": ["lasso2d", "select2d"]}
    return pio.to_html(fig, full_html=False, include_plotlyjs=False, config=config)


def generate_html(df, metrics, scores, diags, recs, timing_df=None, html_path=None):
    m = metrics
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    overall = scores["overall"]
    total = scores["total"]
    ring_cls = overall.lower()
    score_html = (
        f'<div class="card" style="text-align:center">'
        f'<div class="score-ring {ring_cls}">{total}</div>'
        f'<div style="font-weight:700;font-size:1.1rem">{overall}</div>'
        f'<div style="font-size:.8rem;color:var(--text-secondary)">/ 100</div></div>'
    )

    checks_rows = ""
    for name, verdict, detail in scores["checks"]:
        badge = f'<span class="badge {verdict.lower()}">{verdict}</span>'
        checks_rows += f'<tr><td>{name}</td><td>{badge}</td><td>{detail}</td></tr>'
    checks_html = (
        f'<div class="card"><h3 style="margin-bottom:8px">Health Checks</h3>'
        f'<table><tr><th>Check</th><th>Status</th><th>Detail</th></tr>'
        f'{checks_rows}</table></div>'
    )

    # Timing mini-card if available
    timing_card = ""
    if m.get("timing_samples", 0) > 0:
        mpc_avg = m.get("mpc_us_avg", 0) / 1000
        mpc_max = m.get("mpc_us_max", 0) / 1000
        over_pct = m.get("mpc_over_deadline_pct", 0)
        color = "var(--pass)" if over_pct < 10 else ("var(--warn)" if over_pct < 50 else "var(--fail)")
        timing_card = (
            f'<div class="card">'
            f'<h3 style="margin-bottom:8px">MPC Timing (ARM64)</h3>'
            f'<table>'
            f'<tr><td>Samples</td><td>{m["timing_samples"]}</td></tr>'
            f'<tr><td>MPC avg</td><td>{mpc_avg:.1f} ms</td></tr>'
            f'<tr><td>MPC max</td><td>{mpc_max:.1f} ms</td></tr>'
            f'<tr><td>Over deadline</td><td style="color:{color};font-weight:700">'
            f'{over_pct:.1f}%</td></tr>'
            f'</table></div>'
        )

    key_cards = f"""
    <div class="grid grid-4" style="margin-bottom:16px">
      <div class="card">{_metric_card("Range", f"{m['impact_range_m']:.0f}m",
                                       f"err {m['range_error_pct']:+.1f}%",
                                       "var(--pass)" if abs(m['range_error_pct'])<5 else "var(--warn)")}</div>
      <div class="card">{_metric_card("Peak Alt AGL", f"{m['peak_alt_m']:.0f}m",
                                       f"at t={m['peak_alt_time_s']:.1f}s")}</div>
      <div class="card">{_metric_card("Max Mach", f"{m['max_mach']:.3f}",
                                       f"Speed {m['max_speed_mps']:.0f} m/s")}</div>
      <div class="card">{_metric_card("Flight Time", f"{m['flight_time_s']:.1f}s",
                                       f"{m['n_steps']} steps")}</div>
    </div>
    <div class="grid grid-4" style="margin-bottom:16px">
      <div class="card">{_metric_card("Max |α| (flight)",
                                       f"{m.get('max_alpha_flight_deg', m['max_alpha_deg']):.1f}°",
                                       "excludes terminal tumble")}</div>
      <div class="card">{_metric_card("Max G", f"{m['max_g']:.1f}g",
                                       f"Axial: {m['max_axial_g']:.1f}g")}</div>
      <div class="card">{_metric_card("Fin Max", f"{m['max_fin_cmd_deg']:.1f}°",
                                       f"saturated {m['fin_saturation_pct']:.1f}% of flight")}</div>
      <div class="card">{_metric_card("Propellant", f"{m['propellant_kg']:.2f} kg",
                                       f"{m['mass_initial']:.2f} → {m['mass_final']:.2f} kg")}</div>
    </div>
    """

    diag_html = ""
    for level, title, detail in diags:
        diag_html += (f'<div class="diag {level}"><div class="dtitle">{title}</div>'
                      f'<div class="ddetail">{detail}</div></div>')
    if not diag_html:
        diag_html = ('<div class="diag info"><div class="dtitle">All Clear</div>'
                     '<div class="ddetail">No issues detected</div></div>')

    rec_html = "".join(f'<div class="rec">{r}</div>' for r in recs)

    # Figures
    fig_traj = _fig_trajectory(df)
    fig_att = _fig_attitude(df)
    fig_aero = _fig_aero(df)
    fig_forces = _fig_forces(df)
    fig_ctrl = _fig_control(df)
    fig_3d = _fig_3d_trajectory(df)
    fig_geo = _fig_geo(df)
    fig_portrait = _fig_phase_portrait(df)
    fig_timing = _fig_timing(timing_df)

    tabs = [
        ("Overview", "tab-overview"),
        ("📊 Numbers", "tab-numbers"),
        ("Trajectory", "tab-traj"),
        ("3D View", "tab-3d"),
        ("Attitude", "tab-att"),
        ("Aero & Mass", "tab-aero"),
        ("Forces", "tab-forces"),
        ("Control", "tab-ctrl"),
        ("Phase Portrait", "tab-portrait"),
    ]
    if fig_timing:
        tabs.append(("Timing (ARM64)", "tab-timing"))
    if fig_geo:
        tabs.append(("Ground Track", "tab-geo"))

    tabs_header = '<div class="tabs">'
    for i, (label, tid) in enumerate(tabs):
        active = " active" if i == 0 else ""
        tabs_header += f'<button class="tab-btn{active}" onclick="openTab(event,\'{tid}\')">{label}</button>'
    tabs_header += '</div>'

    overview_grid = (
        f'<div class="grid grid-3" style="margin-bottom:16px">'
        f'{score_html}{checks_html}'
        f'<div class="card"><h3 style="margin-bottom:8px">Diagnostics</h3>{diag_html}</div>'
        f'</div>'
    )
    if timing_card:
        overview_grid += f'<div class="grid grid-2" style="margin-bottom:16px">{timing_card}</div>'

    tab_overview = f"""
    <div id="tab-overview" class="tab-panel active">
      <h2>Mission Summary</h2>
      {overview_grid}
      <div class="card" style="margin-bottom:16px"><h3 style="margin-bottom:8px">Recommendations</h3>{rec_html}</div>
      <h2>Flight Metrics</h2>
      {key_cards}
    </div>"""

    tab_numbers = f'<div id="tab-numbers" class="tab-panel"><h2>📊 All Numerical Metrics</h2>{_metrics_table_html(m, scores)}</div>'
    tab_traj = f'<div id="tab-traj" class="tab-panel"><h2>Trajectory</h2>{_plotly_div(fig_traj)}</div>'
    tab_3d = f'<div id="tab-3d" class="tab-panel"><h2>3D Trajectory</h2>{_plotly_div(fig_3d)}</div>'
    tab_att = f'<div id="tab-att" class="tab-panel"><h2>Attitude & Angular Rates</h2>{_plotly_div(fig_att)}</div>'
    tab_aero = f'<div id="tab-aero" class="tab-panel"><h2>Aerodynamics & Mass</h2>{_plotly_div(fig_aero)}</div>'
    tab_forces = f'<div id="tab-forces" class="tab-panel"><h2>Forces & Energy</h2>{_plotly_div(fig_forces)}</div>'
    tab_ctrl = f'<div id="tab-ctrl" class="tab-panel"><h2>Control & Actuators (PX4 PIL)</h2>{_plotly_div(fig_ctrl)}</div>'
    tab_portrait = f'<div id="tab-portrait" class="tab-panel"><h2>Phase Portraits</h2>{_plotly_div(fig_portrait)}</div>'
    tab_timing = ""
    if fig_timing:
        tab_timing = f'<div id="tab-timing" class="tab-panel"><h2>ARM64 Realtime Timing</h2>{_plotly_div(fig_timing)}</div>'
    tab_geo = ""
    if fig_geo:
        tab_geo = f'<div id="tab-geo" class="tab-panel"><h2>Ground Track (Map)</h2>{_plotly_div(fig_geo)}</div>'

    toolbar = (
        '<div class="toolbar">'
        '<button id="theme-toggle" class="theme-toggle">&#9790; Dark Mode</button>'
        '<span class="pil-banner">PIL — Processor-In-the-Loop (ARM64)</span>'
        '<span style="font-size:.8rem;color:var(--text-secondary)">PX4 running on real hardware</span>'
        '</div>'
    )

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>M130 PIL Analysis &mdash; {html_escape(m['timestamp'])}</title>
{plotly_cdn}
<style>{_CSS}</style>
</head>
<body>
<div class="container">
  <h1>M130 PIL Flight Analysis</h1>
  {toolbar}
  <div style="color:var(--text-secondary);font-size:.85rem;margin-bottom:16px">
    File: {html_escape(m['file'])} | Generated: {now}
  </div>
  {key_cards}
  <div class="tab-container">
    {tabs_header}
    {tab_overview}
    {tab_numbers}
    {tab_traj}
    {tab_3d}
    {tab_att}
    {tab_aero}
    {tab_forces}
    {tab_ctrl}
    {tab_portrait}
    {tab_timing}
    {tab_geo}
  </div>
</div>
<script>{_JS}</script>
</body>
</html>"""

    if html_path:
        Path(html_path).write_text(html, encoding="utf-8")
    return html


# ══════════════════════════════════════════════════════════════════════
#  Console Summary
# ══════════════════════════════════════════════════════════════════════

def print_console_summary(m, scores):
    tag = {"PASS": "✅", "WARN": "⚠️", "FAIL": "❌"}[scores["overall"]]
    print()
    print("═" * 62)
    print(f"  M130 PIL Analysis (ARM64)  {tag} {scores['overall']} ({scores['total']}/100)")
    print("═" * 62)
    print(f"  Range:     {m['impact_range_m']:.0f}m  (err {m['range_error_pct']:+.1f}%)")
    print(f"  Peak Alt:  {m['peak_alt_m']:.0f}m AGL   Max Mach: {m['max_mach']:.3f}   Max G: {m['max_g']:.1f}")
    print(f"  Time:      {m['flight_time_s']:.2f}s   "
          f"Max |α|(flight): {m.get('max_alpha_flight_deg', m['max_alpha_deg']):.1f}°   "
          f"Max fin: {m['max_fin_cmd_deg']:.1f}°")
    print(f"  Mass:      {m['mass_initial']:.2f} → {m['mass_final']:.2f} kg   "
          f"Fin saturation: {m['fin_saturation_pct']:.1f}%")
    if m.get("timing_samples", 0) > 0:
        print(f"  MPC timing: avg={m['mpc_us_avg']/1000:.1f}ms  max={m['mpc_us_max']/1000:.1f}ms  "
              f"over-deadline: {m.get('mpc_over_deadline_pct',0):.0f}%")
    else:
        print(f"  MPC timing: (no timing samples — check adb reverse tcp:5760)")
    print("═" * 62)


# ══════════════════════════════════════════════════════════════════════
#  Entry Points
# ══════════════════════════════════════════════════════════════════════

def analyze_pil_csv(csv_path, open_browser=True):
    csv_path = Path(csv_path)
    df = load_pil_csv(csv_path)
    timing_df = load_pil_timing(csv_path)
    metrics = extract_metrics(df, csv_path, timing_df=timing_df)
    scores = score_run(metrics)
    diags = diagnose(df, metrics)
    recs = recommend(metrics, scores, diags)

    out_dir = csv_path.parent / "plots"
    out_dir.mkdir(parents=True, exist_ok=True)
    html_path = out_dir / f"pil_analysis_{metrics['timestamp']}.html"

    generate_html(df, metrics, scores, diags, recs, timing_df=timing_df,
                  html_path=str(html_path))
    print_console_summary(metrics, scores)
    print(f"  HTML → {html_path}")

    if open_browser:
        webbrowser.open(f"file://{html_path.resolve()}")

    return metrics, scores, str(html_path)


def discover_pil_csvs(results_dir: Path) -> list:
    return sorted(results_dir.glob("pil_*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)


def main():
    parser = argparse.ArgumentParser(description="M130 PIL Flight — Interactive HTML Analysis")
    parser.add_argument("--file", type=str, default=None, help="Analyze specific CSV")
    parser.add_argument("--no-open", action="store_true", help="Don't open browser")
    args = parser.parse_args()

    if args.file:
        csv_files = [Path(args.file)]
    else:
        csv_files = discover_pil_csvs(_RESULTS_DIR)
        # exclude timing csv
        csv_files = [p for p in csv_files if "timing" not in p.name]

    if not csv_files:
        print("ERROR: no pil_*.csv files found in", _RESULTS_DIR)
        sys.exit(1)

    print(f"  Analyzing: {csv_files[0].name}")
    analyze_pil_csv(csv_files[0], open_browser=not args.no_open)


if __name__ == "__main__":
    main()
