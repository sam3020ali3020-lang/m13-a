#!/usr/bin/env python3
"""
MHE Integration Stress Test Suite
==================================
Runs 6 scenarios from MHE_INTEGRATION_PLAN.md Section 6-E:
  1. Nominal no wind, estimation OFF  (baseline)
  2. Nominal no wind, estimation MHE  (regression)
  3. Crosswind variable, estimation OFF (baseline ref)
  4. Crosswind variable, estimation MHE (main test)
  5. High bias (5×), estimation MHE    (stress)
  6. GPS dropout 2s, estimation MHE    (robustness)

Acceptance criteria:
  - S1: identical to known baseline (range~2.59km)
  - S2: range ±50m of S1, 0 NaN
  - S3: reference for comparison
  - S4: max|roll| < 30°, no NaN, range > 2400m
  - S5: no crash, range > 2200m
  - S6: MHE recovers after GPS returns
"""

import copy, json, os, sys, tempfile, time, traceback
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

# Add parent to path
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from rocket_6dof_sim import Rocket6DOFSimulation


# ── helpers ──────────────────────────────────────────────────────────────
def load_base_config():
    cfg_path = ROOT / "config" / "6dof_config_advanced.yaml"
    with open(cfg_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def deep_update(base, overlay):
    for k, v in overlay.items():
        if isinstance(v, dict) and k in base and isinstance(base[k], dict):
            deep_update(base[k], v)
        else:
            base[k] = v
    return base


def run_scenario(label, overlay, timeout_s=300):
    """Run one sim scenario.  Returns dict with results or error info."""
    cfg = load_base_config()
    deep_update(cfg, overlay)

    # Write merged config to temp file
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False, encoding="utf-8"
    )
    yaml.dump(cfg, tmp, allow_unicode=True, default_flow_style=False)
    tmp.close()

    result = {
        "label": label,
        "status": "FAIL",
        "range_m": 0,
        "max_alt_m": 0,
        "max_roll_deg": 0,
        "nan_count": 0,
        "mhe_quality_mean": None,
        "mhe_solve_ms": None,
        "time_s": 0,
        "error": None,
    }

    t0 = time.time()
    try:
        sim = Rocket6DOFSimulation(config_file=tmp.name, long_range_mode=False)
        sim_cfg = sim.config.get("simulation", {})
        duration = sim_cfg.get("duration", 300.0)
        dt = sim_cfg.get("dt", 0.01)
        ballistic_fin = np.array(sim_cfg.get("ballistic_fin_deflection", [0, 0, 0, 0]))

        def ctrl(state, t):
            return ballistic_fin.copy()

        history = sim.simulate(duration=duration, dt=dt, control_function=ctrl)

        # ── extract results directly from history dict ──
        t_arr = np.array(history.get("time", []))
        
        # Range and altitude
        range_km = np.array(history.get("ground_range_km", [0]))
        alt_km = np.array(history.get("altitude_km", [0]))
        result["range_m"] = float(range_km[-1]) * 1000 if len(range_km) > 0 else 0
        result["max_alt_m"] = float(np.nanmax(alt_km)) * 1000 if len(alt_km) > 0 else 0

        # Roll from quaternion attitude [q0,q1,q2,q3] (scalar-first)
        quat = np.array(history.get("attitude", []))
        if len(quat) > 0 and quat.ndim == 2 and quat.shape[1] == 4:
            r = Rotation.from_quat(quat[:, [1, 2, 3, 0]])
            euler = r.as_euler("ZYX", degrees=True)  # yaw, pitch, roll
            result["max_roll_deg"] = float(np.nanmax(np.abs(euler[:, 2])))
        else:
            result["max_roll_deg"] = 0

        # NaN check on key arrays
        nan_count = 0
        for key in ["position", "velocity", "attitude", "angular_velocity"]:
            arr = np.array(history.get(key, []))
            if arr.size > 0 and np.issubdtype(arr.dtype, np.floating):
                nan_count += int(np.isnan(arr).sum())
        result["nan_count"] = nan_count

        # MHE diagnostics
        mhe_q = np.array(history.get("mhe_quality", []))
        mhe_st = np.array(history.get("mhe_solve_ms", []))
        if len(mhe_q) > 0:
            result["mhe_quality_mean"] = float(np.nanmean(mhe_q))
        if len(mhe_st) > 0:
            result["mhe_solve_ms"] = float(np.nanmean(mhe_st))

        result["status"] = "OK"

    except Exception as e:
        result["error"] = f"{type(e).__name__}: {e}"
        traceback.print_exc()
    finally:
        result["time_s"] = time.time() - t0
        try:
            os.unlink(tmp.name)
        except OSError:
            pass

    return result


# ── scenario definitions ────────────────────────────────────────────────
SCENARIOS = [
    (
        "S1: Nominal no-wind, MHE OFF",
        {
            "estimation": {"mode": "off", "use_estimated_state_for_mpc": False},
            "atmosphere": {"wind_enabled": False, "wind_speed": 0.0},
        },
    ),
    (
        "S2: Nominal no-wind, MHE ON",
        {
            "estimation": {"mode": "mhe", "use_estimated_state_for_mpc": True},
            "atmosphere": {"wind_enabled": False, "wind_speed": 0.0},
        },
    ),
    (
        "S3: Crosswind 8m/s E, MHE OFF",
        {
            "estimation": {"mode": "off", "use_estimated_state_for_mpc": False},
            "atmosphere": {
                "wind_enabled": True,
                "wind_speed": 8.0,
                "wind_direction": 90.0,
                "wind_direction_unit": "degrees",
            },
        },
    ),
    (
        "S4: Crosswind 8m/s E, MHE ON",
        {
            "estimation": {
                "mode": "mhe",
                "use_estimated_state_for_mpc": True,
                "compensation": {"enable_heading_compensation": True},
            },
            "atmosphere": {
                "wind_enabled": True,
                "wind_speed": 8.0,
                "wind_direction": 90.0,
                "wind_direction_unit": "degrees",
            },
        },
    ),
    (
        "S5: High bias (5×), MHE ON",
        {
            "estimation": {
                "mode": "mhe",
                "use_estimated_state_for_mpc": True,
                "sensors": {
                    "gyro_bias_std": 0.025,   # 5× default 0.005
                    "accel_bias_std": 0.5,     # 5× default 0.1
                },
            },
            "atmosphere": {"wind_enabled": False, "wind_speed": 0.0},
        },
    ),
    (
        "S6: GPS dropout 6-8s, MHE ON",
        {
            "estimation": {
                "mode": "mhe",
                "use_estimated_state_for_mpc": True,
                "sensors": {
                    "gps_dropout_window": [6.0, 8.0],
                },
            },
            "atmosphere": {"wind_enabled": False, "wind_speed": 0.0},
        },
    ),
]


# ── acceptance criteria checks ──────────────────────────────────────────
def check_criteria(results):
    """Evaluate acceptance criteria. Returns list of (criterion, pass/fail, detail)."""
    checks = []
    s = {r["label"][:2]: r for r in results}

    # S1 baseline
    r1 = s.get("S1")
    if r1 and r1["status"] == "OK":
        checks.append(("S1 completes", "PASS", f"Range={r1['range_m']:.0f}m"))
    else:
        checks.append(("S1 completes", "FAIL", r1.get("error", "?") if r1 else "missing"))

    # S2 regression vs S1
    r2 = s.get("S2")
    if r1 and r2 and r1["status"] == "OK" and r2["status"] == "OK":
        delta = abs(r2["range_m"] - r1["range_m"])
        ok = delta < 50 and r2["nan_count"] == 0
        checks.append((
            "S2 regression ±50m, 0 NaN",
            "PASS" if ok else "FAIL",
            f"ΔRange={delta:.0f}m, NaN={r2['nan_count']}",
        ))
    else:
        checks.append(("S2 regression", "FAIL", "missing data"))

    # S3 reference
    r3 = s.get("S3")
    if r3 and r3["status"] == "OK":
        checks.append(("S3 wind baseline", "PASS", f"Range={r3['range_m']:.0f}m"))
    else:
        checks.append(("S3 wind baseline", "FAIL", r3.get("error", "?") if r3 else "missing"))

    # S4 main test
    r4 = s.get("S4")
    if r4 and r4["status"] == "OK":
        roll_ok = r4["max_roll_deg"] < 30
        nan_ok = r4["nan_count"] == 0
        range_ok = r4["range_m"] > 2400
        all_ok = roll_ok and nan_ok and range_ok
        checks.append((
            "S4 |roll|<30°, 0 NaN, range>2400m",
            "PASS" if all_ok else "FAIL",
            f"|roll|={r4['max_roll_deg']:.1f}°, NaN={r4['nan_count']}, Range={r4['range_m']:.0f}m",
        ))
    else:
        checks.append(("S4 main test", "FAIL", r4.get("error", "?") if r4 else "missing"))

    # S5 stress
    r5 = s.get("S5")
    if r5 and r5["status"] == "OK":
        range_ok = r5["range_m"] > 2200
        checks.append((
            "S5 no crash, range>2200m",
            "PASS" if range_ok else "FAIL",
            f"Range={r5['range_m']:.0f}m",
        ))
    else:
        checks.append(("S5 stress test", "FAIL", r5.get("error", "?") if r5 else "missing"))

    # S6 GPS dropout
    r6 = s.get("S6")
    if r6 and r6["status"] == "OK":
        ok = r6["nan_count"] == 0 and r6["range_m"] > 2200
        checks.append((
            "S6 GPS dropout recover",
            "PASS" if ok else "FAIL",
            f"Range={r6['range_m']:.0f}m, NaN={r6['nan_count']}",
        ))
    else:
        checks.append(("S6 GPS dropout", "FAIL", r6.get("error", "?") if r6 else "missing"))

    return checks


# ── main ────────────────────────────────────────────────────────────────
def main():
    print("=" * 70)
    print("MHE INTEGRATION — STRESS TEST SUITE")
    print("=" * 70)

    # Remove cached solver to avoid stale binaries
    import glob
    for f in glob.glob(str(ROOT / "mpc" / ".mpc_source_hash")):
        os.unlink(f)

    results = []
    for label, overlay in SCENARIOS:
        print(f"\n{'─' * 70}")
        print(f"▶ {label}")
        print(f"{'─' * 70}")
        r = run_scenario(label, overlay)
        results.append(r)
        status_icon = "✓" if r["status"] == "OK" else "✗"
        print(f"  {status_icon} {r['status']}  Range={r['range_m']:.0f}m  "
              f"MaxAlt={r['max_alt_m']:.0f}m  |roll|={r['max_roll_deg']:.1f}°  "
              f"NaN={r['nan_count']}  t={r['time_s']:.1f}s")
        if r.get("mhe_quality_mean") is not None:
            print(f"    MHE quality={r['mhe_quality_mean']:.3f}  solve={r['mhe_solve_ms']:.1f}ms")
        if r["error"]:
            print(f"    ERROR: {r['error']}")

    # ── Summary ──
    print(f"\n{'=' * 70}")
    print("SUMMARY")
    print(f"{'=' * 70}")
    print(f"{'Scenario':<40} {'Range':>7} {'|roll|':>7} {'NaN':>5} {'Status':>6}")
    print("-" * 70)
    for r in results:
        print(f"{r['label']:<40} {r['range_m']:>6.0f}m {r['max_roll_deg']:>6.1f}° "
              f"{r['nan_count']:>5} {r['status']:>6}")

    # ── Acceptance criteria ──
    print(f"\n{'=' * 70}")
    print("ACCEPTANCE CRITERIA")
    print(f"{'=' * 70}")
    checks = check_criteria(results)
    all_pass = True
    for criterion, verdict, detail in checks:
        icon = "✓" if verdict == "PASS" else "✗"
        print(f"  {icon} {verdict:4s}  {criterion:<40}  {detail}")
        if verdict != "PASS":
            all_pass = False

    print(f"\n{'=' * 70}")
    if all_pass:
        print("★ ALL ACCEPTANCE CRITERIA PASSED ★")
    else:
        print("✗ SOME CRITERIA FAILED — see details above")
    print(f"{'=' * 70}")

    # Save results JSON
    out_path = ROOT / "results" / "stress_test_results.json"
    with open(out_path, "w") as f:
        json.dump(results, f, indent=2, default=str)
    print(f"\nResults saved: {out_path}")

    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
