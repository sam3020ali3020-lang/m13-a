#!/usr/bin/env python3
"""
ground_runner.py — مشغّل الاختبار الأرضي التكاملي
===================================================

يشغّل الهاتف في وضع Real Flight مع حساسات حقيقية ويراقب:
  1. تقارب EKF2 وصحته
  2. توقيت MPC/MHE (من RktGNC)
  3. حمل CPU وحرارة
  4. استقرار الاتجاه (drift)

الاستخدام:
    python3 ground_runner.py                     # standard (5 min)
    python3 ground_runner.py --preset quick       # 2 min
    python3 ground_runner.py --preset preflight   # 1 min (pre-launch)
    python3 ground_runner.py --preset extended    # 15 min (thermal)
    python3 ground_runner.py --duration 600       # custom
    python3 ground_runner.py --compare-pil ../pil/results/pil_timing.csv
"""

from __future__ import annotations

import argparse
import json
import logging
import socket
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import yaml

_SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_SCRIPT_DIR))

from ground_reader import GroundReader  # noqa: E402
from sensor_reader import build_heartbeat_gcs  # noqa: E402

logger = logging.getLogger("ground_runner")


# ============================================================================
# EKF2 flags decoder
# ============================================================================

EKF2_FLAGS = {
    0x0001: "attitude",
    0x0002: "velocity_horiz",
    0x0004: "velocity_vert",
    0x0008: "pos_horiz_rel",
    0x0010: "pos_horiz_abs",
    0x0020: "pos_vert_abs",
    0x0040: "pos_vert_agl",
    0x0080: "const_pos_mode",
    0x0100: "pred_pos_horiz_rel",
    0x0200: "pred_pos_horiz_abs",
    0x0400: "gps_glitch",
    0x0800: "accel_error",
}


def decode_ekf_flags(flags: int) -> List[str]:
    active = []
    for bit, name in EKF2_FLAGS.items():
        if flags & bit:
            active.append(name)
    return active


# ============================================================================
# Analysis functions
# ============================================================================

def analyze_ekf2(reader: GroundReader, thresholds: dict, warmup_s: float) -> dict:
    """Analyze EKF2 convergence and health."""
    th = thresholds.get("ekf2", {})
    metrics = {}
    failures = []

    if not reader.estimator_samples:
        return {"error": "No ESTIMATOR_STATUS received", "passed": None, "failures": ["No data"]}

    # Trim warmup
    t0 = reader.estimator_samples[0].t_wall_s
    samples = reader.estimator_samples

    # ── Convergence time ──
    # EKF2 "converged" = flags has attitude + velocity + pos bits
    required_flags = th.get("required_flags", 0x0F)
    convergence_time = None
    for s in samples:
        if (s.flags & required_flags) == required_flags:
            convergence_time = s.t_wall_s - t0
            break

    if convergence_time is not None:
        metrics["convergence_time_s"] = round(convergence_time, 1)
        max_conv = th.get("max_convergence_time_s", 30.0)
        if convergence_time > max_conv:
            failures.append(f"EKF2 convergence too slow: {convergence_time:.1f}s > {max_conv}s")
    else:
        metrics["convergence_time_s"] = None
        failures.append("EKF2 never converged (required flags not set)")

    # ── Post-warmup analysis ──
    post = [s for s in samples if s.t_wall_s >= t0 + warmup_s]
    if post:
        vel_ratios = [s.vel_ratio for s in post]
        pos_h_ratios = [s.pos_horiz_ratio for s in post]
        pos_v_ratios = [s.pos_vert_ratio for s in post]
        mag_ratios = [s.mag_ratio for s in post]
        pos_h_acc = [s.pos_horiz_accuracy for s in post]
        pos_v_acc = [s.pos_vert_accuracy for s in post]
        flags_list = [s.flags for s in post]

        metrics["vel_ratio_mean"] = round(float(np.mean(vel_ratios)), 3)
        metrics["vel_ratio_max"] = round(float(np.max(vel_ratios)), 3)
        metrics["pos_horiz_ratio_mean"] = round(float(np.mean(pos_h_ratios)), 3)
        metrics["pos_vert_ratio_mean"] = round(float(np.mean(pos_v_ratios)), 3)
        metrics["mag_ratio_mean"] = round(float(np.mean(mag_ratios)), 3)
        metrics["pos_horiz_accuracy_mean_m"] = round(float(np.mean(pos_h_acc)), 2)
        metrics["pos_vert_accuracy_mean_m"] = round(float(np.mean(pos_v_acc)), 2)
        metrics["flags_final"] = f"0x{flags_list[-1]:04X}"
        metrics["flags_decoded"] = decode_ekf_flags(flags_list[-1])

        # Check innovation ratios
        for name, vals, key in [
            ("vel", vel_ratios, "max_vel_ratio"),
            ("pos_horiz", pos_h_ratios, "max_pos_horiz_ratio"),
            ("pos_vert", pos_v_ratios, "max_pos_vert_ratio"),
            ("mag", mag_ratios, "max_mag_ratio"),
        ]:
            mean_val = float(np.mean(vals))
            max_val = float(np.max(vals))
            limit = th.get(key, 1.0)
            if mean_val > limit:
                failures.append(f"EKF2 {name} ratio too high: mean={mean_val:.2f} > {limit}")

        # Check accuracy
        if np.mean(pos_h_acc) > th.get("max_pos_horiz_accuracy_m", 5.0):
            failures.append(f"Horizontal accuracy poor: {np.mean(pos_h_acc):.1f}m")
        if np.mean(pos_v_acc) > th.get("max_pos_vert_accuracy_m", 10.0):
            failures.append(f"Vertical accuracy poor: {np.mean(pos_v_acc):.1f}m")

    # ── Attitude drift ──
    if reader.attitude_samples:
        att_post = [s for s in reader.attitude_samples if s.t_wall_s >= t0 + warmup_s]
        if len(att_post) > 10:
            rolls = np.degrees([s.roll for s in att_post])
            pitchs = np.degrees([s.pitch for s in att_post])
            yaws = np.degrees([s.yaw for s in att_post])

            # Drift = max deviation from mean
            roll_drift = float(np.max(np.abs(rolls - np.mean(rolls))))
            pitch_drift = float(np.max(np.abs(pitchs - np.mean(pitchs))))
            att_drift = max(roll_drift, pitch_drift)

            # Yaw drift (handle wrap-around) — s.yaw is already in radians
            yaw_unwrapped = np.unwrap([s.yaw for s in att_post])
            yaw_drift = float(np.degrees(np.max(yaw_unwrapped) - np.min(yaw_unwrapped)))

            metrics["roll_mean_deg"] = round(float(np.mean(rolls)), 2)
            metrics["pitch_mean_deg"] = round(float(np.mean(pitchs)), 2)
            metrics["yaw_mean_deg"] = round(float(np.mean(yaws)), 2)
            metrics["attitude_drift_deg"] = round(att_drift, 2)
            metrics["yaw_drift_deg"] = round(yaw_drift, 2)

            max_att = th.get("max_attitude_drift_deg", 2.0)
            if att_drift > max_att:
                failures.append(f"Attitude drift: {att_drift:.1f}° > {max_att}°")
            max_yaw = th.get("max_yaw_drift_deg", 5.0)
            if yaw_drift > max_yaw:
                failures.append(f"Yaw drift: {yaw_drift:.1f}° > {max_yaw}°")

    metrics["n_samples"] = len(reader.estimator_samples)
    metrics["passed"] = len(failures) == 0
    metrics["failures"] = failures
    return metrics


def analyze_timing(reader: GroundReader, thresholds: dict, warmup_s: float) -> dict:
    """Analyze MPC/MHE timing."""
    th = thresholds.get("mpc", {})
    metrics = {}
    failures = []

    if not reader.timing_samples:
        return {"error": "No RktGNC timing received (MPC not running?)",
                "passed": None, "failures": ["No timing data — MPC may not be armed/running"]}

    t0 = reader.timing_samples[0].t_wall_s
    post = [s for s in reader.timing_samples if s.t_wall_s >= t0 + warmup_s]

    if not post:
        return {"error": "No post-warmup timing data", "passed": None, "failures": ["No data"]}

    mpc_us = np.array([s.mpc_solve_us for s in post])
    mhe_us = np.array([s.mhe_solve_us for s in post])
    cycle_us = np.array([s.cycle_us for s in post])

    mpc_ms = mpc_us / 1000.0
    mhe_ms = mhe_us / 1000.0
    cycle_ms = cycle_us / 1000.0

    metrics["mpc_solve_mean_ms"] = round(float(np.mean(mpc_ms)), 2)
    metrics["mpc_solve_p50_ms"] = round(float(np.percentile(mpc_ms, 50)), 2)
    metrics["mpc_solve_p95_ms"] = round(float(np.percentile(mpc_ms, 95)), 2)
    metrics["mpc_solve_p99_ms"] = round(float(np.percentile(mpc_ms, 99)), 2)
    metrics["mpc_solve_max_ms"] = round(float(np.max(mpc_ms)), 2)

    metrics["mhe_solve_mean_ms"] = round(float(np.mean(mhe_ms)), 2)
    metrics["mhe_solve_p99_ms"] = round(float(np.percentile(mhe_ms, 99)), 2)
    metrics["mhe_solve_max_ms"] = round(float(np.max(mhe_ms)), 2)

    metrics["cycle_mean_ms"] = round(float(np.mean(cycle_ms)), 2)
    metrics["cycle_p99_ms"] = round(float(np.percentile(cycle_ms, 99)), 2)
    metrics["cycle_max_ms"] = round(float(np.max(cycle_ms)), 2)

    metrics["n_samples"] = len(post)
    metrics["timing_rate_hz"] = round(len(post) / (post[-1].t_wall_s - post[0].t_wall_s), 1) \
        if len(post) > 1 else 0.0

    # PASS/FAIL
    if np.percentile(mpc_ms, 99) > th.get("max_mpc_solve_p99_ms", 15.0):
        failures.append(f"MPC p99={np.percentile(mpc_ms,99):.1f}ms > "
                        f"{th.get('max_mpc_solve_p99_ms', 15.0)}ms")
    if np.mean(mpc_ms) > th.get("max_mpc_solve_mean_ms", 8.0):
        failures.append(f"MPC mean={np.mean(mpc_ms):.1f}ms > "
                        f"{th.get('max_mpc_solve_mean_ms', 8.0)}ms")
    if np.percentile(cycle_ms, 99) > th.get("max_cycle_p99_ms", 25.0):
        failures.append(f"Cycle p99={np.percentile(cycle_ms,99):.1f}ms > "
                        f"{th.get('max_cycle_p99_ms', 25.0)}ms")
    if np.percentile(mhe_ms, 99) > th.get("max_mhe_solve_p99_ms", 10.0):
        failures.append(f"MHE p99={np.percentile(mhe_ms,99):.1f}ms > "
                        f"{th.get('max_mhe_solve_p99_ms', 10.0)}ms")

    metrics["passed"] = len(failures) == 0
    metrics["failures"] = failures
    return metrics


def analyze_system(reader: GroundReader, thresholds: dict, warmup_s: float) -> dict:
    """Analyze CPU load and temperature."""
    th = thresholds.get("system", {})
    metrics = {}
    failures = []

    # CPU load from SYS_STATUS
    if reader.sys_status_samples:
        t0 = reader.sys_status_samples[0].t_wall_s
        post = [s for s in reader.sys_status_samples if s.t_wall_s >= t0 + warmup_s]
        if post:
            loads = [s.load / 10.0 for s in post]  # permille → %
            metrics["cpu_load_mean_pct"] = round(float(np.mean(loads)), 1)
            metrics["cpu_load_max_pct"] = round(float(np.max(loads)), 1)
            metrics["cpu_load_min_pct"] = round(float(np.min(loads)), 1)

            max_cpu = th.get("max_cpu_load_pct", 80.0)
            if np.mean(loads) > max_cpu:
                failures.append(f"CPU load too high: {np.mean(loads):.0f}% > {max_cpu}%")

            # Battery (if available)
            voltages = [s.voltage_battery for s in post if s.voltage_battery > 0]
            if voltages:
                metrics["battery_voltage_mV"] = int(np.mean(voltages))

    # Temperature from IMU HIGHRES_IMU
    if reader.imu_samples:
        t0 = reader.imu_samples[0].t_wall_s
        post = [s for s in reader.imu_samples if s.t_wall_s >= t0 + warmup_s]
        if post:
            temps = [s.temperature for s in post]
            if any(t != 0.0 for t in temps):
                metrics["temperature_start_C"] = round(float(temps[0]), 1)
                metrics["temperature_end_C"] = round(float(temps[-1]), 1)
                metrics["temperature_max_C"] = round(float(np.max(temps)), 1)
                metrics["temperature_rise_C"] = round(float(temps[-1] - temps[0]), 1)

                max_temp = th.get("max_temperature_C", 55.0)
                if np.max(temps) > max_temp:
                    failures.append(f"Temperature too high: {np.max(temps):.0f}°C > {max_temp}°C")
                max_rise = th.get("max_temperature_rise_C", 15.0)
                temp_rise = temps[-1] - temps[0]
                if temp_rise > max_rise:
                    failures.append(f"Temperature rise: {temp_rise:.0f}°C > {max_rise}°C")

    metrics["n_sys_status"] = len(reader.sys_status_samples)
    metrics["passed"] = len(failures) == 0
    metrics["failures"] = failures
    return metrics


def compare_with_pil(timing_metrics: dict, pil_csv_path: Path,
                     thresholds: dict) -> dict:
    """Compare ground timing with PIL results."""
    th = thresholds.get("pil_comparison", {})
    metrics = {}
    failures = []

    try:
        pil_data = np.genfromtxt(pil_csv_path, delimiter=",", names=True,
                                  dtype=None, encoding="utf-8")
    except Exception as e:
        return {"error": f"Cannot read PIL CSV: {e}", "passed": None}

    # Find MPC column in PIL data
    pil_mpc_ms = None
    for col_name in ["mpc_solve_us", "mpc_us"]:
        if col_name in pil_data.dtype.names:
            pil_mpc_ms = pil_data[col_name].astype(float) / 1000.0
            break

    if pil_mpc_ms is None or len(pil_mpc_ms) == 0:
        return {"error": "No MPC timing in PIL CSV", "passed": None}

    pil_mean = float(np.mean(pil_mpc_ms[pil_mpc_ms > 0]))
    ground_mean = timing_metrics.get("mpc_solve_mean_ms", 0)

    if pil_mean > 0 and ground_mean > 0:
        increase_pct = 100.0 * (ground_mean - pil_mean) / pil_mean
        metrics["pil_mpc_mean_ms"] = round(pil_mean, 2)
        metrics["ground_mpc_mean_ms"] = round(ground_mean, 2)
        metrics["mpc_increase_pct"] = round(increase_pct, 1)

        max_inc = th.get("max_mpc_time_increase_pct", 30.0)
        if increase_pct > max_inc:
            failures.append(f"MPC {increase_pct:.0f}% slower than PIL (limit {max_inc}%)")

    metrics["passed"] = len(failures) == 0
    metrics["failures"] = failures
    return metrics


# ============================================================================
# Main runner
# ============================================================================

def run_ground_test(config: dict, result_dir: Path,
                    pil_csv: Optional[Path] = None,
                    auto_arm: bool = False) -> bool:
    """Run the ground integration test."""

    conn = config.get("connection", {})
    test_cfg = config.get("test", {})
    thresholds = config.get("thresholds", {})

    duration = test_cfg.get("duration_s", 300)
    warmup = test_cfg.get("warmup_s", 15)

    # Auto-arm can come from config or CLI flag
    if not auto_arm:
        auto_arm = test_cfg.get("auto_arm", False)

    print(f"\n{'='*60}")
    print(f"  /ground — Ground Integration Test")
    print(f"  حساسات حقيقية + EKF2 + MPC — بدون طيران")
    print(f"{'='*60}")
    print(f"  Duration:  {duration}s ({duration/60:.0f} min)")
    print(f"  Warmup:    {warmup}s")
    print(f"  Auto-arm:  {'ON (force-arm)' if auto_arm else 'OFF (arm manually from QGC)'}")
    print(f"  Results:   {result_dir}")
    print(f"{'='*60}")
    print(f"\n  تأكد أن:")
    print(f"    1. الهاتف يعمل بـ Airframe 22002 (Real Flight)")
    print(f"    2. الهاتف ثابت على سطح مستوٍ")
    print(f"    3. GPS USB متصل (إذا متوفر)")
    print(f"    4. adb forward tcp:5760 tcp:5760")
    print()

    # Connect
    reader = GroundReader(
        host=conn.get("host", "127.0.0.1"),
        port=conn.get("port", 5760),
        timeout_s=conn.get("timeout_s", 10.0),
    )

    if not reader.connect():
        print(f"\n  [FAIL] لا يمكن الاتصال بـ {conn.get('host')}:{conn.get('port')}")
        return False

    # Send heartbeat immediately to keep PX4 from closing the connection
    reader._send(build_heartbeat_gcs())

    # Request streams
    reader.request_streams_from_config(config)

    # Send another heartbeat, then wait for streams to start
    reader._send(build_heartbeat_gcs())
    time.sleep(2)

    # ── Live monitoring during recording ──
    print(f"\n  Recording ({duration}s) ...")
    print(f"  {'─'*50}")

    reader.clear()
    reader._sock.settimeout(0.5)
    t_start = time.monotonic()
    t_last_hb = 0.0
    t_last_status = 0.0
    t_last_arm = 0.0
    arm_attempt = 0
    arm_reported = False
    nav_reported = False
    launch_reported = False

    while True:
        now = time.monotonic()
        elapsed = now - t_start
        if elapsed >= duration + warmup:
            break

        # Heartbeat
        if now - t_last_hb >= 1.0:
            reader._send(build_heartbeat_gcs())
            t_last_hb = now

        # Auto-arm: send ARM command every 2s until armed
        if auto_arm and not reader.is_armed and now - t_last_arm >= 2.0:
            reader.send_arm(force=True)
            arm_attempt += 1
            t_last_arm = now
            if arm_attempt <= 3 or arm_attempt % 5 == 0:
                print(f"  {elapsed:5.0f}s | ARM attempt #{arm_attempt} ...")

        # Report when armed
        if reader.is_armed and not arm_reported:
            arm_reported = True
            print(f"  {elapsed:5.0f}s | ✅ ARMED (after {arm_attempt} attempts"
                  f"{', ACK received' if reader.arm_ack_received else ', via HEARTBEAT'})")

        # Report arm-time navigation sanity (first GNC nav sample after arm)
        if reader.is_armed and not nav_reported and reader.gnc_nav_samples:
            nav = reader.gnc_nav_samples[-1]
            nav_reported = True
            bearing = nav.bearing_deg
            tgt_range = nav.target_range_remaining
            print(f"  {elapsed:5.0f}s | 📍 Bearing: {bearing:.1f}°  Target: {tgt_range:.0f}m", end="")
            if tgt_range > 0:
                print(f"  ✅ Target in front")
            else:
                print(f"  ❌ Target BEHIND (range ≤ 0)!")

        # Report when launch detected (MPC starts solving → mpc_solve_us > 0)
        if not launch_reported and reader.timing_samples:
            last_mpc_us = reader.timing_samples[-1].mpc_solve_us
            if last_mpc_us > 0:
                launch_reported = True
                print(f"  {elapsed:5.0f}s | 🚀 LAUNCH DETECTED — MPC solving ({last_mpc_us/1000:.1f}ms)")

        # Live status every 5s
        if now - t_last_status >= 5.0:
            n_imu = len(reader.imu_samples)
            n_est = len(reader.estimator_samples)
            n_tim = len(reader.timing_samples)
            n_sys = len(reader.sys_status_samples)

            # EKF2 flags
            ekf_str = "?"
            if reader.estimator_samples:
                last_flags = reader.estimator_samples[-1].flags
                ekf_str = f"0x{last_flags:04X}"

            # MPC timing
            mpc_str = "—"
            if reader.timing_samples:
                last_mpc = reader.timing_samples[-1].mpc_solve_us / 1000.0
                mpc_str = f"{last_mpc:.1f}ms"

            # CPU
            cpu_str = "—"
            if reader.sys_status_samples:
                cpu_str = f"{reader.sys_status_samples[-1].load/10.0:.0f}%"

            # Temperature
            temp_str = "—"
            if reader.imu_samples and reader.imu_samples[-1].temperature != 0:
                temp_str = f"{reader.imu_samples[-1].temperature:.0f}°C"

            # Armed indicator
            arm_str = "ARMED" if reader.is_armed else "DISARMED"

            print(f"  {elapsed:5.0f}s | IMU={n_imu:5d} EST={n_est:3d} "
                  f"TIM={n_tim:4d} | EKF={ekf_str} MPC={mpc_str} "
                  f"CPU={cpu_str} T={temp_str} [{arm_str}]")
            t_last_status = now

        # Receive
        try:
            data = reader._sock.recv(4096)
            if not data:
                print("  [WARN] Connection closed")
                break
            reader._process(data, now - reader._t0)
        except socket.timeout:
            continue
        except OSError:
            continue
        except Exception as e:
            logger.warning(f"Process error: {e}")
            continue

    reader.disconnect()

    # Save CSVs (all data including warmup — analysis functions trim internally)
    reader.save_all_ground(result_dir)

    # ── Analysis ──
    print(f"\n  {'='*50}")
    print(f"  ANALYSIS")
    print(f"  {'='*50}")

    ekf_results = analyze_ekf2(reader, thresholds, warmup)
    timing_results = analyze_timing(reader, thresholds, warmup)
    system_results = analyze_system(reader, thresholds, warmup)

    pil_results = None
    if pil_csv and pil_csv.exists():
        pil_results = compare_with_pil(timing_results, pil_csv, thresholds)

    # ── Print results ──
    all_pass = True

    print(f"\n  ── EKF2 ──")
    _print_section(ekf_results)
    if ekf_results.get("passed") is False:
        all_pass = False

    print(f"\n  ── MPC/MHE Timing ──")
    _print_section(timing_results)
    if timing_results.get("passed") is False:
        all_pass = False

    print(f"\n  ── System (CPU/Thermal) ──")
    _print_section(system_results)
    if system_results.get("passed") is False:
        all_pass = False

    if pil_results:
        print(f"\n  ── PIL Comparison ──")
        _print_section(pil_results)
        if pil_results.get("passed") is False:
            all_pass = False

    # Handle None (no data) cases
    for r in [ekf_results, timing_results, system_results]:
        if r.get("passed") is None:
            all_pass = False

    # ── Verdict ──
    verdict = "GO" if all_pass else "NO-GO"
    print(f"\n  {'='*50}")
    print(f"  VERDICT: {'✅' if all_pass else '❌'} {verdict}")
    print(f"  {'='*50}\n")

    # Save results
    all_metrics = {
        "ekf2": ekf_results,
        "timing": timing_results,
        "system": system_results,
        "arming": {
            "auto_arm_requested": auto_arm,
            "armed": reader.is_armed,
            "arm_ack_received": reader.arm_ack_received,
            "arm_attempts": arm_attempt if auto_arm else 0,
        },
    }
    if pil_results:
        all_metrics["pil_comparison"] = pil_results

    with open(result_dir / "ground_metrics.json", "w") as f:
        json.dump(all_metrics, f, indent=2, ensure_ascii=False, default=str)

    with open(result_dir / "GO_NOGO.txt", "w") as f:
        f.write(f"VERDICT: {verdict}\n")
        f.write(f"Timestamp: {datetime.now().isoformat()}\n\n")
        for section, data in all_metrics.items():
            f.write(f"[{'PASS' if data.get('passed') else 'FAIL'}] {section}\n")
            for fail in data.get("failures", []):
                f.write(f"  - {fail}\n")

    return all_pass


def _print_section(data: dict):
    """Pretty-print a results section."""
    passed = data.get("passed")
    status = "PASS" if passed else ("FAIL" if passed is False else "NO DATA")
    print(f"    Status: [{status}]")

    for key, val in data.items():
        if key in ("passed", "failures", "flags_decoded", "error"):
            continue
        if isinstance(val, float):
            print(f"    {key}: {val:.3f}")
        elif isinstance(val, list):
            print(f"    {key}: [{', '.join(str(v) for v in val[:6])}]")
        else:
            print(f"    {key}: {val}")

    if data.get("error"):
        print(f"    ERROR: {data['error']}")
    for fail in data.get("failures", []):
        print(f"    ❌ {fail}")
    if data.get("flags_decoded"):
        print(f"    EKF2 flags: {', '.join(data['flags_decoded'])}")


# ============================================================================
# CLI
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Ground Integration Test — حساسات حقيقية + EKF2 + MPC")
    parser.add_argument("--config", default=str(_SCRIPT_DIR / "ground_config.yaml"))
    parser.add_argument("--preset", choices=["quick", "standard", "extended", "preflight"],
                        default="standard")
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--results-dir", type=str, default=None)
    parser.add_argument("--compare-pil", type=str, default=None,
                        help="Path to PIL timing CSV for comparison")
    parser.add_argument("--arm", action="store_true",
                        help="Auto-arm PX4 (force-arm) to get MPC timing data")
    parser.add_argument("-v", "--verbose", action="store_true")

    args = parser.parse_args()

    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=level,
                        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s")

    with open(args.config, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    # Apply preset
    if args.preset != "standard":
        preset = config.get("presets", {}).get(args.preset, {})
        config.setdefault("test", {})
        for k, v in preset.items():
            config["test"][k] = v

    # Override duration
    if args.duration:
        config.setdefault("test", {})["duration_s"] = args.duration

    # Result directory
    if args.results_dir:
        result_dir = Path(args.results_dir)
    else:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        result_dir = _SCRIPT_DIR / "results" / ts
    result_dir.mkdir(parents=True, exist_ok=True)

    pil_csv = Path(args.compare_pil) if args.compare_pil else None

    success = run_ground_test(config, result_dir, pil_csv, auto_arm=args.arm)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
