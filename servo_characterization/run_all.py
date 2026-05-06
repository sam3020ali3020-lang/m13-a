#!/usr/bin/env python3
"""Top-level entry point: connect, run battery of servo tests, fit models,
emit a JSON summary + HTML report.

Usage:
  python3 run_all.py                                # all 4 fins, default port
  python3 run_all.py --port /dev/ttyACM0
  python3 run_all.py --fins 0,1                    # only test fins 0 and 1
  python3 run_all.py --tests step,chirp,staircase  # subset of tests
  python3 run_all.py --quick                       # faster smoke test
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import sys
import time
from dataclasses import asdict

import numpy as np

from xqpower import XqpowerBus
from tester import Tester, TestConfig
import signals
import analyze


DEFAULT_FINS = [0, 1, 2, 3]

STEP_AMPLITUDES_DEG = [1.0, 2.0, 5.0, 10.0, 15.0]
STEP_DURATION_S = 1.5
STEP_TIME_S = 0.3

SQUARE_FREQS_HZ = [0.5, 1.0, 2.0, 5.0, 10.0]
SQUARE_AMPLITUDE_DEG = 5.0
SQUARE_DURATION_S = 3.0

CHIRP_F_START_HZ = 0.2
CHIRP_F_END_HZ = 25.0
CHIRP_AMPLITUDE_DEG = 5.0
CHIRP_DURATION_S = 20.0

STAIRCASE_LEVELS_DEG = [0, 0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 15.0, 18.0,
                        15.0, 10.0, 5.0, 2.0, 1.0, 0.5, 0.25, 0.1, 0.0,
                        -0.1, -0.25, -0.5, -1.0, -2.0, -5.0, -10.0, -15.0, -18.0,
                        -15.0, -10.0, -5.0, -2.0, -1.0, -0.5, -0.25, -0.1, 0.0]
STAIRCASE_HOLD_S = 0.6

SLEW_AMPLITUDE_DEG = 18.0
SLEW_DURATION_S = 4.0

HYST_AMPLITUDE_DEG = 15.0
HYST_PERIOD_S = 8.0
HYST_DURATION_S = 16.0

NOISE_DURATION_S = 3.0

CROSSTALK_AMPLITUDE_DEG = 15.0

DEFAULT_FS_HZ = 200.0
DEFAULT_FB_INTERVAL_MS = 5


# ---------------------------------------------------------------------- test builders

def _zero_for_others(fin: int, n_servos: int, n_samples: int) -> dict[int, np.ndarray]:
    out = {}
    for s in range(n_servos):
        if s != fin:
            out[s] = np.zeros(n_samples, dtype=float)
    return out


def build_step_tests(fin: int, n_servos: int, fs_hz: float) -> list[TestConfig]:
    cfgs = []
    for amp in STEP_AMPLITUDES_DEG:
        for sign, label in ((+1, "pos"), (-1, "neg")):
            u = signals.step(0.0, sign * amp, STEP_TIME_S, STEP_DURATION_S, fs_hz)
            cmd = {fin: u}
            cmd.update(_zero_for_others(fin, n_servos, len(u)))
            cfgs.append(TestConfig(
                name=f"step_fin{fin}_{label}_{int(round(amp))}deg",
                fs_hz=fs_hz,
                cmd_per_fin=cmd,
                description=f"{'+' if sign > 0 else '-'}{amp}° step on fin {fin}",
            ))
    return cfgs


def build_square_tests(fin: int, n_servos: int, fs_hz: float) -> list[TestConfig]:
    cfgs = []
    for f_hz in SQUARE_FREQS_HZ:
        u = signals.square_wave(f_hz, SQUARE_AMPLITUDE_DEG, SQUARE_DURATION_S, fs_hz)
        cmd = {fin: u}
        cmd.update(_zero_for_others(fin, n_servos, len(u)))
        cfgs.append(TestConfig(
            name=f"square_fin{fin}_{f_hz:.1f}Hz",
            fs_hz=fs_hz,
            cmd_per_fin=cmd,
            description=f"±{SQUARE_AMPLITUDE_DEG}° square @ {f_hz:.1f} Hz on fin {fin}",
        ))
    return cfgs


def build_chirp_test(fin: int, n_servos: int, fs_hz: float) -> TestConfig:
    u = signals.log_chirp(CHIRP_F_START_HZ, CHIRP_F_END_HZ,
                          CHIRP_AMPLITUDE_DEG, CHIRP_DURATION_S, fs_hz)
    cmd = {fin: u}
    cmd.update(_zero_for_others(fin, n_servos, len(u)))
    return TestConfig(
        name=f"chirp_fin{fin}",
        fs_hz=fs_hz,
        cmd_per_fin=cmd,
        description=f"Log chirp {CHIRP_F_START_HZ}->{CHIRP_F_END_HZ} Hz on fin {fin}",
    )


def build_staircase_test(fin: int, n_servos: int, fs_hz: float) -> TestConfig:
    u = signals.staircase(STAIRCASE_LEVELS_DEG, STAIRCASE_HOLD_S, fs_hz)
    cmd = {fin: u}
    cmd.update(_zero_for_others(fin, n_servos, len(u)))
    return TestConfig(
        name=f"staircase_fin{fin}", fs_hz=fs_hz, cmd_per_fin=cmd,
        description=f"Staircase on fin {fin}",
    )


def build_slew_test(fin: int, n_servos: int, fs_hz: float) -> TestConfig:
    u = signals.slew_burst(SLEW_AMPLITUDE_DEG, SLEW_DURATION_S, fs_hz)
    cmd = {fin: u}
    cmd.update(_zero_for_others(fin, n_servos, len(u)))
    return TestConfig(
        name=f"slew_fin{fin}", fs_hz=fs_hz, cmd_per_fin=cmd,
        description=f"Hard ±{SLEW_AMPLITUDE_DEG}° reversals on fin {fin}",
    )


def build_hyst_test(fin: int, n_servos: int, fs_hz: float) -> TestConfig:
    u = signals.slow_triangle(HYST_AMPLITUDE_DEG, HYST_PERIOD_S, HYST_DURATION_S, fs_hz)
    cmd = {fin: u}
    cmd.update(_zero_for_others(fin, n_servos, len(u)))
    return TestConfig(
        name=f"hysteresis_fin{fin}", fs_hz=fs_hz, cmd_per_fin=cmd,
        description=f"Slow triangle ±{HYST_AMPLITUDE_DEG}° on fin {fin}",
    )


def build_noise_test(n_servos: int, fs_hz: float) -> TestConfig:
    u = signals.constant(0.0, NOISE_DURATION_S, fs_hz)
    cmd = {s: u.copy() for s in range(n_servos)}
    return TestConfig(
        name="noise_quiescent", fs_hz=fs_hz, cmd_per_fin=cmd,
        description="All fins at 0° — feedback noise floor",
    )


def build_crosstalk_test(fin: int, n_servos: int, fs_hz: float) -> TestConfig:
    u = signals.square_wave(0.5, CROSSTALK_AMPLITUDE_DEG, 4.0, fs_hz)
    cmd = {fin: u}
    cmd.update(_zero_for_others(fin, n_servos, len(u)))
    return TestConfig(
        name=f"crosstalk_drive_fin{fin}", fs_hz=fs_hz, cmd_per_fin=cmd,
        description=f"Drive only fin {fin}; observe other fins",
    )


# ---------------------------------------------------------------------- analysis

def analyze_step_csv(path: str, fin: int, amp_deg: float) -> dict:
    data = analyze.load_test_csv(path)
    if fin not in data["cmd"]:
        return {}
    t = data["t_cmd"]
    u = data["cmd"][fin]
    fs = data["fs_hz"]
    y = analyze.resample_fb_to_cmd_grid(t, data["fb_t"][fin], data["fb"][fin])
    fit = analyze.fit_fopdt(u, y, fs)
    sm = analyze.step_metrics(t, y, t_step=STEP_TIME_S, amplitude_deg=amp_deg)
    return {"fit": asdict(fit), "step_metrics": asdict(sm)}


def analyze_chirp_csv(path: str, fin: int) -> dict:
    data = analyze.load_test_csv(path)
    if fin not in data["cmd"]:
        return {}
    t = data["t_cmd"]
    u = data["cmd"][fin]
    fs = data["fs_hz"]
    y = analyze.resample_fb_to_cmd_grid(t, data["fb_t"][fin], data["fb"][fin])
    bode = analyze.estimate_bode(u, y, fs)
    bw = analyze.bandwidth_minus3db(bode)
    fit = analyze.fit_fopdt(u, y, fs)
    return {
        "bandwidth_minus3db_hz": bw,
        "fit": asdict(fit),
        "bode_points": [asdict(b) for b in bode],
    }


def analyze_staircase_csv(path: str, fin: int) -> dict:
    data = analyze.load_test_csv(path)
    if fin not in data["cmd"]:
        return {}
    t = data["t_cmd"]
    u = data["cmd"][fin]
    y = analyze.resample_fb_to_cmd_grid(t, data["fb_t"][fin], data["fb"][fin])
    deadband = analyze.estimate_deadband(u, y)
    # Linear regression over steady-state samples (second half of each hold)
    hold_n = int(round(STAIRCASE_HOLD_S * data["fs_hz"]))
    cmd_means, fb_means = [], []
    for k in range(0, len(u), hold_n):
        lvl = u[k:k + hold_n]
        if len(lvl) < hold_n // 2:
            break
        ss = slice(k + hold_n // 2, k + hold_n)
        cmd_means.append(float(np.mean(u[ss])))
        fb_means.append(float(np.mean(y[ss])))
    gain = float("nan")
    offset = float("nan")
    if len(cmd_means) >= 3:
        cmdA = np.array(cmd_means)
        fbA = np.array(fb_means)
        A = np.vstack([cmdA, np.ones_like(cmdA)]).T
        (gain, offset), *_ = np.linalg.lstsq(A, fbA, rcond=None)
    return {
        "deadband_deg": deadband,
        "gain": float(gain),
        "offset_deg": float(offset),
        "levels_cmd": cmd_means,
        "levels_fb": fb_means,
    }


def analyze_slew_csv(path: str, fin: int) -> dict:
    data = analyze.load_test_csv(path)
    if fin not in data["cmd"]:
        return {}
    t_fb = data["fb_t"][fin]
    y = data["fb"][fin]
    rate = analyze.estimate_slew_rate(t_fb, y)
    return {"max_slew_deg_s": rate}


def analyze_hyst_csv(path: str, fin: int) -> dict:
    data = analyze.load_test_csv(path)
    if fin not in data["cmd"]:
        return {}
    t = data["t_cmd"]
    u = data["cmd"][fin]
    y = analyze.resample_fb_to_cmd_grid(t, data["fb_t"][fin], data["fb"][fin])
    return {"hysteresis_deg": analyze.estimate_hysteresis(u, y)}


def analyze_noise_csv(path: str, n_servos: int) -> dict:
    data = analyze.load_test_csv(path)
    out = {}
    for s in range(n_servos):
        if s in data["fb"]:
            # Skip first 0.5s transient
            t = data["fb_t"][s]
            y = data["fb"][s]
            mask = t > 0.5
            out[f"fin{s}_noise_rms_deg"] = analyze.estimate_noise_floor(y[mask])
            out[f"fin{s}_mean_deg"] = float(np.mean(y[mask])) if np.any(mask) else float("nan")
    return out


def analyze_crosstalk_csv(path: str, drive_fin: int, n_servos: int) -> dict:
    data = analyze.load_test_csv(path)
    out = {"drive_fin": drive_fin}
    for s in range(n_servos):
        if s not in data["fb"]:
            continue
        y = data["fb"][s]
        # Motion on non-driven fins should be ~noise level
        out[f"fin{s}_pp_deg"] = float(np.ptp(y)) if len(y) else float("nan")
    return out


# ---------------------------------------------------------------------- aggregation

def fuse_fin_summary(fin: int, step_results: list[dict],
                     chirp_result: dict, staircase_result: dict,
                     slew_result: dict, hyst_result: dict,
                     noise_result: dict) -> dict:
    """Combine per-fin results into a single model-parameter summary."""
    fits = [r.get("fit") for r in step_results if r.get("fit")]
    tau_arr = np.array([f["tau_s"] for f in fits], dtype=float)
    td_arr = np.array([f["delay_s"] for f in fits], dtype=float)
    K_arr = np.array([f["K"] for f in fits], dtype=float)

    step_summary = {
        "tau_s_median": float(np.median(tau_arr)) if len(tau_arr) else float("nan"),
        "tau_s_min": float(np.min(tau_arr)) if len(tau_arr) else float("nan"),
        "tau_s_max": float(np.max(tau_arr)) if len(tau_arr) else float("nan"),
        "delay_s_median": float(np.median(td_arr)) if len(td_arr) else float("nan"),
        "delay_s_min": float(np.min(td_arr)) if len(td_arr) else float("nan"),
        "delay_s_max": float(np.max(td_arr)) if len(td_arr) else float("nan"),
        "gain_median": float(np.median(K_arr)) if len(K_arr) else float("nan"),
        "rmse_median_deg": float(np.median([f["rmse_deg"] for f in fits])) if fits else float("nan"),
        "per_amplitude": step_results,
    }

    return {
        "fin": fin,
        "step": step_summary,
        "chirp": chirp_result,
        "staircase": staircase_result,
        "slew": slew_result,
        "hysteresis": hyst_result,
        "noise_rms_deg": noise_result.get(f"fin{fin}_noise_rms_deg", float("nan")),
        "recommended_model": {
            "type": "first_order_plus_delay",
            "tau_s": step_summary["tau_s_median"],
            "delay_s": step_summary["delay_s_median"],
            "gain": step_summary["gain_median"],
        },
    }


# ---------------------------------------------------------------------- main

def parse_args():
    p = argparse.ArgumentParser(description="XQPOWER CAN servo characterization")
    p.add_argument("--port", default=None,
                   help="Explicit hidraw path (auto-detected if omitted)")
    p.add_argument("--fins", default=",".join(str(f) for f in DEFAULT_FINS),
                   help="Comma-separated slot indices to test (0..3)")
    p.add_argument("--tests", default="noise,step,square,chirp,staircase,slew,hysteresis,crosstalk",
                   help="Comma-separated test names")
    p.add_argument("--fs", type=float, default=DEFAULT_FS_HZ,
                   help="Command rate (Hz)")
    p.add_argument("--fb-ms", type=int, default=DEFAULT_FB_INTERVAL_MS,
                   help="Servo PDO auto-report interval (ms, >=1)")
    p.add_argument("--results-dir", default=None,
                   help="Output directory (default: results/<timestamp>)")
    p.add_argument("--quick", action="store_true",
                   help="Reduce iterations (smoke test: 1 amplitude, 1 freq)")
    p.add_argument("--no-analysis", action="store_true",
                   help="Only run tests; skip model fitting")
    return p.parse_args()


def apply_quick(args):
    global STEP_AMPLITUDES_DEG, SQUARE_FREQS_HZ, CHIRP_DURATION_S, HYST_DURATION_S
    STEP_AMPLITUDES_DEG = [5.0, 15.0]
    SQUARE_FREQS_HZ = [1.0, 5.0]
    CHIRP_DURATION_S = 8.0
    HYST_DURATION_S = 8.0


def main():
    args = parse_args()
    if args.quick:
        apply_quick(args)

    fins = [int(x) for x in args.fins.split(",") if x.strip()]
    tests = [x.strip() for x in args.tests.split(",") if x.strip()]

    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    results_dir = args.results_dir or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "results", timestamp
    )
    os.makedirs(results_dir, exist_ok=True)
    print(f"Results dir: {results_dir}")

    summary = {
        "timestamp": timestamp,
        "port": args.port,
        "fs_hz": args.fs,
        "fb_interval_ms": args.fb_ms,
        "fins_tested": fins,
        "tests_run": tests,
        "per_fin": {},
        "noise": {},
        "crosstalk": {},
    }

    # ---- connect
    print(f"\nOpening {args.port} ...")
    bus = XqpowerBus(port=args.port)
    bus.open()
    try:
        print(f"Initializing servos (PDO report = {args.fb_ms} ms) ...")
        bus.init_all_servos(report_interval_ms=args.fb_ms, settle_s=0.8)
        if not bus.wait_for_all_online(timeout_s=5.0):
            online = [fb.online for fb in bus.get_all_feedback()]
            print(f"  WARNING: not all servos online: {online}")
        else:
            print("  All 4 servos online.")

        tester = Tester(bus, results_dir)

        # --- noise test (single; uses all fins)
        per_fin_results: dict[int, dict] = {f: {} for f in fins}
        if "noise" in tests:
            r = tester.run(build_noise_test(bus.n_servos, args.fs))
            summary["noise"] = analyze_noise_csv(r.csv_path, bus.n_servos) if not args.no_analysis else {}

        for fin in fins:
            fin_res: dict = {}

            if "step" in tests:
                step_cfgs = build_step_tests(fin, bus.n_servos, args.fs)
                step_analyses = []
                for cfg, amp in zip(step_cfgs,
                                    sum(([a, a] for a in STEP_AMPLITUDES_DEG), [])):
                    r = tester.run(cfg)
                    signed_amp = amp if "_pos_" in cfg.name else -amp
                    if not args.no_analysis:
                        step_analyses.append(analyze_step_csv(r.csv_path, fin, signed_amp))
                fin_res["step_analyses"] = step_analyses

            if "square" in tests:
                for cfg in build_square_tests(fin, bus.n_servos, args.fs):
                    tester.run(cfg)

            if "chirp" in tests:
                r = tester.run(build_chirp_test(fin, bus.n_servos, args.fs))
                if not args.no_analysis:
                    fin_res["chirp"] = analyze_chirp_csv(r.csv_path, fin)

            if "staircase" in tests:
                r = tester.run(build_staircase_test(fin, bus.n_servos, args.fs))
                if not args.no_analysis:
                    fin_res["staircase"] = analyze_staircase_csv(r.csv_path, fin)

            if "slew" in tests:
                r = tester.run(build_slew_test(fin, bus.n_servos, args.fs))
                if not args.no_analysis:
                    fin_res["slew"] = analyze_slew_csv(r.csv_path, fin)

            if "hysteresis" in tests:
                r = tester.run(build_hyst_test(fin, bus.n_servos, args.fs))
                if not args.no_analysis:
                    fin_res["hysteresis"] = analyze_hyst_csv(r.csv_path, fin)

            if "crosstalk" in tests:
                r = tester.run(build_crosstalk_test(fin, bus.n_servos, args.fs))
                if not args.no_analysis:
                    summary["crosstalk"][f"drive_fin{fin}"] = analyze_crosstalk_csv(
                        r.csv_path, fin, bus.n_servos)

            per_fin_results[fin] = fin_res

        # ---- fuse summaries
        if not args.no_analysis:
            for fin in fins:
                fr = per_fin_results[fin]
                summary["per_fin"][str(fin)] = fuse_fin_summary(
                    fin=fin,
                    step_results=fr.get("step_analyses", []),
                    chirp_result=fr.get("chirp", {}),
                    staircase_result=fr.get("staircase", {}),
                    slew_result=fr.get("slew", {}),
                    hyst_result=fr.get("hysteresis", {}),
                    noise_result=summary.get("noise", {}),
                )

        # ---- write summary JSON
        summary_path = os.path.join(results_dir, "summary.json")
        with open(summary_path, "w") as f:
            json.dump(summary, f, indent=2, default=float)
        print(f"\nSummary: {summary_path}")

        # ---- concise recommended-model printout
        if not args.no_analysis:
            print("\n=== Recommended FOPDT parameters per fin ===")
            print(f"{'Fin':>4} | {'τ [ms]':>8} | {'Td [ms]':>8} | {'K':>6} | {'RMSE [°]':>9}")
            for fin in fins:
                rec = summary["per_fin"][str(fin)]["recommended_model"]
                rmse = summary["per_fin"][str(fin)]["step"]["rmse_median_deg"]
                print(f"{fin:>4} | {rec['tau_s']*1000:>8.2f} | {rec['delay_s']*1000:>8.2f} "
                      f"| {rec['gain']:>6.3f} | {rmse:>9.3f}")

    finally:
        print("\nShutting down — zeroing all fins ...")
        for s in range(bus.n_servos):
            bus.set_position_deg(s, 0.0)
        time.sleep(0.3)
        bus.close()
        print("Done.")


if __name__ == "__main__":
    main()
