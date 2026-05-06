#!/usr/bin/env python3
"""
e2e_runner.py — End-to-end latency test runner
==================================================

Measures transport delay from phone IMU input to physical servo movement
by capturing all relevant MAVLink streams simultaneously and computing
inter-stream timestamp deltas.

Usage:
    python3 e2e_runner.py                       # quick (60s passive)
    python3 e2e_runner.py --preset standard     # 5 min
    python3 e2e_runner.py --preset full         # 15 min including tap
    python3 e2e_runner.py --test passive --duration 120
    python3 e2e_runner.py --analyze-only results/20260503_180000/
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import yaml

_SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_SCRIPT_DIR))

from e2e_reader import E2EReader  # noqa: E402
import e2e_analysis  # noqa: E402

logger = logging.getLogger("e2e_runner")


# ============================================================================
# Test base class
# ============================================================================

class E2ETest:
    """Base class for e2e_latency sub-tests."""

    name: str = "base"
    description_ar: str = ""

    def __init__(self, config: dict, thresholds: dict, result_dir: Path):
        self.config = config
        self.thresholds = thresholds
        self.result_dir = result_dir

    def run(self, reader: E2EReader):
        raise NotImplementedError


# ============================================================================
# Test 1: Passive — works any time
# ============================================================================

class PassiveTest(E2ETest):
    name = "passive"
    description_ar = "تسجيل سلبي لكل MAVLink streams، حساب L_sensor / L_mpc / L_actuator"

    def run(self, reader: E2EReader, duration_override: Optional[float] = None):
        cfg = self.config.get("tests", {}).get("passive", {})
        duration = duration_override if duration_override is not None \
                   else cfg.get("duration_s", 60)

        print(f"\n{'='*70}")
        print(f"  Passive E2E Latency Test — {duration:.0f}s")
        print(f"  {self.description_ar}")
        print(f"{'='*70}")
        print("  [النصيحة] لقياس L_actuator، يجب أن يأمر MPC بحركة الفينات.")
        print("    - إذا الصاروخ على الطاولة في pre-launch، L_actuator سيُعلَن 'no movement'.")
        print("    - شغّل HITL مع المحاكي لتفعيل MPC وتحريك الفينات.")
        print("    - L_sensor + L_mpc يُقاسان دائماً حتى لو السيرفو ساكن.")
        print(f"  Recording for {duration:.0f}s...")
        print()

        reader.clear()
        reader.record(duration_s=duration, progress_interval_s=10.0)

        # Save CSVs
        reader.save_e2e_csv(self.result_dir)

        # Quick stats
        print(f"\n  IMU samples:        {len(reader.imu_samples)}")
        print(f"  ATTITUDE samples:   {len(reader.attitude_samples)}")
        print(f"  RktGNC samples:     {len(reader.rktgnc_samples)}")
        print(f"  SRV_FB samples:     {len(reader.srv_fb_samples)}")
        print(f"  SERVO_RAW samples:  {len(reader.servo_raw_samples)}")


# ============================================================================
# Test 2: Tap test — operator-driven impulse
# ============================================================================

class TapTest(E2ETest):
    name = "tap"
    description_ar = "Tap الهاتف، نلتقط peak في IMU ونقيس استجابة السيرفو"

    def run(self, reader: E2EReader, duration_override: Optional[float] = None):
        cfg = self.config.get("tests", {}).get("tap", {})
        duration = duration_override if duration_override is not None \
                   else cfg.get("duration_s", 30)
        num_taps = cfg.get("num_taps", 10)
        tap_window_s = cfg.get("tap_window_s", 3.0)

        print(f"\n{'='*70}")
        print(f"  Tap Test — {num_taps} taps × {tap_window_s:.1f}s windows")
        print(f"  {self.description_ar}")
        print(f"{'='*70}")
        print("  هذا الاختبار يقيس استجابة الـ pipeline عبر impulse فيزيائي.")
        print("  - اضغط ENTER لبدء كل tap window")
        print("  - أعطِ tap واضحاً للهاتف خلال كل window (3 ثوانٍ)")
        print("  - L_total = الفرق بين peak في IMU و peak في حركة السيرفو")
        print()

        reader.clear()

        for i in range(num_taps):
            input(f"\n  >> Tap {i+1}/{num_taps} — اضغط ENTER عندما تكون جاهزاً ...")
            print(f"     جارٍ التسجيل لـ {tap_window_s:.1f}s — TAP الآن!")
            reader.record(duration_s=tap_window_s,
                           progress_interval_s=tap_window_s + 1.0,  # silent
                           heartbeat_interval_s=1.0)

        # Save CSVs
        reader.save_e2e_csv(self.result_dir)

        print(f"\n  Total samples: IMU={len(reader.imu_samples)}, "
              f"SRV_FB={len(reader.srv_fb_samples)}")


# ============================================================================
# Registry
# ============================================================================

TESTS = {
    "passive": PassiveTest,
    "tap": TapTest,
}


# ============================================================================
# Runner
# ============================================================================

def setup_logging(verbose: bool = False):
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(level=level,
                        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s")


def load_config(config_path: Path) -> dict:
    with open(config_path) as f:
        return yaml.safe_load(f)


def run_tests(test_names: List[str], duration_override: Optional[float],
              config: dict, result_dir: Path) -> Dict:
    """Run selected tests sequentially against the same E2EReader."""
    conn = config.get("connection", {})
    reader = E2EReader(host=conn.get("host", "127.0.0.1"),
                        port=conn.get("port", 5760),
                        timeout_s=conn.get("timeout_s", 10.0))

    if not reader.connect():
        return {"error": "connection failed",
                "host": conn.get("host"),
                "port": conn.get("port")}

    try:
        # Request streams from config
        reader.request_streams_from_config(config)

        # Settle stream rate negotiations
        time.sleep(0.5)

        # Run tests
        thresholds = config.get("thresholds", {})

        for name in test_names:
            cls = TESTS.get(name)
            if cls is None:
                logger.warning(f"Unknown test '{name}' — skipping")
                continue
            test = cls(config, thresholds, result_dir)
            test.run(reader, duration_override=duration_override)

    finally:
        reader.disconnect()

    # Analyze + report
    metrics = e2e_analysis.analyze_directory(result_dir, thresholds=thresholds)

    return metrics


def save_outputs(result_dir: Path, metrics: Dict, save_plot: bool = True):
    """Persist JSON, text report, and (optional) HTML plot."""
    # JSON
    with open(result_dir / "latency.metrics.json", "w") as f:
        json.dump(metrics, f, indent=2)

    # Report
    report = e2e_analysis.format_report(metrics)
    with open(result_dir / "latency_report.txt", "w") as f:
        f.write(report)
    print(report)

    # Plot
    if save_plot:
        e2e_analysis.maybe_plot(result_dir, metrics)


def main():
    p = argparse.ArgumentParser(description="End-to-end latency test runner")
    p.add_argument("--config", type=Path,
                   default=_SCRIPT_DIR / "e2e_config.yaml",
                   help="Path to e2e_config.yaml")
    p.add_argument("--preset", choices=["quick", "standard", "full"],
                   default=None,
                   help="Run a predefined preset (overrides --test)")
    p.add_argument("--test", action="append", default=None,
                   choices=list(TESTS.keys()),
                   help="Run specific test(s). Can repeat: --test passive --test tap")
    p.add_argument("--duration", type=float, default=None,
                   help="Override duration in seconds for the test(s)")
    p.add_argument("--results-dir", type=Path, default=None,
                   help="Custom result directory (default: results/<timestamp>/)")
    p.add_argument("--analyze-only", type=Path, default=None,
                   help="Skip recording, analyze existing results dir")
    p.add_argument("--no-plot", action="store_true",
                   help="Skip Plotly HTML generation")
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args()

    setup_logging(args.verbose)

    # Analyze only mode
    if args.analyze_only:
        config = load_config(args.config)
        thresholds = config.get("thresholds", {})
        metrics = e2e_analysis.analyze_directory(args.analyze_only,
                                                   thresholds=thresholds)
        save_outputs(args.analyze_only, metrics, save_plot=not args.no_plot)
        return 0 if metrics.get("pass_fail", {}).get("passed") else 1

    # Load config
    if not args.config.exists():
        logger.error(f"Config not found: {args.config}")
        return 2
    config = load_config(args.config)

    # Determine which tests to run
    if args.preset:
        preset_cfg = config.get("presets", {}).get(args.preset)
        if not preset_cfg:
            logger.error(f"Preset '{args.preset}' not found in config")
            return 2
        test_names = preset_cfg.get("tests", ["passive"])
        logger.info(f"Preset '{args.preset}' → tests: {test_names}")
    elif args.test:
        test_names = args.test
    else:
        # Default: quick → passive only
        test_names = ["passive"]

    # Result directory
    if args.results_dir:
        result_dir = args.results_dir
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        result_dir = _SCRIPT_DIR / "results" / timestamp
    result_dir.mkdir(parents=True, exist_ok=True)
    logger.info(f"Results → {result_dir}")
    logger.info(f"Tests:   {test_names}")

    # Save config snapshot for reproducibility
    with open(result_dir / "config_used.yaml", "w") as f:
        yaml.safe_dump(config, f, allow_unicode=True, sort_keys=False)

    # Run
    metrics = run_tests(test_names, args.duration, config, result_dir)

    # Save outputs
    save_outputs(result_dir, metrics, save_plot=not args.no_plot)

    # Exit code reflects pass/fail
    pf = metrics.get("pass_fail", {})
    return 0 if pf.get("passed", True) else 1


if __name__ == "__main__":
    sys.exit(main())
