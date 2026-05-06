#!/usr/bin/env python3
"""
PIL Test Runner — مستقل تماماً
==============================

يُشغّل جسر PIL الذي يتصل بجهاز ARM64 خارجي عبر TCP MAVLink،
ويُصدِر `pil_flight.csv` و `pil_timing.csv` فقط.

لا توجد محاكاة مرجعية (baseline) ولا مقارنة داخل هذا المُشغِّل.

Usage:
    python pil_runner.py
    python pil_runner.py --config path/to/pil_config.yaml
    python pil_runner.py --pil-csv out.csv --timing-csv timing.csv
"""

from __future__ import annotations

import argparse
import os
import sys
from datetime import datetime
from pathlib import Path

import yaml


def _stamped(name: str, ts: str) -> str:
    """Insert ``_<ts>`` before the file extension. ``flight.csv`` → ``flight_<ts>.csv``."""
    p = Path(name)
    return f"{p.stem}_{ts}{p.suffix}"

_SCRIPT_DIR = Path(__file__).resolve().parent
_SIM_DIR = _SCRIPT_DIR.parent
_RESULTS = _SCRIPT_DIR / "results"
sys.path.insert(0, str(_SIM_DIR))
sys.path.insert(0, str(_SCRIPT_DIR))


def run_pil(cfg_path: str, flight_csv: str, timing_csv: str) -> tuple[str, str]:
    """يُطلق جسر PIL ويُصدِر CSV للرحلة والتوقيت."""
    from mavlink_bridge_pil import PILBridge

    print("=" * 70)
    print("  PIL: connecting to remote target")
    print("=" * 70)

    bridge = PILBridge(cfg_path)
    bridge.run(flight_csv, timing_csv)
    return flight_csv, timing_csv


def main():
    ap = argparse.ArgumentParser(description="M130 PIL Test Runner (standalone)")
    ap.add_argument("--config", default=str(_SCRIPT_DIR / "pil_config.yaml"))
    ap.add_argument("--pil-csv", default=None)
    ap.add_argument("--timing-csv", default=None)
    args = ap.parse_args()

    with open(args.config, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    out = cfg.get("output", {})
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    pil_csv = args.pil_csv or str(
        _RESULTS / _stamped(out.get("csv_name", "pil_flight.csv"), ts)
    )
    timing_csv = args.timing_csv or str(
        _RESULTS / _stamped(out.get("timing_csv", "pil_timing.csv"), ts)
    )
    os.makedirs(_RESULTS, exist_ok=True)

    run_pil(args.config, pil_csv, timing_csv)

    # Auto-run HTML analysis
    try:
        from pil_analysis import analyze_pil_csv
        print()
        print("╔══════════════════════════════════════════════════════╗")
        print("║       GENERATING HTML ANALYSIS REPORT (PIL)         ║")
        print("╚══════════════════════════════════════════════════════╝")
        analyze_pil_csv(pil_csv, open_browser=True)
    except Exception as e:
        print(f"  WARNING: HTML analysis failed: {e}")
        print(f"  You can run it manually:  python3 pil_analysis.py --file {pil_csv}")


if __name__ == "__main__":
    main()
