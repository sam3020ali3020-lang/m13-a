#!/usr/bin/env python3
"""
jitter_runner.py — Scheduler Jitter Test CLI.

Runs one or more scenarios (baseline / light_load / heavy_load / all),
captures MAVLink streams for the configured duration, computes jitter
statistics on internal PX4 timestamps, applies pass/fail thresholds,
and optionally produces a multi-scenario comparison report.

Usage:
    python3 jitter_runner.py --scenario baseline
    python3 jitter_runner.py --scenario heavy_load
    python3 jitter_runner.py --all
    python3 jitter_runner.py --all --no-plot

heavy_load automatically spawns 'yes' processes on the phone via adb.
baseline and light_load expect the user to prepare the phone manually.
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from typing import List, Optional

try:
    import yaml
except ImportError:
    print('ERROR: PyYAML is required. Install with: pip install pyyaml', file=sys.stderr)
    sys.exit(1)

# Ensure imports work when run from any directory
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from jitter_reader import run_capture, CaptureResult       # noqa: E402
from jitter_analysis import (                              # noqa: E402
    analyze_capture,
    save_result,
    save_comparison_report,
    save_histograms,
)


# ---------------------------------------------------------------------------
# ADB stress helpers (heavy_load scenario)
# ---------------------------------------------------------------------------

def _adb(cfg: dict, *args: str, capture: bool = False) -> str:
    adb = cfg.get('stress', {}).get('adb_path', 'adb')
    cmd = [adb] + list(args)
    if capture:
        return subprocess.run(cmd, capture_output=True, text=True).stdout
    subprocess.run(cmd, capture_output=True)
    return ''


def spawn_stress(cfg: dict, n_procs: int) -> None:
    """Spawn N 'yes' processes on the phone via a single adb shell call."""
    if n_procs <= 0:
        return
    # One-shot: nohup + </dev/null prevents adb from hanging on background jobs.
    spawn_cmd = (
        f'for i in $(seq 1 {n_procs}); do '
        f'nohup yes </dev/null >/dev/null 2>&1 & '
        f'done; echo spawned'
    )
    _adb(cfg, 'shell', spawn_cmd, capture=True)
    time.sleep(1.0)
    count = _adb(cfg, 'shell', 'pgrep -c yes', capture=True).strip()
    print(f'  [stress] {count} yes processes running (target: {n_procs})')


def kill_stress(cfg: dict) -> None:
    _adb(cfg, 'shell', 'pkill yes', capture=True)
    time.sleep(0.5)
    count = _adb(cfg, 'shell', 'pgrep -c yes 2>/dev/null || echo 0', capture=True).strip()
    print(f'  [stress] cleanup complete — remaining yes processes: {count}')


# ---------------------------------------------------------------------------
# Pretty printing
# ---------------------------------------------------------------------------

_BAR = '═' * 66
_SUB = '─' * 66


def _progress_cb(cap_state: dict) -> None:
    elapsed = cap_state['elapsed_s']
    remain = cap_state['remaining_s']
    imu_rate = cap_state['imu_rate_hz']
    rkt_rate = cap_state['rkt_rate_hz']
    imu_cnt = cap_state['imu_count']
    rkt_cnt = cap_state['rkt_count']
    sys.stdout.write(
        f'\r  ⏱ {elapsed:5.1f}s  '
        f'IMU: {imu_rate:5.1f} Hz ({imu_cnt})  '
        f'RktGNC: {rkt_rate:5.1f} Hz ({rkt_cnt})  '
        f'remain: {remain:5.1f}s  '
    )
    sys.stdout.flush()


def _print_stats(stats: dict, verdict: dict) -> None:
    label = stats.get('label', '?')
    if 'error' in stats:
        print(f'  📊 {label}: insufficient data (n={stats["count"]})')
        return
    tag = '✅' if verdict.get('overall_pass') else '❌'
    print(f'  📊 {label}  {tag}')
    print(f'     target:    {stats["target_ms"]:6.2f} ms   (n={stats["count"]})')
    print(f'     mean:      {stats["mean_ms"]:6.2f} ms')
    print(f'     stddev:    {stats["stddev_ms"]:6.2f} ms   ← jitter')
    print(f'     min/max:   {stats["min_ms"]:6.2f} / {stats["max_ms"]:6.2f} ms')
    print(f'     p50/p95:   {stats["p50_ms"]:6.2f} / {stats["p95_ms"]:6.2f} ms')
    print(f'     p99/p99.9: {stats["p99_ms"]:6.2f} / {stats["p99_9_ms"]:6.2f} ms')
    print(f'     late >2×:  {stats["late_2x"]} ({stats["late_2x_pct"]:.2f}%)')
    print(f'     late >3×:  {stats["late_3x"]}')
    print(f'     dropped:   {stats["dropped_est"]}  (gaps >5× target)')
    if not verdict.get('overall_pass') and verdict.get('failures'):
        print(f'     ❌ failed: {", ".join(verdict["failures"])}')


# ---------------------------------------------------------------------------
# Scenario runner
# ---------------------------------------------------------------------------

def run_scenario(cfg: dict, scenario: str, override_duration: Optional[float] = None
                 ) -> tuple[CaptureResult, dict]:
    sc_cfg = cfg['scenarios'][scenario]
    duration = override_duration if override_duration else sc_cfg['duration_s']
    n_stress = sc_cfg.get('stress_yes_procs', 0)

    print(_BAR)
    print(f'  Scenario: {scenario}  —  duration: {duration}s')
    print(f'  {sc_cfg["description"]}')
    print(_BAR)

    if scenario == 'baseline':
        print('  📱 Phone: idle, no background apps, screen on')
    elif scenario == 'light_load':
        print('  📱 Phone: WhatsApp or camera open (user-initiated)')
    elif scenario == 'heavy_load':
        print(f'  🔥 Phone: auto-spawn {n_stress} yes processes (via adb)')

    if n_stress > 0:
        spawn_stress(cfg, n_stress)

    print()
    try:
        capture = run_capture(cfg, scenario, duration, on_progress=_progress_cb)
        print()
    finally:
        if n_stress > 0 and cfg.get('stress', {}).get('cleanup_on_exit', True):
            kill_stress(cfg)

    report = analyze_capture(capture, cfg)

    print(_SUB)
    print('  🕐 INTERNAL PX4 TIMESTAMPS (used for pass/fail)')
    print(_SUB)
    _print_stats(report['imu_stats'], report['imu_verdict'])
    print()
    _print_stats(report['rkt_stats'], report['rkt_verdict'])
    print(_SUB)

    verdict_tag = '✅ PASS' if report['overall_pass'] else '❌ FAIL'
    print(f'  Verdict: {verdict_tag}')
    print(_BAR)

    return capture, report


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def load_config(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def main() -> int:
    default_cfg = os.path.join(HERE, 'jitter_config.yaml')
    p = argparse.ArgumentParser(
        description='Scheduler Jitter Test — measure PX4 control loop jitter under varying load.'
    )
    p.add_argument('--config', default=default_cfg, help='Path to jitter_config.yaml')
    p.add_argument('--scenario', choices=['baseline', 'light_load', 'heavy_load'],
                   help='Run a single scenario')
    p.add_argument('--all', action='store_true',
                   help='Run all three scenarios sequentially (prompts user before each)')
    p.add_argument('--duration', type=float, default=None,
                   help='Override scenario duration (seconds)')
    p.add_argument('--no-plot', action='store_true', help='Skip histogram plot')
    p.add_argument('--results-dir', default=None,
                   help='Override results directory (default: from config)')
    args = p.parse_args()

    if not args.scenario and not args.all:
        p.error('must specify --scenario <name> or --all')

    cfg = load_config(args.config)
    results_dir = args.results_dir or os.path.join(HERE, cfg['output']['results_dir'])
    os.makedirs(results_dir, exist_ok=True)

    scenarios: List[str] = (
        ['baseline', 'light_load', 'heavy_load'] if args.all else [args.scenario]
    )

    captures: List[CaptureResult] = []
    reports: List[dict] = []

    for i, sc in enumerate(scenarios):
        if args.all and i > 0:
            # Prompt for baseline/light_load; heavy_load is automatic.
            if sc != 'heavy_load':
                input(f'\n>> Press ENTER when phone is ready for {sc}... ')

        capture, report = run_scenario(cfg, sc, override_duration=args.duration)
        captures.append(capture)
        reports.append(report)

        if cfg['output'].get('save_json', True):
            path = save_result(report, results_dir)
            print(f'  💾 Saved {path}')

    # Multi-scenario deliverables
    if len(reports) > 1 and cfg['output'].get('compare_scenarios', True):
        md_path = save_comparison_report(reports, cfg, results_dir)
        print(f'\n  📄 Comparison report: {md_path}')

        if not args.no_plot and cfg['output'].get('save_plot', True):
            png_path = save_histograms(reports, captures, cfg, results_dir)
            if png_path:
                print(f'  🖼  Histograms: {png_path}')
            else:
                print('  (skipped plot — matplotlib not available)')

    any_fail = any(not r['overall_pass'] for r in reports)
    return 1 if any_fail else 0


if __name__ == '__main__':
    sys.exit(main())
