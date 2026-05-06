#!/usr/bin/env python3
"""
cpu_profile.py — per-thread CPU profiler for PX4 on Android.

Samples `top -H -p <PX4 PID>` once per second for N seconds via adb,
parses the per-thread CPU percentages, and produces:
  - time-series CSV per thread
  - trend analysis (steady / rising / falling / oscillating)
  - top-N hot threads
  - markdown report

Usage:
    python3 cpu_profile.py --duration 60
    python3 cpu_profile.py --duration 60 --during-stress
    python3 cpu_profile.py --duration 60 --pid 23025
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import re
import statistics
import subprocess
import sys
import time
from collections import defaultdict
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))


# ---------------------------------------------------------------------------
# ADB helpers
# ---------------------------------------------------------------------------

def adb_capture(*args: str) -> str:
    return subprocess.run(['adb'] + list(args),
                          capture_output=True, text=True).stdout


def find_px4_pid(package: str = 'com.ardophone.px4v17') -> Optional[int]:
    out = adb_capture('shell', f'pidof {package}').strip()
    return int(out) if out.isdigit() else None


def get_thread_names(pid: int) -> Dict[int, str]:
    """Map tid -> thread name via /proc/<pid>/task/<tid>/comm."""
    cmd = (f'for tid in /proc/{pid}/task/*/; do '
           f'tid_n=$(basename $tid); '
           f'name=$(cat $tid/comm 2>/dev/null); '
           f'echo "$tid_n|$name"; done')
    out = adb_capture('shell', cmd)
    names: Dict[int, str] = {}
    for line in out.splitlines():
        line = line.strip()
        if '|' not in line:
            continue
        tid_s, name = line.split('|', 1)
        try:
            names[int(tid_s)] = name.strip()
        except ValueError:
            pass
    return names


def spawn_stress(n: int = 8) -> None:
    cmd = (f'for i in $(seq 1 {n}); do '
           f'nohup yes </dev/null >/dev/null 2>&1 & done; '
           f'echo spawned')
    adb_capture('shell', cmd)
    time.sleep(1)


def kill_stress() -> None:
    adb_capture('shell', 'pkill yes')


# ---------------------------------------------------------------------------
# top output parser
# ---------------------------------------------------------------------------

# Android toybox `top -H -b -p <pid>` output (one frame per sample):
#
# Header (varies by toybox version) then rows:
#   PID USER PR NI VIRT RES SHR S [%CPU] [%MEM] TIME+ ARGS
#
# We use `top -H -p $PID -b -n N -d 1 -q` which prints headers once and
# then one row per thread per sample. To split frames, we look for blank
# lines between samples.

# Robust row regex: tid is a number, then the rest. The exact column layout
# differs slightly between toybox versions, so we match by structure:
#   <tid> <user> <pri> <nice> <virt> <res> <shr> <state> <cpu%> <mem%> <time> <name...>
_ROW_RE = re.compile(
    r'^\s*(?P<tid>\d+)\s+\S+\s+\S+\s+\S+\s+\S+\s+\S+\s+\S+\s+'
    r'(?P<state>[A-Z])\s+(?P<cpu>\d+\.?\d*)\s+(?P<mem>\d+\.?\d*)\s+'
    r'(?P<time>\d+:\d+\.\d+)\s+(?P<name>.+?)\s*$'
)


def parse_top_output(raw: str, names: Dict[int, str]) -> List[List[Tuple[int, str, float]]]:
    """Split `top -H -b` output into frames (list of [tid, name, cpu_pct]).

    A frame ends when we see a blank line followed by a header line, OR EOF.
    We rely on the fact that toybox prints a header at the start of each
    sample frame.
    """
    frames: List[List[Tuple[int, str, float]]] = []
    current: List[Tuple[int, str, float]] = []
    seen_data_in_frame = False

    for line in raw.splitlines():
        stripped = line.strip()
        # Header lines start with "PID" or contain "USER"
        if stripped.startswith('PID') or (stripped == ''):
            if seen_data_in_frame:
                frames.append(current)
                current = []
                seen_data_in_frame = False
            continue

        m = _ROW_RE.match(line)
        if not m:
            continue
        tid = int(m.group('tid'))
        cpu = float(m.group('cpu'))
        # Prefer name from /proc/<pid>/task/.../comm (more accurate than truncated top output)
        name = names.get(tid, m.group('name').strip())
        current.append((tid, name, cpu))
        seen_data_in_frame = True

    if seen_data_in_frame:
        frames.append(current)

    return frames


# ---------------------------------------------------------------------------
# Capture
# ---------------------------------------------------------------------------

@dataclass
class ProfileResult:
    pid: int
    duration_s: int
    samples_taken: int
    started_at: float
    # series[tid] = list of (frame_idx, cpu_pct)
    series: Dict[int, List[Tuple[int, float]]]
    names: Dict[int, str]


def run_profile(pid: int, duration_s: int,
                on_progress=None) -> ProfileResult:
    """Run `top -H -b` for duration_s seconds, parse, return time series."""
    names = get_thread_names(pid)
    n_samples = duration_s

    # Run top in batch mode. We pipe output back via adb.
    # -d 1 = 1 second delay
    # -n N = N iterations
    # -q   = no buffering / fewer redraws
    # -b   = batch mode (no terminal control codes)
    proc = subprocess.Popen(
        ['adb', 'shell', f'top -H -p {pid} -b -n {n_samples} -d 1 -q'],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )

    started = time.time()
    raw_lines: List[str] = []
    last_progress = 0.0

    while True:
        line = proc.stdout.readline()
        if not line:
            break
        raw_lines.append(line.rstrip('\n'))
        now = time.monotonic()
        if on_progress and (now - last_progress) >= 1.0:
            last_progress = now
            elapsed = time.time() - started
            on_progress({
                'elapsed_s': elapsed,
                'remaining_s': max(0.0, duration_s - elapsed),
                'lines_so_far': len(raw_lines),
            })

    proc.wait(timeout=5)

    raw = '\n'.join(raw_lines)
    frames = parse_top_output(raw, names)

    # Build per-tid series
    series: Dict[int, List[Tuple[int, float]]] = defaultdict(list)
    for fi, frame in enumerate(frames):
        for tid, _, cpu in frame:
            series[tid].append((fi, cpu))

    return ProfileResult(
        pid=pid,
        duration_s=duration_s,
        samples_taken=len(frames),
        started_at=started,
        series=dict(series),
        names=names,
    )


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------

def detect_trend(values: List[float]) -> str:
    """Classify a CPU time series as steady / rising / falling / oscillating."""
    if len(values) < 5:
        return 'too-short'
    n = len(values)
    half = n // 2
    early = statistics.mean(values[:half])
    late = statistics.mean(values[half:])
    delta = late - early
    overall_mean = statistics.mean(values) or 1e-9
    overall_std = statistics.stdev(values) if n > 1 else 0.0
    cv = overall_std / overall_mean if overall_mean > 0 else 0.0

    if abs(delta) < 1.0 and cv < 0.5:
        return 'steady'
    if delta >= 2.0 and cv < 1.0:
        return 'rising'
    if delta <= -2.0 and cv < 1.0:
        return 'falling'
    if cv >= 0.7:
        return 'oscillating'
    return 'steady'


def thread_summary(result: ProfileResult) -> List[dict]:
    """Compute summary per thread."""
    rows: List[dict] = []
    for tid, pts in result.series.items():
        values = [v for _, v in pts]
        if not values:
            continue
        rows.append({
            'tid': tid,
            'name': result.names.get(tid, '?'),
            'samples': len(values),
            'mean_cpu': round(statistics.mean(values), 2),
            'max_cpu': round(max(values), 2),
            'stddev_cpu': round(statistics.stdev(values), 2) if len(values) > 1 else 0.0,
            'trend': detect_trend(values),
        })
    rows.sort(key=lambda r: r['mean_cpu'], reverse=True)
    return rows


def total_per_frame(result: ProfileResult) -> List[float]:
    """Sum CPU% across all PX4 threads, per frame."""
    n_frames = result.samples_taken
    totals = [0.0] * n_frames
    for tid, pts in result.series.items():
        for fi, cpu in pts:
            if 0 <= fi < n_frames:
                totals[fi] += cpu
    return totals


# ---------------------------------------------------------------------------
# Reports
# ---------------------------------------------------------------------------

def write_csv(result: ProfileResult, path: str) -> None:
    """Wide CSV: one row per frame, one column per tid."""
    tids = sorted(result.series.keys())
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        header = ['frame_idx'] + [f'{tid}:{result.names.get(tid, "?")}' for tid in tids]
        w.writerow(header)
        # Build dense matrix
        n = result.samples_taken
        matrix = [[0.0] * len(tids) for _ in range(n)]
        for col, tid in enumerate(tids):
            for fi, cpu in result.series[tid]:
                if 0 <= fi < n:
                    matrix[fi][col] = cpu
        for fi in range(n):
            w.writerow([fi] + [f'{v:.2f}' for v in matrix[fi]])


def write_markdown(result: ProfileResult, summary: List[dict],
                   totals: List[float], path: str, label: str) -> None:
    lines: List[str] = []
    lines.append(f'# CPU Profile — {label}\n')
    lines.append(f'PX4 PID: {result.pid}, samples: {result.samples_taken}, duration: {result.duration_s}s\n')

    # Total trend
    if totals:
        mean_total = statistics.mean(totals)
        max_total = max(totals)
        std_total = statistics.stdev(totals) if len(totals) > 1 else 0.0
        lines.append('## Total PX4 CPU usage (sum of all threads)\n')
        lines.append(f'- mean: **{mean_total:.1f}%**')
        lines.append(f'- max:  {max_total:.1f}%')
        lines.append(f'- stddev: {std_total:.1f}%')
        lines.append(f'- trend: **{detect_trend(totals)}**\n')

    # Top hot threads
    lines.append('## Top threads by mean CPU\n')
    lines.append('| Rank | TID | Name | mean% | max% | stddev% | trend |')
    lines.append('|---|---|---|---|---|---|---|')
    for i, row in enumerate(summary[:15], 1):
        lines.append(
            f'| {i} | {row["tid"]} | `{row["name"]}` | '
            f'{row["mean_cpu"]:.1f} | {row["max_cpu"]:.1f} | '
            f'{row["stddev_cpu"]:.1f} | {row["trend"]} |'
        )

    # Oscillating threads (potential issue)
    osc = [r for r in summary if r['trend'] == 'oscillating' and r['mean_cpu'] >= 1.0]
    if osc:
        lines.append('\n## Oscillating threads (CV ≥ 0.7, mean ≥ 1%)\n')
        lines.append('These show unstable CPU consumption — investigate scheduling/contention.\n')
        for r in osc:
            lines.append(f'- TID {r["tid"]} `{r["name"]}` — '
                         f'mean {r["mean_cpu"]:.1f}%, stddev {r["stddev_cpu"]:.1f}%')

    # Rising threads (potential leak)
    rising = [r for r in summary if r['trend'] == 'rising' and r['mean_cpu'] >= 1.0]
    if rising:
        lines.append('\n## Rising threads (mean ≥ 1%, late half > early half)\n')
        lines.append('These are increasing in CPU — possible leak/runaway.\n')
        for r in rising:
            lines.append(f'- TID {r["tid"]} `{r["name"]}` — '
                         f'mean {r["mean_cpu"]:.1f}%')

    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _on_progress(state: dict) -> None:
    sys.stdout.write(
        f'\r  ⏱ {state["elapsed_s"]:5.1f}s  remaining: {state["remaining_s"]:5.1f}s  '
        f'lines: {state["lines_so_far"]}  '
    )
    sys.stdout.flush()


def main() -> int:
    p = argparse.ArgumentParser(
        description='Per-thread CPU profiler for PX4 on Android.'
    )
    p.add_argument('--duration', type=int, default=60,
                   help='Capture duration in seconds (default: 60)')
    p.add_argument('--pid', type=int, default=None,
                   help='PX4 PID (auto-detected from package if omitted)')
    p.add_argument('--package', default='com.ardophone.px4v17')
    p.add_argument('--during-stress', action='store_true',
                   help='Auto-spawn 8 yes processes during the capture')
    p.add_argument('--label', default=None,
                   help='Label for output files (default: idle or stressed)')
    p.add_argument('--results-dir', default=os.path.join(HERE, 'results'))
    args = p.parse_args()

    pid = args.pid or find_px4_pid(args.package)
    if not pid:
        print(f'ERROR: PX4 process not found ({args.package}). '
              f'Open the app and press Start PX4.', file=sys.stderr)
        return 2

    label = args.label or ('stressed' if args.during_stress else 'idle')

    print(f'═══════════════════════════════════════════════════════════════')
    print(f'  CPU profile — PX4 PID {pid}  ({label})  duration {args.duration}s')
    print(f'═══════════════════════════════════════════════════════════════')

    if args.during_stress:
        print('  🔥 spawning 8 yes processes...')
        spawn_stress(8)

    try:
        result = run_profile(pid, args.duration, on_progress=_on_progress)
    finally:
        print()
        if args.during_stress:
            kill_stress()
            print('  🧹 stress cleanup done')

    summary = thread_summary(result)
    totals = total_per_frame(result)

    os.makedirs(args.results_dir, exist_ok=True)
    csv_path = os.path.join(args.results_dir, f'cpu_profile_{label}.csv')
    md_path = os.path.join(args.results_dir, f'cpu_profile_{label}.md')
    json_path = os.path.join(args.results_dir, f'cpu_profile_{label}.json')

    write_csv(result, csv_path)
    write_markdown(result, summary, totals, md_path, label)

    with open(json_path, 'w') as f:
        json.dump({
            'pid': result.pid,
            'label': label,
            'duration_s': result.duration_s,
            'samples_taken': result.samples_taken,
            'started_at': result.started_at,
            'summary': summary,
            'totals_per_frame': totals,
        }, f, indent=2)

    # Console summary
    print()
    if totals:
        mean_total = statistics.mean(totals)
        max_total = max(totals)
        print(f'  📊 Total PX4 CPU: mean={mean_total:.1f}%, max={max_total:.1f}%, '
              f'trend={detect_trend(totals)}')
    print(f'  📊 Top 10 threads:')
    for i, r in enumerate(summary[:10], 1):
        print(f'     {i:2}. {r["mean_cpu"]:5.1f}% ({r["max_cpu"]:5.1f}% peak)  '
              f'[{r["trend"]:11}]  {r["name"]}  (tid {r["tid"]})')

    print(f'\n  💾 {csv_path}')
    print(f'  💾 {md_path}')
    print(f'  💾 {json_path}')

    return 0


if __name__ == '__main__':
    sys.exit(main())
