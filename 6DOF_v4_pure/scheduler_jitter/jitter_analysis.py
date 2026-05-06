"""
jitter_analysis.py — statistics + pass/fail + comparison report for jitter test.

Computes inter-arrival interval statistics from raw timestamps captured by
jitter_reader, applies thresholds from jitter_config.yaml, and produces:
  - per-scenario JSON summary
  - multi-scenario comparison_report.md (markdown table)
  - optional histogram plot (if matplotlib available)
"""
from __future__ import annotations

import json
import os
import statistics
from dataclasses import asdict
from typing import Dict, List, Optional

from jitter_reader import CaptureResult


# ---------------------------------------------------------------------------
# Interval computation
# ---------------------------------------------------------------------------

def intervals_from_us(timestamps_us: List[int]) -> List[float]:
    """Convert monotonically-increasing µs timestamps to ms intervals.

    Drops non-positive deltas (guard against clock glitches / duplicates).
    """
    out: List[float] = []
    for i in range(len(timestamps_us) - 1):
        dt_us = timestamps_us[i + 1] - timestamps_us[i]
        if dt_us > 0:
            out.append(dt_us / 1000.0)
    return out


def intervals_from_wall(wall_s: List[float]) -> List[float]:
    """Convert monotonic wall clock (s) to ms intervals."""
    return [(wall_s[i + 1] - wall_s[i]) * 1000.0 for i in range(len(wall_s) - 1)]


# ---------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------

def compute_stats(intervals_ms: List[float], target_ms: float,
                  label: str) -> dict:
    """Compute jitter statistics for a sequence of inter-arrival intervals."""
    n = len(intervals_ms)
    if n < 10:
        return {
            'label': label,
            'count': n,
            'target_ms': target_ms,
            'error': 'insufficient_data',
        }

    sorted_iv = sorted(intervals_ms)

    def pct(p: float) -> float:
        idx = min(n - 1, int(p / 100.0 * n))
        return sorted_iv[idx]

    mean = statistics.mean(intervals_ms)
    stdev = statistics.stdev(intervals_ms) if n > 1 else 0.0

    late_2x = sum(1 for v in intervals_ms if v > 2.0 * target_ms)
    late_3x = sum(1 for v in intervals_ms if v > 3.0 * target_ms)
    dropped = sum(1 for v in intervals_ms if v > 5.0 * target_ms)

    return {
        'label': label,
        'count': n,
        'target_ms': round(target_ms, 3),
        'mean_ms': round(mean, 3),
        'stddev_ms': round(stdev, 3),
        'min_ms': round(sorted_iv[0], 3),
        'max_ms': round(sorted_iv[-1], 3),
        'p50_ms': round(pct(50), 3),
        'p95_ms': round(pct(95), 3),
        'p99_ms': round(pct(99), 3),
        'p99_9_ms': round(pct(99.9), 3),
        'late_2x': late_2x,
        'late_2x_pct': round(100.0 * late_2x / n, 3),
        'late_3x': late_3x,
        'dropped_est': dropped,
    }


# ---------------------------------------------------------------------------
# Threshold application
# ---------------------------------------------------------------------------

def apply_thresholds(stats: dict, thresholds: dict) -> dict:
    """Compare stats against thresholds; return dict with per-check pass/fail."""
    if 'error' in stats:
        return {'overall_pass': False, 'reason': stats['error']}

    checks = {
        'stddev_ms':    stats['stddev_ms']    <= thresholds['stddev_ms_max'],
        'p99_ms':       stats['p99_ms']       <= thresholds['p99_ms_max'],
        'p99_9_ms':     stats['p99_9_ms']     <= thresholds['p99_9_ms_max'],
        'late_2x_pct':  stats['late_2x_pct']  <= thresholds['late_2x_pct_max'],
        'late_3x':      stats['late_3x']      <= thresholds['late_3x_max'],
        'dropped_est':  stats['dropped_est']  <= thresholds['dropped_max'],
    }
    failures = [k for k, ok in checks.items() if not ok]
    return {
        'overall_pass': len(failures) == 0,
        'checks': checks,
        'failures': failures,
    }


# ---------------------------------------------------------------------------
# Pipeline: capture → analyze → serialize
# ---------------------------------------------------------------------------

def analyze_capture(capture: CaptureResult, cfg: dict) -> dict:
    """Run full analysis pipeline on one capture result."""
    imu_target = cfg['streams']['highres_imu']['target_ms']
    rkt_target = cfg['streams']['debug_float_array']['target_ms']

    imu_iv_internal = intervals_from_us(capture.imu_time_usec)
    rkt_iv_internal = intervals_from_us(capture.rkt_time_usec)
    imu_iv_wall = intervals_from_wall(capture.imu_wall)
    rkt_iv_wall = intervals_from_wall(capture.rkt_wall)

    imu_stats = compute_stats(imu_iv_internal, imu_target, 'HIGHRES_IMU (internal)')
    rkt_stats = compute_stats(rkt_iv_internal, rkt_target, 'RktGNC (internal)')
    imu_wall_stats = compute_stats(imu_iv_wall, imu_target, 'HIGHRES_IMU (wall/TCP)')
    rkt_wall_stats = compute_stats(rkt_iv_wall, rkt_target, 'RktGNC (wall/TCP)')

    imu_verdict = apply_thresholds(imu_stats, cfg['thresholds']['highres_imu'])
    rkt_verdict = apply_thresholds(rkt_stats, cfg['thresholds']['debug_float_array'])

    return {
        'scenario': capture.scenario,
        'duration_s': capture.duration_s,
        'started_at': capture.started_at,
        'total_msgs': capture.total_msgs,
        'imu_stats': imu_stats,
        'rkt_stats': rkt_stats,
        'imu_wall_stats': imu_wall_stats,
        'rkt_wall_stats': rkt_wall_stats,
        'imu_verdict': imu_verdict,
        'rkt_verdict': rkt_verdict,
        'overall_pass': imu_verdict['overall_pass'] and rkt_verdict['overall_pass'],
        'other_msg_counts': {str(k): v for k, v in capture.other_msg_counts.items()},
    }


def save_result(report: dict, results_dir: str) -> str:
    """Serialize one-scenario report to JSON. Returns file path."""
    os.makedirs(results_dir, exist_ok=True)
    path = os.path.join(results_dir, f'jitter_{report["scenario"]}.json')
    with open(path, 'w') as f:
        json.dump(report, f, indent=2)
    return path


# ---------------------------------------------------------------------------
# Multi-scenario comparison report (Markdown)
# ---------------------------------------------------------------------------

def build_comparison_report(reports: List[dict], cfg: dict) -> str:
    """Assemble a Markdown comparison across scenarios.

    Accepts a list of per-scenario reports (from analyze_capture).
    """
    if not reports:
        return '# Jitter Comparison — (no scenarios)\n'

    scenarios = [r['scenario'] for r in reports]
    lines = []
    lines.append('# Scheduler Jitter — Scenario Comparison\n')
    lines.append('Generated from internal PX4 timestamps (HRT). '
                 'Wall/TCP figures are not used for pass/fail '
                 '(they are affected by TCP buffering).\n')

    def row(metric: str, fmt: str, extractor) -> str:
        vals = [fmt.format(extractor(r)) for r in reports]
        return f'| {metric} | ' + ' | '.join(vals) + ' |'

    for stream, stats_key, threshold_key in [
        ('HIGHRES_IMU (50 Hz, target 20 ms)', 'imu_stats', 'highres_imu'),
        ('RktGNC (target 40 ms)', 'rkt_stats', 'debug_float_array'),
    ]:
        lines.append(f'\n## {stream}\n')
        header = '| Metric | ' + ' | '.join(scenarios) + ' |'
        sep = '|---' * (1 + len(scenarios)) + '|'
        lines.append(header)
        lines.append(sep)
        lines.append(row('count',         '{:d}',    lambda r: r[stats_key].get('count', 0)))
        lines.append(row('mean (ms)',     '{:.2f}',  lambda r: r[stats_key].get('mean_ms', 0)))
        lines.append(row('stddev (ms)',   '{:.2f}',  lambda r: r[stats_key].get('stddev_ms', 0)))
        lines.append(row('p50 (ms)',      '{:.2f}',  lambda r: r[stats_key].get('p50_ms', 0)))
        lines.append(row('p95 (ms)',      '{:.2f}',  lambda r: r[stats_key].get('p95_ms', 0)))
        lines.append(row('p99 (ms)',      '{:.2f}',  lambda r: r[stats_key].get('p99_ms', 0)))
        lines.append(row('p99.9 (ms)',    '{:.2f}',  lambda r: r[stats_key].get('p99_9_ms', 0)))
        lines.append(row('max (ms)',      '{:.2f}',  lambda r: r[stats_key].get('max_ms', 0)))
        lines.append(row('late>2× (%)',   '{:.2f}',  lambda r: r[stats_key].get('late_2x_pct', 0)))
        lines.append(row('late>3× (n)',   '{:d}',    lambda r: r[stats_key].get('late_3x', 0)))
        lines.append(row('dropped (n)',   '{:d}',    lambda r: r[stats_key].get('dropped_est', 0)))

        thr = cfg['thresholds'][threshold_key]
        lines.append('')
        lines.append(f'Thresholds: stddev ≤ {thr["stddev_ms_max"]} ms, '
                     f'p99 ≤ {thr["p99_ms_max"]} ms, '
                     f'late>2× ≤ {thr["late_2x_pct_max"]}%, '
                     f'late>3× ≤ {thr["late_3x_max"]}, '
                     f'dropped ≤ {thr["dropped_max"]}.')

    # Overall pass/fail per scenario
    lines.append('\n## Verdict\n')
    lines.append('| Scenario | IMU | RktGNC | Overall |')
    lines.append('|---|---|---|---|')
    for r in reports:
        imu_ok = '✅' if r['imu_verdict']['overall_pass'] else '❌'
        rkt_ok = '✅' if r['rkt_verdict']['overall_pass'] else '❌'
        ov = '✅ PASS' if r['overall_pass'] else '❌ FAIL'
        lines.append(f'| {r["scenario"]} | {imu_ok} | {rkt_ok} | {ov} |')

    # Failure details
    any_fail = False
    for r in reports:
        for side in ('imu_verdict', 'rkt_verdict'):
            if not r[side]['overall_pass']:
                if not any_fail:
                    lines.append('\n## Failure details\n')
                    any_fail = True
                fails = ', '.join(r[side].get('failures', []))
                lines.append(f'- **{r["scenario"]}** / {side}: {fails}')

    return '\n'.join(lines) + '\n'


def save_comparison_report(reports: List[dict], cfg: dict,
                           results_dir: str) -> str:
    md = build_comparison_report(reports, cfg)
    os.makedirs(results_dir, exist_ok=True)
    path = os.path.join(results_dir, 'comparison_report.md')
    with open(path, 'w') as f:
        f.write(md)
    # Also save HTML report with explicit numerical tables.
    html_path = os.path.join(results_dir, 'comparison_report.html')
    with open(html_path, 'w') as f:
        f.write(build_comparison_html(reports, cfg))
    return path


def build_comparison_html(reports: List[dict], cfg: dict) -> str:
    """Render the same data as build_comparison_report but in HTML with
    proper numerical tables — one stat per row, scenarios as columns."""
    if not reports:
        return ('<!DOCTYPE html><html><body><h1>Jitter Comparison — '
                '(no scenarios)</h1></body></html>')

    scenarios = [r['scenario'] for r in reports]
    style = (
        'body{font-family:-apple-system,Segoe UI,Roboto,sans-serif;'
        'margin:16px;background:#fafafa;color:#222}'
        '.card{background:#fff;border:1px solid #ddd;border-radius:6px;'
        'padding:12px;margin-bottom:16px;box-shadow:0 1px 2px rgba(0,0,0,.04)}'
        'table{border-collapse:collapse;width:100%;margin-bottom:8px}'
        'th,td{padding:6px 10px;border-bottom:1px solid #eee;text-align:right}'
        'th{background:#f3f4f6;font-weight:600;text-align:center}'
        'th:first-child,td:first-child{text-align:left}'
        'tr:hover td{background:#fafbff}'
        '.metric{font-family:ui-monospace,monospace;font-size:.85rem;color:#333}'
        '.pass{color:#0a8a0a;font-weight:700}.fail{color:#c00;font-weight:700}'
        '.cat{background:#f0f4f8;font-weight:700;text-align:left}'
        '.thr{font-size:.85rem;color:#666;margin-top:4px}'
    )

    def cell(v, fmt):
        try:
            return fmt.format(v)
        except Exception:
            return str(v)

    out = ['<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8">',
           '<title>Scheduler Jitter — Comparison</title>',
           f'<style>{style}</style></head><body>',
           '<h1>📊 Scheduler Jitter — Scenario Comparison</h1>',
           '<p style="color:#666;font-size:.9rem">Generated from internal PX4 timestamps (HRT). '
           'Wall/TCP figures are not used for pass/fail.</p>']

    rows_def = [
        ('count',        '{:d}',   lambda s: s.get('count', 0)),
        ('mean (ms)',    '{:.2f}', lambda s: s.get('mean_ms', 0)),
        ('stddev (ms)',  '{:.2f}', lambda s: s.get('stddev_ms', 0)),
        ('p50 (ms)',     '{:.2f}', lambda s: s.get('p50_ms', 0)),
        ('p95 (ms)',     '{:.2f}', lambda s: s.get('p95_ms', 0)),
        ('p99 (ms)',     '{:.2f}', lambda s: s.get('p99_ms', 0)),
        ('p99.9 (ms)',   '{:.2f}', lambda s: s.get('p99_9_ms', 0)),
        ('max (ms)',     '{:.2f}', lambda s: s.get('max_ms', 0)),
        ('late>2× (%)',  '{:.2f}', lambda s: s.get('late_2x_pct', 0)),
        ('late>3× (n)',  '{:d}',   lambda s: s.get('late_3x', 0)),
        ('dropped (n)',  '{:d}',   lambda s: s.get('dropped_est', 0)),
    ]

    for stream_label, stats_key, threshold_key in [
        ('HIGHRES_IMU (50 Hz, target 20 ms)', 'imu_stats', 'highres_imu'),
        ('RktGNC (target 40 ms)',             'rkt_stats', 'debug_float_array'),
    ]:
        out.append(f'<div class="card"><h2 style="margin-top:0">{stream_label}</h2>')
        out.append('<table><thead><tr><th>Metric</th>'
                   + ''.join(f'<th>{s}</th>' for s in scenarios)
                   + '</tr></thead><tbody>')
        for label, fmt, extractor in rows_def:
            cells = ''.join(f'<td class="metric">{cell(extractor(r[stats_key]), fmt)}</td>' for r in reports)
            out.append(f'<tr><td><b>{label}</b></td>{cells}</tr>')
        out.append('</tbody></table>')
        thr = cfg['thresholds'][threshold_key]
        out.append('<div class="thr">'
                   f'Thresholds: stddev ≤ {thr["stddev_ms_max"]} ms, '
                   f'p99 ≤ {thr["p99_ms_max"]} ms, '
                   f'late&gt;2× ≤ {thr["late_2x_pct_max"]}%, '
                   f'late&gt;3× ≤ {thr["late_3x_max"]}, '
                   f'dropped ≤ {thr["dropped_max"]}.'
                   '</div></div>')

    # Verdict
    out.append('<div class="card"><h2 style="margin-top:0">Verdict</h2>'
               '<table><thead><tr><th>Scenario</th><th>IMU</th><th>RktGNC</th><th>Overall</th></tr></thead><tbody>')
    for r in reports:
        imu_ok = r['imu_verdict']['overall_pass']
        rkt_ok = r['rkt_verdict']['overall_pass']
        ov_ok  = r['overall_pass']
        def b(ok):
            return f'<span class="{"pass" if ok else "fail"}">{"PASS" if ok else "FAIL"}</span>'
        out.append(f'<tr><td><b>{r["scenario"]}</b></td><td>{b(imu_ok)}</td>'
                   f'<td>{b(rkt_ok)}</td><td>{b(ov_ok)}</td></tr>')
    out.append('</tbody></table></div>')

    # Failures
    any_fail = False
    for r in reports:
        for side in ('imu_verdict', 'rkt_verdict'):
            if not r[side]['overall_pass']:
                if not any_fail:
                    out.append('<div class="card"><h2 style="margin-top:0">Failure Details</h2><ul>')
                    any_fail = True
                fails = ', '.join(r[side].get('failures', []))
                out.append(f'<li><b>{r["scenario"]}</b> / {side}: {fails}</li>')
    if any_fail:
        out.append('</ul></div>')

    out.append('</body></html>')
    return '\n'.join(out)


# ---------------------------------------------------------------------------
# Optional histogram plot
# ---------------------------------------------------------------------------

def save_histograms(reports: List[dict], captures: List[CaptureResult],
                    cfg: dict, results_dir: str) -> Optional[str]:
    """Render jitter histograms (one per stream, overlay scenarios). Needs matplotlib."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return None

    if not reports or not captures:
        return None

    imu_target = cfg['streams']['highres_imu']['target_ms']
    rkt_target = cfg['streams']['debug_float_array']['target_ms']

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    for cap in captures:
        iv_imu = intervals_from_us(cap.imu_time_usec)
        iv_rkt = intervals_from_us(cap.rkt_time_usec)
        axes[0].hist(iv_imu, bins=60, alpha=0.5, label=cap.scenario, range=(0, imu_target * 3))
        axes[1].hist(iv_rkt, bins=60, alpha=0.5, label=cap.scenario, range=(0, rkt_target * 3))

    axes[0].axvline(imu_target, color='red', linestyle='--', label=f'target {imu_target} ms')
    axes[0].set_title('HIGHRES_IMU inter-arrival intervals (internal)')
    axes[0].set_xlabel('interval (ms)')
    axes[0].set_ylabel('count')
    axes[0].legend()
    axes[0].grid(alpha=0.3)

    axes[1].axvline(rkt_target, color='red', linestyle='--', label=f'target {rkt_target} ms')
    axes[1].set_title('RktGNC inter-arrival intervals (internal)')
    axes[1].set_xlabel('interval (ms)')
    axes[1].set_ylabel('count')
    axes[1].legend()
    axes[1].grid(alpha=0.3)

    plt.tight_layout()
    path = os.path.join(results_dir, 'jitter_histograms.png')
    plt.savefig(path, dpi=120)
    plt.close(fig)
    return path
