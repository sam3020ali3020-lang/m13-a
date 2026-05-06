"""
watchdog_analysis.py — metrics + report for /watchdog scenarios
================================================================

Consumes the JSONL event log pulled from the phone plus the per-iteration
wall-clock timestamps the runner recorded, then emits:

  - A dict of per-iteration metrics (detection_ms, restart_ms, recovery_ms,
    cascading_deaths).
  - A Pass/Fail verdict by threshold-comparison.
  - A Markdown report with per-scenario tables.
  - An optional plotly timeline (if plotly is available).

The JSONL schema is defined by cpp/watchdog_native.cpp::log_event().
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from statistics import mean, median
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger("watchdog_analysis")


# ============================================================================
# Event-log parsing helpers
# ============================================================================

# Event types emitted by watchdog_native.cpp.  Keep in sync.
EVT_INIT                = "init"
EVT_SHUTDOWN            = "shutdown"
EVT_ALIVE               = "alive"
EVT_DEAD                = "dead"
EVT_CRASH_REQUESTED     = "crash_requested"
EVT_CRASH_COMPLETE      = "crash_complete"
EVT_CRASH_FAILED        = "crash_failed"
EVT_RESTART_REQUESTED   = "restart_requested"
EVT_RESTART_COMPLETE    = "restart_complete"
EVT_RESTART_FAILED      = "restart_failed"
EVT_AUTO_RESTART_TRIGGERED = "auto_restart_triggered"
EVT_AUTO_RESTART_OK     = "auto_restart_complete"
EVT_AUTO_RESTART_FAILED = "auto_restart_failed"
EVT_AUTO_RESTART_SET    = "auto_restart_set"


def _events_for_module(events: List[dict], module: str) -> List[dict]:
    return [e for e in events if e.get("module") == module]


def _first_after(events: List[dict], kind: str, min_t_us: int) -> Optional[dict]:
    for e in events:
        if e.get("event") == kind and int(e.get("t_us", 0)) >= min_t_us:
            return e
    return None


def _last_before(events: List[dict], kind: str, max_t_us: int) -> Optional[dict]:
    candidate = None
    for e in events:
        if e.get("event") == kind and int(e.get("t_us", 0)) <= max_t_us:
            candidate = e
    return candidate


# ============================================================================
# Per-iteration metrics
# ============================================================================

def _measure_crash_iteration(events: List[dict], module: str,
                             crash_completed_us: int) -> Dict[str, Any]:
    """Given the crash_complete event for a module, compute downstream timing.

    Returns a dict of metric_name → value_in_ms (or None if the phase
    wasn't observed).
    """
    scoped = _events_for_module(events, module)

    # 1) Detection: crash_complete → next 'dead' for same module
    dead = _first_after(scoped, EVT_DEAD, crash_completed_us)
    detection_ms: Optional[float] = None
    if dead:
        detection_ms = (int(dead["t_us"]) - crash_completed_us) / 1e3

    # 2) Restart: dead → next (auto|manual)_restart_complete
    restart_ms: Optional[float] = None
    restart_t_us: Optional[int] = None
    if dead:
        dead_t = int(dead["t_us"])
        rc = _first_after(
            scoped, EVT_AUTO_RESTART_OK, dead_t) or \
            _first_after(scoped, EVT_RESTART_COMPLETE, dead_t)
        if rc:
            restart_t_us = int(rc["t_us"])
            restart_ms = (restart_t_us - dead_t) / 1e3

    # 3) Recovery: dead → next 'alive'
    recovery_ms: Optional[float] = None
    if dead:
        alive = _first_after(scoped, EVT_ALIVE, int(dead["t_us"]))
        if alive:
            recovery_ms = (int(alive["t_us"]) - int(dead["t_us"])) / 1e3

    return {
        "detection_ms": detection_ms,
        "restart_ms":   restart_ms,
        "recovery_ms":  recovery_ms,
        "dead_t_us":    int(dead["t_us"]) if dead else None,
        "restart_t_us": restart_t_us,
    }


def _bystanders_affected(events: List[dict], bystanders: List[str],
                         window_start_us: int,
                         window_end_us: int) -> Dict[str, Dict[str, Any]]:
    """For each bystander, compute whether it went dead during the window
    and how long it took to recover.
    """
    report: Dict[str, Dict[str, Any]] = {}
    for b in bystanders:
        scoped = _events_for_module(events, b)
        dead_in_window = [
            e for e in scoped
            if e.get("event") == EVT_DEAD
            and window_start_us <= int(e["t_us"]) <= window_end_us
        ]
        if not dead_in_window:
            report[b] = {"died": False, "recovery_ms": 0.0}
            continue
        first_dead = dead_in_window[0]
        alive_after = _first_after(scoped, EVT_ALIVE, int(first_dead["t_us"]))
        if alive_after:
            recovery_ms = (int(alive_after["t_us"])
                           - int(first_dead["t_us"])) / 1e3
        else:
            recovery_ms = None  # didn't recover before log ended
        report[b] = {
            "died": True,
            "dead_t_us": int(first_dead["t_us"]),
            "recovery_ms": recovery_ms,
        }
    return report


# ============================================================================
# Scenario-level analysis
# ============================================================================

def analyse_scenario(result, thresholds: dict,
                     modules_cfg: dict) -> Dict[str, Any]:
    """Compute metrics + pass/fail for one scenario's ScenarioResult.

    This function is pure and side-effect free; the runner writes the
    returned dict to disk.
    """
    ev = result.events
    iterations = result.iterations
    per_iter: List[Dict[str, Any]] = []
    fail_reasons: List[str] = []

    # If the scenario aborted before running any iterations, nothing to do.
    if not iterations:
        return {
            "scenario":         result.name,
            "passed":           False,
            "iterations":       [],
            "aggregate":        {},
            "failure_reasons":  ["no iterations executed"] + result.failures,
            "event_count":      len(ev),
        }

    # Find the crash_complete events in the log to anchor timing.  The
    # runner's `t_crash_wallclock` is only an ordering hint because HRT
    # (device) and wallclock (host) aren't aligned.  We instead match
    # each iteration to the N-th crash_complete event for the same
    # target module.
    per_module_crash_idx: Dict[str, int] = {}
    crash_events = [e for e in ev if e.get("event") == EVT_CRASH_COMPLETE]

    for it in iterations:
        is_cascade = "victim" in it
        victim = it.get("victim") or it.get("target")
        idx = per_module_crash_idx.get(victim, 0)
        matching = [e for e in crash_events if e.get("module") == victim]
        if idx >= len(matching):
            per_iter.append({
                **it,
                "metrics": None,
                "note":    "no crash_complete event in log for this iteration",
            })
            fail_reasons.append(
                f"{victim}: missing crash_complete event "
                f"(iter {it.get('rep')})"
            )
            per_module_crash_idx[victim] = idx + 1
            continue

        crash_evt = matching[idx]
        per_module_crash_idx[victim] = idx + 1
        t_crash = int(crash_evt["t_us"])
        metrics = _measure_crash_iteration(ev, victim, t_crash)

        # For manual_restart: recovery should be measured from the MANUAL
        # restart, not from the auto path (which is disabled).  Distinguish
        # by the iteration's presence of t_restart_wallclock.
        if "t_restart_wallclock" in it:
            # Override restart timing: manual path uses explicit request.
            scoped = _events_for_module(ev, victim)
            rc = _first_after(scoped, EVT_RESTART_COMPLETE, t_crash)
            if rc and metrics["dead_t_us"] is not None:
                metrics["restart_ms"] = (int(rc["t_us"])
                                         - metrics["dead_t_us"]) / 1e3
                metrics["restart_t_us"] = int(rc["t_us"])

        # Cascading: also measure bystanders.
        if is_cascade:
            window_start = t_crash
            # window extends to last event of any kind seen after the crash
            window_end = max((int(e["t_us"]) for e in ev
                              if int(e["t_us"]) >= t_crash),
                             default=t_crash)
            metrics["bystanders"] = _bystanders_affected(
                ev, it.get("bystanders", []), window_start, window_end)

        # Threshold checks per iteration.
        # Detection cap is per-module: stale_ms + detection_safety_ms.
        # This prevents spurious failures for modules with deliberately
        # long stale windows (mavlink_tcp_bridge=2000ms, navigator=5000ms)
        # while keeping flight-critical modules tight.
        it_fail: List[str] = []
        victim_cfg = (modules_cfg or {}).get(victim, {}) or {}
        stale_ms = victim_cfg.get("stale_ms")
        if stale_ms is not None:
            detection_cap = float(stale_ms) + float(
                thresholds.get("detection_safety_ms", 250))
        else:
            # Fall back to legacy behaviour if stale_ms is missing.
            detection_cap = float(thresholds.get("detection_ms_max", 1e9))
        # Optional absolute ceiling (e.g. to disqualify obviously stuck
        # cases regardless of how lax the per-module cap is).
        absolute_cap = thresholds.get("detection_ms_absolute_max", -1)
        if absolute_cap is not None and absolute_cap > 0:
            detection_cap = min(detection_cap, float(absolute_cap))

        if metrics["detection_ms"] is None:
            it_fail.append("never detected 'dead'")
        elif metrics["detection_ms"] > detection_cap:
            it_fail.append(
                f"detection {metrics['detection_ms']:.0f}ms "
                f"> {detection_cap:.0f}ms (stale={stale_ms}+safety)"
            )
        if metrics["recovery_ms"] is None:
            it_fail.append("never recovered")
        elif metrics["recovery_ms"] > thresholds.get("recovery_ms_max", 1e9):
            it_fail.append(
                f"recovery {metrics['recovery_ms']:.0f}ms "
                f"> {thresholds.get('recovery_ms_max')}ms"
            )
        if metrics["restart_ms"] is not None and \
           metrics["restart_ms"] > thresholds.get("restart_ms_max", 1e9):
            it_fail.append(
                f"restart {metrics['restart_ms']:.0f}ms "
                f"> {thresholds.get('restart_ms_max')}ms"
            )

        if is_cascade and metrics.get("bystanders"):
            cap = thresholds.get("bystander_recovery_ms_max", 1e9)
            for b, b_info in metrics["bystanders"].items():
                rec = b_info.get("recovery_ms")
                if b_info.get("died") and (rec is None or rec > cap):
                    it_fail.append(
                        f"bystander {b} recovery "
                        f"{'none' if rec is None else f'{rec:.0f}ms'} > {cap}ms"
                    )

        if it_fail:
            fail_reasons.extend([f"{victim}#{it.get('rep')}: {m}"
                                 for m in it_fail])

        per_iter.append({**it, "metrics": metrics, "iter_fail": it_fail})

    aggregate = _aggregate(per_iter)
    max_fails = thresholds.get("max_failures_per_scenario", 0)
    passed = len(fail_reasons) <= max_fails and bool(iterations)

    return {
        "scenario":         result.name,
        "passed":           passed,
        "iterations":       per_iter,
        "aggregate":        aggregate,
        "failure_reasons":  fail_reasons,
        "event_count":      len(ev),
    }


def _aggregate(iters: List[dict]) -> Dict[str, Dict[str, float]]:
    """Group metrics by module, compute mean / median / max."""
    grouped: Dict[str, Dict[str, List[float]]] = {}
    for it in iters:
        m = it.get("metrics") or {}
        if not m:
            continue
        target = it.get("victim") or it.get("target", "?")
        gb = grouped.setdefault(target, {
            "detection_ms": [], "restart_ms": [], "recovery_ms": []
        })
        for k in ("detection_ms", "restart_ms", "recovery_ms"):
            v = m.get(k)
            if v is not None:
                gb[k].append(float(v))

    out: Dict[str, Dict[str, float]] = {}
    for target, cols in grouped.items():
        row: Dict[str, float] = {}
        for k, vs in cols.items():
            if not vs:
                continue
            row[f"{k}_mean"]   = mean(vs)
            row[f"{k}_median"] = median(vs)
            row[f"{k}_max"]    = max(vs)
            row[f"{k}_count"]  = float(len(vs))
        out[target] = row
    return out


# ============================================================================
# Report writer
# ============================================================================

def write_report(results: List, cfg: dict, out_dir: Path) -> Path:
    path = out_dir / "watchdog_report.md"
    lines: List[str] = []
    lines.append("# Watchdog Test Report\n")
    lines.append(f"- results dir: `{out_dir}`\n")
    lines.append(f"- scenarios run: {len(results)}\n")
    overall = all(r.passed for r in results) and bool(results)
    lines.append(f"- overall: **{'PASS' if overall else 'FAIL'}**\n")

    thresholds = cfg.get("thresholds", {})
    lines.append("\n## Thresholds\n")
    lines.append("| metric | max |\n|---|---|\n")
    for k, v in thresholds.items():
        lines.append(f"| `{k}` | {v} |\n")

    for r in results:
        metrics_path = out_dir / f"{r.name}_metrics.json"
        metrics = json.loads(metrics_path.read_text()) \
            if metrics_path.exists() else {}
        lines.append(f"\n## Scenario: `{r.name}` "
                     f"— {'PASS' if r.passed else 'FAIL'}\n")
        if r.failures:
            lines.append("**Failures:**\n")
            for f in r.failures:
                lines.append(f"- {f}\n")

        # per-module aggregate table
        agg = metrics.get("aggregate", {})
        if agg:
            lines.append("\n| module | det_med (ms) | det_max | "
                         "restart_med | recovery_med | recovery_max | N |\n")
            lines.append("|---|---|---|---|---|---|---|\n")
            for mod, row in agg.items():
                lines.append(
                    f"| `{mod}` "
                    f"| {_fmt(row.get('detection_ms_median'))} "
                    f"| {_fmt(row.get('detection_ms_max'))} "
                    f"| {_fmt(row.get('restart_ms_median'))} "
                    f"| {_fmt(row.get('recovery_ms_median'))} "
                    f"| {_fmt(row.get('recovery_ms_max'))} "
                    f"| {int(row.get('detection_ms_count', 0))} |\n"
                )

    path.write_text("".join(lines))
    _maybe_plot(results, out_dir)
    return path


def _fmt(v: Optional[float]) -> str:
    if v is None:
        return "—"
    return f"{v:.0f}"


def _maybe_plot(results: List, out_dir: Path) -> None:
    try:
        import plotly.graph_objects as go  # type: ignore
    except ImportError:
        return
    import plotly.io as pio  # type: ignore

    # Simple bar chart per scenario: recovery_ms by module.
    fig = go.Figure()
    for r in results:
        metrics_path = out_dir / f"{r.name}_metrics.json"
        if not metrics_path.exists():
            continue
        m = json.loads(metrics_path.read_text())
        agg = m.get("aggregate", {})
        if not agg:
            continue
        xs = list(agg.keys())
        ys = [agg[m0].get("recovery_ms_median") or 0 for m0 in xs]
        fig.add_trace(go.Bar(name=r.name, x=xs, y=ys))
    if fig.data:
        fig.update_layout(
            title="Watchdog recovery_ms median by module",
            yaxis_title="ms", barmode="group"
        )
        fig_html = fig.to_html(full_html=False, include_plotlyjs="cdn")
        table_html = _watchdog_metrics_table_html(results, out_dir)
        page = (
            '<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8">'
            '<title>Watchdog Test — Report</title>'
            '<style>body{font-family:-apple-system,Segoe UI,Roboto,sans-serif;'
            'margin:16px;background:#fafafa;color:#222}'
            '.card{background:#fff;border:1px solid #ddd;border-radius:6px;'
            'padding:12px;margin-bottom:16px;box-shadow:0 1px 2px rgba(0,0,0,.04)}'
            'table{border-collapse:collapse;width:100%}'
            'th,td{padding:6px 10px;border-bottom:1px solid #eee;text-align:left}'
            'th{background:#f3f4f6;font-weight:600}'
            '.num{text-align:right;font-variant-numeric:tabular-nums;font-weight:600}'
            '.unit{color:#888;font-size:.85rem}'
            '.cat{background:#f0f4f8;font-weight:700}'
            '.pass{color:#0a8a0a}.fail{color:#c00}'
            '</style></head><body>'
            '<h1>Watchdog Test — Report</h1>'
            f'{table_html}<div class="card">{fig_html}</div></body></html>'
        )
        (out_dir / "watchdog_plot.html").write_text(page, encoding="utf-8")


def _watchdog_metrics_table_html(results, out_dir: Path) -> str:
    """Render watchdog scenarios + per-module stats as a flat HTML table."""
    rows = []
    def cat(label):
        rows.append((label, None, None, None))
    def add(k, v, unit="", cls=None):
        if v is None:
            v = "—"
        elif isinstance(v, float):
            v = f"{v:.2f}" if abs(v) < 100 else f"{v:.0f}"
        elif isinstance(v, int):
            v = f"{v:,}"
        else:
            v = str(v)
        rows.append((k, v, unit, cls))

    cat("Scenario Summary")
    for r in results:
        passed = bool(getattr(r, "passed", False))
        rows.append((r.name, "PASS" if passed else "FAIL", "",
                     "pass" if passed else "fail"))
        for f in getattr(r, "failures", []) or []:
            rows.append((f"  {r.name}.failure", str(f), "", "fail"))

    # Per-scenario aggregate stats from cached JSON
    for r in results:
        metrics_path = out_dir / f"{r.name}_metrics.json"
        if not metrics_path.exists():
            continue
        try:
            data = json.loads(metrics_path.read_text())
        except Exception:
            continue
        cat(f"Scenario: {r.name}")
        for k, v in data.items():
            if k in ("aggregate", "iterations", "failure_reasons"):
                continue
            if isinstance(v, (int, float, bool, str)) or v is None:
                add(f"{r.name}.{k}", v)
        agg = data.get("aggregate", {})
        if isinstance(agg, dict):
            for module, stats in agg.items():
                if not isinstance(stats, dict):
                    continue
                for sk, sv in stats.items():
                    unit = "ms" if "ms" in sk else ("" if any(s in sk for s in ("count","fail","pass")) else "")
                    add(f"{r.name}.{module}.{sk}", sv, unit)

    body = ""
    for label, val, unit, cls in rows:
        if val is None:
            body += f'<tr class="cat"><td colspan="3">■ {label}</td></tr>'
        else:
            cls_attr = f' class="num {cls}"' if cls else ' class="num"'
            body += (f'<tr><td style="font-family:ui-monospace,monospace;font-size:.85rem">{label}</td>'
                     f'<td{cls_attr}>{val}</td>'
                     f'<td class="unit">{unit}</td></tr>')
    return ('<div class="card"><h2 style="margin-top:0">📊 Numerical Metrics</h2>'
            '<table><thead><tr><th>Metric</th><th style="text-align:right">Value</th>'
            f'<th>Unit</th></tr></thead><tbody>{body}</tbody></table></div>')


# ============================================================================
# Re-analyse an existing results dir
# ============================================================================

def reanalyse_dir(old_dir: Path, cfg: dict) -> int:
    """Rebuild the report from cached JSON metrics + events."""
    class _Stub:
        pass

    results = []
    for metrics_file in sorted(old_dir.glob("*_metrics.json")):
        name = metrics_file.stem.replace("_metrics", "")
        data = json.loads(metrics_file.read_text())
        stub = _Stub()
        stub.name = name
        stub.passed = data.get("passed", False)
        stub.failures = data.get("failure_reasons", [])
        stub.events = []        # rebuilding from JSONL optional
        jsonl = old_dir / f"{name}_events.jsonl"
        if jsonl.exists():
            stub.events = [json.loads(l) for l in jsonl.read_text().splitlines()
                           if l.strip()]
        stub.iterations = data.get("iterations", [])
        results.append(stub)

    path = write_report(results, cfg, old_dir)
    print(f"re-analysed → {path}")
    return 0
