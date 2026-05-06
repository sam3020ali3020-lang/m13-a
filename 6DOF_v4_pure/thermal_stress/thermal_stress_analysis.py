#!/usr/bin/env python3
"""
thermal_stress_analysis.py — Post-run analysis
================================================

Given a result directory containing ``thermal_log.csv`` and
``mpc_timing.csv``, compute:

1. **MPC solve-time percentiles** — p50 / p95 / p99 / p99.9 of
   ``mpc_solve_us`` and ``cycle_us``.  Uses dt_max for a tighter worst-case
   cycle bound when available.

2. **Deadline-miss rate** — fraction of cycles where ``max(cycle_us,
   dt_max*1e6)`` exceeds ``mpc.deadline_us``.  Reported per-minute and
   overall.

3. **Thermal throttling detection** — time windows where
   ``throttle_ratio_gold < 1.0`` (kernel capped big-core max frequency
   below hardware max).  Also flag ``thermal_status >= 2`` (MODERATE).

4. **Pre-throttle vs post-throttle comparison** — split MPC samples at
   the first throttle onset and compare p50/p99 across halves.

5. **Visualization** — Plotly HTML dashboard with three synchronized
   traces: temperature, CPU freq cap, MPC solve time.

PASS / FAIL logic pulls thresholds from ``config_used.yaml`` (or config
passed in by the runner).
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger("thermal_stress_analysis")


# ---------------------------------------------------------------------------
# CSV loaders (dependency-free — no pandas)
# ---------------------------------------------------------------------------

def _load_csv(path: Path) -> Tuple[List[str], np.ndarray]:
    """Load CSV as (header, 2D array of floats).  Non-numeric cells become NaN."""
    if not path.exists():
        return [], np.zeros((0, 0))
    with open(path) as f:
        header = f.readline().strip().split(",")
        rows = []
        for line in f:
            if not line.strip():
                continue
            parts = line.rstrip("\n").split(",")
            row = []
            for p in parts:
                try:
                    row.append(float(p))
                except (ValueError, TypeError):
                    row.append(float("nan"))
            # Pad/truncate to header length
            if len(row) < len(header):
                row.extend([float("nan")] * (len(header) - len(row)))
            elif len(row) > len(header):
                row = row[:len(header)]
            rows.append(row)
    if not rows:
        return header, np.zeros((0, len(header)))
    return header, np.array(rows, dtype=float)


def _col(header: List[str], name: str, data: np.ndarray) -> Optional[np.ndarray]:
    try:
        idx = header.index(name)
        return data[:, idx]
    except (ValueError, IndexError):
        return None


# ---------------------------------------------------------------------------
# Percentile helper
# ---------------------------------------------------------------------------

def _percentiles(arr: np.ndarray, qs=(50, 95, 99, 99.9)) -> Dict[str, float]:
    """Return {pQ: value} for the given quantiles (percent).  NaN-safe."""
    if arr is None or len(arr) == 0:
        return {f"p{q}".replace(".", "_"): float("nan") for q in qs}
    finite = arr[np.isfinite(arr)]
    if len(finite) == 0:
        return {f"p{q}".replace(".", "_"): float("nan") for q in qs}
    # Use linear interp — numpy default
    vals = np.percentile(finite, qs)
    return {f"p{q}".replace(".", "_"): float(v) for q, v in zip(qs, vals)}


# ---------------------------------------------------------------------------
# Throttling timeline detection
# ---------------------------------------------------------------------------

@dataclass
class ThrottleWindow:
    t_start: float
    t_end: float
    min_ratio: float  # min throttle_ratio_gold during window
    peak_status: int  # max thermal_status during window

    @property
    def duration_s(self) -> float:
        return self.t_end - self.t_start

    def to_dict(self) -> dict:
        return {
            "t_start": self.t_start,
            "t_end": self.t_end,
            "duration_s": self.duration_s,
            "min_ratio": self.min_ratio,
            "peak_status": self.peak_status,
        }


def detect_throttle_windows(t: np.ndarray,
                             ratio_gold: np.ndarray,
                             thermal_status: np.ndarray,
                             ratio_threshold: float = 0.98,
                             status_threshold: int = 1,
                             min_duration_s: float = 2.0
                             ) -> List[ThrottleWindow]:
    """Detect contiguous windows where gold throttle_ratio drops below
    ``ratio_threshold`` OR ``thermal_status >= status_threshold``.

    Windows shorter than ``min_duration_s`` are filtered out (single-poll
    glitches).
    """
    if len(t) == 0:
        return []

    mask = (ratio_gold < ratio_threshold) | (thermal_status >= status_threshold)
    windows: List[ThrottleWindow] = []
    in_window = False
    w_start_i = 0
    for i in range(len(t)):
        if mask[i] and not in_window:
            in_window = True
            w_start_i = i
        elif not mask[i] and in_window:
            in_window = False
            _finalize(windows, t, ratio_gold, thermal_status,
                      w_start_i, i - 1, min_duration_s)
    if in_window:
        _finalize(windows, t, ratio_gold, thermal_status,
                  w_start_i, len(t) - 1, min_duration_s)
    return windows


def _finalize(windows, t, ratio_gold, thermal_status,
              start_i, end_i, min_duration_s):
    if end_i < start_i:
        return
    dur = t[end_i] - t[start_i]
    if dur < min_duration_s:
        return
    windows.append(ThrottleWindow(
        t_start=float(t[start_i]),
        t_end=float(t[end_i]),
        min_ratio=float(np.nanmin(ratio_gold[start_i:end_i + 1])),
        peak_status=int(np.nanmax(thermal_status[start_i:end_i + 1])),
    ))


# ---------------------------------------------------------------------------
# Main analysis
# ---------------------------------------------------------------------------

def analyze_directory(result_dir: Path,
                       thresholds: Optional[dict] = None,
                       mpc_cfg: Optional[dict] = None) -> dict:
    """Load all CSVs in ``result_dir`` and compute full metrics."""
    result_dir = Path(result_dir)
    thresholds = thresholds or {}
    mpc_cfg = mpc_cfg or {}
    deadline_us = float(mpc_cfg.get("deadline_us", 40000))
    warn_us = float(mpc_cfg.get("warn_us", 25000))

    # ------------------------------------------------------------------
    # Load MPC timing
    # ------------------------------------------------------------------
    mpc_path = result_dir / "mpc_timing.csv"
    mpc_hdr, mpc_data = _load_csv(mpc_path)

    mpc_metrics = _analyze_mpc(mpc_hdr, mpc_data, deadline_us, warn_us)

    # ------------------------------------------------------------------
    # Load thermal
    # ------------------------------------------------------------------
    th_path = result_dir / "thermal_log.csv"
    th_hdr, th_data = _load_csv(th_path)

    thermal_metrics, throttle_windows = _analyze_thermal(th_hdr, th_data)

    # ------------------------------------------------------------------
    # Split MPC stats by throttling (pre-first-throttle vs post)
    # ------------------------------------------------------------------
    split_metrics = _split_by_throttle(mpc_hdr, mpc_data, throttle_windows,
                                        deadline_us)

    # ------------------------------------------------------------------
    # PASS / FAIL verdict
    # ------------------------------------------------------------------
    pf = _verdict(mpc_metrics, thermal_metrics, split_metrics, thresholds,
                  mpc_cfg)

    return {
        "result_dir": str(result_dir),
        "deadline_us": deadline_us,
        "mpc": mpc_metrics,
        "thermal": thermal_metrics,
        "throttle_windows": [w.to_dict() for w in throttle_windows],
        "split_by_throttle": split_metrics,
        "pass_fail": pf,
    }


def _analyze_mpc(header: List[str], data: np.ndarray,
                  deadline_us: float, warn_us: float) -> dict:
    if len(data) == 0:
        return {"error": "no MPC data (mpc_timing.csv missing or empty)",
                "samples": 0}

    t = _col(header, "t_wall_s", data)
    mpc_us = _col(header, "mpc_solve_us", data)
    mhe_us = _col(header, "mhe_solve_us", data)
    cycle_us = _col(header, "cycle_us", data)
    dt_actual = _col(header, "dt_actual_s", data)
    dt_max_s = _col(header, "dt_max_s", data)
    dt_min_s = _col(header, "dt_min_s", data)
    solver_status = _col(header, "mpc_solver_status", data)
    launched = _col(header, "launched", data)

    # Only include rows with non-trivial mpc_solve_us.  If MPC is idle
    # (pre-launch, solver not invoked), mpc_solve_us may be 0 — those
    # shouldn't drag down our p50.  But keep them for deadline-miss count
    # because dt_max still reflects scheduling hiccups.
    mpc_valid = mpc_us > 0 if mpc_us is not None else None

    pct_mpc = _percentiles(mpc_us[mpc_valid] if mpc_valid is not None and mpc_valid.any() else mpc_us)
    pct_mhe = _percentiles(mhe_us)
    pct_cycle = _percentiles(cycle_us)

    # For deadline detection, use max(cycle_us, dt_max_s*1e6).  dt_max
    # reflects the worst scheduling interval on PX4 side between consecutive
    # RktGNC samples — very good signal.
    if dt_max_s is not None:
        dt_max_us = dt_max_s * 1e6
    else:
        dt_max_us = np.full_like(cycle_us, np.nan) if cycle_us is not None else None

    if cycle_us is not None and dt_max_us is not None:
        eff_cycle = np.fmax(cycle_us, dt_max_us)
    elif cycle_us is not None:
        eff_cycle = cycle_us
    elif dt_max_us is not None:
        eff_cycle = dt_max_us
    else:
        eff_cycle = None
    pct_eff_cycle = _percentiles(eff_cycle)

    # Deadline miss rate — only count rows where PX4 was actively solving
    miss_count = 0
    warn_count = 0
    miss_total = 0
    if eff_cycle is not None:
        finite_mask = np.isfinite(eff_cycle) & (eff_cycle > 0)
        miss_total = int(np.sum(finite_mask))
        if miss_total > 0:
            miss_count = int(np.sum(eff_cycle[finite_mask] > deadline_us))
            warn_count = int(np.sum(eff_cycle[finite_mask] > warn_us))

    miss_rate = miss_count / miss_total if miss_total > 0 else 0.0
    warn_rate = warn_count / miss_total if miss_total > 0 else 0.0

    # Solver errors
    solver_error_count = 0
    if solver_status is not None:
        solver_error_count = int(np.sum(solver_status > 0))

    # Launched stats
    launched_count = int(np.sum(launched > 0.5)) if launched is not None else 0

    # Rate of RktGNC messages (cadence check)
    rate_hz = 0.0
    if t is not None and len(t) > 2:
        span = float(t[-1] - t[0])
        if span > 0:
            rate_hz = (len(t) - 1) / span

    return {
        "samples": int(len(data)),
        "launched_samples": launched_count,
        "duration_s": round(float(t[-1] - t[0]), 2) if t is not None and len(t) > 1 else 0.0,
        "sample_rate_hz": round(rate_hz, 2),
        "mpc_solve_us": {
            "mean": round(float(np.nanmean(mpc_us)) if mpc_us is not None else float("nan"), 1),
            "min": round(float(np.nanmin(mpc_us[mpc_us > 0])) if mpc_us is not None and np.any(mpc_us > 0) else float("nan"), 1),
            "max": round(float(np.nanmax(mpc_us)) if mpc_us is not None else float("nan"), 1),
            **{k: round(v, 1) for k, v in pct_mpc.items()},
        },
        "mhe_solve_us": {k: round(v, 1) for k, v in pct_mhe.items()},
        "cycle_us": {k: round(v, 1) for k, v in pct_cycle.items()},
        "eff_cycle_us": {
            "max": round(float(np.nanmax(eff_cycle)) if eff_cycle is not None else float("nan"), 1),
            **{k: round(v, 1) for k, v in pct_eff_cycle.items()},
        },
        "deadline_miss": {
            "deadline_us": deadline_us,
            "warn_us": warn_us,
            "total_cycles": miss_total,
            "miss_count": miss_count,
            "warn_count": warn_count,
            "miss_rate": round(miss_rate, 6),
            "warn_rate": round(warn_rate, 6),
        },
        "solver_errors": solver_error_count,
    }


def _analyze_thermal(header: List[str], data: np.ndarray
                      ) -> Tuple[dict, List[ThrottleWindow]]:
    if len(data) == 0:
        return ({"error": "no thermal data", "samples": 0}, [])

    t = _col(header, "t_wall_s", data)
    cpu_silver = _col(header, "cpu_silver_max_C", data)
    cpu_gold = _col(header, "cpu_gold_max_C", data)
    gpu = _col(header, "gpu_max_C", data)
    ddr = _col(header, "ddr_max_C", data)
    batt = _col(header, "battery_C", data)
    skin = _col(header, "hal_skin_C", data)
    status = _col(header, "thermal_status", data)
    throttle_gold = _col(header, "throttle_ratio_gold", data)
    throttle_silver = _col(header, "throttle_ratio_silver", data)

    if t is not None and len(t) > 0:
        t_zero = t - t[0]
    else:
        t_zero = np.array([])

    # Detect throttle windows (use zero-based time)
    windows = detect_throttle_windows(
        t_zero,
        throttle_gold if throttle_gold is not None else np.ones_like(t_zero),
        status if status is not None else np.zeros_like(t_zero),
        ratio_threshold=0.98,
        status_threshold=1,
        min_duration_s=2.0,
    )

    def _stats(arr):
        if arr is None:
            return {"mean": float("nan"), "min": float("nan"),
                    "max": float("nan"), "final": float("nan")}
        finite = arr[np.isfinite(arr)]
        if len(finite) == 0:
            return {"mean": float("nan"), "min": float("nan"),
                    "max": float("nan"), "final": float("nan")}
        return {
            "mean": round(float(np.mean(finite)), 2),
            "min": round(float(np.min(finite)), 2),
            "max": round(float(np.max(finite)), 2),
            "final": round(float(finite[-1]), 2),
        }

    metrics = {
        "samples": int(len(data)),
        "duration_s": round(float(t[-1] - t[0]), 1) if t is not None and len(t) > 1 else 0.0,
        "cpu_silver_C": _stats(cpu_silver),
        "cpu_gold_C": _stats(cpu_gold),
        "gpu_C": _stats(gpu),
        "ddr_C": _stats(ddr),
        "battery_C": _stats(batt),
        "skin_C": _stats(skin),
        "thermal_status_peak": int(np.nanmax(status)) if status is not None and len(status) > 0 else 0,
        "thermal_status_samples_gt0": int(np.sum(status > 0)) if status is not None else 0,
        "throttle_ratio_gold": _stats(throttle_gold),
        "throttle_ratio_silver": _stats(throttle_silver),
        "throttle_windows_count": len(windows),
        "total_throttle_s": round(sum(w.duration_s for w in windows), 1),
    }
    return metrics, windows


def _split_by_throttle(mpc_header: List[str], mpc_data: np.ndarray,
                        throttle_windows: List[ThrottleWindow],
                        deadline_us: float) -> dict:
    """Compare MPC stats pre- and post-first-throttle onset.

    If no throttle window was observed, returns empty dict.  Otherwise
    uses the first window's start as the split point and reports p99
    solve time + deadline miss rate on each side.
    """
    if not throttle_windows or len(mpc_data) == 0:
        return {"no_throttle_observed": True}

    t = _col(mpc_header, "t_wall_s", mpc_data)
    mpc_us = _col(mpc_header, "mpc_solve_us", mpc_data)
    cycle_us = _col(mpc_header, "cycle_us", mpc_data)
    dt_max_s = _col(mpc_header, "dt_max_s", mpc_data)

    if t is None or len(t) == 0:
        return {"no_mpc_data": True}

    # Normalize MPC time to same zero as thermal time
    t0 = t[0]
    t_rel = t - t0
    split_t = throttle_windows[0].t_start

    pre_mask = t_rel < split_t
    post_mask = t_rel >= split_t

    def _half_stats(mask):
        if not np.any(mask):
            return {"n": 0}
        m = mask & (mpc_us > 0) if mpc_us is not None else mask
        n = int(np.sum(m))
        if n == 0:
            return {"n": 0}
        pct_mpc = _percentiles(mpc_us[m], qs=(50, 95, 99))
        pct_cycle = _percentiles(cycle_us[mask], qs=(50, 95, 99)) if cycle_us is not None else {}
        eff_cycle = cycle_us
        if cycle_us is not None and dt_max_s is not None:
            eff_cycle = np.fmax(cycle_us, dt_max_s * 1e6)
        miss_rate = 0.0
        if eff_cycle is not None:
            mm = mask & np.isfinite(eff_cycle) & (eff_cycle > 0)
            total = int(np.sum(mm))
            miss = int(np.sum(eff_cycle[mm] > deadline_us))
            miss_rate = miss / total if total > 0 else 0.0
        return {
            "n": n,
            "mpc_p50_us": round(pct_mpc["p50"], 1),
            "mpc_p95_us": round(pct_mpc["p95"], 1),
            "mpc_p99_us": round(pct_mpc["p99"], 1),
            "cycle_p99_us": round(pct_cycle.get("p99", float("nan")), 1),
            "deadline_miss_rate": round(miss_rate, 6),
        }

    return {
        "first_throttle_at_s": round(float(split_t), 1),
        "pre_throttle": _half_stats(pre_mask),
        "post_throttle": _half_stats(post_mask),
    }


# ---------------------------------------------------------------------------
# Verdict
# ---------------------------------------------------------------------------

def _verdict(mpc_metrics: dict, thermal_metrics: dict,
              split_metrics: dict, thresholds: dict, mpc_cfg: dict) -> dict:
    """Compute overall PASS / FAIL based on thresholds."""
    th_mpc = thresholds.get("mpc_solve_us", {})
    th_cycle = thresholds.get("cycle_us", {})
    th_miss = thresholds.get("deadline_miss_rate", {})
    th_degrade = thresholds.get("throttle_degradation", {})
    allow_solver_errors = bool(thresholds.get("allow_solver_errors", False))

    failures: List[str] = []
    warnings: List[str] = []

    # --- No MPC data sentinel --------------------------------------------
    # If mpc_timing.csv has 0 RktGNC rows we can't claim PASS — the MPC was
    # never observed running (PX4 likely off or rocket didn't launch).
    if isinstance(mpc_metrics, dict):
        if mpc_metrics.get("error") or mpc_metrics.get("samples", 0) == 0:
            warnings.append("No MPC data captured — PX4 may not have been running")
        elif mpc_metrics.get("launched_samples", 0) == 0:
            warnings.append("Rocket never marked 'launched' — MPC may be in pre-flight idle")

    # --- MPC solve time ---------------------------------------------------
    mpc_s = mpc_metrics.get("mpc_solve_us", {}) if isinstance(mpc_metrics, dict) else {}
    for q, limit_key in (("p50", "p50_max"), ("p95", "p95_max"),
                         ("p99", "p99_max"), ("p99_9", "p999_max")):
        v = mpc_s.get(q)
        limit = th_mpc.get(limit_key)
        if v is None or limit is None:
            continue
        if math.isnan(v):
            continue
        if v > limit:
            failures.append(f"MPC solve {q}={v:.0f}μs > {limit:.0f}μs")

    # --- Cycle time -------------------------------------------------------
    cyc_s = mpc_metrics.get("eff_cycle_us", {}) if isinstance(mpc_metrics, dict) else {}
    for q, limit_key in (("p50", "p50_max"), ("p95", "p95_max"),
                         ("p99", "p99_max"), ("p99_9", "p999_max")):
        v = cyc_s.get(q)
        limit = th_cycle.get(limit_key)
        if v is None or limit is None:
            continue
        if math.isnan(v):
            continue
        if v > limit:
            failures.append(f"Cycle {q}={v:.0f}μs > {limit:.0f}μs")

    # --- Deadline miss rate ----------------------------------------------
    dm = mpc_metrics.get("deadline_miss", {}) if isinstance(mpc_metrics, dict) else {}
    miss_rate = dm.get("miss_rate", 0.0)
    warn_limit = th_miss.get("warn", 0.05)
    fail_limit = th_miss.get("fail", 0.20)
    if miss_rate >= fail_limit:
        failures.append(f"Deadline miss rate {miss_rate*100:.2f}% >= {fail_limit*100:.0f}%")
    elif miss_rate >= warn_limit:
        warnings.append(f"Deadline miss rate {miss_rate*100:.2f}% >= {warn_limit*100:.0f}%")

    # --- Solver errors ----------------------------------------------------
    se = mpc_metrics.get("solver_errors", 0)
    if se > 0 and not allow_solver_errors:
        failures.append(f"MPC solver errors observed: {se}")

    # --- Throttle-induced degradation ------------------------------------
    if "pre_throttle" in split_metrics and "post_throttle" in split_metrics:
        pre = split_metrics["pre_throttle"]
        post = split_metrics["post_throttle"]
        if pre.get("n", 0) > 10 and post.get("n", 0) > 10:
            pre_p99 = pre.get("mpc_p99_us") or 1.0
            post_p99 = post.get("mpc_p99_us") or 1.0
            if pre_p99 > 0:
                ratio = post_p99 / pre_p99
                ratio_max = th_degrade.get("solve_us_ratio_max", 2.0)
                if ratio > ratio_max:
                    failures.append(f"Post-throttle p99 solve {ratio:.2f}× > "
                                    f"pre-throttle (limit {ratio_max}×)")

            pre_miss = pre.get("deadline_miss_rate", 0.0)
            post_miss = post.get("deadline_miss_rate", 0.0)
            delta = post_miss - pre_miss
            delta_max = th_degrade.get("miss_rate_delta_max", 0.10)
            if delta > delta_max:
                failures.append(
                    f"Post-throttle deadline misses increased by "
                    f"{delta*100:.1f}pp > {delta_max*100:.0f}pp")

    # --- Thermal sanity ---------------------------------------------------
    status_peak = thermal_metrics.get("thermal_status_peak", 0) if isinstance(thermal_metrics, dict) else 0
    if status_peak >= 3:
        failures.append(f"Phone thermal status reached SEVERE ({status_peak}) "
                        f"— system-level shutdown risk")
    elif status_peak >= 2:
        warnings.append(f"Phone thermal status reached MODERATE ({status_peak})")

    passed = len(failures) == 0
    return {
        "passed": passed,
        "failures": failures,
        "warnings": warnings,
    }


# ---------------------------------------------------------------------------
# Report + Plot
# ---------------------------------------------------------------------------

def _ms(x):
    try:
        if math.isnan(x):
            return "  nan   "
    except (TypeError, ValueError):
        return str(x)
    return f"{x/1000:7.2f}ms"


def _us(x):
    try:
        if math.isnan(x):
            return "  nan  "
    except (TypeError, ValueError):
        return str(x)
    return f"{x:7.0f}μs"


def format_report(metrics: dict) -> str:
    lines = []
    lines.append("═" * 72)
    lines.append("  THERMAL STRESS TEST — REPORT")
    lines.append("═" * 72)

    mpc = metrics.get("mpc", {})
    deadline_us = metrics.get("deadline_us", 40000)
    if "error" in mpc:
        lines.append(f"  MPC: {mpc['error']}")
    else:
        lines.append(f"  Samples: {mpc.get('samples', 0):>6}  "
                     f"Duration: {mpc.get('duration_s', 0):>6.1f}s  "
                     f"Rate: {mpc.get('sample_rate_hz', 0):>5.1f}Hz  "
                     f"Launched: {mpc.get('launched_samples', 0)} rows")
        m = mpc.get("mpc_solve_us", {})
        lines.append(f"\n  MPC solve time:")
        lines.append(f"    p50={_us(m.get('p50'))}  p95={_us(m.get('p95'))}  "
                     f"p99={_us(m.get('p99'))}  p99.9={_us(m.get('p99_9'))}")
        lines.append(f"    min={_us(m.get('min'))}  max={_us(m.get('max'))}  "
                     f"mean={_us(m.get('mean'))}")

        ec = mpc.get("eff_cycle_us", {})
        lines.append(f"\n  Cycle time (max of cycle_us, dt_max):")
        lines.append(f"    p50={_us(ec.get('p50'))}  p95={_us(ec.get('p95'))}  "
                     f"p99={_us(ec.get('p99'))}  p99.9={_us(ec.get('p99_9'))}")
        lines.append(f"    max={_us(ec.get('max'))}")

        dm = mpc.get("deadline_miss", {})
        miss_pct = dm.get("miss_rate", 0) * 100
        warn_pct = dm.get("warn_rate", 0) * 100
        lines.append(f"\n  Deadline ({deadline_us}μs = {deadline_us/1000:.0f}ms):")
        lines.append(f"    miss: {dm.get('miss_count', 0)}/{dm.get('total_cycles', 0)} "
                     f"({miss_pct:.3f}%)   warn: {dm.get('warn_count', 0)} "
                     f"({warn_pct:.3f}%)")

        se = mpc.get("solver_errors", 0)
        lines.append(f"  Solver errors: {se}")

    # Thermal --------------------------------------------------------------
    th = metrics.get("thermal", {})
    if "error" in th:
        lines.append(f"\n  Thermal: {th['error']}")
    else:
        lines.append(f"\n  Thermal ({th.get('samples', 0)} samples, "
                     f"{th.get('duration_s', 0):.0f}s):")

        def _tline(name: str, d: dict) -> str:
            return (f"    {name:14s} min={d.get('min', 0):5.1f} "
                    f"mean={d.get('mean', 0):5.1f} "
                    f"max={d.get('max', 0):5.1f} "
                    f"final={d.get('final', 0):5.1f} °C")

        for n in ("cpu_silver_C", "cpu_gold_C", "gpu_C", "ddr_C",
                  "battery_C", "skin_C"):
            if n in th:
                lines.append(_tline(n, th[n]))

        lines.append(f"\n  Thermal status peak:   {th.get('thermal_status_peak', 0)} "
                     f"(NONE=0, LIGHT=1, MODERATE=2, SEVERE=3)")
        lines.append(f"  Samples w/ status>0:   {th.get('thermal_status_samples_gt0', 0)}")
        tg = th.get("throttle_ratio_gold", {})
        ts = th.get("throttle_ratio_silver", {})
        lines.append(f"  Throttle ratio gold:   min={tg.get('min', 1):.3f} "
                     f"mean={tg.get('mean', 1):.3f}")
        lines.append(f"  Throttle ratio silver: min={ts.get('min', 1):.3f} "
                     f"mean={ts.get('mean', 1):.3f}")
        lines.append(f"  Throttle windows:      {th.get('throttle_windows_count', 0)} "
                     f"(total {th.get('total_throttle_s', 0):.1f}s)")

    # Split by throttle ----------------------------------------------------
    sp = metrics.get("split_by_throttle", {})
    if "pre_throttle" in sp and "post_throttle" in sp:
        lines.append(f"\n  Pre-throttle vs post-throttle (split at "
                     f"{sp.get('first_throttle_at_s')}s):")
        pre = sp["pre_throttle"]
        post = sp["post_throttle"]
        lines.append(f"    n:                pre={pre.get('n', 0):>6} "
                     f"post={post.get('n', 0):>6}")
        lines.append(f"    mpc_p50_us:       pre={pre.get('mpc_p50_us', 0):>6.0f} "
                     f"post={post.get('mpc_p50_us', 0):>6.0f}")
        lines.append(f"    mpc_p99_us:       pre={pre.get('mpc_p99_us', 0):>6.0f} "
                     f"post={post.get('mpc_p99_us', 0):>6.0f}")
        lines.append(f"    deadline_miss_pct:pre={pre.get('deadline_miss_rate', 0)*100:>6.3f} "
                     f"post={post.get('deadline_miss_rate', 0)*100:>6.3f}")

    # Verdict --------------------------------------------------------------
    pf = metrics.get("pass_fail", {})
    lines.append("")
    lines.append("─" * 72)
    if pf.get("passed"):
        lines.append("  VERDICT: ✓ PASS")
    else:
        lines.append("  VERDICT: ✗ FAIL")
    lines.append("─" * 72)
    for f in pf.get("failures", []):
        lines.append(f"  [FAIL]  {f}")
    for w in pf.get("warnings", []):
        lines.append(f"  [WARN]  {w}")
    if not pf.get("failures") and not pf.get("warnings"):
        lines.append("  No issues detected.")
    lines.append("═" * 72)
    return "\n".join(lines)


def maybe_plot(result_dir: Path, metrics: dict):
    """Generate HTML dashboard (if plotly is available)."""
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError:
        logger.info("plotly not installed — skipping HTML plot "
                    "(pip install plotly)")
        return

    result_dir = Path(result_dir)
    th_hdr, th_data = _load_csv(result_dir / "thermal_log.csv")
    mpc_hdr, mpc_data = _load_csv(result_dir / "mpc_timing.csv")

    th_t = _col(th_hdr, "t_wall_s", th_data)
    mpc_t = _col(mpc_hdr, "t_wall_s", mpc_data)

    have_thermal = th_t is not None and len(th_t) > 0
    have_mpc = mpc_t is not None and len(mpc_t) > 0
    if not have_thermal and not have_mpc:
        logger.warning("no data for plot (both csvs empty)")
        return

    # Align times to first available sample
    t0 = float(th_t[0]) if have_thermal else float(mpc_t[0])
    th_tr = (th_t - t0) if have_thermal else None
    mpc_tr = (mpc_t - t0) if have_mpc else None

    fig = make_subplots(rows=3, cols=1, shared_xaxes=True,
                         subplot_titles=("Temperatures",
                                         "CPU Frequency Cap (throttle ratio)",
                                         "MPC Timing"),
                         vertical_spacing=0.06)

    # --- Temperatures ----------------------------------------------------
    def _add_trace(col_name, label, color=None):
        if not have_thermal:
            return
        c = _col(th_hdr, col_name, th_data)
        if c is None:
            return
        fig.add_trace(go.Scatter(x=th_tr, y=c, name=label,
                                  line=dict(width=1.5, color=color),
                                  mode="lines"),
                      row=1, col=1)

    _add_trace("cpu_silver_max_C", "CPU silver (LITTLE)", "#1f77b4")
    _add_trace("cpu_gold_max_C", "CPU gold (big)", "#d62728")
    _add_trace("gpu_max_C", "GPU", "#2ca02c")
    _add_trace("ddr_max_C", "DDR", "#9467bd")
    _add_trace("battery_C", "Battery", "#ff7f0e")
    _add_trace("hal_skin_C", "Skin (HAL)", "#8c564b")

    if have_thermal:
        # Hot threshold reference line
        fig.add_hline(y=65, line=dict(color="red", dash="dash", width=1),
                      row=1, col=1, annotation_text="65°C")
    fig.update_yaxes(title_text="°C", row=1, col=1)

    # --- Throttle ratio --------------------------------------------------
    if have_thermal:
        gold = _col(th_hdr, "throttle_ratio_gold", th_data)
        silver = _col(th_hdr, "throttle_ratio_silver", th_data)
        status = _col(th_hdr, "thermal_status", th_data)
        if gold is not None:
            fig.add_trace(go.Scatter(x=th_tr, y=gold, name="Throttle gold",
                                      line=dict(width=1.5, color="#d62728")),
                          row=2, col=1)
        if silver is not None:
            fig.add_trace(go.Scatter(x=th_tr, y=silver, name="Throttle silver",
                                      line=dict(width=1.5, color="#1f77b4")),
                          row=2, col=1)
        if status is not None:
            # Show status as a step function scaled to [0,1] for overlay
            fig.add_trace(go.Scatter(x=th_tr, y=status/6.0,
                                      name="Thermal status (÷6)",
                                      line=dict(width=1, color="#bcbd22", dash="dot")),
                          row=2, col=1)
        fig.update_yaxes(title_text="ratio / status", row=2, col=1,
                         range=[0, 1.05])

    # --- MPC timing ------------------------------------------------------
    if have_mpc:
        mpc_us = _col(mpc_hdr, "mpc_solve_us", mpc_data)
        cycle_us = _col(mpc_hdr, "cycle_us", mpc_data)
        dt_max_s = _col(mpc_hdr, "dt_max_s", mpc_data)

        if mpc_us is not None:
            fig.add_trace(go.Scatter(x=mpc_tr, y=mpc_us / 1000.0,
                                      name="mpc_solve_ms",
                                      line=dict(width=1, color="#2ca02c")),
                          row=3, col=1)
        if cycle_us is not None:
            fig.add_trace(go.Scatter(x=mpc_tr, y=cycle_us / 1000.0,
                                      name="cycle_ms",
                                      line=dict(width=1, color="#ff7f0e")),
                          row=3, col=1)
        if dt_max_s is not None:
            fig.add_trace(go.Scatter(x=mpc_tr, y=dt_max_s * 1000.0,
                                      name="dt_max_ms",
                                      line=dict(width=1, color="#d62728", dash="dash")),
                          row=3, col=1)

        deadline_us = metrics.get("deadline_us", 40000)
        fig.add_hline(y=deadline_us / 1000.0,
                      line=dict(color="red", dash="dash", width=1.5),
                      row=3, col=1,
                      annotation_text=f"deadline {deadline_us/1000:.0f}ms")

    fig.update_yaxes(title_text="ms", row=3, col=1)
    fig.update_xaxes(title_text="time (s)", row=3, col=1)

    # Mark throttle windows on all subplots as vrects
    for w in metrics.get("throttle_windows", []):
        fig.add_vrect(x0=w["t_start"], x1=w["t_end"],
                       fillcolor="red", opacity=0.08, line_width=0,
                       row="all", col=1)

    pf = metrics.get("pass_fail", {})
    verdict = "✓ PASS" if pf.get("passed") else "✗ FAIL"
    duration = metrics.get("mpc", {}).get("duration_s", 0)
    fig.update_layout(
        title=f"Thermal Stress Test — {verdict} — {duration:.0f}s",
        height=900,
        hovermode="x unified",
        showlegend=True,
    )

    out = result_dir / "thermal_stress_plot.html"
    fig_html = fig.to_html(full_html=False, include_plotlyjs="cdn")
    table_html = _metrics_table_html(metrics)
    page = (
        '<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8">'
        f'<title>Thermal Stress — {result_dir.name}</title>'
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
        '.pass{color:#0a8a0a}.fail{color:#c00}.warn{color:#b86b00}'
        '</style></head><body>'
        f'<h1>Thermal Stress — {result_dir.name}</h1>'
        f'{table_html}<div class="card">{fig_html}</div></body></html>'
    )
    out.write_text(page, encoding="utf-8")
    logger.info(f"plot saved → {out}")


def _ts_fmt(v):
    if v is None: return "—"
    if isinstance(v, bool): return "true" if v else "false"
    if isinstance(v, int): return f"{v:,}"
    if isinstance(v, float):
        try:
            import math
            if not math.isfinite(v): return "NaN"
        except Exception:
            pass
        av = abs(v)
        if av == 0: return "0"
        if av >= 1e6 or av < 1e-3: return f"{v:.3e}"
        if av >= 100: return f"{v:.1f}"
        if av >= 1: return f"{v:.3f}"
        return f"{v:.4f}"
    return str(v)


def _metrics_table_html(metrics: dict) -> str:
    """Flatten thermal stress metrics into a single categorized HTML table."""
    rows = []
    def cat(label):
        rows.append((label, None, None, None))
    def add(k, v, unit=""):
        rows.append((k, _ts_fmt(v), unit, None))

    cat("Run Info")
    for k in ("samples", "duration_s", "verdict"):
        if k in metrics:
            unit = "s" if k.endswith("_s") else ""
            v = metrics[k]
            cls = None
            if k == "verdict":
                vs = str(v).upper()
                cls = "pass" if vs in ("PASS","GO") else ("fail" if vs in ("FAIL","NO-GO") else "warn")
            rows.append((k, _ts_fmt(v) if k != "verdict" else str(v), unit, cls))

    # Thermal stats (cpu_*_C, etc.)
    cat("Thermal (°C)")
    for k, v in metrics.items():
        if isinstance(v, dict) and (k.endswith("_C") or "temp" in k.lower()):
            for sub in ("min","mean","p50","p95","max","initial","final"):
                if sub in v:
                    add(f"{k}.{sub}", v[sub], "°C")

    # MPC timing
    mpc = metrics.get("mpc", {}) or metrics.get("mpc_metrics", {})
    if isinstance(mpc, dict) and mpc:
        cat("MPC Timing")
        for k, v in mpc.items():
            if isinstance(v, (int, float)):
                u = "µs" if "us" in k.lower() else ("" if any(s in k.lower() for s in ("count","misses","violations")) else "")
                add(f"mpc.{k}", v, u)
            elif isinstance(v, dict):
                for sub, sv in v.items():
                    if isinstance(sv, (int, float)):
                        add(f"mpc.{k}.{sub}", sv, "µs" if "us" in sub else "")

    # Throttle / split metrics
    sp = metrics.get("split", {}) or metrics.get("split_metrics", {})
    if isinstance(sp, dict) and sp:
        cat("Pre/Post Throttle Split")
        for k, v in sp.items():
            if isinstance(v, (int, float)):
                add(f"split.{k}", v)
            elif isinstance(v, dict):
                for sub, sv in v.items():
                    if isinstance(sv, (int, float)):
                        add(f"split.{k}.{sub}", sv)

    # All other top-level scalar metrics not yet shown
    shown = {"samples","duration_s","verdict","mpc","mpc_metrics","split","split_metrics"}
    other_scalars = [(k, v) for k, v in metrics.items()
                     if k not in shown and not isinstance(v, (dict, list, tuple))]
    if other_scalars:
        cat("Other")
        for k, v in other_scalars:
            add(k, v)

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
