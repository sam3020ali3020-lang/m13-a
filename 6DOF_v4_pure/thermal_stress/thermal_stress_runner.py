#!/usr/bin/env python3
"""
thermal_stress_runner.py — Long-duration MPC thermal stress test
==================================================================

Runs three parallel activities for the configured duration:

1. **Thermal poller** — reads /sys/class/thermal + dumpsys battery +
   thermalservice every 1 s via adb.  Logs to ``thermal_log.csv``.

2. **MAVLink reader** — captures ``DEBUG_FLOAT_ARRAY`` (RktGNC) from PX4
   on the phone at ~25 Hz (one sample per MPC cycle).  Records
   ``mpc_solve_us``, ``cycle_us``, ``dt_actual/min/max`` to
   ``mpc_timing.csv``.

3. **Load driver** — optional; keeps MPC busy and/or heats the SoC.
   See ``load_driver.py`` for modes.

At the end, automatically invokes ``thermal_stress_analysis.py`` to compute
percentiles, deadline-miss rate, throttling windows, and PASS/FAIL verdict,
and to generate an interactive HTML plot.

Usage:
    python3 thermal_stress_runner.py                         # 30 min passive
    python3 thermal_stress_runner.py --preset quick          # 5 min smoke test
    python3 thermal_stress_runner.py --preset desert         # 30 min + preheat + stress
    python3 thermal_stress_runner.py --duration 600 --mode cpu-stress
    python3 thermal_stress_runner.py --analyze-only results/20260503_180000/
"""

from __future__ import annotations

import argparse
import csv
import json
import logging
import signal
import sys
import threading
import time
from datetime import datetime, timedelta
from pathlib import Path
from typing import Optional

import yaml

_SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_SCRIPT_DIR))
# For e2e_reader (reused for MAVLink capture)
sys.path.insert(0, str(_SCRIPT_DIR.parent / "e2e_latency"))

from thermal_poller import ThermalPoller, DEFAULT_ZONE_GROUPS  # noqa: E402
import load_driver                                               # noqa: E402
import thermal_stress_analysis                                   # noqa: E402

logger = logging.getLogger("thermal_stress")


# ---------------------------------------------------------------------------
# MAVLink reader thread
# ---------------------------------------------------------------------------

class MpcReaderThread(threading.Thread):
    """Runs an E2EReader in a loop and flushes RktGNC + timing to CSV.

    We don't buffer indefinitely: the reader's internal lists grow, but we
    snapshot them to CSV every few seconds so that a crash mid-run doesn't
    lose data.
    """

    def __init__(self, host: str, port: int, timeout_s: float,
                 streams: dict, csv_path: Path,
                 flush_interval_s: float = 5.0):
        super().__init__(name="mpc_reader", daemon=True)
        self.host = host
        self.port = port
        self.timeout_s = timeout_s
        self.streams = streams
        self.csv_path = csv_path
        self.flush_interval_s = flush_interval_s
        # Renamed from `_stop` to avoid clashing with Thread._stop()
        # internal slot which Python's join() may call.
        self._halt = threading.Event()
        self._last_mpc_solve_us: float = float("nan")
        self._last_cycle_us: float = float("nan")
        self._last_dt_max: float = float("nan")
        self._reader = None
        self._rktgnc_written = 0
        self._solver_errors = 0
        self._connected = False
        self._lock = threading.Lock()
        self._file = None
        self._writer: Optional[csv.writer] = None

    def stop(self, timeout: float = 3.0):
        self._halt.set()
        self.join(timeout=timeout)

    def status(self) -> dict:
        with self._lock:
            return {
                "connected": self._connected,
                "mpc_solve_us": self._last_mpc_solve_us,
                "cycle_us": self._last_cycle_us,
                "dt_max_ms": self._last_dt_max * 1e3 if self._last_dt_max == self._last_dt_max else float("nan"),
                "rktgnc_rows": self._rktgnc_written,
                "solver_errors": self._solver_errors,
            }

    # ------------------------------------------------------------------
    # Thread body
    # ------------------------------------------------------------------

    def run(self):
        from e2e_reader import E2EReader  # lazy import

        # Silence the reader's verbose per-chunk logging (would spam the
        # thermal stress output with 360+ "Recording ..." lines per 30 min).
        logging.getLogger("sensor_reader").setLevel(logging.WARNING)
        logging.getLogger("e2e_reader").setLevel(logging.WARNING)

        # Open CSV up-front so that even if MAVLink connect fails we have
        # a header row.
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._file = open(self.csv_path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow([
            "t_wall_s", "time_usec", "stage", "t_flight",
            "launched", "dt_actual_s", "dt_min_s", "dt_max_s",
            "mhe_solve_us", "mpc_solve_us", "cycle_us",
            "mpc_solve_count", "mpc_solver_status", "mhe_valid",
        ])
        self._file.flush()

        reader = E2EReader(host=self.host, port=self.port,
                           timeout_s=self.timeout_s)
        self._reader = reader

        # Connect — retry with backoff
        backoff = 1.0
        while not self._halt.is_set():
            try:
                if reader.connect():
                    break
            except Exception as exc:
                logger.debug(f"mpc_reader: connect exception: {exc}")
            logger.warning(f"mpc_reader: connect failed, retry in {backoff}s...")
            if self._halt.wait(backoff):
                return
            backoff = min(backoff * 2, 10.0)

        with self._lock:
            self._connected = True
        logger.info(f"mpc_reader: connected to {self.host}:{self.port}")

        # Request streams
        try:
            reader.request_streams(self.streams)
        except Exception as exc:
            logger.warning(f"mpc_reader: request_streams failed: {exc}")

        # Main capture loop — each iteration reads for `flush_interval_s`
        # seconds, then drains new samples to CSV.
        last_rktgnc_len = 0
        try:
            while not self._halt.is_set():
                # Record a short burst (non-blocking pattern: use the
                # existing `record` helper with a short duration).
                reader.record(duration_s=self.flush_interval_s,
                              progress_interval_s=self.flush_interval_s + 1,
                              heartbeat_interval_s=1.0)

                # Flush new RktGNC samples
                samples = reader.rktgnc_samples
                new_samples = samples[last_rktgnc_len:]
                last_rktgnc_len = len(samples)

                for s in new_samples:
                    self._writer.writerow([
                        f"{s.t_wall_s:.6f}", s.time_usec, s.stage,
                        f"{s.t_flight:.4f}", int(s.launched),
                        f"{s.dt_actual:.6f}", f"{s.dt_min:.6f}", f"{s.dt_max:.6f}",
                        f"{s.mhe_solve_us:.1f}", f"{s.mpc_solve_us:.1f}",
                        f"{s.cycle_us:.1f}",
                        s.mpc_solve_count, s.mpc_solver_status, int(s.mhe_valid),
                    ])

                    if s.mpc_solver_status != 0:
                        self._solver_errors += 1
                self._file.flush()

                if new_samples:
                    last = new_samples[-1]
                    with self._lock:
                        self._last_mpc_solve_us = last.mpc_solve_us
                        self._last_cycle_us = last.cycle_us
                        self._last_dt_max = last.dt_max
                        self._rktgnc_written = last_rktgnc_len
        finally:
            try:
                reader.disconnect()
            except Exception:
                pass
            if self._file:
                self._file.close()


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

def setup_logging(verbose: bool = False):
    logging.basicConfig(
        level=logging.DEBUG if verbose else logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )


def load_config(path: Path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def resolve_preset(cfg: dict, preset: Optional[str]) -> dict:
    """Apply a preset from cfg['presets'] by mutating a copy."""
    if not preset:
        return cfg
    presets = cfg.get("presets", {})
    p = presets.get(preset)
    if not p:
        logger.warning(f"Preset '{preset}' not found; using defaults")
        return cfg
    out = dict(cfg)
    if "duration_s" in p:
        out["duration_s"] = p["duration_s"]
    if "load_mode" in p:
        load = dict(out.get("load", {}))
        load["mode"] = p["load_mode"]
        out["load"] = load
    if "heat_first_s" in p:
        load = dict(out.get("load", {}))
        load["heat_first_s"] = p["heat_first_s"]
        out["load"] = load
    if "heat_target_C" in p:
        load = dict(out.get("load", {}))
        load["heat_target_C"] = p["heat_target_C"]
        out["load"] = load
    return out


def build_stream_map(cfg: dict) -> dict:
    """Convert the mavlink_streams section into {msg_id: rate_hz}."""
    streams_cfg = cfg.get("mavlink_streams", {})
    streams = {}
    for val in streams_cfg.values():
        msg_id = val.get("msg_id")
        rate_hz = val.get("rate_hz", 0)
        if msg_id and rate_hz > 0:
            streams[msg_id] = rate_hz
    return streams


def progress_printer(poller: ThermalPoller, reader: MpcReaderThread,
                     start_t: float, end_t: float,
                     live_interval_s: float,
                     stop_evt: threading.Event):
    """Background thread: every `live_interval_s` print a one-line status."""
    while not stop_evt.is_set():
        now = time.monotonic()
        if now >= end_t:
            break
        elapsed = now - start_t
        remaining = max(0.0, end_t - now)

        t = poller.latest()
        r = reader.status()

        if t is not None:
            cpu_silver = t.group_max_C.get("cpu_silver", float("nan"))
            cpu_gold = t.group_max_C.get("cpu_gold", float("nan"))
            batt = t.battery_C
            throt = t.throttle_ratio_gold
            status = t.thermal_status
            ts_str = (f"CPU_S={cpu_silver:4.1f}°C CPU_G={cpu_gold:4.1f}°C "
                      f"batt={batt:4.1f}°C throt={throt:.2f} st={status}")
        else:
            ts_str = "thermal=pending"

        mpc_us = r.get("mpc_solve_us", float("nan"))
        dtmax_ms = r.get("dt_max_ms", float("nan"))
        mpc_str = (f"mpc={mpc_us:5.0f}μs dt_max={dtmax_ms:5.1f}ms "
                   f"rows={r.get('rktgnc_rows', 0)} "
                   f"errs={r.get('solver_errors', 0)}")

        conn = "✓" if r.get("connected") else "✗"

        sys.stdout.write(
            f"  [{timedelta(seconds=int(elapsed))}/"
            f"{timedelta(seconds=int(end_t-start_t))}] "
            f"{ts_str} | mav{conn} {mpc_str}\n"
        )
        sys.stdout.flush()

        stop_evt.wait(min(live_interval_s, remaining + 0.1))


def run(cfg: dict, result_dir: Path) -> dict:
    """Main orchestration."""
    # ------------------------------------------------------------------
    # Result dir + config snapshot
    # ------------------------------------------------------------------
    result_dir.mkdir(parents=True, exist_ok=True)
    with open(result_dir / "config_used.yaml", "w") as f:
        yaml.safe_dump(cfg, f, allow_unicode=True, sort_keys=False)

    # ------------------------------------------------------------------
    # 1) Optional preheat
    # ------------------------------------------------------------------
    load_cfg = cfg.get("load", {})
    thermal_cfg = cfg.get("thermal", {})
    device = thermal_cfg.get("adb_device", "") or ""

    preheat_s = float(load_cfg.get("heat_first_s", 0) or 0)
    if preheat_s > 0:
        target_C = float(load_cfg.get("heat_target_C", 65))
        workers = int(load_cfg.get("heat_workers", 8))
        print(f"\n[preheat] CPU-stressing phone for up to {preheat_s:.0f}s "
              f"until {target_C:.0f}°C ...")
        load_driver.preheat(device=device, target_C=target_C,
                            max_seconds=preheat_s, workers=workers)

    # ------------------------------------------------------------------
    # 2) Thermal poller
    # ------------------------------------------------------------------
    zone_groups = thermal_cfg.get("zone_groups", DEFAULT_ZONE_GROUPS)
    poll_interval = float(thermal_cfg.get("poll_interval_s", 1.0))
    poller = ThermalPoller(device=device,
                            poll_interval_s=poll_interval,
                            zone_groups=zone_groups)
    poller.start(result_dir / "thermal_log.csv")

    # ------------------------------------------------------------------
    # 3) MAVLink reader
    # ------------------------------------------------------------------
    conn = cfg.get("connection", {})
    streams = build_stream_map(cfg)
    reader = MpcReaderThread(
        host=conn.get("host", "127.0.0.1"),
        port=int(conn.get("port", 5760)),
        timeout_s=float(conn.get("timeout_s", 10.0)),
        streams=streams,
        csv_path=result_dir / "mpc_timing.csv",
        flush_interval_s=5.0,
    )
    reader.start()

    # ------------------------------------------------------------------
    # 4) Load driver
    # ------------------------------------------------------------------
    mode = load_cfg.get("mode", "passive")
    driver = load_driver.make_driver(mode, cfg, _SCRIPT_DIR,
                                      log_dir=result_dir / "pil_logs")
    driver.start()

    # ------------------------------------------------------------------
    # 5) Run for duration
    # ------------------------------------------------------------------
    duration_s = float(cfg.get("duration_s", 1800))
    live_s = float(cfg.get("output", {}).get("live_progress_s", 10.0))
    start_t = time.monotonic()
    end_t = start_t + duration_s

    print(f"\n{'='*72}")
    print(f"  Thermal Stress Test — {duration_s:.0f}s "
          f"({duration_s/60:.1f} min) | mode={mode}")
    print(f"  Results → {result_dir}")
    print(f"{'='*72}\n")

    stop_evt = threading.Event()
    prog_thread = threading.Thread(
        target=progress_printer,
        args=(poller, reader, start_t, end_t, live_s, stop_evt),
        daemon=True, name="progress")
    prog_thread.start()

    # SIGINT handler → early termination (still run analysis)
    interrupted = threading.Event()

    def _sigint(signum, frame):
        if interrupted.is_set():
            # Second ctrl-c = hard exit
            logger.warning("second SIGINT → exiting hard")
            sys.exit(130)
        interrupted.set()
        logger.warning("SIGINT received → stopping test (Ctrl-C again to force)")

    old_handler = signal.signal(signal.SIGINT, _sigint)

    try:
        while time.monotonic() < end_t and not interrupted.is_set():
            time.sleep(0.5)
    finally:
        stop_evt.set()
        prog_thread.join(timeout=2.0)
        # Order matters: stop load first (so MPC settles), then reader, then poller
        driver.stop()
        reader.stop(timeout=5.0)
        poller.stop(join_timeout_s=3.0)
        signal.signal(signal.SIGINT, old_handler)

    # ------------------------------------------------------------------
    # 6) Analyze
    # ------------------------------------------------------------------
    print(f"\n[analyze] computing metrics on {result_dir} ...")
    thresholds = cfg.get("thresholds", {})
    mpc_cfg = cfg.get("mpc", {})
    metrics = thermal_stress_analysis.analyze_directory(
        result_dir, thresholds=thresholds, mpc_cfg=mpc_cfg)

    return metrics


def save_outputs(result_dir: Path, metrics: dict, save_plot: bool = True):
    """Write JSON, text report, HTML plot."""
    out = result_dir / "thermal_stress.metrics.json"
    with open(out, "w") as f:
        json.dump(metrics, f, indent=2, default=str)

    # Report
    report = thermal_stress_analysis.format_report(metrics)
    with open(result_dir / "thermal_stress_report.txt", "w") as f:
        f.write(report)
    print("\n" + report)

    if save_plot:
        try:
            thermal_stress_analysis.maybe_plot(result_dir, metrics)
        except Exception as exc:
            logger.warning(f"plot generation failed: {exc}")


def main() -> int:
    ap = argparse.ArgumentParser(description="MPC thermal stress test runner")
    ap.add_argument("--config", type=Path,
                    default=_SCRIPT_DIR / "thermal_stress_config.yaml",
                    help="Config file path")
    ap.add_argument("--preset", choices=["quick", "standard", "desert", "extreme"],
                    default=None, help="Apply a preset")
    ap.add_argument("--duration", type=float, default=None,
                    help="Override duration in seconds")
    ap.add_argument("--mode", choices=["passive", "cpu-stress", "pil-loop"],
                    default=None, help="Override load mode")
    ap.add_argument("--heat-first", type=float, default=None,
                    help="Preheat for N seconds before the run")
    ap.add_argument("--results-dir", type=Path, default=None,
                    help="Result dir (default: results/<timestamp>/)")
    ap.add_argument("--analyze-only", type=Path, default=None,
                    help="Skip recording, re-analyze an existing result dir")
    ap.add_argument("--no-plot", action="store_true",
                    help="Skip HTML plot generation")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    setup_logging(args.verbose)

    if args.analyze_only:
        if not args.config.exists():
            logger.error(f"config not found: {args.config}")
            return 2
        cfg = load_config(args.config)
        metrics = thermal_stress_analysis.analyze_directory(
            args.analyze_only,
            thresholds=cfg.get("thresholds", {}),
            mpc_cfg=cfg.get("mpc", {}),
        )
        save_outputs(args.analyze_only, metrics, save_plot=not args.no_plot)
        return 0 if metrics.get("pass_fail", {}).get("passed") else 1

    if not args.config.exists():
        logger.error(f"config not found: {args.config}")
        return 2

    cfg = load_config(args.config)
    if args.preset:
        cfg = resolve_preset(cfg, args.preset)
    if args.duration is not None:
        cfg["duration_s"] = args.duration
    if args.mode is not None:
        cfg.setdefault("load", {})["mode"] = args.mode
    if args.heat_first is not None:
        cfg.setdefault("load", {})["heat_first_s"] = args.heat_first

    # Result dir
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    result_dir = args.results_dir or (_SCRIPT_DIR / "results" / ts)

    metrics = run(cfg, result_dir)
    save_outputs(result_dir, metrics,
                 save_plot=not args.no_plot and cfg.get("output", {}).get("save_plot", True))

    pf = metrics.get("pass_fail", {})
    return 0 if pf.get("passed", True) else 1


if __name__ == "__main__":
    sys.exit(main())
