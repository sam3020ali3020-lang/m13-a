#!/usr/bin/env python3
"""
load_driver.py — Load generation for thermal stress tests
==========================================================

Three modes:

* ``passive``      — do nothing.  The user is expected to keep MPC busy
                     externally (e.g. running `/hil` or `/pil` in another
                     terminal).  We simply monitor.

* ``cpu-stress``   — spawn N background workers on the phone via adb that
                     pin all big cores at 100 %.  Used to simulate a hot
                     desert launch pad.  Does NOT drive MPC; combine with
                     an externally-running HIL/PIL session to stress both
                     CPU heat and MPC load.

* ``pil-loop``     — invoke `pil_runner.py` in a background thread, and
                     re-run it whenever it finishes.  This keeps MPC active
                     continuously for the full test duration.  Requires a
                     working PIL setup (PX4 HITL on phone, `adb reverse`
                     tunnels).

Each driver exposes ``start()``/``stop()`` methods.
"""

from __future__ import annotations

import logging
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import List, Optional

logger = logging.getLogger("load_driver")


# ---------------------------------------------------------------------------
# Base class
# ---------------------------------------------------------------------------

class LoadDriver:
    """Abstract load driver."""
    mode = "base"

    def start(self):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    def is_running(self) -> bool:
        return False


# ---------------------------------------------------------------------------
# Passive (monitor-only)
# ---------------------------------------------------------------------------

class PassiveDriver(LoadDriver):
    """No-op.  The caller is responsible for keeping MPC busy."""
    mode = "passive"

    def start(self):
        logger.info("LoadDriver: passive (no load generated — monitor only)")

    def stop(self):
        pass


# ---------------------------------------------------------------------------
# CPU stress (phone-side workers)
# ---------------------------------------------------------------------------

class CpuStressDriver(LoadDriver):
    """Spawn N workers on the phone to heat the SoC.

    Writes a small shell script to /data/local/tmp and launches N background
    processes.  On stop() we kill them by pattern match.

    Two worker types:
      * md5 mode (heavy):  `md5sum /dev/zero`  — uses crypto ext + integer
        ALU hard, heats big cores well.
      * yes mode  (light): `yes > /dev/null`   — pure kernel-side loop,
        lighter heat, often gets throttled quickly.
    """
    mode = "cpu-stress"

    def __init__(self, device: str = "", workers: int = 8, use_md5: bool = True):
        self.device = device
        self.workers = workers
        self.use_md5 = use_md5
        self._started = False

    def _adb(self, *args: str, timeout: float = 5.0) -> subprocess.CompletedProcess:
        cmd = ["adb"]
        if self.device:
            cmd += ["-s", self.device]
        cmd.extend(args)
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)

    def start(self):
        if self._started:
            return
        worker_cmd = "md5sum /dev/zero" if self.use_md5 else "yes"
        # Spawn N detached workers in a single adb shell call.  Each worker
        # has its stdin closed (`</dev/null`) and stdout/stderr redirected
        # so adb shell does NOT linger waiting for FDs.
        # We use `setsid` if available to fully detach; fallback to plain `&`.
        line = (
            f"for i in $(seq 1 {self.workers}); do "
            f"  {worker_cmd} </dev/null >/dev/null 2>&1 & "
            f"done; "
            f"echo started"
        )
        try:
            r = self._adb("shell", line, timeout=4.0)
            if r.returncode != 0:
                logger.error(f"cpu-stress: adb rc={r.returncode}: {r.stderr.strip()}")
                return
        except Exception as exc:
            logger.error(f"cpu-stress: failed to start workers: {exc}")
            return

        self._started = True
        logger.info(f"LoadDriver: cpu-stress started — {self.workers} × "
                    f"'{worker_cmd}' on phone")

    def stop(self):
        if not self._started:
            return
        # Kill by command pattern.  Both yes and md5sum should be covered.
        # Use `pkill -f` for broad pattern match.
        kill_cmd = (
            "pkill -9 md5sum 2>/dev/null; "
            "pkill -9 yes 2>/dev/null; "
            "rm -f /data/local/tmp/thermal_stress.sh 2>/dev/null; "
            "exit 0"
        )
        try:
            self._adb("shell", kill_cmd, timeout=5.0)
        except Exception as exc:
            logger.warning(f"cpu-stress: stop failed: {exc}")
        self._started = False
        logger.info("LoadDriver: cpu-stress stopped")

    def is_running(self) -> bool:
        return self._started


# ---------------------------------------------------------------------------
# PIL loop (chain PIL flights)
# ---------------------------------------------------------------------------

class PilLoopDriver(LoadDriver):
    """Run pil_runner.py back-to-back in a background thread.

    Each PIL flight is ~500 s of simulated rocket flight with MPC running
    continuously on the phone at 25 Hz.  Chaining 3-4 flights covers a
    30-minute thermal test, keeping MPC under realistic load the whole time.

    We redirect PIL stdout/stderr to a per-run log file so that the thermal
    test's stdout stays clean.  The PIL analysis HTML is auto-generated per
    flight and saved under pil/results/.

    NOTE: This driver assumes PX4 is already running on the phone and the
    adb tunnels (`adb reverse tcp:4560 tcp:4560`, `adb forward tcp:5760
    tcp:5760`) are in place.  It does NOT manage the PX4 app lifecycle.
    """
    mode = "pil-loop"

    def __init__(self,
                 pil_runner_path: Path,
                 pil_config_path: Optional[Path] = None,
                 inter_run_sleep_s: float = 3.0,
                 log_dir: Optional[Path] = None):
        self.pil_runner_path = pil_runner_path
        self.pil_config_path = pil_config_path
        self.inter_run_sleep_s = inter_run_sleep_s
        self.log_dir = log_dir

        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._proc: Optional[subprocess.Popen] = None
        self._run_count = 0

    def start(self):
        if self._thread is not None:
            return
        if not self.pil_runner_path.exists():
            logger.error(f"pil-loop: pil_runner not found at {self.pil_runner_path}")
            return
        self._thread = threading.Thread(target=self._loop, name="pil_loop",
                                         daemon=True)
        self._thread.start()
        logger.info(f"LoadDriver: pil-loop started → {self.pil_runner_path}")

    def stop(self):
        self._stop.set()
        proc = self._proc
        if proc is not None and proc.poll() is None:
            try:
                # Graceful SIGINT first (PIL bridge handles it)
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=2.0)
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None
        logger.info(f"LoadDriver: pil-loop stopped (ran {self._run_count} flights)")

    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def _loop(self):
        while not self._stop.is_set():
            self._run_count += 1
            log_file = None
            if self.log_dir is not None:
                self.log_dir.mkdir(parents=True, exist_ok=True)
                log_path = self.log_dir / f"pil_flight_{self._run_count:03d}.log"
                log_file = open(log_path, "w")

            cmd = [sys.executable, str(self.pil_runner_path)]
            if self.pil_config_path is not None:
                cmd += ["--config", str(self.pil_config_path)]

            logger.info(f"pil-loop: starting flight #{self._run_count}")
            try:
                # Disable browser autoopen by setting env var (pil_analysis respects BROWSER=none?)
                env = dict(os.environ)
                env.setdefault("PIL_NO_BROWSER", "1")
                self._proc = subprocess.Popen(
                    cmd,
                    stdout=log_file or subprocess.DEVNULL,
                    stderr=subprocess.STDOUT,
                    env=env,
                    cwd=str(self.pil_runner_path.parent),
                )
                # Wait for completion, but poll _stop periodically
                while not self._stop.is_set():
                    try:
                        rc = self._proc.wait(timeout=1.0)
                        break
                    except subprocess.TimeoutExpired:
                        continue
                else:
                    rc = None

                if rc is not None:
                    logger.info(f"pil-loop: flight #{self._run_count} "
                                f"finished rc={rc}")
            except Exception as exc:
                logger.warning(f"pil-loop: flight #{self._run_count} failed: {exc}")
            finally:
                self._proc = None
                if log_file:
                    log_file.close()

            if self._stop.is_set():
                break

            # Brief pause between flights (lets TCP/4560 close cleanly)
            slept = 0.0
            while slept < self.inter_run_sleep_s and not self._stop.is_set():
                time.sleep(0.5)
                slept += 0.5


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------

def make_driver(mode: str, cfg: dict, script_dir: Path,
                log_dir: Optional[Path] = None) -> LoadDriver:
    """Construct the appropriate LoadDriver from config."""
    mode = (mode or "passive").lower()
    load_cfg = cfg.get("load", {})

    if mode == "passive":
        return PassiveDriver()

    if mode == "cpu-stress":
        return CpuStressDriver(
            device=cfg.get("thermal", {}).get("adb_device", ""),
            workers=int(load_cfg.get("stress_workers", 8)),
            use_md5=bool(load_cfg.get("stress_use_md5", True)),
        )

    if mode == "pil-loop":
        pil_path = load_cfg.get("pil_runner", "../pil/pil_runner.py")
        pil_cfg = load_cfg.get("pil_config", "../pil/pil_config.yaml")
        pil_runner_abs = (script_dir / pil_path).resolve()
        pil_config_abs = (script_dir / pil_cfg).resolve() if pil_cfg else None
        return PilLoopDriver(
            pil_runner_path=pil_runner_abs,
            pil_config_path=pil_config_abs,
            inter_run_sleep_s=float(load_cfg.get("pil_inter_run_sleep_s", 3.0)),
            log_dir=log_dir,
        )

    raise ValueError(f"Unknown load mode: {mode!r}")


# ---------------------------------------------------------------------------
# Heat-up helper (not a driver — used once at startup to preheat)
# ---------------------------------------------------------------------------

def preheat(device: str, target_C: float, max_seconds: float,
            workers: int = 8, poll_fn=None) -> float:
    """Run a CPU-stress burst until target temp is reached or timeout.

    ``poll_fn`` is an optional callable returning the current hottest CPU
    temperature in °C (for progress display).  If None, we query adb
    ourselves at 2 Hz.

    Returns the final max-CPU temperature measured.
    """
    driver = CpuStressDriver(device=device, workers=workers, use_md5=True)
    driver.start()

    def _quick_temp() -> float:
        try:
            p = subprocess.run(
                (["adb"] + (["-s", device] if device else [])
                 + ["shell",
                    'grep -H "" /sys/class/thermal/thermal_zone*/temp 2>/dev/null '
                    '| cut -d: -f2 | sort -rn | head -1']),
                capture_output=True, text=True, timeout=3.0)
            v = p.stdout.strip()
            if v:
                return int(v) / 1000.0
        except Exception:
            pass
        return float("nan")

    t0 = time.monotonic()
    last_T = float("nan")
    try:
        while time.monotonic() - t0 < max_seconds:
            T = poll_fn() if poll_fn else _quick_temp()
            last_T = T
            if T == T and T >= target_C:  # (T not NaN) and (T >= target)
                elapsed = time.monotonic() - t0
                logger.info(f"preheat: reached {T:.1f}°C in {elapsed:.0f}s")
                return T
            sys.stdout.write(f"\r  [preheat] {T:.1f}°C / target {target_C:.1f}°C "
                             f"({time.monotonic()-t0:.0f}s/{max_seconds:.0f}s)     ")
            sys.stdout.flush()
            time.sleep(2.0)
    finally:
        driver.stop()
        sys.stdout.write("\n")
        sys.stdout.flush()

    logger.warning(f"preheat: timeout — stopped at {last_T:.1f}°C")
    return last_T


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s: %(message)s")
    import argparse
    ap = argparse.ArgumentParser(description="Standalone load driver test")
    ap.add_argument("--mode", choices=["passive", "cpu-stress"], default="cpu-stress")
    ap.add_argument("--duration", type=float, default=10.0)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--use-md5", action="store_true", default=True)
    args = ap.parse_args()

    if args.mode == "passive":
        d = PassiveDriver()
    else:
        d = CpuStressDriver(workers=args.workers, use_md5=args.use_md5)

    d.start()
    try:
        for i in range(int(args.duration)):
            time.sleep(1.0)
            print(f"  {i+1}s...", end="\r")
    except KeyboardInterrupt:
        pass
    finally:
        d.stop()
