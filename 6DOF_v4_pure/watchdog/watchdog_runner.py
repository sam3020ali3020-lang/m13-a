#!/usr/bin/env python3
"""
watchdog_runner.py — PX4 module liveness watchdog test runner
==============================================================

Drives the on-device `WatchdogManager` (cpp/watchdog_native.cpp + Kotlin
WatchdogManager) through crash / restart / policy scenarios and captures
the resulting JSONL event log for offline analysis.

Prerequisites
-------------
    1. Phone connected: `adb devices` shows it
    2. Debug-signed APK installed (release rejects crash injection)
    3. PX4 running on the phone (Start PX4 from the UI)
    4. (Optional) `adb forward tcp:5760 tcp:5760` if you also want to
       cross-check via MAVLink in the same session

Usage
-----
    python3 watchdog_runner.py                    # preset "quick"
    python3 watchdog_runner.py --preset standard
    python3 watchdog_runner.py --preset full
    python3 watchdog_runner.py --scenario solo_crash
    python3 watchdog_runner.py --module rocket_mpc        # single-shot
    python3 watchdog_runner.py --analyze-only results/20260503_210000/

Design
------
Commands are sent as ADB broadcast intents that the WatchdogManager
BroadcastReceiver translates into native API calls.  The native layer
writes a JSONL event log to the phone's external app-files directory;
after each scenario the runner pulls that file via `adb pull` and hands
it to `watchdog_analysis` for metrics + report generation.

The native watchdog is always on whenever PX4 is running — this runner
only *triggers* scenarios and *reads* their outcomes; it does not
control the watchdog's poll loop or lifecycle.
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import re
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

_SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_SCRIPT_DIR))

import watchdog_analysis  # noqa: E402

logger = logging.getLogger("watchdog_runner")


# ============================================================================
# ADB helpers
# ============================================================================

class AdbError(RuntimeError):
    pass


class Device:
    """Minimal adb wrapper scoped to a single connected device."""

    def __init__(self, package: str, serial: str = "", timeout_s: float = 5.0,
                 log_path: str = ""):
        self.package = package
        self.serial = serial
        self.timeout_s = timeout_s
        self.log_path = log_path
        self._adb_base = ["adb"]
        if serial:
            self._adb_base += ["-s", serial]
        self._verify_connected()

    # ------------------------------------------------------------------
    def _verify_connected(self) -> None:
        out = subprocess.run(self._adb_base + ["devices"],
                             capture_output=True, text=True, timeout=5)
        if "device\n" not in (out.stdout or "") and \
           "device\r\n" not in (out.stdout or ""):
            raise AdbError(
                "لا يوجد هاتف متصل عبر adb. جرِّب: adb devices\n"
                f"stdout={out.stdout!r} stderr={out.stderr!r}"
            )
        # Check that our package is actually running — otherwise the
        # broadcast receiver is not registered (FlightService not started).
        out = subprocess.run(
            self._adb_base + ["shell", "pidof", self.package],
            capture_output=True, text=True, timeout=5
        )
        pid = (out.stdout or "").strip()
        if not pid:
            raise AdbError(
                f"تطبيق '{self.package}' لا يعمل. اضغط 'Start PX4' على الهاتف أولاً."
            )
        logger.info("connected: %s (pid=%s)", self.package, pid)

    # ------------------------------------------------------------------
    def wait_for_watchdog(self, timeout_s: float = 20.0) -> None:
        """Block until the on-device WatchdogManager is answering broadcasts.

        The native watchdog is initialised on a background thread inside
        start_px4_modules(), so if the runner fires crashes immediately
        after `Start PX4` the first few attempts would race with init and
        return FAIL.  We poll a cheap no-op (`policy prelaunch`) every
        0.5 s until it succeeds.
        """
        deadline = time.time() + timeout_s
        last_err = ""
        while time.time() < deadline:
            try:
                code, data = self.broadcast("policy", module="prelaunch")
                if code == 0 and data == "OK":
                    logger.info("watchdog ready (policy=prelaunch applied)")
                    return
                last_err = f"code={code} data={data!r}"
            except AdbError as e:
                last_err = str(e)
            time.sleep(0.5)
        raise AdbError(
            f"Watchdog not ready after {timeout_s:.0f}s: {last_err}\n"
            "تأكد أن PX4 يعمل وأن جميع الـ modules بدأت. "
            "افتح logcat وابحث عن 'WatchdogManager initialised'."
        )

    # ------------------------------------------------------------------
    def broadcast(self, action: str, module: str = "",
                  **extras: Any) -> Tuple[int, str]:
        """Send ACTION_TEST_CMD with the given extras.  Returns (code, data).

        `code` is the ordered-broadcast result code (0 = ok, 1 = fail,
        -1 = no response).  `data` is the result string (e.g. "OK", "FAIL",
        "REJECT_RELEASE_BUILD") or "" if the receiver didn't set one.
        """
        cmd = self._adb_base + [
            "shell", "am", "broadcast",
            "-a", "com.ardophone.px4v17.WATCHDOG_TEST",
            "--es", "action", action,
        ]
        if module:
            cmd += ["--es", "module", module]
        for k, v in extras.items():
            cmd += ["--es", k, str(v)]

        try:
            proc = subprocess.run(cmd, capture_output=True, text=True,
                                  timeout=self.timeout_s)
        except subprocess.TimeoutExpired as e:
            raise AdbError(f"broadcast timeout: {e}") from e

        if proc.returncode != 0:
            raise AdbError(
                f"am broadcast returned {proc.returncode}:\n"
                f"stdout={proc.stdout!r}\nstderr={proc.stderr!r}"
            )

        # Parse "Broadcast completed: result=N, data=\"XXX\"" (the ordered
        # receiver path) or just "Broadcasting: ..." for non-ordered.
        text = proc.stdout or ""
        m = re.search(r"result=(-?\d+)", text)
        code = int(m.group(1)) if m else -1
        m = re.search(r'data="([^"]*)"', text)
        data = m.group(1) if m else ""
        logger.debug("broadcast action=%s module=%s → code=%d data=%r",
                     action, module, code, data)
        return code, data

    # ------------------------------------------------------------------
    def pull_watchdog_log(self, dest: Path) -> Optional[Path]:
        """Pull the JSONL event log off the device, return local path.

        Returns None if the file doesn't exist (watchdog not running or
        no events recorded yet).
        """
        if not self.log_path:
            return None
        dest.parent.mkdir(parents=True, exist_ok=True)
        cmd = self._adb_base + ["pull", self.log_path, str(dest)]
        proc = subprocess.run(cmd, capture_output=True, text=True,
                              timeout=10.0)
        if proc.returncode != 0:
            logger.warning("adb pull failed: %s", proc.stderr.strip())
            return None
        return dest

    # ------------------------------------------------------------------
    def clear_watchdog_log(self) -> None:
        """Truncate the event log on the device so scenarios start fresh.

        IMPORTANT: must NOT use `adb shell rm` on the log path — the native
        poll thread holds the file open, so unlinking would leave it
        writing to an orphan inode (visible in /proc/<pid>/fd as
        '... (deleted)') and subsequent `adb pull` would fail.

        Instead we broadcast the `clear_log` action and let the app call
        `wd_truncate_log()` in-process, which closes + reopens the fd
        with O_TRUNC so the path stays valid and empty.
        """
        if not self.log_path:
            return
        try:
            code, data = self.broadcast("clear_log")
            if code != 0:
                logger.warning("clear_log broadcast returned code=%d data=%r",
                               code, data)
        except AdbError as e:
            logger.warning("clear_log broadcast failed: %s", e)

    def _adb_base_run(self, tail: List[str]) -> None:
        subprocess.run(self._adb_base + tail,
                       capture_output=True, timeout=5.0)


# ============================================================================
# Scenario base
# ============================================================================

class ScenarioResult:
    def __init__(self, name: str):
        self.name = name
        self.iterations: List[Dict[str, Any]] = []
        self.events: List[Dict[str, Any]] = []      # filled by runner after pull
        self.passed: bool = False
        self.failures: List[str] = []

    def to_dict(self) -> dict:
        return {
            "scenario": self.name,
            "passed": self.passed,
            "iterations": self.iterations,
            "failures": self.failures,
            "event_count": len(self.events),
        }


class Scenario:
    name = "base"
    description_ar = ""

    def __init__(self, cfg: dict, modules_cfg: dict, thresholds: dict,
                 device: Device):
        self.cfg = cfg
        self.modules_cfg = modules_cfg
        self.thresholds = thresholds
        self.device = device

    def run(self) -> ScenarioResult:      # noqa: B027 (abstract)
        raise NotImplementedError

    # ------------------------------------------------------------------
    def set_auto_restart(self, module: str, enable: bool) -> None:
        code, data = self.device.broadcast(
            "set_autorestart", module=module,
            enable="true" if enable else "false",
        )
        if code != 0:
            raise AdbError(
                f"set_autorestart({module}={enable}) failed: "
                f"code={code} data={data!r}"
            )

    def crash(self, module: str) -> float:
        """Inject a crash.  Returns wallclock time (time.time()) just
        before the broadcast for latency accounting."""
        t0 = time.time()
        code, data = self.device.broadcast("crash", module=module)
        if code != 0:
            if data == "REJECT_RELEASE_BUILD":
                raise AdbError(
                    "APK مبني كـ release — crash injection مرفوض. "
                    "ثبّت debug APK: ./gradlew installDebug"
                )
            raise AdbError(
                f"crash({module}) failed: code={code} data={data!r}"
            )
        return t0

    def restart(self, module: str) -> float:
        t0 = time.time()
        code, data = self.device.broadcast("restart", module=module)
        if code != 0:
            raise AdbError(
                f"restart({module}) failed: code={code} data={data!r}"
            )
        return t0

    @staticmethod
    def _sleep_progress(seconds: float, label: str = "wait") -> None:
        """Sleep with a single in-place progress line (no flood).

        Updates every 0.5s.  Skips entirely for very short sleeps
        (<0.3s) where the overhead of the print loop is not worth it.
        """
        if seconds < 0.3:
            time.sleep(max(seconds, 0.0))
            return
        steps = max(1, int(seconds / 0.5))
        step_s = seconds / steps
        for k in range(steps):
            remaining = seconds - k * step_s
            # \r returns to start of line so we overwrite in place;
            # final \n is printed after the loop.
            print(f"      {label}: {remaining:4.1f}s remaining ", end="\r",
                  flush=True)
            time.sleep(step_s)
        # clear the line and move on
        print(f"      {label}: done                  ")


# ============================================================================
# Concrete scenarios
# ============================================================================

class SoloCrashScenario(Scenario):
    name = "solo_crash"
    description_ar = "crash كل module منفرداً مع auto_restart"

    def run(self) -> ScenarioResult:
        result = ScenarioResult(self.name)
        targets = self._expand_targets(self.cfg.get("targets", []))
        auto = bool(self.cfg.get("auto_restart", True))
        pause = float(self.cfg.get("inter_crash_pause_s", 5.0))
        wait = float(self.cfg.get("post_crash_wait_s", 3.0))
        reps = int(self.cfg.get("repetitions", 1))

        for target in targets:
            self.set_auto_restart(target, auto)

        n_total = reps * len(targets)
        n_done = 0
        for i in range(reps):
            for target in targets:
                n_done += 1
                print(f"  [{n_done}/{n_total}] crash({target}) iter {i+1}/{reps}",
                      flush=True)
                try:
                    t0 = self.crash(target)
                except AdbError as e:
                    result.failures.append(f"{target}: {e}")
                    continue
                self._sleep_progress(wait, "post_crash_wait")
                result.iterations.append({
                    "target": target,
                    "rep": i + 1,
                    "t_crash_wallclock": t0,
                    "auto_restart": auto,
                })
                self._sleep_progress(pause, "inter_crash_pause")

        # restore: disable auto_restart for everything so subsequent
        # scenarios see a clean slate.
        for target in targets:
            try:
                self.set_auto_restart(target, False)
            except AdbError:
                pass
        return result

    def _expand_targets(self, raw: List[str]) -> List[str]:
        if raw == ["all"]:
            return list(self.modules_cfg.keys())
        return raw


class RepeatedCrashScenario(SoloCrashScenario):
    """Same plumbing as solo but semantically it's 'hammer one module'."""
    name = "repeated_crash"
    description_ar = "crash نفس الـ module عدة مرات متتالية"


class ManualRestartScenario(Scenario):
    name = "manual_restart"
    description_ar = "crash مع auto_restart=OFF، ثم restart يدوي"

    def run(self) -> ScenarioResult:
        result = ScenarioResult(self.name)
        targets = self.cfg.get("targets", [])
        delay = float(self.cfg.get("manual_restart_delay_s", 2.0))
        wait = float(self.cfg.get("post_restart_wait_s", 3.0))
        reps = int(self.cfg.get("repetitions", 1))

        for target in targets:
            self.set_auto_restart(target, False)

        n_total = reps * len(targets)
        n_done = 0
        for i in range(reps):
            for target in targets:
                n_done += 1
                print(f"  [{n_done}/{n_total}] crash({target}) manual-restart iter {i+1}/{reps}",
                      flush=True)
                try:
                    t_crash = self.crash(target)
                    self._sleep_progress(delay, "manual_delay")
                    t_restart = self.restart(target)
                except AdbError as e:
                    result.failures.append(f"{target}: {e}")
                    continue
                self._sleep_progress(wait, "post_restart_wait")
                result.iterations.append({
                    "target": target,
                    "rep": i + 1,
                    "t_crash_wallclock": t_crash,
                    "t_restart_wallclock": t_restart,
                    "manual_gap_s": delay,
                })
        return result


class CascadingScenario(Scenario):
    name = "cascading"
    description_ar = "crash module أساسي، تحقق أن الـ dependents تتعافى"

    def run(self) -> ScenarioResult:
        result = ScenarioResult(self.name)
        cases = self.cfg.get("cases", [])
        auto = bool(self.cfg.get("auto_restart", True))
        wait = float(self.cfg.get("post_crash_wait_s", 5.0))
        reps = int(self.cfg.get("repetitions", 1))

        all_modules: set[str] = set()
        for case in cases:
            all_modules.add(case["victim"])
            all_modules.update(case.get("bystanders", []))
        for m in all_modules:
            self.set_auto_restart(m, auto)

        for i in range(reps):
            for case in cases:
                victim = case["victim"]
                bystanders = list(case.get("bystanders", []))
                print(f"  • cascade: crash({victim}), watch {bystanders} "
                      f"(iter {i + 1}/{reps})")
                try:
                    t_crash = self.crash(victim)
                except AdbError as e:
                    result.failures.append(f"{victim}: {e}")
                    continue
                self._sleep_progress(wait, "post_crash_wait")
                result.iterations.append({
                    "victim": victim,
                    "bystanders": bystanders,
                    "rep": i + 1,
                    "t_crash_wallclock": t_crash,
                })
        return result


SCENARIO_CLASSES = {
    "solo_crash":     SoloCrashScenario,
    "repeated_crash": RepeatedCrashScenario,
    "manual_restart": ManualRestartScenario,
    "cascading":      CascadingScenario,
}


# ============================================================================
# Runner
# ============================================================================

def load_config(path: Path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def setup_logging(verbose: bool) -> None:
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )


def make_result_dir(base: Path) -> Path:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    d = base / ts
    d.mkdir(parents=True, exist_ok=True)
    return d


def run_scenarios(cfg: dict, chosen: List[str], device: Device,
                  out_dir: Path) -> List[ScenarioResult]:
    modules_cfg = cfg.get("modules", {})
    thresholds = cfg.get("thresholds", {})

    results: List[ScenarioResult] = []
    for name in chosen:
        sc_cfg = cfg.get("scenarios", {}).get(name)
        if not sc_cfg or not sc_cfg.get("enabled", True):
            print(f"[skip] {name} (not enabled)")
            continue
        cls = SCENARIO_CLASSES.get(name)
        if cls is None:
            print(f"[skip] {name} (unknown scenario)")
            continue
        print(f"\n{'=' * 70}\n  Scenario: {name}\n  {cls.description_ar}\n"
              f"{'=' * 70}")

        # Clear on-device log so event diff is easy to interpret.
        device.clear_watchdog_log()

        scenario = cls(sc_cfg, modules_cfg, thresholds, device)
        try:
            result = scenario.run()
        except AdbError as e:
            print(f"  [ABORT] {e}")
            result = ScenarioResult(name)
            result.failures.append(str(e))

        # Let the watchdog's poll loop observe the final recovery state.
        time.sleep(1.0)

        # Pull events.  Each scenario writes to the same device log, but
        # we save a scenario-scoped copy for provenance.
        raw = out_dir / f"{name}_events.jsonl"
        pulled = device.pull_watchdog_log(raw)
        if pulled and pulled.exists():
            result.events = [json.loads(line) for line in
                             pulled.read_text().splitlines() if line.strip()]
            print(f"  [log] {len(result.events)} events pulled → {raw.name}")
        else:
            print("  [log] no events log on device (yet?)")

        metrics = watchdog_analysis.analyse_scenario(result, thresholds,
                                                    modules_cfg)
        result.passed = metrics["passed"]
        result.failures.extend(metrics.get("failure_reasons", []))

        (out_dir / f"{name}_metrics.json").write_text(
            json.dumps(metrics, indent=2, default=str))

        print(f"  [result] {name}: {'PASS' if result.passed else 'FAIL'}")
        for msg in result.failures:
            print(f"    ! {msg}")

        results.append(result)
    return results


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default=str(_SCRIPT_DIR / "watchdog_config.yaml"))
    parser.add_argument("--preset", default="quick",
                        choices=["quick", "standard", "full"])
    parser.add_argument("--scenario", action="append", dest="scenarios",
                        help="Run a specific scenario (may repeat). "
                             "Overrides --preset.")
    parser.add_argument("--module",
                        help="Single-shot: crash this module once with "
                             "auto-restart on, pull log, exit.")
    parser.add_argument("--analyze-only",
                        help="Path to an existing results dir; re-run analysis.")
    parser.add_argument("--verbose", "-v", action="store_true")
    args = parser.parse_args()

    setup_logging(args.verbose)
    cfg = load_config(Path(args.config))

    out_base = _SCRIPT_DIR / cfg.get("output", {}).get("results_dir", "results")

    # ---- analyse-only shortcut ----
    if args.analyze_only:
        old_dir = Path(args.analyze_only)
        return watchdog_analysis.reanalyse_dir(old_dir, cfg)

    out_dir = make_result_dir(out_base)
    print(f"results → {out_dir}")

    # Archive the config used for reproducibility.
    shutil.copy2(args.config, out_dir / "config.yaml")

    dev_cfg = cfg.get("device", {})
    device = Device(
        package=dev_cfg.get("package", "com.ardophone.px4v17"),
        serial=dev_cfg.get("adb_serial", "") or "",
        timeout_s=float(dev_cfg.get("broadcast_timeout_s", 5.0)),
        log_path=dev_cfg.get("log_path", ""),
    )

    # Block until WatchdogManager + native wd_init have finished wiring up.
    # start_px4_modules() runs on a background native thread and wd_init is
    # its last step, so firing the first broadcast too early causes spurious
    # FAILs.  The wait is capped so we don't hang if PX4 never finishes.
    print("Waiting for watchdog to come online (≤20s)...")
    device.wait_for_watchdog(timeout_s=20.0)

    # ---- single-shot --module ----
    if args.module:
        chosen = ["solo_crash"]
        # override scenario with a single-target config
        cfg["scenarios"]["solo_crash"]["targets"] = [args.module]
        cfg["scenarios"]["solo_crash"]["repetitions"] = 1
        cfg["scenarios"]["solo_crash"]["auto_restart"] = True
    elif args.scenarios:
        chosen = args.scenarios
    else:
        preset = cfg.get("presets", {}).get(args.preset)
        if not preset:
            print(f"[error] unknown preset '{args.preset}'")
            return 2
        chosen = preset["scenarios"]
        print(f"preset '{args.preset}' → scenarios: {chosen}")

    results = run_scenarios(cfg, chosen, device, out_dir)

    # Aggregate report
    summary = watchdog_analysis.write_report(results, cfg, out_dir)
    all_pass = all(r.passed for r in results) and results

    print("\n" + "=" * 70)
    print(f"  OVERALL: {'PASS' if all_pass else 'FAIL'}")
    print(f"  Report: {summary}")
    print("=" * 70)
    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
