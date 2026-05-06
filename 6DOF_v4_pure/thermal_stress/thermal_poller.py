#!/usr/bin/env python3
"""
thermal_poller.py — Phone thermal telemetry via adb
====================================================

Polls the connected Android device every N seconds to capture:
- Thermal zone temperatures (CPU silver/gold clusters, GPU, DDR, modem, DCVS)
- Per-core CPU frequencies (cur, scaling_max, hw_max) — throttling detection
- Battery temperature, voltage, level
- Thermal HAL status (0..5 = NONE..SHUTDOWN)

Writes a CSV row per sample to `thermal_log.csv`.

All reads go through **one** `adb shell` compound command per poll — ~80-120 ms
overhead per sample is acceptable at 1 Hz. No root required on modern Android
(paths under /sys/class/thermal and /sys/devices/system/cpu/*/cpufreq are
world-readable on shell user).

Usage (standalone):
    python3 thermal_poller.py --duration 60 --out thermal_log.csv

Usage (library):
    poller = ThermalPoller(device="", poll_interval_s=1.0, zone_groups=...)
    poller.start(result_dir / "thermal_log.csv")
    ...
    poller.stop()
"""

from __future__ import annotations

import argparse
import csv
import logging
import re
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

logger = logging.getLogger("thermal_poller")

# ---------------------------------------------------------------------------
# Default zone grouping for Samsung/Qualcomm SDM845 (Note 9) and similar.
# The runner may pass a custom map from config.
# ---------------------------------------------------------------------------
DEFAULT_ZONE_GROUPS: Dict[str, List[str]] = {
    "cpu_silver": ["cpu0-silver-usr", "cpu1-silver-usr",
                   "cpu2-silver-usr", "cpu3-silver-usr"],
    "cpu_gold":   ["cpu0-gold-usr", "cpu1-gold-usr",
                   "cpu2-gold-usr", "cpu3-gold-usr"],
    "gpu":        ["gpu0-usr", "gpu1-usr"],
    "ddr":        ["ddr-usr"],
    "modem":      ["mdm-core-usr", "mdm-dsp-usr"],
    "dcvs":       ["lmh-dcvs-00", "lmh-dcvs-01"],
}

# The remote shell script is embedded; output has sentinel markers for easy parsing.
#
# We use `grep -H "" file...` instead of shell loops because each `cat` in a
# loop forks a new process in `adb shell`, which costs ~50 ms per fork on
# older hardware.  45+ thermal zones × 2 reads = 90 forks = ~5 s per poll.
# With `grep`, one process dumps all sysfs files: total poll < 200 ms.
_REMOTE_SCRIPT = (
    'grep -H ".*" '
    '/sys/class/thermal/thermal_zone*/type '
    '/sys/class/thermal/thermal_zone*/temp '
    '/sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_cur_freq '
    '/sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_max_freq '
    '/sys/devices/system/cpu/cpu[0-9]*/cpufreq/cpuinfo_max_freq '
    '2>/dev/null; '
    'echo ===DELIM===; '
    'dumpsys battery 2>/dev/null | head -15; '
    'echo ===DELIM===; '
    'dumpsys thermalservice 2>/dev/null | head -30'
)


@dataclass
class ThermalSample:
    """One thermal poll sample."""
    t_wall_s: float = 0.0
    # All zone temps (°C), keyed by zone type name
    zones_C: Dict[str, float] = field(default_factory=dict)
    # Per-core frequencies (kHz), keyed by cpu name → (cur, scaling_max, hw_max)
    cpufreq_khz: Dict[str, tuple] = field(default_factory=dict)
    # Battery fields
    battery_C: float = float("nan")
    battery_level_pct: float = float("nan")
    battery_voltage_mV: float = float("nan")
    battery_status: int = 0
    # Thermal HAL
    thermal_status: int = 0  # 0=none, 1=light, 2=moderate, 3=severe, 4=critical, 5=emergency, 6=shutdown
    hal_cpu_C: float = float("nan")
    hal_skin_C: float = float("nan")
    hal_usb_C: float = float("nan")
    hal_pa_C: float = float("nan")

    # Computed aggregates (filled by ThermalPoller.compute_aggregates)
    # Maximum temperature within each configured zone group
    group_max_C: Dict[str, float] = field(default_factory=dict)
    # Throttle ratio: min(scaling_max / hw_max) over big cores (gold)
    throttle_ratio_gold: float = 1.0
    throttle_ratio_silver: float = 1.0


def _adb_cmd(device: str, cmd: List[str]) -> List[str]:
    if device:
        return ["adb", "-s", device, *cmd]
    return ["adb", *cmd]


def _run_adb(device: str, shell_cmd: str, timeout_s: float = 5.0) -> str:
    """Run `adb shell <shell_cmd>` and return stdout text.

    Raises subprocess.CalledProcessError on non-zero exit.
    """
    proc = subprocess.run(
        _adb_cmd(device, ["shell", shell_cmd]),
        capture_output=True, text=True, timeout=timeout_s,
    )
    if proc.returncode != 0:
        raise subprocess.CalledProcessError(
            proc.returncode, proc.args, output=proc.stdout, stderr=proc.stderr)
    return proc.stdout


_CPU_RE = re.compile(r"/cpu(\d+)/cpufreq/(\w+)$")
_TZ_RE = re.compile(r"/thermal_zone(\d+)/(type|temp)$")


def parse_poll_output(text: str, zone_groups: Dict[str, List[str]]) -> ThermalSample:
    """Parse the remote script output into a ThermalSample.

    The script emits three sections separated by `===DELIM===`:
      1. grep -H output: "path:content" for thermal_zone*/{type,temp}
         and cpu*/cpufreq/{scaling_cur_freq,scaling_max_freq,cpuinfo_max_freq}
      2. dumpsys battery
      3. dumpsys thermalservice
    """
    s = ThermalSample()
    parts = text.split("===DELIM===")
    if len(parts) < 3:
        # malformed — still try to parse what we have
        parts += [""] * (3 - len(parts))

    # --- Section 1: sysfs grep output --------------------------------------
    # Collect per-zone {type, temp} and per-cpu {scaling_cur, scaling_max, hw_max}
    tz_type: Dict[int, str] = {}
    tz_temp: Dict[int, int] = {}
    cpu_vals: Dict[int, Dict[str, int]] = {}

    for raw in parts[0].splitlines():
        if ":" not in raw:
            continue
        path, content = raw.split(":", 1)
        content = content.strip()
        if not content:
            continue

        m = _TZ_RE.search(path)
        if m:
            zi = int(m.group(1))
            kind = m.group(2)
            if kind == "type":
                tz_type[zi] = content
            else:  # temp
                try:
                    tz_temp[zi] = int(content)
                except ValueError:
                    pass
            continue

        m = _CPU_RE.search(path)
        if m:
            ci = int(m.group(1))
            kind = m.group(2)
            try:
                val = int(content)
            except ValueError:
                continue
            cpu_vals.setdefault(ci, {})[kind] = val
            continue

    # Build zones_C dict
    for zi, name in tz_type.items():
        if zi in tz_temp:
            s.zones_C[name] = tz_temp[zi] / 1000.0

    # Build cpufreq_khz dict
    for ci, kv in cpu_vals.items():
        cur = kv.get("scaling_cur_freq", 0)
        scm = kv.get("scaling_max_freq", 0)
        hwm = kv.get("cpuinfo_max_freq", 0)
        s.cpufreq_khz[f"cpu{ci}"] = (cur, scm, hwm)

    # --- Section 2: dumpsys battery ----------------------------------------
    for raw in parts[1].splitlines():
        m = re.match(r"\s*temperature:\s+(-?\d+)", raw)
        if m:
            s.battery_C = int(m.group(1)) / 10.0
            continue
        m = re.match(r"\s*level:\s+(\d+)", raw)
        if m:
            s.battery_level_pct = float(m.group(1))
            continue
        m = re.match(r"\s*voltage:\s+(\d+)", raw)
        if m:
            s.battery_voltage_mV = float(m.group(1))
            continue
        m = re.match(r"\s*status:\s+(\d+)", raw)
        if m:
            s.battery_status = int(m.group(1))
            continue

    # --- Section 3: dumpsys thermalservice ---------------------------------
    for raw in parts[2].splitlines():
        m = re.match(r"\s*Thermal Status:\s+(\d+)", raw)
        if m:
            s.thermal_status = int(m.group(1))
            continue
        # "Temperature{mValue=39.4, mType=1, mName=TYPE_CPU, mStatus=0}"
        # NOTE: must match `Temperature{...}` only, NOT `CoolingDevice{...}`
        # which shares the same mName values but has meaningless mValue.
        m = re.search(r"Temperature\{mValue=([\d.+-]+).*?mName=(\w+)", raw)
        if m:
            val = float(m.group(1))
            name = m.group(2)
            if name == "TYPE_CPU":
                s.hal_cpu_C = val
            elif name == "TYPE_SKIN":
                s.hal_skin_C = val
            elif name == "TYPE_USB_PORT":
                s.hal_usb_C = val
            elif name == "TYPE_POWER_AMPLIFIER":
                s.hal_pa_C = val
            elif name == "TYPE_BATTERY":
                # Note: HAL battery temp ≠ dumpsys battery temp on some devices
                # (HAL reads sensor, battery dumpsys reads PMIC).  We keep the
                # dumpsys battery value as authoritative; ignore HAL here.
                pass

    # Aggregates ------------------------------------------------------------
    for gname, zone_names in zone_groups.items():
        vals = [s.zones_C[n] for n in zone_names if n in s.zones_C]
        s.group_max_C[gname] = max(vals) if vals else float("nan")

    # Throttle ratios per cluster.  On SDM845: cpu0-3 = silver, cpu4-7 = gold.
    gold_ratios = []
    silver_ratios = []
    for cpu, (cur, sc_max, hw_max) in s.cpufreq_khz.items():
        if hw_max <= 0:
            continue
        r = sc_max / hw_max
        # Note 9 has cpu0-3 = silver (LITTLE), cpu4-7 = gold (big)
        idx = int(cpu.replace("cpu", ""))
        if idx >= 4:
            gold_ratios.append(r)
        else:
            silver_ratios.append(r)
    s.throttle_ratio_gold = min(gold_ratios) if gold_ratios else 1.0
    s.throttle_ratio_silver = min(silver_ratios) if silver_ratios else 1.0

    return s


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

class ThermalPoller:
    """Background thermal-telemetry poller.

    Starts a daemon thread that polls adb at `poll_interval_s`, writing each
    sample as a CSV row.  Graceful stop via `stop()`.
    """

    def __init__(self,
                 device: str = "",
                 poll_interval_s: float = 1.0,
                 zone_groups: Optional[Dict[str, List[str]]] = None,
                 adb_timeout_s: float = 5.0):
        self.device = device
        self.poll_interval_s = poll_interval_s
        self.zone_groups = zone_groups or DEFAULT_ZONE_GROUPS
        self.adb_timeout_s = adb_timeout_s

        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._csv_file = None
        self._csv_writer: Optional[csv.writer] = None
        self._samples: List[ThermalSample] = []
        self._errors: int = 0
        self._success: int = 0
        self._last_sample: Optional[ThermalSample] = None
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self, csv_path: Path):
        """Start polling; writes to csv_path."""
        if self._thread is not None:
            raise RuntimeError("ThermalPoller already started")

        csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._csv_file = open(csv_path, "w", newline="")

        # header
        cols = ["t_wall_s"]
        # per-group maxes
        for g in self.zone_groups.keys():
            cols.append(f"{g}_max_C")
        # HAL
        cols += ["hal_cpu_C", "hal_skin_C", "hal_usb_C", "hal_pa_C", "thermal_status"]
        # battery
        cols += ["battery_C", "battery_level_pct", "battery_voltage_mV", "battery_status"]
        # throttle
        cols += ["throttle_ratio_gold", "throttle_ratio_silver"]
        # per-core freq (up to 8 cores; if fewer, blank)
        for i in range(8):
            cols += [f"cpu{i}_cur_khz", f"cpu{i}_scmax_khz", f"cpu{i}_hwmax_khz"]

        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(cols)
        self._csv_file.flush()

        self._thread = threading.Thread(target=self._loop, name="thermal_poller",
                                         daemon=True)
        self._thread.start()
        logger.info(f"ThermalPoller started → {csv_path} (interval={self.poll_interval_s}s)")

    def stop(self, join_timeout_s: float = 2.0):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=join_timeout_s)
            self._thread = None
        if self._csv_file is not None:
            try:
                self._csv_file.close()
            except Exception:
                pass
            self._csv_file = None
        logger.info(f"ThermalPoller stopped (ok={self._success}, err={self._errors})")

    def latest(self) -> Optional[ThermalSample]:
        """Return the most recent sample (for live UI)."""
        with self._lock:
            return self._last_sample

    def stats(self) -> Dict:
        return {"ok": self._success, "err": self._errors,
                "total": self._success + self._errors}

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _loop(self):
        # Prime adb connection once
        try:
            _run_adb(self.device, "echo ready", timeout_s=self.adb_timeout_s)
        except Exception as exc:
            logger.error(f"adb not reachable: {exc}")

        next_t = time.monotonic()
        while not self._stop.is_set():
            t_wall = time.time()
            try:
                out = _run_adb(self.device, _REMOTE_SCRIPT,
                                timeout_s=self.adb_timeout_s)
                s = parse_poll_output(out, self.zone_groups)
                s.t_wall_s = t_wall
                self._append_sample(s)
                self._success += 1
            except subprocess.TimeoutExpired:
                self._errors += 1
                logger.debug("adb poll timeout")
            except subprocess.CalledProcessError as exc:
                self._errors += 1
                logger.debug(f"adb poll failed: rc={exc.returncode}")
            except Exception as exc:
                self._errors += 1
                logger.warning(f"adb poll exception: {exc}")

            # Precise pacing — avoids drift under load
            next_t += self.poll_interval_s
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0:
                self._stop.wait(sleep_s)
            else:
                # Running late — reset next deadline
                next_t = time.monotonic()

    def _append_sample(self, s: ThermalSample):
        with self._lock:
            self._last_sample = s
        self._samples.append(s)

        # Write CSV row
        row = [f"{s.t_wall_s:.3f}"]
        for g in self.zone_groups.keys():
            v = s.group_max_C.get(g, float("nan"))
            row.append(f"{v:.2f}" if v == v else "")  # NaN check
        row += [f"{s.hal_cpu_C:.2f}" if s.hal_cpu_C == s.hal_cpu_C else "",
                f"{s.hal_skin_C:.2f}" if s.hal_skin_C == s.hal_skin_C else "",
                f"{s.hal_usb_C:.2f}" if s.hal_usb_C == s.hal_usb_C else "",
                f"{s.hal_pa_C:.2f}" if s.hal_pa_C == s.hal_pa_C else "",
                s.thermal_status]
        row += [f"{s.battery_C:.1f}" if s.battery_C == s.battery_C else "",
                f"{s.battery_level_pct:.0f}" if s.battery_level_pct == s.battery_level_pct else "",
                f"{s.battery_voltage_mV:.0f}" if s.battery_voltage_mV == s.battery_voltage_mV else "",
                s.battery_status]
        row += [f"{s.throttle_ratio_gold:.4f}",
                f"{s.throttle_ratio_silver:.4f}"]
        for i in range(8):
            name = f"cpu{i}"
            if name in s.cpufreq_khz:
                cur, sc, hw = s.cpufreq_khz[name]
                row += [cur, sc, hw]
            else:
                row += ["", "", ""]
        if self._csv_writer is not None:
            self._csv_writer.writerow(row)
            self._csv_file.flush()


# ---------------------------------------------------------------------------
# Standalone runner (manual use / sanity check)
# ---------------------------------------------------------------------------

def _main():
    ap = argparse.ArgumentParser(description="Phone thermal poller (adb)")
    ap.add_argument("--duration", type=float, default=10.0,
                     help="Poll for N seconds (default 10)")
    ap.add_argument("--interval", type=float, default=1.0,
                     help="Poll interval in seconds (default 1)")
    ap.add_argument("--device", default="", help="adb device serial (optional)")
    ap.add_argument("--out", default="thermal_log.csv",
                     help="Output CSV path")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                         format="%(asctime)s %(levelname)s: %(message)s")

    poller = ThermalPoller(device=args.device,
                            poll_interval_s=args.interval)
    out = Path(args.out)
    poller.start(out)

    try:
        end_t = time.monotonic() + args.duration
        while time.monotonic() < end_t:
            time.sleep(min(1.0, end_t - time.monotonic()))
            s = poller.latest()
            if s is not None:
                print(f"  t={time.strftime('%H:%M:%S')} "
                      f"silver={s.group_max_C.get('cpu_silver', float('nan')):.1f}°C "
                      f"gold={s.group_max_C.get('cpu_gold', float('nan')):.1f}°C "
                      f"batt={s.battery_C:.1f}°C "
                      f"status={s.thermal_status} "
                      f"throttle_gold={s.throttle_ratio_gold:.2f}")
    except KeyboardInterrupt:
        pass
    finally:
        poller.stop()

    st = poller.stats()
    print(f"\nDone. ok={st['ok']}, err={st['err']}, wrote {out}")


if __name__ == "__main__":
    _main()
