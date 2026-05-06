"""High-precision test orchestrator.

Runs a command sequence at a fixed control rate while logging every
position update from the RX thread. All timestamps are monotonic
nanoseconds — never wall clock — so post-processing can compute
inter-sample dt and command-to-feedback latency without TZ/NTP issues.

Output: one CSV per test in the results/ directory, plus one
TestSession summary.
"""

from __future__ import annotations

import csv
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from xqpower import XqpowerBus


@dataclass
class TestConfig:
    name: str
    fs_hz: float                          # command rate
    cmd_per_fin: dict[int, np.ndarray]    # slot -> command sequence (deg)
    settle_s: float = 0.5                 # quiet time before & after
    description: str = ""


@dataclass
class TestResult:
    name: str
    fs_hz: float
    csv_path: str
    n_samples_cmd: int = 0
    n_samples_fb: int = 0
    duration_s: float = 0.0
    fb_rate_hz_per_fin: list[float] = field(default_factory=list)
    online_at_end: list[bool] = field(default_factory=list)
    notes: str = ""


def _busy_wait_until_ns(target_ns: int) -> None:
    """Sleep then busy-wait until monotonic clock reaches target_ns.

    Pure time.sleep on Linux is ~100µs accurate. The trailing busy-wait
    pulls us down to ~10µs at the cost of brief CPU burn — fine because
    the inter-sample interval is at most a few ms.
    """
    while True:
        remaining_ns = target_ns - time.monotonic_ns()
        if remaining_ns <= 0:
            return
        if remaining_ns > 1_500_000:  # > 1.5 ms: sleep most of it
            time.sleep((remaining_ns - 1_000_000) / 1e9)
        else:
            # tight spin
            pass


class Tester:
    def __init__(self, bus: XqpowerBus, results_dir: str):
        self.bus = bus
        self.results_dir = results_dir
        os.makedirs(results_dir, exist_ok=True)

        # Per-test logs (reset on each run)
        self._fb_log: list[tuple[int, int, int, float]] = []  # (mono_ns, slot, dt_ns, pos_deg)
        self._fb_log_lock = threading.Lock()
        self._t0_ns = 0
        self._record_active = False

    # ------------------------------------------------------------------ feedback hook

    def _on_feedback(self, slot: int, pos_deg: float, ts_ns: int) -> None:
        if not self._record_active:
            return
        with self._fb_log_lock:
            t_rel_ns = ts_ns - self._t0_ns
            self._fb_log.append((ts_ns, slot, t_rel_ns, pos_deg))

    # ------------------------------------------------------------------ run

    def run(self, cfg: TestConfig) -> TestResult:
        print(f"\n[Test] {cfg.name}  ({cfg.description or '...'})")
        bus = self.bus
        fs = cfg.fs_hz
        dt_ns = int(round(1e9 / fs))

        # Build per-fin command stream, padding to common length with the
        # last value (so every fin has the same number of samples)
        max_n = max((len(v) for v in cfg.cmd_per_fin.values()), default=0)
        cmd_streams: dict[int, np.ndarray] = {}
        for slot in range(bus.n_servos):
            v = cfg.cmd_per_fin.get(slot)
            if v is None or len(v) == 0:
                cmd_streams[slot] = np.zeros(max_n, dtype=float)
            elif len(v) < max_n:
                pad = np.full(max_n - len(v), v[-1], dtype=float)
                cmd_streams[slot] = np.concatenate([v, pad])
            else:
                cmd_streams[slot] = v[:max_n].astype(float)

        # Pre-test settle: hold zero
        for slot in range(bus.n_servos):
            bus.set_position_deg(slot, 0.0)
        time.sleep(cfg.settle_s)

        # Begin recording
        with self._fb_log_lock:
            self._fb_log.clear()
        self._t0_ns = time.monotonic_ns()
        self._record_active = True
        bus.set_feedback_callback(self._on_feedback)

        # Command log: (t_rel_ns, cmd[0..n_servos-1])
        cmd_log: list[tuple[int, list[float]]] = []
        cmd_log_reserve = max_n
        cmd_log = [None] * cmd_log_reserve  # type: ignore

        start_ns = self._t0_ns
        for k in range(max_n):
            target_ns = start_ns + k * dt_ns
            _busy_wait_until_ns(target_ns)
            t_actual = time.monotonic_ns()
            cmds = [float(cmd_streams[s][k]) for s in range(bus.n_servos)]
            for slot, ang in enumerate(cmds):
                bus.set_position_deg(slot, ang)
            cmd_log[k] = (t_actual - self._t0_ns, cmds)

        # Post-test settle (let last commands play out)
        time.sleep(cfg.settle_s)
        self._record_active = False
        bus.set_feedback_callback(None)
        # Hold-zero between tests
        for slot in range(bus.n_servos):
            bus.set_position_deg(slot, 0.0)

        # Write CSV (long format: one row per event — either CMD or FB)
        csv_path = os.path.join(self.results_dir, f"{cfg.name}.csv")
        n_fb_per_slot = [0] * bus.n_servos
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_rel_ns", "kind", "slot", "value_deg"])
            for t_rel_ns, cmds in cmd_log:
                for slot, ang in enumerate(cmds):
                    w.writerow([t_rel_ns, "CMD", slot, f"{ang:.4f}"])
            with self._fb_log_lock:
                fb_snapshot = list(self._fb_log)
            for ts_ns, slot, t_rel_ns, pos in fb_snapshot:
                w.writerow([t_rel_ns, "FB", slot, f"{pos:.4f}"])
                n_fb_per_slot[slot] += 1

        duration_s = (cmd_log[-1][0] / 1e9) if cmd_log else 0.0
        fb_rates = [n / duration_s if duration_s > 0 else 0.0 for n in n_fb_per_slot]
        online = [fb.online for fb in bus.get_all_feedback()]

        result = TestResult(
            name=cfg.name,
            fs_hz=fs,
            csv_path=csv_path,
            n_samples_cmd=max_n,
            n_samples_fb=sum(n_fb_per_slot),
            duration_s=duration_s,
            fb_rate_hz_per_fin=fb_rates,
            online_at_end=online,
        )
        print(f"  cmd samples = {max_n}, duration = {duration_s:.2f} s")
        print(f"  fb counts   = {n_fb_per_slot}  (rates ≈ {[f'{r:.0f}' for r in fb_rates]} Hz)")
        print(f"  CSV written to {csv_path}")
        return result
