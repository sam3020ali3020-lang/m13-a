"""CAN adapter for /lab — forwards PX4 SITL actuator commands to real servos.

دور هذا الموديول:
  1. يفتح CAN bus (يستخدم /direct/can_driver.py)
  2. يُهيّئ السيرفوهات (NMT start + auto-report)
  3. يستقبل HIL_ACTUATOR_CONTROLS من SITLBridge عبر callback
  4. يحوّل normalized control [-1..+1] → degrees
  5. يُرسل عبر CAN باستخدام /direct/xqpower_protocol.py
  6. يستقبل servo feedback في خيط منفصل ويخزّنه (مع timestamp)
  7. (اختياري) يحقن آخر feedback في dynamics كحقيقة الفينة

التكامل مع SITLBridge:
    bridge = SITLBridge(...)
    can_adapter = LabCanAdapter(cfg)
    can_adapter.start()
    bridge._actuator_callback = can_adapter.on_actuator_controls
    if cfg['bridge']['inject_servo_fb']:
        # patch dynamics to use latest CAN fb instead of internal servo model
        bridge._dynamics.servo_fb_provider = can_adapter.get_latest_fb_rad

    bridge.run()
    can_adapter.stop()
"""

from __future__ import annotations

import csv
import sys
import threading
import time
from pathlib import Path
from typing import List, Optional

import numpy as np

# Allow importing /direct from sibling folder
_LAB_DIR = Path(__file__).resolve().parent
_DIRECT_DIR = _LAB_DIR.parent / "direct"
sys.path.insert(0, str(_DIRECT_DIR))

from can_driver import CanBus, open_can  # noqa: E402
from xqpower_protocol import (  # noqa: E402
    decode_frame,
    encode_nmt_start,
    encode_set_position,
    encode_set_report_interval,
)


class LabCanAdapter:
    """يَعبُر بين SITLBridge actuator outputs و real CAN servos."""

    def __init__(self, cfg: dict):
        self._cfg = cfg
        self._can_cfg = cfg.get("can", {})
        self._xq = cfg.get("xqpower", {})
        self._bridge_cfg = cfg.get("bridge", {})

        self._node_ids: List[int] = [int(x) for x in self._xq.get("node_ids", [1, 2, 3, 4])]
        self._n_servos = len(self._node_ids)
        self._angle_limit = float(self._xq.get("angle_limit_deg", 20.0))
        self._units_per_deg = float(self._xq.get("units_per_deg", 18.0))
        self._report_interval_ms = int(self._xq.get("report_interval_ms", 10))

        self._cmd_rate_limit_hz = float(self._bridge_cfg.get("cmd_rate_limit_hz", 100.0))
        self._send_when_disarmed = bool(self._bridge_cfg.get("send_when_disarmed", False))
        self._zero_on_exit = bool(self._bridge_cfg.get("zero_on_exit", True))

        # State
        self._bus: Optional[CanBus] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()
        self._lock = threading.Lock()
        self._latest_fb_deg: List[Optional[float]] = [None] * self._n_servos
        self._latest_fb_t: List[Optional[float]] = [None] * self._n_servos
        self._fb_count = [0] * self._n_servos

        # Rate limiting
        self._last_tx_t: float = 0.0

        # Logging
        self._log_traffic = bool(cfg.get("output", {}).get("log_can_traffic", True))
        self._log: list = []
        self._t0 = time.monotonic()

    # ── Lifecycle ─────────────────────────────────────────────────────

    def start(self) -> None:
        print(f"[lab-can] opening backend={self._can_cfg.get('backend')}…")
        self._bus = open_can(self._can_cfg)

        # NMT Start + auto-report
        for nid in self._node_ids:
            arb, data = encode_nmt_start(nid)
            self._bus.send(arb, data)
        time.sleep(0.05)
        for nid in self._node_ids:
            arb, data = encode_set_report_interval(nid, self._report_interval_ms)
            self._bus.send(arb, data)
        time.sleep(0.05)

        self._stop_evt.clear()
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name="lab-can-rx", daemon=True
        )
        self._rx_thread.start()

        # Warm-up: some servos (esp. node 0x02) may ignore the very first
        # large command after power-up. Send a small +1° → 0° wake-up
        # sequence so all 4 are responsive before MPC starts issuing real
        # commands. Verified on hardware: without this, slot1 sometimes
        # remains at 0° during the first +10° step.
        for _ in range(2):
            for nid in self._node_ids:
                arb, data = encode_set_position(nid, 1.0, self._angle_limit,
                                                self._units_per_deg)
                self._bus.send(arb, data)
            time.sleep(0.3)
        for nid in self._node_ids:
            arb, data = encode_set_position(nid, 0.0, self._angle_limit,
                                            self._units_per_deg)
            self._bus.send(arb, data)
        time.sleep(0.3)

        print(f"[lab-can] ready — {self._n_servos} servos, "
              f"limit ±{self._angle_limit}°, rate ≤{self._cmd_rate_limit_hz}Hz")

    def stop(self) -> None:
        if self._zero_on_exit and self._bus is not None:
            try:
                for nid in self._node_ids:
                    arb, data = encode_set_position(
                        nid, 0.0, self._angle_limit, self._units_per_deg
                    )
                    self._bus.send(arb, data)
            except Exception as e:
                print(f"[lab-can] zero-on-exit failed: {e}")

        self._stop_evt.set()
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=1.0)
        if self._bus is not None:
            self._bus.close()
            self._bus = None
        print(f"[lab-can] closed. fb_counts={self._fb_count} "
              f"log_samples={len(self._log)}")

    # ── SITLBridge callback ────────────────────────────────────────────

    def on_actuator_controls(self, controls: np.ndarray, t_sim_usec: int) -> None:
        """Called by SITLBridge whenever HIL_ACTUATOR_CONTROLS arrives.

        ``controls`` هو ndarray بحجم 16 (PX4 يحدّد القنوات 0..7 عادة).
        نأخذ القنوات 0..3 (fin1..fin4) كـ normalized [-1..+1] ونحوّلها إلى deg.
        """
        if self._bus is None:
            return

        # Rate limit
        now = time.monotonic()
        if self._cmd_rate_limit_hz > 0:
            min_dt = 1.0 / self._cmd_rate_limit_hz
            if now - self._last_tx_t < min_dt:
                return
        self._last_tx_t = now

        # Map controls[0..3] → fin angles
        # PX4 rocket_mpc (airframe 22003+) publishes fin deflections in
        # RADIANS directly (not normalized). Same convention as /sitl
        # mavlink_bridge.py: self._fin_commands_rad = controls[0..3].
        # Convert rad → deg, clamp to angle_limit for safety.
        try:
            for i in range(self._n_servos):
                rad = float(controls[i]) if i < len(controls) else 0.0
                if not np.isfinite(rad):
                    rad = 0.0
                cmd_deg = np.degrees(rad)
                # Clamp to [-angle_limit, +angle_limit] for safety
                if cmd_deg > self._angle_limit:
                    cmd_deg = self._angle_limit
                elif cmd_deg < -self._angle_limit:
                    cmd_deg = -self._angle_limit
                arb, data = encode_set_position(
                    self._node_ids[i], cmd_deg,
                    self._angle_limit, self._units_per_deg,
                )
                self._bus.send(arb, data)
                if self._log_traffic:
                    self._log.append({
                        "t_s": now - self._t0,
                        "t_sim_s": t_sim_usec / 1e6,
                        "kind": "cmd",
                        "servo_idx": i,
                        "value_deg": cmd_deg,
                    })
        except Exception as e:
            # لا نُلقي استثناءً (سيُمسك في bridge)
            print(f"[lab-can] tx error: {e}")

    # ── RX worker ──────────────────────────────────────────────────────

    def _rx_loop(self) -> None:
        while not self._stop_evt.is_set():
            try:
                frame = self._bus.recv(timeout_s=0.005)
            except Exception:
                time.sleep(0.005)
                continue
            if frame is None:
                continue
            rep = decode_frame(frame.arb_id, frame.data, self._units_per_deg)
            if rep is None or rep.position_deg is None:
                continue
            try:
                idx = self._node_ids.index(rep.node_id)
            except ValueError:
                continue

            with self._lock:
                self._latest_fb_deg[idx] = rep.position_deg
                self._latest_fb_t[idx] = frame.t_s
                self._fb_count[idx] += 1
                if self._log_traffic:
                    self._log.append({
                        "t_s": frame.t_s - self._t0,
                        "t_sim_s": float("nan"),
                        "kind": "fb",
                        "servo_idx": idx,
                        "value_deg": rep.position_deg,
                    })

    # ── Public accessors (used by dynamics injection) ──────────────────

    def get_latest_fb_rad(self):
        """Returns latest 4 fin angles in radians, or None if any servo missing fb.

        Returning None lets the bridge fall back to the MPC command — important
        during the brief window between cmd issue and first servo feedback,
        so the simulator doesn't see fake 0° fins.
        """
        out = np.zeros(self._n_servos, dtype=float)
        with self._lock:
            for i in range(self._n_servos):
                v = self._latest_fb_deg[i]
                if v is None:
                    return None  # not yet ready — bridge will use MPC cmd
                out[i] = np.deg2rad(v)
        return out

    def get_fb_counts(self) -> List[int]:
        with self._lock:
            return list(self._fb_count)

    # ── Logging export ─────────────────────────────────────────────────

    def export_log(self, csv_path: Path) -> int:
        """يحفظ traffic log إلى CSV. يُعيد عدد الأسطر."""
        if not self._log_traffic:
            return 0
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "t_sim_s", "kind", "servo_idx", "value_deg"])
            for row in self._log:
                w.writerow([
                    f"{row['t_s']:.6f}",
                    "" if np.isnan(row["t_sim_s"]) else f"{row['t_sim_s']:.6f}",
                    row["kind"], row["servo_idx"],
                    f"{row['value_deg']:.4f}",
                ])
        return len(self._log)
