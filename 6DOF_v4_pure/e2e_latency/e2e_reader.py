#!/usr/bin/env python3
"""
e2e_reader.py — MAVLink reader for end-to-end latency tests
==============================================================

Extends sensor_reader.SensorReader to also capture:
- SERVO_OUTPUT_RAW (msg 36): MPC fin command timestamps
- DEBUG_FLOAT_ARRAY (msg 350) name="SRV_FB": servo CAN feedback
- DEBUG_FLOAT_ARRAY (msg 350) name="RktGNC": GNC timing diagnostics

All timestamps are in PX4 HRT clock (μs since PX4 boot).
"""

from __future__ import annotations

import logging
import struct
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

# Reuse sensor_reader's MAVLink primitives
_PARENT_DIR = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_PARENT_DIR / "sensor"))

from sensor_reader import (  # noqa: E402
    SensorReader,
    MSG_HIGHRES_IMU,
    MSG_ATTITUDE,
    MSG_GPS_RAW_INT,
    MSG_SCALED_PRESSURE,
    parse_highres_imu,
    parse_attitude,
    parse_gps_raw_int,
    parse_scaled_pressure,
    build_set_message_interval,
)

logger = logging.getLogger("e2e_reader")

# ============================================================================
# Additional MAVLink message IDs
# ============================================================================

MSG_SERVO_OUTPUT_RAW = 36
MSG_DEBUG_FLOAT_ARRAY = 350


# ============================================================================
# Sample dataclasses (separate from sensor_reader IMUSample to keep API clean)
# ============================================================================

@dataclass
class ServoOutputRawSample:
    """Parsed SERVO_OUTPUT_RAW (msg 36).

    Wire order (size-sorted):
        time_usec(u32) + servo1..16_raw(u16 × 16) + port(u8) + servo9..16_raw extension
    Standard 21-byte payload uses servo1..8 only.
    """
    time_usec: int = 0          # PX4 HRT μs (32-bit field, wraps)
    t_wall_s: float = 0.0
    port: int = 0
    servo: List[int] = field(default_factory=list)  # raw values (PWM µs or 0..XXXX)


@dataclass
class SrvFbSample:
    """Servo feedback from xqpower_can driver (debug_array id=1, name='SRV_FB').

    From XqpowerCan.cpp:
        data[0..3]  = cmd_deg per servo (0..3)
        data[4..7]  = fb_deg per servo (0..3)
        data[8..11] = error per servo (cmd - fb)
        data[12]    = online_mask (bit i = servo i online)
        data[13]    = tx_fail_count
    """
    time_usec: int = 0           # debug_array.timestamp (PX4 HRT)
    t_wall_s: float = 0.0
    cmd_deg: List[float] = field(default_factory=lambda: [0.0]*4)
    fb_deg:  List[float] = field(default_factory=lambda: [0.0]*4)
    err_deg: List[float] = field(default_factory=lambda: [0.0]*4)
    online_mask: int = 0
    tx_fail_count: int = 0


@dataclass
class RktGncSample:
    """GNC status snapshot (debug_array id=2, name='RktGNC', decimated 4× of SRV_FB).

    See: PX4-Autopilot/src/modules/mavlink/streams/DEBUG_FLOAT_ARRAY.hpp
    Contains: stage, t_flight, q_dyn, attitudes, fin commands, MPC/MHE timing,
              cycle_us, mpc_solve_us, mhe_solve_us, etc.
    """
    time_usec: int = 0           # rocket_gnc_status.timestamp (PX4 HRT)
    t_wall_s: float = 0.0
    stage: int = 0
    t_flight: float = 0.0
    fin: List[float] = field(default_factory=lambda: [0.0]*4)  # data[10..13]
    launched: bool = False        # data[33] > 0.5
    dt_actual: float = 0.0        # data[34] — full GNC cycle dt (s)
    dt_min: float = 0.0           # data[35]
    dt_max: float = 0.0           # data[36]
    mhe_solve_us: float = 0.0     # data[46]
    mpc_solve_us: float = 0.0     # data[47]
    cycle_us: float = 0.0         # data[48]
    mpc_solve_count: int = 0      # data[21]
    mpc_solver_status: int = 0    # data[38]
    mhe_valid: bool = False       # data[40] > 0.5


# ============================================================================
# Parsers
# ============================================================================

def parse_servo_output_raw(payload: bytes, t_wall: float) -> Optional[ServoOutputRawSample]:
    """SERVO_OUTPUT_RAW (msg 36):
        time_usec(u32) + servo1..16_raw(u16 × 16) + port(u8)
    Wire size-sorted: u32 first, then 16×u16 = 32 bytes, then u8 = 1 byte.
    Total: 4 + 32 + 1 = 37 bytes (with full 16 servos).
    Older variant uses only 8 servos: 4 + 16 + 1 = 21 bytes.
    """
    if len(payload) < 21:
        return None
    try:
        # Always at least 8 servos
        time_usec = struct.unpack_from("<I", payload, 0)[0]
        servos = list(struct.unpack_from("<8H", payload, 4))
        port = struct.unpack_from("<B", payload, 20)[0]

        # Try to read 8 more if payload is long enough (16-servo variant)
        if len(payload) >= 37:
            servos.extend(struct.unpack_from("<8H", payload, 21))

        return ServoOutputRawSample(
            time_usec=time_usec,
            t_wall_s=t_wall,
            port=port,
            servo=servos,
        )
    except struct.error:
        return None


def parse_debug_float_array(payload: bytes, t_wall: float):
    """DEBUG_FLOAT_ARRAY (msg 350):
        time_usec(u64) + array_id(u16) + name(char[10]) + data(float[58])
    Wire size-sorted: u64 first (8), float[58] (232), u16 (2), char[10] (10).
    Total: 8 + 232 + 2 + 10 = 252 bytes.

    Returns (name, sample) where sample is SrvFbSample or RktGncSample
    depending on name.  Returns (None, None) if payload malformed or unknown.
    """
    if len(payload) < 20:  # minimum for time_usec + array_id + at least short name
        return (None, None)
    try:
        time_usec = struct.unpack_from("<Q", payload, 0)[0]

        # data[58] floats
        data_floats = []
        DATA_OFFSET = 8
        DATA_LEN = 58 * 4   # 232 bytes
        if len(payload) >= DATA_OFFSET + DATA_LEN:
            data_floats = list(struct.unpack_from("<58f", payload, DATA_OFFSET))
        else:
            # truncated: pad with zeros
            available = (len(payload) - DATA_OFFSET) // 4
            if available > 0:
                data_floats = list(struct.unpack_from(f"<{available}f", payload, DATA_OFFSET))
            data_floats.extend([0.0] * (58 - len(data_floats)))

        # array_id (u16) + name (char[10])
        AID_OFFSET = DATA_OFFSET + DATA_LEN
        if len(payload) < AID_OFFSET + 2:
            return (None, None)
        array_id = struct.unpack_from("<H", payload, AID_OFFSET)[0]

        NAME_OFFSET = AID_OFFSET + 2
        name_bytes = payload[NAME_OFFSET:NAME_OFFSET+10] if len(payload) >= NAME_OFFSET+10 else payload[NAME_OFFSET:]
        name = name_bytes.split(b"\x00")[0].decode("ascii", errors="replace")
    except struct.error:
        return (None, None)

    # Dispatch by name
    if name == "SRV_FB" and array_id == 1:
        return ("SRV_FB", SrvFbSample(
            time_usec=time_usec,
            t_wall_s=t_wall,
            cmd_deg=data_floats[0:4],
            fb_deg=data_floats[4:8],
            err_deg=data_floats[8:12],
            online_mask=int(data_floats[12]) if len(data_floats) > 12 else 0,
            tx_fail_count=int(data_floats[13]) if len(data_floats) > 13 else 0,
        ))

    if name == "RktGNC" and array_id == 2:
        return ("RktGNC", RktGncSample(
            time_usec=time_usec,
            t_wall_s=t_wall,
            stage=int(data_floats[0]),
            t_flight=data_floats[1],
            fin=data_floats[10:14],
            launched=(data_floats[33] > 0.5) if len(data_floats) > 33 else False,
            dt_actual=data_floats[34] if len(data_floats) > 34 else 0.0,
            dt_min=data_floats[35] if len(data_floats) > 35 else 0.0,
            dt_max=data_floats[36] if len(data_floats) > 36 else 0.0,
            mhe_solve_us=data_floats[46] if len(data_floats) > 46 else 0.0,
            mpc_solve_us=data_floats[47] if len(data_floats) > 47 else 0.0,
            cycle_us=data_floats[48] if len(data_floats) > 48 else 0.0,
            mpc_solve_count=int(data_floats[21]) if len(data_floats) > 21 else 0,
            mpc_solver_status=int(data_floats[38]) if len(data_floats) > 38 else 0,
            mhe_valid=(data_floats[40] > 0.5) if len(data_floats) > 40 else False,
        ))

    return (None, None)


# ============================================================================
# E2EReader — extends SensorReader with E2E-specific streams
# ============================================================================

class E2EReader(SensorReader):
    """Extended reader that ALSO captures servo and GNC streams."""

    def __init__(self, host: str = "127.0.0.1", port: int = 5760,
                 timeout_s: float = 10.0):
        super().__init__(host, port, timeout_s)

        # New stores
        self.servo_raw_samples: List[ServoOutputRawSample] = []
        self.srv_fb_samples: List[SrvFbSample] = []
        self.rktgnc_samples: List[RktGncSample] = []

    # Override clear() to also clear new stores
    def clear(self):
        super().clear()
        self.servo_raw_samples.clear()
        self.srv_fb_samples.clear()
        self.rktgnc_samples.clear()

    def request_streams(self, streams: Optional[dict] = None):
        """Request E2E streams plus default IMU/attitude."""
        if streams is None:
            streams = {
                MSG_HIGHRES_IMU: 100,
                MSG_ATTITUDE: 50,
                MSG_SERVO_OUTPUT_RAW: 50,
                MSG_DEBUG_FLOAT_ARRAY: 50,
                MSG_SCALED_PRESSURE: 5,
                MSG_GPS_RAW_INT: 5,
            }
        super().request_streams(streams)

    def request_streams_from_config(self, config: dict):
        """Override to handle e2e config format."""
        streams_cfg = config.get("mavlink_streams", {})
        streams = {}
        for key, val in streams_cfg.items():
            msg_id = val.get("msg_id")
            rate_hz = val.get("rate_hz", 0)
            if msg_id and rate_hz > 0:
                streams[msg_id] = rate_hz
        super().request_streams(streams)

    def _process(self, data: bytes, t_wall: float):
        """Override to also dispatch SERVO_OUTPUT_RAW and DEBUG_FLOAT_ARRAY."""
        # First, let parent handle known messages (IMU, ATT, etc.)
        # We must NOT call parent's _process directly because it would re-feed
        # the parser. Instead, replicate the dispatch loop here.
        for msg_id, payload in self._parser.feed(data):
            self.msg_counts[msg_id] += 1

            if msg_id == MSG_HIGHRES_IMU:
                s = parse_highres_imu(payload, t_wall)
                if s:
                    self.imu_samples.append(s)

            elif msg_id == MSG_ATTITUDE:
                s = parse_attitude(payload, t_wall)
                if s:
                    self.attitude_samples.append(s)

            elif msg_id == MSG_GPS_RAW_INT:
                s = parse_gps_raw_int(payload, t_wall)
                if s:
                    self.gps_samples.append(s)

            elif msg_id == MSG_SCALED_PRESSURE:
                s = parse_scaled_pressure(payload, t_wall)
                if s:
                    self.baro_samples.append(s)

            elif msg_id == MSG_SERVO_OUTPUT_RAW:
                s = parse_servo_output_raw(payload, t_wall)
                if s:
                    self.servo_raw_samples.append(s)

            elif msg_id == MSG_DEBUG_FLOAT_ARRAY:
                name, s = parse_debug_float_array(payload, t_wall)
                if name == "SRV_FB":
                    self.srv_fb_samples.append(s)
                elif name == "RktGNC":
                    self.rktgnc_samples.append(s)

    # ------------------------------------------------------------------
    # Save helpers
    # ------------------------------------------------------------------

    def save_e2e_csv(self, result_dir: Path):
        """Save all E2E-specific streams to CSV under e2e-friendly names."""
        import csv
        result_dir.mkdir(parents=True, exist_ok=True)

        # Save IMU/attitude under e2e-friendly filenames
        if self.imu_samples:
            self.save_imu_csv(result_dir / "imu.csv")
        if self.attitude_samples:
            self.save_attitude_csv(result_dir / "attitude.csv")
        if self.baro_samples:
            self.save_baro_csv(result_dir / "baro.csv")
        if self.gps_samples:
            self.save_gps_csv(result_dir / "gps.csv")

        # Servo raw
        with open(result_dir / "servo_cmd.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_wall_s", "time_usec", "port",
                        "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7"])
            for s in self.servo_raw_samples:
                row = [f"{s.t_wall_s:.6f}", s.time_usec, s.port]
                row.extend(s.servo[:8] + [0]*max(0, 8-len(s.servo)))
                w.writerow(row)

        # SRV_FB
        with open(result_dir / "servo_fb.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_wall_s", "time_usec",
                        "cmd0", "cmd1", "cmd2", "cmd3",
                        "fb0", "fb1", "fb2", "fb3",
                        "err0", "err1", "err2", "err3",
                        "online_mask", "tx_fail_count"])
            for s in self.srv_fb_samples:
                w.writerow([f"{s.t_wall_s:.6f}", s.time_usec,
                            *[f"{v:.4f}" for v in s.cmd_deg],
                            *[f"{v:.4f}" for v in s.fb_deg],
                            *[f"{v:.4f}" for v in s.err_deg],
                            s.online_mask, s.tx_fail_count])

        # RktGNC
        with open(result_dir / "gnc.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_wall_s", "time_usec", "stage", "t_flight",
                        "fin1", "fin2", "fin3", "fin4", "launched",
                        "dt_actual", "dt_min", "dt_max",
                        "mhe_solve_us", "mpc_solve_us", "cycle_us",
                        "mpc_solve_count", "mpc_solver_status", "mhe_valid"])
            for s in self.rktgnc_samples:
                w.writerow([f"{s.t_wall_s:.6f}", s.time_usec, s.stage,
                            f"{s.t_flight:.4f}",
                            *[f"{v:.4f}" for v in s.fin],
                            int(s.launched),
                            f"{s.dt_actual:.6f}", f"{s.dt_min:.6f}", f"{s.dt_max:.6f}",
                            f"{s.mhe_solve_us:.1f}", f"{s.mpc_solve_us:.1f}",
                            f"{s.cycle_us:.1f}",
                            s.mpc_solve_count, s.mpc_solver_status,
                            int(s.mhe_valid)])

        logger.info(f"E2E CSVs saved to {result_dir} "
                    f"(servo_raw={len(self.servo_raw_samples)}, "
                    f"srv_fb={len(self.srv_fb_samples)}, "
                    f"rktgnc={len(self.rktgnc_samples)})")
