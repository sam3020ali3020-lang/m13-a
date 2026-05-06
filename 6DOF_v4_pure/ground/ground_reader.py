#!/usr/bin/env python3
"""
ground_reader.py — قارئ MAVLink للاختبار الأرضي التكاملي
=========================================================

يقرأ بيانات الحساسات + EKF2 + MPC timing + CPU load من PX4 على الهاتف.
يعتمد على sensor_reader.py ويُوسّعه بـ:
  - ESTIMATOR_STATUS (msg 230) — صحة EKF2
  - SYS_STATUS (msg 1) — حمل CPU
  - DEBUG_FLOAT_ARRAY (msg 350) — RktGNC timing (mhe/mpc/cycle)
  - HEARTBEAT (msg 0) — حالة التسلّح والوضع
"""

from __future__ import annotations

import logging
import struct
import sys
import time
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

_SCRIPT_DIR = Path(__file__).resolve().parent
_SENSOR_DIR = _SCRIPT_DIR.parent / "sensor"
sys.path.insert(0, str(_SENSOR_DIR))

from sensor_reader import (  # noqa: E402
    MAVLINK_STX_V2, MAVLINK_HEADER_LEN_V2, MAVLINK_CHECKSUM_LEN,
    MSG_HEARTBEAT, MSG_HIGHRES_IMU, MSG_ATTITUDE, MSG_SCALED_PRESSURE,
    MSG_GPS_RAW_INT, MSG_SET_MESSAGE_INTERVAL, MSG_COMMAND_LONG,
    MSG_COMMAND_ACK,
    MavParser, SensorReader,
    parse_highres_imu, parse_attitude, parse_gps_raw_int, parse_scaled_pressure,
    build_heartbeat_gcs, build_command_long, build_set_message_interval,
    IMUSample, AttitudeSample, GPSSample, BaroSample,
)

logger = logging.getLogger("ground_reader")

# Additional MAVLink message IDs
MSG_SYS_STATUS = 1
MSG_ESTIMATOR_STATUS = 230
MSG_DEBUG_FLOAT_ARRAY = 350

# CRC extras
CRC_SYS_STATUS = 124
CRC_ESTIMATOR_STATUS = 163
CRC_DEBUG_FLOAT_ARRAY = 232

# MAVLink command IDs
MAV_CMD_COMPONENT_ARM_DISARM = 400

# Arming constants
ARM_MAGIC_FORCE = 21196.0  # force-arm magic value (p2)
MAV_RESULT_ACCEPTED = 0
MAV_MODE_SAFETY_ARMED_BIT = 128  # MAV_MODE_FLAG_SAFETY_ARMED


# ============================================================================
# Additional data types
# ============================================================================

@dataclass
class EstimatorStatusSample:
    """Parsed ESTIMATOR_STATUS (msg 230)."""
    t_boot_us: int = 0
    t_wall_s: float = 0.0
    vel_ratio: float = 0.0          # velocity innovation test ratio
    pos_horiz_ratio: float = 0.0    # horizontal position innovation test ratio
    pos_vert_ratio: float = 0.0     # vertical position innovation test ratio
    mag_ratio: float = 0.0          # magnetometer innovation test ratio
    hagl_ratio: float = 0.0         # height above ground innovation test ratio
    tas_ratio: float = 0.0          # true airspeed innovation test ratio
    pos_horiz_accuracy: float = 0.0 # m
    pos_vert_accuracy: float = 0.0  # m
    flags: int = 0                  # solution_status_flags bitmask


@dataclass
class SysStatusSample:
    """Parsed SYS_STATUS (msg 1)."""
    t_wall_s: float = 0.0
    onboard_control_sensors_present: int = 0
    onboard_control_sensors_enabled: int = 0
    onboard_control_sensors_health: int = 0
    load: int = 0           # ‰ (permille) CPU load
    voltage_battery: int = 0  # mV
    current_battery: int = 0  # cA (10mA)
    battery_remaining: int = 0  # %
    drop_rate_comm: int = 0
    errors_comm: int = 0


@dataclass
class TimingSample:
    """Parsed RktGNC DEBUG_FLOAT_ARRAY (msg 350, array_id=2)."""
    t_wall_s: float = 0.0
    mhe_solve_us: float = 0.0
    mpc_solve_us: float = 0.0
    cycle_us: float = 0.0


@dataclass
class GncNavSample:
    """Navigation snapshot from RktGNC DEBUG_FLOAT_ARRAY."""
    t_wall_s: float = 0.0
    bearing_deg: float = 0.0          # data[31]
    target_range_remaining: float = 0.0  # data[32]
    launched: bool = False             # data[33]
    pos_downrange: float = 0.0         # data[26]
    pos_crossrange: float = 0.0        # data[27]
    altitude: float = 0.0              # data[14]
    t_flight: float = 0.0              # data[1]


@dataclass
class HeartbeatSample:
    """Parsed HEARTBEAT (msg 0)."""
    t_wall_s: float = 0.0
    custom_mode: int = 0
    mav_type: int = 0
    autopilot: int = 0
    base_mode: int = 0
    system_status: int = 0


@dataclass
class CommandAckSample:
    """Parsed COMMAND_ACK (msg 77)."""
    t_wall_s: float = 0.0
    command: int = 0
    result: int = 0       # MAV_RESULT_*
    progress: int = 0     # -1 if N/A
    result_param2: int = 0
    target_system: int = 0
    target_component: int = 0


# ============================================================================
# Parsers
# ============================================================================

def parse_estimator_status(payload: bytes, t_wall: float) -> Optional[EstimatorStatusSample]:
    """ESTIMATOR_STATUS (msg 230) wire order (size-sorted):
    time_usec(u64) + vel_ratio(f) + pos_horiz_ratio(f) + pos_vert_ratio(f)
    + mag_ratio(f) + hagl_ratio(f) + tas_ratio(f) + pos_horiz_accuracy(f)
    + pos_vert_accuracy(f) + flags(u16)
    Total: 42 bytes
    """
    FULL = 42
    if len(payload) < 8:
        return None
    if len(payload) < FULL:
        payload = payload + b"\x00" * (FULL - len(payload))
    try:
        v = struct.unpack("<Q8fH", payload[:FULL])
        return EstimatorStatusSample(
            t_boot_us=v[0], t_wall_s=t_wall,
            vel_ratio=v[1], pos_horiz_ratio=v[2], pos_vert_ratio=v[3],
            mag_ratio=v[4], hagl_ratio=v[5], tas_ratio=v[6],
            pos_horiz_accuracy=v[7], pos_vert_accuracy=v[8],
            flags=v[9],
        )
    except struct.error:
        return None


def parse_sys_status(payload: bytes, t_wall: float) -> Optional[SysStatusSample]:
    """SYS_STATUS (msg 1) wire order (size-sorted):
    onboard_control_sensors_present(u32) + enabled(u32) + health(u32)
    + load(u16) + voltage_battery(u16) + current_battery(i16)
    + drop_rate_comm(u16) + errors_comm(u16) + errors_count1..4(u16×4)
    + battery_remaining(i8)
    Total: 31 bytes
    """
    FULL = 31
    if len(payload) < 12:
        return None
    if len(payload) < FULL:
        payload = payload + b"\x00" * (FULL - len(payload))
    try:
        v = struct.unpack("<IIIHHhHH4Hb", payload[:FULL])
        return SysStatusSample(
            t_wall_s=t_wall,
            onboard_control_sensors_present=v[0],
            onboard_control_sensors_enabled=v[1],
            onboard_control_sensors_health=v[2],
            load=v[3],
            voltage_battery=v[4],
            current_battery=v[5],
            drop_rate_comm=v[6],
            errors_comm=v[7],
            battery_remaining=v[12],
        )
    except struct.error:
        return None


def parse_debug_float_array(payload: bytes, t_wall: float) -> Optional[TimingSample]:
    """DEBUG_FLOAT_ARRAY (msg 350):
    time_usec(u64) + array_id(u16) + name[10] + data[58](f32)
    Total: 252 bytes. MAVLink v2 truncates trailing zeros.

    RktGNC (array_id=2):
        data[46] = mhe_solve_us
        data[47] = mpc_solve_us
        data[48] = cycle_us
    """
    FULL = 252
    if len(payload) < 8:
        return None
    if len(payload) < FULL:
        payload = payload + b"\x00" * (FULL - len(payload))
    try:
        t_us = struct.unpack("<Q", payload[:8])[0]
        array_id = struct.unpack("<H", payload[8:10])[0]
        name = payload[10:20].rstrip(b"\x00").decode("ascii", "replace")

        # Only process RktGNC (array_id=2)
        if array_id != 2 or not name.upper().startswith("RKTGNC"):
            return None

        data = list(struct.unpack("<58f", payload[20:20 + 232]))
        mhe_us = float(data[46])
        mpc_us = float(data[47])
        cycle_us = float(data[48])

        # Skip zero samples (solver not running yet)
        if mhe_us == 0.0 and mpc_us == 0.0 and cycle_us == 0.0:
            return None

        return TimingSample(
            t_wall_s=t_wall,
            mhe_solve_us=mhe_us,
            mpc_solve_us=mpc_us,
            cycle_us=cycle_us,
        )
    except (struct.error, IndexError, TypeError, ValueError):
        return None


def parse_gnc_nav(payload: bytes, t_wall: float) -> Optional[GncNavSample]:
    """Extract navigation fields from RktGNC DEBUG_FLOAT_ARRAY.

    data[1]  = t_flight
    data[14] = altitude
    data[26] = pos_downrange
    data[27] = pos_crossrange
    data[31] = bearing_deg
    data[32] = target_range_remaining
    data[33] = launched (0/1)
    """
    FULL = 252
    if len(payload) < 8:
        return None
    if len(payload) < FULL:
        payload = payload + b"\x00" * (FULL - len(payload))
    try:
        array_id = struct.unpack("<H", payload[8:10])[0]
        name = payload[10:20].rstrip(b"\x00").decode("ascii", "replace")
        if array_id != 2 or not name.upper().startswith("RKTGNC"):
            return None

        data = list(struct.unpack("<58f", payload[20:20 + 232]))
        return GncNavSample(
            t_wall_s=t_wall,
            bearing_deg=data[31],
            target_range_remaining=data[32],
            launched=data[33] > 0.5,
            pos_downrange=data[26],
            pos_crossrange=data[27],
            altitude=data[14],
            t_flight=data[1],
        )
    except (struct.error, IndexError, TypeError, ValueError):
        return None


def parse_heartbeat(payload: bytes, t_wall: float) -> Optional[HeartbeatSample]:
    """HEARTBEAT (msg 0) wire order (size-sorted):
    custom_mode(u32) + type(u8) + autopilot(u8) + base_mode(u8)
    + system_status(u8) + mavlink_version(u8)
    Total: 9 bytes
    """
    FULL = 9
    if len(payload) < 5:
        return None
    if len(payload) < FULL:
        payload = payload + b"\x00" * (FULL - len(payload))
    try:
        v = struct.unpack("<IBBBBB", payload[:FULL])
        return HeartbeatSample(
            t_wall_s=t_wall,
            custom_mode=v[0],
            mav_type=v[1],
            autopilot=v[2],
            base_mode=v[3],
            system_status=v[4],
        )
    except struct.error:
        return None


def parse_command_ack(payload: bytes, t_wall: float) -> Optional[CommandAckSample]:
    """COMMAND_ACK (msg 77) wire order (size-sorted):
    command(u16) + result(u8) + progress(i8) + result_param2(i32)
    + target_system(u8) + target_component(u8)
    Total: 10 bytes (MAVLink v2 may truncate trailing zeros)
    """
    FULL = 10
    if len(payload) < 3:
        return None
    if len(payload) < FULL:
        payload = payload + b"\x00" * (FULL - len(payload))
    try:
        v = struct.unpack("<HBbiBB", payload[:FULL])
        return CommandAckSample(
            t_wall_s=t_wall,
            command=v[0],
            result=v[1],
            progress=v[2],
            result_param2=v[3],
            target_system=v[4],
            target_component=v[5],
        )
    except struct.error:
        return None


def build_arm_command(force: bool = True) -> bytes:
    """Build MAV_CMD_COMPONENT_ARM_DISARM command.

    p1=1.0 → arm, p2=21196 → force-arm (bypasses pre-arm checks).
    """
    p2 = ARM_MAGIC_FORCE if force else 0.0
    return build_command_long(
        command=MAV_CMD_COMPONENT_ARM_DISARM,
        p1=1.0,       # 1 = ARM
        p2=p2,        # force-arm magic
    )


# ============================================================================
# GroundReader — extends SensorReader with EKF2/MPC/CPU monitoring
# ============================================================================

class GroundReader(SensorReader):
    """SensorReader + EKF2 status + MPC timing + CPU load."""

    def __init__(self, host: str = "127.0.0.1", port: int = 5760,
                 timeout_s: float = 10.0):
        super().__init__(host, port, timeout_s)
        self.estimator_samples: List[EstimatorStatusSample] = []
        self.sys_status_samples: List[SysStatusSample] = []
        self.timing_samples: List[TimingSample] = []
        self.gnc_nav_samples: List[GncNavSample] = []
        self.heartbeat_samples: List[HeartbeatSample] = []
        self.ack_samples: List[CommandAckSample] = []
        self._armed: bool = False
        self._arm_ack_received: bool = False

    def clear(self):
        super().clear()
        self.estimator_samples.clear()
        self.sys_status_samples.clear()
        self.timing_samples.clear()
        self.gnc_nav_samples.clear()
        self.heartbeat_samples.clear()
        self.ack_samples.clear()
        self._armed = False
        self._arm_ack_received = False

    def _process(self, data: bytes, t_wall: float):
        """Override to add new message types."""
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

            elif msg_id == MSG_ESTIMATOR_STATUS:
                s = parse_estimator_status(payload, t_wall)
                if s:
                    self.estimator_samples.append(s)

            elif msg_id == MSG_SYS_STATUS:
                s = parse_sys_status(payload, t_wall)
                if s:
                    self.sys_status_samples.append(s)

            elif msg_id == MSG_DEBUG_FLOAT_ARRAY:
                s = parse_debug_float_array(payload, t_wall)
                if s:
                    self.timing_samples.append(s)
                nav = parse_gnc_nav(payload, t_wall)
                if nav:
                    self.gnc_nav_samples.append(nav)

            elif msg_id == MSG_HEARTBEAT:
                s = parse_heartbeat(payload, t_wall)
                if s and s.mav_type != 6:  # Skip GCS heartbeats (type=6)
                    self.heartbeat_samples.append(s)
                    # Track armed state from base_mode
                    if s.base_mode & MAV_MODE_SAFETY_ARMED_BIT:
                        self._armed = True

            elif msg_id == MSG_COMMAND_ACK:
                s = parse_command_ack(payload, t_wall)
                if s:
                    self.ack_samples.append(s)
                    if s.command == MAV_CMD_COMPONENT_ARM_DISARM:
                        if s.result == MAV_RESULT_ACCEPTED:
                            self._arm_ack_received = True
                            self._armed = True

    # ── Export ─────────────────────────────────────────────────────────────

    def save_estimator_csv(self, path: Path):
        import csv
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_wall_s", "t_boot_us", "vel_ratio", "pos_horiz_ratio",
                        "pos_vert_ratio", "mag_ratio", "hagl_ratio", "tas_ratio",
                        "pos_horiz_accuracy_m", "pos_vert_accuracy_m", "flags"])
            for s in self.estimator_samples:
                w.writerow([f"{s.t_wall_s:.6f}", s.t_boot_us,
                            f"{s.vel_ratio:.4f}", f"{s.pos_horiz_ratio:.4f}",
                            f"{s.pos_vert_ratio:.4f}", f"{s.mag_ratio:.4f}",
                            f"{s.hagl_ratio:.4f}", f"{s.tas_ratio:.4f}",
                            f"{s.pos_horiz_accuracy:.2f}", f"{s.pos_vert_accuracy:.2f}",
                            f"0x{s.flags:04X}"])
        logger.info(f"Estimator CSV: {path} ({len(self.estimator_samples)} samples)")

    def save_timing_csv(self, path: Path):
        import csv
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_wall_s", "mhe_solve_us", "mpc_solve_us", "cycle_us"])
            for s in self.timing_samples:
                w.writerow([f"{s.t_wall_s:.6f}",
                            f"{s.mhe_solve_us:.0f}",
                            f"{s.mpc_solve_us:.0f}",
                            f"{s.cycle_us:.0f}"])
        logger.info(f"Timing CSV: {path} ({len(self.timing_samples)} samples)")

    def save_sys_status_csv(self, path: Path):
        import csv
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_wall_s", "cpu_load_permille", "voltage_mV",
                        "current_cA", "battery_pct", "drop_rate", "errors_comm"])
            for s in self.sys_status_samples:
                w.writerow([f"{s.t_wall_s:.6f}", s.load, s.voltage_battery,
                            s.current_battery, s.battery_remaining,
                            s.drop_rate_comm, s.errors_comm])
        logger.info(f"SysStatus CSV: {path} ({len(self.sys_status_samples)} samples)")

    @property
    def is_armed(self) -> bool:
        """True if PX4 reports armed (from HEARTBEAT base_mode or COMMAND_ACK)."""
        return self._armed

    @property
    def arm_ack_received(self) -> bool:
        """True if COMMAND_ACK for arm command was ACCEPTED."""
        return self._arm_ack_received

    def send_arm(self, force: bool = True):
        """Send MAV_CMD_COMPONENT_ARM_DISARM via TCP."""
        pkt = build_arm_command(force=force)
        self._send(pkt)
        logger.info("Sent ARM command (force=%s)", force)

    def save_all_ground(self, result_dir: Path):
        """Save all ground test data."""
        result_dir.mkdir(parents=True, exist_ok=True)
        if self.imu_samples:
            self.save_imu_csv(result_dir / "ground_imu.csv")
        if self.attitude_samples:
            self.save_attitude_csv(result_dir / "ground_attitude.csv")
        if self.gps_samples:
            self.save_gps_csv(result_dir / "ground_gps.csv")
        if self.baro_samples:
            self.save_baro_csv(result_dir / "ground_baro.csv")
        if self.estimator_samples:
            self.save_estimator_csv(result_dir / "ground_estimator.csv")
        if self.timing_samples:
            self.save_timing_csv(result_dir / "ground_timing.csv")
        if self.sys_status_samples:
            self.save_sys_status_csv(result_dir / "ground_sys_status.csv")
