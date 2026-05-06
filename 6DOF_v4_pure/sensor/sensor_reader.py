#!/usr/bin/env python3
"""
sensor_reader.py — MAVLink v2 sensor data reader from PX4 on phone
===================================================================

يتصل بالهاتف عبر TCP:5760 (adb forward) ويقرأ بيانات الحساسات الخام
من رسائل MAVLink v2 بدون اعتماد على pymavlink.

الاستخدام:
    reader = SensorReader("127.0.0.1", 5760)
    reader.connect()
    reader.request_streams()       # SET_MESSAGE_INTERVAL
    reader.record(duration_s=60)   # يسجّل لمدة 60 ثانية
    df = reader.get_dataframe()    # pandas DataFrame
"""

from __future__ import annotations

import logging
import socket
import struct
import time
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger("sensor_reader")

# ============================================================================
# MAVLink v2 Constants
# ============================================================================

MAVLINK_STX_V2 = 0xFD
MAVLINK_HEADER_LEN_V2 = 10
MAVLINK_CHECKSUM_LEN = 2

# Message IDs
MSG_HEARTBEAT = 0
MSG_SYSTEM_TIME = 2
MSG_PARAM_VALUE = 22
MSG_GPS_RAW_INT = 24
MSG_RAW_IMU = 27
MSG_SCALED_PRESSURE = 29
MSG_ATTITUDE = 30
MSG_HIGHRES_IMU = 105
MSG_SET_MESSAGE_INTERVAL = 511
MSG_COMMAND_LONG = 76
MSG_COMMAND_ACK = 77

# CRC extras (from MAVLink v2 spec)
CRC_HEARTBEAT = 50
CRC_SET_MESSAGE_INTERVAL = 90
CRC_COMMAND_LONG = 152

# Sysid/compid
GCS_SYS_ID = 255
GCS_COMP_ID = 190
TARGET_SYS_ID = 1
TARGET_COMP_ID = 1


# ============================================================================
# MAVLink v2 helpers (reused from pil/hil bridges)
# ============================================================================

def _x25_crc(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        crc &= 0xFFFF
    return crc


def _pack_v2(msg_id: int, payload: bytes, crc_extra: int,
             sys_id: int = GCS_SYS_ID, comp_id: int = GCS_COMP_ID,
             seq: list = [0]) -> bytes:
    trimmed = payload.rstrip(b"\x00")
    if not trimmed:
        trimmed = b"\x00"
    tlen = len(trimmed)
    seq_num = seq[0] & 0xFF
    seq[0] = (seq[0] + 1) & 0xFF
    header = struct.pack(
        "<BBBBBBBHB",
        MAVLINK_STX_V2, tlen, 0, 0, seq_num,
        sys_id, comp_id,
        msg_id & 0xFFFF, (msg_id >> 16) & 0xFF,
    )
    crc = _x25_crc(header[1:] + trimmed)
    crc = _x25_crc(bytes([crc_extra]), crc)
    return header + trimmed + struct.pack("<H", crc)


def build_heartbeat_gcs() -> bytes:
    payload = struct.pack("<IBBBBB", 0, 6, 8, 0, 4, 3)
    return _pack_v2(MSG_HEARTBEAT, payload, CRC_HEARTBEAT)


def build_command_long(command: int, target_sys: int = TARGET_SYS_ID,
                       target_comp: int = TARGET_COMP_ID,
                       confirmation: int = 0,
                       p1=0.0, p2=0.0, p3=0.0, p4=0.0,
                       p5=0.0, p6=0.0, p7=0.0) -> bytes:
    """COMMAND_LONG (msg 76): generic builder.

    MAVLink v2 wire order is SIZE-SORTED (largest first):
        param1..param7 (7×f32) + command (u16) + target_system (u8)
        + target_component (u8) + confirmation (u8)
    Total: 33 bytes.
    """
    payload = struct.pack(
        "<7fHBBB",
        p1, p2, p3, p4, p5, p6, p7,
        command,
        target_sys,
        target_comp,
        confirmation,
    )
    return _pack_v2(MSG_COMMAND_LONG, payload, CRC_COMMAND_LONG)


def build_set_message_interval(msg_id: int, interval_us: int) -> bytes:
    """COMMAND_LONG: MAV_CMD_SET_MESSAGE_INTERVAL (511).

    MAV_CMD_SET_MESSAGE_INTERVAL:
        param1 = message ID to stream
        param2 = interval in µs (-1 to disable, 0 = default rate)
    """
    return build_command_long(
        command=511,
        p1=float(msg_id),
        p2=float(interval_us),
    )


# ============================================================================
# Parsers for incoming messages
# ============================================================================

@dataclass
class IMUSample:
    """Parsed HIGHRES_IMU (msg 105) or RAW_IMU (msg 27)."""
    t_boot_us: int = 0
    t_wall_s: float = 0.0
    ax: float = 0.0       # m/s²
    ay: float = 0.0
    az: float = 0.0
    gx: float = 0.0       # rad/s
    gy: float = 0.0
    gz: float = 0.0
    mx: float = 0.0       # Gauss
    my: float = 0.0
    mz: float = 0.0
    abs_pressure: float = 0.0   # hPa
    temperature: float = 0.0    # °C
    source: str = "highres"


@dataclass
class GPSSample:
    """Parsed GPS_RAW_INT (msg 24)."""
    t_boot_us: int = 0
    t_wall_s: float = 0.0
    fix_type: int = 0
    lat_e7: int = 0
    lon_e7: int = 0
    alt_mm: int = 0
    eph: int = 0          # cm HDOP
    epv: int = 0          # cm VDOP
    vel_cms: int = 0      # cm/s
    cog_cdeg: int = 0     # cdeg
    satellites: int = 0


@dataclass
class BaroSample:
    """Parsed SCALED_PRESSURE (msg 29)."""
    t_boot_ms: int = 0
    t_wall_s: float = 0.0
    press_abs: float = 0.0   # hPa
    press_diff: float = 0.0  # hPa
    temperature: float = 0.0 # cdeg → °C (÷100)


@dataclass
class AttitudeSample:
    """Parsed ATTITUDE (msg 30)."""
    t_boot_ms: int = 0
    t_wall_s: float = 0.0
    roll: float = 0.0     # rad
    pitch: float = 0.0
    yaw: float = 0.0
    rollspeed: float = 0.0
    pitchspeed: float = 0.0
    yawspeed: float = 0.0


def parse_highres_imu(payload: bytes, t_wall: float) -> Optional[IMUSample]:
    """HIGHRES_IMU (msg 105): size-sorted wire order.
    time_usec(u64) + xacc(f) + yacc(f) + zacc(f) + xgyro(f) + ygyro(f) + zgyro(f)
    + xmag(f) + ymag(f) + zmag(f) + abs_pressure(f) + diff_pressure(f)
    + pressure_alt(f) + temperature(f) + fields_updated(u16)
    Total: 62 bytes
    """
    FULL = 62
    if len(payload) < 8:
        return None
    if len(payload) < FULL:
        payload = payload + b"\x00" * (FULL - len(payload))
    try:
        v = struct.unpack("<Q13fH", payload[:FULL])
        return IMUSample(
            t_boot_us=v[0], t_wall_s=t_wall,
            ax=v[1], ay=v[2], az=v[3],
            gx=v[4], gy=v[5], gz=v[6],
            mx=v[7], my=v[8], mz=v[9],
            abs_pressure=v[10], temperature=v[13],
            source="highres",
        )
    except struct.error:
        return None


def parse_raw_imu(payload: bytes, t_wall: float) -> Optional[IMUSample]:
    """RAW_IMU (msg 27): size-sorted wire order.
    time_usec(u64) + xacc(i16) + yacc(i16) + zacc(i16)
    + xgyro(i16) + ygyro(i16) + zgyro(i16)
    + xmag(i16) + ymag(i16) + zmag(i16)
    + [id(u8) + temperature(i16)]  -- extensions
    Total: 26+ bytes
    """
    if len(payload) < 26:
        return None
    try:
        v = struct.unpack("<Q9h", payload[:26])
        # Raw values — scaling is platform-dependent.
        # For MPU6050-like: accel in mg (÷1000→g→*9.81→m/s²), gyro in mrad/s (÷1000→rad/s)
        # We keep raw and flag source
        return IMUSample(
            t_boot_us=v[0], t_wall_s=t_wall,
            ax=v[1] * 9.81e-3, ay=v[2] * 9.81e-3, az=v[3] * 9.81e-3,
            gx=v[4] * 1e-3, gy=v[5] * 1e-3, gz=v[6] * 1e-3,
            mx=v[7] * 1e-3, my=v[8] * 1e-3, mz=v[9] * 1e-3,
            source="raw",
        )
    except struct.error:
        return None


def parse_gps_raw_int(payload: bytes, t_wall: float) -> Optional[GPSSample]:
    """GPS_RAW_INT (msg 24): size-sorted wire order.
    time_usec(u64) + lat(i32) + lon(i32) + alt(i32) + eph(u16) + epv(u16)
    + vel(u16) + cog(u16) + fix_type(u8) + satellites_visible(u8)
    Total: 30 bytes
    """
    FULL = 30
    if len(payload) < 8:
        return None
    if len(payload) < FULL:
        payload = payload + b"\x00" * (FULL - len(payload))
    try:
        v = struct.unpack("<QiiiHHHHBB", payload[:FULL])
        return GPSSample(
            t_boot_us=v[0], t_wall_s=t_wall,
            lat_e7=v[1], lon_e7=v[2], alt_mm=v[3],
            eph=v[4], epv=v[5], vel_cms=v[6], cog_cdeg=v[7],
            fix_type=v[8], satellites=v[9],
        )
    except struct.error:
        return None


def parse_scaled_pressure(payload: bytes, t_wall: float) -> Optional[BaroSample]:
    """SCALED_PRESSURE (msg 29): size-sorted wire order.
    time_boot_ms(u32) + press_abs(f) + press_diff(f) + temperature(i16)
    Total: 14 bytes
    """
    FULL = 14
    if len(payload) < 4:
        return None
    if len(payload) < FULL:
        payload = payload + b"\x00" * (FULL - len(payload))
    try:
        v = struct.unpack("<Iffh", payload[:FULL])
        return BaroSample(
            t_boot_ms=v[0], t_wall_s=t_wall,
            press_abs=v[1], press_diff=v[2],
            temperature=v[3] / 100.0,
        )
    except struct.error:
        return None


def parse_attitude(payload: bytes, t_wall: float) -> Optional[AttitudeSample]:
    """ATTITUDE (msg 30): size-sorted wire order.
    time_boot_ms(u32) + roll(f) + pitch(f) + yaw(f)
    + rollspeed(f) + pitchspeed(f) + yawspeed(f)
    Total: 28 bytes
    """
    FULL = 28
    if len(payload) < 4:
        return None
    if len(payload) < FULL:
        payload = payload + b"\x00" * (FULL - len(payload))
    try:
        v = struct.unpack("<I6f", payload[:FULL])
        return AttitudeSample(
            t_boot_ms=v[0], t_wall_s=t_wall,
            roll=v[1], pitch=v[2], yaw=v[3],
            rollspeed=v[4], pitchspeed=v[5], yawspeed=v[6],
        )
    except struct.error:
        return None


# ============================================================================
# MavParser — stateful byte-stream → message parser
# ============================================================================

class MavParser:
    def __init__(self):
        self._buf = bytearray()

    def feed(self, data: bytes) -> List[Tuple[int, bytes]]:
        self._buf.extend(data)
        out = []
        while True:
            idx = self._buf.find(bytes([MAVLINK_STX_V2]))
            if idx < 0:
                self._buf.clear()
                break
            if idx > 0:
                del self._buf[:idx]
            if len(self._buf) < MAVLINK_HEADER_LEN_V2:
                break
            plen = self._buf[1]
            total = MAVLINK_HEADER_LEN_V2 + plen + MAVLINK_CHECKSUM_LEN
            if len(self._buf) < total:
                break
            msg_id = self._buf[7] | (self._buf[8] << 8) | (self._buf[9] << 16)
            payload = bytes(self._buf[MAVLINK_HEADER_LEN_V2:MAVLINK_HEADER_LEN_V2 + plen])
            out.append((msg_id, payload))
            del self._buf[:total]
        return out


# ============================================================================
# SensorReader — main interface
# ============================================================================

class SensorReader:
    """Connects to PX4 on phone via TCP MAVLink and records sensor data."""

    def __init__(self, host: str = "127.0.0.1", port: int = 5760,
                 timeout_s: float = 10.0):
        self.host = host
        self.port = port
        self.timeout_s = timeout_s
        self._sock: Optional[socket.socket] = None
        self._parser = MavParser()
        self._t0: float = 0.0

        # Data stores
        self.imu_samples: List[IMUSample] = []
        self.gps_samples: List[GPSSample] = []
        self.baro_samples: List[BaroSample] = []
        self.attitude_samples: List[AttitudeSample] = []

        # Stats
        self.msg_counts: Dict[int, int] = defaultdict(int)

    def connect(self) -> bool:
        """Connect to PX4 MAVLink TCP port."""
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(self.timeout_s)
            self._sock.connect((self.host, self.port))
            self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._t0 = time.monotonic()
            logger.info(f"Connected to {self.host}:{self.port}")
            return True
        except (socket.error, OSError) as e:
            logger.error(f"Connection failed: {e}")
            return False

    def disconnect(self):
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def request_streams(self, streams: Optional[dict] = None):
        """Send SET_MESSAGE_INTERVAL for each desired stream."""
        if streams is None:
            # PX4 on phone: budget=40KB/s, HIGHRES_IMU~74 bytes → 100Hz max stable
            streams = {
                MSG_HIGHRES_IMU: 100,     # 100 Hz (max stable with 40KB/s budget)
                MSG_SCALED_PRESSURE: 10,  # 10 Hz
                MSG_GPS_RAW_INT: 5,       # 5 Hz (GPS hardware limit)
                MSG_ATTITUDE: 30,         # 30 Hz
            }
        for msg_id, rate_hz in streams.items():
            interval_us = int(1e6 / rate_hz) if rate_hz > 0 else -1
            pkt = build_set_message_interval(msg_id, interval_us)
            self._send(pkt)
            logger.info(f"Requested msg {msg_id} at {rate_hz} Hz")
            time.sleep(0.05)

    def request_streams_from_config(self, config: dict):
        """Parse mavlink_streams from sensor_config.yaml."""
        streams_cfg = config.get("mavlink_streams", {})
        streams = {}
        for key, val in streams_cfg.items():
            msg_id = val.get("msg_id")
            rate_hz = val.get("rate_hz", 0)
            if msg_id and rate_hz > 0:
                streams[msg_id] = rate_hz
        self.request_streams(streams)

    def clear(self):
        """Clear all recorded data."""
        self.imu_samples.clear()
        self.gps_samples.clear()
        self.baro_samples.clear()
        self.attitude_samples.clear()
        self.msg_counts.clear()

    def record(self, duration_s: float, progress_interval_s: float = 5.0,
               heartbeat_interval_s: float = 1.0):
        """Record sensor data for specified duration.

        Sends periodic GCS heartbeats to keep PX4 streaming.
        """
        if not self._sock:
            raise RuntimeError("Not connected — call connect() first")

        self._sock.settimeout(0.5)
        t_start = time.monotonic()
        t_last_hb = 0.0
        t_last_progress = 0.0

        logger.info(f"Recording for {duration_s:.0f}s ...")

        while True:
            now = time.monotonic()
            elapsed = now - t_start
            if elapsed >= duration_s:
                break

            # Heartbeat
            if now - t_last_hb >= heartbeat_interval_s:
                self._send(build_heartbeat_gcs())
                t_last_hb = now

            # Progress
            if now - t_last_progress >= progress_interval_s:
                logger.info(f"  {elapsed:.0f}/{duration_s:.0f}s — "
                            f"IMU={len(self.imu_samples)}, "
                            f"GPS={len(self.gps_samples)}, "
                            f"Baro={len(self.baro_samples)}")
                t_last_progress = now

            # Receive
            try:
                data = self._sock.recv(4096)
                if not data:
                    logger.warning("Connection closed by remote")
                    break
                self._process(data, now - self._t0)
            except socket.timeout:
                continue
            except OSError as e:
                logger.error(f"Socket error: {e}")
                break

        logger.info(f"Recording complete: {len(self.imu_samples)} IMU, "
                    f"{len(self.gps_samples)} GPS, "
                    f"{len(self.baro_samples)} Baro, "
                    f"{len(self.attitude_samples)} Attitude samples")

    def _send(self, data: bytes):
        if self._sock:
            try:
                self._sock.sendall(data)
            except OSError as e:
                logger.error(f"Send failed: {e}")

    def _process(self, data: bytes, t_wall: float):
        """Parse incoming bytes and dispatch to appropriate handlers."""
        for msg_id, payload in self._parser.feed(data):
            self.msg_counts[msg_id] += 1

            if msg_id == MSG_HIGHRES_IMU:
                s = parse_highres_imu(payload, t_wall)
                if s:
                    self.imu_samples.append(s)

            elif msg_id == MSG_RAW_IMU:
                s = parse_raw_imu(payload, t_wall)
                if s:
                    self.imu_samples.append(s)

            elif msg_id == MSG_GPS_RAW_INT:
                s = parse_gps_raw_int(payload, t_wall)
                if s:
                    self.gps_samples.append(s)

            elif msg_id == MSG_SCALED_PRESSURE:
                s = parse_scaled_pressure(payload, t_wall)
                if s:
                    self.baro_samples.append(s)

            elif msg_id == MSG_ATTITUDE:
                s = parse_attitude(payload, t_wall)
                if s:
                    self.attitude_samples.append(s)

    # ── Export ─────────────────────────────────────────────────────────────

    def save_imu_csv(self, path: Path):
        """Save IMU data to CSV."""
        import csv
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_wall_s", "t_boot_us", "ax", "ay", "az",
                        "gx", "gy", "gz", "mx", "my", "mz",
                        "abs_pressure", "temperature", "source"])
            for s in self.imu_samples:
                w.writerow([f"{s.t_wall_s:.6f}", s.t_boot_us,
                            f"{s.ax:.6f}", f"{s.ay:.6f}", f"{s.az:.6f}",
                            f"{s.gx:.6f}", f"{s.gy:.6f}", f"{s.gz:.6f}",
                            f"{s.mx:.6f}", f"{s.my:.6f}", f"{s.mz:.6f}",
                            f"{s.abs_pressure:.2f}", f"{s.temperature:.2f}",
                            s.source])
        logger.info(f"IMU CSV saved: {path} ({len(self.imu_samples)} samples)")

    def save_gps_csv(self, path: Path):
        """Save GPS data to CSV."""
        import csv
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_wall_s", "t_boot_us", "fix_type", "lat_e7", "lon_e7",
                        "alt_mm", "eph_cm", "epv_cm", "vel_cms", "cog_cdeg",
                        "satellites"])
            for s in self.gps_samples:
                w.writerow([f"{s.t_wall_s:.6f}", s.t_boot_us, s.fix_type,
                            s.lat_e7, s.lon_e7, s.alt_mm, s.eph, s.epv,
                            s.vel_cms, s.cog_cdeg, s.satellites])
        logger.info(f"GPS CSV saved: {path} ({len(self.gps_samples)} samples)")

    def save_baro_csv(self, path: Path):
        """Save barometer data to CSV."""
        import csv
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_wall_s", "t_boot_ms", "press_abs_hPa",
                        "press_diff_hPa", "temperature_C"])
            for s in self.baro_samples:
                w.writerow([f"{s.t_wall_s:.6f}", s.t_boot_ms,
                            f"{s.press_abs:.4f}", f"{s.press_diff:.4f}",
                            f"{s.temperature:.2f}"])
        logger.info(f"Baro CSV saved: {path} ({len(self.baro_samples)} samples)")

    def save_attitude_csv(self, path: Path):
        """Save attitude data to CSV."""
        import csv
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_wall_s", "t_boot_ms", "roll_rad", "pitch_rad",
                        "yaw_rad", "rollspeed", "pitchspeed", "yawspeed"])
            for s in self.attitude_samples:
                w.writerow([f"{s.t_wall_s:.6f}", s.t_boot_ms,
                            f"{s.roll:.6f}", f"{s.pitch:.6f}", f"{s.yaw:.6f}",
                            f"{s.rollspeed:.6f}", f"{s.pitchspeed:.6f}",
                            f"{s.yawspeed:.6f}"])
        logger.info(f"Attitude CSV saved: {path} ({len(self.attitude_samples)} samples)")

    def save_all(self, result_dir: Path):
        """Save all data to CSVs in result_dir."""
        result_dir.mkdir(parents=True, exist_ok=True)
        if self.imu_samples:
            self.save_imu_csv(result_dir / "sensor_imu.csv")
        if self.gps_samples:
            self.save_gps_csv(result_dir / "sensor_gps.csv")
        if self.baro_samples:
            self.save_baro_csv(result_dir / "sensor_baro.csv")
        if self.attitude_samples:
            self.save_attitude_csv(result_dir / "sensor_attitude.csv")
