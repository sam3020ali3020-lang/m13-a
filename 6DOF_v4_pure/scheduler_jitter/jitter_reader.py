"""
jitter_reader.py — MAVLink reader for scheduler jitter test.

Opens a TCP MAVLink connection, requests HIGHRES_IMU and DEBUG_FLOAT_ARRAY
streams, and records both INTERNAL PX4 timestamps (time_usec from payload)
and WALL-CLOCK arrival times.

Internal timestamps reflect PX4's actual control loop scheduling.
Wall-clock arrivals are affected by TCP buffering and are only kept for
diagnostic purposes.

This module is framework-agnostic: it exposes a single function
`run_capture(cfg, duration_s, on_progress=None)` that returns raw data
for the analysis layer to compute statistics.
"""
from __future__ import annotations

import socket
import struct
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional


# ---------------------------------------------------------------------------
# MAVLink v2 wire helpers (hand-rolled to avoid heavy pymavlink dependency)
# ---------------------------------------------------------------------------

def _x25_crc(data: bytes, extra: int) -> int:
    crc = 0xFFFF
    for b in list(data) + [extra]:
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        crc &= 0xFFFF
    return crc


def _mk_msg(msg_id: int, payload: bytes, crc_extra: int,
            sysid: int = 255, compid: int = 190) -> bytes:
    hdr = bytes([0xFD, len(payload), 0, 0, 0, sysid, compid,
                 msg_id & 0xFF,
                 (msg_id >> 8) & 0xFF,
                 (msg_id >> 16) & 0xFF])
    crc = _x25_crc(hdr[1:] + payload, crc_extra)
    return hdr + payload + struct.pack('<H', crc)


def _build_heartbeat(sysid: int = 255, compid: int = 190) -> bytes:
    # HEARTBEAT msg_id=0, crc_extra=50
    # MAV_TYPE=GCS(6), MAV_AUTOPILOT=INVALID(8), base_mode=0, custom_mode=0,
    # system_status=4(active), mavlink_version=3
    payload = struct.pack('<IBBBBB', 0, 6, 8, 0, 4, 3)
    return _mk_msg(0, payload, 50, sysid, compid)


def _build_set_interval(msg_id: int, hz: float,
                        sysid: int = 255, compid: int = 190) -> bytes:
    # COMMAND_LONG msg_id=76, crc_extra=152
    # cmd=511 MAV_CMD_SET_MESSAGE_INTERVAL
    # param1=msg_id, param2=interval_us (-1 disable, 0 default)
    interval_us = int(1e6 / hz) if hz > 0 else -1
    payload = struct.pack(
        '<fffffffHBBB',
        float(msg_id),           # param1
        float(interval_us),      # param2
        0.0, 0.0, 0.0, 0.0, 0.0,  # params 3-7
        511,                     # command
        1, 1, 0,                 # target_sys, target_comp, confirmation
    )
    return _mk_msg(76, payload, 152, sysid, compid)


# ---------------------------------------------------------------------------
# Capture data container
# ---------------------------------------------------------------------------

@dataclass
class CaptureResult:
    scenario: str
    duration_s: float
    started_at: float
    # Per-stream: parallel arrays of internal PX4 timestamps and wall-clock arrival
    # IMU: HIGHRES_IMU (msg 105)
    imu_time_usec: List[int] = field(default_factory=list)
    imu_wall: List[float] = field(default_factory=list)
    # RktGNC: DEBUG_FLOAT_ARRAY (msg 350) with array_id=2
    rkt_time_usec: List[int] = field(default_factory=list)
    rkt_wall: List[float] = field(default_factory=list)
    # Counters for other messages (diagnostic only)
    other_msg_counts: Dict[int, int] = field(default_factory=dict)
    total_msgs: int = 0


# ---------------------------------------------------------------------------
# Main capture routine
# ---------------------------------------------------------------------------

def run_capture(cfg: dict, scenario: str, duration_s: float,
                on_progress: Optional[Callable[[dict], None]] = None) -> CaptureResult:
    """Connect to PX4 via MAVLink TCP, request streams, record for duration_s.

    Args:
        cfg: config dict (parsed jitter_config.yaml).
        scenario: name of the scenario (for labeling).
        duration_s: capture duration in seconds.
        on_progress: optional callback(dict) fired ~1 Hz with progress info.

    Returns:
        CaptureResult with raw timestamps.
    """
    host = cfg['mavlink']['host']
    port = cfg['mavlink']['port']
    sysid = cfg['mavlink'].get('sysid_gcs', 255)
    compid = cfg['mavlink'].get('compid_gcs', 190)

    imu_msg_id = cfg['streams']['highres_imu']['msg_id']
    imu_rate = cfg['streams']['highres_imu']['rate_hz']
    rkt_msg_id = cfg['streams']['debug_float_array']['msg_id']
    rkt_rate = cfg['streams']['debug_float_array']['rate_hz']
    rkt_array_id = cfg['streams']['debug_float_array']['array_id']

    result = CaptureResult(
        scenario=scenario,
        duration_s=duration_s,
        started_at=time.time(),
    )

    sock = socket.socket()
    sock.settimeout(5.0)
    sock.connect((host, port))

    # Request streams
    hb = _build_heartbeat(sysid, compid)
    sock.send(_build_set_interval(imu_msg_id, imu_rate, sysid, compid))
    sock.send(_build_set_interval(rkt_msg_id, rkt_rate, sysid, compid))
    time.sleep(0.5)  # let PX4 ack

    buf = bytearray()
    t_start = time.monotonic()
    t_end = t_start + duration_s
    t_next_hb = 0.0
    t_last_progress = 0.0

    sock.settimeout(0.05)
    try:
        while time.monotonic() < t_end:
            now = time.monotonic()
            if now >= t_next_hb:
                try:
                    sock.send(hb)
                    t_next_hb = now + 0.5
                except OSError:
                    break

            try:
                data = sock.recv(16384)
                if data:
                    buf.extend(data)
            except socket.timeout:
                pass
            except OSError:
                break

            t_recv = time.monotonic()
            i = 0
            while i + 12 <= len(buf):
                if buf[i] != 0xFD:
                    i += 1
                    continue
                plen = buf[i + 1]
                msg_id = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
                end = i + 10 + plen + 2
                if end > len(buf):
                    break

                result.total_msgs += 1
                payload = bytes(buf[i + 10:i + 10 + plen])

                if msg_id == imu_msg_id and len(payload) >= 8:
                    t_us = struct.unpack_from('<Q', payload, 0)[0]
                    result.imu_time_usec.append(t_us)
                    result.imu_wall.append(t_recv)
                elif msg_id == rkt_msg_id and len(payload) >= 10:
                    # MAVLink v2 truncation: payload may be < full size. Pad.
                    padded = payload + b'\x00' * max(0, 10 - len(payload))
                    t_us = struct.unpack_from('<Q', padded, 0)[0]
                    arr_id = struct.unpack_from('<H', padded, 8)[0]
                    if arr_id == rkt_array_id:
                        result.rkt_time_usec.append(t_us)
                        result.rkt_wall.append(t_recv)

                # Track other message IDs for diagnostic context
                if msg_id not in (0, imu_msg_id, rkt_msg_id, 77, 253):
                    result.other_msg_counts[msg_id] = \
                        result.other_msg_counts.get(msg_id, 0) + 1

                i = end
            buf = buf[i:]

            if on_progress is not None and (now - t_last_progress) >= 1.0:
                t_last_progress = now
                elapsed = now - t_start
                on_progress({
                    'elapsed_s': elapsed,
                    'remaining_s': duration_s - elapsed,
                    'imu_count': len(result.imu_time_usec),
                    'imu_rate_hz': len(result.imu_time_usec) / elapsed if elapsed > 0 else 0,
                    'rkt_count': len(result.rkt_time_usec),
                    'rkt_rate_hz': len(result.rkt_time_usec) / elapsed if elapsed > 0 else 0,
                })
    finally:
        try:
            sock.close()
        except OSError:
            pass

    return result
