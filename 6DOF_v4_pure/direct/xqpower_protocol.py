"""XQPOWER CAN servo protocol — encoding/decoding فقط (no I/O).

المصدر: AndroidApp/app/src/main/cpp/PX4-Autopilot/src/drivers/xqpower_can/
  XqpowerCan.cpp — ``servo_set_position`` و ``servo_process_rx``.

الثوابت الأساسية:
  - Bitrate: 500 kbps
  - Node IDs: 0x01 .. 0x7F (عادة 0x01..0x04)
  - SDO TX (master → servo): 0x600 + node_id
  - SDO RX (servo → master): 0x580 + node_id  (يحمل ردود SDO + auto-report position)
  - PDO RX (servo → master): 0x180 + node_id  (auto-report بديل)
  - NMT:                     0x000
  - Units: 18 raw units per degree (int16)
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Optional, Tuple


# ─── Constants ──────────────────────────────────────────────────────────────

SDO_TX_BASE = 0x600
SDO_RX_BASE = 0x580
PDO_RX_BASE = 0x180
NMT_ID = 0x000

SDO_WRITE_2B = 0x22    # SDO write — يطابق XQPOWER_SDO_WRITE في XqpowerCan.hpp
SDO_WRITE_1B = 0x2F    # SDO expedited write, 1 byte
SDO_WRITE_4B = 0x23    # SDO expedited write, 4 bytes
SDO_WRITE_ACK = 0x60
SDO_ABORT = 0x80
SDO_READ_REQ = 0x40
SDO_READ_RESP_1B = 0x4F
SDO_READ_RESP_2B = 0x4B
SDO_READ_RESP_3B = 0x47
SDO_READ_RESP_4B = 0x43
SDO_READ_RESP_VAR = 0x42

# object dictionary indices used by the servo
OD_TARGET_POSITION = 0x6003    # write: target angle (raw units)
OD_ACTUAL_POSITION = 0x6002    # read:  current angle (raw units)
OD_STATUS_TEMP_CURR = 0x6005   # read:  temperature + current
OD_REPORT_INTERVAL = 0x2200    # write: auto-report period (ms, 1 byte)
OD_ZERO_CALIBRATE = 0x3009     # write: 1 to calibrate current pos as 0

UNITS_PER_DEG = 18.0

# SDO response bytes that XQPOWER uses to mark a frame as SDO (not auto-report)
SDO_RESPONSE_CMDS = {0x42, 0x43, 0x47, 0x4B, 0x4F, 0x60, 0x80}


# ─── Encoders ───────────────────────────────────────────────────────────────

def encode_set_position(node_id: int, angle_deg: float,
                        angle_limit_deg: float = 20.0,
                        units_per_deg: float = UNITS_PER_DEG) -> Tuple[int, bytes]:
    """Build SDO write frame لضبط ``TARGET_POSITION`` = angle_deg.

    يطابق تماماً ``XqpowerCan::servo_set_position`` في PX4.
    يُعيد (arb_id, data[8]).
    """
    a = max(min(float(angle_deg), angle_limit_deg), -angle_limit_deg)
    raw = int(round(a * units_per_deg))
    if raw < -32768:
        raw = -32768
    elif raw > 32767:
        raw = 32767
    lo = raw & 0xFF
    hi = (raw >> 8) & 0xFF
    data = bytes([SDO_WRITE_2B, 0x03, 0x60, 0x00, lo, hi, 0x00, 0x00])
    return SDO_TX_BASE + (node_id & 0x7F), data


def encode_read_position(node_id: int) -> Tuple[int, bytes]:
    """SDO read request: ``ACTUAL_POSITION`` (0x6002)."""
    data = bytes([SDO_READ_REQ, 0x02, 0x60, 0x00, 0, 0, 0, 0])
    return SDO_TX_BASE + (node_id & 0x7F), data


def encode_set_report_interval(node_id: int, interval_ms: int) -> Tuple[int, bytes]:
    """SDO write interval (1-byte) لتفعيل auto-report كل N ms."""
    iv = max(10, min(255, int(interval_ms)))
    data = bytes([SDO_WRITE_1B, 0x00, 0x22, 0x00, iv, 0, 0, 0])
    return SDO_TX_BASE + (node_id & 0x7F), data


def encode_zero_calibrate(node_id: int) -> Tuple[int, bytes]:
    """SDO write: اضبط الموضع الحالي كصفر جديد (يُحفظ في flash)."""
    data = bytes([SDO_WRITE_4B, 0x09, 0x30, 0x00, 0x01, 0x00, 0x00, 0x00])
    return SDO_TX_BASE + (node_id & 0x7F), data


def encode_nmt_start(node_id: int) -> Tuple[int, bytes]:
    """NMT Start Node — يُخرج السيرفو من Pre-Op إلى Operational."""
    return NMT_ID, bytes([0x01, node_id & 0x7F])


def encode_nmt_stop(node_id: int) -> Tuple[int, bytes]:
    return NMT_ID, bytes([0x02, node_id & 0x7F])


def encode_nmt_reset(node_id: int) -> Tuple[int, bytes]:
    return NMT_ID, bytes([0x81, node_id & 0x7F])


# ─── Decoders ───────────────────────────────────────────────────────────────

@dataclass
class ServoReport:
    """Decoded servo feedback from SDO-RX or PDO-RX frame."""
    node_id: int
    position_deg: Optional[float] = None
    current_mA: Optional[int] = None
    temperature_C: Optional[float] = None
    is_sdo_ack: bool = False
    is_sdo_abort: bool = False
    abort_code: Optional[int] = None
    raw_idx: Optional[int] = None          # OD index (for SDO responses)


def _raw_to_deg(raw: int, units_per_deg: float = UNITS_PER_DEG) -> float:
    return float(raw) / units_per_deg


def decode_frame(arb_id: int, data: bytes,
                 units_per_deg: float = UNITS_PER_DEG) -> Optional[ServoReport]:
    """Parse frame من السيرفو إلى ``ServoReport``.

    يُعيد None إن لم يكن الإطار من سيرفو XQPOWER.
    """
    if len(data) < 2:
        return None

    # SDO-RX range (0x581 .. 0x5FF)
    if SDO_RX_BASE + 1 <= arb_id <= SDO_RX_BASE + 0x7F:
        node_id = arb_id - SDO_RX_BASE
        return _decode_on_sdo_rx(node_id, data, units_per_deg)

    # PDO-RX range (0x181 .. 0x1FF)
    if PDO_RX_BASE + 1 <= arb_id <= PDO_RX_BASE + 0x7F:
        node_id = arb_id - PDO_RX_BASE
        return _decode_pdo(node_id, data, units_per_deg)

    return None


def _decode_on_sdo_rx(node_id: int, data: bytes,
                      units_per_deg: float) -> ServoReport:
    """Frame على 0x580+node. قد يكون SDO response أو auto-report.

    نميّز بـ data[0]: إن كان من SDO_RESPONSE_CMDS → SDO، وإلا auto-report.
    """
    cmd = data[0]
    if cmd in SDO_RESPONSE_CMDS and len(data) >= 4:
        idx = data[1] | (data[2] << 8)
        rpt = ServoReport(node_id=node_id, raw_idx=idx)
        if cmd == SDO_WRITE_ACK:
            rpt.is_sdo_ack = True
        elif cmd == SDO_ABORT:
            rpt.is_sdo_abort = True
            if len(data) >= 8:
                rpt.abort_code = (
                    data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24)
                )
        elif cmd in (SDO_READ_RESP_1B, SDO_READ_RESP_2B,
                     SDO_READ_RESP_3B, SDO_READ_RESP_4B, SDO_READ_RESP_VAR):
            # read response
            if idx == OD_ACTUAL_POSITION and len(data) >= 6:
                raw = struct.unpack("<h", bytes(data[4:6]))[0]
                rpt.position_deg = _raw_to_deg(raw, units_per_deg)
            elif idx == OD_STATUS_TEMP_CURR and len(data) >= 7:
                rpt.current_mA = data[4] | (data[5] << 8)
                rpt.temperature_C = float(data[6])
        return rpt

    # auto-report على نفس ID: first 2 bytes = raw position
    if len(data) >= 2:
        raw = struct.unpack("<h", bytes(data[:2]))[0]
        # نفس حد الصحة في الـ driver الأصلي
        if -800 <= raw <= 800:
            return ServoReport(node_id=node_id,
                               position_deg=_raw_to_deg(raw, units_per_deg))
    return ServoReport(node_id=node_id)


def _decode_pdo(node_id: int, data: bytes, units_per_deg: float) -> ServoReport:
    """Frame على 0x180+node = CANopen PDO. أول 2 بايت = raw position."""
    if len(data) < 2:
        return ServoReport(node_id=node_id)
    raw = struct.unpack("<h", bytes(data[:2]))[0]
    return ServoReport(node_id=node_id,
                       position_deg=_raw_to_deg(raw, units_per_deg))
