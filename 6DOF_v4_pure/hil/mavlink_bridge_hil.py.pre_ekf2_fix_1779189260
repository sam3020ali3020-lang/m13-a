#!/usr/bin/env python3
"""
HIL MAVLink Bridge — جسر مستقل لاختبار Hardware-In-the-Loop
================================================================

مستقل تماماً عن جسر SITL. مخصّص للاتصال بجهاز ARM64 خارجي (Android/Jetson)
عبر TCP على واجهة شبكة حقيقية.

الاختلافات الجوهرية عن جسر SITL:
  • يستمع على 0.0.0.0 (أو أي IP) بدل 127.0.0.1
  • وضع realtime فقط — لا lockstep عبر الشبكة (jitter غير مقبول)
  • لا توجد عمليّات ARM عبر UDP محلي — الهدف يُهيّأ مسبقاً
  • يقرأ رسائل التوقيت DEBUG_VECT ("TIMING") / NAMED_VALUE_FLOAT
    ويحفظها في hil_timing.csv
  • تهيئة مبسّطة للإقلاع (warm-up أقصر، مناسب لعتاد فعلي جاهز)
"""

from __future__ import annotations

import argparse
import csv
import itertools
import logging
import os
import socket
import struct
import sys
import threading
import time
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import yaml

_SCRIPT_DIR = Path(__file__).resolve().parent
_SIM_DIR = _SCRIPT_DIR.parent
sys.path.insert(0, str(_SIM_DIR))

from rocket_6dof_sim import Rocket6DOFSimulation  # noqa: E402

logger = logging.getLogger("hil_bridge")


# ============================================================================
# ثوابت MAVLink v2 (الحد الأدنى — لا تبعية على pymavlink)
# ============================================================================

MAVLINK_STX_V2 = 0xFD
MAVLINK_HEADER_LEN_V2 = 10
MAVLINK_CHECKSUM_LEN = 2

MSG_HEARTBEAT = 0
MSG_HIL_SENSOR = 107
MSG_HIL_GPS = 113
MSG_HIL_STATE_QUATERNION = 115
MSG_HIL_ACTUATOR_CONTROLS = 93
MSG_NAMED_VALUE_FLOAT = 251
MSG_DEBUG_VECT = 250
MSG_DEBUG_FLOAT_ARRAY = 350  # SRV_FB من xqpower_can
MSG_COMMAND_LONG = 76
MSG_COMMAND_ACK = 77
MSG_PARAM_REQUEST_READ = 20
MSG_PARAM_SET = 23
MSG_PARAM_VALUE = 22

CRC_HEARTBEAT = 50
CRC_HIL_SENSOR = 108
CRC_HIL_GPS = 124
CRC_HIL_STATE_QUATERNION = 4
CRC_COMMAND_LONG = 152
CRC_PARAM_REQUEST_READ = 214
CRC_PARAM_SET = 168
CRC_PARAM_VALUE = 220

MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_DO_FLIGHTTERMINATION = 185
MAV_PARAM_TYPE_INT32 = 6
MAV_PARAM_TYPE_REAL32 = 9

SYS_ID = 1
COMP_ID = 1
GCS_SYS_ID = 255
GCS_COMP_ID = 190


def _x25_crc(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        crc &= 0xFFFF
    return crc


# عدّاد تسلسل MAVLink مشترك (thread-safe) — خيط timing على 5760 يُرسل
# heartbeats بالتوازي مع الخيط الرئيسي على 4560.
_seq_lock = threading.Lock()
_seq_iter = itertools.count()


def _pack_v2(msg_id: int, payload: bytes, crc_extra: int,
             sys_id: int = SYS_ID, comp_id: int = COMP_ID) -> bytes:
    trimmed = payload.rstrip(b"\x00")
    if not trimmed:
        trimmed = b"\x00"
    tlen = len(trimmed)
    with _seq_lock:
        seq_num = next(_seq_iter) & 0xFF
    header = struct.pack(
        "<BBBBBBBHB",
        MAVLINK_STX_V2, tlen, 0, 0, seq_num,
        sys_id, comp_id,
        msg_id & 0xFFFF, (msg_id >> 16) & 0xFF,
    )
    crc = _x25_crc(header[1:] + trimmed)
    crc = _x25_crc(bytes([crc_extra]), crc)
    return header + trimmed + struct.pack("<H", crc)


# ─── Builders ────────────────────────────────────────────────────────────────

def build_heartbeat() -> bytes:
    payload = struct.pack("<IBBBBB", 0, 6, 8, 0, 4, 3)
    return _pack_v2(MSG_HEARTBEAT, payload, CRC_HEARTBEAT,
                    sys_id=GCS_SYS_ID, comp_id=GCS_COMP_ID)


def build_command_long(command: int, target_sys: int = 1, target_comp: int = 1,
                       confirmation: int = 0,
                       p1=0.0, p2=0.0, p3=0.0, p4=0.0,
                       p5=0.0, p6=0.0, p7=0.0) -> bytes:
    payload = struct.pack("<7fHBBB",
                          p1, p2, p3, p4, p5, p6, p7,
                          command,
                          target_sys, target_comp, confirmation)
    return _pack_v2(MSG_COMMAND_LONG, payload, CRC_COMMAND_LONG,
                    sys_id=GCS_SYS_ID, comp_id=GCS_COMP_ID)


def build_param_request_read(param_id: str,
                              target_sys: int = 1,
                              target_comp: int = 1) -> bytes:
    """PARAM_REQUEST_READ (msg 20): يطلب قيمة param واحد من PX4.
    الحقول: param_index(h) + target_sys(B) + target_comp(B) + param_id[16].
    نُمرّر ``param_index=-1`` لإجبار PX4 على البحث بالاسم.
    الاستجابة هي PARAM_VALUE على نفس القناة (5760 mavlink العادي)."""
    name = param_id.encode("ascii")[:16].ljust(16, b"\x00")
    payload = struct.pack("<hBB16s", -1, target_sys, target_comp, name)
    return _pack_v2(MSG_PARAM_REQUEST_READ, payload, CRC_PARAM_REQUEST_READ,
                    sys_id=GCS_SYS_ID, comp_id=GCS_COMP_ID)


def build_param_set(param_id: str, value: float,
                    param_type: int = MAV_PARAM_TYPE_INT32,
                    target_sys: int = 1, target_comp: int = 1) -> bytes:
    """PARAM_SET (msg 23): param_value(f) + target_sys(B) + target_comp(B) +
    param_id[16](s) + param_type(B). param_value must encode based on type."""
    if param_type == MAV_PARAM_TYPE_INT32:
        raw = struct.pack("<i", int(value))
        value_f = struct.unpack("<f", raw)[0]
    else:
        value_f = float(value)
    name = param_id.encode("ascii")[:16].ljust(16, b"\x00")
    payload = struct.pack("<fBB16sB", value_f, target_sys, target_comp,
                          name, param_type)
    return _pack_v2(MSG_PARAM_SET, payload, CRC_PARAM_SET,
                    sys_id=GCS_SYS_ID, comp_id=GCS_COMP_ID)


def parse_param_value(payload: bytes) -> Optional[dict]:
    """PARAM_VALUE (msg 22): param_value(f) + param_count(H) + param_index(H) +
    param_id[16](s) + param_type(B). v2 truncation may shrink trailing zeros."""
    if len(payload) < 9:
        return None
    if len(payload) < 25:
        payload = payload + b"\x00" * (25 - len(payload))
    try:
        v_f, cnt, idx = struct.unpack("<fHH", payload[:8])
        name = payload[8:24].rstrip(b"\x00").decode("ascii", "replace")
        ptype = payload[24]
        if ptype == MAV_PARAM_TYPE_INT32:
            value = struct.unpack("<i", struct.pack("<f", v_f))[0]
        else:
            value = float(v_f)
        return {"name": name, "value": value, "type": int(ptype),
                "count": int(cnt), "index": int(idx)}
    except (struct.error, UnicodeDecodeError):
        return None


# حدود حقول MAVLink المُقاسة (scaled ints). تجاوزها يسبّب `struct.error`
# يقتل الجسر ويُضيع كل بيانات الرحلة. لذلك نقصّ (clamp) بدل التفجير.
_INT16_MIN = -32768
_INT16_MAX = 32767
_UINT16_MAX = 65535


def _clip_scaled(val: float, lo: int, hi: int, name: str,
                 _warned: dict = {}) -> int:
    """قصّ قيمة مُقاسة على حدود العدد الصحيح مع تسجيل تحذير مرة واحدة لكل حقل.

    النطاقات الشائعة:
      • int16   : [-32768, 32767]  → سرعة cm/s ≤ ±327.67 m/s، تسارع mG ≤ ±32.767 g
      • uint16  : [0, 65535]       → airspeed cm/s ≤ 655.35 m/s
    """
    v = int(val)
    if v < lo or v > hi:
        if name not in _warned:
            print(f"[HIL] WARNING: MAVLink field {name!r} saturated "
                  f"({v} out of [{lo},{hi}]) — clamping. "
                  "السرعة/التسارع يتجاوز نطاق MAVLink scaled int.")
            _warned[name] = True
        v = max(lo, min(hi, v))
    return v


def build_hil_sensor(t_us: int, accel, gyro, mag,
                     abs_p: float, diff_p: float, alt: float,
                     temp: float = 25.0, fields: int = 0x1FFF) -> bytes:
    payload = struct.pack(
        "<Q3f3f3fffffIB",
        t_us,
        accel[0], accel[1], accel[2],
        gyro[0], gyro[1], gyro[2],
        mag[0], mag[1], mag[2],
        abs_p, diff_p, alt, temp,
        fields, 0,
    )
    return _pack_v2(MSG_HIL_SENSOR, payload, CRC_HIL_SENSOR)


def build_hil_gps(t_us: int, lat, lon, alt_m, vn, ve, vd) -> bytes:
    # MAVLink v2 wire order for HIL_GPS (msg 113) is size-sorted, NOT XML
    # declaration order. Correct layout (non-extension, 36 bytes):
    #   time_usec(u64) + lat(i32) + lon(i32) + alt(i32) +
    #   eph(u16) + epv(u16) + vel(u16) + vn(i16) + ve(i16) + vd(i16) +
    #   cog(u16) + fix_type(u8) + satellites_visible(u8)
    # The previous layout placed fix_type right after time_usec, which
    # shifted every subsequent field by one byte and caused PX4 to decode
    # corrupted lat/lon/fix_type (fix_type ended up as the high byte of cog,
    # typically 0 → NO_FIX). Verified against pymavlink's canonical
    # HIL_GPS unpacker `<QiiiHHHhhhHBB`.
    lat_e7 = int(lat * 1e7)
    lon_e7 = int(lon * 1e7)
    alt_mm = int(alt_m * 1000)
    vel_cm = _clip_scaled(np.sqrt(vn * vn + ve * ve) * 100,
                          0, _UINT16_MAX, "hil_gps.vel")
    cog = int(np.degrees(np.arctan2(ve, vn)) * 100) % 36000
    payload = struct.pack(
        "<QiiiHHHhhhHBB",
        t_us, lat_e7, lon_e7, alt_mm,
        250, 400, vel_cm,
        _clip_scaled(vn * 100, _INT16_MIN, _INT16_MAX, "hil_gps.vn"),
        _clip_scaled(ve * 100, _INT16_MIN, _INT16_MAX, "hil_gps.ve"),
        _clip_scaled(vd * 100, _INT16_MIN, _INT16_MAX, "hil_gps.vd"),
        cog, 3, 12,
    )
    return _pack_v2(MSG_HIL_GPS, payload, CRC_HIL_GPS)


def build_hil_state_quat(t_us: int, quat, omega,
                          lat, lon, alt_m, vn, ve, vd,
                          accel_body, airspeed: float = 0.0) -> bytes:
    # ملاحظة: HIL_STATE_QUATERNION في PX4 يُنشر كـ ground-truth فقط
    # (لا يُغذّي EKF2)، لذا قصّ الحقول هنا آمن ولا يؤثر على تقدير الحالة.
    payload = struct.pack(
        "<Q4f3fiiihhhHHhhh",
        t_us,
        quat[0], quat[1], quat[2], quat[3],
        omega[0], omega[1], omega[2],
        int(lat * 1e7), int(lon * 1e7), int(alt_m * 1000),
        _clip_scaled(vn * 100, _INT16_MIN, _INT16_MAX, "hil_state.vn"),
        _clip_scaled(ve * 100, _INT16_MIN, _INT16_MAX, "hil_state.ve"),
        _clip_scaled(vd * 100, _INT16_MIN, _INT16_MAX, "hil_state.vd"),
        _clip_scaled(airspeed * 100, 0, _UINT16_MAX, "hil_state.airspeed"),
        0,
        _clip_scaled(accel_body[0] / 9.80665 * 1000,
                     _INT16_MIN, _INT16_MAX, "hil_state.xacc"),
        _clip_scaled(accel_body[1] / 9.80665 * 1000,
                     _INT16_MIN, _INT16_MAX, "hil_state.yacc"),
        _clip_scaled(accel_body[2] / 9.80665 * 1000,
                     _INT16_MIN, _INT16_MAX, "hil_state.zacc"),
    )
    return _pack_v2(MSG_HIL_STATE_QUATERNION, payload, CRC_HIL_STATE_QUATERNION)


# ─── Parsers ─────────────────────────────────────────────────────────────────

def parse_actuator_controls(payload: bytes) -> Optional[dict]:
    """HIL_ACTUATOR_CONTROLS (msg 93): time_usec(u64) + flags(u64) + 16 floats + mode(u8).

    MAVLink v2 قد يقلّص الأصفار اللاحقة على حدود بايت (ليس حقل)، لذلك
    نُبَطِّن إلى الحجم الكامل (81 بايت) قبل فك التحزيم. الإصدار السابق كان
    يعتمد على fallback يحسب ``n = (len-16)//4`` ويتجاهل bytes تقصير غير
    مُحاذية (مثل تقصير وسط float)، فكان يعيد قيم أصفار مزيّفة بدلاً من
    القيم الحقيقية. باستخدام تبطين صريح نقرأ الصيغة الكاملة دائماً.
    """
    if len(payload) < 16:
        return None
    FULL = 81
    if len(payload) < FULL:
        payload = payload + b"\x00" * (FULL - len(payload))
    try:
        v = struct.unpack("<QQ16fB", payload[:FULL])
        return {"t_us": v[0], "controls": list(v[2:18])}
    except struct.error:
        return None


def parse_named_value_float(payload: bytes) -> Optional[dict]:
    """NAMED_VALUE_FLOAT (msg 251): time_boot_ms(u32) + value(f32) + name[10].
    MAVLink v2 قد يقلّص الأصفار اللاحقة."""
    if len(payload) < 8:
        return None
    if len(payload) < 18:
        payload = payload + b"\x00" * (18 - len(payload))
    try:
        t_ms, val = struct.unpack("<If", payload[:8])
        name = payload[8:18].rstrip(b"\x00").decode("ascii", "replace")
        return {"t_ms": t_ms, "name": name, "value": float(val)}
    except (struct.error, UnicodeDecodeError):
        return None


def parse_debug_vect(payload: bytes) -> Optional[dict]:
    """DEBUG_VECT (msg 250): name[10] + time_usec(u64) + x(f32) + y(f32) + z(f32).
    MAVLink v2 يحذف الأصفار اللاحقة، لذا قد يكون payload < 30 بايت."""
    if len(payload) < 20:
        return None
    # pad to full 30 bytes (trailing zeros restored)
    if len(payload) < 30:
        payload = payload + b"\x00" * (30 - len(payload))
    try:
        t_us, x, y, z = struct.unpack("<Qfff", payload[:20])
        name = payload[20:30].rstrip(b"\x00").decode("ascii", "replace")
        return {"t_us": t_us, "name": name, "x": float(x), "y": float(y), "z": float(z)}
    except (struct.error, UnicodeDecodeError):
        return None


def parse_debug_float_array(payload: bytes) -> Optional[dict]:
    """DEBUG_FLOAT_ARRAY (msg 350):
        time_usec(u64) + data[58](f32) + array_id(u16) + name[10]
    الحجم الكامل = 252 بايت. MAVLink v2 يقلّص الأصفار اللاحقة.
    يُستخدم لنقل SRV_FB من درايفر xqpower_can:
        data[0..3]  = cmd_deg  (الأمر من Control Allocator)
        data[4..7]  = position_deg (الزاوية الحقيقية من السيرفو عبر CAN)
        data[8..11] = error_deg
        data[12]    = online_mask (bit-mask)
        data[13]    = tx_fail_count
    """
    FULL = 252
    if len(payload) < 8:
        return None
    if len(payload) < FULL:
        payload = payload + b"\x00" * (FULL - len(payload))
    try:
        # MAVLink field order (big→small for non-extensions, then extensions):
        #   time_usec(u64) + array_id(u16) + name[10] + data[58] (extension)
        t_us = struct.unpack("<Q", payload[:8])[0]
        array_id = struct.unpack("<H", payload[8:10])[0]
        name = payload[10:20].rstrip(b"\x00").decode("ascii", "replace")
        data = list(struct.unpack("<58f", payload[20:20 + 232]))
        return {"t_us": int(t_us), "array_id": int(array_id),
                "name": name, "data": data}
    except (struct.error, UnicodeDecodeError):
        return None



class MavParser:
    def __init__(self):
        self._buf = bytearray()

    def feed(self, data: bytes):
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
# أدوات مساعدة
# ============================================================================

def _ned_to_lla(pos, lat0, lon0, alt0):
    lat = lat0 + np.degrees(pos[0] / 6371000.0)
    lon = lon0 + np.degrees(pos[1] / (6371000.0 * np.cos(np.radians(lat0))))
    alt = alt0 - pos[2]
    return lat, lon, alt


def _body_specific_force(f_body, mass, g_ned, quat):
    q0, q1, q2, q3 = quat
    C = np.array([
        [1 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 + q0 * q3), 2 * (q1 * q3 - q0 * q2)],
        [2 * (q1 * q2 - q0 * q3), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 + q0 * q1)],
        [2 * (q1 * q3 + q0 * q2), 2 * (q2 * q3 - q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2)],
    ])
    return f_body / max(mass, 0.1) - C @ g_ned


def _mag_body(lat_deg, quat):
    inc = np.radians(25.0 + 0.5 * (lat_deg - 16.0))
    B_ned = np.array([0.32 * np.cos(inc), 0.0, 0.32 * np.sin(inc)])
    q0, q1, q2, q3 = quat
    C = np.array([
        [1 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 + q0 * q3), 2 * (q1 * q3 - q0 * q2)],
        [2 * (q1 * q2 - q0 * q3), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 + q0 * q1)],
        [2 * (q1 * q3 + q0 * q2), 2 * (q2 * q3 - q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2)],
    ])
    return C @ B_ned


# ============================================================================
# مولّد ضجيج الحسّاسات
# ============================================================================
# نسخة مستقلة (self-contained) من ``sitl.mavlink_bridge.SensorNoise``.
# الغرض: جعل HIL يختبر EKF2 مع ضجيج واقعي من bridge.noise في الـ config
# بدل ضجيج hardcoded مثالي — كان السبب الأساسي لعدم كشف مشاكل biases/drift
# في runs سابقة. كل الخيارات تُقرأ من noise_cfg dict؛ غير المُعرَّف يرجع
# لقيم افتراضية تُطابق SITL تماماً (نفس الأرقام الحرفية).
# يُفضَّل نسخ الصف بدل import من sitl لأن ملف HIL موثَّق في header أنه
# "مستقل تماماً عن جسر SITL".


class SensorNoise:
    """Add realistic sensor noise to simulation outputs (mirrors SITL)."""

    def __init__(self, config: dict, rng_seed: int = 42):
        self.rng = np.random.default_rng(rng_seed)
        noise_cfg = config.get('noise', {})
        self.accel_std = noise_cfg.get('accel_std', 0.35)
        self.gyro_std = noise_cfg.get('gyro_std', 0.005)
        self.gps_pos_std = noise_cfg.get('gps_pos_std', 2.5)
        self.gps_vel_std = noise_cfg.get('gps_vel_std', 0.1)
        self.gps_delay_ms = noise_cfg.get('gps_delay_ms', 100)
        self.mag_std = noise_cfg.get('mag_std', 0.005)
        self.baro_std = noise_cfg.get('baro_std', 0.5)

        # Multipliers for high-noise robustness testing
        if noise_cfg.get('high_noise_enabled', False):
            self.accel_std *= noise_cfg.get('high_noise_accel_multiplier', 3.0)
            self.gyro_std *= noise_cfg.get('high_noise_gyro_multiplier', 3.0)

        # Fixed IMU biases (constant per run) — تُحسب مرة واحدة عند __init__
        # لذا أول استدعاء _sensors يحافظ على نفس bias طوال الرحلة.
        accel_bias_std = noise_cfg.get('accel_bias_std', 0.05)
        gyro_bias_std = noise_cfg.get('gyro_bias_std', 0.001)
        self.accel_bias = self.rng.normal(0, accel_bias_std, 3)  # m/s²
        self.gyro_bias = self.rng.normal(0, gyro_bias_std, 3)   # rad/s

        # GPS delay buffer
        self._gps_buffer: list = []

    def add_imu_noise(self, accel: np.ndarray, gyro: np.ndarray
                      ) -> Tuple[np.ndarray, np.ndarray]:
        """Add noise + bias to IMU readings."""
        noisy_accel = accel + self.accel_bias + self.rng.normal(0, self.accel_std, 3)
        noisy_gyro = gyro + self.gyro_bias + self.rng.normal(0, self.gyro_std, 3)
        return noisy_accel, noisy_gyro

    def add_gps_noise(self, lat: float, lon: float, alt: float,
                      vn: float, ve: float, vd: float, t_usec: int
                      ) -> Optional[Tuple[float, float, float, float, float, float]]:
        """Add noise + delay to GPS. Returns None if delayed data not ready."""
        noisy_lat = lat + self.rng.normal(0, self.gps_pos_std / 111320.0)
        noisy_lon = lon + self.rng.normal(
            0, self.gps_pos_std / (111320.0 * max(np.cos(np.radians(lat)), 0.01))
        )
        noisy_alt = alt + self.rng.normal(0, self.gps_pos_std)
        noisy_vn = vn + self.rng.normal(0, self.gps_vel_std)
        noisy_ve = ve + self.rng.normal(0, self.gps_vel_std)
        noisy_vd = vd + self.rng.normal(0, self.gps_vel_std)

        self._gps_buffer.append((t_usec, noisy_lat, noisy_lon, noisy_alt,
                                  noisy_vn, noisy_ve, noisy_vd))

        delay_usec = self.gps_delay_ms * 1000
        while len(self._gps_buffer) > 1:
            if t_usec - self._gps_buffer[0][0] >= delay_usec:
                _, lat_d, lon_d, alt_d, vn_d, ve_d, vd_d = self._gps_buffer.pop(0)
                return lat_d, lon_d, alt_d, vn_d, ve_d, vd_d
            break

        if len(self._gps_buffer) == 1 and delay_usec == 0:
            _, lat_d, lon_d, alt_d, vn_d, ve_d, vd_d = self._gps_buffer.pop(0)
            return lat_d, lon_d, alt_d, vn_d, ve_d, vd_d

        return None

    def add_mag_noise(self, mag: np.ndarray) -> np.ndarray:
        return mag + self.rng.normal(0, self.mag_std, 3)

    def add_baro_noise(self, alt: float) -> float:
        return alt + self.rng.normal(0, self.baro_std)


# ============================================================================
# جسر HIL
# ============================================================================

class HILBridge:
    """جسر MAVLink HIL بين محاكاة 6DOF على PC وجهاز PX4 بعيد على ARM64."""

    def __init__(self, cfg_path: str):
        with open(cfg_path, "r", encoding="utf-8") as f:
            self.cfg = yaml.safe_load(f)

        tgt = self.cfg.get("target", {})
        tcp = tgt.get("tcp", {})
        self.host = tcp.get("host", "0.0.0.0")
        self.port = int(tcp.get("port", 4560))
        self.accept_timeout_s = float(tcp.get("timeout_s", 60.0))

        scenario = self.cfg.get("scenario", {})
        sim_cfg_path = scenario.get("sim_config", "config/6dof_config_advanced.yaml")
        if not os.path.isabs(sim_cfg_path):
            sim_cfg_path = str(_SIM_DIR / sim_cfg_path)
        # NOTE: dt/duration هنا SoT الوحيد لـ HIL bridge. قيم simulation.dt و
        # simulation.duration في 6dof_config_advanced.yaml تُتجاهَل (على عكس
        # Standalone/SITL/PIL). لو أردت تغيير خطوة التكامل أو مدة الرحلة،
        # عدّل hil_config.yaml scenario.dt_s / scenario.duration_s فقط.
        self.duration = float(scenario.get("duration_s", 500.0))
        self.dt = float(scenario.get("dt_s", 0.01))
        self.seed = int(scenario.get("random_seed", 42))

        timing_cfg = self.cfg.get("timing", {})
        self.timing_enabled = bool(timing_cfg.get("enabled", True))
        self.deadline_us = int(timing_cfg.get("deadline_us", 20000))

        # ─── إعدادات التسليح والـ warm-up ─────────────────────────────
        warmup = self.cfg.get("warmup", {})
        self.warmup_timeout_s = float(warmup.get("timeout_s", 60.0))
        self.warmup_arm_every_s = float(warmup.get("arm_every_s", 1.0))
        self.warmup_settle_s = float(warmup.get("settle_after_arm_s", 2.0))
        self.warmup_max_s = float(warmup.get("max_s", 30.0))
        self.set_hitl_param = bool(warmup.get("set_hitl_param", True))
        self.clear_flight_termination = bool(
            warmup.get("clear_flight_termination", True)
        )
        # إذا لم يصل أي HIL_ACTUATOR_CONTROLS خلال warm-up، افشل صراحة
        # (PX4 على الأرجح ليس في HIL mode — راجع _run_impl لرسالة التعليمات).
        self.abort_on_no_actuator = bool(
            warmup.get("abort_on_no_actuator", True)
        )

        # ─── إعدادات فيدباك السيرفو ──────────────────────────────────────
        # HIL يعمل في closed_loop فقط: الزاوية المقاسة من السيرفوهات
        # الحقيقية (SRV_FB عبر DEBUG_FLOAT_ARRAY) تُحقن مباشرة في
        # الأيروديناميكا، ونموذج Python السيرفو مُعطَّل لتفادي مضاعفة
        # التأخير. هذا يكشف backlash/slew/CAN-latency/jitter في سلوك
        # العتاد الفعلي على الكتلة الهوائية المحاكاة.
        hil_cfg = self.cfg.get("hil", {})
        mode_raw = hil_cfg.get("mode")
        if mode_raw is not None and str(mode_raw).strip().lower() != "closed_loop":
            raise ValueError(
                f"hil.mode must be 'closed_loop' or omitted "
                f"(monitor_only has been removed); got {mode_raw!r}"
            )
        # SRV_FB إلزامي — لا معنى لـ HIL بدون فيدباك العتاد.
        self.use_servo_feedback = True
        self.servo_feedback_timeout_ms = float(
            hil_cfg.get("servo_feedback_timeout_ms", 200.0)
        )
        # إذا شاخ الفيدباك أكثر من abort_ms → إيقاف المحاكاة
        # (اختبار bench، لا نُشغّل flight termination).
        self.servo_feedback_abort_ms = float(
            hil_cfg.get("servo_feedback_abort_ms", 500.0)
        )
        # فترة سماح في بداية الطيران (قبل وصول أول فيدباك): نسمح
        # بالرجوع إلى أمر MPC بدلاً من ABORT فوري في الخطوات الأولى.
        self.servo_feedback_grace_ms = float(
            hil_cfg.get("servo_feedback_grace_ms", 500.0)
        )
        self.require_all_servos_online = bool(
            hil_cfg.get("require_all_servos_online", False)
        )
        # ─── حارس أمان: خطأ تتبع السيرفو (راجع hil_config.yaml) ────────
        # default = 0 (معطَّل). الحارس redundant مع online_mask +
        # servo_feedback_abort_ms + tracking_mae postrun، ويُولّد
        # false-positive بسبب lookahead+transport delay ≈ 20° في boost.
        # ارفعه فقط للتشخيص اليدوي.
        self._servo_abort_rad = float(np.deg2rad(
            hil_cfg.get("servo_tracking_abort_deg", 0.0)
        ))
        self.servo_safety_breach_limit = int(
            hil_cfg.get("servo_tracking_breach_steps", 50)
        )
        self._servo_safety_breach = 0
        self.servo_auto_zero = bool(hil_cfg.get("servo_auto_zero", True))
        self.servo_zero_settle_s = float(hil_cfg.get("servo_zero_settle_s", 4.0))
        self.servo_zero_sample_s = float(hil_cfg.get("servo_zero_sample_s", 2.0))
        self.servo_zero_max_deg = float(hil_cfg.get("servo_zero_max_deg", 10.0))

        # ─── Hybrid Lockstep ─────────────────────────────────────────────
        # انتظر HIL_ACTUATOR_CONTROLS من PX4 قبل التقدم في sim step التالي.
        # يُزيل TCP/scheduler jitter من الحلقة الرقمية بينما يُبقي wall-clock
        # pacing للسيرفو الفيزيائي. راجع hil_config.yaml للتفاصيل.
        self._lockstep = bool(hil_cfg.get("lockstep", True))
        self._lockstep_timeout_s = float(
            hil_cfg.get("lockstep_timeout_ms", 20.0)
        ) / 1000.0
        self._lockstep_warn_threshold = float(
            hil_cfg.get("lockstep_warn_threshold", 0.05)
        )
        # عدّادات إحصائية للخطوات التي انتظرت ACK بنجاح vs. timed out.
        self._lockstep_ack_count = 0
        self._lockstep_timeout_count = 0
        self._lockstep_wait_us_sum = 0      # إجمالي زمن الانتظار (للمتوسط)
        self._lockstep_wait_us_max = 0

        # تحميل إعدادات المحاكاة وضبط وضع "none" (لا تحكم داخلي)
        with open(sim_cfg_path, "r", encoding="utf-8") as f:
            sim_cfg = yaml.safe_load(f)
        sim_cfg["simulation"]["control_type"] = "none"
        sim_cfg.setdefault("estimation", {})["mode"] = "off"
        # Respect initial_conditions.angular_velocity from config.
        # Previously this was forced to zero which silently cancelled any
        # yaw/pitch-rate disturbance tests on real hardware runs.

        # مسار الديناميكا: زاوية السيرفو المقاسة (SRV_FB) → المحاكاة
        # مباشرة. نُعطّل نموذج Python لأن العتاد الحقيقي فيه τ فيزيائي
        # بالفعل، وإضافة τ رياضي فوقه مضاعفة للتأخير.
        sim_cfg["simulation"]["use_actuator_dynamics"] = False

        # ─── Noise generator ─────────────────────────────────────────────
        # يقرأ من bridge.noise في sim config (نفس مصدر SITL/PIL).
        # سابقاً كان HIL يستخدم hardcoded σ=0.1/0.002 فقط للـ IMU ويتجاهل
        # mag/baro/GPS noise تماماً — بيانات مثالية غير واقعية تخفي مشاكل
        # EKF2 biases/drift التي تظهر فقط في real hardware.
        noise_cfg = sim_cfg.get("bridge", {}).get("noise", {})
        self.noise = SensorNoise({"noise": noise_cfg}, rng_seed=self.seed)

        import tempfile
        self._tmp = tempfile.NamedTemporaryFile(
            mode="w", suffix=".yaml", delete=False, encoding="utf-8"
        )
        yaml.dump(sim_cfg, self._tmp, allow_unicode=True, default_flow_style=False)
        self._tmp.close()

        long_range = sim_cfg.get("long_range", {}).get("enabled", False)
        self.sim = Rocket6DOFSimulation(
            config_file=self._tmp.name, long_range_mode=long_range
        )

        # ─── HIL fix: disable simulated actuator dynamics ──────────────────
        # In HIL closed_loop the *real* CAN servos already apply their physical
        # τ ≈ 25 ms first-order lag to the MPC command, and the bridge feeds
        # the post-lag measurement (fin_can) back into the simulator as the
        # control input.  If `use_actuator_dynamics` is left enabled the
        # simulator applies *another* τ filter on top of fin_can — i.e. the
        # rocket's aerodynamic forces see fins lagged by ~50 ms instead of the
        # ~25 ms physical reality.  That double lag is what blew up alpha to
        # 36–47° and crashed HITL scores from PIL's 100/100 down to ~30/100.
        # PIL stays single-lag because it feeds fin_cmd (no real servo) into
        # the same simulator filter once.  Real flight is also single-lag
        # (only the physical servo, no simulator).  Forcing the actuator off
        # here makes HIL match real-flight servo dynamics exactly.
        if getattr(self.sim, "use_actuator_dynamics", False):
            self.sim.use_actuator_dynamics = False
            self.sim.actuator = None
            print("[HIL] use_actuator_dynamics disabled (real CAN servo "
                  "already provides physical τ; double-lag avoided)")

        self.launch_lat = sim_cfg.get("launch", {}).get("latitude", 16.457472)
        self.launch_lon = sim_cfg.get("launch", {}).get("longitude", 44.115361)
        self.launch_alt = sim_cfg.get("launch", {}).get("altitude", 1200.0)

        self.rng = np.random.default_rng(self.seed)

        # الشبكة
        self._sock: Optional[socket.socket] = None
        self._parser = MavParser()

        # الحالة
        self._sim_t_us = 0
        self._fins_rad = np.zeros(4)
        self._fin_can_rad = np.zeros(4)     # آخر زاوية CAN (rad) — للحفظ في flight log
        self._fin_source = "cmd"             # مصدر _fins_rad لهذه الخطوة:
        #   "can"   — الزاوية المقاسة الفعلية للسيرفو (fresh + online)
        #   "hold"  — آخر زاوية CAN معروفة (stale within abort window)
        #   "cmd"   — grace period قبل أول فيدباك
        #   "abort" — إيقاف محاكاة بسبب فقدان feedback
        self._last_controls = np.zeros(16)
        self._actuator_msg_count = 0         # عدد رسائل HIL_ACTUATOR_CONTROLS المستلمة
        self._last_param_values: dict = {}   # name -> {value, type, count, index}
        self._param_lock = threading.Lock()  # يحمي _last_param_values (يُكتب من خيط timing 5760)
        # ``_running`` عَلَم bool يُكتب من عدّة خيوط بلا قفل. CPython يضمن
        # atomicity لـ bool-assign (مرجع واحد في عدد منازل البايتكود)، فالقراءة
        # لا يمكن أن ترى نصف قيمة. الدلالات القوية (happens-before) غير مضمونة
        # رسمياً لكن GIL يجعلها عملياً آمنة. لو أضفنا قفلاً لاحقاً، يجب تغطية:
        # _send / _recv_nonblock / _timing_reader_loop + كل فحوصات while الـ
        # running في حلقات warm-up و flight. M10: مُوثَّق كعَلَم atomic متعمّد.
        self._running = False

        # ─── فيدباك السيرفو الحقيقي ─────────────────────────────────────
        self._servo_fb_rad = np.zeros(4)       # آخر زاوية مقاسة (rad)
        self._servo_cmd_fb_rad = np.zeros(4)   # الأمر كما رآه الدرايفر (rad)
        self._servo_fb_t_us = 0                # timestamp آخر SRV_FB (sim time)
        # monotonic timestamp لحساب عمر الفيدباك بمعزل عن _sim_t_us:
        # _sim_t_us يتبدّل معناه بين wall-clock (warm-up) و t_off+step*dt
        # (flight loop)، واستخدامه كمرجع للعُمر عبر خيوط يُنشئ هشاشة.
        # monotonic_ns مصدر موحّد آمن لا يتأثّر بترتيب تهيئة _t_off_us.
        self._servo_fb_mono_ns = 0
        self._servo_online_mask = 0            # bit-mask للسيرفوهات المتصلة
        self._servo_tx_fail = 0
        self._servo_fb_count = 0               # عدد الرسائل المستلمة
        self._fallback_cmd_count = 0           # مرات استخدام الأمر بدل الفيدباك
        self._servo_fb_lock = threading.Lock()
        # ─── معايرة الصفر التلقائية (zero-offset) ─────────────────────
        # يُطرح هذا الانحراف من fb_deg قبل استخدامه في servo_log وطبقة
        # السلامة، لتعويض الانحراف الميكانيكي الأولي لكل سيرفو.
        self._servo_zero_offset_rad = np.zeros(4)
        self._zero_calib_active = False
        self._zero_calib_samples: list[np.ndarray] = []
        # سجلّ فيدباك السيرفو للحفظ بعد الرحلة
        self.servo_log = {
            "t_sim": [], "cmd_deg": [], "fb_deg": [], "err_deg": [],
            "online_mask": [], "tx_fail": [],
        }

        # سجلّ الرحلة
        self.flight = {
            "time": [], "pos": [], "vel": [], "quat": [], "omega": [],
            "mass": [], "fin_cmd": [], "fin_act": [], "forces": [],
            "altitude": [], "ground_range": [], "alpha": [], "beta": [], "mach": [],
            "lat": [], "lon": [], "alt_msl": [],
            "fin_can": [],     # زاوية CAN الفعلية (rad)
            "fin_source": [], # "can" أو "cmd"
        }

        # سجلّ التوقيت
        self.timing = {
            "t_sim": [], "mhe_us": [], "mpc_us": [], "cycle_us": [],
        }
        self._last_timing = {"mhe_us": 0.0, "mpc_us": 0.0, "cycle_us": 0.0}
        self._timing_lock = threading.Lock()

        # قناة MAVLink ثانوية لالتقاط DEBUG_VECT من mavlink الكامل عبر TCP 5760
        # (simulator_mavlink TCP 4560 يبثّ فقط HIL_* و HIL_ACTUATOR_CONTROLS)
        self._mavlink_tcp_host = tgt.get("mavlink_tcp", {}).get("host", "127.0.0.1")
        self._mavlink_tcp_port = int(tgt.get("mavlink_tcp", {}).get("port", 5760))
        self._timing_thread: Optional[threading.Thread] = None
        self._timing_stop = threading.Event()
        self._timing_sock: Optional[socket.socket] = None
        # event يُرفع بمجرد نجاح اتصال 5760 في خيط timing — يستخدم
        # _set_hitl_param لينتظر جاهزية القناة الصحيحة قبل إرسال PARAM_SET.
        self._timing_ready = threading.Event()
        # قفل لتسلسل الكتابة على _timing_sock بين خيط timing (heartbeat)
        # والخيط الرئيسي (PARAM_SET عند ضبط SYS_HITL).
        self._timing_sock_lock = threading.Lock()
        self._srv: Optional[socket.socket] = None
        self._closed = False

    def __del__(self):
        # احتفظ بالخلاص كملاذ أخير، لكن المسار الرئيسي يمرّ عبر close()
        # الذي يُستدعى من run()/finally. __del__ وحده غير موثوق.
        self.close()

    def close(self):
        """تنظيف صريح للموارد (سوكتات + ملف YAML المؤقت)."""
        if self._closed:
            return
        self._closed = True
        tmp_name = getattr(getattr(self, "_tmp", None), "name", None)
        if tmp_name:
            try:
                os.unlink(tmp_name)
            except (OSError, FileNotFoundError):
                pass

    # ─── الشبكة ──────────────────────────────────────────────────────────────

    def accept(self):
        """انتظار اتصال PX4 من الجهاز الهدف."""
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        srv.bind((self.host, self.port))
        srv.listen(1)
        srv.settimeout(self.accept_timeout_s)
        self._srv = srv
        print(f"[HIL] Listening on TCP {self.host}:{self.port} — "
              f"start PX4 on target device...")
        self._sock, addr = srv.accept()
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        # نُبقي الـ socket blocking افتراضياً. _recv_nonblock يُبدّل الحالة
        # مؤقتاً لقراءة واحدة ثم يعيدها لـ blocking — كلاهما يُستدعى من
        # الخيط الرئيسي فقط، لذا لا يلزم قفل. إبقاء blocking ضروري لأن
        # _send يستخدم sendall() الذي يرفع BlockingIOError على non-blocking
        # socket حين يمتلئ TCP send buffer، فيُعدّ هذا disconnect خاطئاً.
        # (نفس نمط pil/mavlink_bridge_pil.py المُثبت.)
        srv.close()
        self._srv = None
        print(f"[HIL] Target connected from {addr}")

        # بدء خيط التقاط DEBUG_VECT من mavlink الكامل (TCP 5760) بعد الاتصال
        if self.timing_enabled:
            self._timing_thread = threading.Thread(
                target=self._timing_reader_loop, daemon=True
            )
            self._timing_thread.start()

    def _timing_reader_loop(self):
        """يتصل بـ mavlink TCP bridge (port 5760) ويلتقط DEBUG_VECT/NAMED_VALUE_FLOAT."""
        # إعادة المحاولة لأن mavlink_tcp_bridge يبدأ بعد MAVLink نفسه
        sock = None
        parser = MavParser()
        deadline = time.monotonic() + 30.0
        while not self._timing_stop.is_set() and time.monotonic() < deadline:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(2.0)
                sock.connect((self._mavlink_tcp_host, self._mavlink_tcp_port))
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self._timing_sock = sock
                self._timing_ready.set()
                print(f"[HIL] Timing reader connected to "
                      f"{self._mavlink_tcp_host}:{self._mavlink_tcp_port}")
                break
            except (OSError, socket.timeout):
                if sock:
                    try:
                        sock.close()
                    except OSError:
                        pass
                    sock = None
                time.sleep(0.5)
        if sock is None:
            print("[HIL] WARNING: timing reader could not connect — "
                  "CSV timing will be empty. Ensure "
                  "'adb reverse tcp:5760 tcp:5760' is set.")
            return

        sock.settimeout(0.2)
        # PX4 MAVLink لا يُرسل streams إلى partner لم يُستلم منه HEARTBEAT.
        # نرسل heartbeat كل ثانية لتأسيس الرابط وتفعيل DEBUG_VECT stream.
        hb_bytes = build_heartbeat()
        last_hb = 0.0
        total_rx = 0
        msg_counts: dict = {}
        last_stat = time.monotonic()
        while not self._timing_stop.is_set():
            now = time.monotonic()
            if now - last_hb >= 1.0:
                try:
                    with self._timing_sock_lock:
                        sock.send(hb_bytes)
                except OSError:
                    break
                last_hb = now
            try:
                data = sock.recv(4096)
                if not data:
                    break
                total_rx += len(data)
                for msg_id, payload in parser.feed(data):
                    msg_counts[msg_id] = msg_counts.get(msg_id, 0) + 1
                    if msg_id == MSG_DEBUG_VECT:
                        p = parse_debug_vect(payload)
                        if p and p["name"].upper().startswith("TIMING"):
                            with self._timing_lock:
                                self._last_timing["mhe_us"] = p["x"]
                                self._last_timing["mpc_us"] = p["y"]
                                self._last_timing["cycle_us"] = p["z"]
                                self.timing["t_sim"].append(self._sim_t_us / 1e6)
                                self.timing["mhe_us"].append(p["x"])
                                self.timing["mpc_us"].append(p["y"])
                                self.timing["cycle_us"].append(p["z"])
                    elif msg_id == MSG_NAMED_VALUE_FLOAT:
                        p = parse_named_value_float(payload)
                        if p and p["name"] in ("mhe_us", "mpc_us", "cycle_us"):
                            with self._timing_lock:
                                self._last_timing[p["name"]] = p["value"]
                    elif msg_id == MSG_DEBUG_FLOAT_ARRAY:
                        # فيدباك السيرفو من xqpower_can → debug_array → DEBUG_FLOAT_ARRAY
                        self._handle_debug_float_array(payload)
                    elif msg_id == MSG_PARAM_VALUE:
                        # PARAM_VALUE يُبَث فقط على MAVLink العادي (5760)، ليس على
                        # simulator_mavlink (4560) الذي يقتصر على HIL_*
                        p = parse_param_value(payload)
                        if p:
                            with self._param_lock:
                                self._last_param_values[p["name"]] = p
            except socket.timeout:
                pass
            except (OSError, ConnectionResetError):
                break
            if time.monotonic() - last_stat >= 3.0:
                top = sorted(msg_counts.items(), key=lambda kv: -kv[1])[:8]
                print(f"[HIL-timing] rx={total_rx}B msgs={top}")
                last_stat = time.monotonic()
        # الخروج من اللوب: أغلق الـ socket بأمان (shutdown + close)
        self._timing_ready.clear()
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass
        self._timing_sock = None

    def _cleanup_sockets(self) -> None:
        """إغلاق آمن لجميع sockets وإيقاف خيط timing.

        يُستدعى من finally في run() ليضمن عدم تسرّب الـ sockets حتى لو حدث
        استثناء أو خرج المستخدم بـ Ctrl+C.
        """
        # أوقِف خيط timing أولاً
        self._timing_stop.set()
        # أغلق timing socket (يُخرج thread من blocking recv)
        if self._timing_sock is not None:
            try:
                self._timing_sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self._timing_sock.close()
            except OSError:
                pass
            self._timing_sock = None
        # أغلق الـ target socket (4560)
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        # أغلق server socket لو لسه مفتوح (قبل accept)
        if self._srv is not None:
            try:
                self._srv.close()
            except OSError:
                pass
            self._srv = None
        # انتظر خيط timing ينتهي
        if self._timing_thread is not None and self._timing_thread.is_alive():
            self._timing_thread.join(timeout=2.0)

    def _send(self, data: bytes):
        if self._sock is None:
            return
        try:
            self._sock.sendall(data)
        except (BrokenPipeError, ConnectionResetError, OSError) as e:
            print(f"[HIL] Target disconnected on send: {e}")
            self._running = False

    def _recv_nonblock(self) -> list:
        # نُبدّل الـ socket إلى non-blocking لقراءة واحدة فقط ثم نُعيده
        # إلى blocking. الإبقاء على blocking ضروري لـ sendall() في _send.
        # _send و _recv_nonblock يُستدعيان من الخيط الرئيسي فقط (خيط
        # 5760 له socket منفصل)، فلا حاجة لقفل هنا.
        if self._sock is None:
            return []
        try:
            self._sock.setblocking(False)
            data = self._sock.recv(4096)
            if not data:
                self._running = False
                return []
            return self._parser.feed(data)
        except (BlockingIOError, socket.timeout):
            return []
        except (ConnectionResetError, BrokenPipeError, OSError):
            self._running = False
            return []
        finally:
            if self._sock is not None:
                try:
                    self._sock.setblocking(True)
                except OSError:
                    pass

    # ─── بناء بيانات الحسّاسات ───────────────────────────────────────────────

    def _static_sensors(self, state: np.ndarray) -> dict:
        """حسّاسات ثابتة للصاروخ على المنصّة (forces=0, vel=0) مع ضجيج جديد
        كل استدعاء.

        الاستدعاء من حلقات warm-up/auto-zero/PARAM بدل استخدام ``init_sensors``
        مُحسَب مرة واحدة يحلّ M9: ضجيج متجدّد في كل تكرار يمنع EKF2 من بناء
        bias estimation على قيمة ثابتة تماماً طوال ~80s من التهيئة.

        position_lla fix (mirrors PIL bridge LESSONS_LEARNED 2026-05-06):
        in long_range_mode, state[0:3] is ECEF (~1.8M m). Without explicit
        position_lla, _sensors falls to _ned_to_lla(ECEF_as_NED) → baro
        ~3e11 hPa → EKF2 altitude corrupt.
        """
        snap = {"forces": [0, 0, 0], "vel_ned": [0, 0, 0]}
        if getattr(self.sim, "long_range_mode", False):
            snap["position_lla"] = (
                self.launch_lat, self.launch_lon, self.launch_alt
            )
        return self._sensors(snap, state)

    def _sensors(self, snapshot: dict, state: np.ndarray):
        pos = state[0:3]
        vel = state[3:6]
        quat = state[6:10]
        omega = state[10:13]
        mass = state[13] if len(state) > 13 else 30.0

        f_body = np.array(snapshot.get("forces", [0, 0, 0]))
        accel_body = _body_specific_force(f_body, mass, np.array([0, 0, 9.80665]), quat)

        if "position_lla" in snapshot and snapshot["position_lla"] is not None:
            lla = np.asarray(snapshot["position_lla"])
            # snapshot stores lat/lon already in DEGREES (see rocket_6dof_sim.py:2408)
            lat, lon, alt = float(lla[0]), float(lla[1]), float(lla[2])
        else:
            lat, lon, alt = _ned_to_lla(pos, self.launch_lat, self.launch_lon, self.launch_alt)

        vel_ned = np.array(snapshot.get("vel_ned", vel))
        mag = _mag_body(lat, quat)

        rho = 1.225 * (1.0 - 2.25577e-5 * alt) ** 4.25588
        airspeed = float(np.linalg.norm(vel))
        diff_p = 0.5 * rho * airspeed * airspeed / 100.0

        # ضجيج مبني على config (bridge.noise) — IMU/mag/baro يتلقون ضجيج
        # واقعي + biases ثابتة لكل run. GPS noise + delay يُطبَّقان في _run_impl
        # (لحظة البناء) لأنهم يحتاجون t_usec المُحدَّث.
        noisy_accel, noisy_gyro = self.noise.add_imu_noise(accel_body, omega)
        noisy_mag = self.noise.add_mag_noise(mag)
        noisy_alt = self.noise.add_baro_noise(alt)
        noisy_baro_p = 1013.25 * (1.0 - 2.25577e-5 * noisy_alt) ** 5.25588

        return {
            "accel_body": noisy_accel, "gyro_body": noisy_gyro,
            "accel_body_true": accel_body,
            "mag_body": noisy_mag, "baro_p": noisy_baro_p, "diff_p": diff_p,
            "pressure_alt": noisy_alt,
            "lat": lat, "lon": lon, "alt": alt,
            "vel_ned": vel_ned, "quat": quat, "omega": omega,
            "airspeed": airspeed,
        }

    # ─── التشغيل الرئيسي ─────────────────────────────────────────────────────

    def run(self, flight_csv: str, timing_csv: Optional[str] = None) -> dict:
        try:
            return self._run_impl(flight_csv, timing_csv)
        finally:
            self._cleanup_sockets()
            self.close()

    def _run_impl(self, flight_csv: str, timing_csv: Optional[str] = None) -> dict:
        self.accept()
        self._running = True

        sim = self.sim
        dt = self.dt
        n_steps = int(self.duration / dt)

        state = sim._initialize_state()
        sim._update_last_valid_quaternion(state[6:10])

        def _ctrl(_state_dict, _t):
            return self._fins_rad.copy()

        # warm-up موسّع: إرسال HEARTBEAT وحسّاسات متجدّدة الضجيج لتمكين الهاتف من:
        # (1) بدء simulator_mavlink وEKF2، (2) تنفيذ auto-arm، (3) ثبات القياسات.
        # كل حلقة warm-up أدناه تستدعي self._static_sensors(state) في كل تكرار
        # بدل مشاركة dict ثابت (M9): الضجيج يتجدّد → EKF2 لا يبني bias
        # estimation على قيمة محفورة.

        # ─── معايرة صفر السيرفوهات التلقائية (قبل warm-up) ──────────────
        if self.use_servo_feedback and self.servo_auto_zero:
            self._calibrate_servo_zero(state, dt)

        # ─── إلغاء Flight Termination (لو عالقة من جلسة سابقة) ────────
        # بدون هذا، PX4 يرفض ARM ويبقى في حالة إنهاء الطيران.
        if self.clear_flight_termination:
            print("[HIL] Clearing flight termination (if active)...")
            self._send(build_command_long(
                MAV_CMD_DO_FLIGHTTERMINATION,
                p1=0.0,  # 0 = disable termination
            ))
            time.sleep(0.5)

        # ─── تفعيل SYS_HITL=1 في PX4 (قبل warm-up) ─────────────────────
        # بدون هذا، PX4 يعمل في real flight mode ولا يُرسل HIL_ACTUATOR_CONTROLS.
        # نُرسل PARAM_SET ثم نتحقق من PARAM_VALUE المُعاد.
        if self.set_hitl_param:
            self._set_hitl_param(state, dt)

        # ─── Warm-up ديناميكي: ننتظر PX4 يصبح جاهزاً ───────────────
        # الاستراتيجية:
        #   1) نُرسل حساسات + HEARTBEAT باستمرار لـ PX4
        #   2) كل arm_every_s نُرسل أمر ARM عبر COMMAND_LONG
        #   3) نراقب رسائل HIL_ACTUATOR_CONTROLS — أول رسالة =
        #      PX4 سلّم نفسه ويُحكم → جاهز!
        #   4) بعد أول actuator msg ننتظر settle_after_arm_s لاستقرار
        #      EKF النهائي ثم نبدأ المحاكاة فوراً
        #   هذا يضمن أن PX4 يبدأ التحكم من t≈0 بدون فجوة
        print("[HIL] Warm-up: sending sensors + ARM, waiting for PX4 readiness...")
        self._actuator_msg_count = 0
        wu_start = time.monotonic()
        wu_deadline = wu_start + self.warmup_timeout_s
        last_arm_time = 0.0
        arm_count = 0
        armed = False

        while time.monotonic() < wu_deadline:
            now = time.monotonic()
            elapsed = now - wu_start

            # استخدم wall-clock لتفادي drift: PX4 يعتمد على time_usec monotonic
            # مقابل الـ scheduler الفعلي. الزيادة الثابتة +5000µs مع sleep(5ms)
            # الفعلي يمكن أن تُحدث انحرافاً يُعلّم الحسّاسات STALE.
            self._sim_t_us = int(time.monotonic() * 1e6)
            self._send(build_heartbeat())
            self._send(build_hil_state_quat(
                self._sim_t_us,
                np.array([1, 0, 0, 0]), np.zeros(3),
                self.launch_lat, self.launch_lon, self.launch_alt,
                0, 0, 0, np.array([0, 0, -9.80665]), 0.0,
            ))
            self._send(build_hil_gps(
                self._sim_t_us,
                self.launch_lat, self.launch_lon, self.launch_alt, 0, 0, 0,
            ))
            s = self._static_sensors(state)
            self._send(build_hil_sensor(
                self._sim_t_us,
                s["accel_body"], s["gyro_body"],
                s["mag_body"], s["baro_p"],
                s["diff_p"], s["pressure_alt"],
            ))

            # أرسل ARM بشكل متكرر
            if now - last_arm_time >= self.warmup_arm_every_s:
                self._send(build_command_long(
                    MAV_CMD_COMPONENT_ARM_DISARM,
                    p1=1.0,          # 1 = arm
                    p2=21196.0,      # force-arm magic
                ))
                arm_count += 1
                last_arm_time = now

            self._drain_target(dt)

            # أول رسالة HIL_ACTUATOR_CONTROLS = PX4 جاهز
            if self._actuator_msg_count > 0:
                if not armed:
                    armed = True
                    arm_elapsed = now - wu_start
                    print(f"[HIL] PX4 armed! (actuator msg received at {arm_elapsed:.1f}s, "
                          f"{arm_count} ARM attempts)")
                    settle_end = now + self.warmup_settle_s
                    # استمر بإرسال الحساسات أثناء الاستقرار
                    while time.monotonic() < settle_end and self._running:
                        self._sim_t_us = int(time.monotonic() * 1e6)
                        self._send(build_heartbeat())
                        self._send(build_hil_state_quat(
                            self._sim_t_us,
                            np.array([1, 0, 0, 0]), np.zeros(3),
                            self.launch_lat, self.launch_lon, self.launch_alt,
                            0, 0, 0, np.array([0, 0, -9.80665]), 0.0,
                        ))
                        s = self._static_sensors(state)
                        self._send(build_hil_sensor(
                            self._sim_t_us,
                            s["accel_body"], s["gyro_body"],
                            s["mag_body"], s["baro_p"],
                            s["diff_p"], s["pressure_alt"],
                        ))
                        self._drain_target(dt)
                        time.sleep(0.005)
                # إذا تم فصل PX4 أثناء الاستقرار، لا تستمر في الطيران
                # بصمت — ارفع خطأ واضح لتجنّب محاكاة فارغة طولها duration_s.
                if not self._running:
                    raise RuntimeError(
                        "target disconnected during warm-up settle phase"
                    )
                break

            if elapsed > self.warmup_max_s and not armed:
                print(f"[HIL] WARNING: no actuator message after {elapsed:.1f}s — "
                      f"proceeding anyway (PX4 may start late)")
                break

            time.sleep(0.005)

        total_wu = time.monotonic() - wu_start
        print(f"[HIL] Warm-up done in {total_wu:.1f}s  "
              f"(armed={armed}, actuator_msgs={self._actuator_msg_count})")

        # إذا خرجنا من warm-up بدون أي HIL_ACTUATOR_CONTROLS، فغالباً PX4
        # ليس في HIL mode (SYS_HITL=0 أو لم تُحفظ بعد restart) — تشغيل
        # الحلقة في هذه الحالة يُنتج CSV فارغة ثم abort بعد grace period.
        # نفشل صراحةً مع تعليمات واضحة بدلاً من متابعة صامتة.
        if self._actuator_msg_count == 0 and self.abort_on_no_actuator:
            raise RuntimeError(
                "[HIL] PX4 لم يُرسل أي HIL_ACTUATOR_CONTROLS خلال warm-up.\n"
                "  الأسباب المحتملة:\n"
                "    1) SYS_HITL=0 — ضَبطه يتطلب إعادة تشغيل PX4 بعد PARAM_SET.\n"
                "       (شغّل الجلسة مرة، ستُحفظ SYS_HITL=1 في parameters.bson،\n"
                "        ثم أعد التشغيل يدوياً من التطبيق وأعد تشغيل HIL.)\n"
                "    2) simulator_mavlink لم يبدأ على 4560 (تحقق من dmesg/logcat).\n"
                "    3) rocket_mpc لم يُنشر actuator_outputs_sim (تحقق من armed).\n"
                "  لتعطيل هذا الفحص (غير مُوصى): warmup.abort_on_no_actuator=false"
            )

        print("[HIL] Starting flight loop (realtime)...")
        t_wall0 = time.monotonic()
        _t_off_us = self._sim_t_us + int(dt * 1e6)
        step = 0
        t = 0.0

        # معدلات الإرسال:
        #   HIL_SENSOR  : 100Hz (سقف عملي محدَّد بـ dt؛ EKF2 يقبل 80Hz+).
        #   HIL_STATE   : 50Hz (ground-truth عرضي، لا يُغذّي EKF2).
        #   HIL_GPS     : 5Hz (matches real NEO-M8N config).
        # الصيغة السابقة كانت `1.0/(250*dt)` التي تُنتج 0→max(1,0)=1 مع dt=0.01،
        # مُوهمة أن المعدّل 250Hz. الصيغة الجديدة صريحة: `1.0/(100*dt)` مع تعليق
        # يُبرز أن 100 هي سقف فعلي مع dt=0.01 (تجنّب M4 confusion).
        sensor_every = max(1, int(round(1.0 / (100 * dt))))
        gps_every = max(1, int(round(1.0 / (5 * dt))))   # 5 Hz — matches real NEO-M8N
        state_every = max(1, int(round(1.0 / (50 * dt))))
        hb_every = 100

        while step < n_steps and self._running:
            t = step * dt
            self._sim_t_us = _t_off_us + int(t * 1e6)

            # ─── قراءة حالة فيدباك CAN ─────────────────────────────────
            # العمر يُحسب بـ monotonic_ns (مرجع موحّد عبر الخيوط) لا
            # بـ _sim_t_us الذي يتبدّل معناه بين wall-clock و t_off+step*dt.
            now_mono_ns = time.monotonic_ns()
            with self._servo_fb_lock:
                fb_mono_ns_snap = self._servo_fb_mono_ns
                fb_ever_seen = fb_mono_ns_snap > 0
                fb_age_us = (
                    (now_mono_ns - fb_mono_ns_snap) // 1000
                    if fb_ever_seen else 0
                )
                fb_fresh = (
                    fb_ever_seen
                    and fb_age_us < self.servo_feedback_timeout_ms * 1000
                )
                online_mask_snap = self._servo_online_mask & 0x0F
                all_online = online_mask_snap == 0x0F
                fb_rad = self._servo_fb_rad.copy()
            if fb_fresh:
                self._fin_can_rad = fb_rad.copy()

            # ─── اختيار زاوية الزعنفة للمحاكاة (closed_loop دائماً) ────
            # الزاوية المقاسة تقود الأيروديناميكا. use_actuator_dynamics
            # مُعطَّل في sim_cfg → لا نموذج Python إضافي.
            fb_useable = fb_fresh and (
                not self.require_all_servos_online
                or all_online
            )
            within_grace = t * 1000.0 < self.servo_feedback_grace_ms
            abort_threshold_us = self.servo_feedback_abort_ms * 1000.0
            stale_beyond_abort = (
                not within_grace
                and (not fb_ever_seen or fb_age_us >= abort_threshold_us)
            )
            # تأجيل قرار الإيقاف إلى ما بعد _log ليظهر آخر صف في CSV
            # مع fin_source="abort" (سهولة تشخيص ما قبل السقوط).
            abort_reason: Optional[str] = None
            if fb_useable:
                self._fins_rad = fb_rad
                self._fin_source = "can"
            elif stale_beyond_abort:
                print(f"\n[HIL] ABORT: servo feedback lost "
                      f"(age={fb_age_us/1000:.0f}ms, mask=0x"
                      f"{self._servo_online_mask:02X}). "
                      f"Stopping simulation at t={t:.3f}s.")
                self._fins_rad = self._fin_can_rad.copy()
                self._fin_source = "abort"
                abort_reason = "feedback_lost"
            elif fb_ever_seen:
                # تأخير ضمن window [timeout..abort]: احتفظ بآخر زاوية مقاسة
                self._fins_rad = self._fin_can_rad.copy()
                self._fin_source = "hold"
                self._fallback_cmd_count += 1
            else:
                # grace period قبل أول فيدباك: استخدم الأمر مؤقتاً
                self._fins_rad = self._last_controls[:4].copy()
                self._fin_source = "cmd"
                self._fallback_cmd_count += 1

            # ─── تحقق سلامة: خطأ تتبع > 10° مستدام → فشل سيرفو محتمل ────
            # المرجع المطلوب من العتاد هو أمر MPC الخام
            # (`_last_controls[:4]`)، بصرف النظر عمّا يُحقن في الأيروديناميكا.
            # نفحص فقط القنوات الـ online لتفادي false-positive من قنوات
            # offline التي قد تحتوي على قيم قديمة/صفرية.
            if fb_useable and self._servo_abort_rad > 0.0:
                # عتبة الـ abort تُقرَأ من hil_config.yaml
                # (`servo_tracking_abort_deg`). default=0 (معطَّل).
                err_all = np.abs(fb_rad - self._last_controls[:4])
                if all_online:
                    err_rad = float(np.max(err_all))
                else:
                    online_ch = [i for i in range(4) if online_mask_snap & (1 << i)]
                    err_rad = float(np.max(err_all[online_ch])) if online_ch else 0.0
                if err_rad > self._servo_abort_rad:
                    self._servo_safety_breach += 1
                    if self._servo_safety_breach >= self.servo_safety_breach_limit:
                        print(f"\n[HIL] ABORT: sustained servo tracking error "
                              f">{np.degrees(self._servo_abort_rad):.1f}° for "
                              f"{self._servo_safety_breach} consecutive steps "
                              f"at t={t:.3f}s")
                        self._fin_source = "abort"
                        abort_reason = "tracking_error"
                else:
                    self._servo_safety_breach = 0
            else:
                # فيدباك غير قابل للاستخدام (grace/hold/stale) → الفحص
                # غير مُطبَّق لأن المرجع الحقيقي مفقود. نُصفّر العدّاد حتى
                # لا يتراكم من فترات hold ثم يُشعل abort مزيّفاً بمجرد
                # عودة الفيدباك وخطأ كبير (متوقع بعد فجوة).
                self._servo_safety_breach = 0

            try:
                next_state, snap, t_end, _ = sim._integrate_one_step(state, t, dt, _ctrl)
            except Exception as e:
                print(f"[HIL] sim error at t={t:.3f}: {e}")
                break
            sim._normalize_state(next_state, t_end)
            s = self._sensors(snap, next_state)

            if step % hb_every == 0:
                self._send(build_heartbeat())

            if step % state_every == 0:
                self._send(build_hil_state_quat(
                    self._sim_t_us,
                    s["quat"], s["omega"],
                    s["lat"], s["lon"], s["alt"],
                    s["vel_ned"][0], s["vel_ned"][1], s["vel_ned"][2],
                    s["accel_body_true"], s["airspeed"],
                ))

            if step % gps_every == 0:
                # GPS noise + delay مبني على config: add_gps_noise يُرجع
                # None خلال فترة delay الأولى (EKF2 لا يستقبل حتى يكتمل
                # الـ buffer)، كما يحدث مع GPS حقيقي. يُحاكي NEO-M8N real
                # behavior بدلاً من fix مثالي فوري.
                gps_data = self.noise.add_gps_noise(
                    s["lat"], s["lon"], s["alt"],
                    s["vel_ned"][0], s["vel_ned"][1], s["vel_ned"][2],
                    self._sim_t_us,
                )
                if gps_data is not None:
                    g_lat, g_lon, g_alt, g_vn, g_ve, g_vd = gps_data
                    self._send(build_hil_gps(
                        self._sim_t_us,
                        g_lat, g_lon, g_alt,
                        g_vn, g_ve, g_vd,
                    ))

            if step % sensor_every == 0:
                self._send(build_hil_sensor(
                    self._sim_t_us,
                    s["accel_body"], s["gyro_body"], s["mag_body"],
                    s["baro_p"], s["diff_p"], s["pressure_alt"],
                ))

            self._drain_target(dt)
            self._log(snap, next_state, t_end, s)

            if abort_reason is not None:
                break

            if sim._detect_ground_impact(next_state, t_end):
                print(f"\n[HIL] Ground impact at t={t_end:.3f}s")
                break

            # ─── Hybrid Lockstep (digital loop sync) ──────────────────────
            # انتظر HIL_ACTUATOR_CONTROLS جديدة من PX4 قبل wall-clock pacing.
            # هذا يُزيل TCP/scheduler jitter من حلقة sim↔PX4 مع إبقاء
            # pacing واقعياً للسيرفو الفيزيائي (الذي لا يمكن إيقافه).
            # السيرفو والـ CAN و USB-Serial يبقون على نفس تأخيرهم الفيزيائي —
            # وهذا ما نريد قياسه.
            if self._lockstep and self._actuator_msg_count > 0:
                wait_start = time.monotonic()
                ack_deadline = wait_start + self._lockstep_timeout_s
                target_count = self._actuator_msg_count + 1
                while (self._actuator_msg_count < target_count
                       and time.monotonic() < ack_deadline
                       and self._running):
                    self._drain_target(dt)
                    # sleep قصير جداً لتجنب busy-wait مع إبقاء latency منخفض
                    time.sleep(0.0002)

                wait_us = int((time.monotonic() - wait_start) * 1e6)
                if self._actuator_msg_count >= target_count:
                    self._lockstep_ack_count += 1
                    self._lockstep_wait_us_sum += wait_us
                    if wait_us > self._lockstep_wait_us_max:
                        self._lockstep_wait_us_max = wait_us
                else:
                    self._lockstep_timeout_count += 1

            # pacing (wall-clock): السيرفوهات فيزيائية، لا نسبق الزمن الحقيقي
            target_wall = t_wall0 + t
            now = time.monotonic()
            if now < target_wall:
                time.sleep(target_wall - now)

            state = next_state
            step += 1

            if step % 1000 == 0:
                pct = 100.0 * step / n_steps
                alt = -next_state[2]
                spd = float(np.linalg.norm(next_state[3:6]))
                print(f"\r[HIL] {pct:5.1f}% t={t:6.1f}s alt={alt:7.0f}m "
                      f"v={spd:5.0f}m/s fins=[{self._fins_rad[0]:+.3f},"
                      f"{self._fins_rad[1]:+.3f},{self._fins_rad[2]:+.3f},"
                      f"{self._fins_rad[3]:+.3f}]", end="", flush=True)

        print(f"\n[HIL] Loop done: {step} steps, t={t:.3f}s, "
              f"wall={time.monotonic() - t_wall0:.1f}s")

        # إيقاف خيط التوقيت
        self._timing_stop.set()
        if self._timing_thread is not None:
            self._timing_thread.join(timeout=2.0)
        print(f"[HIL] Timing samples captured: {len(self.timing['t_sim'])}")

        # ─── ملخّص Hybrid Lockstep ────────────────────────────────────
        # B4 (Obs-08) 2026-05-08: توضيح أن النظام HITL hybrid (ليس lockstep خالصاً).
        # حتى مع lockstep:false في hil_config.yaml، الـbridge ينتظر HIL_ACTUATOR_CONTROLS
        # من PX4 قبل تقدّم sim step (sync فعلي عبر actuator)، ثم يطبّق wall-clock pacing
        # للحفاظ على real-time. الـbanner السابق "realtime pacing only" مضلل لأنه يوحي
        # بعدم وجود sync مع PX4 — وهذا غير صحيح.
        if self._lockstep:
            total_waits = self._lockstep_ack_count + self._lockstep_timeout_count
            if total_waits > 0:
                timeout_pct = 100.0 * self._lockstep_timeout_count / total_waits
                avg_wait_us = (self._lockstep_wait_us_sum
                               / max(1, self._lockstep_ack_count))
                print(f"[HIL] Lockstep (strict): {self._lockstep_ack_count} acks, "
                      f"{self._lockstep_timeout_count} timeouts ({timeout_pct:.1f}%), "
                      f"avg_wait={avg_wait_us/1000:.2f}ms, "
                      f"max_wait={self._lockstep_wait_us_max/1000:.1f}ms")
                if timeout_pct > self._lockstep_warn_threshold * 100:
                    print(f"[HIL] ⚠️  Lockstep timeouts {timeout_pct:.1f}% "
                          f"exceeds threshold {self._lockstep_warn_threshold*100:.0f}% — "
                          f"PX4 may be overloaded or thermally throttled. "
                          f"Consider raising lockstep_timeout_ms.")
        else:
            print("[HIL] Sync mode: HITL hybrid "
                  "(actuator-driven sync + PC wall-clock pacing, "
                  f"actuator_msgs={self._actuator_msg_count})")

        # ─── ملخّص فيدباك السيرفو ─────────────────────────────────────
        fb_n = self._servo_fb_count
        print(f"[HIL] Servo feedback: {fb_n} frames, "
              f"fallback_to_cmd={self._fallback_cmd_count} steps, "
              f"online_mask=0x{self._servo_online_mask:02X}, "
              f"tx_fail={self._servo_tx_fail}")
        if fb_n == 0:
            print("[HIL] WARNING: no SRV_FB received — simulation "
                  "aborted or ran on grace-period command fallback. "
                  "Check: xqpower_can running? CAN USB connected? "
                  "DEBUG_FLOAT_ARRAY stream enabled?")

        # ملاحظة: إغلاق sockets يتم في run()/finally عبر _cleanup_sockets
        # لضمان النظافة حتى لو حدث استثناء قبل الوصول لهذه النقطة.

        self._export_flight_csv(flight_csv)
        print(f"[HIL] Flight CSV: {flight_csv}")
        if timing_csv and self.timing_enabled:
            self._export_timing_csv(timing_csv)
            print(f"[HIL] Timing CSV: {timing_csv}")

        # تصدير سجلّ السيرفو (بجوار flight_csv)
        servo_csv = flight_csv.replace(".csv", "_servo.csv")
        if servo_csv == flight_csv:
            servo_csv = flight_csv + ".servo.csv"
        self._export_servo_csv(servo_csv)
        print(f"[HIL] Servo feedback CSV: {servo_csv}")

        return self.flight

    # ─── استقبال رسائل الهدف ────────────────────────────────────────────────

    def _drain_target(self, dt: float):
        """قراءة ما توفّر من الهدف بلا حجب. يحدّث التحكّم فقط.

        simulator_mavlink (TCP 4560) يبثّ HIL_* + HIL_ACTUATOR_CONTROLS فقط.
        SRV_FB وTIMING وPARAM_VALUE تأتي على MAVLink العادي (5760) ويعالجها
        ``_timing_reader_loop`` — فروع تلك الرسائل هنا كانت dead code وتسبّب
        ازدواج تسجيل محتمل (race مع خيط 5760).
        """
        msgs = self._recv_nonblock()
        for msg_id, payload in msgs:
            if msg_id == MSG_HIL_ACTUATOR_CONTROLS:
                p = parse_actuator_controls(payload)
                if p:
                    self._last_controls = np.array(p["controls"])
                    self._actuator_msg_count += 1
                    # 2026-05-08 DIAGNOSTIC: log first 50 + every 100th HIL_ACT msg.
                    # Verifies whether bridge receives non-zero controls when PX4
                    # ULog confirms actuator_outputs_sim has non-zero values.
                    n = self._actuator_msg_count
                    if n <= 50 or n % 100 == 0:
                        c = p["controls"][:4]
                        print(f"[DIAG] HIL_ACT #{n} t_us={p['t_us']} ctrl[0..3]=[{c[0]:+.4f},{c[1]:+.4f},{c[2]:+.4f},{c[3]:+.4f}]  raw_len={len(payload)}", flush=True)

    def _send_via_timing(self, data: bytes) -> bool:
        """أرسل بايتات MAVLink عبر القناة الكاملة (5760) إذا كانت جاهزة.

        PARAM_SET يجب أن يُرسَل على instance mavlink العادي (5760)، وليس
        على ``simulator_mavlink`` (4560) الذي يقتصر على رسائل HIL_*.
        يعود ``True`` عند النجاح، ``False`` إذا لم تكن القناة جاهزة أو
        فشل الإرسال.
        """
        sock = self._timing_sock
        if sock is None or not self._timing_ready.is_set():
            return False
        try:
            with self._timing_sock_lock:
                sock.sendall(data)
            return True
        except OSError:
            return False

    def _await_param_value(self, name: str, timeout_s: float,
                           state: np.ndarray, dt: float
                           ) -> Optional[dict]:
        """ينتظر وصول PARAM_VALUE للـ param المحدّد على قناة 5760.

        خلال الانتظار، يستمر في إرسال heartbeat + HIL_SENSOR على 4560
        للحفاظ على تسليم EKF وتسلسل الـ warm-up. يعود بمُحتوى
        PARAM_VALUE إذا وصل، أو ``None`` عند انتهاء المهلة.
        """
        t_end = time.monotonic() + timeout_s
        # GPS كل ~100ms (10Hz) أثناء الانتظار للحفاظ على GPS fix في EKF2.
        # بدون ذلك، انتظار PARAM_VALUE الطويل قد يُسقط GPS timeout (~1s)
        # ويُلوّث تهيئة التقدير.
        next_gps = 0.0
        while time.monotonic() < t_end and self._running:
            now = time.monotonic()
            self._sim_t_us = int(now * 1e6)
            self._send(build_heartbeat())
            s = self._static_sensors(state)
            self._send(build_hil_sensor(
                self._sim_t_us,
                s["accel_body"], s["gyro_body"],
                s["mag_body"], s["baro_p"],
                s["diff_p"], s["pressure_alt"],
            ))
            if now >= next_gps:
                self._send(build_hil_gps(
                    self._sim_t_us,
                    self.launch_lat, self.launch_lon, self.launch_alt,
                    0, 0, 0,
                ))
                next_gps = now + 0.1
            self._drain_target(dt)

            with self._param_lock:
                pv = self._last_param_values.get(name)
            if pv is not None:
                return pv

            time.sleep(0.02)
        return None

    def _set_hitl_param(self, state: np.ndarray, dt: float) -> None:
        """يتحقّق من أن ``SYS_HITL=1`` في PX4، ويَضبطه إذا لم يكن كذلك.

        **معمارية القناة**:
          • 4560 (``simulator_mavlink``): يستقبل ``HIL_SENSOR``/``HIL_GPS``/
            ``HIL_STATE_QUATERNION`` فقط. ``switch`` في
            ``SimulatorMavlink::handle_message`` ليس فيه ``case
            MAVLINK_MSG_ID_PARAM_SET``. أي ``PARAM_SET`` يصل هنا يُتجاهل
            صامتاً.
          • 5760 (``mavlink`` العادي): يحوي معالج الـ parameters (``_parameters``
            sub-module في MAVLink v2). PARAM_SET/PARAM_REQUEST_READ/PARAM_VALUE
            كلها على هذه القناة فقط.

        **تسلسل Read-then-Set** (بديل حذف الدالة كلياً):
          1) ننتظر جاهزية قناة 5760 (``_timing_ready``).
          2) ``PARAM_REQUEST_READ("SYS_HITL")`` → PX4 يستجيب بـ PARAM_VALUE.
             إذا كانت القيمة 1 أصلاً (airframe rcS ضبطها في init)، نخرج
             بدون PARAM_SET.
          3) إلا نرسل ``PARAM_SET(SYS_HITL=1)`` ونعيد الاستعلام حتى
             ``hitl_param_timeout_s``.

        **لا fallback على 4560**: الفشل هناك dead silent (لا acceptor)،
        فالتجربة تُلوّث اللوج بدون فائدة. إن كانت 5760 غير متاحة، نسجّل
        تحذيراً واضحاً ونتوقّف عن محاولة set — HIL يعتمد في هذه الحالة
        على airframe rcS وحده.
        """
        if not self.timing_enabled:
            print("[HIL] timing channel disabled — skipping SYS_HITL "
                  "verify/set. HIL relies on airframe rcS "
                  "('param set SYS_HITL 1' in init.d/airframes/...).")
            return

        timeout_s = float(self.cfg.get("warmup", {}).get("hitl_param_timeout_s", 5.0))
        print(f"[HIL] Verifying SYS_HITL=1 on TCP {self._mavlink_tcp_port} "
              f"(param channel, timeout={timeout_s:.1f}s)...")

        # انتظر جاهزية قناة 5760. خيط _timing_reader_loop يحاول الاتصال
        # حتى 30s، لكن عادةً يتّصل خلال بضع ميلي ثوان لأن _timing_thread
        # بدأ عند accept() وتبعه _calibrate_servo_zero (~6s).
        if not self._timing_ready.wait(timeout=3.0):
            print("[HIL] WARNING: param channel (TCP 5760) not ready. "
                  "Cannot verify or set SYS_HITL. Ensure the PX4 mavlink "
                  "instance is running and 'adb reverse tcp:5760 tcp:5760' "
                  "is configured. Falling back to airframe rcS for "
                  "SYS_HITL (HITL airframes set it in init).")
            return

        # ─── خطوة 1: READ ─────────────────────────────────────────────
        # نُفرغ أي PARAM_VALUE قديم من جلسة سابقة، ثم نطلب القيمة
        # الحالية من PX4 قبل اتخاذ أي إجراء.
        with self._param_lock:
            self._last_param_values.pop("SYS_HITL", None)

        if not self._send_via_timing(build_param_request_read("SYS_HITL")):
            print("[HIL] WARNING: failed to send PARAM_REQUEST_READ on "
                  "param channel. Skipping SYS_HITL verify/set.")
            return

        pv = self._await_param_value("SYS_HITL", 1.5, state, dt)
        if pv is not None and int(pv.get("value", -1)) == 1:
            print("[HIL] SYS_HITL=1 already set (airframe rcS) — "
                  "no PARAM_SET needed.")
            return

        current = "unknown" if pv is None else str(int(pv.get("value", -1)))
        print(f"[HIL] SYS_HITL current value = {current} — sending "
              f"PARAM_SET to enforce SYS_HITL=1...")

        # ─── خطوة 2: SET + Retry حتى PARAM_VALUE يؤكّد =1 ────────────
        with self._param_lock:
            self._last_param_values.pop("SYS_HITL", None)

        t_end = time.monotonic() + timeout_s
        next_send = 0.0
        next_gps = 0.0
        attempts = 0
        set_ok = False
        param_set_bytes = build_param_set("SYS_HITL", 1, MAV_PARAM_TYPE_INT32)
        param_read_bytes = build_param_request_read("SYS_HITL")

        while time.monotonic() < t_end and self._running:
            now = time.monotonic()
            self._sim_t_us = int(now * 1e6)
            self._send(build_heartbeat())
            s = self._static_sensors(state)
            self._send(build_hil_sensor(
                self._sim_t_us,
                s["accel_body"], s["gyro_body"],
                s["mag_body"], s["baro_p"],
                s["diff_p"], s["pressure_alt"],
            ))
            # GPS كل 100ms للحفاظ على GPS fix أثناء retry loop (قد يمتد
            # إلى hitl_param_timeout_s=5s). بدونه، EKF2 قد يُعلن GPS fault
            # ويؤخّر تهيئة navigation estimator.
            if now >= next_gps:
                self._send(build_hil_gps(
                    self._sim_t_us,
                    self.launch_lat, self.launch_lon, self.launch_alt,
                    0, 0, 0,
                ))
                next_gps = now + 0.1

            if now >= next_send:
                if not self._send_via_timing(param_set_bytes):
                    print("[HIL] WARNING: param channel dropped during "
                          "PARAM_SET retry — aborting set loop.")
                    break
                # نُلحق PARAM_REQUEST_READ لإجبار PARAM_VALUE حتى لو
                # لم تبثّ PX4 التغيير تلقائياً (بعض إعدادات mavlink
                # تبثّ PARAM_VALUE فقط عند استفسار صريح).
                self._send_via_timing(param_read_bytes)
                attempts += 1
                next_send = now + 0.5

            self._drain_target(dt)

            with self._param_lock:
                pv = self._last_param_values.get("SYS_HITL")
            if pv is not None and int(pv.get("value", -1)) == 1:
                set_ok = True
                break

            time.sleep(0.005)

        if set_ok:
            print(f"[HIL] SYS_HITL=1 confirmed by PX4 after {attempts} "
                  f"PARAM_SET attempts. NOTE: PX4 may need a restart from "
                  f"the app to enter HIL mode.")
        else:
            print(f"[HIL] WARNING: SYS_HITL not confirmed as 1 after "
                  f"{attempts} PARAM_SET attempts on TCP "
                  f"{self._mavlink_tcp_port} (timeout {timeout_s:.1f}s). "
                  f"If abort_on_no_actuator=True, warm-up will fail unless "
                  f"airframe rcS sets SYS_HITL=1 itself.")

    def _calibrate_servo_zero(self, state: np.ndarray, dt: float) -> None:
        """معايرة صفر السيرفوهات التلقائية.

        الخطوات:
          1) استقرار: يُرسل HEARTBEAT + HIL_STATE + HIL_SENSOR ثابتة لمدة
             ``servo_zero_settle_s`` للسماح للسيرفوهات بالاستقرار والهاتف
             باستلام أوامر cmd=0 من PX4 الخامل.
          2) جمع العيّنات: يُفعَّل ``_zero_calib_active`` لمدة
             ``servo_zero_sample_s``؛ كل SRV_FB يُخزَّن خام.
          3) حساب offset = median(fb_deg_raw) لكل سيرفو؛ إذا تجاوز
             ``servo_zero_max_deg`` يُطبع تحذير ويُضبط على 0 لذلك المحور.
        """
        print(f"[HIL] Servo auto-zero calibration "
              f"(settle={self.servo_zero_settle_s:.1f}s, "
              f"sample={self.servo_zero_sample_s:.1f}s)...")

        t_start = time.monotonic()
        settle_end = t_start + self.servo_zero_settle_s
        sample_end = settle_end + self.servo_zero_sample_s

        with self._servo_fb_lock:
            self._zero_calib_samples.clear()

        while time.monotonic() < sample_end:
            now = time.monotonic()
            if not self._zero_calib_active and now >= settle_end:
                # تفعيل التقاط العيّنات داخل القفل مع مسح أي عيّنات
                # سابقة من مرحلة الاستقرار (أي `online_mask` تسرّب قبل
                # انتهاء settle_end).
                with self._servo_fb_lock:
                    self._zero_calib_active = True
                    self._zero_calib_samples.clear()

            # wall-clock موحّد عبر كل مراحل warm-up (auto-zero/param/warmup/flight).
            # الزيادة الثابتة +5ms السابقة كانت تبدأ من 0 وتُحدث قفزة هائلة عند
            # انتقال `_set_hitl_param`/warmup إلى wall-clock، ما قد يُربك EKF2.
            self._sim_t_us = int(time.monotonic() * 1e6)
            self._send(build_heartbeat())
            self._send(build_hil_state_quat(
                self._sim_t_us,
                np.array([1, 0, 0, 0]), np.zeros(3),
                self.launch_lat, self.launch_lon, self.launch_alt,
                0, 0, 0, np.array([0, 0, -9.80665]), 0.0,
            ))
            self._send(build_hil_gps(
                self._sim_t_us,
                self.launch_lat, self.launch_lon, self.launch_alt, 0, 0, 0,
            ))
            s = self._static_sensors(state)
            self._send(build_hil_sensor(
                self._sim_t_us,
                s["accel_body"], s["gyro_body"],
                s["mag_body"], s["baro_p"],
                s["diff_p"], s["pressure_alt"],
            ))
            self._drain_target(dt)
            time.sleep(0.005)

        # إنهاء الالتقاط + سحب العيّنات ذرّياً تحت القفل نفسه لمنع
        # append من خيط timing بعد تعطيل الالتقاط (race محتمل في CPython).
        with self._servo_fb_lock:
            self._zero_calib_active = False
            samples = list(self._zero_calib_samples)
            self._zero_calib_samples.clear()

        if len(samples) < 3:
            print(f"[HIL] Servo auto-zero: insufficient samples "
                  f"({len(samples)}) — offset = 0")
            self._servo_zero_offset_rad = np.zeros(4)
            return

        # فلترة per-channel حسب online_mask: قناة offline لا تُحسب في medianها
        # حتى لا تُلوّث بـ fb_deg_raw قديمة/صفرية.
        arr = np.vstack([s[0] for s in samples])            # (N, 4)
        masks = np.array([s[1] for s in samples], dtype=int)  # (N,)
        offset_deg = np.zeros(4)
        n_per_ch = np.zeros(4, dtype=int)
        for ch in range(4):
            sel = (masks & (1 << ch)) != 0
            n_per_ch[ch] = int(sel.sum())
            if n_per_ch[ch] >= 3:
                offset_deg[ch] = float(np.median(arr[sel, ch]))
            else:
                offset_deg[ch] = 0.0  # غير كافي → trust hardware zero

        safe_mask = np.abs(offset_deg) <= self.servo_zero_max_deg
        if not np.all(safe_mask):
            bad = [i + 1 for i, ok in enumerate(safe_mask) if not ok]
            print(f"[HIL] WARNING: servo(s) {bad} exceed "
                  f"{self.servo_zero_max_deg:.1f}° — rejected for zero-offset")
            offset_deg = np.where(safe_mask, offset_deg, 0.0)

        self._servo_zero_offset_rad = np.radians(offset_deg)

        per_ch_std = np.array([
            float(arr[(masks & (1 << ch)) != 0, ch].std())
            if n_per_ch[ch] >= 3 else 0.0
            for ch in range(4)
        ])
        spread_deg = float(np.max(per_ch_std))
        print(f"[HIL] Servo zero-offset [deg] = "
              f"[{offset_deg[0]:+6.2f}, {offset_deg[1]:+6.2f}, "
              f"{offset_deg[2]:+6.2f}, {offset_deg[3]:+6.2f}]  "
              f"(samples={len(samples)}, n_per_ch={n_per_ch.tolist()}, "
              f"max_std={spread_deg:.3f}°)")

    def _handle_debug_float_array(self, payload: bytes):
        """يعالج SRV_FB (array_id=1) من درايفر xqpower_can، و
        RktGNC (array_id=2) من rocket_mpc — يحوي حقول التوقيت (µs)
        كمصدر أساسي بدلاً من DEBUG_VECT("TIMING") الذي أُزيل لتفادي
        الاصطدام مع mavlink_receiver."""
        p = parse_debug_float_array(payload)
        if p is None:
            return

        # RktGNC (array_id=2): حقول التوقيت في slots [46..48]
        #   data[46] = mhe_solve_us
        #   data[47] = mpc_solve_us
        #   data[48] = cycle_us
        # هذه القيم تصل بمعدل 20 Hz في MAVLINK_MODE_ROCKET، كافية
        # لتتبُّع تشغيل الحلّ لكن ليست بتفاصيل الـDEBUG_VECT السابق.
        if p["array_id"] == 2 and p["name"].upper().startswith("RKTGNC"):
            data = p["data"]
            try:
                mhe_us = float(data[46])
                mpc_us = float(data[47])
                cycle_us = float(data[48])
            except (IndexError, TypeError, ValueError):
                return

            # نتجاهل العيّنات التي تكون كل حقولها صفراً (لم يحصل solve بعد)
            if mhe_us == 0.0 and mpc_us == 0.0 and cycle_us == 0.0:
                return

            with self._timing_lock:
                self._last_timing["mhe_us"] = mhe_us
                self._last_timing["mpc_us"] = mpc_us
                self._last_timing["cycle_us"] = cycle_us
                self.timing["t_sim"].append(self._sim_t_us / 1e6)
                self.timing["mhe_us"].append(mhe_us)
                self.timing["mpc_us"].append(mpc_us)
                self.timing["cycle_us"].append(cycle_us)
            return

        # الاصطلاح: array_id=1, name="SRV_FB"
        if p["array_id"] != 1 or not p["name"].upper().startswith("SRV_FB"):
            return
        data = p["data"]
        cmd_deg = np.array(data[0:4], dtype=float)
        fb_deg_raw = np.array(data[4:8], dtype=float)
        online_mask = int(data[12]) & 0xFF
        tx_fail = int(data[13])

        # فحص حالة المعايرة + أي كتابة لحالة السيرفو يتم داخل قفل
        # واحد لتجنّب TOCTOU: لو فحصنا ``_zero_calib_active`` خارج القفل
        # ثم عاد المعايِر وألغاها ونسخ العيّنات قبل أن نُحرز القفل، كان
        # append سيذهب إلى list مُفرَّغة (العيّنة تُفقد) أو نرجع مبكراً
        # ولا نُحدّث فيدباك flight loop رغم انتهاء المعايرة.
        zero_off_deg = np.degrees(self._servo_zero_offset_rad)
        fb_deg = fb_deg_raw - zero_off_deg
        err_deg = cmd_deg - fb_deg
        fb_mono_ns = time.monotonic_ns()
        with self._servo_fb_lock:
            if self._zero_calib_active:
                # أثناء المعايرة: اجمع العيّنات الخام (قبل أي طرح offset)
                # مع قناع online_mask الحالي لاستبعاد القنوات offline لاحقاً per-channel.
                self._zero_calib_samples.append(
                    (fb_deg_raw.copy(), online_mask & 0x0F)
                )
                self._servo_online_mask = online_mask
                self._servo_tx_fail = tx_fail
                return
            # في الوضع العادي: طرح offset ثم حساب err بعد المعايرة
            self._servo_cmd_fb_rad = np.radians(cmd_deg)
            self._servo_fb_rad = np.radians(fb_deg)
            self._servo_fb_t_us = self._sim_t_us   # للحفاظ على التوافق (يستخدم للعرض/CSV)
            self._servo_fb_mono_ns = fb_mono_ns    # مصدر العمر الموثوق
            self._servo_online_mask = online_mask
            self._servo_tx_fail = tx_fail
            self._servo_fb_count += 1
            self.servo_log["t_sim"].append(self._sim_t_us / 1e6)
            self.servo_log["cmd_deg"].append(cmd_deg.copy())
            self.servo_log["fb_deg"].append(fb_deg.copy())
            self.servo_log["err_deg"].append(err_deg.copy())
            self.servo_log["online_mask"].append(online_mask)
            self.servo_log["tx_fail"].append(tx_fail)


    # ─── تسجيل وتصدير ───────────────────────────────────────────────────────

    def _log(self, snap, state, t, s):
        # CSV columns pos_x/y/z and vel_x/y/z are always launch-relative NED
        # for consistency with analysis labels. In long_range_mode,
        # state[0:3] / state[3:6] are ECEF, so we use snapshot
        # pos_ned_launch / vel_ned_launch which are already in launch-fixed NED.
        # Raw ECEF is recoverable from lat/lon/alt_msl columns when needed.
        if self.sim.long_range_mode:
            pos_log = np.asarray(snap.get("pos_ned_launch", state[0:3]))
            vel_log = np.asarray(snap.get("vel_ned_launch", state[3:6]))
        else:
            pos_log = state[0:3]
            vel_log = state[3:6]
        self.flight["time"].append(t)
        self.flight["pos"].append(pos_log.copy())
        self.flight["vel"].append(vel_log.copy())
        self.flight["quat"].append(state[6:10].copy())
        self.flight["omega"].append(state[10:13].copy())
        self.flight["mass"].append(state[13] if len(state) > 13 else 0.0)
        self.flight["fin_cmd"].append(self._last_controls[:4].copy())
        self.flight["fin_act"].append(
            np.array(snap.get("control_fins_rad", np.zeros(4)))
        )
        self.flight["fin_can"].append(self._fin_can_rad.copy())
        self.flight["fin_source"].append(self._fin_source)
        self.flight["forces"].append(np.array(snap.get("forces", [0, 0, 0])))
        # Store MSL altitude (matches PIL bridge convention).  hil_analysis.py
        # converts MSL → AGL by subtracting LAUNCH_ALT_M; subtracting here too
        # double-counts and produces nonsensical "Peak Alt = -1100 m AGL"
        # readings.  Bridge writes raw MSL, analysis owns the AGL conversion.
        self.flight["altitude"].append(s["alt"])
        pos = state[0:3]
        if self.sim.long_range_mode:
            rng = snap.get("ground_range_km", 0.0) * 1000.0
        else:
            rng = float(np.sqrt(pos[0] ** 2 + pos[1] ** 2))
        self.flight["ground_range"].append(rng)
        self.flight["alpha"].append(snap.get("alpha", 0.0))
        self.flight["beta"].append(snap.get("beta", 0.0))
        self.flight["mach"].append(snap.get("mach", 0.0))
        self.flight["lat"].append(s["lat"])
        self.flight["lon"].append(s["lon"])
        self.flight["alt_msl"].append(s["alt"])

    def _export_flight_csv(self, path: str):
        n = len(self.flight["time"])
        if n == 0:
            return
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        pos = np.array(self.flight["pos"])
        vel = np.array(self.flight["vel"])
        q = np.array(self.flight["quat"])
        w = np.array(self.flight["omega"])
        fc = np.array(self.flight["fin_cmd"])
        fa = np.array(self.flight["fin_act"])
        fcan = np.array(self.flight["fin_can"])
        fr = np.array(self.flight["forces"])
        with open(path, "w", newline="", encoding="utf-8") as f:
            wr = csv.writer(f)
            wr.writerow([
                "time", "pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z",
                "q0", "q1", "q2", "q3", "omega_x", "omega_y", "omega_z",
                "mass", "altitude", "ground_range",
                "alpha", "beta", "mach",
                "fin_cmd_1", "fin_cmd_2", "fin_cmd_3", "fin_cmd_4",
                "fin_act_1", "fin_act_2", "fin_act_3", "fin_act_4",
                "fin_can_1", "fin_can_2", "fin_can_3", "fin_can_4",
                "fin_source",
                "force_x", "force_y", "force_z",
                "lat", "lon", "alt_msl",
            ])
            for i in range(n):
                wr.writerow([
                    f"{self.flight['time'][i]:.6f}",
                    f"{pos[i, 0]:.6f}", f"{pos[i, 1]:.6f}", f"{pos[i, 2]:.6f}",
                    f"{vel[i, 0]:.6f}", f"{vel[i, 1]:.6f}", f"{vel[i, 2]:.6f}",
                    f"{q[i, 0]:.8f}", f"{q[i, 1]:.8f}", f"{q[i, 2]:.8f}", f"{q[i, 3]:.8f}",
                    f"{w[i, 0]:.8f}", f"{w[i, 1]:.8f}", f"{w[i, 2]:.8f}",
                    f"{self.flight['mass'][i]:.4f}",
                    f"{self.flight['altitude'][i]:.4f}",
                    f"{self.flight['ground_range'][i]:.4f}",
                    f"{self.flight['alpha'][i]:.8f}",
                    f"{self.flight['beta'][i]:.8f}",
                    f"{self.flight['mach'][i]:.6f}",
                    f"{fc[i, 0]:.8f}", f"{fc[i, 1]:.8f}", f"{fc[i, 2]:.8f}", f"{fc[i, 3]:.8f}",
                    f"{fa[i, 0]:.8f}", f"{fa[i, 1]:.8f}", f"{fa[i, 2]:.8f}", f"{fa[i, 3]:.8f}",
                    f"{fcan[i, 0]:.8f}", f"{fcan[i, 1]:.8f}", f"{fcan[i, 2]:.8f}", f"{fcan[i, 3]:.8f}",
                    self.flight["fin_source"][i],
                    f"{fr[i, 0]:.4f}", f"{fr[i, 1]:.4f}", f"{fr[i, 2]:.4f}",
                    f"{self.flight['lat'][i]:.8f}",
                    f"{self.flight['lon'][i]:.8f}",
                    f"{self.flight['alt_msl'][i]:.4f}",
                ])

    def _export_timing_csv(self, path: str):
        n = len(self.timing["t_sim"])
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w", newline="", encoding="utf-8") as f:
            wr = csv.writer(f)
            wr.writerow(["t_sim", "mhe_us", "mpc_us", "cycle_us"])
            for i in range(n):
                wr.writerow([
                    f"{self.timing['t_sim'][i]:.6f}",
                    f"{self.timing['mhe_us'][i]:.3f}",
                    f"{self.timing['mpc_us'][i]:.3f}",
                    f"{self.timing['cycle_us'][i]:.3f}",
                ])
        if n == 0:
            print("[HIL] WARNING: no timing samples received — "
                  "verify target publishes DEBUG_VECT 'TIMING' or "
                  "NAMED_VALUE_FLOAT (mhe_us/mpc_us/cycle_us).")

    def _export_servo_csv(self, path: str):
        """يُصدّر سجلّ فيدباك السيرفو (SRV_FB) — جوهر حلقة HIL المغلقة."""
        n = len(self.servo_log["t_sim"])
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w", newline="", encoding="utf-8") as f:
            wr = csv.writer(f)
            wr.writerow([
                "t_sim",
                "cmd_1_deg", "cmd_2_deg", "cmd_3_deg", "cmd_4_deg",
                "fb_1_deg",  "fb_2_deg",  "fb_3_deg",  "fb_4_deg",
                "err_1_deg", "err_2_deg", "err_3_deg", "err_4_deg",
                "online_mask", "tx_fail",
            ])
            for i in range(n):
                cmd = self.servo_log["cmd_deg"][i]
                fb = self.servo_log["fb_deg"][i]
                er = self.servo_log["err_deg"][i]
                wr.writerow([
                    f"{self.servo_log['t_sim'][i]:.6f}",
                    f"{cmd[0]:.4f}", f"{cmd[1]:.4f}", f"{cmd[2]:.4f}", f"{cmd[3]:.4f}",
                    f"{fb[0]:.4f}",  f"{fb[1]:.4f}",  f"{fb[2]:.4f}",  f"{fb[3]:.4f}",
                    f"{er[0]:.4f}",  f"{er[1]:.4f}",  f"{er[2]:.4f}",  f"{er[3]:.4f}",
                    self.servo_log["online_mask"][i],
                    self.servo_log["tx_fail"][i],
                ])



# ============================================================================
# CLI
# ============================================================================

def main():
    ap = argparse.ArgumentParser(description="M130 HIL MAVLink Bridge (standalone)")
    ap.add_argument("--config", default=str(_SCRIPT_DIR / "hil_config.yaml"))
    ap.add_argument("--csv", default=None)
    ap.add_argument("--timing-csv", default=None)
    ap.add_argument("--host", default=None, help="Override listen host")
    ap.add_argument("--port", type=int, default=None, help="Override listen port")
    args = ap.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )

    bridge = HILBridge(args.config)
    if args.host:
        bridge.host = args.host
    if args.port:
        bridge.port = args.port

    out = bridge.cfg.get("output", {})
    results_dir = _SCRIPT_DIR / "results"
    flight_csv = args.csv or str(results_dir / out.get("csv_name", "hil_flight.csv"))
    timing_csv = args.timing_csv or str(results_dir / out.get("timing_csv", "hil_timing.csv"))
    bridge.run(flight_csv, timing_csv)


if __name__ == "__main__":
    main()
