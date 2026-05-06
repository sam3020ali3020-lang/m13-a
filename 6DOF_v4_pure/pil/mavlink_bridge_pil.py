#!/usr/bin/env python3
"""
PIL MAVLink Bridge — جسر مستقل لاختبار Processor-In-the-Loop
================================================================

مستقل تماماً عن جسر SITL. مخصّص للاتصال بجهاز ARM64 خارجي (Android/Jetson)
عبر TCP على واجهة شبكة حقيقية.

الاختلافات الجوهرية عن جسر SITL:
  • يستمع على 0.0.0.0 (أو أي IP) بدل 127.0.0.1
  • وضع realtime فقط — لا lockstep عبر الشبكة (jitter غير مقبول)
  • لا توجد عمليّات ARM عبر UDP محلي — الهدف يُهيّأ مسبقاً
  • يقرأ رسائل التوقيت DEBUG_VECT ("TIMING") / NAMED_VALUE_FLOAT
    ويحفظها في pil_timing.csv
  • تهيئة مبسّطة للإقلاع (warm-up أقصر، مناسب لعتاد فعلي جاهز)
"""

from __future__ import annotations

import argparse
import csv
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

logger = logging.getLogger("pil_bridge")


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
MSG_DEBUG_FLOAT_ARRAY = 350  # RktGNC (array_id=2) من rocket_mpc

CRC_HEARTBEAT = 50
CRC_HIL_SENSOR = 108
CRC_HIL_GPS = 124
CRC_HIL_STATE_QUATERNION = 4
CRC_DEBUG_FLOAT_ARRAY = 232

SYS_ID = 1
COMP_ID = 1
GCS_SYS_ID = 255
GCS_COMP_ID = 190

_INT16_MIN = -32768
_INT16_MAX = 32767
_UINT16_MAX = 65535
_INT32_MIN = -2147483648
_INT32_MAX = 2147483647

# تسجيل مرّة واحدة فقط لتفادي إغراق الطرفية بنفس تحذير saturation
_warned: dict = {}


def _clip_scaled(val, lo: int, hi: int, name: str) -> int:
    """قصّ قيمة scaled-int إلى نطاق الحقل MAVLink مع تحذير لمرة واحدة.

    النطاقات الشائعة:
      • int16  : [-32768, 32767] → سرعة cm/s ≤ ±327.67 m/s
      • uint16 : [0, 65535]      → airspeed cm/s ≤ 655.35 m/s
    بدون هذا القصّ، ``struct.pack`` يرمي ``struct.error`` عند تجاوز النطاق
    ويُنهي الجسر فجأة وسط الرحلة لصواريخ عالية السرعة.
    """
    v = int(val)
    if v < lo or v > hi:
        if name not in _warned:
            print(f"[PIL] WARNING: MAVLink field {name!r} saturated "
                  f"({v} out of [{lo},{hi}]) — clamping.")
            _warned[name] = True
        v = max(lo, min(hi, v))
    return v


def _x25_crc(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        crc &= 0xFFFF
    return crc


def _pack_v2(msg_id: int, payload: bytes, crc_extra: int,
             sys_id: int = SYS_ID, comp_id: int = COMP_ID,
             seq: list = [0]) -> bytes:
    trimmed = payload.rstrip(b"\x00")
    # قاعدة MAVLink v2 empty-byte truncation تتطلب بقاء بايت واحد على الأقل
    # حتى لو كان كامل payload أصفاراً. بدون هذا الضمان قد نُرسل رسالة
    # بطول 0 ثم فك التحزيم في PX4 يفشل.
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


# ─── Builders ────────────────────────────────────────────────────────────────

def build_heartbeat() -> bytes:
    payload = struct.pack("<IBBBBB", 0, 6, 8, 0, 4, 3)
    return _pack_v2(MSG_HEARTBEAT, payload, CRC_HEARTBEAT,
                    sys_id=GCS_SYS_ID, comp_id=GCS_COMP_ID)


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
    # HIL_GPS unpacker `<QiiiHHHhhhHBB` (يطابق hil/mavlink_bridge_hil.py).
    # نقصّ lat/lon/alt إلى نطاق int32 لتجنّب struct.error عند tumbling/blow-up
    lat_e7 = _clip_scaled(lat * 1e7, _INT32_MIN, _INT32_MAX, "hil_gps.lat")
    lon_e7 = _clip_scaled(lon * 1e7, _INT32_MIN, _INT32_MAX, "hil_gps.lon")
    alt_mm = _clip_scaled(alt_m * 1000, _INT32_MIN, _INT32_MAX, "hil_gps.alt")
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
        _clip_scaled(lat * 1e7, _INT32_MIN, _INT32_MAX, "hil_state.lat"),
        _clip_scaled(lon * 1e7, _INT32_MIN, _INT32_MAX, "hil_state.lon"),
        _clip_scaled(alt_m * 1000, _INT32_MIN, _INT32_MAX, "hil_state.alt"),
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


def build_srv_fb_debug_array(t_us: int, fins_deg, mask: int = 0x0F) -> bytes:
    """DEBUG_FLOAT_ARRAY (msg 350) with id=1, name="SRV_FB" — mimics
    xqpower_can SRV_FB feedback so PX4's RocketMPC can read actual fin
    positions instead of falling back to first-order filter approximation.

    Format expected by RocketMPC.cpp (lines 1163-1180):
      data[0..3]   = unused
      data[4..7]   = fin angles in DEGREES (fin1, fin2, fin3, fin4)
      data[8..11]  = unused
      data[12]     = online_mask (uint8 stored as float; 0x0F = all 4 online)
      data[13..57] = unused

    Wire layout for DEBUG_FLOAT_ARRAY (MAVLink v2 size-sorted):
      time_usec(u64=8) + array_id(u16=2) + name[10] + data[58 floats] = 252 bytes
    """
    data = [0.0] * 58
    for i in range(min(4, len(fins_deg))):
        data[4 + i] = float(fins_deg[i])
    data[12] = float(mask)
    name_bytes = b"SRV_FB".ljust(10, b"\x00")
    payload = struct.pack("<Q", int(t_us))
    payload += struct.pack("<H", 1)  # array_id=1 (matches xqpower_can)
    payload += name_bytes
    payload += struct.pack("<58f", *data)
    return _pack_v2(MSG_DEBUG_FLOAT_ARRAY, payload, CRC_DEBUG_FLOAT_ARRAY)


# ─── Parsers ─────────────────────────────────────────────────────────────────

def parse_actuator_controls(payload: bytes) -> Optional[dict]:
    """HIL_ACTUATOR_CONTROLS (msg 93): time_usec(u64) + flags(u64) + 16 floats + mode(u8).

    MAVLink v2 قد يقلّص الأصفار اللاحقة على حدود بايت (ليس حقل)، لذلك
    نُبَطِّن إلى الحجم الكامل (81 بايت) قبل فك التحزيم. الإصدار السابق كان
    يعتمد على fallback يحسب ``n = (len-16)//4`` ويتجاهل bytes تقصير غير
    مُحاذية (مثل تقصير وسط float)، فكان يعيد قيم أصفار مزيّفة بدلاً من
    القيم الحقيقية للتحكم. باستخدام تبطين صريح نقرأ الصيغة الكاملة دائماً.
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
    """NAMED_VALUE_FLOAT (msg 251) wire order (size-sorted):
        time_boot_ms(u32) + value(f32) + name[10]
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
    """DEBUG_VECT (msg 250) wire order (size-sorted, NOT XML order):
        time_usec(u64) + x(f32) + y(f32) + z(f32) + name[10]
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
        time_usec(u64) + array_id(u16) + name[10] + data[58](f32 extension)
    الحجم الكامل = 252 بايت. MAVLink v2 يحذف الأصفار اللاحقة.

    rocket_mpc يبثّ array_id=2 name="RktGNC". حقول التوقيت في:
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

# WGS84 constants (used for small-NED → LLA conversion fallback)
_WGS84_A = 6378137.0          # semi-major axis (m)
_WGS84_F = 1.0 / 298.257223563  # flattening
_WGS84_E2 = _WGS84_F * (2.0 - _WGS84_F)  # first eccentricity squared


def _ned_to_lla(pos, lat0, lon0, alt0):
    """تحويل NED محلي → LLA باستخدام أنصاف أقطار تقوس WGS84.

    الصيغة السابقة استخدمت نصف قطر ثابت 6371 km بدون تسطيح، مما يُراكم
    خطأً ملموساً لمديات > 100 km (يدخل مباشرة إلى HIL_GPS ويُغذّي EKF2).
    هنا نستخدم meridian (M) و prime-vertical (N) radii الفعليين عند lat0:
        M = a(1-e²)/(1-e²sin²lat)^(3/2)     → يقوّم dlat من dN
        N = a/(1-e²sin²lat)^(1/2)            → يقوّم dlon من dE
    """
    lat0_rad = np.radians(lat0)
    sin_lat = np.sin(lat0_rad)
    cos_lat = np.cos(lat0_rad)
    w = np.sqrt(max(1.0 - _WGS84_E2 * sin_lat * sin_lat, 1e-12))
    R_meridian = _WGS84_A * (1.0 - _WGS84_E2) / (w ** 3)
    R_normal = _WGS84_A / w
    lat = lat0 + np.degrees(pos[0] / R_meridian)
    lon = lon0 + np.degrees(pos[1] / (R_normal * max(cos_lat, 1e-9)))
    alt = alt0 - pos[2]
    return lat, lon, alt


def _body_specific_force(f_body, mass, g_ned, quat):
    """Specific force in body frame (what an IMU accelerometer measures).

    f_body = thrust + aero (from 6DOF sim, NO gravity).
    Newton: a_inertial = F_ext/m + g   →   specific_force = a - g = F_ext/m.
    Mirrors SITL bridge `_body_specific_force` (sitl/mavlink_bridge.py:436-441).
    """
    del g_ned, quat  # kept for API symmetry with previous signature
    return f_body / max(mass, 0.1)


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
# الغرض: جعل PIL يختبر EKF2 مع ضجيج واقعي من bridge.noise في الـ config
# بدل ضجيج hardcoded مثالي — كان السبب الأساسي لعدم كشف مشاكل biases/drift
# التي تظهر على ARM64 مع hardware. كل الخيارات تُقرأ من noise_cfg dict؛
# غير المُعرَّف يرجع لقيم افتراضية تُطابق SITL تماماً.
# نُسخت الفئة بدل import من sitl للحفاظ على استقلال الجسور (كما في HIL).


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
# جسر PIL
# ============================================================================

class PILBridge:
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
        self.duration = float(scenario.get("duration_s", 500.0))
        self.dt = float(scenario.get("dt_s", 0.01))
        self.seed = int(scenario.get("random_seed", 42))

        timing_cfg = self.cfg.get("timing", {})
        self.timing_enabled = bool(timing_cfg.get("enabled", True))
        self.deadline_us = int(timing_cfg.get("deadline_us", 20000))

        # Clock mode: "realtime" (default, wall-clock pacing) أو "lockstep"
        # (ننتظر HIL_ACTUATOR_CONTROLS قبل التقدّم — يُحاكي Gazebo lockstep
        # لـ SITL). في lockstep، الرحلة تُقاس بـ MPC throughput لا wall-clock،
        # وتُزال قطعة الأثر الناتجة عن TCP/scheduler jitter.
        clock_cfg = self.cfg.get("clock", {})
        self.clock_mode = str(clock_cfg.get("mode", "realtime")).lower()
        # في lockstep: نُسَنكِن كل `mpc_cycle_hz` مع response من PX4.
        # 25Hz يطابق rocket_mpc MPC rate الافتراضي (sensor_combined @ 100Hz
        # مقسوم على 4). يمكن رفعه لـ 50Hz إذا MPC rate أُبقي 50Hz.
        self.mpc_cycle_hz = float(clock_cfg.get("mpc_cycle_hz", 25.0))
        # timeout لكل sync point — إذا MPC فشل يُحل خلال هذا الوقت نتابع بـ
        # last actuator (fallback آمن).
        self.lockstep_timeout_s = float(clock_cfg.get("lockstep_timeout_s", 1.0))
        # realtime_pacing: hybrid lockstep — بعد استلام actuator، ننام حتى
        # wall-clock يلحق sim time. MPC solve يأكل من الـ budget واقعياً.
        # راجع pil_config.yaml للتفاصيل. (الاسم القديم: inject_mpc_delay)
        self.realtime_pacing = bool(
            clock_cfg.get("realtime_pacing",
                          clock_cfg.get("inject_mpc_delay", False))
        )

        warmup_cfg = self.cfg.get("warmup", {})
        self.warmup_duration_s = float(warmup_cfg.get("duration_s", 15.0))
        self.settle_after_arm_s = float(warmup_cfg.get("settle_after_arm_s", 2.0))
        # إذا لم تصل أي HIL_ACTUATOR_CONTROLS خلال warm-up، أفشل صراحة
        # (PX4 على الأغلب ليس في HIL mode: SYS_HITL=0 أو لم يُقلع EKF2 بعد).
        self.abort_on_no_actuator = bool(warmup_cfg.get("abort_on_no_actuator", True))

        # تحميل إعدادات المحاكاة وضبط وضع "none" (لا تحكم داخلي)
        with open(sim_cfg_path, "r", encoding="utf-8") as f:
            sim_cfg = yaml.safe_load(f)
        sim_cfg["simulation"]["control_type"] = "none"
        sim_cfg.setdefault("estimation", {})["mode"] = "off"
        # ملاحظة: سابقاً كان يُصفَّر angular_velocity هنا "لمطابقة baseline".
        # لكن هذا يلغي أي initial conditions من config، فاختبارات yaw rate
        # وغيرها لم تكن تُطبَّق فعلاً. الآن نحترم config.

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

        self.launch_lat = sim_cfg.get("launch", {}).get("latitude", 16.457472)
        self.launch_lon = sim_cfg.get("launch", {}).get("longitude", 44.115361)
        self.launch_alt = sim_cfg.get("launch", {}).get("altitude", 1200.0)

        self.rng = np.random.default_rng(self.seed)

        # ─── Noise generator ─────────────────────────────────────────────
        # يقرأ من bridge.noise في sim config (نفس مصدر SITL/HIL).
        # سابقاً كان PIL يستخدم hardcoded σ=0.1/0.002 فقط للـ IMU ويتجاهل
        # mag/baro/GPS noise تماماً — بيانات مثالية غير واقعية تخفي مشاكل
        # EKF2 biases/drift التي تظهر على ARM64. مع هذا التعديل يصبح PIL
        # يُحاكي نفس ضجيج SITL (accel/gyro biases ثابتة + mag/baro + GPS delay).
        noise_cfg = sim_cfg.get("bridge", {}).get("noise", {})
        self.noise = SensorNoise({"noise": noise_cfg}, rng_seed=self.seed)

        # الشبكة
        self._sock: Optional[socket.socket] = None
        self._parser = MavParser()

        # الحالة
        self._sim_t_us = 0
        self._fins_rad = np.zeros(4)
        self._last_controls = np.zeros(16)
        # عدّاد رسائل HIL_ACTUATOR_CONTROLS المستلمة — يُستخدم كـproxy لـاستعداد
        # الهدف في warm-up (أول رسالة = PX4 سلّم نفسه وفي HIL mode).
        self._actuator_msg_count = 0
        self._running = False

        # ── Compute-delay injection (يجعل lockstep واقعي) ──
        # قراءة العتبة من config: 0 = معطّل، >0 = حقن mpc_us كتأخير على الـactuator.
        self._compute_delay_max_us = int(
            clock_cfg.get("inject_compute_delay_max_ms", 0) * 1000
        )
        # طابور (apply_at_sim_us, fins[16]) — الـactuator يُطبَّق فقط
        # عندما sim_T >= apply_at_sim_us (يحاكي تأخير CPU الحقيقي).
        self._cmd_queue: list = []
        # آخر controls مُطبَّقة (يبقى قائماً حتى يستحقّ أمر جديد)
        self._active_controls = np.zeros(16)

        # سجلّ الرحلة
        self.flight = {
            "time": [], "pos": [], "vel": [], "quat": [], "omega": [],
            "mass": [], "fin_cmd": [], "fin_act": [], "forces": [],
            "altitude": [], "ground_range": [], "alpha": [], "beta": [], "mach": [],
            "lat": [], "lon": [], "alt_msl": [],
        }

        # سجلّ التوقيت
        self.timing = {
            "t_sim": [], "mhe_us": [], "mpc_us": [], "cycle_us": [],
        }
        self._last_timing = {"mhe_us": 0.0, "mpc_us": 0.0, "cycle_us": 0.0}
        # سجلّ MHE diagnostics (من DEBUG_FLOAT_ARRAY RktGNC)
        self.mhe_log = {
            "t_sim": [], "quality": [], "valid": [], "status": [],
            "mpc_solver_status": [], "mpc_sqp_iter": [],
        }
        self._timing_lock = threading.Lock()

        # قناة MAVLink ثانوية لالتقاط DEBUG_VECT من mavlink الكامل عبر TCP 5760
        # (simulator_mavlink TCP 4560 يبثّ فقط HIL_* و HIL_ACTUATOR_CONTROLS)
        self._mavlink_tcp_host = tgt.get("mavlink_tcp", {}).get("host", "127.0.0.1")
        self._mavlink_tcp_port = int(tgt.get("mavlink_tcp", {}).get("port", 5760))
        self._timing_thread: Optional[threading.Thread] = None
        self._timing_stop = threading.Event()

    def __del__(self):
        tmp = getattr(self, "_tmp", None)
        if tmp is None:
            return
        try:
            os.unlink(tmp.name)
        except (OSError, AttributeError):
            pass

    # ─── الشبكة ──────────────────────────────────────────────────────────────

    def accept(self):
        """انتظار اتصال PX4 من الجهاز الهدف. يرفع RuntimeError عند انتهاء المهلة."""
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            srv.bind((self.host, self.port))
        except OSError as e:
            srv.close()
            raise RuntimeError(
                f"[PIL] Failed to bind {self.host}:{self.port}: {e}"
            ) from e
        srv.listen(1)
        srv.settimeout(self.accept_timeout_s)
        print(f"[PIL] Listening on TCP {self.host}:{self.port} — "
              f"start PX4 on target device (timeout={self.accept_timeout_s:.0f}s)...")
        try:
            self._sock, addr = srv.accept()
        except socket.timeout as e:
            srv.close()
            raise RuntimeError(
                f"[PIL] No target connected within {self.accept_timeout_s:.0f}s. "
                "تحقّق أن PX4 مُقلع على الجهاز الهدف وأن عنوان الشبكة صحيح."
            ) from e
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        # نُبقي المقبس في وضع blocking افتراضياً لأن _send يستخدم sendall()
        # الذي يرمي BlockingIOError على non-blocking socket حين يمتلئ TCP
        # send buffer. التبديل إلى non-blocking يحدث داخل _recv_nonblock فقط.
        srv.close()
        print(f"[PIL] Target connected from {addr}")

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
                print(f"[PIL] Timing reader connected to "
                      f"{self._mavlink_tcp_host}:{self._mavlink_tcp_port}")
                break
            except (OSError, socket.timeout):
                if sock:
                    sock.close()
                    sock = None
                time.sleep(0.5)
        if sock is None:
            print("[PIL] WARNING: timing reader could not connect — "
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
                    if msg_id == MSG_DEBUG_FLOAT_ARRAY:
                        # RktGNC (array_id=2, name="RktGNC") — المصدر الأساسي
                        # للتوقيت بعد حذف debug_vect TIMING من rocket_mpc
                        # (لتفادي الاصطدام مع mavlink_receiver).
                        p = parse_debug_float_array(payload)
                        if (p and p["array_id"] == 2
                                and p["name"].upper().startswith("RKTGNC")):
                            data_arr = p["data"]
                            try:
                                # Timing fields (broken in lockstep: hrt=sim)
                                mhe_us = float(data_arr[46])
                                mpc_us = float(data_arr[47])
                                cycle_us = float(data_arr[48])
                                # MHE diagnostics (صحيحة حتى في lockstep)
                                mhe_quality = float(data_arr[20])
                                mhe_valid = float(data_arr[40])
                                mhe_status = float(data_arr[41])
                                mpc_solver_status = float(data_arr[38])
                                mpc_sqp_iter = float(data_arr[39])
                            except (IndexError, TypeError, ValueError):
                                continue
                            with self._timing_lock:
                                # MHE log (كل رسالة RktGNC)
                                self.mhe_log["t_sim"].append(self._sim_t_us / 1e6)
                                self.mhe_log["quality"].append(mhe_quality)
                                self.mhe_log["valid"].append(mhe_valid)
                                self.mhe_log["status"].append(mhe_status)
                                self.mhe_log["mpc_solver_status"].append(mpc_solver_status)
                                self.mhe_log["mpc_sqp_iter"].append(mpc_sqp_iter)
                                # Timing (قد يكون 0 في lockstep)
                                if mhe_us > 0 or mpc_us > 0 or cycle_us > 0:
                                    self._last_timing["mhe_us"] = mhe_us
                                    self._last_timing["mpc_us"] = mpc_us
                                    self._last_timing["cycle_us"] = cycle_us
                    elif msg_id == MSG_DEBUG_VECT:
                        # احتياط خلفي: نبقي القراءة لتوافق مع أي بنية ثابتة
                        # (firmware قديم لا يزال يبث TIMING).
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
            except socket.timeout:
                pass
            except (OSError, ConnectionResetError):
                break
            if time.monotonic() - last_stat >= 3.0:
                top = sorted(msg_counts.items(), key=lambda kv: -kv[1])[:8]
                print(f"[PIL-timing] rx={total_rx}B msgs={top}")
                last_stat = time.monotonic()
        try:
            sock.close()
        except OSError:
            pass

    def _send(self, data: bytes):
        if self._sock is None:
            return
        try:
            self._sock.sendall(data)
        except (BrokenPipeError, ConnectionResetError, OSError) as e:
            print(f"[PIL] Target disconnected on send: {e}")
            self._running = False

    def _recv_nonblock(self) -> list:
        # نُبدّل الـsocket إلى non-blocking لقراءة واحدة فقط ثم نُعيده إلى
        # blocking. الإبقاء على blocking ضروري لـ sendall() في _send الذي يرمي
        # BlockingIOError على non-blocking socket حين يمتلئ TCP send buffer
        # (مطابق لنمط hil/mavlink_bridge_hil.py::_recv_nonblock).
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
        # واقعي + biases ثابتة لكل run. GPS noise + delay يُطبَّقان في
        # _run_impl (لحظة البناء) لأنهم يحتاجون t_usec المُحدَّث.
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
        self.accept()
        self._running = True

        sim = self.sim
        dt = self.dt
        n_steps = int(self.duration / dt)

        state = sim._initialize_state()
        sim._update_last_valid_quaternion(state[6:10])

        def _ctrl(_state_dict, _t):
            return self._fins_rad.copy()

        # warm-up: إرسال HEARTBEAT وحسّاسات ثابتة حتّى نستقبل أول HIL_ACTUATOR_CONTROLS
        # من PX4 (يدلّ على تسليم HIL mode وتقارب EKF2)، ثم ننتظر settle_after_arm_s
        # إضافية لاستقرار الحالة، ثم نبدأ حلقة الرحلة.
        # Pad forces: launch-rail reaction = -m·g_body. With specific_force = F_ext/m
        # (post-fix), this gives accel_body = -C·g_ned ≡ IMU output for stationary
        # body, enabling EKF2 tilt alignment. Parity with SITL bridge.
        q0, q1, q2, q3 = state[6:10]
        C_ned2b = np.array([
            [1 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 + q0 * q3), 2 * (q1 * q3 - q0 * q2)],
            [2 * (q1 * q2 - q0 * q3), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 + q0 * q1)],
            [2 * (q1 * q3 + q0 * q2), 2 * (q2 * q3 - q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2)],
        ])
        pad_mass = state[13] if len(state) > 13 else 12.74
        pad_forces = -pad_mass * (C_ned2b @ np.array([0.0, 0.0, 9.80665]))
        init_sensors = self._sensors(
            {"forces": pad_forces, "vel_ned": [0, 0, 0]}, state
        )
        # Warm-up: حتمي (lockstep) — عدد ثابت من الخطوات بدلاً من wall-clock.
        # يضمن أن EKF2 يستقبل نفس العدد الدقيق من الرسائل في كل تشغيل،
        # فتصبح حالته عند بدء الرحلة متطابقة → نتائج حتمية.
        wu_max_steps = int(self.warmup_duration_s / dt)
        settle_steps = int(self.settle_after_arm_s / dt)
        print(f"[PIL] Warm-up (lockstep, up to {wu_max_steps} steps): "
              "waiting for target arm + EKF convergence...")
        wu_start = time.monotonic()
        armed = False
        settle_remaining = settle_steps
        wu_step = 0
        while wu_step < wu_max_steps and self._running:
            self._sim_t_us += int(dt * 1e6)
            self._send(build_heartbeat())
            # CRITICAL: use the actual launch quaternion (not identity).
            # EKF2 cross-checks gravity direction inferred from accel against
            # the groundtruth attitude during alignment. Sending identity quat
            # while accel_body shows tilted-pad gravity creates an internal
            # inconsistency → mag_innovation rejects yaw alignment forever.
            # Mirrors SITL bridge (sitl/mavlink_bridge.py:850-865).
            self._send(build_hil_state_quat(
                self._sim_t_us,
                state[6:10], np.zeros(3),
                self.launch_lat, self.launch_lon, self.launch_alt,
                0, 0, 0, init_sensors["accel_body_true"], 0.0,
            ))
            self._send(build_hil_gps(
                self._sim_t_us,
                self.launch_lat, self.launch_lon, self.launch_alt, 0, 0, 0,
            ))
            # Re-noise per tick: prevents PX4 DataValidator marking sensors
            # STALE (it rejects 100+ identical samples). Mirrors SITL bridge
            # (sitl/mavlink_bridge.py:898-908). Without this, baro/accel/mag
            # all time out and EKF2 never gets aligned.
            wu_noisy_accel, wu_noisy_gyro = self.noise.add_imu_noise(
                init_sensors["accel_body_true"], np.zeros(3)
            )
            wu_noisy_mag = self.noise.add_mag_noise(init_sensors["mag_body"])
            wu_noisy_alt = self.noise.add_baro_noise(self.launch_alt)
            wu_noisy_baro_p = 1013.25 * (1.0 - 2.25577e-5 * wu_noisy_alt) ** 5.25588
            self._send(build_hil_sensor(
                self._sim_t_us,
                wu_noisy_accel, wu_noisy_gyro,
                wu_noisy_mag, wu_noisy_baro_p,
                init_sensors["diff_p"], wu_noisy_alt,
            ))
            self._drain_target(dt)

            if self._actuator_msg_count > 0 and not armed:
                armed = True
                print(f"\n[PIL] Target armed at step {wu_step} "
                      f"(actuator_msgs={self._actuator_msg_count}).")
            # إيقاع ثابت: ننتظر dt دائماً ليتمكن PX4 من المعالجة.
            # الحتمية تأتي من عدد الخطوات الثابت (wu_max_steps + settle_steps)،
            # وليس من إلغاء الانتظار.
            time.sleep(dt)
            wu_step += 1

        total_wu = time.monotonic() - wu_start
        print(f"[PIL] Warm-up done in {total_wu:.1f}s ({wu_step} steps, "
              f"armed={armed}, actuator_msgs={self._actuator_msg_count})")

        if self._actuator_msg_count == 0 and self.abort_on_no_actuator:
            raise RuntimeError(
                "[PIL] PX4 لم يُرسل أي HIL_ACTUATOR_CONTROLS خلال warm-up.\n"
                "  الأسباب المحتملة:\n"
                "    1) SYS_HITL=0 — ضبطه يتطلب إعادة تشغيل PX4 بعد PARAM_SET.\n"
                "    2) simulator_mavlink ليس متصلاً بالجسر (تحقّق من البورت 4560).\n"
                "    3) EKF2 لم يتقارب (GPS lock مفقود — راجع warm-up أطول).\n"
                "  لتجاوز هذا الفحص: warmup.abort_on_no_actuator: false في pil_config.yaml."
            )

        mode_label = "lockstep" if self.clock_mode == "lockstep" else "realtime"
        print(f"[PIL] Starting flight loop ({mode_label})...")
        t_wall0 = time.monotonic()
        _t_off_us = self._sim_t_us + int(dt * 1e6)
        step = 0
        t = 0.0

        # معدلات الإرسال (dt=0.01 → سقف عملي 100Hz):
        #   HIL_SENSOR : 100Hz (EKF2 يقبل 80Hz+).
        #   HIL_STATE  : 50Hz (ground-truth عرضي، لا يُغذّي EKF2).
        #   HIL_GPS    : 10Hz (تحديث GPS قياسي).
        # الصيغة السابقة كانت `1.0/(250*dt)` التي تُنتج 0→max(1,0)=1 مع dt=0.01،
        # مُوهمة أن المعدّل 250Hz. الصيغة الجديدة صريحة تطابق تصحيح hil/ (M4).
        sensor_every = max(1, int(round(1.0 / (100 * dt))))
        gps_every = max(1, int(round(1.0 / (10 * dt))))
        state_every = max(1, int(round(1.0 / (50 * dt))))
        hb_every = 100

        # lockstep: كل mpc_every خطوة نتوقّف وننتظر actuator جديد (sync point).
        # هذا يطابق سلوك Gazebo lockstep: الديناميكا تتقدّم بـ n خطوات dt ثم
        # تتزامن مع استجابة MPC. خارج نقاط sync نتابع بأقصى سرعة بدون wall-clock.
        lockstep_on = (self.clock_mode == "lockstep")
        mpc_every = max(1, int(round(1.0 / (self.mpc_cycle_hz * dt))))
        lockstep_missed = 0
        mode_label = "hybrid-lockstep" if (lockstep_on and self.realtime_pacing) else \
                     "lockstep-pure" if lockstep_on else "realtime"
        if self._compute_delay_max_us > 0:
            mode_label += f" + compute-delay-injected (max {self._compute_delay_max_us//1000}ms)"
        print(f"[PIL] Pacing mode: {mode_label}")

        while step < n_steps and self._running:
            t = step * dt
            self._sim_t_us = _t_off_us + int(t * 1e6)

            # ── Compute-delay-aware actuator selection ──
            # إذا inject_compute_delay مفعّل: نُطبّق فقط الأوامر التي حانت
            # نقطة تنفيذها (apply_at <= sim_T_us). يحاكي phase lag الحقيقي.
            if self._compute_delay_max_us > 0:
                while self._cmd_queue and self._cmd_queue[0][0] <= self._sim_t_us:
                    _, ctrl = self._cmd_queue.pop(0)
                    self._active_controls = ctrl
                self._fins_rad = self._active_controls[:4].copy()
            else:
                # وضع كلاسيكي: آخر controls فوراً
                self._fins_rad = self._last_controls[:4].copy()

            try:
                next_state, snap, t_end, _ = sim._integrate_one_step(state, t, dt, _ctrl)
            except Exception as e:
                print(f"[PIL] sim error at t={t:.3f}: {e}")
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

            # ── SRV_FB: send actual (delayed) fin positions to PX4 so
            # RocketMPC.cpp can use them in x_mpc[12..14] instead of
            # falling back to first-order filter approximation.  This
            # eliminates model-plant mismatch when simulator uses pure
            # transport delay (delay_steps>0).  Mimics xqpower_can SRV_FB.
            if step % sensor_every == 0:
                fins_rad_actual = snap.get("control_fins_rad",
                                           np.zeros(4))
                fins_deg_actual = np.degrees(np.asarray(fins_rad_actual))
                self._send(build_srv_fb_debug_array(
                    self._sim_t_us, fins_deg_actual, mask=0x0F,
                ))

            self._drain_target(dt)
            self._log(snap, next_state, t_end, s)

            if sim._detect_ground_impact(next_state, t_end):
                print(f"\n[PIL] Ground impact at t={t_end:.3f}s")
                break

            # ── Pacing ──────────────────────────────────────────────────
            if lockstep_on:
                # نقطة sync كل mpc_every خطوة: ننتظر actuator جديد قبل التقدّم.
                if (step + 1) % mpc_every == 0:
                    _px4_wait_t0 = time.monotonic()
                    if not self._wait_for_new_actuator(self.lockstep_timeout_s):
                        lockstep_missed += 1
                        if lockstep_missed <= 5 or lockstep_missed % 50 == 0:
                            print(f"\n[PIL] lockstep timeout #{lockstep_missed} "
                                  f"at t={t:.3f}s (MPC may be stuck)")
                    # ── سجّل وقت معالجة PX4 (wall-clock) كقياس دقيق ──
                    # في lockstep، hrt_absolute_time داخل PX4 = sim time → cycle_us=0.
                    # لذلك نُقيس نحن في Bridge وقت الـ wait = وقت معالجة PX4 الحقيقي
                    # (EKF2×4 + MPC + MHE) بالـ wall-clock.
                    _px4_processing_us = int((time.monotonic() - _px4_wait_t0) * 1e6)
                    if self.timing_enabled and _px4_processing_us > 0:
                        with self._timing_lock:
                            self.timing["t_sim"].append(self._sim_t_us / 1e6)
                            self.timing["mhe_us"].append(0.0)
                            self.timing["mpc_us"].append(float(_px4_processing_us))
                            self.timing["cycle_us"].append(float(_px4_processing_us))

            # ── Wall-clock pacing (hybrid lockstep — كما في HIL) ──────
            # بعد استلام actuator، ننام حتى wall-clock يلحق sim time.
            # MPC solve time يأكل من budget الـ 40ms — تماماً كالطيران الحقيقي.
            # realtime_pacing=false: lockstep نقي (للتشخيص فقط — MPC solve مخفي).
            if self.realtime_pacing or not lockstep_on:
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
                print(f"\r[PIL] {pct:5.1f}% t={t:6.1f}s alt={alt:7.0f}m "
                      f"v={spd:5.0f}m/s fins=[{self._fins_rad[0]:+.3f},"
                      f"{self._fins_rad[1]:+.3f},{self._fins_rad[2]:+.3f},"
                      f"{self._fins_rad[3]:+.3f}]", end="", flush=True)

        print(f"\n[PIL] Loop done: {step} steps, t={t:.3f}s, "
              f"wall={time.monotonic() - t_wall0:.1f}s")
        if lockstep_on and lockstep_missed > 0:
            print(f"[PIL] lockstep timeouts: {lockstep_missed} "
                  f"(ignored — continued with last actuator)")
        if lockstep_on and self.realtime_pacing:
            wall_total = time.monotonic() - t_wall0
            sim_total = step * dt
            ratio = wall_total / sim_total if sim_total > 0 else 0
            print(f"[PIL] Hybrid lockstep: wall={wall_total:.1f}s  sim={sim_total:.1f}s  "
                  f"ratio={ratio:.2f}x (1.0=realtime)")

        # ─── MHE diagnostics summary ──────────────────────────────────
        n_mhe = len(self.mhe_log["quality"])
        if n_mhe > 0:
            import numpy as _np
            q = _np.array(self.mhe_log["quality"])
            v = _np.array(self.mhe_log["valid"])
            s = _np.array(self.mhe_log["status"])
            q_nonzero = q[q > 0]
            valid_pct = float(v.mean()) * 100
            q_mean = float(q_nonzero.mean()) if len(q_nonzero) > 0 else 0.0
            q_min = float(q_nonzero.min()) if len(q_nonzero) > 0 else 0.0
            status_ok = float((s == 0).mean()) * 100
            print(f"[PIL] MHE: {n_mhe} samples  valid={valid_pct:.1f}%  "
                  f"quality_avg={q_mean:.2f}  quality_min={q_min:.2f}  "
                  f"status_ok={status_ok:.1f}%")
            if q_mean < 0.5 and len(q_nonzero) > 0:
                print(f"[PIL]   ⚠️  MHE quality low — check solver health")
            if valid_pct < 80 and n_mhe > 20:
                print(f"[PIL]   ⚠️  MHE not valid in {100-valid_pct:.0f}% of samples")
        else:
            print(f"[PIL] MHE: no samples received from DEBUG_FLOAT_ARRAY")

        # إيقاف خيط التوقيت
        self._timing_stop.set()
        if self._timing_thread is not None:
            self._timing_thread.join(timeout=2.0)
        print(f"[PIL] Timing samples captured: {len(self.timing['t_sim'])}")

        if self._sock:
            self._sock.close()
            self._sock = None

        self._export_flight_csv(flight_csv)
        print(f"[PIL] Flight CSV: {flight_csv}")
        if timing_csv and self.timing_enabled:
            self._export_timing_csv(timing_csv)
            print(f"[PIL] Timing CSV: {timing_csv}")

        return self.flight

    # ─── استقبال رسائل الهدف ────────────────────────────────────────────────

    def _wait_for_new_actuator(self, timeout_s: float) -> bool:
        """ينتظر رسالة HIL_ACTUATOR_CONTROLS جديدة من PX4 (lockstep sync).

        يُستدعى في lockstep mode فقط — يحجب المحاكاة حتى يكتمل MPC solve على
        الهدف (ARM64) ويُرسل response. هذا يُزيل قطعة الأثر الناتجة عن realtime
        pacing حيث الديناميكا تتقدّم بينما MPC ما زال يحل، مسبّبة phase lag
        عشوائي وعدم استقرار.

        Returns:
            True إذا وصل actuator جديد، False إذا انتهت المهلة (fallback لـ
            آخر actuator معروف).
        """
        act_before = self._actuator_msg_count
        deadline = time.monotonic() + timeout_s
        # Poll كل 0.5ms حتى نستلم رسالة جديدة أو تنتهي المهلة. النومات
        # الصغيرة تُقلّل تحميل CPU دون التأثير على الـ latency المقاسة.
        while self._actuator_msg_count == act_before:
            self._drain_target(0.0)
            if self._actuator_msg_count > act_before:
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.0005)
        return True

    def _drain_target(self, dt: float):
        """قراءة ما توفّر من الهدف بلا حجب. يحدّث التحكّم وسجلّ التوقيت.

        ملاحظة: رسائل التوقيت عبر القناة الأساسية (TCP 4560 — simulator_mavlink)
        نادرة جداً (هذه القناة تبثّ HIL_* و HIL_ACTUATOR_CONTROLS فقط)، لكنّنا
        نحتفظ بدعمها كـfallback. عبر القفل _timing_lock نتجنّب التسابق مع
        _timing_reader_loop (TCP 5760) الذي يكتب في نفس self.timing.
        """
        msgs = self._recv_nonblock()
        timing_updates: list = []  # نجمّع ثم نقفل مرة واحدة
        for msg_id, payload in msgs:
            if msg_id == MSG_HIL_ACTUATOR_CONTROLS:
                p = parse_actuator_controls(payload)
                if p:
                    new_ctrl = np.array(p["controls"])
                    self._last_controls = new_ctrl
                    self._actuator_msg_count += 1
                    # ── Compute-delay injection ──
                    # نضيف الأمر للطابور مع تأخير = mpc_us المقاس على الهاتف.
                    # الهدف: محاكاة phase lag الحقيقي بدون MAVLink overhead.
                    if self._compute_delay_max_us > 0:
                        with self._timing_lock:
                            mpc_us_meas = float(self._last_timing.get("mpc_us", 0.0))
                        # clamp لتجنّب قيم شاذة
                        delay_us = int(min(max(mpc_us_meas, 0.0),
                                           float(self._compute_delay_max_us)))
                        apply_at = self._sim_t_us + delay_us
                        self._cmd_queue.append((apply_at, new_ctrl))
                    else:
                        # وضع كلاسيكي: تطبيق فوري
                        self._active_controls = new_ctrl
            elif self.timing_enabled and msg_id == MSG_NAMED_VALUE_FLOAT:
                p = parse_named_value_float(payload)
                if p and p["name"] in ("mhe_us", "mpc_us", "cycle_us"):
                    timing_updates.append(("named", p["name"], p["value"]))
            elif self.timing_enabled and msg_id == MSG_DEBUG_VECT:
                p = parse_debug_vect(payload)
                # الاصطلاح: name="TIMING", x=mhe_us, y=mpc_us, z=cycle_us
                if p and p["name"].upper().startswith("TIMING"):
                    timing_updates.append((("vect", p["x"], p["y"], p["z"])))
        if not timing_updates:
            return
        with self._timing_lock:
            for upd in timing_updates:
                if upd[0] == "named":
                    self._last_timing[upd[1]] = upd[2]
                else:
                    _, x, y, z = upd
                    self._last_timing["mhe_us"] = x
                    self._last_timing["mpc_us"] = y
                    self._last_timing["cycle_us"] = z
            # عيّنة واحدة لكل استدعاء _drain_target (تجنّب التضخيم الكاذب للعينات)
            self.timing["t_sim"].append(self._sim_t_us / 1e6)
            self.timing["mhe_us"].append(self._last_timing["mhe_us"])
            self.timing["mpc_us"].append(self._last_timing["mpc_us"])
            self.timing["cycle_us"].append(self._last_timing["cycle_us"])

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
        self.flight["fin_cmd"].append(self._fins_rad.copy())
        self.flight["fin_act"].append(
            np.array(snap.get("control_fins_rad", np.zeros(4)))
        )
        self.flight["forces"].append(np.array(snap.get("forces", [0, 0, 0])))
        # Log absolute MSL altitude to match the baseline comparison feed.
        # The previous `s["alt"] - self.launch_alt` produced an AGL signal which
        # differed from the baseline's MSL trace by a constant self.launch_alt
        # (= 1200 m by default), producing a fake steady 1200 m "error" in the
        # altitude comparison report.
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
        os.makedirs(os.path.dirname(path), exist_ok=True)
        pos = np.array(self.flight["pos"])
        vel = np.array(self.flight["vel"])
        q = np.array(self.flight["quat"])
        w = np.array(self.flight["omega"])
        fc = np.array(self.flight["fin_cmd"])
        fa = np.array(self.flight["fin_act"])
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
                    f"{fr[i, 0]:.4f}", f"{fr[i, 1]:.4f}", f"{fr[i, 2]:.4f}",
                    f"{self.flight['lat'][i]:.8f}",
                    f"{self.flight['lon'][i]:.8f}",
                    f"{self.flight['alt_msl'][i]:.4f}",
                ])

    def _export_timing_csv(self, path: str):
        n = len(self.timing["t_sim"])
        os.makedirs(os.path.dirname(path), exist_ok=True)
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
            print("[PIL] WARNING: no timing samples received — "
                  "verify target publishes DEBUG_VECT 'TIMING' or "
                  "NAMED_VALUE_FLOAT (mhe_us/mpc_us/cycle_us).")


# ============================================================================
# CLI
# ============================================================================

def main():
    ap = argparse.ArgumentParser(description="M130 PIL MAVLink Bridge (standalone)")
    ap.add_argument("--config", default=str(_SCRIPT_DIR / "pil_config.yaml"))
    ap.add_argument("--csv", default=None)
    ap.add_argument("--timing-csv", default=None)
    ap.add_argument("--host", default=None, help="Override listen host")
    ap.add_argument("--port", type=int, default=None, help="Override listen port")
    args = ap.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )

    bridge = PILBridge(args.config)
    if args.host:
        bridge.host = args.host
    if args.port:
        bridge.port = args.port

    out = bridge.cfg.get("output", {})
    results_dir = _SCRIPT_DIR / "results"
    flight_csv = args.csv or str(results_dir / out.get("csv_name", "pil_flight.csv"))
    timing_csv = args.timing_csv or str(results_dir / out.get("timing_csv", "pil_timing.csv"))
    bridge.run(flight_csv, timing_csv)


if __name__ == "__main__":
    main()
