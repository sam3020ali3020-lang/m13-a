"""
XQPOWER CAN servo protocol — direct USB via pyusb (CAN_LIN_Tool V6.0).

Architecture (verified live on /dev/bus/usb/<bus>/<dev>):
  - The CAN_LIN_Tool exposes 3 interfaces. CAN data flows on interface 2
    (Human Interface Device class), endpoints OUT 0x03 / IN 0x83 (interrupt).
  - Linux binds hid-generic to interface 2 by default; we detach it and
    claim it via libusb.
  - The CDC interface (/dev/ttyACM0) does NOT carry CAN traffic.

Frame format (interface 2, OUT direction, 20 bytes per CAN frame):
  [0]    cmd        0x01 = CAN frame
  [1:5]  std_id     little-endian 32-bit
  [5:9]  ext_id     0
  [9]    id_type    0 = standard
  [10]   frame_type 0 = data
  [11]   dlc
  [12:20] data, zero-padded

RX direction returns up to 62 bytes per read with one or more 17-byte
embedded frames:
  [0]    DLC
  [1]    flags
  [2:4]  CAN ID LE
  [4:8]  ext id (zero)
  [8]    DLC repeated
  [9:17] data

Position scale: 18 raw units / degree (verified against XqpowerCan.cpp).

Servo PDO auto-report (OD 0x2200) is REJECTED by the firmware on this
hardware (SDO ABORT 0x30000906). We therefore acquire feedback by SDO
read polling of OD 0x6002 (data[0]=0x4B, data[4:6]=int16 position).

Round-robin polling at 5 ms per request gives ~50 Hz per fin (~200 Hz
combined for 4 fins) — sufficient for FOPDT identification at 200 Hz fs.
"""

from __future__ import annotations

import collections
import struct
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

import usb.core
import usb.util


# CAN IDs
NMT_ID = 0x000
SDO_TX_BASE = 0x600  # master -> servo
SDO_RX_BASE = 0x580  # servo -> master
PDO_RX_BASE = 0x180  # servo -> master (CANopen PDO, unused on this fw)

# SDO command bytes
SDO_WRITE = 0x22
SDO_READ = 0x40
SDO_RESP_READ_2B = 0x4B   # 2-byte expedited read response
SDO_RESP_READ_4B = 0x43
SDO_RESP_WRITE_ACK = 0x60
SDO_RESP_ABORT = 0x80

OD_POSITION_TARGET = 0x6003  # subindex 0
OD_POSITION_ACTUAL = 0x6002  # subindex 0 (read)

RAW_PER_DEG = 18.0

# CAN_LIN_Tool device IDs
CANLIN_VENDOR_ID = 0x2E3C
CANLIN_PRODUCT_ID = 0x5750
CANLIN_HID_INTERFACE = 2

# Adapter config commands (sent as raw bytes, NOT as CAN frames)
CMD_BAUD_500K = bytes([0x03, 0x01, 0xF4, 0x01, 0x00, 0x00, 0x12, 0x00, 0x00, 0x05, 0x00])
CMD_SAVE = bytes([0x03, 0x05])
CMD_TERMINATION_ON = bytes([0x06, 0x01])
CMD_RECEIVE_ALL = bytes([0x03, 0x02, 0x03])


@dataclass
class CanFrame:
    can_id: int
    data: bytes
    timestamp_mono_ns: int = 0


@dataclass
class ServoFeedback:
    position_deg: float = 0.0
    timestamp_mono_ns: int = 0
    online: bool = False
    sample_count: int = 0
    abort_count: int = 0


def _encode_can_tx(can_id: int, data: bytes) -> bytes:
    if len(data) > 8:
        raise ValueError(f"DLC {len(data)} > 8")
    padded = data + b"\x00" * (8 - len(data))
    return (
        bytes([0x01])
        + struct.pack("<I", can_id)
        + b"\x00" * 4
        + bytes([0, 0, len(data)])
        + padded
    )


def _decode_rx_chunk(buf: bytearray) -> tuple[list[CanFrame], int]:
    """Scan the 62-byte HID payload(s) for embedded 17-byte CAN frames.

    Heuristic mirrors XqpowerCan.cpp:578-615: a frame begins with DLC
    (0x02 or 0x08) at offset 0, then flags at +1, CAN ID LE at +2/+3.

    NOTE: لا نقبل ``can_id == 0x000`` — ليس RX حقيقياً بعد init، بل يسبّب
    parser misalignment تحت ضغط TX (يُنتج phantom frames تُفوّت SDO
    responses الحقيقية). NMT broadcast نُرسله لكن لا نحتاج معالجة echo.
    """
    frames: list[CanFrame] = []
    rd = 0
    n = len(buf)
    now_ns = time.monotonic_ns()
    while rd + 17 <= n:
        b0 = buf[rd]
        if b0 not in (0x02, 0x08):
            rd += 1
            continue
        can_id = buf[rd + 2] | (buf[rd + 3] << 8)
        valid = (
            0x180 <= can_id <= 0x184
            or 0x580 <= can_id <= 0x584
            or 0x600 <= can_id <= 0x604
        )
        if not valid:
            rd += 1
            continue
        dlc = b0
        if dlc > 8:
            rd += 1
            continue
        data = bytes(buf[rd + 9 : rd + 9 + dlc])
        frames.append(CanFrame(can_id=can_id, data=data, timestamp_mono_ns=now_ns))
        rd += 17
    return frames, rd


def find_canlin_device() -> Optional[usb.core.Device]:
    return usb.core.find(idVendor=CANLIN_VENDOR_ID, idProduct=CANLIN_PRODUCT_ID)


class XqpowerBus:
    """XQPOWER CAN servo bus over CAN_LIN_Tool USB HID adapter.

    Uses pyusb directly (libusb) on interface 2, mirroring the Android driver
    which writes/reads via USBDEVFS_BULK. Feedback is acquired by SDO-read
    polling because the servos' firmware does not implement OD 0x2200
    (set_report_interval).
    """

    def __init__(
        self,
        port: Optional[str] = None,  # legacy/ignored; kept for API compat
        node_ids: tuple[int, ...] = (0x01, 0x02, 0x03, 0x04),
        tx_inter_frame_us: int = 500,
        rx_log_size: int = 100_000,
        poll_interval_us: int = 5_000,    # 5 ms between SDO read requests
    ):
        del port  # unused now; auto-detect
        self.node_ids = tuple(node_ids)
        self.n_servos = len(self.node_ids)
        self.tx_inter_frame_us = tx_inter_frame_us
        self.poll_interval_us = poll_interval_us

        self._dev: Optional[usb.core.Device] = None
        self._ep_out = None
        self._ep_in = None
        self._tx_lock = threading.Lock()

        self._rx_thread: Optional[threading.Thread] = None
        self._poll_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._poll_enabled = threading.Event()

        self._fb: list[ServoFeedback] = [ServoFeedback() for _ in range(self.n_servos)]
        self._fb_lock = threading.Lock()
        self._fb_callback: Optional[Callable[[int, float, int], None]] = None

        self.rx_log: collections.deque[CanFrame] = collections.deque(maxlen=rx_log_size)
        self._rx_log_enabled = False

        self.tx_count = 0
        self.tx_fail = 0
        self.sdo_read_count = 0
        self.sdo_write_ack_count = 0
        self.sdo_abort_count = 0

    # ------------------------------------------------------------------ open/close

    def open(self) -> None:
        dev = find_canlin_device()
        if dev is None:
            raise RuntimeError(
                f"CAN_LIN_Tool not found ({CANLIN_VENDOR_ID:04x}:{CANLIN_PRODUCT_ID:04x})"
            )

        # Detach kernel drivers (cdc_acm, hid-generic) on all interfaces
        for ifn in (0, 1, 2):
            try:
                if dev.is_kernel_driver_active(ifn):
                    dev.detach_kernel_driver(ifn)
            except (usb.core.USBError, NotImplementedError):
                pass

        usb.util.claim_interface(dev, CANLIN_HID_INTERFACE)
        cfg = dev.get_active_configuration()
        intf = cfg[(CANLIN_HID_INTERFACE, 0)]
        self._ep_out = usb.util.find_descriptor(
            intf,
            custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress)
            == usb.util.ENDPOINT_OUT,
        )
        self._ep_in = usb.util.find_descriptor(
            intf,
            custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress)
            == usb.util.ENDPOINT_IN,
        )
        if self._ep_out is None or self._ep_in is None:
            raise RuntimeError("Could not find IN/OUT endpoints on interface 2")
        self._dev = dev

        # Configure adapter: receive-all (so we get RX traffic to host)
        self._raw_write(CMD_RECEIVE_ALL)
        time.sleep(0.05)
        self._drain_input()

        # Spin RX + poll threads
        self._stop.clear()
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name="xqpower-rx", daemon=True
        )
        self._rx_thread.start()
        self._poll_thread = threading.Thread(
            target=self._poll_loop, name="xqpower-poll", daemon=True
        )
        self._poll_thread.start()

    def close(self) -> None:
        self._stop.set()
        for t in (self._rx_thread, self._poll_thread):
            if t is not None:
                t.join(timeout=1.0)
        self._rx_thread = None
        self._poll_thread = None
        if self._dev is not None:
            try:
                usb.util.release_interface(self._dev, CANLIN_HID_INTERFACE)
            except usb.core.USBError:
                pass
            try:
                usb.util.dispose_resources(self._dev)
            except Exception:
                pass
            self._dev = None
        self._ep_out = None
        self._ep_in = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    # ------------------------------------------------------------------ low-level I/O

    def _raw_write(self, data: bytes) -> int:
        if self._ep_out is None:
            return -1
        try:
            n = self._ep_out.write(data, timeout=200)
            return int(n)
        except usb.core.USBError:
            return -1

    def _raw_read(self, timeout_ms: int = 50) -> bytes:
        if self._ep_in is None:
            return b""
        try:
            return bytes(self._ep_in.read(64, timeout=timeout_ms))
        except usb.core.USBError:
            return b""

    def _drain_input(self) -> None:
        for _ in range(20):
            if not self._raw_read(timeout_ms=10):
                return

    # ------------------------------------------------------------------ CAN send

    def can_send(self, can_id: int, data: bytes) -> bool:
        if self._ep_out is None:
            return False
        frame = _encode_can_tx(can_id, data)
        with self._tx_lock:
            n = self._raw_write(frame)
            if n != len(frame):
                self.tx_fail += 1
                return False
            self.tx_count += 1
        # NOTE: pacing-sleep يقع خارج الـ lock عمداً — وإلّا يُجوِّع poll
        # thread تحت ضغط TX من main loop (lock starvation → fb stall).
        # الـ adapter (CAN_LIN_Tool) يُدير pacing الفعلي على بص CAN.
        if self.tx_inter_frame_us > 0:
            time.sleep(self.tx_inter_frame_us / 1e6)
        return True

    # ------------------------------------------------------------------ RX thread

    def _rx_loop(self) -> None:
        buf = bytearray()
        while not self._stop.is_set():
            chunk = self._raw_read(timeout_ms=20)
            if not chunk:
                continue
            buf.extend(chunk)
            frames, consumed = _decode_rx_chunk(buf)
            if consumed:
                del buf[:consumed]
            for f in frames:
                if self._rx_log_enabled:
                    self.rx_log.append(f)
                self._handle_frame(f)
            # Cap buffer growth
            if len(buf) > 4096:
                del buf[:-512]

    def _handle_frame(self, f: CanFrame) -> None:
        cid = f.can_id
        if not (SDO_RX_BASE + 1 <= cid <= SDO_RX_BASE + 127):
            return
        node = cid - SDO_RX_BASE
        try:
            slot = self.node_ids.index(node)
        except ValueError:
            return
        if len(f.data) < 1:
            return
        cmd = f.data[0]
        if cmd == SDO_RESP_READ_2B and len(f.data) >= 6:
            idx = f.data[1] | (f.data[2] << 8)
            if idx == OD_POSITION_ACTUAL:
                pos_raw = struct.unpack("<h", f.data[4:6])[0]
                pos_deg = pos_raw / RAW_PER_DEG
                self._publish_position(slot, pos_deg, f.timestamp_mono_ns)
                self.sdo_read_count += 1
        elif cmd == SDO_RESP_WRITE_ACK:
            self.sdo_write_ack_count += 1
        elif cmd == SDO_RESP_ABORT:
            self.sdo_abort_count += 1
            with self._fb_lock:
                self._fb[slot].abort_count += 1

    def _publish_position(self, slot: int, pos_deg: float, ts_ns: int) -> None:
        with self._fb_lock:
            fb = self._fb[slot]
            fb.position_deg = pos_deg
            fb.timestamp_mono_ns = ts_ns
            fb.online = True
            fb.sample_count += 1
        cb = self._fb_callback
        if cb is not None:
            cb(slot, pos_deg, ts_ns)

    # ------------------------------------------------------------------ poll thread

    def _poll_loop(self) -> None:
        """Round-robin SDO read of OD 0x6002 across all nodes."""
        read_payload = bytes([SDO_READ, 0x02, 0x60, 0, 0, 0, 0, 0])
        idx = 0
        while not self._stop.is_set():
            if not self._poll_enabled.is_set():
                time.sleep(0.005)
                continue
            slot = idx % self.n_servos
            node = self.node_ids[slot]
            self.can_send(SDO_TX_BASE + node, read_payload)
            idx += 1
            time.sleep(self.poll_interval_us / 1e6)

    def start_polling(self) -> None:
        self._poll_enabled.set()

    def stop_polling(self) -> None:
        self._poll_enabled.clear()

    # ------------------------------------------------------------------ XQPOWER API

    def nmt_start(self, slot: int) -> bool:
        node = self.node_ids[slot]
        return self.can_send(NMT_ID, bytes([0x01, node]))

    def nmt_stop(self, slot: int) -> bool:
        node = self.node_ids[slot]
        return self.can_send(NMT_ID, bytes([0x02, node]))

    def read_position(self, slot: int) -> bool:
        node = self.node_ids[slot]
        return self.can_send(SDO_TX_BASE + node, bytes([SDO_READ, 0x02, 0x60, 0, 0, 0, 0, 0]))

    def set_position_deg(self, slot: int, angle_deg: float, limit_deg: float = 20.0) -> bool:
        if angle_deg > limit_deg:
            angle_deg = limit_deg
        elif angle_deg < -limit_deg:
            angle_deg = -limit_deg
        position = int(round(angle_deg * RAW_PER_DEG))
        if position > 32767:
            position = 32767
        elif position < -32768:
            position = -32768
        node = self.node_ids[slot]
        data = struct.pack("<BBBBhBB", SDO_WRITE, 0x03, 0x60, 0x00, position, 0, 0)
        return self.can_send(SDO_TX_BASE + node, data)

    def set_position_all(self, angles_deg: list[float], limit_deg: float = 20.0) -> int:
        ok = 0
        for slot, ang in enumerate(angles_deg[: self.n_servos]):
            if self.set_position_deg(slot, ang, limit_deg):
                ok += 1
        return ok

    # ------------------------------------------------------------------ feedback

    def get_feedback(self, slot: int) -> ServoFeedback:
        with self._fb_lock:
            fb = self._fb[slot]
            return ServoFeedback(
                position_deg=fb.position_deg,
                timestamp_mono_ns=fb.timestamp_mono_ns,
                online=fb.online,
                sample_count=fb.sample_count,
                abort_count=fb.abort_count,
            )

    def get_all_feedback(self) -> list[ServoFeedback]:
        return [self.get_feedback(i) for i in range(self.n_servos)]

    def set_feedback_callback(self, cb: Optional[Callable[[int, float, int], None]]) -> None:
        self._fb_callback = cb

    def enable_rx_log(self, enabled: bool = True) -> None:
        self._rx_log_enabled = enabled

    # ------------------------------------------------------------------ high-level init

    def init_all_servos(self, report_interval_ms: int = 5, settle_s: float = 0.5) -> None:
        """NMT-Start every servo, drain abort responses, then enable polling.

        report_interval_ms is reused as the SDO-poll interval per-fin
        (the firmware's PDO auto-report (OD 0x2200) is not supported, so we
        emulate it via SDO read polling).
        """
        # Use the supplied interval (in ms) per fin. For 4 fins, total bus
        # request rate = n_servos / interval_ms requests per ms.
        per_fin_us = max(1000, report_interval_ms * 1000)
        self.poll_interval_us = per_fin_us // self.n_servos

        for slot in range(self.n_servos):
            self.nmt_start(slot)
            time.sleep(0.05)
        time.sleep(settle_s)
        # Drain any abort responses left over
        for _ in range(50):
            self._raw_read(timeout_ms=5)
        self.start_polling()

    def wait_for_all_online(self, timeout_s: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            online = sum(1 for fb in self.get_all_feedback() if fb.online)
            if online == self.n_servos:
                return True
            time.sleep(0.1)
        return False
