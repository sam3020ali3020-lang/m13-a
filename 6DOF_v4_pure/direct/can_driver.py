"""CAN backend abstraction for /direct.

يوفّر واجهة موحّدة CanBus فوق عدة backends:
  - socketcan (الأفضل — SocketCAN على Linux)
  - slcan     (شبه-قياسي USB-Serial)
  - serial    (Waveshare USB_CAN_A و CAN_LIN_Tool — توافق مع العتاد الموجود)
  - virtual   (vcan — اختبار بلا عتاد)

كل backend يُعيد Iterator لـ CanFrame مع timestamp نانوثاني من ساعة monotonic.
"""

from __future__ import annotations

import abc
import struct
import threading
import time
from dataclasses import dataclass
from queue import Empty, Queue
from typing import Iterable, Optional

try:
    import can  # python-can
except ImportError:  # pragma: no cover
    can = None

try:
    import serial as pyserial  # for Waveshare/CanLin raw framing
except ImportError:  # pragma: no cover
    pyserial = None

try:
    import usb.core  # for libusb-based CAN_LIN_Tool backend
    import usb.util
except ImportError:  # pragma: no cover
    usb = None  # type: ignore


@dataclass
class CanFrame:
    """إطار CAN مع timestamp من ساعة monotonic (ثوانٍ)."""
    t_s: float
    arb_id: int
    data: bytes

    def __repr__(self) -> str:
        hex_data = " ".join(f"{b:02X}" for b in self.data)
        return f"CanFrame(t={self.t_s:.6f}s id=0x{self.arb_id:03X} [{hex_data}])"


class CanBus(abc.ABC):
    """واجهة abstract لـ CAN bus."""

    @abc.abstractmethod
    def send(self, arb_id: int, data: bytes) -> None:
        """إرسال إطار (standard 11-bit ID، dlc=len(data))."""

    @abc.abstractmethod
    def recv(self, timeout_s: float = 0.0) -> Optional[CanFrame]:
        """استقبال إطار واحد (أو None عند timeout)."""

    @abc.abstractmethod
    def close(self) -> None:
        """إغلاق الاتصال بنظافة."""

    def __enter__(self) -> "CanBus":
        return self

    def __exit__(self, *exc) -> None:
        try:
            self.close()
        except Exception:
            pass


# ─── python-can backends (socketcan/slcan/virtual) ──────────────────────────

class _PythonCanBus(CanBus):
    """Wrapper فوق python-can.interface.Bus."""

    def __init__(self, bus: "can.BusABC") -> None:
        self._bus = bus

    def send(self, arb_id: int, data: bytes) -> None:
        if can is None:
            raise RuntimeError("python-can غير مثبَّت — pip install python-can")
        msg = can.Message(
            arbitration_id=arb_id,
            data=bytes(data),
            is_extended_id=False,
            is_fd=False,
        )
        self._bus.send(msg)

    def recv(self, timeout_s: float = 0.0) -> Optional[CanFrame]:
        msg = self._bus.recv(timeout=timeout_s)
        if msg is None:
            return None
        # python-can timestamp = time.time() (epoch) — نحوّل إلى monotonic approx
        # لا ضمانة على المصدر؛ للاتساق نستخدم time.monotonic() عند الاستقبال.
        return CanFrame(
            t_s=time.monotonic(),
            arb_id=int(msg.arbitration_id),
            data=bytes(msg.data),
        )

    def close(self) -> None:
        try:
            self._bus.shutdown()
        except Exception:
            pass


def _open_socketcan(cfg: dict) -> CanBus:
    if can is None:
        raise RuntimeError("python-can غير مثبَّت — pip install python-can")
    channel = cfg.get("channel", "can0")
    bus = can.interface.Bus(interface="socketcan", channel=channel)
    return _PythonCanBus(bus)


def _open_slcan(cfg: dict) -> CanBus:
    if can is None:
        raise RuntimeError("python-can غير مثبَّت — pip install python-can")
    bus = can.interface.Bus(
        interface="slcan",
        channel=cfg.get("port", "/dev/ttyUSB0"),
        bitrate=int(cfg.get("bitrate", 500000)),
        ttyBaudrate=int(cfg.get("ttyBaudrate", 2000000)),
    )
    return _PythonCanBus(bus)


def _open_virtual(cfg: dict) -> CanBus:
    if can is None:
        raise RuntimeError("python-can غير مثبَّت — pip install python-can")
    bus = can.interface.Bus(
        interface="virtual",
        channel=cfg.get("channel", "vcan0"),
    )
    return _PythonCanBus(bus)


# ─── Waveshare USB_CAN_A (CH340 + raw framing) backend ──────────────────────
#
# هيكل الإطار المستخدم في Android PX4 (XqpowerCan.cpp):
#   Byte 0   : 0xAA (start byte) — frame info
#   Byte 1   : 0xC8 (info: standard + data + dlc=8)
#   Byte 2-3 : arbitration id (little-endian)
#   Byte 4-11: data (8 bytes always)
#   Byte 12  : 0x55 (end byte)
#
# RX format نفسه تقريباً. نستخدم parser state-machine بسيط.

class _WaveshareSerialBus(CanBus):
    """Waveshare USB_CAN_A (CH340) raw frame protocol.

    هذا البروتوكول ليس SLCAN — هو إطار بدائي يُستخدم في المشروع. يُقدّم هنا
    كـ fallback إن لم يتوفّر socketcan/slcan adapter.
    """

    FRAME_START = 0xAA
    FRAME_END = 0x55
    FRAME_INFO = 0xC8       # standard + data + dlc=8

    def __init__(self, port: str, tty_baud: int = 2_000_000) -> None:
        if pyserial is None:
            raise RuntimeError("pyserial غير مثبَّت — pip install pyserial")
        self._ser = pyserial.Serial(
            port=port,
            baudrate=tty_baud,
            timeout=0.0,     # non-blocking
            write_timeout=1.0,
        )
        # خيط استقبال خلفي — يدفع frames إلى queue
        self._rx_queue: "Queue[CanFrame]" = Queue(maxsize=10_000)
        self._stop = threading.Event()
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name="waveshare-rx", daemon=True
        )
        self._rx_thread.start()

    def _rx_loop(self) -> None:
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(64)
                if not chunk:
                    time.sleep(0.0002)
                    continue
            except Exception:
                break
            buf.extend(chunk)

            # state machine: ابحث عن 0xAA .. 0x55 (13 بايت بالضبط)
            while len(buf) >= 13:
                if buf[0] != self.FRAME_START:
                    buf.pop(0)
                    continue
                if buf[12] != self.FRAME_END:
                    buf.pop(0)
                    continue
                arb_id = buf[2] | (buf[3] << 8)
                data = bytes(buf[4:12])
                del buf[:13]
                try:
                    self._rx_queue.put_nowait(CanFrame(time.monotonic(), arb_id, data))
                except Exception:
                    pass

    def send(self, arb_id: int, data: bytes) -> None:
        if len(data) > 8:
            raise ValueError("CAN DLC max 8 bytes")
        pad = bytes(data) + b"\x00" * (8 - len(data))
        frame = bytes([
            self.FRAME_START,
            self.FRAME_INFO,
            arb_id & 0xFF,
            (arb_id >> 8) & 0xFF,
            *pad,
            self.FRAME_END,
        ])
        self._ser.write(frame)

    def recv(self, timeout_s: float = 0.0) -> Optional[CanFrame]:
        try:
            return self._rx_queue.get(timeout=timeout_s) if timeout_s > 0 \
                   else self._rx_queue.get_nowait()
        except Empty:
            return None

    def close(self) -> None:
        self._stop.set()
        try:
            self._ser.close()
        except Exception:
            pass


# ─── Factory ────────────────────────────────────────────────────────────────

def open_can(cfg: dict) -> CanBus:
    """Open CAN bus وفق ``cfg['backend']``."""
    backend = cfg.get("backend", "socketcan")
    if backend == "socketcan":
        return _open_socketcan(cfg.get("socketcan", {}))
    if backend == "slcan":
        return _open_slcan(cfg.get("slcan", {}))
    if backend == "virtual":
        return _open_virtual(cfg.get("virtual", {}))
    if backend == "serial":
        sub = cfg.get("serial", {})
        variant = sub.get("variant", "waveshare")
        if variant == "waveshare":
            return _WaveshareSerialBus(
                port=sub.get("port", "/dev/ttyUSB0"),
                tty_baud=int(sub.get("tty_baud", 2_000_000)),
            )
        if variant == "canlin":
            return _CanLinSerialBus(
                port=sub.get("port", "/dev/ttyACM0"),
                tty_baud=int(sub.get("tty_baud", 2_000_000)),
            )
        raise NotImplementedError(f"serial variant '{variant}' غير مدعوم بعد")
    if backend == "canlin_usb":
        sub = cfg.get("canlin_usb", {})
        return _CanLinUsbBus(
            vendor_id=int(sub.get("vendor_id", 0x2E3C)),
            product_id=int(sub.get("product_id", 0x5750)),
            interface=int(sub.get("interface", 2)),
        )
    if backend == "xqpower_bus":
        # Reuse tested XqpowerBus from servo_characterization (مضمون يعمل)
        sub = cfg.get("xqpower_bus", {})
        node_ids = tuple(int(x) for x in sub.get("node_ids", [1, 2, 3, 4]))
        return _XqpowerBusAdapter(
            node_ids=node_ids,
            poll_interval_us=int(sub.get("poll_interval_us", 5000)),
        )
    raise ValueError(f"unknown CAN backend: {backend}")


# ─── CAN_LIN_Tool V6.0 (CDC, VID=0x2E3C) backend ────────────────────────────
#
# Frame format (matches XqpowerCan.cpp's _is_canlin_tool path):
#   TX (20 bytes):
#     [0]     0x01 (command: CAN frame)
#     [1-4]   standard_id (LE, 4 bytes)
#     [5-8]   extended_id (0 for standard)
#     [9]     id_type: 0x00 (standard)
#     [10]    frame_type: 0x00 (data)
#     [11]    dlc
#     [12-19] data[8]  (padded with 0x00)
#   RX (17 bytes — scan-based):
#     [0]     DLC (0x08 or 0x02)            ← frame-start marker
#     [1]     flags/count
#     [2-3]   CAN ID (LE)
#     [4-7]   extended ID (zeros)
#     [8]     DLC again
#     [9-16]  data[8]
#   Valid CAN IDs (XQPOWER): 0x000, 0x180+n, 0x580+n, 0x600+n  (n = 1..7F)

class _CanLinSerialBus(CanBus):
    """CAN_LIN_Tool V6.0 (CDC class on /dev/ttyACM0)."""

    # CAN_LIN_Tool V6.0 init sequence (مأخوذة بدقة من XqpowerCan.cpp ~line 405-427).
    # تُرسَل بترتيب صارم مع أوقات انتظار بينها.
    CMD_SET_BAUD_500K = bytes([
        0x03, 0x01, 0xF4, 0x01, 0x00, 0x00, 0x12, 0x00, 0x00, 0x05, 0x00,
    ])                                          # set 500 kbps
    CMD_SAVE_CONFIG = bytes([0x03, 0x05])       # commit config to adapter
    CMD_ENABLE_120OHM = bytes([0x06, 0x01])     # 120Ω termination ON
    CMD_RECEIVE_ALL = bytes([0x03, 0x02, 0x03]) # disable HW filters

    def __init__(self, port: str, tty_baud: int = 2_000_000,
                 debug: bool = False) -> None:
        if pyserial is None:
            raise RuntimeError("pyserial غير مثبَّت — pip install pyserial")
        self._debug = debug
        # ttyACM (CDC) لا يهتمّ بمعدّل البود لكن نمرّره لتوافقية pyserial
        self._ser = pyserial.Serial(
            port=port, baudrate=tty_baud,
            timeout=0.0, write_timeout=1.0,
        )

        # 0) flush stale bytes قبل init
        try:
            self._ser.read(4096)
        except Exception:
            pass

        # 1) set bitrate 500 kbps
        self._ser.write(self.CMD_SET_BAUD_500K)
        time.sleep(0.12)
        # 2) save
        self._ser.write(self.CMD_SAVE_CONFIG)
        time.sleep(0.12)
        # 3) 120Ω termination ON
        self._ser.write(self.CMD_ENABLE_120OHM)
        time.sleep(0.06)
        # 4) receive all (disable filters)
        self._ser.write(self.CMD_RECEIVE_ALL)
        time.sleep(0.06)

        # flush adapter responses before real traffic starts
        try:
            init_resp = self._ser.read(4096)
            if self._debug and init_resp:
                print(f"[canlin] init response ({len(init_resp)}B): "
                      f"{init_resp.hex(' ')[:120]}...")
        except Exception:
            pass

        self._rx_queue: "Queue[CanFrame]" = Queue(maxsize=10_000)
        self._stop = threading.Event()
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name="canlin-rx", daemon=True
        )
        self._rx_thread.start()

    def _rx_loop(self) -> None:
        buf = bytearray()
        valid_id = lambda i: (i == 0x000
                              or 0x180 <= i <= 0x1FF
                              or 0x580 <= i <= 0x5FF
                              or 0x600 <= i <= 0x67F)
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(128)
            except Exception:
                break
            if not chunk:
                time.sleep(0.0002)
                continue
            buf.extend(chunk)

            # Scan: find 17-byte frames anywhere في الـ stream
            while len(buf) >= 17:
                b0 = buf[0]
                if b0 not in (0x02, 0x08):
                    buf.pop(0)
                    continue
                arb_id = buf[2] | (buf[3] << 8)
                if not valid_id(arb_id):
                    buf.pop(0)
                    continue
                dlc = b0
                if dlc > 8:
                    buf.pop(0)
                    continue
                data = bytes(buf[9:9 + dlc])
                # pad حتى 8 bytes للسهولة في decode_frame
                if len(data) < 8:
                    data = data + b"\x00" * (8 - len(data))
                del buf[:17]
                try:
                    self._rx_queue.put_nowait(
                        CanFrame(time.monotonic(), arb_id, data)
                    )
                except Exception:
                    pass

    def send(self, arb_id: int, data: bytes) -> None:
        if len(data) > 8:
            raise ValueError("CAN DLC max 8 bytes")
        pad = bytes(data) + b"\x00" * (8 - len(data))
        frame = bytes([
            0x01,                          # command: CAN frame
            arb_id & 0xFF,                 # std_id LE [1..4]
            (arb_id >> 8) & 0xFF,
            0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,        # ext_id = 0
            0x00,                          # id_type: standard
            0x00,                          # frame_type: data
            len(data),                     # dlc
            *pad,                          # data[8]
        ])
        self._ser.write(frame)

    def recv(self, timeout_s: float = 0.0) -> Optional[CanFrame]:
        try:
            if timeout_s > 0:
                return self._rx_queue.get(timeout=timeout_s)
            return self._rx_queue.get_nowait()
        except Empty:
            return None

    def close(self) -> None:
        self._stop.set()
        try:
            self._ser.close()
        except Exception:
            pass


# ─── CAN_LIN_Tool V6.0 over libusb (HID interface 2) — RECOMMENDED ──────────
#
# هذا الـ backend يطابق سلوك Android driver (XqpowerCan.cpp) بالضبط:
#   - pyusb (libusb) بدل pyserial/CDC
#   - يفصل kernel drivers (cdc_acm, hid-generic) من interfaces 0, 1, 2
#   - يطالب interface 2 (HID) + bulk endpoints
#   - init يقتصر على CMD_RECEIVE_ALL (الباقي مضبوط مسبقاً في firmware)
#
# مستمد من /home/yoga/m13/m13/servo_characterization/xqpower.py (مُختَبَر يعمل)
#
# ملاحظة هامة: السيرفوهات XQPOWER لا تدعم auto-report عبر OD 0x2200 — يُطلِق
# SDO ABORT 0x11000906. يجب أن يقوم runner بـ SDO polling على OD 0x6002 كل
# 5-10ms لجمع feedback. direct_runner.py يُدير ذلك تلقائياً.

class _CanLinUsbBus(CanBus):
    """CAN_LIN_Tool V6.0 عبر libusb (interface 2, bulk endpoints)."""

    CMD_RECEIVE_ALL = bytes([0x03, 0x02, 0x03])

    def __init__(self, vendor_id: int = 0x2E3C, product_id: int = 0x5750,
                 interface: int = 2) -> None:
        if usb is None:
            raise RuntimeError(
                "pyusb غير مثبَّت — pip install pyusb  (يحتاج libusb-1.0)"
            )

        dev = usb.core.find(idVendor=vendor_id, idProduct=product_id)
        if dev is None:
            raise RuntimeError(
                f"CAN_LIN_Tool غير متصل ({vendor_id:04x}:{product_id:04x})"
            )

        # فصل kernel drivers عن interfaces 0,1,2 (cdc_acm + hid-generic)
        for ifn in (0, 1, 2):
            try:
                if dev.is_kernel_driver_active(ifn):
                    dev.detach_kernel_driver(ifn)
            except (usb.core.USBError, NotImplementedError):
                pass

        try:
            usb.util.claim_interface(dev, interface)
        except usb.core.USBError as e:
            raise RuntimeError(
                f"تعذّر claim interface {interface}: {e}\n"
                "(قد تحتاج صلاحية — جرّب sudo أو udev rule)"
            ) from e

        cfg = dev.get_active_configuration()
        intf = cfg[(interface, 0)]
        self._ep_out = usb.util.find_descriptor(
            intf, custom_match=lambda e:
            usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT
        )
        self._ep_in = usb.util.find_descriptor(
            intf, custom_match=lambda e:
            usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN
        )
        if self._ep_out is None or self._ep_in is None:
            raise RuntimeError(
                f"تعذّر العثور على bulk endpoints على interface {interface}"
            )
        self._dev = dev
        self._interface = interface

        # init: استقبل كل الرسائل (بقية config مضبوط مسبقاً في adapter)
        self._raw_write(self.CMD_RECEIVE_ALL)
        time.sleep(0.05)
        # flush أي ردود قديمة
        for _ in range(20):
            if not self._raw_read(timeout_ms=10):
                break

        self._rx_queue: "Queue[CanFrame]" = Queue(maxsize=10_000)
        self._stop = threading.Event()
        self._tx_lock = threading.Lock()
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name="canlin-usb-rx", daemon=True
        )
        self._rx_thread.start()

    # ── low-level USB ─────────────────────────────────────────────────

    def _raw_write(self, data: bytes) -> int:
        if self._ep_out is None:
            return -1
        try:
            return int(self._ep_out.write(data, timeout=200))
        except usb.core.USBError:
            return -1

    def _raw_read(self, timeout_ms: int = 20) -> bytes:
        if self._ep_in is None:
            return b""
        try:
            return bytes(self._ep_in.read(64, timeout=timeout_ms))
        except usb.core.USBError:
            return b""

    # ── RX parser (يعثر على إطارات 17-byte ضمن HID payloads) ──────────

    def _rx_loop(self) -> None:
        buf = bytearray()
        valid_id = lambda i: (
            i == 0x000
            or 0x180 <= i <= 0x1FF
            or 0x580 <= i <= 0x5FF
            or 0x600 <= i <= 0x67F
        )
        while not self._stop.is_set():
            chunk = self._raw_read(timeout_ms=20)
            if not chunk:
                continue
            buf.extend(chunk)
            rd = 0
            n = len(buf)
            now = time.monotonic()
            while rd + 17 <= n:
                b0 = buf[rd]
                if b0 not in (0x02, 0x08):
                    rd += 1
                    continue
                arb_id = buf[rd + 2] | (buf[rd + 3] << 8)
                if not valid_id(arb_id):
                    rd += 1
                    continue
                dlc = b0
                if dlc > 8:
                    rd += 1
                    continue
                data = bytes(buf[rd + 9:rd + 9 + dlc])
                if len(data) < 8:
                    data = data + b"\x00" * (8 - len(data))
                try:
                    self._rx_queue.put_nowait(CanFrame(now, arb_id, data))
                except Exception:
                    pass
                rd += 17
            if rd > 0:
                del buf[:rd]
            # cap buffer growth
            if len(buf) > 4096:
                del buf[:-512]

    # ── CanBus interface ──────────────────────────────────────────────

    def send(self, arb_id: int, data: bytes) -> None:
        if len(data) > 8:
            raise ValueError("CAN DLC max 8 bytes")
        pad = bytes(data) + b"\x00" * (8 - len(data))
        frame = bytes([0x01]) + (arb_id & 0xFFFFFFFF).to_bytes(4, "little") \
                + b"\x00" * 4 + bytes([0, 0, len(data)]) + pad
        with self._tx_lock:
            n = self._raw_write(frame)
            if n != len(frame):
                raise RuntimeError(f"canlin_usb TX failed ({n}/{len(frame)})")

    def recv(self, timeout_s: float = 0.0) -> Optional[CanFrame]:
        try:
            if timeout_s > 0:
                return self._rx_queue.get(timeout=timeout_s)
            return self._rx_queue.get_nowait()
        except Empty:
            return None

    def close(self) -> None:
        self._stop.set()
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=1.0)
        try:
            usb.util.release_interface(self._dev, self._interface)
        except Exception:
            pass
        try:
            usb.util.dispose_resources(self._dev)
        except Exception:
            pass
        self._dev = None
        self._ep_out = None
        self._ep_in = None


# ─── XqpowerBus adapter (servo_characterization/xqpower.py) ─────────────────
#
# يُعيد استخدام XqpowerBus المُختَبَرة من مشروع servo_characterization. هذا الـ
# backend يُقدّم كل طبقة XQPOWER (NMT, polling, position commands) داخلياً،
# ويُعيد إطارات CAN_LIN_Tool الخام إلى direct_runner عبر واجهة CanBus.
#
# الاستخدام الموصى به لأن servo_characterization هو الوحيد الذي أثبت تحريك
# السيرفوهات حتى الآن.

class _XqpowerBusAdapter(CanBus):
    """Wrapper حول XqpowerBus (من /servo_characterization/xqpower.py)."""

    def __init__(self, node_ids: tuple = (1, 2, 3, 4),
                 poll_interval_us: int = 5000) -> None:
        import sys
        from pathlib import Path
        # make /servo_characterization importable
        sc_path = Path("/home/yoga/m13/m13/servo_characterization")
        if str(sc_path) not in sys.path:
            sys.path.insert(0, str(sc_path))
        try:
            from xqpower import XqpowerBus
        except ImportError as e:
            raise RuntimeError(
                f"تعذّر استيراد XqpowerBus من {sc_path}: {e}"
            ) from e

        self._bus = XqpowerBus(
            node_ids=tuple(node_ids),
            poll_interval_us=poll_interval_us,
        )
        self._bus.open()
        self._bus.enable_rx_log(True)       # نحتاج log للحصول على CanFrame
        self._bus.init_all_servos(report_interval_ms=5, settle_s=0.8)
        self._bus.wait_for_all_online(timeout_s=5.0)
        self._node_ids = node_ids
        self._rx_cursor = 0                  # موضع القراءة في rx_log

    def send(self, arb_id: int, data: bytes) -> None:
        if not self._bus.can_send(arb_id, bytes(data)):
            raise RuntimeError(f"XqpowerBus tx failed id=0x{arb_id:03X}")

    def recv(self, timeout_s: float = 0.0) -> Optional[CanFrame]:
        # XqpowerBus يخزن الـ frames في rx_log (deque). نقرأ الجديد فقط.
        # بما أن rx_log deque يفقد القديم عند الامتلاء، نستخدم cursor تقريبي.
        t_end = time.monotonic() + max(0.0, timeout_s)
        while True:
            log = self._bus.rx_log
            # snapshot to list (deque has no __getitem__ for stale cursors)
            snap = list(log)
            if self._rx_cursor < len(snap):
                sc_frame = snap[self._rx_cursor]
                self._rx_cursor += 1
                return CanFrame(
                    t_s=sc_frame.timestamp_mono_ns / 1e9,
                    arb_id=sc_frame.can_id,
                    data=sc_frame.data,
                )
            # إذا انزاح cursor بسبب overflow، نصفّره
            if self._rx_cursor > len(snap):
                self._rx_cursor = len(snap)
            if timeout_s <= 0 or time.monotonic() >= t_end:
                return None
            time.sleep(0.001)

    def close(self) -> None:
        try:
            self._bus.close()
        except Exception:
            pass
