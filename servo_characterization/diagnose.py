#!/usr/bin/env python3
"""Low-level diagnostic: open CAN_LIN_Tool, send init, NMT, position commands
and dump every raw byte received. Used to debug why feedback is silent."""

import os
import select
import sys
import time

from xqpower import (
    CMD_BAUD_500K, CMD_RECEIVE_ALL, CMD_SAVE, CMD_TERMINATION_ON,
    _encode_canlin_tx, _decode_canlin_stream,
    NMT_ID, SDO_TX_BASE, SDO_WRITE,
    find_canlin_hidraw,
)

PORT = sys.argv[1] if len(sys.argv) > 1 else (find_canlin_hidraw() or "/dev/hidraw2")

print(f"[diag] Opening {PORT} ...")
fd = os.open(PORT, os.O_RDWR | os.O_NONBLOCK)
poll = select.poll()
poll.register(fd, select.POLLIN)


def hid_write(payload: bytes) -> int:
    """Pad to 63-byte HID report and write."""
    if len(payload) > 62:
        raise ValueError(f"payload {len(payload)} > 62")
    report = b"\x00" + payload + b"\x00" * (62 - len(payload))
    return os.write(fd, report)


def drain():
    try:
        while True:
            if not poll.poll(0):
                return
            os.read(fd, 4096)
    except (OSError, BlockingIOError):
        pass


def read_available(wait_s):
    deadline = time.monotonic() + wait_s
    out = bytearray()
    while time.monotonic() < deadline:
        rem_ms = max(1, int((deadline - time.monotonic()) * 1000))
        if poll.poll(rem_ms):
            try:
                out.extend(os.read(fd, 4096))
            except (OSError, BlockingIOError):
                break
    return bytes(out)


def write_and_dump(label, data, wait_s=0.15):
    print(f"\n[diag] >> {label}  ({len(data)} bytes: {data.hex(' ')})")
    hid_write(data)
    rx = read_available(wait_s)
    if rx:
        print(f"[diag] << {len(rx)} bytes RX: {rx.hex(' ')}")
    else:
        print(f"[diag] << (no RX)")
    return rx

# 1) Raw init sequence
print("\n=== CAN_LIN_Tool V6.0 init ===")
write_and_dump("BAUD_500K", CMD_BAUD_500K)
write_and_dump("SAVE", CMD_SAVE)
write_and_dump("TERMINATION_ON", CMD_TERMINATION_ON)
write_and_dump("RECEIVE_ALL", CMD_RECEIVE_ALL)

drain()
print("\n=== NMT Start all nodes ===")
for node in (1, 2, 3, 4):
    frame = _encode_canlin_tx(NMT_ID, bytes([0x01, node]))
    write_and_dump(f"NMT_START node=0x{node:02X}", frame, wait_s=0.1)

print("\n=== Wait 1s for background PDO ===")
rx = read_available(1.0)
print(f"[diag] Passive RX ({len(rx)} bytes): {rx[:64].hex(' ') if rx else '(none)'}")

print("\n=== Set position to +5° on node 1 (repeated 10x) ===")
import struct
for i in range(10):
    pos_raw = int(round(5.0 * 18.0))
    data = struct.pack("<BBBBhBB", SDO_WRITE, 0x03, 0x60, 0x00, pos_raw, 0, 0)
    frame = _encode_canlin_tx(SDO_TX_BASE + 1, data)
    hid_write(frame)
    time.sleep(0.05)
rx = read_available(0.5)
print(f"[diag] RX after 10 position commands ({len(rx)} bytes):")
if rx:
    for i in range(0, min(len(rx), 320), 32):
        print("  ", rx[i:i+32].hex(' '))
    buf = bytearray(rx)
    frames, consumed = _decode_canlin_stream(buf)
    print(f"[diag] Decoded {len(frames)} CAN frames, consumed {consumed}/{len(rx)} bytes")
    for f in frames:
        print(f"    id=0x{f.can_id:03X}  data={f.data.hex(' ')}")

print("\n=== Set position 0° ===")
data = struct.pack("<BBBBhBB", SDO_WRITE, 0x03, 0x60, 0x00, 0, 0, 0)
for node in (1, 2, 3, 4):
    frame = _encode_canlin_tx(SDO_TX_BASE + node, data)
    hid_write(frame)
    time.sleep(0.05)

os.close(fd)
print("\n[diag] Done.")
