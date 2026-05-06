#!/usr/bin/env python3
"""Visual proof: move servos by large amounts so user can SEE motion.

This script sends alternating ±15° commands to each of the 4 servos with
2-second pauses. If the CAN hardware is real, the fins will move visibly.
If there's a problem (no CAN, wrong backend, servo offline), nothing moves.
"""
import sys
import time
from pathlib import Path

# Allow importing /direct modules
_THIS = Path(__file__).resolve().parent
_DIRECT = _THIS.parent / "direct"
sys.path.insert(0, str(_DIRECT))

from can_driver import open_can
from xqpower_protocol import (
    encode_nmt_start,
    encode_set_position,
    encode_set_report_interval,
    decode_frame,
)

# Same backend as /lab config
CAN_CFG = {
    "backend": "xqpower_bus",
    "xqpower_bus": {"node_ids": [1, 2, 3, 4], "poll_interval_us": 5000},
}

NODE_IDS = [1, 2, 3, 4]
ANGLE_LIMIT = 20.0
UNITS_PER_DEG = 18.0


def main():
    print("="*70)
    print("  🎬 VISUAL SERVO TEST — watch the fins!")
    print("="*70)
    print()
    print("Opening CAN bus with xqpower_bus backend...")
    bus = open_can(CAN_CFG)
    print(f"  ✅ bus opened: {type(bus).__name__}")
    print()

    # NMT Start
    print("Sending NMT Start to all servos...")
    for nid in NODE_IDS:
        arb, data = encode_nmt_start(nid)
        bus.send(arb, data)
    time.sleep(0.1)

    # Enable feedback reporting @ 10ms
    for nid in NODE_IDS:
        arb, data = encode_set_report_interval(nid, 10)
        bus.send(arb, data)
    time.sleep(0.1)

    print()
    print("🎯 TEST SEQUENCE:")
    print("   Each servo will move: 0° → +15° → -15° → +15° → 0°")
    print("   2 seconds between each position")
    print("   You should see ~30° total swing per fin — OBVIOUSLY visible")
    print()
    print("="*70)
    print()

    sequence = [
        (0.0,  "Zero position"),
        (15.0, "MAX POSITIVE (+15°)"),
        (-15.0,"MAX NEGATIVE (-15°)"),
        (15.0, "Back to +15°"),
        (-15.0,"Back to -15°"),
        (0.0,  "Return to zero"),
    ]

    for servo_idx, nid in enumerate(NODE_IDS):
        print(f"\n┌─ SERVO {servo_idx} (node 0x{nid:02X}) ──────────────────────────┐")
        for angle, desc in sequence:
            arb, data = encode_set_position(nid, angle, ANGLE_LIMIT, UNITS_PER_DEG)
            bus.send(arb, data)
            print(f"│  {desc:30s} → sent cmd={angle:+.1f}°")
            # Wait and read feedback during pause
            t_start = time.monotonic()
            fb_values = []
            while time.monotonic() - t_start < 2.0:
                # Try to read feedback
                try:
                    frame = bus.recv(timeout_s=0.05)
                    if frame is not None:
                        decoded = decode_frame(frame.arb_id, frame.data)
                        if decoded and decoded.node_id == nid:
                            if decoded.position_deg is not None:
                                fb_values.append(decoded.position_deg)
                except Exception:
                    pass
            if fb_values:
                print(f"│      fb_last = {fb_values[-1]:+.2f}°  ({len(fb_values)} frames)")
            else:
                print(f"│      ⚠️  NO FEEDBACK RECEIVED")
        print(f"└──────────────────────────────────────────────────────┘")

    # Zero all servos at end (safety)
    print("\nZeroing all servos...")
    for nid in NODE_IDS:
        arb, data = encode_set_position(nid, 0.0, ANGLE_LIMIT, UNITS_PER_DEG)
        bus.send(arb, data)
    time.sleep(0.5)

    bus.close()
    print()
    print("="*70)
    print("  ✅ TEST COMPLETE")
    print("="*70)
    print()
    print("WHAT YOU SHOULD HAVE SEEN:")
    print("  - Each fin swings ±15° visibly (30° total)")
    print("  - Movements are clear, not tremors")
    print("  - 4 servos move ONE AT A TIME (each for ~12 seconds)")
    print("  - Total test: ~48 seconds")
    print()


if __name__ == "__main__":
    main()
