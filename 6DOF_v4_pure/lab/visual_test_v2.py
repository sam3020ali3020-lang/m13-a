#!/usr/bin/env python3
"""Visual servo test v2 — uses XqpowerBus directly like /direct does.

Sends large alternating commands to each servo so you can SEE motion.
"""
import sys
import time
from pathlib import Path

# Import XqpowerBus from /servo_characterization (proven working)
sys.path.insert(0, "/home/yoga/m13/m13/servo_characterization")
from xqpower import XqpowerBus  # type: ignore

NODE_IDS = (1, 2, 3, 4)


def main():
    print("="*70)
    print("  🎬 VISUAL SERVO TEST — using XqpowerBus directly")
    print("="*70)
    print()
    print("Opening XqpowerBus and initializing servos...")

    bus = XqpowerBus(node_ids=NODE_IDS, poll_interval_us=5000)
    bus.open()
    bus.enable_rx_log(True)
    bus.init_all_servos(report_interval_ms=10, settle_s=0.8)

    # Wait for all servos to come online
    print("Waiting for all servos to report online...")
    try:
        bus.wait_for_all_online(timeout_s=5.0)
        print("  ✅ all 4 servos ONLINE")
    except Exception as e:
        print(f"  ⚠️ servos not all online: {e}")

    # Print initial positions (slot index 0-3, not node_id)
    for i in range(len(NODE_IDS)):
        fb = bus.get_feedback(i)
        print(f"  Servo {i} (node 0x{NODE_IDS[i]:02X}): pos={fb.position_deg:+.2f}° online={fb.online} samples={fb.sample_count}")

    print()
    print("🎯 TEST SEQUENCE: each servo moves 0° → +25° → -25° → +25° → -25° → 0°")
    print("   ⚠️  WARNING: 25° خارِج النِّطاق الأمِن ±20° — راقِب السيرفو جيّداً!")
    print("   2 seconds per position")
    print()
    print("="*70)

    sequence = [
        (0.0,   "Zero"),
        (+25.0, "MAX POSITIVE +25°"),
        (-25.0, "MAX NEGATIVE -25°"),
        (+25.0, "Back to +25°"),
        (-25.0, "Back to -25°"),
        (0.0,   "Return to ZERO"),
    ]

    for slot in range(len(NODE_IDS)):
        nid = NODE_IDS[slot]
        print(f"\n┌─ SERVO {slot} (node 0x{nid:02X}) ──────────────────────────────┐")
        for angle, desc in sequence:
            # Send position command (slot, angle, limit)
            ok = bus.set_position_deg(slot, angle, 25.0)
            print(f"│  {desc:40s}", end='')
            # Wait 2 seconds for movement
            time.sleep(2.0)
            # Read feedback after motion
            fb = bus.get_feedback(slot)
            if fb.online and fb.sample_count > 0:
                err = abs(fb.position_deg - angle) if angle != 0 else abs(fb.position_deg)
                marker = "" if err < 2.0 else " "
                print(f" cmd={angle:+6.1f}°  fb={fb.position_deg:+6.2f}°  err={err:.2f}° {marker}")
            else:
                print(f" cmd={angle:+6.1f}°   NO FEEDBACK")
        print(f"└────────────────────────────────────────────────────┘")

    # Safety: zero all
    print("\nReturning all servos to ZERO...")
    for slot in range(len(NODE_IDS)):
        bus.set_position_deg(slot, 0.0, 25.0)
    time.sleep(1.5)

    print("\nFinal positions after zero command:")
    for slot in range(len(NODE_IDS)):
        fb = bus.get_feedback(slot)
        print(f"  Servo {slot}: pos={fb.position_deg:+.2f}° (online={fb.online})")

    bus.close()
    print()
    print("="*70)
    print("  ✅ TEST DONE — هل رَأَيت الحَرَكَة الآن؟")
    print("="*70)


if __name__ == "__main__":
    main()
