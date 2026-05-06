#!/usr/bin/env python3
"""اختبار سيرفو واحِد فَقَط بزاوية ±25°.

استخدام:  python3 test_single_servo.py <slot>
         حَيث slot = 0, 1, 2, 3  (للسيرفو 1, 2, 3, 4)
"""
import sys
import time

sys.path.insert(0, "/home/yoga/m13/m13/servo_characterization")
from xqpower import XqpowerBus  # type: ignore

NODE_IDS = (1, 2, 3, 4)


def main():
    if len(sys.argv) != 2:
        print("usage: python3 test_single_servo.py <slot 0-3>")
        sys.exit(1)
    target_slot = int(sys.argv[1])
    assert 0 <= target_slot <= 3, "slot must be 0-3"

    print("="*70)
    print(f"  🎯 اختبار السيرفو {target_slot+1} (slot={target_slot}, node=0x{NODE_IDS[target_slot]:02X})")
    print("="*70)

    bus = XqpowerBus(node_ids=NODE_IDS, poll_interval_us=5000)
    bus.open()
    bus.enable_rx_log(True)
    bus.init_all_servos(report_interval_ms=10, settle_s=0.8)
    bus.wait_for_all_online(timeout_s=5.0)
    print("\n✅ All 4 servos online (but only testing this one)")
    print()

    # Initial state
    fb0 = bus.get_feedback(target_slot)
    print(f"📊 حالَة ابتِدائيَّة: pos={fb0.position_deg:+.3f}° samples={fb0.sample_count}")
    print()

    # Sequence: 0 → +25 → -25 → +25 → -25 → 0
    sequence = [
        (0.0,   "ZERO"),
        (+25.0, "+25°"),
        (-25.0, "-25°"),
        (+25.0, "+25°"),
        (-25.0, "-25°"),
        (0.0,   "ZERO"),
    ]

    print("Sequence: 0° → +25° → -25° → +25° → -25° → 0°")
    print("3 seconds per position. Watch the fin!\n")
    print(f"{'step':>6s}  {'cmd':>8s}  {'fb_قَبل':>10s}  {'fb_بَعد':>10s}  {'خَطأ':>8s}  {'age_ms':>8s}  {'status':>8s}")
    print("-"*70)

    failures = 0
    for i, (ang, desc) in enumerate(sequence):
        fb_before = bus.get_feedback(target_slot)
        bus.set_position_deg(target_slot, ang, 30.0)
        time.sleep(3.0)
        fb_after = bus.get_feedback(target_slot)
        age_ms = (time.monotonic_ns() - fb_after.timestamp_mono_ns) / 1e6
        err = abs(fb_after.position_deg - ang)
        ok = err < 1.0
        if not ok:
            failures += 1
        status = "✅" if ok else "❌"
        print(f"{i+1:>6d}  {ang:+8.1f}  {fb_before.position_deg:+10.3f}  "
              f"{fb_after.position_deg:+10.3f}  {err:>8.3f}  {age_ms:>8.0f}  {status:>8s}")

    print()
    print("="*70)
    print(f"  النَّتيجَة: {len(sequence) - failures}/{len(sequence)} steps نَجَحَت")
    if failures > 0:
        print(f"  ⚠️  {failures} فَشَل!")
    print("="*70)

    # Safety: zero all
    for s in range(4):
        bus.set_position_deg(s, 0.0, 30.0)
    time.sleep(1.0)
    bus.close()


if __name__ == "__main__":
    main()
