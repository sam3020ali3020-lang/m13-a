#!/usr/bin/env python3
"""ثَبِّت السيرفو على +25° لمُدَّة طَويلَة لقِياس بَصَري دَقيق.

استخدام:  python3 test_hold_25.py [slot] [angle]
         slot  = 0..3      (افتِراضي 0)
         angle = الزاوية   (افتِراضي 25.0)

اضغَط Ctrl+C لإنهاء الاختِبار وعَودَة السيرفو إلى 0°.
"""
import sys
import time

sys.path.insert(0, "/home/yoga/m13/m13/servo_characterization")
from xqpower import XqpowerBus  # type: ignore

NODE_IDS = (1, 2, 3, 4)


def main():
    target_slot = int(sys.argv[1]) if len(sys.argv) >= 2 else 0
    angle = float(sys.argv[2]) if len(sys.argv) >= 3 else 25.0
    assert 0 <= target_slot <= 3, "slot must be 0-3"

    print("=" * 70)
    print(f"  🎯 تَثبيت السيرفو {target_slot + 1} على {angle:+.1f}°")
    print(f"     (slot={target_slot}, node=0x{NODE_IDS[target_slot]:02X})")
    print("=" * 70)
    print()
    print("⚠️  تأَكَّد أنّ الدَّفَّة مَفصولَة عَن السيرفو")
    print("⚠️  حَضِّر مِنقَلَة وقِس زاوية شَفت السيرفو")
    print()
    print("  الجَدوَل المَرجِعي:")
    print(f"     سيناريو A (لا linkage):  الشَّفت يَدور {angle:.1f}°")
    print(f"     سيناريو B (مَع linkage): الشَّفت يَدور {angle * 1.8:.1f}°")
    print()
    print("اضغَط Ctrl+C لإنهاء الاختِبار وإعادَة السيرفو إلى 0°.")
    print()

    bus = XqpowerBus(node_ids=NODE_IDS, poll_interval_us=5000)
    bus.open()
    bus.enable_rx_log(True)
    bus.init_all_servos(report_interval_ms=10, settle_s=0.8)
    bus.wait_for_all_online(timeout_s=5.0)
    print("✅ السيرفوهات online\n")

    # Zero first
    for s in range(4):
        bus.set_position_deg(s, 0.0, 30.0)
    time.sleep(1.0)

    # Hold target angle — re-send periodically for safety
    print(f"📍 إرسال أَمر {angle:+.1f}° إلى السيرفو {target_slot + 1}...")
    bus.set_position_deg(target_slot, angle, 30.0)
    print()
    print("─" * 70)
    print(f"  ⏱️  الآن قِس زاوية الشَّفت بالمِنقَلَة")
    print("─" * 70)
    print()
    print(f"{'t (s)':>6s}  {'feedback':>10s}  {'sample_count':>14s}")
    print("─" * 70)

    t0 = time.time()
    try:
        while True:
            time.sleep(1.0)
            # Re-send command every cycle for safety (servo may timeout)
            bus.set_position_deg(target_slot, angle, 30.0)
            fb = bus.get_feedback(target_slot)
            t = time.time() - t0
            print(f"{t:>6.1f}  {fb.position_deg:+10.3f}  {fb.sample_count:>14d}")
    except KeyboardInterrupt:
        print()
        print("─" * 70)
        print("  ⏹️  إيقاف، إعادَة جَميع السيرفوهات إلى 0°...")
        for s in range(4):
            bus.set_position_deg(s, 0.0, 30.0)
        time.sleep(1.5)
        bus.close()
        print("  ✅ تَمّ")


if __name__ == "__main__":
    main()
