#!/usr/bin/env python3
"""اختِبار: أَرسِل نَفس الزاوية لكلّ السيرفوهات الأَربَعَة، وقارِن fb.

الاستِخدام:
    python3 test_all_at_angle.py [angle_deg]
    افتِراضي: 14°
"""
import sys
import time

sys.path.insert(0, "/home/yoga/m13/m13/servo_characterization")
from xqpower import XqpowerBus  # type: ignore

NODE_IDS = (1, 2, 3, 4)


def main():
    angle = float(sys.argv[1]) if len(sys.argv) >= 2 else 14.0

    print("=" * 70)
    print(f"  🎯 إرسال {angle:+.1f}° لكلّ السيرفوهات الأَربَعَة")
    print("=" * 70)

    bus = XqpowerBus(node_ids=NODE_IDS, poll_interval_us=5000)
    bus.open()
    bus.init_all_servos(report_interval_ms=10, settle_s=0.8)
    bus.wait_for_all_online(timeout_s=5.0)
    print("✅ السيرفوهات online")
    print()

    print(f"📍 إرسال cmd={angle:+.1f}° لكلّ سيرفو...")
    print()
    print(f"  {'t':>5s}  {'srv1':>10s}  {'srv2':>10s}  "
          f"{'srv3':>10s}  {'srv4':>10s}")
    print("─" * 65)

    t_start = time.time()
    last_print = 0.0
    try:
        while True:
            for s in range(4):
                bus.set_position_deg(s, angle, 60.0)

            now = time.time() - t_start
            if now - last_print >= 0.5:
                vals = []
                for s in range(4):
                    fb = bus.get_feedback(s).position_deg
                    err = fb - angle
                    mark = "⚠️" if abs(err) > 1.0 else "✅"
                    vals.append(f"{fb:+7.2f}°{mark}")
                print(f"  {now:>5.1f}  {vals[0]:>10s}  {vals[1]:>10s}  "
                      f"{vals[2]:>10s}  {vals[3]:>10s}")
                last_print = now
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass

    # عَودَة لـ 0
    for s in range(4):
        bus.set_position_deg(s, 0.0, 60.0)
    time.sleep(1.5)
    bus.close()


if __name__ == "__main__":
    main()
