#!/usr/bin/env python3
"""تَثبيت كلّ السيرفوهات الأَربَعَة عَلى 0° للمُلاحَظَة البَصَريَّة.

يُرسِل cmd=0° مُستَمِرّ لكلّ السيرفوهات ويَطبَع fb لتَرى أَين يَستَقِرّ
كلّ سيرفو فيزيائيّاً (إذا zero مُعايَر بشَكل صَحيح، الكلّ يَكون في المَركَز).
"""
import sys
import time

sys.path.insert(0, "/home/yoga/m13/m13/servo_characterization")
from xqpower import XqpowerBus  # type: ignore

NODE_IDS = (1, 2, 3, 4)


def main():
    bus = XqpowerBus(node_ids=NODE_IDS, poll_interval_us=5000)
    bus.open()
    bus.init_all_servos(report_interval_ms=10, settle_s=0.8)
    bus.wait_for_all_online(timeout_s=5.0)
    print("✅ السيرفوهات online — كلّها على 0°")
    print()
    print("──────────────────────────────────────────────────────────")
    print(f"  {'t':>5s}  {'srv1':>10s}  {'srv2':>10s}  "
          f"{'srv3':>10s}  {'srv4':>10s}")
    print("──────────────────────────────────────────────────────────")

    t_start = time.time()
    last_print = 0.0
    try:
        while True:
            # cmd = 0 لكلّ السيرفوهات
            for s in range(4):
                bus.set_position_deg(s, 0.0, 60.0)

            now = time.time() - t_start
            if now - last_print >= 0.5:
                vals = []
                for s in range(4):
                    fb = bus.get_feedback(s).position_deg
                    vals.append(f"{fb:+9.2f}°")
                print(f"  {now:>5.1f}  {vals[0]:>10s}  {vals[1]:>10s}  "
                      f"{vals[2]:>10s}  {vals[3]:>10s}")
                last_print = now
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass

    bus.close()


if __name__ == "__main__":
    main()
