#!/usr/bin/env python3
"""🔬 قِياس حَرَكَة يَدَويَّة لجَميع السيرفوهات

الفِكرَة:
  - نُفَعِّل CAN communication + polling (لاستِقبال fb)
  - لا نُرسِل أَيّ cmd مَوقِع → السيرفو يَبقى سَلبي (أَمَل)
  - أَنت تُحَرِّك الدَّفَّات يَدَوِيّاً
  - نَقيس fb لكلّ سيرفو في الوَقت الحَقيقي

⚠️ إذا لا يَزال السيرفو يُقاوِم، جَرِّب "hand-follow mode":
    python3 measure_manual_movement.py --follow

الاستِخدام:
    python3 measure_manual_movement.py
    python3 measure_manual_movement.py --follow   # يَتبَع يَدَك
"""
import sys
import time

sys.path.insert(0, "/home/yoga/m13/m13/servo_characterization")
from xqpower import XqpowerBus  # type: ignore

NODE_IDS = (1, 2, 3, 4)
NMT_ID = 0x000


def main():
    follow_mode = "--follow" in sys.argv

    print("=" * 70)
    print("  🔬 قِياس حَرَكَة يَدَويَّة لجَميع السيرفوهات")
    print("=" * 70)
    if follow_mode:
        print("  وَضع: HAND-FOLLOW (السيرفو يَتبَع حَرَكَة يَدَك)")
    else:
        print("  وَضع: PASSIVE (لا نُرسِل أَيّ cmd)")
    print()
    print("  التَّعليمات:")
    print("   1. ستَرى fb لجَميع السيرفوهات الأَربَعَة في الوَقت الحَقيقي")
    print("   2. حَرِّك كلّ دَفَّة يَدَوِيّاً إلى أَقصى زاوية مُمكِنَة")
    print("   3. راقِب ماذا يَقرَأ encoder")
    print("   4. اضغَط Ctrl+C للإنهاء ورُؤيَة المُلَخَّص")
    print()
    input("اضغَط Enter للبَدء...")
    print()

    # نَفتَح bus بدون init_all_servos (لا نُرسِل cmd)
    bus = XqpowerBus(node_ids=NODE_IDS, poll_interval_us=5000)
    bus.open()

    # NMT START فَقَط (تَفعيل CAN) — بدون auto-report إعداد
    # نُعَطِّل polling أَوَّلاً
    bus._poll_enabled.clear()
    time.sleep(0.2)
    for slot, node in enumerate(NODE_IDS):
        bus.can_send(NMT_ID, bytes([0x01, node]))
        time.sleep(0.05)

    # نُفَعِّل polling لاستِقبال fb (قِراءَة فَقَط، ليست cmd)
    bus._poll_enabled.set()
    time.sleep(1.0)

    # انتَظِر أَوَّل قِراءَة
    bus.wait_for_all_online(timeout_s=5.0)
    print("✅ CAN communication فَعّال — الآن حَرِّك الدَّفَّات يَدَوِيّاً")
    print()

    print("─" * 70)
    print(f"  {'t':>5s}  "
          f"{'srv1':>10s}  {'srv2':>10s}  {'srv3':>10s}  {'srv4':>10s}")
    print("─" * 70)

    t_start = time.time()
    last_fb = [0.0, 0.0, 0.0, 0.0]
    max_fb = [0.0, 0.0, 0.0, 0.0]
    min_fb = [0.0, 0.0, 0.0, 0.0]
    last_print = 0.0
    try:
        while True:
            now = time.time() - t_start

            # في follow mode: كلّ سيرفو يَتبَع fb الحالي (cmd = fb)
            if follow_mode:
                for s in range(4):
                    fb = bus.get_feedback(s)
                    bus.set_position_deg(s, fb.position_deg, 60.0)

            # كلّ 0.2s اطبَع الحالَة
            if now - last_print >= 0.2:
                vals = []
                for s in range(4):
                    fb = bus.get_feedback(s)
                    fb_deg = fb.position_deg
                    if fb_deg > max_fb[s]:
                        max_fb[s] = fb_deg
                    if fb_deg < min_fb[s]:
                        min_fb[s] = fb_deg
                    dfb = fb_deg - last_fb[s]
                    mark = "⚡" if abs(dfb) > 1.0 else " "
                    vals.append(f"{fb_deg:+7.2f}°{mark}")
                    last_fb[s] = fb_deg
                print(f"  {now:>5.1f}  {vals[0]:>10s}  {vals[1]:>10s}  "
                      f"{vals[2]:>10s}  {vals[3]:>10s}")
                last_print = now

            time.sleep(0.05)
    except KeyboardInterrupt:
        pass

    print("─" * 70)
    print()
    print("  📋 مُلَخَّص الحَرَكَة المَرصودَة لكلّ سيرفو:")
    print("─" * 70)
    print(f"  {'servo':>6s}  {'max (+)':>10s}  {'min (-)':>10s}  "
          f"{'total':>10s}  verdict")
    print("─" * 70)
    for s in range(4):
        total = max_fb[s] - min_fb[s]
        if total < 5.0:
            verdict = "🔴 لَم يَتَحَرَّك — مُجَمَّد"
        elif max_fb[s] < 20 and min_fb[s] > -20:
            verdict = "⚠️ حَرَكَة مَحدودَة"
        else:
            verdict = "✅ encoder شَغّال"
        print(f"  #{s+1:>5d}  {max_fb[s]:>+10.2f}  {min_fb[s]:>+10.2f}  "
              f"{total:>10.2f}  {verdict}")
    print("─" * 70)

    bus.close()


if __name__ == "__main__":
    main()
