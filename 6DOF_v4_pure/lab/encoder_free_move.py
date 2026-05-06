#!/usr/bin/env python3
"""🔬 اختِبار encoder مَع تَعطيل السيرفو — يَسمَح بالحَرَكَة اليَدَويَّة.

الفِكرَة:
  1. نُنشِئ bus ونُهَيِّئ السيرفو
  2. نُرسِل NMT STOP → السيرفو يُصبِح "limp" (بدون عَزم إمساك)
  3. أَنت تُحَرِّكه بيَدِك بحُرِّيَّة تامَّة
  4. كلّ ثانية نُرسِل NMT START قَصير لقِراءَة fb، ثُمَّ NMT STOP مُجَدَّداً
  5. نَرى إذا encoder يَتَتَبَّع حَرَكَتك

⚠️ قَد لا يَعمَل على بَعض السيرفوهات الَّتي تَفصِل SDO في حالَة Stopped.
   إذا لَم يُسَجِّل fb أَيّ تَغَيُّر، جَرِّب encoder_power_cycle.py

الاستِخدام:
    python3 encoder_free_move.py [slot]
"""
import sys
import time

sys.path.insert(0, "/home/yoga/m13/m13/servo_characterization")
from xqpower import XqpowerBus  # type: ignore

NODE_IDS = (1, 2, 3, 4)
READ_INTERVAL_S = 0.8  # كلّ 800ms نَقرأ fb


def main():
    slot = int(sys.argv[1]) if len(sys.argv) >= 2 else 0
    assert 0 <= slot <= 3

    print("=" * 65)
    print(f"  🔬 اختِبار encoder مَع تَعطيل السيرفو")
    print(f"     السيرفو {slot + 1} (slot={slot}, node=0x{NODE_IDS[slot]:02X})")
    print("=" * 65)
    print()
    print("  الخُطوات:")
    print("   1. السكريبت يُهَيِّئ ثُمَّ يُعَطِّل السيرفو (NMT STOP)")
    print("   2. ستَشعُر أنّ السيرفو أَصبَحَ 'limp' (بلا عَزم إمساك)")
    print("   3. حَرِّكه بيَدِك إلى زَوايا مُختَلِفَة (+30°، -30°، إلخ)")
    print("   4. راقِب العَمود fb بَعد كلّ حَرَكَة")
    print("   5. اضغَط Ctrl+C للإنهاء")
    print()
    input("اضغَط Enter للبَدء...")
    print()

    bus = XqpowerBus(node_ids=NODE_IDS, poll_interval_us=5000)
    bus.open()
    bus.init_all_servos(report_interval_ms=10, settle_s=0.8)
    bus.wait_for_all_online(timeout_s=5.0)
    print("✅ السيرفو online — الآن أُعَطِّله")

    # إيقاف polling المُستَمِرّ
    bus._poll_enabled.clear()
    time.sleep(0.2)

    # NMT STOP → السيرفو limp
    bus.nmt_stop(slot)
    time.sleep(0.3)
    print("   ⚠️ NMT STOP أُرسِلَ — جَرِّب تَحريك السيرفو بيَدِك الآن")
    print()

    print("─" * 65)
    print(f"  {'t (s)':>8s}  {'fb':>10s}  {'Δ':>8s}  {'samples':>10s}")
    print("─" * 65)

    t_start = time.time()
    last_fb = 0.0
    max_fb = 0.0
    min_fb = 0.0
    try:
        while True:
            now = time.time() - t_start

            # NMT START سَريع → قِراءَة fb → NMT STOP مُجَدَّداً
            bus._send_nmt_start_slot(slot) if hasattr(bus, "_send_nmt_start_slot") else bus.can_send(0x000, bytes([0x01, NODE_IDS[slot]]))
            time.sleep(0.05)
            # أَرسِل read request
            bus.read_position(slot)
            time.sleep(0.15)

            fb = bus.get_feedback(slot)
            fb_deg = fb.position_deg
            dfb = fb_deg - last_fb

            if fb_deg > max_fb:
                max_fb = fb_deg
            if fb_deg < min_fb:
                min_fb = fb_deg

            mark = "⚡" if abs(dfb) > 1.0 else " "
            print(f"  {now:>8.2f}  {fb_deg:>+10.3f}  {dfb:>+8.3f}  "
                  f"{fb.sample_count:>10d}  {mark}")
            last_fb = fb_deg

            # NMT STOP مُجَدَّداً ليَبقى limp
            bus.nmt_stop(slot)
            time.sleep(READ_INTERVAL_S)
    except KeyboardInterrupt:
        pass

    print("─" * 65)
    print()
    print(f"  📋 مُلَخَّص:")
    print(f"     أَقصى fb مَوجَب: {max_fb:+.2f}°")
    print(f"     أَقصى fb سالِب: {min_fb:+.2f}°")
    print(f"     المَدى الَّذي رَآه encoder: {max_fb - min_fb:.2f}°")
    print()

    if max_fb - min_fb < 5.0:
        print("  🔴 encoder لَم يُسَجِّل حَرَكَة — إمَّا:")
        print("       - encoder عالِق (خَلَل داخِلي)")
        print("       - أَو NMT STOP مَنَع feedback reading")
        print("       - جَرِّب power cycle وأَعِد الاختِبار")
    elif max_fb < 20 and min_fb > -20:
        print("  ⚠️ encoder رَأَى حَرَكَة قَليلَة — حَرِّك السيرفو أَكثَر")
    else:
        print(f"  ✅ encoder شَغّال — رَأَى مَدى {max_fb - min_fb:.1f}°")
        print(f"      المَدى المُوجَب: {max_fb:.1f}°")
        print(f"      المَدى السالِب: {min_fb:.1f}°")

    # إعادَة تَفعيل السيرفو
    bus.can_send(0x000, bytes([0x01, NODE_IDS[slot]]))
    time.sleep(0.3)
    bus.set_position_deg(slot, 0.0, 30.0)
    time.sleep(1.0)
    bus.close()


if __name__ == "__main__":
    main()
