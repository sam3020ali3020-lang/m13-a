#!/usr/bin/env python3
"""🔬 فَحص صَلاحيَّة encoder: قَرَأَ fb مُباشَرَةً بَينَما أَنت تُحَرِّك يَدَوِيّاً.

الفِكرَة:
  1. نَبدَأ بـ cmd=0° (السيرفو يُحاوِل الإمساك بالصِّفر)
  2. أَنت تَدفَع السيرفو بيَدِك إلى زاوية مُختَلِفَة
  3. السكريبت يَطبَع fb بشَكل مُستَمِرّ
  4. إذا encoder شَغّال → fb يَتَغَيَّر مَع حَرَكَتِك
  5. إذا encoder عالِق → fb يَبقى عَلى قيمَة واحِدَة

الاستِخدام:
    python3 encoder_sanity_check.py [slot]
    (ثُمَّ حَرِّك السيرفو يَدَوِيّاً، Ctrl+C للإنهاء)
"""
import sys
import time

sys.path.insert(0, "/home/yoga/m13/m13/servo_characterization")
from xqpower import XqpowerBus  # type: ignore

NODE_IDS = (1, 2, 3, 4)


def main():
    slot = int(sys.argv[1]) if len(sys.argv) >= 2 else 0
    assert 0 <= slot <= 3

    print("=" * 65)
    print(f"  🔬 فَحص encoder — السيرفو {slot + 1}")
    print(f"     (slot={slot}, node=0x{NODE_IDS[slot]:02X})")
    print("=" * 65)
    print()
    print("  ⚠️  تَعليمات:")
    print("      1. الآن السيرفو ثابِت عَلى 0°")
    print("      2. ادفَع ذِراع السيرفو بيَدِك إلى زَوايا مُختَلِفَة")
    print("      3. راقِب قِراءَة fb في الشاشَة")
    print("      4. اضغَط Ctrl+C للإنهاء")
    print()
    print("  النَّتيجَة المُتَوَقَّعَة:")
    print("     ✅ fb يَتَغَيَّر مَع حَرَكَتِك → encoder شَغّال")
    print("     ❌ fb عالِق عَلى قيمَة واحِدَة → encoder مُعَطَّل")
    print()
    input("اضغَط Enter للبَدء...")
    print()

    bus = XqpowerBus(node_ids=NODE_IDS, poll_interval_us=5000)
    bus.open()
    bus.init_all_servos(report_interval_ms=10, settle_s=0.8)
    bus.wait_for_all_online(timeout_s=5.0)
    print("✅ السيرفوهات online")
    print()

    # cmd=0° مُستَمِرّ (السيرفو سيُحاوِل الإمساك بالصِّفر، لكِن بعَزم مَحدود)
    bus.set_position_deg(slot, 0.0, 30.0)
    time.sleep(0.5)

    print("─" * 65)
    print(f"  {'t (s)':>8s}  {'fb':>10s}  {'Δ':>8s}  {'samples':>10s}")
    print("─" * 65)

    t_start = time.time()
    last_fb = 0.0
    max_fb = 0.0
    min_fb = 0.0
    last_print = 0.0
    try:
        while True:
            now = time.time() - t_start
            fb = bus.get_feedback(slot)
            fb_deg = fb.position_deg
            dfb = fb_deg - last_fb

            # تَتَبَّع أَقصى مَدى وَصَل إليه encoder
            if fb_deg > max_fb:
                max_fb = fb_deg
            if fb_deg < min_fb:
                min_fb = fb_deg

            # نَطبَع 10 مَرَّات/ثانية
            if now - last_print >= 0.1:
                mark = "⚡" if abs(dfb) > 1.0 else " "
                print(f"  {now:>8.2f}  {fb_deg:>+10.3f}  "
                      f"{dfb:>+8.3f}  {fb.sample_count:>10d}  {mark}")
                last_print = now
                last_fb = fb_deg

            # استَمِرّ بإرسال cmd=0 لكَي لا يَفقِد السيرفو النَّشاط
            if int(now * 5) % 5 == 0:
                bus.set_position_deg(slot, 0.0, 30.0)

            time.sleep(0.05)
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
        print("  🔴 encoder يَبدو عالِقاً! لَم يُسَجِّل حَرَكَة كافيَة.")
    elif max_fb < 20 and min_fb > -20:
        print("  ⚠️ encoder لَم يَرَ حَرَكَة كَبيرَة — جَرِّب دَفعَ السيرفو لزَوايا أَكبَر.")
    else:
        print(f"  ✅ encoder شَغّال — رَأَى مَدى {max_fb - min_fb:.1f}°")

    # عَودَة للصِّفر
    bus.set_position_deg(slot, 0.0, 30.0)
    time.sleep(1.0)
    bus.close()


if __name__ == "__main__":
    main()
