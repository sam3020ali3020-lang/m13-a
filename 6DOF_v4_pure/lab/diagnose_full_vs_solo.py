#!/usr/bin/env python3
"""تَشخيص: لِماذا السيرفو 4 يَعمَل لِوَحده لكن لا يَعمَل في الاختبار الكامِل؟

هذا السكربت:
  1. يُشغِّل سيكوينس كامِل عَلى السيرفوهات 1-3 (مِثل visual_test الكامِل)
  2. ثُمّ يَختَبِر السيرفو 4 بِدِقَّة مَع رَصد:
     - timestamp staleness (هَل الـ feedback جَديد؟)
     - abort_count (هَل هُناك أَخطاء SDO؟)
     - sample_count progression (هَل لا يَزال يُبلِغ؟)
  3. ثُمّ يَختَبِر السيرفو 4 لِوَحده مَرَّةً ثانِيَة مُباشَرَةً
  4. يُقارِن الفَرق
"""
import sys
import time

sys.path.insert(0, "/home/yoga/m13/m13/servo_characterization")
from xqpower import XqpowerBus  # type: ignore

NODE_IDS = (1, 2, 3, 4)


def fb_summary(bus, slot, label=""):
    fb = bus.get_feedback(slot)
    age_ms = (time.monotonic_ns() - fb.timestamp_mono_ns) / 1e6 if fb.timestamp_mono_ns else -1
    return f"{label}fb={fb.position_deg:+7.3f}° age={age_ms:6.0f}ms samples={fb.sample_count} abort={fb.abort_count}"


def test_servo(bus, slot, label="", angle=25.0, settle=2.0):
    """اِختَبِر سيرفو واحِد مَع رَصد شامِل."""
    print(f"\n  {'─'*60}")
    print(f"  {label}")
    print(f"  {'─'*60}")
    sequence = [(0.0, "ZERO   "), (+angle, f"+{angle}°"), (-angle, f"-{angle}°"), (0.0, "ZERO   ")]
    for ang, desc in sequence:
        # قَبل الأَمر
        before = fb_summary(bus, slot, "before: ")
        # أَرسِل
        bus.set_position_deg(slot, ang, angle + 5.0)
        # انتَظِر
        time.sleep(settle)
        # بَعد الأَمر
        after = fb_summary(bus, slot, "after:  ")
        fb = bus.get_feedback(slot)
        err = abs(fb.position_deg - ang)
        ok = "✅" if err < 1.0 else "❌"
        print(f"   cmd={ang:+6.1f}° ({desc}) {ok}")
        print(f"      {before}")
        print(f"      {after}  err={err:.2f}°")


def main():
    print("="*70)
    print("  🔬 تَشخيص: سيرفو 4 — كامِل vs لِوَحدِه")
    print("="*70)

    bus = XqpowerBus(node_ids=NODE_IDS, poll_interval_us=5000)
    bus.open()
    bus.enable_rx_log(True)
    bus.init_all_servos(report_interval_ms=10, settle_s=0.8)
    bus.wait_for_all_online(timeout_s=5.0)
    print("\n✅ All 4 servos online")

    # حالَة ابتِدائيَّة
    print("\n📊 الحالَة الابتِدائيَّة:")
    for s in range(4):
        print(f"  Slot {s} (node 0x{NODE_IDS[s]:02X}): {fb_summary(bus, s)}")

    # ==========  Phase 1: تَشغيل السيرفوهات 1-3 سيكوينس كامِل (محاكاة العِبء) ==========
    print("\n" + "="*70)
    print("  ⏱️  Phase 1: تَشغيل سَريع سيرفوهات 1-3 (مِثل الاختبار الكامِل)")
    print("="*70)
    for s in [0, 1, 2]:
        print(f"\n  → سيرفو {s+1} (slot {s})")
        for ang in [+25.0, -25.0, +25.0, -25.0, 0.0]:
            bus.set_position_deg(s, ang, 30.0)
            time.sleep(2.0)
        print(f"    {fb_summary(bus, s, '    خَتَم: ')}")

    # ==========  Phase 2: حالَة جَميع السيرفوهات قَبل اختبار السيرفو 4 ==========
    print("\n" + "="*70)
    print("  📊 Phase 2: حالَة كلّ السيرفوهات قَبل اختبار السيرفو 4")
    print("="*70)
    for s in range(4):
        print(f"  Slot {s}: {fb_summary(bus, s)}")

    # ==========  Phase 3: اختبار السيرفو 4 (في سياق الاختبار الكامِل) ==========
    print("\n" + "="*70)
    print("  🎯 Phase 3: اختبار السيرفو 4 (slot 3) — في سياق الاختبار الكامِل")
    print("="*70)
    print("  ⚠️  راقِب السيرفو 4 الآن! هَل يَتَحَرَّك ±25°؟")
    test_servo(bus, 3, label="Servo 4 (slot 3) — IN FULL TEST CONTEXT", angle=25.0, settle=3.0)

    # ==========  Phase 4: ضَع كلّ السيرفوهات في 0° ==========
    print("\n" + "="*70)
    print("  Phase 4: تَصفير كلّ السيرفوهات + انتِظار 3s")
    print("="*70)
    for s in range(4):
        bus.set_position_deg(s, 0.0, 30.0)
    time.sleep(3.0)
    for s in range(4):
        print(f"  Slot {s}: {fb_summary(bus, s)}")

    # ==========  Phase 5: اختبار السيرفو 4 لِوَحده (نَفس البَس النَّشِط) ==========
    print("\n" + "="*70)
    print("  🎯 Phase 5: اختبار السيرفو 4 لِوَحده الآن (نَفس البَس)")
    print("="*70)
    print("  ⚠️  راقِب السيرفو 4 الآن! هَل يَتَحَرَّك ±25°؟")
    test_servo(bus, 3, label="Servo 4 (slot 3) — SOLO NOW", angle=25.0, settle=3.0)

    # ==========  Phase 6: مُلَخَّص ==========
    print("\n" + "="*70)
    print("  📊 Phase 6: المُلَخَّص النِّهائي")
    print("="*70)
    print("\nحالَة كلّ السيرفوهات في النِّهايَة:")
    for s in range(4):
        print(f"  Slot {s}: {fb_summary(bus, s)}")

    # تَصفير نِهائي
    for s in range(4):
        bus.set_position_deg(s, 0.0, 30.0)
    time.sleep(1.0)

    bus.close()
    print("\n" + "="*70)
    print("  ✅ تَمّ")
    print("="*70)
    print("\nالأَسئِلَة المُهِمَّة:")
    print("  Q1: هَل تَحَرَّك السيرفو 4 في Phase 3 (سِياق الاختبار الكامِل)؟")
    print("  Q2: هَل تَحَرَّك السيرفو 4 في Phase 5 (لِوَحده)؟")
    print("  Q3: إن كان Q2=نَعَم وQ1=لا → مَشكَلَة طاقَة/CAN عِند تَشغيل مُتَتالي")


if __name__ == "__main__":
    main()
