#!/usr/bin/env python3
"""🔬 اختِبار حاسِم: هَل المُعامِل 18 يَتَضَمَّن linkage 1.8 أَم لا؟

═══════════════════════════════════════════════════════════════════════
                       تَعليمات التَّنفيذ المُهِمَّة
═══════════════════════════════════════════════════════════════════════

⚠️  قَبل التَّشغيل:
    1. افصِل الدَّفَّة عَن horn السيرفو فيزيائيّاً (أَزِل البُرغي/المُرَكَّب)
    2. تأَكَّد أنّ شَفت السيرفو يَدور بحُرِّيَّة بدون أيّ قَيد ميكانيكي
    3. ضَع عَلامَة مَرجِع بشَريط لاصِق على شَفت السيرفو وجِسمه
    4. أَحضِر مِنقَلَة (protractor) أو هاتِف بتَطبيق قِياس زاوية

⚠️  السيناريو سيُرسِل أَوامِر بزاوية 5°, 10°, 15°, 20°
    وسَيَنتَظِر 5 ثَوانٍ بَين كلّ زاوية لتَقيس بصَريّاً.

═══════════════════════════════════════════════════════════════════════
                       جَدوَل القِراءَة الحاسِم
═══════════════════════════════════════════════════════════════════════

  أَمر    │  شَفت = أَمر  │  شَفت = أَمر × 1.8
  ──────  │  ──────────  │  ──────────────────
  +5°     │     5°       │      9°
  +10°    │    10°       │     18°
  +15°    │    15°       │     27°
  +20°    │    20°       │     36°
          │              │
          │  السيناريو A │   السيناريو B
          │  يَلزَم × 1.8  │   الكود سَليم ✅
          │  في PX4      │   لا تَعديل

═══════════════════════════════════════════════════════════════════════

استخدام:  python3 test_linkage_factor.py <slot>
         حَيث slot = 0, 1, 2, 3
"""
import sys
import time

sys.path.insert(0, "/home/yoga/m13/m13/servo_characterization")
from xqpower import XqpowerBus  # type: ignore

NODE_IDS = (1, 2, 3, 4)


def main():
    if len(sys.argv) != 2:
        print("usage: python3 test_linkage_factor.py <slot 0-3>")
        sys.exit(1)
    target_slot = int(sys.argv[1])
    assert 0 <= target_slot <= 3, "slot must be 0-3"

    print("=" * 70)
    print("  🔬 اختِبار linkage factor — سيرفو مَفصول عَن الدَّفَّة")
    print(f"     السيرفو {target_slot + 1} (slot={target_slot}, "
          f"node=0x{NODE_IDS[target_slot]:02X})")
    print("=" * 70)
    print()
    print("⚠️  تأَكَّد أنّ الدَّفَّة مَفصولَة عَن السيرفو قَبل المُتابَعَة!")
    print("⚠️  ضَع عَلامَة مَرجِع على الشَّفت لقِياس الزاوية بَصَريّاً")
    print()
    input("اضغَط Enter للمُتابَعَة عِندَما تَكون جاهِزاً...")
    print()

    bus = XqpowerBus(node_ids=NODE_IDS, poll_interval_us=5000)
    bus.open()
    bus.enable_rx_log(True)
    bus.init_all_servos(report_interval_ms=10, settle_s=0.8)
    bus.wait_for_all_online(timeout_s=5.0)
    print("✅ السيرفوهات online")
    print()

    # Zero all servos first
    for s in range(4):
        bus.set_position_deg(s, 0.0, 30.0)
    time.sleep(1.5)

    fb0 = bus.get_feedback(target_slot)
    print(f"📊 حالَة ابتِدائيَّة: feedback={fb0.position_deg:+.3f}°")
    print()

    # Test angles — increasing magnitude
    test_angles = [5.0, 10.0, 15.0, 20.0]

    print("=" * 70)
    print("  ابدأ القِياس البَصَري الآن. كلّ زاوية ستَدوم 5 ثَوانٍ.")
    print("=" * 70)
    print()
    print(f"{'#':>3s}  {'أَمر':>8s}  {'feedback':>10s}  "
          f"{'إذا A (شَفت)':>14s}  {'إذا B (شَفت)':>14s}")
    print("-" * 70)

    for i, ang in enumerate(test_angles):
        # Send command
        bus.set_position_deg(target_slot, ang, 30.0)

        # Wait for stabilization
        time.sleep(2.0)

        # Read feedback
        fb = bus.get_feedback(target_slot)

        # Predictions
        shaft_if_A = ang          # المُعامِل غَير مَدمَج → الشَّفت يُطابِق الأَمر
        shaft_if_B = ang * 1.8    # المُعامِل مَدمَج → الشَّفت يَدور أَكبَر بـ 1.8×

        print(f"{i + 1:>3d}  {ang:+8.1f}  {fb.position_deg:+10.3f}  "
              f"{shaft_if_A:>13.1f}°  {shaft_if_B:>13.1f}°")
        print(f"     ⏱️  قِس الآن زاوية شَفت السيرفو بصَريّاً وسَجِّلها...")

        time.sleep(3.0)
        print()

    # Return to zero
    print("=" * 70)
    print("  العَودَة إلى 0°")
    print("=" * 70)
    bus.set_position_deg(target_slot, 0.0, 30.0)
    time.sleep(2.0)

    # Safety: zero all
    for s in range(4):
        bus.set_position_deg(s, 0.0, 30.0)
    time.sleep(1.0)
    bus.close()

    print()
    print("=" * 70)
    print("  📋 المَطلوب مِنك الآن:")
    print("=" * 70)
    print("  أَخبِرني بقِراءاتك البَصَريَّة لزاوية الشَّفت في كلّ خَطوَة.")
    print("  إذا كانَت قَريبَة مِن عَمود 'إذا A' → يَلزَم إضافة × 1.8 في PX4")
    print("  إذا كانَت قَريبَة مِن عَمود 'إذا B' → الكود سَليم تَماماً ✅")
    print("=" * 70)


if __name__ == "__main__":
    main()
