#!/usr/bin/env python3
"""🎯 اختِبار نِسبَة بَصَري — بدون مِنقَلَة

الفِكرَة الذَّكيَّة:
  - نُرسِل 10° ثُمَّ 20°
  - إذا الحَرَكَة مِن 0→10 قَريبَة مِن الحَرَكَة مِن 10→20 → linearity عادي
  - نُقارِن الحَرَكَة الكُلّيَّة (20°) مَع عَلامَات مَرجِع مَعروفَة:
      * زاوية ساعَة (12 → 2): 60°
      * زاوية ساعَة (12 → 1): 30°
      * رُبع دائِرَة: 90°

الاستِخدام:  python3 test_ratio_visual.py [slot]
"""
import sys
import time

sys.path.insert(0, "/home/yoga/m13/m13/servo_characterization")
from xqpower import XqpowerBus  # type: ignore

NODE_IDS = (1, 2, 3, 4)


def hold(bus, slot, ang, secs):
    bus.set_position_deg(slot, ang, 30.0)
    for _ in range(int(secs)):
        time.sleep(1.0)
        bus.set_position_deg(slot, ang, 30.0)


def main():
    slot = int(sys.argv[1]) if len(sys.argv) >= 2 else 0
    assert 0 <= slot <= 3

    print("=" * 70)
    print(f"  🎯 اختِبار نِسبَة بَصَري — السيرفو {slot + 1}")
    print("=" * 70)
    print()
    print("⚠️  قَبل البَدء:")
    print("    - الدَّفَّة مَفصولَة عَن السيرفو")
    print("    - ضَع هاتِف فَوق horn السيرفو (على ظَهرِه)")
    print("    - افتَح تَطبيق Bubble Level / Angle Meter")
    print("    - أَو استَخدِم مُقارَنَة بَصَريَّة مَع ساعَة/مُرَبَّع")
    print()
    input("اضغَط Enter للبَدء...")
    print()

    bus = XqpowerBus(node_ids=NODE_IDS, poll_interval_us=5000)
    bus.open()
    bus.enable_rx_log(True)
    bus.init_all_servos(report_interval_ms=10, settle_s=0.8)
    bus.wait_for_all_online(timeout_s=5.0)
    print("✅ السيرفوهات online")
    print()

    # Zero all
    for s in range(4):
        bus.set_position_deg(s, 0.0, 30.0)
    time.sleep(1.5)

    # Sequence: 0 → +30 → 0 → -30 → 0 → +30 (hold long) → 0
    steps = [
        ("🟢 0°       ", 0.0, 5,
         "اضبِط هاتِفك الآن على 0° (calibrate)"),
        ("🔵 +30°     ", +30.0, 10,
         "اقرأ زاوية الهاتِف أَو قارِن بَصَريّاً"),
        ("🟢 0°       ", 0.0, 3,
         "عَودَة للصِّفر"),
        ("🔵 -30°     ", -30.0, 10,
         "اقرأ زاوية الهاتِف في الاتِّجاه المُعاكِس"),
        ("🟢 0°       ", 0.0, 3,
         "عَودَة للصِّفر"),
        ("🔵 +30° طَويل", +30.0, 20,
         "اترُك 20 ثانية لقِياس هادِئ ودَقيق"),
        ("🟢 0°       ", 0.0, 3,
         "إنهاء"),
    ]

    for label, ang, secs, msg in steps:
        print(f"┌─ {label} ───────────────────────────────────────────┐")
        print(f"│  📍 {msg}")
        fb_before = bus.get_feedback(slot)
        hold(bus, slot, ang, secs)
        fb_after = bus.get_feedback(slot)
        print(f"│  feedback: قَبل={fb_before.position_deg:+.2f}°  "
              f"بَعد={fb_after.position_deg:+.2f}°  "
              f"(الفَرق={fb_after.position_deg - fb_before.position_deg:+.2f}°)")
        print(f"└{'─' * 65}┘\n")

    # Safety: zero all
    for s in range(4):
        bus.set_position_deg(s, 0.0, 30.0)
    time.sleep(1.0)
    bus.close()

    print("=" * 70)
    print("  📋 أَخبِرني الآن:")
    print("     عِندَما كان الأَمر +30° (الخَطوَة الطَّويلَة)،")
    print("     كَم كانَت زاوية شَفت السيرفو الَّتي قَرَأَها الهاتِف؟")
    print()
    print("     ≈ 30°  → سيناريو A (لا linkage داخِلي، نَحتاج × 1.8)")
    print("     ≈ 54°  → سيناريو B (linkage مَدمَج، الكود سَليم ✅)")
    print("=" * 70)


if __name__ == "__main__":
    main()
