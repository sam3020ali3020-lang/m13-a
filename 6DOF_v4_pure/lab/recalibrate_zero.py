#!/usr/bin/env python3
"""🎯 إعادَة تَعايُر zero للسيرفو في مَوقِع جَديد.

السيرفو يُخَزِّن zero internal في الـ flash. لو كان الـ zero الحالي
في طَرَف المَدى المُمكِن، نَنقُلُه إلى مَركَز أَفضَل ليُصبِح المَدى مُتَناظِراً.

الإجراء:
  1. السكريبت يَضَع كلّ السيرفوهات في follow mode (compliant)
  2. أَنت تُحَرِّك السيرفو المَطلوب يَدَوِيّاً إلى الزاوية الجَديدَة المَطلوبَة
     (مَثَلاً السيرفو 1 إلى -20° لزِيادَة المَدى المُوجَب)
  3. تَضغَط Enter — السكريبت يُثَبِّت السيرفو في تِلك النُّقطَة
  4. السكريبت يُرسِل أَمر zero calibrate (OD 0x3009)
  5. السكريبت يُرسِل أَمر save to flash (OD 0x1010)
  6. تَأكيد: cmd=0° يُرجِع السيرفو لِنَفس الزاوية الجَديدَة (يَعني تَمَّ التَّعايُر)

⚠️ التَّعايُر يُحفَظ في flash السيرفو ويَدوم بَعد power cycle.

الاستِخدام:
    python3 recalibrate_zero.py [slot]
    افتِراضي: slot=0 (السيرفو 1)
"""
import sys
import time

sys.path.insert(0, "/home/yoga/m13/m13/servo_characterization")
from xqpower import XqpowerBus  # type: ignore

NODE_IDS = (1, 2, 3, 4)
SDO_TX_BASE = 0x600

# الأَوامِر مِن XqpowerCan.cpp:1502, 1511
ZERO_CALIBRATE_PAYLOAD = bytes([0x22, 0x09, 0x30, 0x00, 0x01, 0x00, 0x00, 0x00])
# "save" ASCII = 0x73 0x61 0x76 0x65
SAVE_PAYLOAD = bytes([0x22, 0x10, 0x10, 0x01, 0x73, 0x61, 0x76, 0x65])


def main():
    slot = int(sys.argv[1]) if len(sys.argv) >= 2 else 0
    assert 0 <= slot <= 3
    node = NODE_IDS[slot]

    print("=" * 70)
    print(f"  🎯 إعادَة تَعايُر zero للسيرفو {slot + 1}")
    print(f"     (slot={slot}, node=0x{node:02X})")
    print("=" * 70)
    print()
    print("  الخُطوات:")
    print("   1. كلّ السيرفوهات في follow mode (يُمكِنك تَحريكها بسُهولَة)")
    print("   2. حَرِّك السيرفو {0} يَدَوِيّاً إلى الزاوية المَطلوبَة"
          .format(slot + 1))
    print("      (مَثَلاً: ادفَعه باتِّجاه السالِب لتَحصُل على مَدى مُوجَب أَكبَر)")
    print("   3. اضغَط Enter عِندَما تَكون في الوَضع الجَديد المَطلوب")
    print("   4. السكريبت يُرسِل أَمر zero calibrate + save to flash")
    print()
    input("اضغَط Enter للبَدء...")
    print()

    bus = XqpowerBus(node_ids=NODE_IDS, poll_interval_us=5000)
    bus.open()

    # NMT START بدون init كامِل (لا نُرسِل أَوامِر cmd ثابِتَة)
    NMT_ID = 0x000
    bus._poll_enabled.clear()
    time.sleep(0.2)
    for n in NODE_IDS:
        bus.can_send(NMT_ID, bytes([0x01, n]))
        time.sleep(0.05)
    bus._poll_enabled.set()
    time.sleep(1.0)

    bus.wait_for_all_online(timeout_s=5.0)
    print("✅ السيرفوهات online — الآن في follow mode")
    print()
    print(f"  حَرِّك السيرفو {slot + 1} يَدَوِيّاً إلى الزاوية الجَديدَة...")
    print()

    print("─" * 60)
    print(f"  {'t':>5s}  {'fb':>10s}    اضغَط Enter عِندَما جاهِز")
    print("─" * 60)

    # نُشغِّل follow mode في خَيط مُنفَصِل
    import threading
    stop_event = threading.Event()
    captured_pos = [0.0]

    def follow_loop():
        while not stop_event.is_set():
            for s in range(4):
                fb = bus.get_feedback(s).position_deg
                bus.set_position_deg(s, fb, 60.0)
            time.sleep(0.02)

    def display_loop():
        while not stop_event.is_set():
            fb = bus.get_feedback(slot).position_deg
            captured_pos[0] = fb
            print(f"\r  ●  fb={fb:>+10.3f}°  (السيرفو {slot+1})  ",
                  end="", flush=True)
            time.sleep(0.2)

    follow_thread = threading.Thread(target=follow_loop, daemon=True)
    display_thread = threading.Thread(target=display_loop, daemon=True)
    follow_thread.start()
    display_thread.start()

    try:
        input()
    except KeyboardInterrupt:
        print("\nأُلغي.")
        stop_event.set()
        bus.close()
        return

    # تَجميد الموَقِع
    locked_pos = captured_pos[0]
    print(f"\n  📌 المَوقِع المَلتَقَط: {locked_pos:+.3f}°")
    print(f"  🔒 تَثبيت السيرفو هُنا...")

    # نُوقِف follow loop ونُثَبِّت السيرفو في المَوقِع
    stop_event.set()
    time.sleep(0.3)
    bus.set_position_deg(slot, locked_pos, 60.0)
    time.sleep(0.5)

    # إرسال أَمر zero calibrate
    print(f"  📤 إرسال zero calibrate إلى node 0x{node:02X}...")
    can_id = SDO_TX_BASE + node
    bus.can_send(can_id, ZERO_CALIBRATE_PAYLOAD)
    time.sleep(0.5)

    # إرسال أَمر save to flash
    print(f"  📤 إرسال save to flash إلى node 0x{node:02X}...")
    bus.can_send(can_id, SAVE_PAYLOAD)
    time.sleep(1.0)

    # قِراءَة fb الجَديد
    print()
    print("  ✅ تَمَّ! التَّحَقُّق مِن التَّعايُر الجَديد:")
    print("─" * 60)
    bus.set_position_deg(slot, 0.0, 60.0)
    time.sleep(2.0)
    fb_zero = bus.get_feedback(slot).position_deg
    print(f"     cmd=0°  →  fb={fb_zero:+.3f}°  "
          f"(يَجِب أن يَكون قَريب مِن 0°)")

    # اختِبار +20°
    print()
    print("  📊 اختِبار: cmd=+20° لرُؤيَة المَدى الجَديد...")
    bus.set_position_deg(slot, 20.0, 60.0)
    time.sleep(2.0)
    fb_20 = bus.get_feedback(slot).position_deg
    err_20 = fb_20 - 20.0
    print(f"     cmd=+20°  →  fb={fb_20:+.3f}°  "
          f"({'✅ وَصَل' if abs(err_20) < 1.5 else '⚠️ لا يَزال مَحدود'})")

    # اختِبار -20°
    print()
    print("  📊 اختِبار: cmd=-20° للسالِب...")
    bus.set_position_deg(slot, -20.0, 60.0)
    time.sleep(2.0)
    fb_neg20 = bus.get_feedback(slot).position_deg
    err_neg20 = fb_neg20 - (-20.0)
    print(f"     cmd=-20°  →  fb={fb_neg20:+.3f}°  "
          f"({'✅ وَصَل' if abs(err_neg20) < 1.5 else '⚠️ لا يَزال مَحدود'})")

    # عَودَة لـ 0
    bus.set_position_deg(slot, 0.0, 60.0)
    time.sleep(1.0)
    bus.close()

    print()
    print("─" * 60)
    if abs(err_20) < 1.5 and abs(err_neg20) < 1.5:
        print("  🎉 التَّعايُر نَجَحَ — السيرفو يَصِل +20° و -20°")
    else:
        print("  ⚠️ بَعض القُيود لا تَزال — قَد تَحتاج تَكرار التَّعايُر")
    print("─" * 60)


if __name__ == "__main__":
    main()
