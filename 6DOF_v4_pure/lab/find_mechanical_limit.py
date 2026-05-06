#!/usr/bin/env python3
"""🎯 العُثور على الحَدّ الميكانيكي للسيرفو (end-stop)

يَرفَع الأَمر تَدريجيّاً ويَكشِف تَلقائيّاً عِندما يَتَوَقَّف السيرفو:
  - إذا fb لَم يَتَغَيَّر أَكثَر مِن MIN_PROGRESS بَين خَطوَتَين → وَصَل للحَدّ
  - إذا |fb - cmd| > LAG_THRESHOLD → يَتَأَخَّر كَثيراً، نَقِف

السكريبت آمِن:
  - يَرفَع 5° كلّ مَرَّة (ليس فَجأَة)
  - حَدّ أَقصى مُطلَق MAX_ABS_DEG = 90° (لا يَتَجاوَزه أَبَداً)
  - يَعود إلى 0° تَلقائيّاً في النِّهايَة

الاستِخدام:
    python3 find_mechanical_limit.py [slot] [direction]
    slot:       0..3  (افتِراضي 0 = السيرفو 1)
    direction:  pos | neg | both  (افتِراضي both)
"""
import sys
import time

sys.path.insert(0, "/home/yoga/m13/m13/servo_characterization")
from xqpower import XqpowerBus  # type: ignore

NODE_IDS = (1, 2, 3, 4)
STEP_DEG = 5.0          # مِقدار الزِّيادَة لكلّ خَطوَة
DWELL_S = 1.2           # وَقت انتِظار بَعد كلّ خَطوَة
MIN_PROGRESS = 1.5      # إذا fb تَقَدَّم أَقَلّ مِن هذا → نَعتَبِرُه وَقَفَ
LAG_THRESHOLD = 4.0     # إذا cmd - fb > هذا → نَعتَبِرُه وَقَفَ
MAX_ABS_DEG = 90.0      # حَدّ مُطلَق آمِن
MAX_CMD_LIMIT = 95.0    # تَمرير للـ bus.set_position_deg clamp


def sweep_until_stop(bus, slot, direction_sign, dir_name):
    """يَرفَع الأَمر باتِّجاه direction_sign حَتّى يَتَوَقَّف السيرفو."""
    sign = +1.0 if direction_sign > 0 else -1.0
    label = "🔵 المُوجَب" if sign > 0 else "🟢 السالِب"

    print(f"\n┌─ {label} ({dir_name}) ─────────────────────────────────────┐")
    print(f"│  {'cmd':>8s}  {'fb':>10s}  {'Δfb':>8s}  {'lag':>8s}  حالَة")
    print(f"├{'─' * 60}┤")

    # ابدأ مِن 0
    bus.set_position_deg(slot, 0.0, MAX_CMD_LIMIT)
    time.sleep(1.0)

    prev_fb = 0.0
    last_good_cmd = 0.0
    last_good_fb = 0.0
    stuck_reason = None

    k = 1
    while True:
        cmd = sign * k * STEP_DEG
        if abs(cmd) > MAX_ABS_DEG:
            stuck_reason = f"⛔ وَصَل للحَدّ المُطلَق الآمِن ({MAX_ABS_DEG}°)"
            break

        bus.set_position_deg(slot, cmd, MAX_CMD_LIMIT)
        time.sleep(DWELL_S)
        fb = bus.get_feedback(slot)
        fb_deg = fb.position_deg

        dfb = fb_deg - prev_fb
        lag = cmd - fb_deg

        # تَشخيص الحالَة
        stuck_now = False
        mark = "✅"
        if abs(dfb) < MIN_PROGRESS and k > 1:
            stuck_now = True
            mark = "⛔ STUCK (no progress)"
            stuck_reason = f"تَوَقَّف عَن التَّقَدُّم (Δfb={dfb:+.2f}°)"
        elif abs(lag) > LAG_THRESHOLD:
            stuck_now = True
            mark = "⛔ STUCK (large lag)"
            stuck_reason = f"تَأخُّر كَبير (lag={lag:+.2f}°)"

        print(f"│  {cmd:+8.2f}  {fb_deg:+10.3f}  {dfb:+8.3f}  {lag:+8.3f}  {mark}")

        if stuck_now:
            break

        last_good_cmd = cmd
        last_good_fb = fb_deg
        prev_fb = fb_deg
        k += 1

    print(f"└{'─' * 60}┘")
    print(f"\n  🏁 الحَدّ الميكانيكي ({dir_name}):")
    print(f"     آخِر fb ناجِح:   {last_good_fb:+.2f}°")
    print(f"     آخِر cmd ناجِح: {last_good_cmd:+.2f}°")
    print(f"     السَّبَب:         {stuck_reason}")

    # عَودَة للصِّفر بأَمان
    print(f"\n  ⏮️  عَودَة إلى 0°...")
    bus.set_position_deg(slot, 0.0, MAX_CMD_LIMIT)
    time.sleep(2.0)

    return last_good_fb


def main():
    slot = int(sys.argv[1]) if len(sys.argv) >= 2 else 0
    direction = sys.argv[2] if len(sys.argv) >= 3 else "both"
    assert 0 <= slot <= 3
    assert direction in ("pos", "neg", "both")

    print("=" * 65)
    print(f"  🎯 العُثور عَلى الحَدّ الميكانيكي")
    print(f"     السيرفو {slot + 1}  (slot={slot}, node=0x{NODE_IDS[slot]:02X})")
    print(f"     الاتِّجاه: {direction}")
    print(f"     حَدّ أَقصى آمِن: ±{MAX_ABS_DEG}°")
    print("=" * 65)

    bus = XqpowerBus(node_ids=NODE_IDS, poll_interval_us=5000)
    bus.open()
    bus.enable_rx_log(True)
    bus.init_all_servos(report_interval_ms=10, settle_s=0.8)
    bus.wait_for_all_online(timeout_s=5.0)
    print("\n✅ السيرفوهات online")

    results = {}

    if direction in ("pos", "both"):
        results["pos"] = sweep_until_stop(bus, slot, +1.0, "الاتِّجاه المُوجَب")

    if direction in ("neg", "both"):
        results["neg"] = sweep_until_stop(bus, slot, -1.0, "الاتِّجاه السالِب")

    # مُلَخَّص نِهائي
    print("\n" + "=" * 65)
    print(f"  📋 مُلَخَّص السيرفو {slot + 1}")
    print("=" * 65)
    if "pos" in results:
        print(f"     الحَدّ المُوجَب:  {results['pos']:+.2f}°")
    if "neg" in results:
        print(f"     الحَدّ السالِب:  {results['neg']:+.2f}°")
    if "pos" in results and "neg" in results:
        travel = results["pos"] - results["neg"]
        midpt = (results["pos"] + results["neg"]) / 2.0
        print(f"     المَدى الكُلّي: {travel:+.2f}°  (وسيط={midpt:+.2f}°)")
    print("=" * 65)

    # ضَمان الصِّفر لكلّ السيرفوهات
    for s in range(4):
        bus.set_position_deg(s, 0.0, MAX_CMD_LIMIT)
    time.sleep(1.0)
    bus.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠️  تَوَقُّف مِن المُستَخدِم — يَنصَح بإعادة تَصفير السيرفو يَدَوِيّاً!")
        sys.exit(1)
