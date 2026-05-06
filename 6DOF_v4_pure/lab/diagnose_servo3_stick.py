#!/usr/bin/env python3
"""🔬 تَشخيص عُلوق السيرفو 3 في الاتِّجاه المُوجَب

الفِكرَة:
  - زِيادَة تَدريجيَّة 0° → +30° بخَطوَة 2°
  - قِراءَة feedback عِند كلّ خَطوَة
  - تَحديد عِند أيّ زاوية يَعلَق (cmd ≠ fb)
  - مُقارَنَة بالاتِّجاه السالِب 0° → -30°

النَّتائِج المُحتَمَلَة:
  ① fb يُطابِق cmd دائِماً → عُلوق ميكانيكي بَعد السيرفو (لا يَعرِف)
  ② fb يَتَجَمَّد عِند زاوية ما → السيرفو عالِق فِعلاً (encoder ثابِت)
  ③ fb يَختَلِف بـ Δ ثابِت → backlash أَو bias
  ④ كلّ المَوجَب يَفشَل لكِن السالِب OK → خَلَل ميكانيكي اتِّجاهي

الاستِخدام:  python3 diagnose_servo3_stick.py
"""
import sys
import time

sys.path.insert(0, "/home/yoga/m13/m13/servo_characterization")
from xqpower import XqpowerBus  # type: ignore

NODE_IDS = (1, 2, 3, 4)
SLOT = int(sys.argv[1]) if len(sys.argv) >= 2 else 3  # افتِراضي: السيرفو 4
STEP = 2.0
HOLD_S = 1.5
MAX_ANG = 30.0


def sweep(bus, slot, target_max, label):
    """Ramp 0 → target_max بخَطوَة STEP، يُعيد قائِمَة (cmd, fb) لكلّ خَطوَة."""
    sign = 1.0 if target_max > 0 else -1.0
    n_steps = int(abs(target_max) / STEP) + 1

    print(f"\n┌─ {label} ─────────────────────────────────────────────┐")
    print(f"│  {'cmd':>8s}  {'fb':>10s}  {'err':>8s}  {'samples':>9s}  حالَة")
    print(f"├{'─' * 65}┤")

    results = []
    bus.set_position_deg(slot, 0.0, MAX_ANG)
    time.sleep(1.0)

    fb_prev = None
    stuck_count = 0
    for k in range(n_steps):
        cmd = sign * k * STEP
        if abs(cmd) > abs(target_max):
            cmd = target_max
        bus.set_position_deg(slot, cmd, MAX_ANG)
        time.sleep(HOLD_S)
        fb = bus.get_feedback(slot)
        err = fb.position_deg - cmd
        # Detect stuck: fb doesn't change between consecutive steps
        is_stuck = (fb_prev is not None
                    and abs(fb.position_deg - fb_prev) < 0.3
                    and abs(cmd - (cmd - sign * STEP)) > 0.1)
        if is_stuck:
            stuck_count += 1
            mark = "⚠️ STUCK"
        elif abs(err) > 1.0:
            mark = "❌ DEV"
        else:
            mark = "✅"
        print(f"│  {cmd:+8.2f}  {fb.position_deg:+10.3f}  "
              f"{err:+8.3f}  {fb.sample_count:>9d}  {mark}")
        results.append((cmd, fb.position_deg, err, fb.sample_count, is_stuck))
        fb_prev = fb.position_deg

    print(f"└{'─' * 65}┘")
    return results


def analyze(results, label):
    """تَحليل نَتائج المَسح."""
    print(f"\n📊 تَحليل {label}:")
    if not results:
        print("    لا بَيانات")
        return

    max_err = max(abs(r[2]) for r in results)
    stuck_steps = [r for r in results if r[4]]
    big_err = [r for r in results if abs(r[2]) > 1.0]

    print(f"    أَقصى خَطَأ |cmd - fb|: {max_err:.3f}°")
    print(f"    خَطوات عالِقَة: {len(stuck_steps)}")
    print(f"    خَطوات بخَطَأ > 1°: {len(big_err)}")

    if big_err:
        first_bad = big_err[0]
        print(f"    🔴 أَوَّل انحِراف عِند cmd={first_bad[0]:+.1f}° "
              f"(fb={first_bad[1]:+.2f}°, err={first_bad[2]:+.2f}°)")

    if stuck_steps:
        first_stuck = stuck_steps[0]
        idx = results.index(first_stuck)
        prev = results[idx - 1] if idx > 0 else None
        print(f"    🔴 أَوَّل عُلوق عِند cmd={first_stuck[0]:+.1f}°")
        if prev:
            print(f"        الزاوية الَّتي عَلِق فيها: {prev[1]:+.2f}°")


def main():
    print("=" * 70)
    print("  🔬 تَشخيص السيرفو 3 — عُلوق في الاتِّجاه المُوجَب")
    print(f"     (slot={SLOT}, node=0x{NODE_IDS[SLOT]:02X})")
    print("=" * 70)
    print()

    bus = XqpowerBus(node_ids=NODE_IDS, poll_interval_us=5000)
    bus.open()
    bus.enable_rx_log(True)
    bus.init_all_servos(report_interval_ms=10, settle_s=0.8)
    bus.wait_for_all_online(timeout_s=5.0)
    print("✅ السيرفوهات online")

    # Test 1: Positive sweep (where the problem is)
    res_pos = sweep(bus, SLOT, +MAX_ANG, "🔵 مَسح مُوجَب  0° → +30°")

    # Recover
    bus.set_position_deg(SLOT, 0.0, MAX_ANG)
    time.sleep(2.0)

    # Test 2: Negative sweep (control reference)
    res_neg = sweep(bus, SLOT, -MAX_ANG, "🟢 مَسح سالِب  0° → -30°")

    # Recover
    bus.set_position_deg(SLOT, 0.0, MAX_ANG)
    time.sleep(2.0)

    # Test 3: Try to JUMP to +30 directly (large step test)
    print("\n┌─ 🔵 قَفزَة مُباشَرَة 0° → +30° ──────────────────────────────┐")
    bus.set_position_deg(SLOT, 0.0, MAX_ANG)
    time.sleep(1.0)
    fb0 = bus.get_feedback(SLOT)
    bus.set_position_deg(SLOT, +30.0, MAX_ANG)
    time.sleep(0.5)
    fb_05 = bus.get_feedback(SLOT)
    time.sleep(1.0)
    fb_15 = bus.get_feedback(SLOT)
    time.sleep(1.5)
    fb_30 = bus.get_feedback(SLOT)
    print(f"│  t=0.0s  fb={fb0.position_deg:+.2f}°  (قَبل الأَمر)")
    print(f"│  t=0.5s  fb={fb_05.position_deg:+.2f}°  (بَعد 0.5s مِن +30°)")
    print(f"│  t=1.5s  fb={fb_15.position_deg:+.2f}°  (بَعد 1.5s)")
    print(f"│  t=3.0s  fb={fb_30.position_deg:+.2f}°  (بَعد 3.0s)")
    err_jump = abs(fb_30.position_deg - 30.0)
    if err_jump > 1.0:
        print(f"│  ⚠️  لَم يَصِل: خَطَأ {err_jump:+.2f}°")
    else:
        print(f"│  ✅ وَصَل بنَجاح")
    print(f"└{'─' * 65}┘")

    # Recover
    bus.set_position_deg(SLOT, 0.0, MAX_ANG)
    time.sleep(2.0)

    # Final analysis
    print("\n" + "=" * 70)
    print("  📋 التَّحليل النِّهائي")
    print("=" * 70)
    analyze(res_pos, "الاتِّجاه المُوجَب 🔵")
    analyze(res_neg, "الاتِّجاه السالِب 🟢")

    # Diagnosis
    print("\n" + "=" * 70)
    print("  🩺 التَّشخيص")
    print("=" * 70)
    pos_max_err = max(abs(r[2]) for r in res_pos)
    neg_max_err = max(abs(r[2]) for r in res_neg)
    asymmetric = pos_max_err > 2.0 and neg_max_err < 1.0

    if asymmetric:
        print("  🔴 خَلَل اتِّجاهي مُؤَكَّد!")
        print("     السالِب يَعمَل بشَكل صَحيح، لكِن المُوجَب فيه عُلوق.")
        print("     الأَسباب المُحتَمَلَة:")
        print("        ① عائِق ميكانيكي في الاتِّجاه المُوجَب (دَفَّة، كابِل، إلخ)")
        print("        ② backlash كَبير في linkage عِند عَكس الاتِّجاه")
        print("        ③ تَلَف داخِلي في السيرفو (gearbox أَو brushes)")
    elif pos_max_err > 1.0:
        print("  🟡 خَلَل عام في السيرفو (المُوجَب أَسوَأ)")
    else:
        print("  ✅ السيرفو يَعمَل بشَكل عادي في كِلا الاتِّجاهَين")

    # Safety: zero all
    for s in range(4):
        bus.set_position_deg(s, 0.0, MAX_ANG)
    time.sleep(1.0)
    bus.close()


if __name__ == "__main__":
    main()
