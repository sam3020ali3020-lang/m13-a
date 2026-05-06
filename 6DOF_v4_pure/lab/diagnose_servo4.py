#!/usr/bin/env python3
"""تَشخيص السيرفو 4 (slot=3, node=0x04).

الاختبار: حَرَكات تَدَرُّجيَّة 0° → 5° → 10° → 15° → 20° → 25°
مع قِراءَة feedback مَرَّتَين لكلّ خَطوَة (قَبل وبَعد الانتِظار)
لِلكَشف عَن: stale data, frozen encoder, mechanical block.
"""
import sys
import time

sys.path.insert(0, "/home/yoga/m13/m13/servo_characterization")
from xqpower import XqpowerBus  # type: ignore

NODE_IDS = (1, 2, 3, 4)
TARGET_SLOT = 3   # السيرفو رقم 4 (node 0x04)


def main():
    print("="*70)
    print(f"  🔬 تَشخيص السيرفو {TARGET_SLOT+1} (slot={TARGET_SLOT}, node=0x{NODE_IDS[TARGET_SLOT]:02X})")
    print("="*70)
    print()

    bus = XqpowerBus(node_ids=NODE_IDS, poll_interval_us=5000)
    bus.open()
    bus.enable_rx_log(True)
    bus.init_all_servos(report_interval_ms=10, settle_s=0.8)
    bus.wait_for_all_online(timeout_s=5.0)
    print("✅ All servos online\n")

    # Initial state of all servos
    print("الحالَة الابتِدائيَّة لِكلّ السيرفوهات:")
    for slot in range(4):
        fb = bus.get_feedback(slot)
        print(f"  Servo {slot+1} (node 0x{NODE_IDS[slot]:02X}): pos={fb.position_deg:+.3f}° samples={fb.sample_count}")
    print()

    # === Phase 1: تَدَرُّج بَطيء على السيرفو 4 فَقَط ===
    print("="*70)
    print(f"  Phase 1: تَدَرُّج بَطيء على السيرفو 4 فَقَط")
    print("="*70)
    print()
    print(f"  {'الزاوِيَة':>8s}  {'fb_قَبل':>10s}  {'fb_بَعد':>10s}  {'تَغَيُّر':>10s}  {'samples':>10s}")
    print(f"  {'─'*8}  {'─'*10}  {'─'*10}  {'─'*10}  {'─'*10}")

    angles = [0.0, 3.0, 6.0, 10.0, 15.0, 20.0, 25.0]
    for ang in angles:
        # Read fb BEFORE sending command
        fb_before = bus.get_feedback(TARGET_SLOT)
        samples_before = fb_before.sample_count

        # Send command
        ok = bus.set_position_deg(TARGET_SLOT, ang, 25.0)

        # Wait 3 seconds (longer to ensure motion completes)
        time.sleep(3.0)

        # Read fb AFTER waiting
        fb_after = bus.get_feedback(TARGET_SLOT)
        samples_after = fb_after.sample_count
        change = fb_after.position_deg - fb_before.position_deg
        new_samples = samples_after - samples_before

        print(f"  {ang:+8.1f}°  {fb_before.position_deg:+10.3f}  {fb_after.position_deg:+10.3f}  {change:+10.3f}  {new_samples:>10d}")

    print()

    # === Phase 2: ذَهاب-إيَاب سَريع (لِفَحص الاستِجابَة الديناميكيَّة) ===
    print("="*70)
    print(f"  Phase 2: ذَهاب-إياب +25° / -25° مع قِراءَة feedback كلّ 0.5s")
    print("="*70)
    print()

    bus.set_position_deg(TARGET_SLOT, 0.0, 25.0)
    time.sleep(2.0)

    for cycle in range(2):
        print(f"\n  دَورَة {cycle+1}:")
        # Send +25
        bus.set_position_deg(TARGET_SLOT, +25.0, 25.0)
        t0 = time.monotonic()
        for _ in range(8):  # 4 seconds of monitoring at 0.5s
            time.sleep(0.5)
            fb = bus.get_feedback(TARGET_SLOT)
            t = time.monotonic() - t0
            print(f"    cmd=+25° t={t:.1f}s → fb={fb.position_deg:+.3f}° samples={fb.sample_count}")

        # Send -25
        bus.set_position_deg(TARGET_SLOT, -25.0, 25.0)
        t0 = time.monotonic()
        for _ in range(8):
            time.sleep(0.5)
            fb = bus.get_feedback(TARGET_SLOT)
            t = time.monotonic() - t0
            print(f"    cmd=-25° t={t:.1f}s → fb={fb.position_deg:+.3f}° samples={fb.sample_count}")

    # === Phase 3: مُقارَنَة مَع السيرفو 1 (الَّذي قُلت أنّه يَعمَل) ===
    print()
    print("="*70)
    print(f"  Phase 3: نَفس الاختبار على السيرفو 1 (slot=0) لِلمُقارَنَة")
    print("="*70)
    print()
    print(f"  {'الزاوِيَة':>8s}  {'fb_قَبل':>10s}  {'fb_بَعد':>10s}  {'تَغَيُّر':>10s}")
    print(f"  {'─'*8}  {'─'*10}  {'─'*10}  {'─'*10}")
    bus.set_position_deg(0, 0.0, 25.0)
    time.sleep(2.0)
    for ang in angles:
        fb_before = bus.get_feedback(0)
        bus.set_position_deg(0, ang, 25.0)
        time.sleep(3.0)
        fb_after = bus.get_feedback(0)
        change = fb_after.position_deg - fb_before.position_deg
        print(f"  {ang:+8.1f}°  {fb_before.position_deg:+10.3f}  {fb_after.position_deg:+10.3f}  {change:+10.3f}")

    # Safety: zero all
    print("\n→ Returning all servos to zero...")
    for slot in range(4):
        bus.set_position_deg(slot, 0.0, 25.0)
    time.sleep(1.5)

    bus.close()
    print()
    print("="*70)
    print("  ✅ Diagnostic complete")
    print("="*70)
    print()
    print("التَّحليل:")
    print("  - إذا كانت feedback تَزيد بنفس قِيمَة command = السيرفو 4 يَعمَل")
    print("  - إذا فب يُساوي cmd بدِقَّة لكنَّك لا تَرى حَرَكَة = encoder open-loop (مَشكَلَة!)")
    print("  - إذا fb يَبقى ثابِت = السيرفو لا يَستَلِم الأَوامِر")
    print("  - قارِن نَتيجَة Phase 1 مَع Phase 3")


if __name__ == "__main__":
    main()
