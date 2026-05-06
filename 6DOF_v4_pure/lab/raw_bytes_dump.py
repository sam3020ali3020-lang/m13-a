#!/usr/bin/env python3
"""🔬 طَباعَة raw bytes مِن السيرفو للتَّأكُّد مِن clipping.

للسيرفو 1 فَقَط. نُراقِب SDO read responses ونَطبَع:
  byte[4] byte[5]  → value  → degrees

إذا byte[5] دائماً = 0 عِند الحَدّ → firmware السيرفو يُقَصقِص عِند 0xFF.
إذا byte[5] يَتَغَيَّر لكِن القِيَم غَريبَة → مُشكِلَة encoder.

الاستِخدام:
    python3 raw_bytes_dump.py
    (ثُمَّ حَرِّك الدَّفَّة 1 يَدَوِيّاً، راقِب byte[4], byte[5])
"""
import sys
import time
import struct

sys.path.insert(0, "/home/yoga/m13/m13/servo_characterization")
from xqpower import XqpowerBus  # type: ignore

NODE_IDS = (1, 2, 3, 4)
NMT_ID = 0x000
SDO_RX_BASE = 0x580


def main():
    print("=" * 75)
    print("  🔬 Raw bytes dump — السيرفو 1")
    print("=" * 75)
    print()
    print("  حَرِّك الدَّفَّة 1 يَدَوِيّاً، راقِب الـ bytes:")
    print("   • إذا byte[5]=0x00 دائماً → firmware يُقَصقِص عِند 255")
    print("   • إذا byte[5]=0xFF (سالِب) → قَد يَكون underflow")
    print("   • إذا byte[5] يَتَزايَد 0→1→2→3 → كلّ شَيء سَليم")
    print()
    input("اضغَط Enter للبَدء...")
    print()

    bus = XqpowerBus(node_ids=NODE_IDS, poll_interval_us=5000)
    bus.open()
    bus.enable_rx_log(True)  # فَعِّل تَسجيل raw frames

    # NMT START
    bus._poll_enabled.clear()
    time.sleep(0.2)
    for node in NODE_IDS:
        bus.can_send(NMT_ID, bytes([0x01, node]))
        time.sleep(0.05)

    # polling لـ fb
    bus._poll_enabled.set()
    time.sleep(1.0)
    bus.wait_for_all_online(timeout_s=5.0)
    print("✅ CAN فَعّال — حَرِّك الدَّفَّة 1 بيَدِك")
    print()

    # follow mode لمَنع servo 1 مِن الإمساك
    for s in range(4):
        bus.set_position_deg(s, 0.0, 60.0)

    print("─" * 75)
    print(f"  {'t':>5s}  {'byte[4]':>8s} {'byte[5]':>8s}  "
          f"{'raw':>8s}  {'deg':>10s}  hex")
    print("─" * 75)

    last_raw = 0
    max_raw = 0
    min_raw = 0
    last_print = 0
    t_start = time.time()

    while True:
        try:
            now = time.time() - t_start

            # follow mode - السيرفو يَتبَع
            fb = bus.get_feedback(0)
            bus.set_position_deg(0, fb.position_deg, 60.0)

            # فَحص raw log (deque thread-safe)
            frames = list(bus.rx_log)[-50:]  # آخِر 50 إطار

            # نَبحَث عَن إطارات من node 0x01
            for f in frames:
                if f.can_id != SDO_RX_BASE + 0x01:
                    continue
                if len(f.data) < 6:
                    continue
                if f.data[0] != 0x4B:  # SDO READ RESPONSE 2-byte
                    continue
                b4 = f.data[4]
                b5 = f.data[5]
                raw = struct.unpack("<h", bytes([b4, b5]))[0]
                deg = raw / 18.0

                if raw != last_raw and now - last_print > 0.15:
                    if raw > max_raw:
                        max_raw = raw
                    if raw < min_raw:
                        min_raw = raw
                    mark = "⚡" if abs(raw - last_raw) > 20 else " "
                    print(f"  {now:>5.1f}  "
                          f"0x{b4:02X}     0x{b5:02X}     "
                          f"{raw:>+8d}  {deg:>+10.3f}°  "
                          f"[{b4:02X} {b5:02X}]  {mark}")
                    last_raw = raw
                    last_print = now

            time.sleep(0.02)
        except KeyboardInterrupt:
            break

    print("─" * 75)
    print()
    print(f"  📋 مُلَخَّص raw values:")
    print(f"     أَقصى raw مُوجَب: {max_raw}  ({max_raw / 18.0:+.2f}°)")
    print(f"     أَقصى raw سالِب: {min_raw}  ({min_raw / 18.0:+.2f}°)")
    print()
    if max_raw == 255:
        print("  🔴 تَأكيد! firmware السيرفو 1 يُقَصقِص عِند raw=255 (0xFF)")
        print("      هذا خَلَل داخِلي في السيرفو — لا عَلاج إلّا بتَبديله")
    elif max_raw >= 500:
        print("  ✅ الـ encoder يَقرَأ قِيَم كَبيرَة — لا clipping")
    else:
        print(f"  ⚠️ encoder وَصَل {max_raw} فَقَط — غَير مُتَوَقَّع")

    bus.close()


if __name__ == "__main__":
    main()
