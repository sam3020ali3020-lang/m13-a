# /direct — PC ↔ CAN ↔ XQPOWER servos

اختبار سيرفو مباشر من PC: **لا هاتف، لا محاكاة، لا MPC**. يُنتج baseline نقي
لخصائص السيرفو (transport delay / τ / overshoot / slew / bandwidth /
backlash) يُقارَن لاحقاً بنتائج `/bench` و `/hil` لفصل overhead الطبقات الأعلى.

```
PC (python) ──USB──▶ CAN adapter ──CAN──▶ servos (×4)
            ◀──────────────────────────◀
```

---

## المتطلبات

- **Linux** مع SocketCAN (الأفضل) أو USB-Serial.
- **USB↔CAN adapter**:
  - CANable / PCAN-USB / MCP2515 / Peak / KVaser → `backend: socketcan`
  - Lawicel / CANtact / Innomaker → `backend: slcan`
  - Waveshare USB_CAN_A (CH340) → `backend: serial` (متوافق مع العتاد الموجود)
  - للاختبار بلا عتاد → `backend: virtual`
- Python 3.9+ والحزم في `requirements.txt`.

## تثبيت

```bash
pip install -r requirements.txt

# (أ) إذا backend=socketcan:
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# (ب) إذا backend=virtual (لا عتاد):
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

## التشغيل السريع

```bash
# من داخل /direct:
python3 direct_runner.py                              # step response
python3 direct_runner.py --pattern freq_sweep         # Bode
python3 direct_runner.py --pattern ramp               # slew-rate
python3 direct_runner.py --pattern backlash           # hysteresis
python3 direct_runner.py --pattern replay             # إعادة ملف HIL/PIL CSV

# ═══════════ Tier-4 — fault detection / pre-flight ═══════════
python3 direct_runner.py --pattern preflight_check    # 🛡️ GO/NO-GO قبل الطيران
python3 direct_runner.py --pattern wiring_audit       # 🔌 كَشف خَلط أَسلاك
python3 direct_runner.py --pattern fault_scan         # ⚠️ راصِد انوماليّات
```

## 🚀 Pre-launch checklist (موصى به قبل كل إطلاق)

```bash
# 1. تَأكَّد أَنّ الأَسلاك سَليمَة (لا cross-wiring)
python3 direct_runner.py --pattern wiring_audit
#    → ابحث عن "WIRING OK" في metrics.txt

# 2. شغِّل GO/NO-GO check
python3 direct_runner.py --pattern preflight_check
#    → ابحث عن "PREFLIGHT VERDICT: GO" في metrics.txt

# 3. (اختياري) راقِب انوماليّات لمدة 30s
python3 direct_runner.py --pattern fault_scan
#    → ابحث عن "NO FAULTS DETECTED" في metrics.txt
```

أيّ FAIL في أيٍّ من هذه = **STOP — لا تُطلِق**.

المخرجات تُحفظ في `results/`:
- `direct_<pattern>_<timestamp>.csv` — الخام (t_s, servo_idx, node_id, cmd_deg, fb_deg)
- `*.metrics.txt` — ملخّص رقمي
- `*.plot.html` — رسم Plotly تفاعلي

## ماذا يقيس كل نمط؟

| النمط          | يقيس                                          |
|----------------|-----------------------------------------------|
| `step`         | transport delay, τ, overshoot, settling time |
| `freq_sweep`   | Bode plot، bandwidth (-3dB)، phase margin    |
| `ramp`         | slew-rate max، saturation، tracking error    |
| `backlash`     | hysteresis (deadband ميكانيكي)                |
| `replay`       | استجابة لمسار طيران حقيقي من HIL/PIL/SITL    |
| `linearity`    | cmd↔fb regression، dead-band واسع            |
| `hold_drift`   | drift على مَواقع ثابِتَة (slope °/s)           |
| `end_stop`     | سُلوك السيرفو عِند حَدٍّ مَعروف (validation)     |
| `dead_band`    | أَصغَر أَمر يُسَبِّب استِجابَة (~encoder bin)     |
| `stiction`     | breakaway lag عِند ramp بَطيء                 |
| `staircase`    | stalls مُتَقَطِّعَة في خَطَوات تَراكُميَّة 1°      |
| `mech_limits`  | **⭐ اكتِشاف الحَدّ الفِعلي + offset + asymmetry** |
| `preflight_check` | **🛡️ GO/NO-GO شامِل قَبل الطيران** (7 مَراحِل) |
| `wiring_audit` | **🔌 كَشف خَلط أَسلاك** عَبر بَصمَة تَرَدُّديَّة فَريدَة |
| `fault_scan`   | **⚠️ راصِد انوماليّات**: gaps/jumps/sat/sign/OS  |

## الهيكل

```
/direct/
  ├── README.md                 ← هذا الملف
  ├── direct_config.yaml        ← backend / pattern / safety / loop
  ├── requirements.txt
  ├── can_driver.py             ← abstraction + Waveshare serial backend
  ├── xqpower_protocol.py       ← SDO/PDO encoding/decoding
  ├── patterns/
  │   ├── __init__.py           ← PatternSpec + factory
  │   ├── step.py
  │   ├── freq_sweep.py         ← chirp (linear/log)
  │   ├── ramp.py
  │   ├── backlash.py
  │   └── replay.py             ← من CSV سابق
  ├── direct_runner.py          ← Main entry
  ├── direct_analysis.py        ← fits + plots (قابل للاستدعاء مستقل)
  └── results/                  ← CSV + metrics + HTML
```

## البروتوكول

يستخدم نفس بروتوكول `XqpowerCan.cpp` على PX4:

- Bitrate: **500 kbps**
- SDO TX: `0x600 + node_id` (master → servo)
- Feedback: `0x580 + node_id` (auto-report position) و/أو `0x180 + node_id`
- **18 raw units per degree** (`int16` little-endian)
- NMT Start (`0x000 [0x01, node_id]`) لإخراج السيرفو من Pre-Op

للتفاصيل: `xqpower_protocol.py`.

## السلامة

- `angle_limit_deg` يحدّ كل أوامر pattern (افتراضي ±10°).
- `max_angle_abs_deg` = حد صلب يُرفض تجاوزه في الـ config.
- `zero_before_s` / `zero_after_s`: استقرار على 0° قبل/بعد كل نمط.
- **`SIGINT` يُرسل 0° قبل الخروج** (`zero_on_exit: true`).
- `try/finally` يضمن أمر 0° نهائي حتى عند الاستثناءات.

## مقارنة مع باقي الاختبارات

```
/direct   — PC → servo                    baseline نقي للسيرفو
/bench    — PC → Phone(PX4) → servo       يضيف phone overhead
/hil      — /bench + ديناميكا PC          يضيف closed-loop coupling
/pil      — PC sim + Phone(MPC) + sim srv يعزل MPC perf
/sitl     — كل شيء software                اختبار الخوارزمية
```

الفوارق:
```
bench.delay - direct.delay = phone_overhead_ms
hil.delay   - bench.delay  = dynamics_coupling_ms
```

## استخدام `replay`

لإعادة نفس أوامر طيران سابق على البنش:

```yaml
pattern:
  name: replay
  servos: [0]
  replay:
    csv_path: "../hil/results/flight_YYYYMMDD_HHMMSS.csv"
    cmd_column: fin_cmd_1   # rad — يُحوَّل تلقائياً إلى deg
    duration_cap_s: 20.0
```

## مشاكل شائعة

| المشكلة | الحل |
|---|---|
| `OSError: [Errno 19] No such device` | `sudo ip link set can0 up type can bitrate 500000` |
| `CAN TX failed` | افحص أن البص ليس busy off: `ip -det link show can0` |
| `fb_count=0` في warm-up | السيرفو في Pre-Op؛ تأكد من NMT start أو كوابل CAN |
| overshoot كبير جداً | قلّل `angle_limit_deg` أو زد `report_interval_ms` |
| timestamps غير دقيقة | استخدم `socketcan` بدل serial (الجيتّر أقل 10×) |
