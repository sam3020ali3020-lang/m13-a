# السلسلة الكاملة: من البناء إلى تشغيل محاكاة HIL

## ما الفرق بين HIL و PIL؟

| | PIL | HIL |
|---|---|---|
| **المحاكاة** | 6DOF على PC | 6DOF على PC |
| **التحكم** | PX4 + rocket_mpc على الجهاز | PX4 + rocket_mpc على الجهاز |
| **السيرفوهات** | ❌ لا يوجد عتاد حقيقي | ✅ سيرفوهات حقيقية على CAN |
| **مصدر الزعانف** | أمر MPC مباشرة | **الزاوية المقاسة من السيرفو** (SRV_FB) |
| **ماذا يكشف؟** | أداء MPC/MHE فقط | أداء MPC + backlash + slew + CAN latency + jitter |

**HIL = Hardware-In-the-Loop**: السيرفوهات الحقيقية تتحرك فعلاً، وزاوية الزعنفة المقاسة
(وليس الأمر) هي ما يدخل حسابات الأيروديناميكا. هذا يكشف كل عيوب العتاد الحقيقي.

---

## نظرة عامة على السلسلة

```
┌─────────────────────────────────────────────────────────────────────┐
│  الخطوة 1: توليد كود Solver (على PC) — إذا تغيّر شيء               │
│  Python → c_generated_code/ (MPC + MHE C sources)                  │
├─────────────────────────────────────────────────────────────────────┤
│  الخطوة 2: Cross-compile لـ ARM64 (على PC)                          │
│  c_generated_code/ → libm130_solvers.a                             │
├─────────────────────────────────────────────────────────────────────┤
│  الخطوة 3: بناء APK (على PC)                                        │
│  PX4 + rocket_mpc + libm130_solvers.a → APK                       │
├─────────────────────────────────────────────────────────────────────┤
│  الخطوة 4: تثبيت APK على الجهاز                                     │
├─────────────────────────────────────────────────────────────────────┤
│  الخطوة 5: توصيل العتاد (سيرفوهات + CAN + تغذية)                   │
├─────────────────────────────────────────────────────────────────────┤
│  الخطوة 6: إعداد أنفاق ADB                                          │
│  adb reverse tcp:4560 + adb forward tcp:5760                       │
├─────────────────────────────────────────────────────────────────────┤
│  الخطوة 7: تشغيل HIL على PC                                         │
│  python3 hil_runner.py                                              │
├─────────────────────────────────────────────────────────────────────┤
│  الخطوة 8: ضغط START في التطبيق (أو تلقائي عبر preflight_reset)    │
│  PX4 يتصل → warm-up → معايرة سيرفو → حلقة الطيران                 │
└─────────────────────────────────────────────────────────────────────┘
```

---

## الخطوات 1–4: البناء والتثبيت (مطابقة لـ PIL)

```bash
# ═══ 1. توليد solver (فقط إذا تغيّر النموذج/الأوزان/القيود) ═══
cd /home/yoga/m13/6DOF_v4_pure
python3 rocket_6dof_sim.py

# ═══ 2. Cross-compile لـ ARM64 ═══
cd /home/yoga/m13
./scripts/build_m130_solvers_arm64.sh

# ═══ 3. بناء APK ═══
cd /home/yoga/m13/AndroidApp
./gradlew assembleDebug

# ═══ 4. تثبيت على الجهاز ═══
adb install -r app/build/outputs/apk/debug/app-debug.apk
```

---

## الخطوة 5: توصيل العتاد

هذه الخطوة خاصة بـ HIL ولا توجد في PIL.

```
┌─────────────────┐
│  هاتف Android   │
│  (S23 Ultra)    │
│                 │
│  PX4 + MPC      │
│  XqpowerCan     │──── USB-C OTG Hub ────┐
│  driver         │                       │
└─────────────────┘                       │
                                    ┌─────┴──────┐
                                    │ Waveshare  │
                                    │ USB_CAN_A  │
                                    │ (CH340)    │
                                    │ VID=0x1A86 │
                                    └─────┬──────┘
                                          │ CAN Bus (500 kbps)
                            ┌─────┬───────┼───────┬─────┐
                            │     │       │       │     │
                          ┌─┴─┐ ┌─┴─┐   ┌─┴─┐ ┌──┴──┐  │
                          │ 1 │ │ 2 │   │ 3 │ │  4  │  │
                          └───┘ └───┘   └───┘ └─────┘  │
                          XQPOWER سيرفوهات (عناوين 1-4) │
                                                        │
                                              ┌─────────┴───┐
                                              │ مصدر تغذية  │
                                              │ 12V/24V     │
                                              │ (E-stop!)   │
                                              └─────────────┘
```

**قائمة المتطلبات:**
- **هاتف Android** مع تطبيق `com.ardophone.px4v17` مُثبّت
- **Waveshare USB_CAN_A** (CH340, VID=0x1A86) عبر USB-C OTG hub
- **4 سيرفوهات XQPOWER** على CAN bus (عناوين 1..4)
- **مصدر تغذية** للسيرفوهات (12V/24V حسب النوع) — **مع زر E-stop!**
- **كابل USB** بين اللابتوب والهاتف (لـ ADB)

---

## الخطوة 6: إعداد أنفاق ADB

```bash
# simulator_mavlink: PC → الجهاز (بيانات HIL_SENSOR/HIL_GPS)
adb reverse tcp:4560 tcp:4560

# mavlink readback: الجهاز → PC (SRV_FB + TIMING + DEBUG_FLOAT_ARRAY)
adb forward tcp:5760 tcp:5760
```

**⚠️ ملاحظة مهمة:** في HIL نستخدم `adb forward` (وليس `adb reverse`) للمنفذ 5760:

| المنفذ | الاتجاه | الأمر | السبب |
|--------|---------|-------|-------|
| 4560 | PC ← الجهاز | `adb reverse` | PX4 يتصل بـ localhost:4560 → يُوجَّه إلى PC |
| 5760 | PC → الجهاز | `adb forward` | PC يتصل بـ localhost:5760 → يُوجَّه إلى الجهاز |

---

## الخطوة 7: تشغيل HIL على الحاسوب

```bash
cd /home/yoga/m13/6DOF_v4_pure/hil

# الوضع الكامل: HIL + تحليل servo/timing
python3 hil_runner.py

# أو: HIL فقط بدون تحليل
python3 hil_runner.py --hil-only

# أو: تحليل CSV موجودة بدون تشغيل HIL
python3 hil_runner.py --compare-only

# أو: الجسر مباشرة (بدون تحليل، بدون preflight)
python3 mavlink_bridge_hil.py
```

### ماذا يحصل عند `python3 hil_runner.py`؟

```
1. preflight_reset() ← تلقائي (إذا preflight_reset: true في الإعدادات)
   ├── adb shell am force-stop com.ardophone.px4v17   ← إيقاف التطبيق
   ├── adb reverse --remove-all                        ← تنظيف أنفاق قديمة
   ├── adb forward --remove-all
   ├── adb reverse tcp:4560 tcp:4560                   ← إنشاء من جديد
   ├── adb forward tcp:5760 tcp:5760
   ├── adb shell am start -n com.ardophone.px4v17/.MainActivity  ← تشغيل التطبيق
   └── انتظار 5 ثوانٍ لتهيئة PX4 modules

2. HILBridge(hil_config.yaml)
   ├── تحميل إعدادات المحاكاة (6dof_config_advanced.yaml)
   ├── ضبط control_type = "none" (التحكم من PX4، ليس Python)
   └── تهيئة محاكاة 6DOF

3. bridge.accept()
   ├── TCP server على 0.0.0.0:4560
   ├── ينتظر اتصال PX4 (حتى 600 ثانية)
   └── يبدأ خيط التقاط SRV_FB + TIMING من TCP 5760

4. bridge.run()
   ├── ★ Warm-up:
   │   ├── يرسل HEARTBEAT + HIL_SENSOR كل خطوة
   │   ├── يرسل أمر ARM كل ثانية
   │   ├── ينتظر أول HIL_ACTUATOR_CONTROLS (حتى 60 ثانية)
   │   └── استقرار EKF (8 ثوانٍ إضافية)
   │
   ├── ★ معايرة السيرفو (servo_auto_zero):
   │   ├── يُرسل أوامر cmd=0 لكل سيرفو
   │   ├── انتظار 4 ثوانٍ للاستقرار
   │   ├── جمع عيّنات 2 ثانية من SRV_FB
   │   ├── حساب متوسط الانحراف الميكانيكي (zero-offset)
   │   └── يُطرح تلقائياً من كل قياس لاحق
   │
   ├── ★ حلقة الطيران (500 ثانية، dt=0.01):
   │   كل خطوة @ 100Hz:
   │   │
   │   │   PC (HIL Bridge)                    Android (PX4)
   │   │   ──────────────                    ──────────────
   │   │   → HIL_SENSOR (حسّاسات)      ──→   EKF2
   │   │   → HIL_GPS (موقع)            ──→   EKF2
   │   │   → HIL_STATE_QUATERNION       ──→   ground truth
   │   │                                      rocket_mpc يحسب التحكم
   │   │   ← HIL_ACTUATOR_CONTROLS     ←──   أوامر الزعانف
   │   │                                      XqpowerCan يُرسل أوامر CAN
   │   │                                      سيرفوهات تتحرك فعلياً
   │   │   ← SRV_FB (DEBUG_FLOAT_ARRAY) ←──  الزاوية المقاسة الحقيقية
   │   │
   │   │   ★ الزاوية المقاسة (SRV_FB) تُحقن في الأيروديناميكا
   │   │     (وليس أمر MPC) ← هذا جوهر HIL
   │   │
   │   └── يحفظ: fin_cmd, fin_act, fin_can, fin_source لكل خطوة
   │
   └── حفظ النتائج في results/

5. compare() ← تحليل تلقائي
   ├── تحليل servo_tracking: MAE و P95 (fin_act vs fin_cmd)
   ├── تحليل timing: MPC/MHE p95/p99/deadline misses
   └── بوابة pass/fail ضد العتبات في hil_config.yaml
```

---

## الخطوة 8: ضغط START (أو تلقائي)

- إذا `preflight_reset: true` (الافتراضي) → **`hil_runner.py` يعيد تشغيل التطبيق تلقائياً** ثم يبدأ PX4.
- إذا `preflight_reset: false` → عليك فتح التطبيق يدوياً وضغط **START**.

عند ضغط START (أو التشغيل التلقائي):
```
التطبيق → FlightService → PX4Bridge.startPX4():
  ├── SYS_AUTOSTART = 22004 (HITL airframe)
  ├── SYS_HITL = 1 (تلقائي)
  ├── تشغيل modules: EKF2, commander, rocket_mpc, XqpowerCan...
  ├── simulator_mavlink يتصل بـ localhost:4560 → ADB → PC
  └── XqpowerCan يبدأ إرسال/استقبال CAN مع السيرفوهات
```

---

## المخرجات

```
hil/results/
├── hil_flight.csv           ← مسار الرحلة الكامل
│   أعمدة مهمة:
│   ├── fin_cmd_1..4         ← أمر MPC (rad)
│   ├── fin_act_1..4         ← الزاوية المقاسة من السيرفو (rad)
│   ├── fin_can_1..4         ← نفس fin_act (مصدر CAN)
│   └── fin_source           ← مصدر الزاوية: can/hold/cmd/abort
│
├── hil_flight_servo.csv     ← سجلّ تفصيلي لكل SRV_FB
│   ├── cmd/fb/err لكل سيرفو
│   ├── online_mask
│   └── tx_fail
│
└── hil_timing.csv           ← توقيت MHE/MPC/Cycle على PX4
    ├── mhe_us
    ├── mpc_us
    └── cycle_us
```

---

## العتبات (pass/fail)

من `hil_config.yaml`:

```yaml
thresholds:
  servo_tracking:
    tracking_mae_max_deg: 1.5    # متوسط خطأ مطلق (deg)
    tracking_p95_max_deg: 3.0    # 95th percentile (deg)
  timing:
    mpc_p95_max_us: 10000        # MPC 95th percentile (μs)
    mpc_p99_max_us: 15000        # MPC 99th percentile (μs)
    cycle_p95_max_us: 15000      # Cycle 95th percentile (μs)
    deadline_miss_max: 0         # عدد خطوات > 20ms (يجب = 0)
    jitter_std_max_us: 2000      # انحراف معياري للتوقيت (μs)
```

---

## ملخص الأوامر كاملة (نسخ/لصق)

```bash
# ═══ على PC ═══

# 1. توليد solver (إذا تغيّر شيء)
cd /home/yoga/m13/6DOF_v4_pure
python3 rocket_6dof_sim.py

# 2. Cross-compile لـ ARM64
cd /home/yoga/m13
ANDROID_NDK_HOME=/home/yoga/Android/Sdk/ndk/27.2.12479018 ./scripts/build_m130_solvers_arm64.sh

# 3. بناء APK
cd /home/yoga/m13/AndroidApp
./gradlew assembleDebug

# 4. تثبيت على الجهاز
adb install -r app/build/outputs/apk/debug/app-debug.apk

# 5. توصيل العتاد: USB_CAN_A + 4 سيرفوهات + تغذية

# 6. أنفاق ADB (إذا preflight_reset=false، وإلا تلقائي)
adb reverse tcp:4560 tcp:4560
adb forward tcp:5760 tcp:5760

# 7. تشغيل HIL (يعيد تشغيل التطبيق تلقائياً ثم ينتظر)
cd /home/yoga/m13/6DOF_v4_pure/hil
python3 hil_runner.py

# ═══ النتائج ═══
# hil/results/hil_flight.csv        ← بيانات الرحلة + زوايا السيرفو
# hil/results/hil_flight_servo.csv  ← تفاصيل فيدباك CAN
# hil/results/hil_timing.csv        ← أزمنة MHE/MPC على ARM64
```

---

## استكشاف الأخطاء

| المشكلة | السبب المحتمل | الحل |
|---------|---------------|------|
| `No target connected within 600s` | PX4 لم يقلع أو ADB غير مفعّل | `adb devices` + تحقق من التطبيق |
| `abort_on_no_actuator` | SYS_HITL ≠ 1 | تحقق من airframe = 22004 |
| `Servo feedback: N=0` | CAN غير متصل | 1) `adb forward tcp:5760` 2) `adb logcat \| grep xqpower` 3) تحقق USB_CAN_A 4) تغذية السيرفوهات |
| `online_mask ≠ 0x0F` | سيرفو واحد أو أكثر offline | تحقق عناوين CAN (1..4) + الأسلاك |
| `tx_fail > 0` | فشل إرسال CAN | تحقق من USB_CAN_A + الأسلاك |
| `servo_zero_max_deg exceeded` | انحراف ميكانيكي كبير | تحقق من تركيب الزعنفة على السيرفو |
| `ABORT: stale feedback` | فيدباك CAN انقطع > 500ms | تحقق من اتصال CAN + تغذية |
| `Failed to bind 0.0.0.0:4560` | المنفذ مستخدم | `lsof -i :4560` ثم `kill` |
| timing CSV فارغ | لا DEBUG_FLOAT_ARRAY | `adb forward tcp:5760 tcp:5760` |
| `Compiler not found: ...android26-clang` | NDK 29 غيّر تسمية المُترجمات | `ANDROID_NDK_HOME=.../ndk/27.2.12479018 ./scripts/build_m130_solvers_arm64.sh` |
| `Command 'python' not found` | النظام يستخدم `python3` | استخدم `python3` أو `sudo apt install python-is-python3` |

---

## نصائح السلامة

1. **دائماً** اجعل مصدر تغذية السيرفوهات قابلاً للفصل السريع (E-stop)
2. **أول مرة:** اختبر السيرفوهات بدون حمل ميكانيكي (بدون زعانف)
3. تحقق من **اتجاه الدوران** عبر `XQCAN_REV` parameter
4. تحقق من **حدود الزوايا** عبر `XQCAN_LIMIT`
5. لا تقف في مسار الزعانف أثناء الاختبار
