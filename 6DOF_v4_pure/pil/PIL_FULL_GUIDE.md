# السلسلة الكاملة: من البناء إلى تشغيل محاكاة PIL

## نظرة عامة على السلسلة

```
┌─────────────────────────────────────────────────────────────────────┐
│  الخطوة 1: توليد كود Solver (على PC)                                │
│  Python → c_generated_code/ (MPC + MHE C sources)                  │
├─────────────────────────────────────────────────────────────────────┤
│  الخطوة 2: Cross-compile للـ ARM64 (على PC)                         │
│  c_generated_code/ → libm130_solvers.a                             │
├─────────────────────────────────────────────────────────────────────┤
│  الخطوة 3: بناء تطبيق Android (على PC)                              │
│  PX4 + rocket_mpc + libm130_solvers.a → APK                       │
├─────────────────────────────────────────────────────────────────────┤
│  الخطوة 4: تثبيت APK على الجهاز                                     │
│  adb install → الجهاز                                               │
├─────────────────────────────────────────────────────────────────────┤
│  الخطوة 5: إعداد أنفاق ADB                                          │
│  adb reverse tcp:4560/5760                                         │
├─────────────────────────────────────────────────────────────────────┤
│  الخطوة 6: تشغيل PIL على PC                                         │
│  python pil_runner.py (ينتظر اتصال الجهاز)                         │
├─────────────────────────────────────────────────────────────────────┤
│  الخطوة 7: ضغط START في التطبيق                                     │
│  التطبيق → FlightService → PX4Bridge.startPX4() → يتصل بـ PC      │
└─────────────────────────────────────────────────────────────────────┘
```

---

## الخطوة 1: توليد كود الـ Solver (MPC + MHE)

هذه الخطوة تولّد كود C من تعريف مسألة التحكم الأمثل (OCP) في Python.

**متى تحتاجها؟** فقط إذا غيّرت شيئاً في تعريف المسألة (أوزان، قيود، عدد الخطوات N، النموذج الديناميكي...).

```bash
# تأكد أن acados مُعَدّ
export ACADOS_SOURCE_DIR=/home/yoga/m13/acados-main

# توليد MPC solver (N=200)
# أو لتوليد MHE وحده:
cd /home/yoga/m13/6DOF_v4_pure
python3 rocket_6dof_sim.py
# هذا يستدعي MpcController الذي بدوره يستدعي:
#   mpc/m130_ocp_setup.py → create_m130_ocp()
#   mpc/m130_mhe_ocp_setup.py → create_m130_mhe_solver()
# والنتيجة: c_generated_code/ تُملأ بملفات .c و .h

```

**المخرجات:**
```
c_generated_code/
├── acados_solver_m130_rocket.c/.h      ← MPC solver
├── acados_solver_m130_mhe.c/.h         ← MHE solver
├── acados_sim_solver_m130_rocket.c/.h  ← MPC integrator
├── acados_sim_solver_m130_mhe.c/.h     ← MHE integrator
├── m130_rocket_model/                  ← ODE functions (MPC)
├── m130_mhe_model/                     ← ODE functions (MHE)
└── m130_mhe_cost/                      ← Cost functions (MHE)
```

---

## الخطوة 2: Cross-compile لـ ARM64

هذا يأخذ كود C المولّد ويبنيه كمكتبة ثابتة لمعالج ARM64 (Android).

**المتطلبات:** Android NDK 27+ مُثبّت.

```bash
cd /home/yoga/m13
# NDK 29 غيّر تسمية المُترجمات — نحدد NDK 27 صراحةً (يطابق build.gradle.kts)
ANDROID_NDK_HOME=/home/yoga/Android/Sdk/ndk/27.2.12479018 ./scripts/build_m130_solvers_arm64.sh
```

**ماذا يفعل:**
1. يجد Android NDK تلقائياً (من `ANDROID_NDK_HOME` أو `~/Android/Sdk/ndk/`)
2. يتحقق أن `M130_ROCKET_N = 200` في الهيدرات
3. يترجم 16 ملف C بـ `aarch64-linux-android26-clang`
4. يجمعها في مكتبة ثابتة
5. ينسخ الهيدرات إلى مجلد Android

**المخرجات:**
```
AndroidApp/app/src/main/cpp/acados_arm64/lib/libm130_solvers.a   ← المكتبة
AndroidApp/app/src/main/cpp/m130_mpc/acados_solver_m130_*.h      ← الهيدرات
```

---

## الخطوة 3: بناء تطبيق Android (APK)

```bash
cd /home/yoga/m13/AndroidApp

# بناء debug APK
./gradlew assembleDebug

# أو بناء release APK
./gradlew assembleRelease
```

**ماذا يحصل:**
- Gradle يستدعي CMake لبناء الكود الأصلي (C/C++):
  - PX4 modules (EKF2, commander, rocket_mpc, mhe_estimator...)
  - `libm130_solvers.a` (المكتبة المبنية في الخطوة 2)
  - JNI bridge (`px4_jni.cpp`)
  - MAVLink bridges, sensor reader, servo output...
- ثم يبني كود Kotlin (UI, services)
- النتيجة: `app/build/outputs/apk/debug/app-debug.apk`

**ملاحظة:** البناء الأول يأخذ وقتاً طويلاً (PX4 كاملة). البناءات اللاحقة أسرع بكثير.

---

## الخطوة 4: تثبيت APK على الجهاز

```bash
# وصّل الجهاز بـ USB وتأكد من ظهوره
~/Android/Sdk/platform-tools/adb devices

# تثبيت (يستبدل النسخة القديمة)
~/Android/Sdk/platform-tools/adb install -r /home/yoga/m13/AndroidApp/app/build/outputs/apk/debug/app-debug.apk

# أو اختصار Gradle:
cd /home/yoga/m13/AndroidApp
./gradlew installDebug
```

---

## الخطوة 5: إعداد أنفاق ADB

هذا يربط منافذ الجهاز بمنافذ الحاسوب عبر USB:

```bash
# المنفذ الرئيسي: simulator_mavlink (بيانات HIL_SENSOR/HIL_GPS/HIL_ACTUATOR_CONTROLS)
~/Android/Sdk/platform-tools/adb reverse tcp:4560 tcp:4560

# المنفذ الثانوي: mavlink_tcp_bridge (لالتقاط بيانات التوقيت DEBUG_FLOAT_ARRAY)
~/Android/Sdk/platform-tools/adb reverse tcp:5760 tcp:5760
```

**التوضيح:**
```
Android (PX4)                    USB/ADB                     PC (PIL)
─────────────                    ───────                     ────────
connect localhost:4560  ──→  adb reverse  ──→  TCP server 0.0.0.0:4560
connect localhost:5760  ──→  adb reverse  ──→  TCP client 127.0.0.1:5760
```

---

## الخطوة 6: تشغيل PIL على الحاسوب

```bash
cd /home/yoga/m13/6DOF_v4_pure/pil
python3 pil_runner.py
```

**ماذا يحصل:**
```
[PIL] Listening on TCP 0.0.0.0:4560 — start PX4 on target device (timeout=600s)...
```
PIL الآن **ينتظر** اتصال PX4 من الجهاز. لا تغلق هذه النافذة.

---

## الخطوة 7: ضغط START في التطبيق

1. **افتح التطبيق** على الجهاز (com.ardophone.px4v17)
2. **اضغط زر START**

**ماذا يحصل بالتسلسل داخل التطبيق:**

```
[زر START]
    │
    ▼
MainActivity → Intent(ACTION_START) → FlightService
    │
    ▼
FlightService.onStartCommand():
    ├── startForeground()              ← إشعار "PX4 is running"
    ├── FlightIsolationManager.engageFull()  ← WakeLock + WifiLock + DND
    ├── PX4Bridge.startPX4(storagePath)      ← ★ هنا يبدأ PX4
    │       │
    │       ▼
    │   px4_jni.cpp: startPX4()
    │       ├── native_sensor_start()        ← قراءة حساسات الهاتف
    │       └── start_px4_modules():
    │           ├── param_load("./eeprom/parameters")
    │           ├── SYS_AUTOSTART = 22004 (HITL)
    │           ├── SYS_HITL = 1 (تلقائي من airframe 22004)
    │           ├── CAL_ACC0_ID = 1310988 (sim sensor)
    │           ├── بقية معاملات EKF2 للصاروخ
    │           └── تشغيل modules: EKF2, commander, rocket_mpc...
    │
    ├── sensorReader.start()           ← نشر حساسات الهاتف على uORB
    └── usbServoManager.start()        ← التحكم بالسيرفو USB
```

**بعد ضغط START:**
- PX4 يقلع ويضبط `SYS_HITL=1`
- `simulator_mavlink` module يحاول الاتصال بـ `localhost:4560`
- ADB يوجّهه إلى حاسوبك حيث PIL ينتظر
- PIL يكتشف الاتصال: `[PIL] Target connected from ...`

**ثم تبدأ المحاكاة:**
```
PC (PIL Bridge)                              Android (PX4)
───────────────                              ─────────────
→ HIL_SENSOR (accel, gyro, mag, baro)  ──→   EKF2 يعالج البيانات
→ HIL_GPS (lat, lon, alt, vel)         ──→   EKF2 يدمج GPS
                                             rocket_mpc يحسب التحكم
← HIL_ACTUATOR_CONTROLS (fin commands) ←──   أوامر الزعانف الأربع
  يطبّق على محاكاة 6DOF
  يحسب الفيزياء الجديدة
  يكرر @ 100 Hz...
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
~/Android/Sdk/platform-tools/adb install -r app/build/outputs/apk/debug/app-debug.apk

# 5. أنفاق ADB
~/Android/Sdk/platform-tools/adb reverse tcp:4560 tcp:4560
~/Android/Sdk/platform-tools/adb reverse tcp:5760 tcp:5760

# 6. تشغيل PIL (ينتظر اتصال الجهاز)
cd /home/yoga/m13/6DOF_v4_pure/pil
python3 pil_runner.py

# ═══ على الجهاز ═══
# 7. افتح التطبيق واضغط START
#    → PX4 يتصل بـ PIL تلقائياً
#    → المحاكاة تبدأ

# ═══ النتائج ═══
# pil/results/pil_flight.csv   ← بيانات الرحلة
# pil/results/pil_timing.csv   ← أزمنة MHE/MPC على ARM64
```

---

## استكشاف الأخطاء

| المشكلة | السبب المحتمل | الحل |
|---------|---------------|------|
| `No target connected within 600s` | PX4 لم يقلع أو ADB reverse غير مفعّل | تحقق من `adb reverse` وأن التطبيق يعمل |
| `abort_on_no_actuator` | PX4 ليس في HITL mode | تحقق أن airframe = 22004 (يضبط SYS_HITL=1 تلقائياً) |
| `M130_ROCKET_N is not 200` | solver مولّد بـ N مختلف | أعد توليد solver (الخطوة 1) |
| `Failed to bind 0.0.0.0:4560` | منفذ مستخدم من عملية سابقة | `lsof -i :4560` ثم `kill` العملية |
| timing CSV فارغ | لا بيانات DEBUG_FLOAT_ARRAY | تحقق من `adb reverse tcp:5760 tcp:5760` |
| `Compiler not found: ...android26-clang` | NDK 29 غيّر تسمية المُترجمات | `ANDROID_NDK_HOME=.../ndk/27.2.12479018 ./scripts/build_m130_solvers_arm64.sh` |
| `Command 'python' not found` | النظام يستخدم `python3` | استخدم `python3` أو `sudo apt install python-is-python3` |
