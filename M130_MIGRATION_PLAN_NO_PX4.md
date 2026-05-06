# خطة هجرة M130: نقل MPC إلى التطبيق بدون PX4

> **الإصدار:** 1.0 — 2026-04-23
> **المؤلف:** تحليل تقني للريبو `shebah2091-cell/m13`
> **الهدف:** استبدال طبقة PX4-Autopilot بغلاف C++ مستقل يستضيف MPC+MHE ويتحدث مباشرة إلى الحساسات والسيرفوهات.
> **ملاحظة:** هذه خطة تصميمية نصية بالكامل — لا كود.

---

## 0. الملخص التنفيذي

النظام الحالي يحمل **21,668 ملف PX4** لتشغيل **4,240 سطر** من منطق MPC/MHE/Guidance الفعلي. بقية الـ PX4 (uORB, EKF2, commander, navigator, WorkQueue, MAVLink, mc/fw controllers, …) إما غير مستخدم، أو مستخدم فقط كـ "أنابيب" تنقل البيانات. الخطة تستبدل هذه الأنابيب باستدعاءات دوال مباشرة وخيوط POSIX عادية.

**النتيجة المتوقعة:**
- APK أصغر بمرات.
- زمن بناء: دقائق بدل ساعات.
- إزالة 5 تصحيحات خاصة بـ Android داخل PX4.
- لا حاجة لمولّدات Python (uORB/MAVLink/params) عند البناء.
- debug أبسط بكثير (لا uORB indirection، لا `SCHED_FIFO`/`SCHED_OTHER` hacks).

**التكلفة:** ~2–3 أسابيع عمل مركّز + أسبوع اختبارات HIL.

---

## 1. البنية الحالية — ما يوجد اليوم

### 1.1 الطبقات
```
Kotlin UI  ──JNI──▶  PX4 modules (21,668 ملف)  ──▶  acados solvers  + USB/CAN drivers
```
- `PX4Bridge.kt` ← 15 دالة JNI فقط (start/stop, getRoll/Pitch/Yaw/Altitude, setXxxFd).
- `apps.cpp` ← يسجل 21 موديول PX4 + 8 أوامر نظام.
- الموديول الأساسي: `rocket_mpc` (يحل محل `rocket_gnc` القديم).

### 1.2 ما يعتمد عليه `rocket_mpc` من PX4

| فئة | المحتوى | ملاحظات |
|-----|---------|---------|
| **uORB subscriptions** | `sensor_combined`, `vehicle_attitude`, `vehicle_local_position`, `vehicle_air_data`, `sensor_gps`, `vehicle_status`, `parameter_update`, `debug_array` | ناقل رسائل داخلي — قابل للاستبدال بـ struct + mutex |
| **uORB publications** | `actuator_outputs_sim`, `actuator_servos`, `rocket_gnc_status` | نفس الشيء |
| **PX4 params** | 20 `PARAM_DEFINE_FLOAT`/`INT32` في `rocket_mpc_params.c` | قابل للاستبدال بـ JSON/SharedPreferences |
| **WorkQueue scheduler** | `ScheduledWorkItem`, `ModuleBase` | قابل للاستبدال بـ `std::thread` + `timerfd` |
| **hrt_absolute_time()** | timestamp عالي الدقة | `clock_gettime(CLOCK_MONOTONIC)` يكفي |
| **mathlib/geo** | بعض دوال المتجهات والإسقاط الجغرافي | Eigen أو بدائل مدمجة |

### 1.3 ما يعتمد عليه `xqpower_can` (درايفر السيرفو) من PX4
- `ModuleBase`, `ModuleParams`, `ScheduledWorkItem` ← نفس الاستبدال أعلاه.
- `uORB::Subscription` لـ `actuator_servos`, `actuator_outputs_sim`, `actuator_outputs`, `actuator_armed`.
- `uORB::Publication` لـ `debug_array` (تغذية راجعة SRV_FB).
- **منطق SLCAN/CAN نفسه مستقل عن PX4** — هذا ما سنحتفظ به.

### 1.4 القطع التي لا تعتمد على PX4 أصلاً
- **acados solvers**: `acados_solver_m130_rocket.{h,c}` و `acados_solver_m130_mhe.{h,c}` (C خالص، مولّدة من CasADi).
- **منطق التحكم**: `mpc_controller.cpp` (648 سطر)، `mhe_estimator.cpp` (587 سطر)، `los_guidance.cpp` (149 سطر).
- **قراءة الحساسات**: `native_sensor_reader.cpp`, `gps_usb_ubx.cpp` ← مكتوبان أصلاً خارج PX4.
- **جسور USB**: `mavlink_pty_usb_bridge.cpp`, `mavlink_tcp_bridge.cpp` ← مستقلة.

---

## 2. البنية الهدف — ما سنبنيه

### 2.1 الطبقات الجديدة
```
┌─ Android UI (Kotlin/Compose) ─────────────────────────┐
│  ViewModel ↔ M130Bridge (JNI) ↔ libm130.so            │
└───────────────────────────────┬───────────────────────┘
                                │
┌───────────────────────────────▼────────────── libm130.so ─┐
│                                                            │
│  ┌────────────────────────────────────────────────────┐   │
│  │  FlightManager (الخيط الرئيسي، state machine)      │   │
│  │  ├─ IDLE → ARMED → PRELAUNCH → FLIGHT → TERMINATED │   │
│  │  └─ يحمل SharedState (mutex-guarded)                │   │
│  └────────┬─────────────────────────────────┬─────────┘   │
│           │                                  │             │
│  ┌────────▼──────────┐            ┌─────────▼──────────┐  │
│  │ SensorThread       │            │ ControlThread      │  │
│  │ @ 200 Hz           │            │ @ 100 Hz (MPC rate)│  │
│  │ ├─ phone IMU        │            │ ├─ MHE.update()   │  │
│  │ ├─ baro/mag         │            │ ├─ MPC.solve()    │  │
│  │ └─ GPS (gps_usb_ubx)│            │ └─ mixer → fins   │  │
│  └────────┬──────────┘            └─────────┬──────────┘  │
│           │                                  │             │
│           └──────── SharedState ◄────────────┘             │
│                                                            │
│  ┌────────────────────────────────────────────────────┐   │
│  │  ActuatorThread @ 50 Hz                             │   │
│  │  └─ CanSender (SLCAN over USB) → XQPOWER servos    │   │
│  └────────────────────────────────────────────────────┘   │
│                                                            │
│  ┌────────────────────────────────────────────────────┐   │
│  │  Params (JSON on FS)  •  Logger (CSV/binary)        │   │
│  └────────────────────────────────────────────────────┘   │
│                                                            │
│  (اختياري) MavlinkServer ← mavlink c_library_v2 + UDP    │
└────────────────────────────────────────────────────────────┘
```

### 2.2 مبادئ التصميم
1. **لا ناقل رسائل داخلي**: الاتصال بين الخيوط عبر `SharedState` واحدة محمية بـ `std::mutex` + `std::atomic`. التحديثات عبر `snapshot` → نسخ → إفراج.
2. **خيط واحد لكل مهمة**: Sensor/Control/Actuator خيوط مستقلة؛ لا WorkQueue، لا scheduling hacks.
3. **واجهة JNI ضيقة**: ~20 دالة (start, stop, arm, disarm, setParam, getParam, getTelemetry, setUsbFd × 4).
4. **لا أفكار جديدة في الخوارزمية**: كل ما نفعله هو **حذف PX4**؛ منطق MPC/MHE/Guidance/Mixer يُنقل كما هو.
5. **اختبار قبل نقل CAN**: نثبت MPC+MHE يعملان في HIL قبل لمس درايفر السيرفو.

---

## 3. مراحل الهجرة

### المرحلة 0 — تحضير (يومان)
- إنشاء فرع `devin/migration-no-px4` من الفرع الحالي.
- إنشاء مجلد `AndroidApp/app/src/main/cpp/m130_core/` (المكتبة الجديدة).
- توثيق الوضع الأساسي: حفظ snapshot لنتائج HIL الحالية (trajectory, miss distance, solve time) كمرجع للمقارنة.
- تجهيز test harness في Python يُغذّي MPC/MHE ببيانات مُسجّلة من `m130_mhe_ocp.json` ويقارن المخرجات.
- **معيار النجاح**: تشغيل HIL الحالية (PX4 + acados) وتسجيل مرجعية.

### المرحلة 1 — فصل acados عن PX4 (3–5 أيام)
**الهدف:** استخراج منطق MPC/MHE/Guidance/Mixer إلى مكتبة C++ مستقلة، قابلة للاختبار بمعزل عن PX4.

**الملفات المصدر (ننقلها كما هي مع تعديلات دنيا):**
- `mpc_controller.{cpp,h}`
- `mhe_estimator.{cpp,h}`
- `los_guidance.{cpp,h}`
- `sensor_bridge.{cpp,h}` ← نحذف `#include <uORB/…>` ونُبقي منطق التحويلات (NED, GPS projection, timestamp validation).

**ما يُحذف من كل ملف:**
- كل `#include <uORB/…>` و `#include <px4_platform_common/…>`.
- `hrt_absolute_time()` → wrapper جديد حول `clock_gettime`.
- `PX4_INFO/PX4_WARN/PX4_ERR` → `android_log_print` أو callback قابل للحقن.
- `ModuleParams` → قراءة من struct بسيط (`Params params`).

**ما نضيف:**
- `MpcCore` class: واجهة `update(const SensorSnapshot&, const Params&) → ControlOutput`.
- `MheCore` class: نفس النمط.
- `GeoUtils` namespace: يستبدل `lib/geo` بدوال محدودة (`ned_from_wgs84`, `wgs84_from_ned`).
- `TimeUtils::now_us()` يستبدل `hrt_absolute_time`.

**ملفات CMake:**
- `m130_core/CMakeLists.txt` جديد يُولّد `libm130_core.a` مستقلاً.
- لا يربط بـ PX4 أبداً.

**اختبار هذه المرحلة:**
- Unit tests على Linux desktop (خارج Android) تُغذّي MHE ببيانات مسجلة وتتأكد أن الحالة المقدّرة متطابقة ± ε مع التسجيل المرجعي.
- Unit tests على MPC مع معادلات النموذج الثابتة (steady-state).

**معيار النجاح:**
- `libm130_core.a` يُبنى دون أي include من PX4.
- الاختبارات أعلاه تمر.
- نتائج MHE/MPC متطابقة مع المرجع بفارق دون 10⁻⁶.

---

### المرحلة 2 — قارئات الحساسات (2–3 أيام)
**الهدف:** تجميع مصادر البيانات خارج PX4 في واجهة موحّدة.

**المصادر الموجودة فعلاً:**
- `native_sensor_reader.cpp` ← IMU/Baro/Mag الهاتف عبر Android `Sensor` API.
- `gps_usb_ubx.cpp` ← UBX u-blox USB مباشرة.

**ما يُضاف:**
- `SensorHub` class يستقبل تحديثات من كلا المصدرين ويبني `SensorSnapshot` موحّدة كل 5 ms.
- منطق "staleness" (الحالي داخل `sensor_bridge.h`: `GPS_STALE_TIMEOUT_US=500ms`, `BARO_STALE_TIMEOUT_US=500ms`) يُنقل كما هو.
- إسقاط GPS → NED: منطق `MapProjection` من PX4 يُبسَّط بدالة azimuthal equidistant ~30 سطر.

**ما يُحذف:**
- `vehicle_attitude`, `vehicle_local_position`, `vehicle_air_data` ← هذه كانت مخرجات EKF2 داخل PX4. **MHE يحل محل EKF2**، لذا لا نحتاجها.
- `sensor_combined` ← نستبدله بقراءة مباشرة.

**نقطة حرجة:** هل كان MHE يُستخدم مع EKF2 في وضع "cooperative blend"؟ من `sensor_bridge.h` نرى `update_from_lpos` تُدمج EKF2 مع MHE. **القرار:** إن كان الـ blend حيوياً، نُبقي EKF2 مؤقتاً كمكتبة مستقلة من `ecl/EKF`. وإن كان تزيينياً، نحذفه ونعتمد على MHE فقط (الأولوية المقترحة).

**معيار النجاح:**
- `SensorHub` يُنتج snapshot بمعدل ثابت 200 Hz خلال 10 دقائق بدون drift في timestamp.
- GPS outage (فصل USB) يُعلَم في `.gps_fresh=false` خلال 500 ms.

---

### المرحلة 3 — FlightManager + State Machine (3 أيام)
**الهدف:** استبدال `commander` و `navigator` بآلة حالة بسيطة.

**الحالات:**
- `IDLE` ← بعد التشغيل، لا شيء نشط.
- `INIT` ← تحميل params، فتح USB، init acados solvers.
- `ARMED` ← الحساسات تعمل، MHE يعمل، MPC لا يُخرج أوامر سيرفو (zero output).
- `PRELAUNCH` ← نفس ARMED + monitoring لـ launch detection (IMU acceleration threshold).
- `FLIGHT` ← MPC نشط، أوامر السيرفو تذهب إلى CAN.
- `TERMINATED` ← burnout أو apogee أو impact detection؛ الزعانف تتجمد في موضع آمن.

**مصدر كل حالة:**
- الحالي `RocketGNC.cpp` يحتوي منطق launch detection ومراحل الطيران. نستخرجه كما هو.
- `commander` PX4 كان يُزوّد `vehicle_status.arming_state` ← نستبدله بـ getter/setter داخل FlightManager.

**JNI API:**
- `arm()`, `disarm()`, `terminate()`, `getFlightPhase()`, `getTelemetry()`.

**معيار النجاح:**
- انتقال الحالات صحيح مع unit tests.
- `arm()` من Kotlin UI يُفعِّل الخيوط خلال 50 ms.

---

### المرحلة 4 — نظام Params (يوم واحد)
**الهدف:** استبدال PX4 param system.

**التصميم:**
- ملف `m130_params.json` داخل `/data/data/<app>/files/` يحوي الـ 20 معاملاً (`ROCKET_XTRGT`, `ROCKET_MASS_F`, …).
- قراءة عند `INIT`، إعادة قراءة عند حدث `setParam` من UI.
- UI: Kotlin screen يعرض الجدول ويُرسل تحديثات عبر `setParam(name, value)` JNI.
- **لا QGC param tuning** في هذه المرحلة (إن احتجناه نعيده في المرحلة 7).

**ما نخسره:** القدرة على تغيير params من QGC أثناء الطيران. البديل: من UI الأندرويد مباشرة.

**معيار النجاح:** تعديل param من UI يؤثر على MPC في ≤ 100 ms.

---

### المرحلة 5 — Logger (يوم واحد)
**الهدف:** استبدال PX4 `.ulog` بتسجيل بسيط.

**التصميم:**
- تسجيل CSV لكل إطار (timestamp, sensor snapshot, MHE state, MPC output) بمعدل 50 Hz.
- تدوير (rotation) كل 50 MB أو نهاية الطيران.
- تخزين في `/storage/emulated/0/Android/data/<pkg>/files/flights/flight_<ts>.csv`.
- تحليل لاحق في Python/MATLAB بدون أدوات PX4.

**معيار النجاح:** طيران HIL 10 دقائق → ملف CSV سليم قابل للتحليل.

---

### المرحلة 6 — نقل xqpower_can (3–5 أيام)
**الهدف:** تخليص درايفر CAN من PX4.

**الخطوات:**
1. استخراج `XqpowerCan.cpp` (1555 سطر) إلى `m130_can/`.
2. حذف `ModuleBase`, `ModuleParams`, `ScheduledWorkItem` ← استبدال بـ class عادي + خيط.
3. حذف `uORB::Subscription/Publication` ← يستقبل أوامر عبر `enqueue(ServoCommand)` من FlightManager.
4. الإبقاء الكامل على:
   - منطق SLCAN (open TTY, send/receive frames).
   - فك/بناء رسائل CAN لبروتوكول XQPOWER.
   - `SRV_FB` feedback (سنوجهه إلى logger بدل `debug_array`).
5. الاختبار على bench مع سيرفو حقيقي قبل HIL كامل.

**نقطة خطر:** توقيت CAN. PX4 WorkQueue يضمن 50 Hz ثابتة ذات jitter منخفض. خيط POSIX عادي قد يُعاني jitter على Android. **الحل:** استخدام `timerfd` مع priority عالي (`SCHED_FIFO` لهذا الخيط تحديداً — مسموح في Android مع permission CAP_SYS_NICE عبر JNI).

**معيار النجاح:**
- جلسة bench: إرسال أوامر deflection متعاقبة، قياس jitter < 2 ms (الحالي مع PX4 ~1 ms).
- HIL كامل: صاروخ يُحاكى في 6DOF Python، أوامر تذهب عبر CAN إلى سيرفوهات bench، feedback يعود إلى logger.

---

### المرحلة 7 (اختيارية) — MAVLink لـ QGroundControl (5–7 أيام)
**الهدف:** إن كان دعم QGC مطلوباً، نعيده بدون PX4.

**التصميم:**
- استخدام مكتبة `mavlink c_library_v2` (header-only) من ArduPilot/mavlink.
- خيط `MavlinkServer` يستمع UDP على 14550 ويرسل:
  - `HEARTBEAT` (1 Hz)
  - `ATTITUDE` (50 Hz) ← من MHE
  - `GLOBAL_POSITION_INT` (10 Hz)
  - `ACTUATOR_CONTROL_TARGET` (50 Hz)
  - `PARAM_VALUE` / `PARAM_SET` للضبط
- الجسر الحالي `mavlink_tcp_bridge.cpp` + `mavlink_pty_usb_bridge.cpp` يبقى كما هو (مستقل عن PX4).

**قرار:** إن لم يكن QGC في خطة الاستخدام الفعلي، **احذف هذه المرحلة تماماً** واوفر أسبوعاً.

---

### المرحلة 8 — حذف PX4 من البناء (نصف يوم)
**الهدف:** إزالة التبعية بالكامل.

**التغييرات في `AndroidApp/app/src/main/cpp/CMakeLists.txt`:**
- إزالة كل ما يخص `PX4_ROOT`, uORB generation, MAVLink generation, params generation.
- استبدال `add_subdirectory(PX4-Autopilot)` بـ `add_subdirectory(m130_core)` + `add_subdirectory(m130_can)` + (اختياري) `add_subdirectory(m130_mavlink)`.
- حذف `apps.cpp`, `apps.h`, `android_uorb_publishers.*`, `platforms/` (خاصة بـ PX4).

**ملفات تُحذف:**
- `AndroidApp/app/src/main/cpp/PX4-Autopilot/` (كامل المجلد).
- `generated/` (كامل، مولّدات PX4 params).
- `AndroidApp/HOW_TO_REPLACE_PX4.md` + `MIGRATION_PLAN_DETAILED.md` + `MIGRATION_V2_*` (لم تعد قابلة للتطبيق).

**ملفات تُبقى:**
- `m130_core/`, `m130_can/`, `m130_mavlink/` (جديدة).
- `native_sensor_reader.*`, `gps_usb_ubx.*`, `mavlink_*_bridge.*` (مستقلة).
- `c_generated_code/` (acados solver sources — مطلوبة).

**معيار النجاح:**
- `./gradlew assembleDebug` يُكمل في < 3 دقائق (الحالي ~11s فقط للإضافي، لكن الـ clean build يأخذ دقائق طويلة).
- APK حجم يتقلص بشكل محسوس.

---

## 4. مصفوفة المقارنة

| الوظيفة | الحالة الحالية (PX4) | الحالة الهدف | الخسارة |
|---------|---------------------|-------------|---------|
| MPC solver | acados داخل rocket_mpc | acados داخل m130_core | — |
| MHE estimator | acados داخل rocket_mpc | acados داخل m130_core | — |
| LOS guidance | داخل rocket_mpc | داخل m130_core | — |
| IMU/Baro/Mag | PX4 `sensors` + `ekf2` | `native_sensor_reader` → MHE مباشرة | EKF2 blend (إن كان يُستخدم) |
| GPS | PX4 `gps` driver | `gps_usb_ubx` → MHE مباشرة | — |
| State estimation | EKF2 + MHE | MHE فقط | fallback EKF2 (إن كان ضرورياً) |
| Arming/safety | PX4 `commander` | `FlightManager` state machine | QGC arm/disarm (إلا إن أعيد في م7) |
| Navigation | PX4 `navigator` | منطق LOS مدمج + waypoints في UI | تخطيط مسار معقد |
| Servo output (CAN) | `xqpower_can` PX4 module | `m130_can` standalone | — |
| Servo output (USB fallback) | `servo_usb_output` PX4 | يُنقل بنفس الطريقة (أو يُلغى) | — |
| Telemetry | PX4 mavlink module | `m130_mavlink` (اختياري) | QGC (إن حُذفت م7) |
| Parameters | PX4 params + QGC | JSON + Android UI | QGC param editor |
| Logging | `.ulog` | CSV | أدوات PX4 التحليلية |
| Control Allocator | PX4 module | mixer داخل MPC | — |
| Actuator test | PX4 `actuator_test` | Android UI screen | أمر `actuator_test` من QGC |

---

## 5. المخاطر وخطط التخفيف

| المخاطرة | الاحتمال | التأثير | التخفيف |
|----------|---------|--------|---------|
| **MHE بدون EKF2 blend يفقد دقة** | متوسط | عالي | المرحلة 1 تحتفظ بقدرة blend؛ الاختبار قبل قرار الحذف. |
| **jitter توقيت خيط POSIX > PX4 WorkQueue** | متوسط | متوسط | استخدام `SCHED_FIFO` + `timerfd` + measurement في المرحلة 0. |
| **xqpower_can يعتمد على features PX4 خفية** | منخفض | عالي | المرحلة 6 تبدأ bench testing قبل HIL كامل. |
| **اختلاف mavlink c_library_v2 عن PX4 mavlink** | منخفض | منخفض | اختيارية؛ تأخير المرحلة 7 حتى يثبت الباقي. |
| **فقدان شُبكة السلامة (commander's safety checks)** | متوسط | عالي | `FlightManager` يُعيد تنفيذ الفحوصات (GPS fix, battery, IMU health) كقائمة واضحة. |
| **HIL test يحتاج PX4 moduleX لم نلاحظه** | منخفض | متوسط | المرحلة 0 تُسجّل نتائج HIL مرجعية قبل التعديل. |
| **build فشل بسبب حذف ملفات مشتركة** | متوسط | منخفض | حذف PX4 في فرع مستقل بعد اكتمال الاستبدال. |

---

## 6. معايير النجاح الكلي

1. **وظيفي**: HIL كامل (6DOF Python + Android app) يُنتج مسار طيران بنفس miss distance ± 1 m مقارنة بالنسخة الحالية.
2. **أداء**: MPC solve time لكل iteration ≤ النسخة الحالية × 1.1.
3. **استقرار**: 10 طيرانات HIL متتالية بدون crash أو divergence.
4. **بناء**: clean build ≤ 3 دقائق (الحالي: طويل بسبب PX4 codegen).
5. **حجم**: APK يتقلص بنسبة ≥ 40%.
6. **صيانة**: لا يوجد "HOW_TO_REPLACE_PX4" — تحديث logic داخلي لا يتطلب merge PX4 upstream.

---

## 7. الجدول الزمني المقترح

| الأسبوع | المراحل | المخرجات |
|---------|---------|----------|
| 1 | م0 + م1 + م2 | `libm130_core.a` مستقلة، unit tests تمر |
| 2 | م3 + م4 + م5 + م6 (نصف) | FlightManager + params + logger + CAN bench |
| 3 | م6 (نصف) + م8 + HIL validation | PX4 محذوف، HIL يعمل |
| (+1) | م7 | mavlink + QGC (اختياري) |

**المجموع:**
- بدون QGC: **3 أسابيع**.
- مع QGC: **4 أسابيع**.

---

## 8. قرارات تحتاج منك

قبل بدء التنفيذ، نحتاج إجابة على:

1. **هل تستخدم QGroundControl فعلياً؟**
   - نعم → شمول المرحلة 7.
   - لا → حذفها، توفير أسبوع.

2. **هل الـ EKF2↔MHE cooperative blend حيوي أم تزييني؟**
   - حيوي → إبقاء `ecl/EKF` كمكتبة مستقلة.
   - تزييني → حذف EKF2 كلياً، اعتماد MHE فقط.

3. **هل `servo_usb_output` (المسار البديل عبر CP2102) ضروري؟**
   - نعم → نقله لـ `m130_usb_servo`.
   - لا → حذفه، الاعتماد على CAN فقط.

4. **ما أولوية التوافق مع فرع `rocket_gnc` القديم؟**
   - لا أولوية → حذف كل ما يخصه.
   - مهم → إبقاء `rocket_gnc` PX4 module في فرع parallel للرجوع.

5. **هل هناك اختبارات طيران حقيقية مجدولة قريباً؟**
   - نعم خلال شهر → **لا تبدأ الهجرة** الآن، استقر على النسخة الحالية.
   - لا → ابدأ الهجرة.

---

## 9. توصية نهائية

**ابدأ**، لكن بتدرج:
- نفّذ المرحلة 1 (فصل acados) كـ PR مستقل دون حذف PX4. هذا وحده يُثبت الجدوى ويوفر مكتبة قابلة للاختبار بمعزل.
- بعدها أكمل بقية المراحل بفروع صغيرة متتالية، لا بـ "big bang" واحد.
- احتفظ بـ PX4 قابلاً للتشغيل في فرع `legacy/px4-autopilot` لمدة شهرين بعد الهجرة للطوارئ.

هذه الخطة واقعية. الثقل الحقيقي ليس في الرياضيات (MPC/MHE موجودان ويعملان)، بل في حذف الكود الزائد بحذر وإثبات التكافؤ في كل خطوة.
