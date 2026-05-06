# تقرير فحص شامل لـ `AndroidApp/app/src/main/cpp/`

**التاريخ**: 2026-04-30
**النطاق**: 19 ملف، 4440 سطر
**الفاحص**: Cascade

---

## ملخص تنفيذي

| الفئة | العدد |
|---|---|
| 🔴 مشاكل حرجة | 4 |
| 🟡 مشاكل متوسطة | 5 |
| 🟢 تنظيف منخفض الأولوية | 6 |
| ✅ مكونات سليمة | متعددة |

**النتيجة الإجمالية**: الكود يعمل بشكل صحيح في PIL/HITL (نتائج 98.9–100/100). المشاكل الحرجة تظهر تأثيرها بشكل أساسي في **Real Flight** أو في **صيانة الكود** مستقبلاً.

---

## فهرس الملفات المفحوصة

| الملف | الأسطر | المشاكل |
|---|---:|---|
| `apps.h` + `apps.cpp` | 95 | ✅ نظيف |
| `board_config.h` + `px4_boardconfig.h` | 109 | ✅ نظيف |
| `shared_sensor_data.h` | 57 | ✅ نظيف |
| `android_uorb_publishers.h/cpp` | 308 | 🔴 #3, #4 — 🟢 #12 |
| `native_sensor_reader.h/cpp` | 318 | 🔴 #1 — 🟢 #11 |
| `px4_jni.cpp` | 971 | 🔴 #2 — 🟡 #9 — 🟢 #10, #13 |
| `mavlink_tcp_bridge.h/cpp` | 162 | 🟡 #7, #8 |
| `mavlink_pty_usb_bridge.h/cpp` | 254 | 🟡 #5, #6 |
| `gps_usb_ubx.h/cpp` | 721 | 🟢 #14 |
| `servo_usb_output.h/cpp` | 442 | ✅ نظيف |
| `CMakeLists.txt` | 1011 | ✅ تم إصلاحه (N=80 ديناميكي) |

---

# 🔴 المشاكل الحرجة (4)

## #1 — اسم الباقة الخطأ في ASensorManager

**الموقع**: `native_sensor_reader.cpp:164`

```cpp
ASensorManager* mgr = ASensorManager_getInstanceForPackage("com.ardophone.flight");
```

**المشكلة**: الباقة الفعلية للتطبيق هي `com.ardophone.px4v17` لكن الكود يطلب `com.ardophone.flight`.

**ماذا تفعل الدالة؟**
`ASensorManager_getInstanceForPackage()` تربط مدير الحسّاسات بباقة. Android يستخدم الاسم لـ:
- تطبيق سياسات Doze mode و App Standby
- التحقق من `HIGH_SAMPLING_RATE_SENSORS` (مطلوب لمعدلات > 200Hz)
- Foreground service prioritization
- إحصائيات استهلاك البطارية

**الأثر**:
| السيناريو | السلوك |
|---|---|
| HITL/PIL | ⬜ لا أثر (sensors stopped على أي حال) |
| Real Flight (22005) | ⚠️ معدل IMU قد يهبط إلى ~200Hz بدلاً من 400Hz |
| EKF2 | ابتكار أكبر، gain تعديل أبطأ |

**الإصلاح**:
```cpp
ASensorManager* mgr = ASensorManager_getInstanceForPackage("com.ardophone.px4v17");
```

**الأولوية**: 🔴 عالية للطيران الحقيقي

---

## #2 — Land detector دائماً "multicopter"

**الموقع**: `px4_jni.cpp:643`

```cpp
const char* ld_argv[] = {"land_detector", "start", "multicopter", nullptr};
land_detector_main(3, (char**)ld_argv);
```

**المشكلة**: التعليق فوقه يقول _"multicopter افتراضياً — يتغير حسب SYS_AUTOSTART"_ لكن **لا يوجد منطق فعلي يغيّره**.

**ماذا يفعل land_detector multicopter؟**
يكتشف الهبوط بناءً على:
- ✓ throttle منخفض
- ✓ ثبات gyro/accel
- ✓ ارتفاع منخفض

**لماذا خطأ للصاروخ؟**
الصاروخ MAV_TYPE=9:
- ✗ يدور بـ spin سريع → فحص "ثبات gyro" يفشل
- ✗ قد يخطئ ويُفعّل عند **apogee** إذا الباروميتر ضوضاء

**الأثر**:
| السيناريو | الأثر |
|---|---|
| الصاروخ في apogee | 🔴 auto-disarm خاطئ → فقد التحكم |
| ارتطام بالأرض دواراً | لا يُكتشف → servo يبقى نشطاً |
| Logger | لا يقفل ملفات → فقد بيانات محتمل |
| Failsafe | `_have_taken_off_since_arming` غير مضبوط |

**الإصلاح**: استخدم `fixedwing` mode (أو احذف land_detector نهائياً للصاروخ):

```cpp
// قراءة airframe لاختيار mode المناسب
int32_t airframe = 0;
param_get(param_find("SYS_AUTOSTART"), &airframe);

const char* ld_mode = "fixedwing"; // أنسب للصاروخ
const char* ld_argv[] = {"land_detector", "start", ld_mode, nullptr};
land_detector_main(3, (char**)ld_argv);
```

**الأولوية**: 🔴 عالية جداً للطيران الحقيقي

---

## #3 — دالة ميتة `sensor_ts_to_hrt`

**الموقع**: `android_uorb_publishers.cpp:78-97`

```cpp
static hrt_abstime sensor_ts_to_hrt(int64_t android_ns) {
    const hrt_abstime now       = hrt_absolute_time();
    const hrt_abstime sensor_us = (hrt_abstime)(android_ns / 1000);
    if (!s_clock_offset_valid || (now - s_last_sync_us) > CLOCK_SYNC_INTERVAL_US) {
        s_clock_offset_us   = now - sensor_us;
        s_clock_offset_valid = true;
        s_last_sync_us       = now;
    }
    return sensor_us + s_clock_offset_us;
}
```

**المشكلة**: الدالة معرّفة كـ `static` و**لا تُستدعى أبداً**. Publisher يستخدم `g_sensor_data.accel.hrt_receipt_us` مباشرة.

**التحقق**:
```bash
grep -n "sensor_ts_to_hrt" android_uorb_publishers.cpp
# 85: تعريف فقط — لا استدعاء
```

**ما يستخدم الآن (نهج أصح)**:
```cpp
// native_sensor_reader.cpp:82
const hrt_abstime receipt_us = hrt_absolute_time();
g_sensor_data.accel.hrt_receipt_us = receipt_us;

// android_uorb_publishers.cpp:117
const hrt_abstime sample_ts = g_sensor_data.accel.hrt_receipt_us;
```

**الأثر السلوكي**: ⬜ لا شيء (النهج الجديد أبسط وأصح).

**أثر الصيانة**:
| العنصر | المشكلة |
|---|---|
| 20 سطر تعليق يشرح "الحل" | يخلق فهماً خاطئاً للكود الفعلي |
| 3 file-scope variables (`s_clock_offset_us`, ...) | ذاكرة + ضوضاء في debugger |
| مخاطر مستقبلية | مطور قد "يصلح" استدعاء الدالة → يعيد drift |

**الإصلاح**: حذف الدالة + المتغيرات + التعليق:

```cpp
// احذف الأسطر 65-97 بالكامل
```

**الأولوية**: 🟡 متوسطة (نظافة كود، لا أثر سلوكي)

---

## #4 — `s_cached_ekf_status` لا يُحدَّث

**المواقع**:
- `android_uorb_publishers.cpp:53` — تعريف
- `android_uorb_publishers.cpp:256` — getter
- `px4_jni.cpp:923-926` — JNI exposed

```cpp
static std::atomic<int> s_cached_ekf_status{0};
// ...
int get_ekf_status() { return s_cached_ekf_status.load(...); }
```

**المشكلة**: المتغير معرّف ومكشوف عبر JNI لـ Kotlin، لكن **لا يوجد كود يكتب فيه**. يُرجع 0 دائماً.

**ما المتوقع**: قراءة من `estimator_status` topic:
```cpp
// مفقود حالياً:
estimator_status_s est{};
if (s_estimator_sub.copy(&est)) {
    s_cached_ekf_status.store(est.pre_flt_fail_innov_heading || ... ? 0 : 1);
}
```

**الأثر**:
- شاشة UI تعرض "EKF: 0" دائماً
- المستخدم لا يعرف إذا EKF متقارب قبل الإطلاق
- يفقد طبقة أمان مرئية للمستخدم

**الإصلاح**: إضافة subscription لـ `estimator_status` في `publisher_loop()`:

```cpp
#include <uORB/topics/estimator_status_flags.h>
static uORB::Subscription s_estimator_sub{ORB_ID(estimator_status_flags)};

// في publisher_loop داخل بلوك UI update (50ms):
estimator_status_flags_s est_flags{};
if (s_estimator_sub.copy(&est_flags)) {
    bool ekf_ok = est_flags.cs_yaw_align && est_flags.cs_global_pos;
    s_cached_ekf_status.store(ekf_ok ? 1 : 0, std::memory_order_relaxed);
}
```

**الأولوية**: 🟡 متوسطة (UX/أمان مرئي)

---

# 🟡 المشاكل المتوسطة (5)

## #5 — `shutdown()` على PTY fd

**الموقع**: `mavlink_pty_usb_bridge.cpp:197`

```cpp
if (g_pty_master >= 0) (void)shutdown(g_pty_master, SHUT_RDWR);
```

**المشكلة**: `shutdown()` يعمل على sockets فقط. PTY master ليس socket → سلوك غير معرّف (عادة يفشل بـ ENOTSOCK).

**النية الأصلية**: إيقاظ thread العالق في `read(g_pty_master)`.

**يعمل بالصدفة** لأن:
- `g_threads_should_run = false`
- `poll(g_pty_master, 100ms)` يخرج بعد 100ms
- thread يفحص الـ flag ويخرج

**الإصلاح**: احذف السطر، أو استخدم `close()` (لكنه يحتاج معالجة دقيقة):
```cpp
// احذف السطر — poll timeout 100ms كافٍ
```

---

## #6 — Globals بدون `static` في PTY bridge

**الموقع**: `mavlink_pty_usb_bridge.cpp:35-42`

```cpp
std::mutex g_bridge_mutex;
std::atomic<bool> g_running{false};
std::thread g_th_pty_to_usb;
std::thread g_th_usb_to_pty;
int g_usb_fd{-1};
int g_pty_master{-1};
std::string g_slave_path;
std::atomic<bool> g_threads_should_run{false};
```

**المشكلة**: تتسرّب لـ linker namespace العام. تعارض محتمل مع نفس الأسماء في ملفات أخرى.

**الإصلاح**: أضف `static` على كل واحد:
```cpp
static std::mutex g_bridge_mutex;
static std::atomic<bool> g_running{false};
// ... إلخ
```

---

## #7 — سباق في TCP bridge stop

**الموقع**: `mavlink_tcp_bridge.cpp:144-153`

```cpp
void mavlink_tcp_bridge_start(...) {
    s_bridge_thread = std::thread(bridge_loop, ...);
    s_bridge_thread.detach();  // ← detached!
}

void mavlink_tcp_bridge_stop() {
    s_bridge_running = false;
    if (s_tcp_server_fd >= 0) {
        close(s_tcp_server_fd);  // ← يغلق fd من thread آخر
        s_tcp_server_fd = -1;
    }
}
```

**المشكلة**:
1. `s_bridge_thread.detach()` → لا يمكن `join()`
2. إغلاق `s_tcp_server_fd` من thread آخر بينما `accept()/poll()` قيد التشغيل = سلوك غير معرّف
3. قد يحدث use-after-close إذا جاء client جديد بنفس fd رقم

**الإصلاح**: لا تستخدم `detach()`، اترك `bridge_loop` يغلق الـ fd بنفسه:
```cpp
// start: لا detach
s_bridge_thread = std::thread(bridge_loop, tcp_port, udp_port);

// stop:
s_bridge_running = false;
if (s_bridge_thread.joinable()) {
    s_bridge_thread.join();  // ينتظر loop تنتهي وتغلق fd
}
```

ثم في `bridge_loop` تغلق `s_tcp_server_fd` بنفسها بعد الخروج من حلقة `while`.

---

## #8 — لا `TCP_NODELAY` على QGC connection

**الموقع**: `mavlink_tcp_bridge.cpp:65` (بعد accept)

**المشكلة**: TCP افتراضياً يفعّل Nagle's algorithm (تجميع packets صغيرة). MAVLink ينتج رسائل صغيرة (~30-280 byte) عالية التردد. Nagle يضيف تأخير ~40ms أحياناً.

**الإصلاح**:
```cpp
int tcp_client_fd = accept(...);
// أضف فوراً:
int nodelay = 1;
setsockopt(tcp_client_fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
```

---

## #9 — auto-arm hardcoded sleep في HITL

**الموقع**: `px4_jni.cpp:783-789`

```cpp
std::thread([]() {
    sleep(3); // قصير: ...
    LOGI("HITL auto-arm: invoking 'commander arm -f'");
    const char* arm_argv[] = {"commander", "arm", "-f", nullptr};
    commander_main(3, (char**)arm_argv);
}).detach();
```

**المشكلة**: تأخير 3 ثوانٍ مثبت. إذا EKF لم يتقارب أو `simulator_mavlink` لم يتصل بعد، أمر `arm` يفشل بصمت.

**الإصلاح**: poll حتى `vehicle_status.pre_flight_checks_pass`:
```cpp
std::thread([]() {
    uORB::Subscription status_sub{ORB_ID(vehicle_status)};
    for (int i = 0; i < 100; i++) {  // 10s timeout
        usleep(100000);
        vehicle_status_s s{};
        if (status_sub.copy(&s) && s.pre_flight_checks_pass) {
            const char* arm_argv[] = {"commander", "arm", "-f", nullptr};
            commander_main(3, (char**)arm_argv);
            LOGI("HITL auto-arm succeeded after %d ms", i * 100);
            return;
        }
    }
    LOGE("HITL auto-arm timeout (10s) — pre-flight checks never passed");
}).detach();
```

---

# 🟢 التنظيف منخفض الأولوية (6)

## #10 — `extern pthread_t _shell_task_id` غير مستخدم

**الموقع**: `px4_jni.cpp:39`
**الإصلاح**: احذف السطر.

---

## #11 — `g_sensor_counts.imu` يُزاد على gyro فقط

**الموقع**: `native_sensor_reader.cpp:116`

```cpp
case ASENSOR_TYPE_GYROSCOPE: case 16: {
    // ...
    g_sensor_counts.imu.fetch_add(1, ...);  // ← imu counter on gyro events
}
```

**المشكلة**: اسم `imu` مضلل — يحسب gyro فقط. لا يحسب accel.

**الإصلاح**: إما أعد التسمية إلى `gyro` أو احسب accel أيضاً.

---

## #12 — `s_mag_x/y/z` يمكن أن تكون locals

**الموقع**: `android_uorb_publishers.cpp:41-43`

```cpp
static float s_mag_x = 0.0f;
static float s_mag_y = 0.0f;
static float s_mag_z = 0.0f;
```

**المشكلة**: تُستخدم فقط داخل بلوك واحد في `publisher_loop`. لا داعي لأن تكون file-scope.

**الإصلاح**: حوّلها إلى locals داخل البلوك.

---

## #13 — MAVLink rate `40000` مثبت

**الموقع**: `px4_jni.cpp:714`

```cpp
const char* mav_argv[] = {"mavlink", "start", "-u", "14550", "-o", "14551",
                          "-t", "127.0.0.1", "-r", "40000", "-m", "config", nullptr};
```

**الإصلاح**: اقرأ من PARAM (مثل `MAV_0_RATE`) لتعديل وقت التشغيل بدون إعادة بناء.

---

## #14 — log message غير متطابق في GPS

**الموقع**: `gps_usb_ubx.cpp:611-612`

```cpp
} else {
    snprintf(s_usb_gps_msg, sizeof(s_usb_gps_msg), "cfg FAIL after 3 attempts");
    LOGE_G("u-blox configure failed after 3 attempts");
}
```

**المشكلة**: الحلقة فعلياً 5 محاولات (`for (int attempt = 0; attempt < 5; attempt++)` في السطر 545)، لكن log يقول 3.

**الإصلاح**:
```cpp
snprintf(s_usb_gps_msg, sizeof(s_usb_gps_msg), "cfg FAIL after 5 attempts");
LOGE_G("u-blox configure failed after 5 attempts");
```

---

## #15 — log غير دقيق في PTY bridge

**الموقع**: `mavlink_pty_usb_bridge.cpp:100`

```cpp
if (r == 0) ioctl(fd, USBDEVFS_RELEASEINTERFACE, &iface);
```

**ملاحظة**: قد يقع `r` على قيمة من `cp210x_configure` بدلاً من نتيجة `CLAIMINTERFACE`. (في الحقيقة `cp210x_configure` يستخدم `r` محلي لكن المنطق هنا هش).

---

# ✅ ما يعمل بشكل ممتاز

## ADPF Integration (`RocketMPC.cpp`)
- Dynamic loading via `dlsym` → يعمل على Android 12+ مع fallback لإصدارات أقدم
- CPU affinity على prime cluster (cores 4-7 على Snapdragon 8 Gen 3)
- nice -20 fallback عندما SCHED_FIFO يفشل (بدون root)
- ReportActual في كل MPC cycle → governor يحافظ على التردد

## Sensor Pipeline
- `phone_to_frd()` rotation رياضياً صحيح
- `hrt_receipt_us` يُسجَّل لحظة الاستقبال = أصح timestamp_sample
- 3-second warmup قبل `sensors_main` يضمن وصول first batch
- ALooper polling indefinite + ALooper_wake للإيقاف نظيف

## HITL Optimization
- يوقف `native_sensor_reader` و `gps_usb_ubx` في HITL
- يوفر CPU للـ MPC solver
- HIL_SENSOR/HIL_GPS من simulator يحلان محل phone hardware

## Multi-source Servo Output (`servo_usb_output.cpp`)
- نظام أولويات نظيف: actuator_test > actuator_servos > actuator_outputs_sim
- توثيق ممتاز للـ scaling distinction (`MAX_ANGLE_DEG=25` vs `XQCAN_LIMIT=20`)
- التعامل مع disarm → fins zeroed

## Parameter Boot Logic (`px4_jni.cpp`)
- `first_rocket_run` detection عبر SYS_AUTOSTART
- `airframe_changed` detection عبر SYS_AUTOCONFIG
- يحافظ على تعديلات المستخدم عبر إعادة التشغيل
- branches منفصلة لـ Real / HITL / SITL

---

# خطة التنفيذ المقترحة

## المرحلة 1 — إصلاحات سريعة (15 دقيقة)
- [ ] #1: تصحيح اسم الباقة → `com.ardophone.px4v17`
- [ ] #10: حذف `extern pthread_t _shell_task_id`
- [ ] #14: تصحيح log message لـ GPS

## المرحلة 2 — إصلاحات حرجة للطيران (30 دقيقة)
- [ ] #2: تغيير land_detector إلى `fixedwing` للصاروخ
- [ ] #4: ربط `s_cached_ekf_status` بـ `estimator_status_flags`

## المرحلة 3 — تصلّب الـ bridges (45 دقيقة)
- [ ] #5: حذف `shutdown()` الخطأ على PTY
- [ ] #6: إضافة `static` على globals
- [ ] #7: إصلاح race condition في TCP bridge
- [ ] #8: إضافة `TCP_NODELAY`
- [ ] #9: استبدال auto-arm sleep بـ poll

## المرحلة 4 — تنظيف (15 دقيقة)
- [ ] #3: حذف `sensor_ts_to_hrt` الدالة الميتة
- [ ] #11: إعادة تسمية `imu` counter أو إضافة accel
- [ ] #12: تحويل `s_mag_x/y/z` إلى locals
- [ ] #13: نقل MAVLink rate إلى PARAM

**إجمالي الوقت المقدّر**: ~1.75 ساعة

---

# الإصلاحات السابقة (مرجع)

تم إنجازها في جلسات سابقة:

| الإصلاح | الموقع |
|---|---|
| `ROCKET_MPC_TF` 4.0 → 1.6 | `generated/parameters/px4_parameters.hpp:8522` |
| `XQCAN_LIMIT` 25.0 → 20.0 | `generated/parameters/px4_parameters.hpp:11691` |
| CMake N check ديناميكي (كان hardcoded N=40) | `CMakeLists.txt:944-963` |
| N=80 ثابت في كل النظام | YAML + Python + headers + lib + APK |
| ADPF كامل | `RocketMPC.cpp` + `mpc_controller.cpp` |

---

**نهاية التقرير**
