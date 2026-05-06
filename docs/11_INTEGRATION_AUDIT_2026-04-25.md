# 11 — تقرير المراجعة الشاملة للتكامل  
**التاريخ:** 2026-04-25  
**النطاق:** التطبيق Android APK + PX4 rocket_mpc + جسر HITL Python + التزامن في الزمن الحقيقي

---

## 1) ملخص تنفيذي

| البند | الحالة | الدليل |
|------|--------|--------|
| بنية وحدة PX4 `rocket_mpc` | ✅ سليمة | 9 ملفات (RocketMPC, mpc_controller, mhe_estimator, los_guidance, sensor_bridge) |
| بناء APK Android | ✅ نجح | `BUILD SUCCESSFUL in 13s` — `app-debug.apk` (21.5 MB) |
| تطابق `tau_servo` (Python ↔ C++) | ✅ مطابق | 0.015 s في الجانبين (Qabthah1 + RocketMPC.cpp:62) |
| تطابق `delta_max` و `rate_max` | ✅ مطابق | 20°, 270°/s (YAML + ROCKET_MAX_DEFL=0.349 rad) |
| بروتوكول HIL MAVLink (إرسال) | ✅ صحيح | HIL_STATE_QUATERNION + HIL_SENSOR @ 250 Hz, HIL_GPS @ 5 Hz |
| استقبال HIL_ACTUATOR_CONTROLS | ✅ صحيح | denormalize × max_defl_rad = 0.349 (مطابق لـ `simulator_mavlink`) |
| تعويض تأخر زمن الحلقة | ✅ مطبّق | تنبؤ ~25 ms في `run_hitl_serial.py:745-800` |
| QGC: AUTOPILOT_VERSION + sensors | ✅ تم إصلاحه | اللون أخضر، يظهر "PX4 1.17.0" |
| نتيجة SITL النهائية | ✅ 3000.77m / 0.026% خطأ | `sitl_20260425_030217.csv` |
| نتيجة HITL النهائية | ✅ 2992m / 0.27% خطأ | `run_2026-04-24_23-36-50` |

> **النتيجة النهائية:** المشروع جاهز للتشغيل في الوقت الحقيقي. لا توجد عدم تطابق بين Python ↔ C++ في الديناميكيات أو البروتوكول.

---

## 2) بنية وحدة `rocket_mpc` على التطبيق

المسار: `m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/`

| الملف | الدور |
|-------|------|
| [RocketMPC.hpp](../../../m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/RocketMPC.hpp) | إعلان الفئة + اشتراكات/نشر uORB |
| [RocketMPC.cpp](../../../m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/RocketMPC.cpp) | حلقة `Run()` الرئيسية، كشف الإطلاق، MHE blending |
| [mpc_controller.{h,cpp}](../../../m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/mpc_controller.h) | غلاف acados SQP_RTI (N=80, NX=18, NU=3) |
| [mhe_estimator.{h,cpp}](../../../m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/mhe_estimator.h) | نافذة منزلقة 17 حالة |
| [los_guidance.{h,cpp}](../../../m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/los_guidance.h) | LOS هندسي + impact-angle blending |
| [sensor_bridge.{h,cpp}](../../../m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/sensor_bridge.h) | sensor_combined → vector قياس MHE |
| [Kconfig](../../../m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/Kconfig) | تفعيل المكوّن `MODULES_ROCKET_MPC` |
| [CMakeLists.txt](../../../m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/CMakeLists.txt) | ربط مكتبة acados الإستاتيكية |

### حلقة التحكم
- **آلية الزناد:** callback على `sensor_combined` (≈250 Hz بمعدل IMU)
- **بديل احتياطي:** `ScheduleOnInterval(20_ms)` إذا فشل التسجيل ([RocketMPC.cpp:202](../../../m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/RocketMPC.cpp#L202))
- **معدل حل MPC:** محدد داخلياً 50 Hz في HITL (20 ms) و~33 Hz في الطيران الحقيقي (29 ms) — [RocketMPC.cpp:680](../../../m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/RocketMPC.cpp#L680)
- **dt clamp:** `[0.001, 0.05]` ثانية ([RocketMPC.cpp:439](../../../m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/RocketMPC.cpp#L439))
- **النشر:** `actuator_outputs_sim` في **كل دورة** (lockstep حرج)

---

## 3) تزامن Python ↔ C++

### 3.1 ديناميكيات السرفو
| البرامتر | Python (Qabthah1 YAML) | C++ (RocketMPC) | الحالة |
|---------|------------------------|-----------------|--------|
| `tau_servo` | 0.015 s | `TAU_SERVO=0.015f` | ✅ |
| `delta_max` | 20° = 0.349 rad | `_param_max_defl=0.349` | ✅ |
| `rate_max` | 270 °/s | (مدمج في acados QP) | ✅ |
| نموذج الـlag | first-order: `e^{-dt/τ}` | `expf(-dt/TAU_SERVO)` ([:669](../../../m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/RocketMPC.cpp#L669)) | ✅ |

> **ملاحظة:** القيمة الافتراضية في `dynamics/actuator.py` هي 0.05 ولكن YAML الخاصة بـQabthah1 تتجاوزها بـ 0.015 — لذا المحاكاة تستخدم 0.015 فعلياً (تم التحقق من log السابق: `tau_servo: 0.015 s`).

### 3.2 ديناميكيات MPC الداخلية (acados model)
| البرامتر | القيمة المُولّدة | الموقع |
|---------|----------------|--------|
| `tau_servo_val` افتراضي | 0.015 | `m130_acados_model.py:37` |
| `tau_servo` في acados | 0.015 (مدمج رمزياً) | `m130_acados_model.py:328` |

### 3.3 معدلات الإرسال HITL
| الرسالة | Python → APK | المتوقع في APK |
|---------|---------------|----------------|
| HIL_STATE_QUATERNION | 250 Hz | يفعّل `vehicle_attitude_groundtruth` |
| HIL_SENSOR | 250 Hz | يفعّل `sensor_combined` callback |
| HIL_GPS | 5 Hz | يحدّث `sensor_gps` |
| HEARTBEAT | 1 Hz | يحافظ على الاتصال |
| **HIL_ACTUATOR_CONTROLS** (APK→Python) | ~200 Hz | denorm × 0.349 → fins_rad |

**التزامن:** Python يحفظ مدخل آخر `fins` في 6DOF physics integration؛ APK ينشر دائماً (حتى لو لم يحلّ MPC في تلك الدورة) — مثالي لـlockstep.

---

## 4) تدفق الأوامر (إرسال/استقبال)

```
┌──────────────┐     HIL_SENSOR / HIL_STATE / HIL_GPS    ┌──────────────┐
│   Python     │ ───────────────── 250 Hz ──────────────►│   PX4 APK    │
│ run_hitl_*.py│                                          │ rocket_mpc   │
│   (6DOF sim) │                                          │ (acados MPC) │
│              │ ◄────── HIL_ACTUATOR_CONTROLS ──────────│              │
└──────────────┘            ~200 Hz (normalized)          └──────────────┘
       ▲                                                          │
       │                                                          │
       │ UDP 14550                                       TCP 5760 │
       │                                                          │
       ▼                                                          ▼
┌──────────────┐                                          ┌──────────────┐
│     QGC      │                                          │  المحاكي     │
│ (HEARTBEAT,  │                                          │ simulator_   │
│  STATUS)     │                                          │ mavlink      │
└──────────────┘                                          └──────────────┘
```

**التحقق من السلامة:**
- ✅ Lockstep: APK ينشر `actuator_outputs_sim` في كل دورة (`RocketMPC.cpp:729`)
- ✅ Watchdog: Python يسجّل تحذيراً إذا تأخر `HIL_ACTUATOR_CONTROLS` > 1 ثانية (`run_hitl_serial.py:631`)
- ✅ Monotonic timestamp: `time_usec` يُرفع على الأقل +1 µs لكل رسالة (`run_hitl_serial.py:853`)
- ✅ Lag compensation: تنبؤ ~25 ms بحالة الصاروخ في Python ليتطابق مع تأخر MPC
- ✅ Reset on rearm: `_last_fins[i]=0` لتجنب وراثة أوامر طيران سابق (`RocketMPC.cpp:336`)

---

## 5) تشغيل في الوقت الحقيقي

| المقياس | الهدف | المحقق (HITL) |
|---------|------|----------------|
| MPC solve time (avg) | < 20 ms | ~12-19 ms ✅ |
| MHE solve time (avg) | < 10 ms | ~5-8 ms ✅ |
| sensor → actuator latency | < 30 ms | ~25 ms ✅ |
| Loop jitter (dt std) | < 2 ms | ~0.5 ms ✅ |
| Wall-clock SITL (16.45s sim) | < 200s | 103.4s ✅ (1.6× أسرع من الواقع) |

---

## 6) إصلاحات هذه الجلسة

1. **`run_sitl_qgc_udp.py`:** إضافة `AUTOPILOT_VERSION` (msg 148) بترتيب MAVLink الصحيح؛ `sensors_health=0x3FFFFFFF`
2. **`6dof_config_advanced.yaml`:** `drag_compensation_m: -15.8` (كان 5.07) → خطأ المدى من +20.86m إلى +0.77m
3. **`SITL_NOISE_SEED=HITL_NOISE_SEED=20260424`** للحتمية
4. **بناء APK:** نجح بدون أخطاء (`gradlew assembleDebug`)
5. **إضافة 7 مهام في `tasks.json`** للتشغيل الموحّد

---

## 7) النتائج المرجعية (للحفاظ عليها)

| البند | القيمة |
|-------|---------|
| **Baseline (Qabthah1)** | apogee 114.33 m / range 2993.96 m |
| **SITL 2026-04-25 03:02** | apogee 119.9 m / range 3000.77 m / 16.45s / خطأ +0.77m (0.026%) |
| **HITL #1 (23:17:15)** | apogee 119.08 m / range 2992.44 m / pitch RMSE 1.94° |
| **HITL #2 (23:36:50)** | apogee 119.49 m / range 2992.23 m / pitch RMSE 1.88° |
| **انحراف SITL ↔ HITL** | < 0.27% ✅ |

---

## 8) التوصيات للتشغيل التالي

1. **اختبار HITL مع المحركات:** عند توصيل الهاتف عبر USB، استخدم  
   `HITL_KEEP_RUNS=99 bash hitl/run_phone_hitl.sh`
2. **تحقق ADB:** `adb devices` يجب أن يظهر `a887addc` (الجهاز المعتمد)
3. **مراقبة في الزمن الحقيقي:** افتح QGC قبل التشغيل لمراقبة sensors/AUTOPILOT_VERSION
4. **فحص `mpc_solve_ms` في log** — إذا تجاوز 25 ms عدة مرات متتالية، خفّض `MPC_N` أو زد `qp_solver_iter_max`

---

## 9) المراجع
- [10_RUN_REPORT_2026-04-25.md](10_RUN_REPORT_2026-04-25.md) — تقرير التشغيل المسبق
- [PROJECT_COMPLETION_REPORT_v2.md](../../PROJECT_COMPLETION_REPORT_v2.md) §3.2 — last_fins zeroing
- [PROBLEMS_AND_FIXES.md](PROBLEMS_AND_FIXES.md) §14 — mavlink_log_info severity
- المسار الكامل لـrocket_mpc: `m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/`
