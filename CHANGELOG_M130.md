# سجل تعديلات M130

قائمة مبسّطة: **ماذا تغيّر، ولماذا، وأين.** لا تاريخ محاولات ولا سرد.

---

## ⚠️ حالة التطبيق — جلسة 2026-05-07

**ملخّص**: الأقسام أدناه (1–15) **مُؤجَّلة بالكامل** بقرار مُراجع موثَّق.
**السبب**: الـ HITL وَصل لـ baseline مُستقر بإصلاح **lockstep=false** فقط
(انظر `AI_GOVERNANCE/BASELINES.md` — Run #5 score 66.5/100). المُعاينة
أَكَّدت أن لا تَعديل من #1–#15 يُعالج المشكلة المُتبقّية في الـ runs
(Range −52%، Peak Alt 61m فقط) — هذه مشكلة **flight dynamics**،
بينما كل التَعديلات تَستهدف **infrastructure** (EKF2/sensors/ARM).

### تَفصيل التَأجيل لكل تَعديل

| # | التَعديل | الأثر المُتوقَّع في HITL الحالي | شَرط العَودة |
|---|---|---|---|
| #1 | `EKF2_MAG_TYPE` ديناميكي 1⇔5 | **no-op** (EKF2 مُعطَّل في HITL، `ROCKET_USE_GT=1`) | عند الانتقال إلى Real Flight أو حالة EKF2 مُفعَّلة |
| #2 | Launch detection بـ `ax > 1g` | منطق جديد، **لن يُغيّر trajectory** بعد الإطلاق | عند تَشخيص حقيقي لمشكلة launch detection (لم تَظهر في runs الحالية) |
| #3 | أفق MPC `N=40, tf=1.6` | لم يُجرَّب — يَحتاج SITL→PIL→HITL parity | بعد إنجاز diff testing بين Python ↔ C++ MPC |
| #4 | `ROCKET_MPC_TF=1.6` في generated params | متّصل بـ #3 | مع #3 |
| #5 | فحص N ديناميكي في CMake | تنظيمي، لا أثر runtime | عند tooling overhaul |
| #6 | فلتر α/β في sitl_analysis | SITL فقط | لا يَخصّ HITL |
| #7 | `angular_velocity = [0,0,0]` | يَخصّ Python sim | عند تَحديث نموذج الإطلاق |
| #8 | تحديث نموذج actuator | يَحتاج SITL parity testing أولاً | بعد diff testing |
| #9 | تنظيف تعليقات | لا أثر runtime | متى شُئنا |
| #10a | IMU 400→100 Hz comments | تنظيمي | متى شُئنا |
| **#10b** | `samplingPeriodUs = 10000` | للحرارة (runs الحالية 7s، لا حاجة) | عند runs > 5 دقائق أو thermal stress |
| **#10c** | `usleep(2500)→usleep(10000)` | **🔴 مُؤكَّد سَبَّب regression سابقاً** | فقط مع benchmarking دقيق + A/B test |
| #11 | MHE `horizon_dt` (توثيقي) | تَوثيق فقط | لا يَحتاج تَطبيق |
| **#12.1** | `EKF2_REQ_EPH/EPV` | **no-op** (EKF2 مُعطَّل في HITL) | Real Flight أو EKF2 مُفعَّل |
| #12.2 | `EKF2_MAG_ACCLIM=5.0` | EKF2 فقط | كذلك |
| **#12.3** | `_debug_array_pub` كل cycle | **🔴 مُؤكَّد سَبَّب 100% lockstep timeouts** | فقط بـ rate-limit ≤ 10 Hz **و** قناة منفصلة |
| #12.4 | `EKF2_REQ_GPS_H = 1.0` | EKF2 فقط | Real Flight |
| **#12.5** | `COM_CPU_MAX/RAM_MAX = -1` | يَحلّ ARM-block (load_mon)، **لم نَرَه في runs الأخيرة** | إن ظَهر "No CPU/RAM load info" يَمنع ARM |
| #12.6 | حذف `EKF2_GPS_V/P_NOISE` | EKF2 فقط | Real Flight |
| #12.7 | حذف `EKF2_MAG_TYPE` من airframes | متّصل بـ #1 | مع #1 |
| #12.8 | `EKF2_MAG_TYPE` reset 1→0 | متّصل بـ #1 | مع #1 |
| #12.9 | `EKF2_ABL_LIM` 1.0→0.8 | EKF2 فقط | Real Flight |
| #13 | XqpowerCan 200→100 Hz | السيرفو حالياً يَعمل CAN=100% | عند مُلاحظة CAN saturation فعلي |
| #14 | EKF2 mag/GPS tuning (5 params) | EKF2 فقط | Real Flight |
| #15 | `EKF2_ANGERR_INIT=0.01` | EKF2 فقط | Real Flight |

### الشُروط العامة لإعادة التَطبيق (مُتّفَق عليها)

أيّ تَعديل يَعود فقط بعد **استيفاء الـ 4 شُروط**:

1. **سبب مَلموس بالأرقام**: log أو metric يُثبت المشكلة التي يُعالجها التَعديل (لا تَطبيق "احترازي").
2. **A/B test مُسجَّل**: قبل/بعد على نفس الـ baseline (Run #5 الحالي = baseline reference).
3. **revert فوري إن ساء أيّ metric**: الالتزام بـ `LESSONS_LEARNED.md` rule.
4. **single change at a time**: لا دَمج تَعديلَين في run واحد.

### Backups المحفوظة لكلّ تَعديل بَدأ في الجلسة

- `px4_jni.cpp.pre_round1_12.5_1778139280` — قبل #12.5 (revert تَمَّ، لكن backup يُحفَظ للمُراجَعة)
- `hil_config.yaml.pre_lockstep_fix_1778138419` — قبل lockstep=false (الإصلاح المُعتَمَد)

---

## 1. `EKF2_MAG_TYPE` ديناميكي (AUTO قبل الإطلاق، NONE بعده)

**الملفات:**
- `AndroidApp/app/src/main/cpp/px4_jni.cpp` (~سطر 482)
- `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/RocketMPC.cpp` (في `_reset_flight_state()` وعند كشف الإطلاق)

**ماذا:**
- القيمة الابتدائية عند startup: `0 (AUTO)` (افتراضي PX4 — لم يعد يُضبط من `px4_jni.cpp`).
- عند arm/disarm: تُستعاد إلى `0`.
- عند كشف الإطلاق: تُبدَّل إلى `5` عبر `param_set_no_notification`.

**لماذا:** قبل الإطلاق نحتاج mag (وضع AUTO) ليتقارب yaw. بعد الإطلاق، التشويش الكهرومغناطيسي + تغيّر المجال الأرضي مع الارتفاع يُفسد قراءة mag، فنوقف fusion ونعتمد على gyro + GPS course.

---

## 2. تبسيط كشف الإطلاق

**الملفات:**
- `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/RocketMPC.cpp` (~سطر 990)
- `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/RocketMPC.hpp` (حذف `_launch_dv`)

**ماذا:** الشرط الآن هو `armed && ax > 1g` فقط. حُذف شرط تراكم Δv.

```cpp
if (!_launched && _armed && sc.timestamp > 0 && ax > 1.0f * 9.80665f) {
    _launched = true;
    ...
}
```

**لماذا:** كشف أسرع وأبسط. الشرط `armed` يحمي من false-positive من الصدمات.

---

## 3. أفق MPC موحَّد: `N = 40`, `tf = 1.6`

**الملف:** `6DOF_v4_pure/mpc/m130_ocp_setup.py`

**ماذا:**
| المتغيّر | قبل | بعد |
|---|---|---|
| `N` | 200 | **40** |
| `ocp.solver_options.tf` | 4.0 | **1.6** |
| `TAU_TRANSPORT_S` | 0.0 | **0.110** |
| `DELAY_MODEL` | `'pade'` | محذوف |

**لماذا:** لتوحيد الأفق بين كل الطبقات (acados generated code + px4 params + Android headers). أي عدم تطابق في `dt_stage = tf/N` يُنتج MPC يحلّ بافتراضات زمنية مختلفة عن الـ controller.

---

## 4. `ROCKET_MPC_TF` في الـ generated params

**الملف:** `AndroidApp/app/src/main/cpp/generated/parameters/px4_parameters.hpp` (~سطر 8521)

**ماذا:** `.val = { .f = 4.0 }` → `.val = { .f = 1.6 }`.

**لماذا:** الملف مُولَّد لكنّه مُلتزَم في git (لأن build system الأندرويد لا يعيد توليده). لو بقي 4.0، سيحصل mismatch مع الـ solver المُصدَّر بـ `tf=1.6`.

---

## 5. فحص N ديناميكي في CMake

**الملف:** `AndroidApp/app/src/main/cpp/CMakeLists.txt` (~سطر 944)

**ماذا:** مقارنة header الـ Android مع header الـ SITL المرجعي بدل تثبيت `N=200`.

```cmake
file(STRINGS ".../m130_mpc/acados_solver_m130_rocket.h" _mpc_n_line REGEX "^#define M130_ROCKET_N ")
file(STRINGS ".../c_generated_code/acados_solver_m130_rocket.h" _sitl_n_line REGEX "^#define M130_ROCKET_N ")
if(NOT "${_mpc_n_line}" STREQUAL "${_sitl_n_line}")
    message(FATAL_ERROR ...)
endif()
```

**لماذا:** لا يكسر البناء عند تغيير الأفق لاحقاً، ويكشف mismatch بين headers الـ ARM64 والـ SITL.

---

## 6. فلتر α/β قبل الإطلاق في تحليل SITL

**الملف:** `6DOF_v4_pure/sitl/sitl_analysis.py` (~سطر 161)

**ماذا:** حساب `max_alpha_deg` و `max_beta_deg` من عيّنات `speed_total > 10 m/s` فقط.

**لماذا:** `α` و `β` من `atan2(...)` غير معرَّفين عند `V → 0`. قبل التعديل كانت القيم المزيّفة تصل إلى 139°+ وتُفسد score التقييم.

---

## 7. تصفير السرعة الزاوية الابتدائية

**الملف:** `6DOF_v4_pure/config/6dof_config_advanced.yaml` (~سطر 150)

**ماذا:** `angular_velocity: [0.1, 0, 0] → [0, 0, 0]`.

**لماذا:** اختبارات نظيفة بشروط ابتدائية صفرية. أي اضطراب يُعاد لاحقاً عند اختبارات stress.

---

## 8. تحديث نموذج المُشغِّل (actuator)

**الملف:** `6DOF_v4_pure/data/rocket_models/Qabthah1/rocket_properties.yaml` (كتلة `actuator`)

**ماذا:**
| المعامل | قبل | بعد |
|---|---|---|
| `model` | 1 | **2** (second-order + delay) |
| `wn` | 100.0 | **75** |
| `zeta_wn` | 100.0 | **75** |
| `delay_steps` | 11 | **2** |
| `rate_max` | 270.0 | **300** |
| `backlash` | 0.0 | **0.08** |

**لماذا:** مطابقة أدقّ لسيرفو KST X20 الفعلي (rate المقاس = 300 dps، backlash صغير موجود فعلاً). تقليل `delay_steps` يجنّب mismatch مع افتراضات MPC.

---

## 9. تنظيف كود تأخير المُشغِّل (بلا أثر runtime)

> هذه التغييرات تنظيفية فقط. `N_DELAY_BUFFERS = 0` في كلتا النسختين، فالكود المعدَّل لا يُنفَّذ. **يمكن تجاهلها.**

| الملف | ماذا |
|---|---|
| `6DOF_v4_pure/mpc/m130_acados_model.py` | حذف فرع Padé(2,2) + معامل `delay_model` من توقيع الدالة |
| `6DOF_v4_pure/mpc/m130_mpc_autopilot.py` | تهيئة delay buffers بـ `full(last_delta)` بدل `zeros` |
| `AndroidApp/.../rocket_mpc/mhe_estimator.cpp` (~585) | تعليق توضيحي فقط |
| `6DOF_v4_pure/mpc/m130_mhe_estimator.py` (~273) | تعليق توضيحي فقط |
| `6DOF_v4_pure/results/advanced_analysis.py` (~86) | سطرين فارغين |
| `AndroidApp/.../rocket_mpc/RocketMPC.cpp` (قبل `lookahead_stage = 6`) | حذف تعليق 5 أسطر |

---

## 10. ترحيل سلسلة IMU من 400 Hz إلى 100 Hz

**الملفات:**
- `AndroidApp/app/src/main/cpp/native_sensor_reader.cpp` (~سطر 200–241)
- `AndroidApp/app/src/main/cpp/android_uorb_publishers.cpp` (~سطر 232–247)
- `AndroidApp/app/src/main/cpp/px4_jni.cpp` (~سطر 222–232, 345–374)

**الدافع:**

| المرحلة | المعدل القديم | المستهلك الفعلي | الهدر |
|---|---|---|---|
| Phone HW (accel/gyro) | ~400–500 Hz | EKF2 + MPC | 4× |
| EKF2 prediction step | 200 Hz | لا أحد | 2× |
| `VehicleAngularVelocity` republish | 400 Hz | rate controller داخلي للصاروخ (غير مستخدم — MPC هو الحاكم) | 4× |
| MPC outer loop | 25 Hz | controller المخارج | 1× (هذا هو الذي يحدد الباقي) |
| MHE solve | 25 Hz فعلياً (محدود بنقطة الاستدعاء) | EKF/MPC blend | 1× |

النتيجة: استهلاك CPU وبطارية بلا فائدة، ومطابقة سيئة بين معدل الـ IMU الفعلي ومعدل الـ EKF (الـ downsampler كان يرمي 3 من كل 4 عينات).

الهدف: **تثبيت سلسلة الاستشعار كاملة عند 100 Hz** مع إبقاء الـ MHE/MPC كما هما (25 Hz) لأن الـ acados solver المُولَّد مرتبط بـ `horizon_dt = 0.02 s`.

**المسار الكامل للـ IMU:**

```
[1] Phone HW (gyro/accel chip)
        ↓ Android Sensor HAL (kernel driver)
[2] native_sensor_reader.cpp::sensor_thread_func()
    └─ ASensorEventQueue_registerSensor(queue, gyro, samplingPeriodUs, 0)
       └─ samplingPeriodUs يحدد الحد الأعلى لمعدل التسليم
    └─ ALooper_pollOnce(-1, ...)         ← ينتظر event
    └─ process_event()
       └─ phone_to_frd() (Android XYZ → FRD)
       └─ يكتب في g_sensor_data.{accel,gyro} + has_new_data=true
        ↓ SharedSensorData (mutex-guarded struct, in-process)
[3] android_uorb_publishers.cpp::publisher_loop()
    └─ usleep(N) ← polling thread (لا يستيقظ على event)
    └─ يقرأ has_new_data، يحوّل timestamp Android→HRT
    └─ s_accel_pub.publish(sensor_accel_s) / s_gyro_pub.publish(sensor_gyro_s)
        ↓ uORB topic: sensor_accel + sensor_gyro
[4] PX4 sensors module → VehicleIMU.cpp
    └─ يجمع/يكامل العينات إلى delta_angle/delta_velocity
    └─ يَنشر vehicle_imu_s كل 1/IMU_INTEG_RATE ثانية
        ↓ uORB topic: vehicle_imu
[5] EKF2.cpp::Run()
    └─ يقرأ vehicle_imu → EstimatorInterface::setIMUData()
    └─ _imu_down_sampler.update(imu_sample)
       └─ إذا IMU أسرع من EKF2_PREDICT_US: يجمّع ويُسقط
       └─ إذا أبطأ أو مساوٍ: pass-through
    └─ يدفع للـ _imu_buffer كل EKF2_PREDICT_US
        ↓
[6] Ekf::update()
    └─ predictCovariance() + predictState()
    └─ controlFusionModes()  ← دمج GPS/Mag/Baro/...
    └─ output_predictor.correctOutputStates()
        ↓ vehicle_attitude / vehicle_local_position publishers
```

نقاط التحكم الفعلية:

1. **`samplingPeriodUs`** في الخطوة [2] — يحدد المعدل من الـ HW.
2. **`usleep`** في الخطوة [3] — يحدد دقة إعادة النشر إلى uORB.
3. **`IMU_INTEG_RATE`** في الخطوة [4] — معدل packets `vehicle_imu` التي يستهلكها EKF.
4. **`EKF2_PREDICT_US`** في الخطوة [5] — معدل خطوة التنبؤ في EKF.
5. **`IMU_GYRO_RATEMAX`** — معدل `vehicle_angular_velocity` (مستخدم بواسطة rate controller التقليدي).

**ماذا (native_sensor_reader.cpp):**

قبل:
```cpp
ASensorEventQueue_registerSensor(queue, accel, 0, 0);  // fastest available
ASensorEventQueue_registerSensor(queue, gyro,  0, 0);
ASensorEventQueue_registerSensor(queue, mag,   0, 0);
ASensorEventQueue_registerSensor(queue, baro,  0, 0);
```

بعد:
```cpp
static constexpr int32_t IMU_PERIOD_US = 10'000;  // 100 Hz
ASensorEventQueue_registerSensor(queue, accel, IMU_PERIOD_US, 0);  // 100 Hz
ASensorEventQueue_registerSensor(queue, gyro,  IMU_PERIOD_US, 0);  // 100 Hz
ASensorEventQueue_registerSensor(queue, mag,   0, 0);             // native rate (~50–100 Hz)
ASensorEventQueue_registerSensor(queue, baro,  0, 0);             // native rate (~25 Hz)
```

ملاحظة: الـ Android Sensor API يعامل `samplingPeriodUs` كـ **حد أعلى للمعدل** (يعني: لا تسلّمني أسرع من هذا). الـ HW قد يُسلّم أبطأ إذا كان معدله الأصلي أقل، لكن لن يُسلّم أسرع. على هواتف اليوم، 10 ms يقابل 100 Hz بدقة.

**ماذا (android_uorb_publishers.cpp):**

قبل:
```cpp
usleep(2500);  // ~400 Hz polling rate
```

بعد:
```cpp
usleep(10000);  // ~100 Hz polling rate (matches IMU)
```

لماذا 100 Hz وليس 200 Hz (كان مقترحاً مبدئياً):

- **Nyquist لا ينطبق هنا:** هذا producer/consumer drain، وليس signal sampling. حجة "Nyquist headroom" للـ polling rate خاطئة مفاهيمياً.
- **`timestamp_sample` مُلتقَط في `native_sensor_reader` عند وصول الـ event** (السطر 82 و 101 في `native_sensor_reader.cpp`)، وليس عند النشر. ولذا EKF يرى الزمن الفعلي للعيّنة بدقّة، بغضّ النظر عن latency النشر.
- **`EKF2_DELAY_MAX = 200 ms`** — أي 0–10 ms من publish jitter لا تأثير له بتاتاً.
- **خطر فقدان عيّنة:** الـ `has_new_data` flag بدون buffer (mailbox بفتحة واحدة)، فإذا حدثت preemption > 10 ms قد تُفقَد عيّنة. لكن خيط `native_sensor_reader` يعمل بأولوية SCHED `-19` (`URGENT_AUDIO`) — تذبذبات بهذا الحجم نادرة جداً، و EKF يتحمّل خسارة عيّنة منعزلة بسهولة.

النتيجة: 100 Hz polling = مساوٍ لـ 200 Hz رياضياً مع نصف الـ CPU. والمستخدم محقّ في قراءته للأمر.

**ماذا (px4_jni.cpp):**

داخل كتلة "rc.rocket_defaults — المعاملات المشتركة" (السطر ~271):

قبل:
```cpp
// -- EKF2 معدل التنبؤ (200 Hz للصاروخ) --
p = param_find("EKF2_PREDICT_US");
if (p != PARAM_INVALID) { int32_t v = 5000; param_set(p, &v); }
```

بعد:
```cpp
// -- EKF2 معدل التنبؤ (100 Hz للصاروخ) --
p = param_find("EKF2_PREDICT_US");
if (p != PARAM_INVALID) { int32_t v = 10000; param_set(p, &v); }

// -- IMU integration rate (must match EKF2_PREDICT_US) --
p = param_find("IMU_INTEG_RATE");
if (p != PARAM_INVALID) { int32_t v = 100; param_set(p, &v); }

// -- Inner-loop gyro publication rate --
p = param_find("IMU_GYRO_RATEMAX");
if (p != PARAM_INVALID) { int32_t v = 100; param_set(p, &v); }
```

**الأثر المتوقع:**

| البند | التفصيل |
|---|---|
| CPU event handling | خفض ~75% من معدل event handling في `native_sensor_reader` (من 400 إلى 100 Hz) |
| CPU polling | خفض ~50% من معدل polling في `publisher_loop` (من 400 إلى 200 Hz) |
| EKF predict | خفض ~50% من خطوات EKF predict (من 200 إلى 100 Hz) |
| VehicleIMU | خفض ~75% من خطوات integration |
| الحمل الإجمالي | ~5–10% انخفاض في حمل CPU على الهاتف — يفيد MPC solver (كان يضرب deadline misses في HITL) |
| الدقة | لا تغيّر فعلي: HAL يطبّق LPF مدمج عند downsampling، و `IMU_GYRO_CUTOFF` افتراضياً 40 Hz — مع IMU عند 100 Hz الـ Nyquist هو 50 Hz، فلا aliasing |
| MPC | لا يتأثر مباشرة (25 Hz). الأثر الإيجابي غير المباشر: تقليل تنافس threads ⇒ تقليل p95 لزمن حل MPC |

**ما لم يُعدَّل — ROMFS `rc.rocket_defaults`:**

`PX4-Autopilot/ROMFS/px4fmu_common/init.d/rc.rocket_defaults` يحتوي `param set EKF2_PREDICT_US 5000`. هذا الملف **لا يُحمَّل على Android** — الكتلة في `px4_jni.cpp:271` هي البديل. لو أراد المشروع تشغيل بنية PX4 الأصلية على HW حقيقي مستقبلاً، يجب تحديث هذا الملف ليتطابق.

**اختبارات يجب إجراؤها بعد البناء:**

1. Boot log — تأكد من ظهور: `NativeSensor: Accel registered @100Hz` / `Gyro registered @100Hz`
2. Rate log (كل 5 ثوانٍ): `NativeSensor: RATES: IMU=100/s  Baro=≤25/s  Mag=50/s` — يجب أن يكون IMU ≈ 100 (±5 jitter طبيعي)
3. EKF2 status: `listener vehicle_imu_status` — تأكد أن `gyro_rate_hz` و `accel_rate_hz` ≈ 100
4. Logger — افتح `.ulg` في PlotJuggler وارسم `vehicle_imu/gyro_rad`؛ يجب أن يكون التباعد ~10 ms ثابتاً
5. HITL deadline check — أعِد تشغيل أحد سيناريوهات HITL وتأكد أن `mpc_solve` p95 لم يرتفع (يفترض ينخفض)

---

## 11. عدم تطابق MHE `horizon_dt` (موثّق — لم يُصلَح)

> هذا القسم توثيقي فقط. المشكلة قائمة منذ ما قبل جلسة الترحيل وليست من نطاقها.

**الملف:** `PX4-Autopilot/src/modules/rocket_mpc/RocketMPC.cpp` (~سطر 312–316)

**الوضع الحالي:**
```cpp
mhe_cfg.horizon_steps = MHE_N;
mhe_cfg.horizon_dt    = 0.02f;
mhe_cfg.solve_rate_hz = 50.0f;
```

**المشكلة:**
- `solve_rate_hz = 50` لا يعمل عملياً — نقطة الاستدعاء (`_mhe.push_measurement` و `_mhe.update`) محدودة بحلقة MPC:
```cpp
const bool do_mpc_this_cycle = (now - _last_mpc_solve_time >= 39_ms);  // ≈ 25 Hz
```
- الحدّ الأبطأ (الـ caller @ 25 Hz) هو السائد.
- `horizon_dt = 0.02 s` (20 ms) مخبوز داخل الـ acados solver المُولَّد في `c_generated_code/m130_mhe_model/`.

**عدم التطابق الجوهري:**
- المراحل في الـ acados-NLP تفترض تباعد 20 ms، لكن الـ measurements تُدفع كل 40 ms.
- الـ solver "يرى" زمن النافذة مضاعفاً (20 stages × 20 ms = 400 ms model time)، بينما الزمن الحقيقي للـ buffer هو (20 stages × 40 ms = 800 ms).
- هذا خطأ تقدير جوهري في الديناميكا.

**لإصلاحه لاحقاً:** إعادة توليد الـ MHE acados solver بـ `horizon_dt = 0.04` (يحتاج تعديل سكريبت Python في `m130_mhe_ocp.json` وإعادة تشغيل `acados_ocp_solver`).

---

## 12. تدقيق البارميترات العميق (2026-05-01)

التعديلات التالية أُضيفت بعد فحص شامل لمطابقة بارميترات `px4_jni.cpp` مع ROMFS `rc.rocket_defaults` و `22005_m130_rocket_mpc_real`. التفاصيل الكاملة كانت في `DEEP_AUDIT_PARAMS_ARMING_2026_05_01.md`.

### 12.1 إضافة `EKF2_REQ_EPH` و `EKF2_REQ_EPV` (كتلة دائمة)

**الملف:** `AndroidApp/app/src/main/cpp/px4_jni.cpp` (~سطر 640)

**المشكلة:** PX4 default (3.0/5.0 م) صارم جداً — GPS عادي يُبلغ EPH 3-8 م. مع `EKF2_GPS_CHECK=1037` (bit 2 = EPH، bit 3 = EPV) في وضع 22005 (الطيران الحقيقي)، إذا أبلغ GPS عن EPH > 3.0 م ← EKF2 يرفض GPS ← "Preflight Fail: Global position estimate required" رغم 13+ قمر.

**ماذا:**
```cpp
p = param_find("EKF2_REQ_EPH");
if (p != PARAM_INVALID) { float v = 10.0f; param_set(p, &v); }
p = param_find("EKF2_REQ_EPV");
if (p != PARAM_INVALID) { float v = 15.0f; param_set(p, &v); }
```

ملاحظة: نفس القيم موجودة أيضاً في الكتلة المشتركة (سطر ~342) — الكتلة الدائمة تضمن التطبيق حتى لو القيم القديمة المحفوظة أعادت الضبط الافتراضي.

### 12.2 تخفيض `EKF2_MAG_ACCLIM` من 30.0 إلى 5.0

**الملف:** `AndroidApp/app/src/main/cpp/px4_jni.cpp` (~سطر 525، فرع 22005 Real flight)

**المشكلة:** القيمة 30.0 تتجاوز الحدّ الأقصى الموثّق في PX4 YAML (`max: 5.0`). رغم أن `param_set()` لا يرفضها، إلا أن أي تحديث مستقبلي لـ PX4 قد يُلزم الحدود ويقيّد القيمة بصمت.

قبل:
```cpp
p = param_find("EKF2_MAG_ACCLIM");
if (p != PARAM_INVALID) { float v = 30.0f; param_set(p, &v); }
```

بعد:
```cpp
p = param_find("EKF2_MAG_ACCLIM");
if (p != PARAM_INVALID) { float v = 5.0f; param_set(p, &v); }
```

**لا تأثير وظيفي:** لأن `EKF2_MAG_TYPE=1` (HEADING) يدمج yaw مباشرة بدون اعتماد على هذا العلم، ثم يتحوّل إلى `5` (NONE) عند الإطلاق.

### 12.3 نشر `x_mpc[18]` عبر `DEBUG_FLOAT_ARRAY` للـ tlog

**المشكلة:** مصفوفة حالة MPC الـ 18 (`x_mpc[]`) كانت تُسجَّل في ulog فقط عبر `rocket_gnc_status.x_mpc[]`، لكنها لا تُرسل عبر MAVLink ← لا تظهر في tlog.

**الملفات المُعدَّلة:**

1. **`RocketMPC.hpp`** — سطر 47-48, 147:
   - أضيف `#include <uORB/PublicationMulti.hpp>`
   - أضيف `uORB::PublicationMulti<debug_array_s> _debug_array_pub{ORB_ID(debug_array)};`
   - استخدم `PublicationMulti` (وليس `Publication`) لتخصيص instance منفصل عن SRV_FB (id=1) المنشور من xqpower_can على instance 0

2. **`RocketMPC.cpp`** — سطر 2119-2129:
```cpp
// ---- Publish x_mpc[18] via DEBUG_FLOAT_ARRAY for tlog telemetry ----
{
    debug_array_s dbg{};
    dbg.timestamp = now;
    dbg.id = 2;
    strncpy(dbg.name, "MPC_X", sizeof(dbg.name));
    for (int i = 0; i < 18 && i < (int)debug_array_s::ARRAY_SIZE; i++) {
        dbg.data[i] = _have_x_mpc ? _last_x_mpc[i] : 0.0f;
    }
    _debug_array_pub.publish(dbg);
}
```

**تفاصيل الحالة المنشورة (`data[0..17]`):**

| Index | اسم الحالة | الوحدة |
|---|---|---|
| 0 | φ (roll) | rad |
| 1 | θ (pitch) | rad |
| 2 | ψ (yaw) | rad |
| 3 | p (roll rate) | rad/s |
| 4 | q (pitch rate) | rad/s |
| 5 | r (yaw rate) | rad/s |
| 6 | Vx (downrange velocity) | m/s |
| 7 | Vy (crossrange velocity) | m/s |
| 8 | Vz (vertical velocity) | m/s |
| 9 | x (downrange position) | m/1000 (X_SCALE) |
| 10 | y (crossrange position) | m/1000 (Y_SCALE) |
| 11 | h (altitude) | m/100 (H_SCALE) |
| 12 | δe_s (elevator servo state) | rad |
| 13 | δr_s (rudder servo state) | rad |
| 14 | δa_s (aileron servo state) | rad |
| 15 | δe_act (elevator actual) | rad |
| 16 | δr_act (rudder actual) | rad |
| 17 | δa_act (aileron actual) | rad |

ملاحظات:
- `id=2, name="MPC_X"` — لا يتعارض مع SRV_FB (id=1) من xqpower_can
- يُنشر كل دورة Run() (~100 Hz) — MAVLink يُرسله حسب معدل البث المُهيَّأ
- القيم تكون صفراً قبل أول حلّ MPC (`_have_x_mpc=false`)

### 12.4 إضافة `EKF2_REQ_GPS_H = 1.0` (كتلة دائمة)

**الملف:** `AndroidApp/app/src/main/cpp/px4_jni.cpp` (~سطر 648)

**المشكلة:** PX4 default = 10.0 ثانية — يتطلب 10 ثوانٍ متواصلة من GPS صحّي قبل بدء دمجه في EKF2. مع وقت تهيئة EKF2 (~5-10 ثوانٍ)، النظام يحتاج ~25 ثانية للتقارب ← "Preflight Fail: ekf2 missing data" أثناء الانتظار.

**ماذا:**
```cpp
p = param_find("EKF2_REQ_GPS_H");
if (p != PARAM_INVALID) { float v = 1.0f; param_set(p, &v); }
```

### 12.5 إضافة `COM_CPU_MAX = -1` و `COM_RAM_MAX = -1` (كتلة دائمة)

**الملف:** `AndroidApp/app/src/main/cpp/px4_jni.cpp` (~سطر 656)

**المشكلة:** `load_mon` يعمل على work queue `lp_default` — على Android قد لا يُجدوَل بانتظام (`ScheduledWorkItem` posix) فيبقى `cpuload` topic متأخراً > 2s ← "Preflight Fail: No CPU and RAM load information".

**ماذا:** تعطيل فحص CPU/RAM بدل منع التسليح بسبب مشكلة جدولة:
```cpp
p = param_find("COM_CPU_MAX");
if (p != PARAM_INVALID) { float v = -1.0f; param_set(p, &v); }
p = param_find("COM_RAM_MAX");
if (p != PARAM_INVALID) { float v = -1.0f; param_set(p, &v); }
```

### 12.6 حذف `EKF2_GPS_V_NOISE` و `EKF2_GPS_P_NOISE`

**الملف:** `AndroidApp/app/src/main/cpp/px4_jni.cpp` (~سطر 299-302)

**المشكلة:** كان الكود يضبط `EKF2_GPS_V_NOISE = 1.5` (افتراضي PX4: 0.5) و `EKF2_GPS_P_NOISE = 2.0` (افتراضي PX4: 1.5) في الكتلة المشتركة. هذه القيم المرتفعة تُبطئ تقارب EKF لأنها تقلل ثقة الفلتر بـ GPS.

**ماذا:** حُذفت من الكتلة المشتركة — الآن تستخدم الافتراضي PX4 أو يُتحكَّم بها من QGroundControl.

### 12.7 حذف `EKF2_MAG_TYPE` من كل كتل الـ airframe

**الملف:** `AndroidApp/app/src/main/cpp/px4_jni.cpp` (~سطر 508-522, 548, 571)

**المشكلة:** كان الكود يضبط `EKF2_MAG_TYPE = 1` (HEADING) للطيران الحقيقي و `5` (NONE) لـ HITL/SITL. مغناطيس الهاتف قريب من الشاشة/البطارية/معدن الصاروخ ← heading innovation عالي دائماً ← "X/Y position control Error".

**ماذا:** حُذف ضبط `EKF2_MAG_TYPE` من الكتل الثلاث (22005, HITL, SITL) — الآن يستخدم الافتراضي PX4 (`0 = AUTO`) أو يُتحكَّم به من QGroundControl. هذا يسمح بتجربة أوضاع مختلفة (0=AUTO, 1=HEADING, 5=NONE, 6=INIT) بدون إعادة بناء.

### 12.8 تغيير قيمة reset لـ `EKF2_MAG_TYPE` من 1 إلى 0

**الملف:** `PX4-Autopilot/src/modules/rocket_mpc/RocketMPC.cpp` (~سطر 496، في `_reset_flight_state()`)

قبل:
```cpp
int32_t v = 1;  // HEADING
param_set_no_notification(mt, &v);
```

بعد:
```cpp
int32_t v = 0;  // AUTO
param_set_no_notification(mt, &v);
```

**لماذا:** متوافق مع حذف `EKF2_MAG_TYPE` من `px4_jni.cpp` (القسم 12.7). عند arm/disarm يُستعاد إلى `0 (AUTO)` بدل `1 (HEADING)`. كذلك عُدّل تعليق كشف الإطلاق ليذكر AUTO بدل HEADING.

### 12.9 تصحيح `EKF2_ABL_LIM` من 1.0 إلى 0.8

**الملف:** `AndroidApp/app/src/main/cpp/px4_jni.cpp` (~سطر 320)

**المشكلة:** القيمة 1.0 تتجاوز الحدّ الأقصى المعرّف في PX4 YAML (`max: 0.8`, ملف `params_accel_bias.yaml`). رغم أن `param_set()` لا يرفضها، PX4 يقصّها داخلياً إلى 0.8.

قبل:
```cpp
p = param_find("EKF2_ABL_LIM");
if (p != PARAM_INVALID) { float v = 1.0f; param_set(p, &v); }
```

بعد:
```cpp
p = param_find("EKF2_ABL_LIM");
if (p != PARAM_INVALID) { float v = 0.8f; param_set(p, &v); }
```

---

## 13. XqpowerCan 200→100 Hz

**الملف:** `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/drivers/xqpower_can/XqpowerCan.cpp` (~سطر 130)

**ماذا:**
```cpp
// قبل:
ScheduleOnInterval(5000);   // 200 Hz

// بعد:
ScheduleOnInterval(10000);  // 100 Hz
```

**لماذا:** `actuator_outputs_sim` يُنشر بمعدل 100 Hz من `RocketMPC::Run()`. تشغيل الدرايفر بـ 200 Hz يعني نصف الدورات لا تجد بيانات جديدة (`_actuator_outputs_sim_sub.update()` يعيد false). توحيد المعدل عند 100 Hz يوفّر CPU للـ MPC solver ويزيل الدورات الفارغة. الفرق في worst-case uORB→CAN latency (5ms→10ms) ضئيل مقارنة بالـ ~120 ms المقاس في HIL pure-delay.

---

## 14. ضبط بارميترات EKF2 — المغناطيس و GPS (v1.md §16A)

**الملف:** `px4_jni.cpp` — الكتلة المشتركة (`first_rocket_run || airframe_changed`)
**المرجع:** `v1.md` — القسم 16A — بارميترات تحتاج تعديل (أرقام 2–6)

> **اكتشاف مهم:** ملف `rc.rocket_defaults` (ROMFS) **لا يُستخدم في بناء Android**.
> CMakeLists.txt لا يشير إلى ROMFS إطلاقاً. المصدر الوحيد للبارميترات هو `px4_jni.cpp`.
>
> **مشكلة التخزين الخارجي:** ملف البارميترات المحفوظ يقع في:
> `/storage/emulated/0/Android/data/com.ardophone.px4v17/files/px4/eeprom/parameters`
> هذا المسار على **التخزين الخارجي** — **لا يُحذف عند إزالة التطبيق** (سلوك Android 11+).
> القيم القديمة تبقى → `param_load_default()` يقرأها → `SYS_AUTOSTART=22004` →
> `first_rocket_run=false` → الكتلة المشتركة لا تُنفَّذ → القيم القديمة تستمر.
> **الحل:** حذف `eeprom/parameters` يدوياً عند الحاجة لإعادة ضبط كامل.

### التعديلات المُطبَّقة (5 بارميترات):

| # | البارميتر | القيمة السابقة | القيمة الجديدة | أين يُستخدم في EKF2 | لماذا هذه القيمة؟ |
|:-:|---|:-:|:-:|---|---|
| 2 | `EKF2_HDG_GATE` | 2.6 (افتراضي) | **10.0 SD** | `yaw_fusion.cpp` — بوابة تصفية innovation الـ heading | الافتراضي 2.6 SD يرفض قراءات مغناطيس الهاتف الضوضائية → `pre_flt_fail_innov_heading`. التوسيع يسمح بالمرور |
| 3 | `EKF2_HEAD_NOISE` | 0.3 rad (17°) | **0.7 rad (40°)** | `mag_control.cpp` — فحص `mag_heading_consistent` + تباين yaw عند reset | 0.3 rad ضيق → المغناطيس يُعتبر غير متسق باستمرار. 0.7 rad يجعله يمر + يقلل ثقة EKF بالمغناطيس (يعتمد على الجايرو أكثر) |
| 4 | `EKF2_GPS_DELAY` | 200 ms | **110 ms** | تعويض تأخير GPS مقارنة بـ IMU | 200 ms مبالغ فيه لـ USB u-blox (~100 ms). الفرق يسبب عدم محاذاة IMU/GPS |
| 5 | `EKF2_GPS_P_NOISE` | 2.0 m | **1.0 m** | تباين قياس موقع GPS في Kalman filter | 2.0 m يجعل EKF لا يثق بالموقع كفاية → تقارب بطيء. 1.0 m واقعي أكثر |
| 6 | `EKF2_GPS_V_NOISE` | 1.5 m/s | **0.5 m/s** | تباين قياس سرعة GPS في Kalman filter | 1.5 m/s مبالغ فيه. 0.5 m/s (الافتراضي PX4) مناسب لـ GPS USB |

### سلوك المغناطيس الديناميكي (لم يُعدَّل):

| المرحلة | `EKF2_MAG_TYPE` | التأثير | الكود |
|---|:-:|---|---|
| **ARM / على المنصة** | 0 (AUTO) | المغناطيس يعمل — EKF يتقارب على heading. `HDG_GATE` و `HEAD_NOISE` مؤثران هنا | `RocketMPC::_reset_flight_state()` سطر 494 |
| **كشف الإطلاق** | 5 (NONE) | المغناطيس يتوقف — yaw يستمر من الجايرو فقط. `HDG_GATE` و `HEAD_NOISE` **لا تأثير** | `RocketMPC::Run()` سطر 1017 |
| **DISARM** | 0 (AUTO) | يعود للمغناطيس للدورة التالية | `RocketMPC::_reset_flight_state()` سطر 494 |

---

## 15. إصلاح تأخر ظهور تقدير الحالة (attitude) بعد Start

**الملف:** `px4_jni.cpp` — الكتلة المشتركة (`first_rocket_run || airframe_changed`)

### المشكلة

بعد الضغط على Start في الفريم 22005، الـ roll/pitch/yaw لا تظهر إلا بعد فترة طويلة.

### التشخيص

سلسلة التبعيات التي تمنع ظهور الـ attitude:

```
attitude_valid() = tilt_align
    ← getTiltVariance() < sq(3°) = 0.00274
        ← التباين الابتدائي = sq(EKF2_ANGERR_INIT) = sq(0.1) = 0.01 > 0.00274 ❌
            ← يحتاج ZeroVelocityUpdate لتقليل التباين
                ← يحتاج vehicle_at_rest = true
                    ← يحتاج 1 ثانية بدون اهتزاز (gyro_vibe < 0.02)
                        ← مستشعرات الهاتف ضوضائية → at_rest = false لفترة طويلة ❌
```

**الملفات المعنية:**
- `estimator_interface.h:191` — `attitude_valid()` يرجع `tilt_align`
- `control.cpp:72-77` — حد tilt_align = `sq(radians(3°))` = 0.00274
- `covariance.cpp:56,316` — تباين ابتدائي = `sq(EKF2_ANGERR_INIT)`
- `LandDetector.cpp:246-256` — `at_rest` يحتاج `gyro_vibe < 0.02` لمدة 1 ثانية
- `ZeroVelocityUpdate.cpp:54-57` — يحتاج `at_rest` لتشغيل ZVU

### الحل

| البارميتر | القيمة السابقة | القيمة الجديدة | التأثير |
|---|:-:|:-:|---|
| `EKF2_ANGERR_INIT` | 0.1 rad (افتراضي) | **0.01 rad** | تباين ابتدائي = sq(0.01) = **0.0001** < 0.00274 → `tilt_align = true` فوراً |

### لماذا آمن؟

- `initialiseTilt()` يتحقق أن التسارع ضمن ±20% من الجاذبية وأن الجايرو < 15°/s
- محاذاة الميل من الجاذبية دقيقة إلى ~1° للهاتف الثابت
- **لا يؤثر على الطيران**: التباين يتقارب لحالة مستقرة (steady-state) خلال ثوانٍ بغض النظر عن القيمة الابتدائية — بحلول وقت ARM والإطلاق، القيمة الابتدائية مُنسَى

---

## جدول ملخّص

| # | الملف | التغيير | الأهمية |
|:-:|---|---|:-:|
| 1 | `px4_jni.cpp` + `RocketMPC.cpp` | `EKF2_MAG_TYPE` ديناميكي 1 ⇔ 5 | 🔴 حرجة |
| 2 | `RocketMPC.cpp` + `.hpp` | كشف الإطلاق بـ `ax > 1g` | 🟡 متوسطة |
| 3 | `m130_ocp_setup.py` | `N=40, tf=1.6, τ_transport=0.110` | 🔴 حرجة |
| 4 | `px4_parameters.hpp` (generated) | `ROCKET_MPC_TF = 1.6` | 🔴 حرجة |
| 5 | `CMakeLists.txt` | فحص N ديناميكي | 🟢 منخفضة |
| 6 | `sitl_analysis.py` | فلتر α/β | 🟠 عالية |
| 7 | `6dof_config_advanced.yaml` | `angular_velocity = [0,0,0]` | 🟢 منخفضة |
| 8 | `rocket_properties.yaml` | 6 قيم في `actuator` | 🟡 متوسطة |
| 9 | 6 ملفات متفرّقة | تنظيفية / تعليقات | ⚪ صفر |
| 10 | `native_sensor_reader` + `uorb_publishers` + `px4_jni` | IMU 400→100 Hz + EKF2 params | 🔴 حرجة |
| 11 | `RocketMPC.cpp` (توثيقي) | عدم تطابق MHE `horizon_dt` | 🟠 عالية |
| 12 | `px4_jni.cpp` + `RocketMPC.cpp/.hpp` | تدقيق بارميترات عميق (9 تعديلات) | 🔴 حرجة |
| 13 | `XqpowerCan.cpp` | 200→100 Hz (مطابقة لمعدل MPC) | 🟡 متوسطة |
| 14 | `px4_jni.cpp` | ضبط EKF2 mag/GPS (5 بارميترات من v1.md §16A) | 🔴 حرجة |
| 15 | `px4_jni.cpp` | `EKF2_ANGERR_INIT=0.01` — إصلاح تأخر ظهور attitude | 🟡 متوسطة |

---

## ملفات جديدة في المشروع

| الملف | الغرض |
|---|---|
| `CHANGELOG_M130.md` | هذا السجل |
| `SITL_COMMANDS_ORDERED.md` | ترتيب أوامر SITL/PIL/HITL |
| `ground_test_plan_mpc.md` | خطة الاختبار الأرضي |
| `IMU_100HZ_MIGRATION.md` | توثيق ترحيل IMU (دُمج في هذا السجل) |

---

## جدول الملفات المُعدَّلة (أقسام 10–13)

| الملف | السطر(الأسطر) | التغيير |
|---|---|---|
| `app/src/main/cpp/native_sensor_reader.cpp` | 200–241 | `samplingPeriodUs = 10 000` لـ accel و gyro |
| `app/src/main/cpp/android_uorb_publishers.cpp` | 232–247 | `usleep(2500)` → `usleep(10000)` (100 Hz polling) |
| `app/src/main/cpp/px4_jni.cpp` | 222–232 | تحديث تعليق HITL القديم من "~400 Hz" إلى "100 Hz" |
| `app/src/main/cpp/px4_jni.cpp` | 345–374 | `EKF2_PREDICT_US=10000` + `IMU_INTEG_RATE=100` + `IMU_GYRO_RATEMAX=100` |
| `app/src/main/cpp/px4_jni.cpp` | ~320 | `EKF2_ABL_LIM`: 1.0 → 0.8 |
| `app/src/main/cpp/px4_jni.cpp` | ~299-302 | حذف `EKF2_GPS_V_NOISE` و `EKF2_GPS_P_NOISE` |
| `app/src/main/cpp/px4_jni.cpp` | ~508-522, ~548, ~571 | حذف `EKF2_MAG_TYPE` من كل كتل airframe |
| `app/src/main/cpp/px4_jni.cpp` | ~640-660 | كتلة دائمة: `EKF2_REQ_EPH/EPV` + `EKF2_REQ_GPS_H` + `COM_CPU/RAM_MAX` |
| `PX4-Autopilot/.../RocketMPC.cpp` | ~496 | `EKF2_MAG_TYPE` reset: 1→0 في `_reset_flight_state()` |
| `PX4-Autopilot/.../RocketMPC.cpp` | ~1004 | تعليق كشف الإطلاق: HEADING→AUTO |
| `PX4-Autopilot/.../RocketMPC.hpp` | 47-48, 147 | `PublicationMulti<debug_array_s>` لنشر x_mpc |
| `PX4-Autopilot/.../RocketMPC.cpp` | 2119-2129 | نشر `x_mpc[18]` عبر `DEBUG_FLOAT_ARRAY` |
