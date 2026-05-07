# LESSONS_LEARNED.md
## دفتر دروس مشروع M130 — bugs سابقة وحلولها

> أي ذكاء اصطناعي **يقرأ هذا الملف قبل اقتراح أي حل**.
> grep هنا أولاً لتجنّب تكرار حلول سبق ورُفضت أو إعادة bugs سبق وحُلّت.

---

## صيغة الإدخال (إلزامية)

```markdown
## YYYY-MM-DD — <ملخص سطر واحد>
**Layer**: Python | SITL | PIL | HITL | Real
**Symptom**: <ما الذي لوحظ بالأرقام>
**Root cause**: <الملف:السطر + شرح>
**Fix**: <ما تغيّر — diff موجز>
**Why this fix (not workaround)**: <لماذا يحل الجذر>
**Verification**: <الأرقام التي أثبتت الحل>
**Regression test**: <اسم الاختبار أو N/A>
**Backup of pre-fix**: <مسار النسخة الاحتياطية>
**Author**: <user/AI session id>
```

---

## الدروس المسجّلة

<!-- أضف الإدخالات الجديدة هنا، الأحدث في الأعلى -->

## 2026-05-06 — PIL bridge: position_lla مفقود في warmup snapshot
**Layer**: PIL
**Symptom**: HIL_SENSOR شخّص `abs_p=3.2e11 Pa, alt=-1.79e6 m` (logs 20:35).
EKF2 يرفض baro → "Preflight Fail: No valid data from Baro 0" → no alignment.
**Root cause**: `pil/mavlink_bridge_pil.py:893` — يستدعي `_sensors()` بـ snapshot
بدون `position_lla`. في `long_range_mode=true`، `state[0:3]` يحوي ECEF (~6e6 m)
لا NED. `_sensors()` يقع في else branch ويُعالج ECEF كـ NED → قيم باروميتر مجنونة.
**Fix**: إضافة `"position_lla": (launch_lat, launch_lon, launch_alt)` للـsnapshot.
**Why this fix (not workaround)**: SITL bridge يتجاوز المشكلة بحساب baro مباشرة من
`self.launch_alt` (سطر 898-899). الإصلاح في PIL يجعله يسلك المسار الصحيح بدلاً
من تكرار حل SITL.
**Verification**: بعد الإصلاح: `abs_p=877.2 hPa, alt=1199.6 m` (logs 20:42).
**Regression test**: TODO — اختبار يفحص `abs_p ∈ [800, 1100] hPa` في warmup.
**Backup of pre-fix**: `pil/mavlink_bridge_pil.py.pre_gravity_fix`
**Author**: session 2026-05-05

## 2026-05-06 — Android publisher: skip_imu race يأخذ instance 0
**Layer**: PIL
**Symptom**: `accel0_inst=1` في logs HIL_SENSOR. `Preflight Fail: No valid data
from Accel 0`. simulator_mavlink يأخذ instance 1 لأن NativeSensor سبقه.
**Root cause**: `AndroidApp/.../android_uorb_publishers.cpp` — `publisher_loop`
يبدأ بـ `skip_imu=false`. `SYS_HITL` يُضبط لاحقاً (after thread start).
في تلك النافذة، `s_accel_pub.publish()` يستدعى → يحجز instance 0.
**Fix**: قراءة `SYS_AUTOSTART` مبكراً قبل أول publish. إن كان 22001/22004
→ `skip_imu = true` فوراً.
**Why this fix (not workaround)**: المشكلة race condition حقيقي. الحل يُغلق
النافذة الزمنية بدلاً من workaround في طبقة أعلى.
**Verification**: بعد الإصلاح: `accel0_inst=0`, Accel/Gyro `valid` (logs 20:54).
**Regression test**: TODO — اختبار يفحص أن أول HIL_SENSOR يُظهر `accel0_inst=0`.
**Backup of pre-fix**: `AndroidApp/.../android_uorb_publishers.cpp.pre_skip_imu`
**Author**: session 2026-05-05

## 2026-05-07 — HITL: lockstep=true يُسبّب slowdown 3-6× و non-determinism
**Layer**: HITL
**Symptom**:
  - 3 runs متتاليين بنفس الكود يُعطون نتائج متباينة جداً (score 25.0 / 58.8 / 68.5)
  - في run بصيغة سيّئة: missile tumbles α=178°
  - bridge يَستلم `Lockstep: 0 acks, N timeouts (100.0%)` دائماً
  - `wall=3-6× sim time` (مُحاكاة بطيئة جداً)
  - `actuator_msgs` متباين 59-186 (يَجب أن يكون ~800 في 8s @100Hz)
**Root cause**: `6DOF_v4_pure/hil/mavlink_bridge_hil.py:1394` — كل step يَنتظر
  `lockstep_timeout_ms` لرسالة `HIL_ACTUATOR_CONTROLS` جديدة من PX4. لكن PX4 يُرسل
  هذه الرسائل في **bursts** (TCP buffering / Nagle's algorithm على Android) لا
  تَتناغم مع نوافذ الـ steps. النتيجة: 100% timeout لكل step ⟹ كل step يَهدر
  20-50ms ⟹ المُحاكاة 3-6× أبطأ من realtime ⟹ MPC على بيانات قديمة ⟹ التحكم
  غير حتمي.
**Fix**: `hil_config.yaml:74` — `lockstep: false` (كان `true`).
  bridge يَعتمد على wall-clock pacing فقط (السيرفوهات الفيزيائية على CAN
  تُنظّم الإيقاع بطبيعتها).
**Why this fix (not workaround)**:
  - في HIL، السيرفوهات الفيزيائية تَعمل بـ wall-time ولا يُمكن إيقافها → wall-clock
    pacing إجباري بطبيعته. lockstep يُضيف انتظاراً عَقيماً فوقه.
  - رفع timeout 20→50ms لم يَحلّ (Run #3 بـ 50ms ما زال 100% timeouts) — يُؤكّد أن
    المشكلة ليست في طول النافذة بل بنيوية (TCP burst vs step-window mismatch).
  - Run #5 بعد الإصلاح: wall=sim (1.0×)، score 66.5، Peak Alt 61m (vs 6m قبلاً)،
    لا tumbling.
**Verification**:
  | Metric | قبل (lockstep=true) | بعد (lockstep=false) |
  |---|---|---|
  | wall/sim ratio | 3.1-6.2× | **1.0×** ✓ |
  | timeouts | 100% | N/A (disabled) |
  | actuator_msgs (8s) | 59-186 | 171-192 |
  | Peak Alt | 3-7m | **16-61m** |
  | Range | 32-797m | **1112-1241m** |
  | Score | 25-68 | 28-66 |
  | Tumbling? | yes في Run #2 | no |
**Regression test**: TODO — `tests/hil/test_lockstep_disabled.py` يَفحص أن
  `Lockstep: disabled` تَظهر في output و wall/sim ≤ 1.5×.
**Backup of pre-fix**: `6DOF_v4_pure/hil/hil_config.yaml.pre_lockstep_fix_1778138419`
**Author**: session 2026-05-07

## 2026-05-07 — `adb install -r` قد يَفشل صامتاً ⟹ APK لا يُستبدَل لساعات
**Layer**: HITL build/install workflow
**Symptom**: تَعديلات الكود ولا أثر لها رغم rebuild + install ناجحَين ظاهرياً.
  لساعات اعتُقد أن fixes فاشلة، فعُكِسَت — كانت كلها على نفس البناء القديم!
**Root cause**: `adb install -r` يَفشل أحياناً بدون رسالة واضحة (signature
  mismatch، storage full، أو session leak). الأمر يَخرج بـ 0 لكن APK لم يَتغيّر.
**Fix**:
  1. **دائماً** فحص `adb shell dumpsys package <pkg> | grep lastUpdateTime` بعد كل install.
  2. عند الشك: `adb uninstall <pkg> && adb install <apk>` بدلاً من `-r`.
  3. للتأكيد القاطع: `adb shell pm clear <pkg>` لمسح EEPROM المُستديم.
**Why this fix (not workaround)**: السبب الجذري قد يكون android storage أو
  package manager state. لا يُوجد إصلاح واحد، لكن **التحقق دائماً** من
  lastUpdateTime يَكشف الفشل قبل ضياع ساعات في تشخيص خاطئ.
**Verification**: في الجلسة، بعد `pm uninstall` + `adb install`، lastUpdateTime
  أَظهر الوقت الصحيح، وكل الـ fixes ظَهرت أثرها فوراً.
**Regression test**: لا يُمكن — workflow lesson، ليست code bug.
**Author**: session 2026-05-07

## 2026-05-07 — `_debug_array_pub` كل MPC cycle = MAVLink saturation في HITL
**Layer**: HITL
**Symptom**: تطبيق CHANGELOG #12.3 (نشر MPC state via debug_array كل cycle 100Hz)
  أَنزل lockstep من ~5% timeouts إلى 100%، score من ~67 إلى 54.9 ثم 25.0
  مع tumbling في run تالٍ.
**Root cause**: `RocketMPC.cpp` كان يَنشر `debug_array` في uORB كل MPC cycle
  (100 Hz). `mavlink/streams/DEBUG_ARRAY` يُحوّل ذلك تلقائياً إلى MAVLink
  رسائل `DEBUG_FLOAT_ARRAY` على القناة الرئيسية، فيُشبع bandwidth ويُؤخّر
  `HIL_ACTUATOR_CONTROLS`.
**Fix**: revert `_debug_array_pub` بالكامل من `RocketMPC.cpp` و `RocketMPC.hpp`.
  إن لزم MPC telemetry للتشخيص، يَجب rate-limit ≤ 10 Hz أو فصل القناة.
**Why this fix (not workaround)**: 100 Hz MAVLink stream على نفس القناة يَتنافس
  مع HIL_SENSOR/HIL_ACTUATOR_CONTROLS. الـ rate-limit إلى 10 Hz كافٍ لتشخيص
  MPC state ولا يُشبع القناة.
**Verification**: revert كامل أَعاد actuator_msgs من ~25 إلى ~170+ في 8s warmup.
**Regression test**: TODO — اختبار يَفحص MAVLink bandwidth في HITL ≤ 80%.
**Backup of pre-fix**: في صحيفة الجلسة (`SESSION_2026-05-07_*.md`)
**Author**: session 2026-05-07

## 2026-05-07 — قاعدة التطبيق: 4 شُروط قبل إعادة تَطبيق أيّ CHANGELOG item
**Layer**: governance / process
**Lesson**: في الجلسة، طبَّقت 8 تَعديلات من CHANGELOG_M130 معاً → regression حادّ
  (score 67 → 25 → tumbling). لاحقاً تَبَيَّن أن:
  - 6 من الـ 8 **no-op في HITL** (EKF2 مُعطَّل بـ `ROCKET_USE_GT=1`)
  - 1 (#10c) سَبَّب CPU contention
  - 1 (#12.3) سَبَّب MAVLink saturation
  - **لا واحد منها كان لازماً للـ baseline الحالي**
**Rule (مُتّفَق عليها مع المالك)**: أيّ CHANGELOG item يَعود فقط بعد:
  1. **سبب مَلموس بالأرقام**: log/metric يُثبت المشكلة (لا تَطبيق "احترازي")
  2. **A/B test مُسجَّل**: قبل/بعد على نفس الـ baseline
  3. **revert فوري إن ساء أيّ metric**
  4. **single change at a time**: لا دَمج تَعديلَين في run واحد
**Author**: session 2026-05-07

<!-- نهاية الإدخالات -->
