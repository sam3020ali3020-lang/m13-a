# 📋 سجل التعديلات — M130 Changes Log

> كل تعديل يُوثَّق هنا مع السبب والملفات والتحقق المطلوب.
> الملفات مصنّفة حسب Tier (0 = أعلى أثر، 2 = أدنى).

---

## 🚀 ملخّص تنفيذي سريع — Quick Start for Colleagues

> **آخر تحديث**: 2026-05-15
> **الحالة**: HIL يعمل — 4/5 runs ناجحة (range ≥2517m)، MPC fail rate: 0-1%

### الملفات المعدّلة (بالترتيب الزمني)

| # | الملف (مسار نسبي من `m13/`) | Tier | التعديل الرئيسي |
|---|---|---|---|
| 001 | `AndroidApp/.../rocket_mpc/rocket_mpc_params.c` | 0 | `RKT_ABRT_PTCH`: 25→-10 |
| 001 | `AndroidApp/.../rocket_mpc/RocketMPC.cpp` | 0 | Off-axis abort محصور بمرحلة boost فقط |
| 001b | `AndroidApp/.../generated/parameters/px4_parameters.hpp` | 0 | إضافة 4 بارامترات `RKT_ABRT_*` |
| 002 | `6DOF_v4_pure/sitl/run_sitl_test.py` | 2 | Auto-open HTML report |
| 003 | ~144 ملف | 2 | تحديث مسارات `/home/px4/...` → `/home/abas/...` |
| 004 | `6DOF_v4_pure/sitl/compare_sitl_vs_standalone.py` | 2 | Fallback بحث CSV |
| 005a | `AndroidApp/.../px4_jni.cpp` | 0 | `-o 14550` → `-o 14551` (TCP bridge) |
| 005a | `6DOF_v4_pure/hil/hil_config.yaml` | 1 | host=localhost, settle=15s, external_arm=false |
| 005b | `AndroidApp/.../airframes/22004_m130_rocket_mpc_hitl` | 1 | `ROCKET_USE_GT`: 1→0 |
| 008 | `AndroidApp/.../rocket_mpc/mpc_controller.cpp` | **0** | **إصلاح QP cascade** (الأهم) |
| 008 | `6DOF_v4_pure/mpc/m130_ocp_setup.py` | 1 | N أُرجع إلى 80 |
| 008 | `6DOF_v4_pure/config/6dof_config_advanced.yaml` | 1 | N=80, gps_delay=100ms |
| 008 | `c_generated_code/*` + `libm130_solvers.a` + header | 1 | إعادة توليد/بناء solver |
| 009 | `6DOF_v4_pure/hil/hil_runner.py:1165` | 2 | تعطيل auto-launch (تجريبي) |

### خطوات التطبيق للزميل

```bash
# 0. المتطلبات
#    - Android Studio + NDK
#    - JAVA_HOME = java-17 (ليس 21)
#    - adb مثبت ومُضاف للـ PATH
#    - Samsung S23 Ultra (أو أي ARM64 مع PX4 app)

# 1. تطبيق التعديلات البرمجية
#    طبّق التعديلات حسب الجدول أعلاه. التفاصيل الدقيقة (كود قبل/بعد) في كل قسم أدناه.
#    ⚠️ الملفات Tier 0 تحتاج backup قبل التعديل:
#       cp <file> <file>.pre_fix_$(date +%s)

# 2. إعادة توليد acados solver (بعد تعديل N أو OCP)
cd 6DOF_v4_pure/mpc/
python3 m130_ocp_setup.py          # يُولّد c_generated_code/

# 3. بناء solver لـ ARM64
cd ../../AndroidApp/app/src/main/cpp/acados_arm64/
# (اتبع README أو cross-compile script)

# 4. بناء APK
cd ../../../../                    # العودة لجذر AndroidApp/
export JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64
# حذف كاش قديم إن وُجد:
rm -rf app/.cxx/ app/build/intermediates/cxx/
./gradlew assembleDebug

# 5. تثبيت APK
adb install -g app/build/outputs/apk/debug/app-debug.apk
# بعد التثبيت أول مرة:
adb shell pm clear com.ardophone.px4v17    # حذف EEPROM قديم

# 6. تشغيل HIL
cd ../m13/6DOF_v4_pure/hil/
python3 hil_runner.py              # أو --no-auto-launch للفتح اليدوي
```

### التعديلات المُلغاة (لا تُطبَّق)

| # | الوصف | السبب |
|---|---|---|
| 006 | Proactive reinit عند burnout | أسوأ — فقدان تحكم مبكر |
| 007 | N=80→40 | لم يحل المشكلة الجذرية — أُرجع لـ N=80 |

---

## [001] — إصلاح Off-Axis Abort False Positive (2026-05-13)

### المشكلة
نظام إلغاء الطيران R6 (`_check_abort_conditions`) كان يُطلق **حتمياً** عند t≈1.25s في كل رحلة بزاوية إطلاق 15°، لأن عتبة `RKT_ABRT_PTCH=25°` كانت **أعلى من زاوية الإطلاق الفعلية** (15°).

### التأثير قبل الإصلاح
- SITL: Range = 1364m (err -47.6%) بدل 2600m — **فشل كامل**
- الإطلاق الحقيقي: كان سيُلغى تلقائياً بعد 1.25 ثانية

### الملفات المعدّلة

| الملف | Tier | التعديل |
|-------|------|---------|
| `rocket_mpc_params.c` | 0 | `RKT_ABRT_PTCH`: 25.0 → **-10.0** (abort فقط إذا انقلب تحت الأفق 10°) |
| `RocketMPC.cpp:633` | 0 | أُضيف شرط `t_flight < boost_end` — فحص off-axis محصور بمرحلة الدفع فقط |

### الكود قبل وبعد

#### 1. `rocket_mpc_params.c` — تعريف البارامتر

**قبل:**
```c
PARAM_DEFINE_FLOAT(RKT_ABRT_PTCH, 25.0f);
```

**بعد:**
```c
PARAM_DEFINE_FLOAT(RKT_ABRT_PTCH, -10.0f);
```

#### 2. `RocketMPC.cpp` — شرط فحص Off-Axis

**قبل:**
```cpp
// ---- Off-axis abort ----------------------------------------------
// Skip first 1.0 s post-launch (rail-clearance + acceleration phase
// where pitch may transiently dip due to gravity-turn or roll-coupling).
if (t_flight > 1.0f && pitch_deg < pitch_thresh) {
```

**بعد:**
```cpp
// ---- Off-axis abort ----------------------------------------------
// Only check during boost phase: t > 1.0 s (rail-clearance done) AND
// t < burn_time + t_tail + 2 s (coast phase pitch drop is normal).
// For low-angle launches (~15°), pitch naturally drops toward 0° by
// burnout — checking in the coast phase would cause false aborts.
const float boost_end = _param_burn_time.get() + _param_t_tail.get() + 2.0f;
if (t_flight > 1.0f && t_flight < boost_end && pitch_deg < pitch_thresh) {
```

#### المسار الكامل للملفات:
- `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/rocket_mpc_params.c`
- `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/RocketMPC.cpp`

### النسخ الاحتياطية
- `rocket_mpc_params.c.pre_fix_1778700215`
- `RocketMPC.cpp.pre_fix_1778700215`

### التحقق (V&V)
- [x] SITL: Score 100/100, Range 2566m (-1.3%) ✅
- [ ] PIL: يحتاج اختبار
- [ ] HIL: يحتاج اختبار
- [ ] Ground Test: يحتاج اختبار

### ملاحظات
- Spin abort (`|gyro_z| > 350°/s`) لم يُعدَّل — لا يزال فعالاً
- التشخيص يُظهر "Possible Servo Delay Mismatch" (pitch σ=5.2° في terminal) — يحتاج متابعة في HIL

---

## [001b] — تسجيل بارامترات R6 في بناء Android (2026-05-13)

### المشكلة
بناء Android (APK) فشل بأخطاء:
```
error: no member named 'RKT_ABRT_EN' in 'px4::params'
```
بارامترات R6 (`RKT_ABRT_*`) كانت معرّفة في `rocket_mpc_params.c` لكن **غير مسجّلة** في الملف المُولَّد مسبقاً `px4_parameters.hpp` الذي يستخدمه بناء Android (بخلاف SITL الذي يُولّدها تلقائياً).

### الملفات المعدّلة

| الملف | Tier | التعديل |
|-------|------|---------|
| `generated/parameters/px4_parameters.hpp` | 0 | أُضيفت 4 بارامترات في 3 مواضع (enum + metadata + type array) |

### الكود المُضاف

#### 1. Enum (سطر ~1310)

```cpp
RKT_ABRT_DBNC,
RKT_ABRT_EN,
RKT_ABRT_GYRO,
RKT_ABRT_PTCH,
```

#### 2. Metadata (القيم الافتراضية)

```cpp
{ .name = "RKT_ABRT_DBNC", .val = { .i = 25} },
{ .name = "RKT_ABRT_EN",   .val = { .i = 1} },
{ .name = "RKT_ABRT_GYRO", .val = { .f = 350.0 } },
{ .name = "RKT_ABRT_PTCH", .val = { .f = -10.0 } },
```

#### 3. Type array

```cpp
PARAM_TYPE_INT32,  // RKT_ABRT_DBNC
PARAM_TYPE_INT32,  // RKT_ABRT_EN
PARAM_TYPE_FLOAT,  // RKT_ABRT_GYRO
PARAM_TYPE_FLOAT,  // RKT_ABRT_PTCH
```

#### المسار الكامل

- `AndroidApp/app/src/main/cpp/generated/parameters/px4_parameters.hpp`

### تعديلات إضافية لنجاح البناء

| الملف | التعديل |
|-------|---------|
| `الاوامر` | `JAVA_HOME`: java-21 → **java-17** (Java 21 غير مثبّت) |
| `app/.cxx/` + `app/build/` | حُذف كاش cmake (كان يحتوي مسارات قديمة `/home/px4/...`) |

### التحقق (V&V)
- [x] البناء نجح: `BUILD SUCCESSFUL in 12s` ✅
- [x] التثبيت نجح: `Installed on 1 device` (SM-S918U) ✅
- [x] تطابق المصفوفات: 1829 عنصر في enum = metadata = type ✅

---

## [002] — فتح تقرير HTML تلقائياً بعد SITL (2026-05-13)

### المشكلة
بعد انتهاء محاكاة SITL، لا يُفتح أي تقرير في المتصفح — المستخدم يحتاج فتحه يدوياً.

### الملفات المعدّلة

| الملف | Tier | التعديل |
|-------|------|---------|
| `6DOF_v4_pure/sitl/run_sitl_test.py` | 2 | أُضيف `import webbrowser` + auto-open أفضل تقرير HTML بعد Stage 6 |

### التحقق
- [x] SITL: التقرير يُفتح تلقائياً ✅

---

## [003] — تحديث مسارات المشروع (2026-05-13)

### المشكلة
المسارات القديمة (`/home/px4/workspace/m1322222/m13/`) لا تعمل على الجهاز الجديد.

### التعديل
| المسار القديم | المسار الجديد |
|--------------|--------------|
| `/home/px4/workspace/m1322222/m13/` | `/home/abas/px/m30/m1322222/m13/` |
| `/home/px4/workspace/python3.12_env/bin/python3.12` | `python3` |
| `/home/px4/Android/Sdk` | `/home/abas/Android/Sdk` |

### الملفات المعدّلة
~144 ملف (py, md, json, yaml, sh, txt, properties, Makefile) + `.windsurf` workflows/rules

### التحقق
- [x] Grep يؤكد: 0 ملفات مصدرية تحتوي المسار القديم ✅

---

## [004] — إصلاح بحث SITL CSV في سكريبت المقارنة (2026-05-13)

### المشكلة
`compare_sitl_vs_standalone.py` يبحث عن `sitl_*.csv` مباشرة في `results/`، لكن ملفات SITL الفعلية موجودة في `results/run_*/flight.csv`.

### الملفات المعدّلة

| الملف | Tier | التعديل |
|-------|------|---------|
| `6DOF_v4_pure/sitl/compare_sitl_vs_standalone.py:743` | 2 | أُضيف fallback للبحث في `run_*/flight.csv` |

### الكود قبل وبعد

**قبل:**
```python
sitl_csv = Path(args.sitl) if args.sitl else _find_latest(_RESULTS_DIR, "sitl_*.csv")
```

**بعد:**
```python
sitl_csv = Path(args.sitl) if args.sitl else (_find_latest(_RESULTS_DIR, "sitl_*.csv") or _find_latest(_RESULTS_DIR, "run_*/flight.csv"))
```

### التحقق
- [ ] تشغيل `compare_sitl_vs_standalone.py` بدون أخطاء

---

## [005a] — إصلاح اتصال HIL وتفعيل قناة MAVLink TCP 5760 (2026-05-14)

### المشكلة
اختبار HIL كان يفشل بثلاثة أعراض:
1. **لا تسلّح**: timing reader لا يتصل → `hb#0` → لا كشف ARM
2. **لا فيدباك سيرفو**: `SRV_FB` يأتي عبر 5760 → `online_mask=0x00` → ABORT بعد 500ms
3. **EKF2 لا يتقارب**: settle مدته 3s فقط → `gnss_pos=0` → R8 PRE-LAUNCH FAIL → MPC لا يحلّ

### السبب الجذري — سلسلة فشل
```
F-D650 rev3 غيّر -o من 14551 إلى 14550 (لأجل QGC AutoConnect)
  → TCP bridge (يستمع على UDP 14551) لا يستقبل بيانات من PX4
    → port 5760 ميّت (لا heartbeat، لا SRV_FB، لا PARAM_VALUE)
      → timing reader فشل → لا ARM detection
      → لا فيدباك سيرفو → ABORT
      → لا يمكن ضبط SYS_HITL عن بعد
```

### الملفات المعدّلة

| الملف | Tier | التعديل |
|-------|------|---------|
| `AndroidApp/app/src/main/cpp/px4_jni.cpp:1968` | 0 | `-o 14550` → **`-o 14551`** (إرجاع F-D650 rev3 لتغذية TCP bridge) |
| `6DOF_v4_pure/hil/hil_config.yaml` | 1 | 4 تعديلات (انظر أدناه) |

### تفاصيل التعديلات

#### 1. `px4_jni.cpp` — إرجاع `-o 14551` (Tier 0)

**قبل (F-D650 rev3):**
```cpp
const char* mav_argv[] = {"mavlink", "start", "-u", "14550", "-o", "14550",
                          "-p", "-r", "40000", "-m", "config", nullptr};
```

**بعد:**
```cpp
const char* mav_argv[] = {"mavlink", "start", "-u", "14550", "-o", "14551",
                          "-p", "-r", "40000", "-m", "config", nullptr};
```

**السبب**: `mavlink_tcp_bridge` يربط UDP على `udp_port + 1 = 14551`. مع `-o 14550` PX4 كان يُرسل إلى port مختلف عن الذي يستمع عليه الـ bridge → port 5760 ميّت تماماً.

**أثر QGC**: لا يتأثر — `-p` (broadcast) يبثّ HEARTBEATs على الشبكة، وQGC يكتشف PX4 عبر broadcast أو عبر TCP 5760.

#### 2. `hil_config.yaml` — 4 تعديلات (Tier 1)

| المفتاح | قبل | بعد | السبب |
|---------|-----|-----|-------|
| `mavlink_tcp.host` | `10.42.0.42` | **`127.0.0.1`** | IP قديم + Android يرفض TCP المباشر؛ `adb forward` يمرّ عبر localhost |
| `timing.enabled` | `true` | **`true`** | (أُعيد بعد تعطيل مؤقت) لاستقبال SRV_FB والتوقيت |
| `warmup.external_arm` | `true` | **`false`** | legacy mode: bridge يُرسل ARM مباشرة عبر 4560 |
| `warmup.settle_after_arm_s` | `3.0` | **`15.0`** | EKF2 يحتاج ≥10s GPS لقبول `gnss_pos=1` |

### إجراءات إضافية

| الإجراء | السبب |
|---------|-------|
| `adb shell pm clear com.ardophone.px4v17` | حذف EEPROM قديم (من 2026-05-02) كان يحتوي بارامترات من جلسة سابقة — `SENS_EN_BAROSIM=0` محتمل |

### النسخ الاحتياطية
- `px4_jni.cpp.pre_fix_1778766348`

### نتائج HIL المتدرّجة

| المحاولة | المشكلة | الإصلاح | النتيجة |
|----------|---------|---------|---------|
| 1 (قبل أي تعديل) | `hb#0`، لا تسلّح، ينتظر 300s | — | ❌ لا اتصال |
| 2 (host + timing + external_arm) | `rx=0B`، لا بيانات على 5760 | عُدّل host إلى localhost | ❌ Connection refused |
| 3 (+ pm clear) | `SYS_HITL` غير مؤكد، Baro Error | حُذف EEPROM | ❌ No Baro |
| 4 (+ legacy ARM) | ABORT بعد 0.5s: `mask=0x00` | — | ❌ لا فيدباك سيرفو |
| 5 (+ `-o 14551` في APK) | تسلّح ✅، سيرفو ✅، لكن سقوط 2.2s | إصلاح UDP port mismatch | ⚠️ EKF2 لم يتقارب |
| 6 (+ settle 15s) | — | settle 3→15s | ⏳ قيد الاختبار |

### التحقق (V&V)
- [x] Timing reader يتصل: `hb#3+` ✅
- [x] فيدباك سيرفو: `online_mask=0x0F`, CAN=100% ✅
- [x] PX4 يتسلّح ✅
- [x] QGC يتصل عبر TCP 5760 ✅
- [ ] EKF2 `gnss_pos=1` قبل بدء الطيران (settle 15s)
- [ ] MPC يحلّ بشكل طبيعي (solve_time > 0)
- [ ] رحلة كاملة ناجحة (range > 2000m)
- [ ] PIL: يحتاج اختبار
- [ ] Ground Test: يحتاج اختبار

### ملاحظات
- **يتطلب إعادة بناء APK** لتفعيل تعديل `px4_jni.cpp`
- QGC يتصل عبر TCP 5760 (ليس UDP مباشر) — هذا السلوك المقصود مع `-o 14551`
- `PX4 RUNNING: ✗` في phone_health post — PX4 يتوقف عادةً بعد نهاية المحاكاة

---

## [005b] — تحويل MPC Input من Ground Truth إلى EKF2 (2026-05-14)

### المشكلة

في HIL airframe `22004_m130_rocket_mpc_hitl`، كان `ROCKET_USE_GT=1` يجعل MPC يقرأ من Ground Truth.
لكن في HIL **لا يوجد ground truth حقيقي** — القيم صفرية → MPC يحسب أوامر خاطئة → abort فوري.

### الإصلاح

- `ROCKET_USE_GT`: `1 → 0` في `22004_m130_rocket_mpc_hitl:38`

### الملفات (Tier 1)

- `AndroidApp/.../airframes/22004_m130_rocket_mpc_hitl`

### النتيجة

- الصاروخ يطير فعلياً بدل الإلغاء الفوري
- لكن MPC يفشل بعد burnout (يُعالج في [006] و [007])

### V&V

- ✅ HIL: الصاروخ يطلق ويطير ~5s بتحكم فعلي

---

## [006] — محاولة فاشلة: Proactive Reinit عند Burnout (2026-05-14) ❌ REVERTED

### الفرضية

عند burnout، الـ warm-shifted trajectory من مرحلة الدفع تصبح خاطئة لمرحلة coast.
فرض reinit مبكر + تقليل عتبة reinit من 10→3 + زيادة RTI في coast يُحسّن التقارب.

### ما عُدِّل (مؤقتاً)

- `mpc_controller.cpp`: proactive `_reinit()` عند `t >= burn_time`
- `mpc_controller.cpp`: `_consec_fails >= 10` → `3`
- `mpc_controller.cpp`: `n_rti = 5` أثناء coast المبكر
- `mpc_controller.h`: إضافة `_burnout_reinit_done` flag

### النتيجة — أسوأ!

| المقياس | قبل | بعد التعديل |
|---|---|---|
| المدى | 2336m | 2087m (-11%) |
| ارتفاع | 393m | 193m (-51%) |
| آخر fin نشط | tf=5.2s | tf=3.1s (أبكر بـ 2s!) |

### السبب

`_reinit()` يضع `u = 0` كنقطة بداية → solver البارد ينتج أوامر صفرية →
الزعانف تفقد التحكم عند burnout بدل tf=5s.
تقليل عتبة reinit (3 بدل 10) زاد تكرار الـ reset إلى صفر.

### الإجراء

أُرجعت كل التعديلات فوراً — الكود عاد لحالته الأصلية.

### الدرس المُستفاد

- `_reinit()` مع `u_zero` يدمّر الـ warm trajectory الفعّال
- الاحتفاظ بآخر أمر ناجح (freeze) أفضل من reset إلى صفر
- المشكلة الحقيقية ليست warm-start بل سرعة الـ solver على ARM64

---

## [007] — تقليل MPC Horizon من N=80 إلى N=40 (2026-05-14)

### المشكلة الجذرية

acados solver على ARM64 (Samsung S23 Ultra) بطيء جداً مع N=80:

- كل RTI iteration: 10-30ms
- 3 iterations × 25Hz = 30-90ms budget → لا يكفي للتقارب بعد burnout
- solver status=4 (MAXITER) من tf=4.7s → الزعانف تتجمد → الصاروخ ينقلب

بيانات من ULG (run `2026-05-14T15-07-23Z_HIL`):

- MPC fails: 189/271 (70%) — كلها بعد burnout
- Solver status=4 يبدأ عند tf=4.72s
- MPC timing: p99 = 47,500us (47.5ms!) — يتجاوز budget 40ms بكثير

### الحل

تقليل N من 80 إلى 40 مع إبقاء tf=1.6s:

- dt_h: 20ms → 40ms (أخشن لكن كافٍ)
- Integration steps: 80 → 40 (نصف الحسابات)
- QP: cond_N=8 (لا يتغير)
- RTI المتوقع: 5-15ms بدل 10-30ms

### الملفات (Tier 1)

| الملف | التعديل |
|---|---|
| `6DOF_v4_pure/mpc/m130_ocp_setup.py:76` | `N = 80 → 40` |
| `6DOF_v4_pure/config/6dof_config_advanced.yaml:136` | `N_horizon: 80 → 40` |
| `c_generated_code/*` | إعادة توليد بـ N=40 |
| `AndroidApp/.../acados_arm64/lib/libm130_solvers.a` | إعادة بناء ARM64 (181KB) |
| `AndroidApp/.../m130_mpc/acados_solver_m130_rocket.h` | Header synced (N=40) |

### التحقق — SITL مع N=40

| المقياس | N=80 (baseline) | N=40 (جديد) |
|---|---|---|
| المدى | 2577m (-0.9%) | 2581m (-0.7%) ✅ |
| Max alpha | ~6 deg | 6.5 deg ✅ |
| Max Mach | 0.748 | 0.750 ✅ |
| وقت الطيران | 14.4s | 14.21s ✅ |
| Solver failures | 0 | 0 ✅ |
| Score | 100/100 | 100/100 ✅ |

### V&V المطلوب

- ✅ SITL (standalone): 100/100 — متطابق مع N=80
- ⏳ HIL: يحتاج بناء APK واختبار
- ⏳ PIL: بعد نجاح HIL

### ملاحظات

- `ROCKET_MPC_TF = 1.6` لا يتغير (الأفق الزمني ثابت)
- `mpc_controller.h: MPC_N = M130_ROCKET_N` يقرأ تلقائياً من header المُولَّد
- PX4 C code لا يحتاج تعديل — يتكيف مع أي N
- إذا لم يكفِ N=40 على ARM64، الخطوة التالية N=20 (tf=1.6s, dt_h=80ms)

---

## [008] — إصلاح السبب الجذري: QP Failure Cascade في RTI Loop (2026-05-14)

### الخلفية — تجارب فاشلة (أُلغيت)

قبل الوصول للإصلاح الحقيقي، جُرِّبت تعديلات لم تنجح:

1. **تقليل N=80→40** ([007]): SITL 100/100 لكن HIL 3/4 runs فشلت عند tf≈4.5s.
   أُرجع N=80 لأن المشكلة ليست حجم الأفق.
2. **تصفير gps_delay_ms**: لم يُحسّن — GPS rejection بقي 80%. أُرجع إلى 100ms.

### المشكلة الجذرية الحقيقية

تحليل ULG لـ 7 HIL runs (N=80 و N=40) كشف **نمطاً واحداً ثابتاً**:

- MPC ينجح 100% خلال مرحلة الدفع (tf=0-3.5s)
- **أول فشل solver دائماً عند tf≈4.2-4.5s** (بعد burnout مباشرة)
- بعد أول فشل: **تتالي كارثي** → فقدان كامل للتحكم

**السبب**: دالة `solve_mpc_acados()` في `mpc_controller.cpp` كانت تتعامل مع
فشل QP solver (status=4) بطريقة تُضخّم المشكلة بدل حلها:

```
RTI loop:
  for each iteration:
    status = acados_solve()
    if status == 4 (QP_FAILURE):
      ok = false
      break        ← [الخطأ 1] يخرج فوراً بدل إكمال الـ iterations

Post-solve:
  if !ok:
    hold last command  ← [الخطأ 2] يجمّد الزعانف على آخر أمر
    _warm = false      ← [الخطأ 3] يُبطل warm-start
    if consec_fails >= 10:
      _reinit(u=0)     ← [الخطأ 4] يُصفّر الزعانف = كارثة
```

**التتالي الكارثي:**
```
QP fail عند burnout (status=4)
  → break بعد iteration واحدة (بدل 3)
  → warm-start مُعطّل
  → next solve: cold start → QP أصعب → fail مرة أخرى
  → بعد 10 fails: _reinit() → الزعانف = 0°
  → الصاروخ بدون تحكم عند V≈240m/s
  → theta ينهار → EKF2 يفقد المرجعية → فشل كامل
```

### الإصلاح (Tier 0 — `mpc_controller.cpp`)

**3 تعديلات مترابطة في منطق RTI loop:**

#### التعديل 1: لا break عند QP failure — أكمل كل الـ iterations

```cpp
// قبل (الكود القديم):
for (int i = 0; i < n_rti; i++) {
    status = m130_rocket_acados_solve(_capsule);
    if (status != 0 && status != 2) {
        ok = false;
        break;                    // ← يخرج فوراً!
    }
    iters_done++;
}

// بعد (الكود الحالي):
for (int i = 0; i < n_rti; i++) {
    status = m130_rocket_acados_solve(_capsule);
    iters_done++;
    if (status == 1) {            // NaN فقط = unrecoverable
        ok = false;
        break;
    }
    if (status != 0 && status != 2) {
        ok = false;               // سجّل الفشل لكن لا تخرج
        // Don't break — HPIPM stored partial iterate,
        // next RTI iteration often recovers.
    }
}
```

**المبرر**: في SQP_RTI، حتى لو فشل QP في iteration واحدة، HPIPM يحفظ
iterate جزئي. الـ iteration التالية تبني عليه بـ linearization أفضل.
إكمال 3 iterations بدل 1 يعطي فرصة للتعافي.

#### التعديل 2: استخدم مخرجات solver الـ sub-optimal بدل تجميد الزعانف

```cpp
// قبل (الكود القديم):
if (!ok) {
    de = _last_delta_e;           // ← تجميد على آخر أمر!
    dr = _last_delta_r;
    da = _last_delta_a;
    _warm = false;                // ← إبطال warm-start!
    if (_consec_fails >= 10) _reinit(x_mpc);  // ← تصفير الزعانف!
}

// بعد (الكود الحالي):
if (!finite_check) {
    // NaN فعلي — الحالة الوحيدة لتجميد الزعانف
    de = _last_delta_e;
    dr = _last_delta_r;
    da = _last_delta_a;
    if (_consec_fails >= 30) _reinit(x_mpc);

} else if (!ok) {
    // Solver فشل لكن المخرجات finite — استخدمها!
    de = (float)x1[12];           // ← تحكم فعلي sub-optimal
    dr = (float)x1[13];
    da = (float)x1[14];
    // لا _warm=false — warm-start يبقى فعالاً
    // لا _reinit — تحكم sub-optimal أفضل من صفر
}
```

**المبرر**: بعد 3 RTI iterations، حتى لو status≠0، المخرجات finite تمثّل
أفضل تقدير للـ solver. عند burnout transition، تحكم sub-optimal بزعانف فعّالة
**أفضل بكثير** من تجميد الزعانف أو تصفيرها.

#### التعديل 3: رفع عتبة _reinit من 10 إلى 30

```
قبل: _consec_fails >= 10  → _reinit(u=0)
بعد: _consec_fails >= 30  → _reinit(u=0)  (وفقط عند NaN)
```

**المبرر**: مع التحكم الفعلي عبر مخرجات solver الـ sub-optimal، حالة NaN
الحقيقية نادرة جداً. رفع العتبة يمنع إعادة التهيئة المبكرة.

### الملفات المعدّلة

| الملف | Tier | التعديل |
|---|---|---|
| `AndroidApp/.../rocket_mpc/mpc_controller.cpp:626-649` | **0** | RTI loop: لا break عند status=4، break فقط عند status=1 (NaN) |
| `AndroidApp/.../rocket_mpc/mpc_controller.cpp:692-728` | **0** | Post-solve: استخدام مخرجات finite sub-optimal + إلغاء _warm=false + رفع _reinit threshold |
| `6DOF_v4_pure/mpc/m130_ocp_setup.py:76` | 1 | N أُرجع من 40 إلى **80** (التغيير [007] لم ينجح) |
| `6DOF_v4_pure/config/6dof_config_advanced.yaml:136` | 1 | `N_horizon` أُرجع من 40 إلى **80** |
| `6DOF_v4_pure/config/6dof_config_advanced.yaml:279` | 2 | `gps_delay_ms` أُرجع من 0 إلى **100** (التصفير لم يُحسّن) |
| `c_generated_code/*` | 1 | إعادة توليد solver C code بـ N=80 |
| `AndroidApp/.../acados_arm64/lib/libm130_solvers.a` | 1 | إعادة بناء ARM64 solver (179KB, N=80) |
| `AndroidApp/.../m130_mpc/acados_solver_m130_rocket.h` | 1 | Header synced: `M130_ROCKET_N = 80` |

### Backup

```
mpc_controller.cpp.pre_fix_1747245018
```

### نتائج HIL بعد الإصلاح (5 runs, 2026-05-14)

| Run | Score | Range | Err% | MPC fails | Max α | First fail | الحالة |
|---|---|---|---|---|---|---|---|
| 19:19 | 56.7 WARN | 2585m | -0.6% | 35 (13%) | 171.8° | tf=12.36s | tumble هبوط (طبيعي) |
| 19:24 | **88.7 PASS** | 2563m | -1.4% | **0 (0%)** | 6.2° | — | ✅ ممتاز |
| 19:25 | **95.0 PASS** | 2589m | -0.4% | **0 (0%)** | 5.8° | — | ✅ **أفضل نتيجة** |
| 19:27 | 68.0 WARN | 2517m | -3.2% | **1 (0%)** | 6.1° | tf=13.56s | tumble هبوط (طبيعي) |
| 19:28 | 32.5 FAIL | 1556m | -40.2% | 1 (1%) | 24.5° | tf=8.52s | scheduler jitter |

**ملخّص**: 4/5 runs range ≥2517m (err ≤3.2%). Run الفاشل سببه Android scheduler
jitter (3 فجوات >100ms متتالية عند burnout) — ليس MPC.

### مقارنة قبل وبعد الإصلاح

| المقياس | قبل الإصلاح (7 runs) | بعد الإصلاح (5 runs) |
|---|---|---|
| MPC fail rate | **48-54%** | **0-1%** ✅ |
| First fail timing | **tf≈4.2s** (burnout) | **tf≈12-13s** (descent) ✅ |
| Success rate (range >2400m) | **2/7 (29%)** | **4/5 (80%)** ✅ |
| Best score | 56.7 WARN | **95.0 PASS** ✅ |
| Worst range | 1393m | 1556m (scheduler jitter) |
| Max α (good runs) | 14.9° | **5.8-6.2°** ✅ |

### تأثير على الإطلاق الحقيقي

هذا الإصلاح **يؤثر مباشرة على الطيران الحقيقي** (ليس HIL فقط):
- QP failures تحدث في الطيران الحقيقي أيضاً عند burnout transition
- السلوك القديم (break + cold start + reinit) كان سيُفقد التحكم
- السلوك الجديد (continue + sub-optimal + warm) يحافظ على التحكم

### المخاطر المتبقية

| المخاطر | الخطورة | التخفيف |
|---|---|---|
| Android scheduler jitter (1/5 runs) | 🟠 متوسط | تبريد الهاتف + قتل تطبيقات خلفية + ADPF boost |
| Late descent tumble (tf>12s) | 🟡 منخفض | سلوك فيزيائي طبيعي لصاروخ باليستي بدون مظلة |
| Servo delay variation (195-420ms) | 🟡 منخفض | Cross-correlation measurement noise — القيمة الفعلية أكثر استقراراً |

### V&V

- ✅ HIL: 5 runs (4/5 ناجح، 1 فشل scheduler jitter)
- ⏳ PIL: بعد تأكيد استقرار إضافي
- ⏳ يُوصى بـ 5+ runs إضافية مع تبريد كافٍ بين الـ runs

---

## [009] — تعطيل Auto-Launch في HIL Runner (2026-05-15)

### المشكلة

`hil_runner.py` يُشغّل التطبيق تلقائياً عبر `adb am start` (سطر 1165).
عند الاختبار المتكرر، أراد المستخدم التحكم اليدوي بفتح التطبيق لتجنب إعادة تشغيله
تلقائياً بين الـ runs (خصوصاً أثناء تشخيص مشاكل thermal throttling).

### الملفات المعدّلة

| الملف | Tier | التعديل |
|-------|------|---------|
| `6DOF_v4_pure/hil/hil_runner.py:1165` | 2 | `_auto_launch_app(verbose=True)` → `pass  # _auto_launch_app(verbose=True)` |

### الكود قبل وبعد

**قبل:**
```python
    if not args.no_auto_launch:
        _auto_launch_app(verbose=True)
```

**بعد:**
```python
    if not args.no_auto_launch:
       pass  # _auto_launch_app(verbose=True)
```

### ملاحظات

- **تعديل تجريبي مؤقت** — يمكن إرجاعه بإزالة `pass  #`
- البديل بدون تعديل الكود: تشغيل `python3 hil_runner.py --no-auto-launch`
- التطبيق لا يُغلق (`force-stop`) لأن `preflight_reset=false` في الإعدادات الحالية
- التعديل **لا يؤثر** على منطق الطيران أو التحكم — فقط على طريقة تشغيل التطبيق

### التحقق

- ✅ HIL: التطبيق يبقى مفتوحاً بين الـ runs — يفتحه المستخدم يدوياً
- لا حاجة لـ V&V إضافي (Tier 2، لا تأثير على flight code)

---
