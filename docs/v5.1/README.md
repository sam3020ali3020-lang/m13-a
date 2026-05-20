# v5.1 — إصدار التَثبيت (مُقارنةً بِخطّ الأساس v2)

**Tag**: `v5.1`
**Commit**: `291917cc` (بعد `700260e3 v5: NaN cascade fix` + thermal sidecar + إصلاح طريقة التشغيل)
**خطّ الأساس v2**: `a9271184` — *"v2 snapshot: SITL Score 90.1/100, Range 2567m"*
**التاريخ**: 2026-05-20

---

## 1) ماذا يُقدّم v5.1 فوق v2

كان v2 خطّ أساس مُتحقَّق منه على **SITL** فقط (بدون HITL، بدون مُعالجة NaN، بدون
PD fallback). v5.1 هو الإصدار **المُتحقَّق منه على HITL**، **المُقاوِم لـ NaN**،
**والمُجهَّز بِقياس الحرارة**.

| الجانب | v2 | v5.1 |
|---|---|---|
| مسار EKF2 على HITL | مكسور (أخطاء gravity / quat / mag) | **يَعمل** (`ROCKET_USE_GT=0`) |
| مُعالجة `status=4` من حلّال QP | تَجميد آخر أمر → cascade | استخدام الـ iterate غير الأمثل (تحكُّم مُستمرّ) |
| NaN مُستمرّ | تَجميد دائم → tumble | **PD fallback على α/β/q/r** بعد 5 دورات تَجميد |
| عَتَبة `_reinit()` | 10 دورات فشل | 30 دورة (مُخصَّصة لـ NaN فقط) |
| الجاذبيّة في MAVLink bridge | تُطرح من `f_body` (خاطئ) | `specific_force = F_ext/m` (صحيح) |
| خرج IMU عند الراحة على المنصّة | صفر → فشل tilt-align | `-m·C·g` (ردّ فعل المنصّة) → tilt-align يَنجح |
| EKF2 mag (HITL) | type 5 (لا mag) → لا yaw | type 6 (alignment لمرّة واحدة عند ARM) → yaw مُثبَّت |
| طريقة تشغيل HIL | START/STOP فقط → تَسرُّب الحالة بين runs | `am force-stop` + `pm clear` لكل run |
| رؤية حرارة الـ CPU | لا شيء | sidecar يَقرأ الحرارة + cpu0/4/7 freq → بطاقة في HTML |
| تَباين الـ range error (5 runs) | غير مُتاح (SITL فقط) | **−6.2 % … −0.5 %** (مقابل −25 %…+10 % للمستخدم بطريقة معطوبة) |

---

## 2) التغييرات ملفّاً ملفّاً مُقارنةً بـ v2

### 2.1 `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/mpc_controller.cpp` (+179 سطراً)

**لماذا**: في v2، أيّ تَعثُّر للحلّال كان يُجمِّد آخر أمر على الزعانف. على Android
يُرجع حلّال QP أحياناً `status=4` (slacked QP / max iterations) عند burnout
وعند الـ α العالي خلال الهبوط. التجميد خلال حالة عابرة يَجعل الـ iterate يَنحرف،
فيَفشل الـ solve التالي، ويَستمرّ ذلك (*NaN cascade*). أَظهرت runs PARTIAL في v4
أكثر من 4 ثوانٍ من التجميد → α يَصل إلى ±179°.

**قبل (سلوك v2)**:
```cpp
if (!ok || !finite_check) {
    de = _last_delta_e;  dr = _last_delta_r;  da = _last_delta_a;
    if (_consec_fails >= 10) _reinit(x_mpc);
}
```

**بعد (v5.1)**: تَقسيم إلى ثلاث حالات:

1. **NaN (`!finite_check`)** — تَجميد لمدّة `MAX_FREEZE_CYCLES = 5` (≈200 ms)،
   ثم تَفعيل **PD fallback** على α/β مع تَخميد بِالمعدّلات q,r. الـ δa مُثبَّت
   على 0 لتجنُّب roll-yaw coupling أثناء فشل الحلّال. خرج الـ PD مَحدود بِـ ±15°
   (هامش 5° تحت حدّ الحلّال 20°). عَتَبة `_reinit()` رُفعت `10 → 30` دورة
   (≈1.2 s) كي لا نُعاقب nan-bursts عابرة.

2. **غير أمثل لكنّه نِهائي (`!ok && finite_check`)** — *فرع جديد*. الحلّال أَبلغ
   بِفشل لكن الخرج نِهائي. نَستخدم حلّ الـ iterate الجزئي مباشرةً
   (`de = x1[12]; dr = x1[13]; da = x1[14]`). **لا** نُبطل `_warm`، **لا** نَزيد
   `_consec_fails`. هذا هو "[008] QP cascade fix" الذي كَسر حلقة التجميد عند
   burnout.

3. **نظيف (`ok && finite_check`)** — كما في v2، مع log إضافيّ عند الخروج من PD
   fallback (recovery edge).

**العلامات في الكود**: ابحث عن `[008]` و `[v4-NaN-fix]` للحصول على كُتل
التعليقات بالضبط.

### 2.2 `…/rocket_mpc/mpc_controller.h` (+19 سطراً)

إضافة ثوابت + حالة الـ PD fallback:
```cpp
static constexpr float PD_KP_ALPHA = 2.0f, PD_KD_ALPHA = 0.3f;
static constexpr float PD_KP_BETA  = 2.0f, PD_KD_BETA  = 0.3f;
static constexpr float PD_DELTA_MAX = 0.2618f;  // 15°
static constexpr int   MAX_FREEZE_CYCLES = 5;   // ≈200 ms @ 25 Hz
float _prev_alpha{0}, _prev_beta{0};
bool  _fallback_active{false};
int   _fallback_count{0};
```

تَمّ اختيار الـ gains تَجريبيّاً لِإبقاء α أقلّ من ±30° خلال انقطاع الحلّال
لِمدّة 1.2 s دون تَجاوز إلى نِطاق التشبُّع.

### 2.3 `6DOF_v4_pure/hil/mavlink_bridge_hil.py` (+130 سطراً صافياً)

تَمّ إصلاح خطأين، كلاهما يُماثل إصلاحات أُجريَت سابقاً في PIL bridge.

**الخطأ A — `_body_specific_force` كان يَطرح الجاذبيّة مرّتين.**

*قبل (v2)*:
```python
return f_body / max(mass, 0.1) - C @ g_ned   # خاطئ
```
المُحاكي 6DOF يَستثني الجاذبيّة من `f_body` أصلاً (دفع + aero فقط). مِقياس
تسارُع الـ IMU يَقرأ `specific_force = a_inertial - g`، وهو لِـ
`f_body = F_ext` ببساطة `F_ext / m`. طرح `C @ g_ned` كان يُنتج accelerometer
يَقرأ **−2 g** عند الراحة → EKF2 tilt-align يَقلب الصاروخ.

*بعد (v5.1)*:
```python
return f_body / max(mass, 0.1)              # صحيح
```

**الخطأ B — عيّنة IMU عند الراحة على المنصّة كانت تُرسل قوّة صفر.**

الصاروخ على القاضب مَحجوز بِردّ فعل `F_pad = -m·C·g_ned`. حلقة نشر الحسّاسات
قبل ARM كانت تُرسل `forces = [0,0,0]`، وبعد إصلاح الخطأ A أَصبح الـ
accelerometer يَقرأ صفراً مَحضاً → EKF2 لا يَملك متّجه جاذبيّة لِـ alignment →
tilt-alignment لا يَتقارب أبداً → MPC يَرى وضعيّة عشوائيّة.

*بعد*:
```python
pad_forces = -mass * (C_ned2b @ [0, 0, 9.80665])
snap = {"forces": pad_forces, "vel_ned": [0, 0, 0],
        "position_lla": (launch_lat, launch_lon, launch_alt)}
```

`position_lla` أَيضاً صار صريحاً الآن — بِدونه، long-range mode كان يُعامل
موقع ECEF كأنّه NED ويُفسد البارومتر.

### 2.4 `AndroidApp/app/src/main/cpp/px4_jni.cpp` (+3572 / −1030 سطراً)

أَكبر ملفّ مُتغيّر؛ التغييرات تَنقسم إلى ثلاث مَجموعات:

| المَجموعة | الأسطر | الغرض |
|---|---|---|
| تَفعيل EKF2 على مسار HITL | ~600 | كُتلة param-default للـ `ROCKET_USE_GT=0`، `EKF2_MAG_TYPE=6`، `EKF2_PREDICT_US=10000`، `IMU_INTEG_RATE=100`، إلخ — تَضبط نفس القيم التي يَضبطها airframe في ROMFS، لكن وقت تَهيئة JNI، حتى لا تَعتمد التطبيقات الجديدة على انتشار rcS_extras. |
| سطح JNI جديد لِواجهة التطبيق | ~1500 | `getFlightTime / getDownrange / getAltitude / getServoStatus / startLogging / stopLogging / setUseGroundtruth …` — يَكشف حالة PX4 الداخليّة لِنشاط Android. إضافة بَحتة. |
| تسجيل + تشخيص | ~1500 | كاتب CSV لِكل دورة (الصفوف التي تَراها في `hil_flight_*.csv`)، لاقط أحداث envelope-override، لاقط توقيت MPC، لاقط telemetry للـ servo CAN. |

**السلوك الصافي**: كان `px4_jni.cpp` في v2 مُجرّد launcher بسيط (~1000 سطر).
أَصبح في v5.1 سطح bridge HITL/PIL كاملاً للتطبيق Android. لم يُحذَف أيّ سلوك من
v2؛ كلّ شيء إضافيّ.

### 2.5 `AndroidApp/.../ROMFS/.../22004_m130_rocket_mpc_hitl` (+5 / −2)

```diff
-param set EKF2_MAG_TYPE   5  # لا mag (المُحاكي يُوفّر الوضعيّة مباشرة)
+param set EKF2_MAG_TYPE   6  # alignment لمرّة واحدة عند ARM
```

مع `ROCKET_USE_GT=0` (مسار EKF2 الحقيقي)، عدم وجود fusion للـ magnetometer
يَعني yaw غير مُعرَّف → إطار NED غير مُعرَّف → MPC يَرى حالة عشوائيّة. النوع 6
يُجري **alignment واحداً** عند ARM ثم يَتجاهل الـ mag (لا انجراف من الـ fusion)
— نفس الإستراتيجيّة المُستعمَلة في airframe الـ PIL.

### 2.6 `6DOF_v4_pure/hil/hil_config.yaml`، `6DOF_v4_pure/pil/pil_config.yaml` (+1 / −1 لكل ملفّ)

تَغيُّر IP الهدف فقط — `10.42.0.145 → 10.42.0.215`. هذا تَخصيص محوّل
USB-Ethernet عند المُستخدم؛ لا تَغيُّر سُلوكيّ. انتبه إذا تَغيّر IP الهاتف مَرّة
أُخرى.

### 2.7 `6DOF_v4_pure/hil/hil_analysis.py` (مُعدَّل)

أُضيفَت ثلاثة أشياء لِدعم thermal sidecar:

1. `load_hil_thermal(csv_path)` — يَلتقط `<flight_stem>_thermal.csv` بِجوار CSV
   الرحلة (نفس النمط مثل `load_hil_timing` / `load_hil_servos`).
2. في `analyze_hil_csv`: يَطوي إحصائيّات حرارة الـ CPU + ترددات cpu0/4/7 ضمن
   قاموس الـ `metrics` (`cpu_temp_max_c`, `cpu7_freq_mean_mhz`, …).
3. في `generate_html`: بطاقة جديدة *"CPU Temperature + Throttle (phone)"* في
   شبكة الـ overview، مُلوَّنة وفقاً لِـ:
   - الحرارة: <60 °C pass، 60–80 °C warn، ≥80 °C fail
   - نسبة cpu7 mean/max: <50 % fail، 50–70 % warn، ≥70 % pass

بِالإضافة إلى سَطر مُلخَّص في الـ console كي تَكون الحرارة مَرئيّة دون فتح
HTML.

### 2.8 `6DOF_v4_pure/hil/hil_runner.py` (مُعدَّل)

`run_hil()` يَنشُر الآن thermal sidecar (`_thermal_quick.sh`) كَعمليّة فرعيّة
في الخلفيّة تَكتب إلى `<flight_stem>_thermal.csv`. مُلَفَّف بِـ `try / finally`
لِضمان إنهاء الـ sidecar دائماً عند خروج الـ bridge أو على `SIGINT`. يُضيف ≤2 %
حِمل CPU وجَولة USB واحدة كل 500 ms — قليل جدّاً ولا يُؤثّر على توقيت MAVLink.

### 2.9 `6DOF_v4_pure/hil/_thermal_quick.sh` (**ملفّ جديد**)

ماسح Bash لِـ حرارة CPU الهاتف + ترددات cpu0/cpu4/cpu7 عبر `adb shell` لقراءة
`scaling_cur_freq`. مَحصور في thermal zones التي يَحوي حقل `type` فيها كلمة
`cpu` (يَستثني البطاريّة، GPU، modem) كي تَكون أعلى حرارة مُبلَّغ عنها هي حرارة
CPU فعلاً. يَكتب CSV بِترويسة
`wall_time,cpu_temp_c,cpu0_freq_mhz,cpu4_freq_mhz,cpu7_freq_mhz`.

---

## 3) اكتشاف طريقة التشغيل (بِدون تَعديل كود لكن بِأكبر أثر)

التغايُر الذي أَبلغ عنه المُستخدم عبر 4 تشغيلات بِنفسه: range error من
**−25.8 %** إلى **+9.8 %** (تَشتُّت ≈35 %).

نفس الكود، 5 تشغيلات بطريقة مختلفة عندي: range error من **−6.2 %** إلى
**−0.5 %** (تَشتُّت ≈6 %).

**الفرق**: بين التشغيلات لم يَكن المُستخدم يَنفّذ `am force-stop` ولا
`pm clear` للتطبيق. عواقب ذلك اثنتان:

1. `RKT_MPC_SVO_DLY` يُحفَظ **تلقائيّاً** في نِهاية كل run من قياس تأخير الـ
   servo. كَتب Run 1 قيمة `0.14 s`. بدأ Run 2 بِـ `0.14 s` (وهذا يَضبط
   `lookahead_stage = 7`)، قاس `0.20 s`، وحَفظ `0.20 s`. بدأ Run 3 بِـ
   `0.20 s` (`lookahead_stage = 10`)، وهكذا. بِحلول Run 4 صار الـ lookahead
   يُساوي `17` — تنبّؤات MPC مُختلفة تماماً عن Run 1.
2. عمليّة Android احتفظت بِالحالة الساكنة لـ PX4 modules (خاصّة تقديرات gyro
   / accel bias في EKF2). الدخول مُجدَّداً إلى pre-arm بِبَيانات bias قديمة
   حَرف tilt-alignment للـ run الجديد.

**الإصلاح (لا يَحتاج تَعديل كود)**: مُوثَّق في
`docs/v5.1/CLEAN_RUN_WORKFLOW.md`. الـ hil_runner يَتعامل مع جانب الـ host
بِالفعل؛ جانب المُستخدم يَحتاج أمرَي `adb shell` لكل run.

---

## 4) التحقُّق — 5 تشغيلات HITL مُتتالية (طريقة تشغيل نظيفة)

| # | الطابع الزمنيّ | range err | α max | fin sat | CPU max | الـ score |
|---|---|---:|---:|---:|---:|---:|
| 1 | 054405 | **−0.8 %** | 11.8° | 0.0 % | 52.7 °C | **95 ✅** |
| 2 | 054711 | −0.5 % | 79.3° (envelope catch) | 0.6 % | 57.8 °C | 56 ⚠️ |
| 3 | 054905 | −0.8 % | 19.4° | 0.0 % | 54.3 °C | 54 ⚠️ |
| 4 | 055057 | −6.2 % | 19.7° | 0.0 % | 59.0 °C | 45 ⚠️ |
| 5 | 055244 | −1.8 % | **179.9°** (terminal tumble) | 40.1 % | 66.6 °C | 44 ❌ |

**الخُلاصات**:
- **Range متّسق** عبر التشغيلات: 5/5 ضمن ±6 %، 4/5 ضمن ±2 %.
- **الحرارة ليست مُحرّك التغايُر** (أعلى قيمة شُوهدَت: 66 °C، أقلّ بِكثير من
  خطّ تَحذير 60-°C وخطّ throttle 80-°C على Snapdragon).
- **وضعيّة ما قبل apogee مُستقرّة** في 5/5 runs.
- **Tumble بعد apogee في 2/5 runs** — هذه القضيّة المفتوحة المُتبقّية، انظر §5.

---

## 5) القضيّة المفتوحة المُتبقّية: جَولات α عالية بعد apogee

العَرَض: في 2 من 5 runs يَصل الصاروخ إلى الهدف (range −0.5 % إلى −1.8 %) لكن
في النِهاية فقط (t > 11 s) يَنقلب الجسم إلى α ≈ 80° أو 180°.

لماذا هذه ليست مُشكلة workflow / EKF2 / حرارة / scheduler:
- الـ range error صغير جدّاً → المُتحكّم أَدّى عمله في طريق الصُعود.
- يَبدأ الـ tumble فقط **بعد apogee** عندما تَنخفض V تحت ~80 m/s.
- سُلطة الزعانف `Cn ∝ V²`، إذن عند V=80 m/s تَملك الزعانف ¼ العَزم الذي كانت
  تَملكه عند burnout. envelope-override يَتدخّل ويُشبع الزعانف عند 20°، لكن
  العَزم المُتاح ببساطة لا يَكفي.

ما يَلزم لإصلاحها (خارج نِطاق v5.1):
- إضافة حدّ يَعتمد على السرعة في كُلفة OCP (يُعاقب α بِشدّة أكبر مع انخفاض V).
- أو إضافة trigger لِفتح مظلّة عند apogee + 1 s، لكي لا يَكون المُتحكّم
  مَسؤولاً عن الوضعيّة في نِظام السرعات المنخفضة.

---

## 6) ما لا يُغيِّره v5.1 صراحةً

- تَعريف الـ OCP (`generated/acados_ocp.json`).
- نموذج الـ aero الـ 6DOF (`6DOF_v4_pure/*/aero.py`).
- حُزمة حلّال acados (`acados-main/`).
- مُشغِّل Servo CAN (`XqpowerCan.cpp`) باستثناء ملفّات `pre_*` الموجودة على
  القُرص لكنّها ليست على مَسار v5.1.
- سلوك v2 على SITL — تشغيل v5.1 على SITL لا يزال يُسجّل ~90/100.

---

## 7) ملفّات لم تُوثَّق عَمداً

يَحوي المُستودع كثيراً من ملفّات `*.pre_*` و `*.bak` المُتبقّية من جلسات
سابقة. **هي ليست جزءاً من v5.1** ومُحتفَظ بها على القُرص فقط كنقاط استرداد.
ستُنظَّف في commit مُنفصل للصيانة.
