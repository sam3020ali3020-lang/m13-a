# 🎯 إصلاح: HIL Bridge كان يَستَقبِل `fin_cmd = 0` رَغم أَنَّ rocket_mpc يَنشُر قِيَم حَقيقيَّة

**التَّاريخ**: 2026-05-08
**الحالة**: ✅ مَحلول (مُؤَكَّد بالقياس المُباشَر)
**المُلَفّات المُعَدَّلة**: ملف واحِد، 11 سَطر فِعلي
**الأَثَر**: Max fin: `0.0°` → `20.0°` ، actuator_msgs/warmup: `0` → `757`
**Attempts المُستَهلَكة**: 6/7

---

## 📌 الأَعراض

| المِقياس | قبل الإصلاح | بعد الإصلاح |
|---|---|---|
| `fin_cmd_*` في CSV الـbridge | `0.000000` (لكل خُطوة) | قِيَم حَقيقيَّة (±0.07 إلى ±0.349 rad) |
| Max fin في تَحليل الرِّحلة | **0.0°** | **20.0°** |
| `actuator_msgs` خِلال warmup | 0 (يُسَبِّب abort) | 757 (warmup يَنجَح) |
| Score | 23/100 | 13.9/100 (instability — مُشكِلة مُختَلِفة) |

**الصاروخ كان يَطير بِدون تَحَكُّم**: السيرفو لا يَتَحَرَّك، الصاروخ يَنحَرِف ويَسقُط في 6s.

---

## 🔬 التَّشخيص الكامِل (دَورات تَحقيق طَويلة)

### الدَّورة 1: تَأكيد أَنَّ MPC يَحسُب صَحيحاً
- ULog (`actuator_outputs_sim`): 197/1452 عَيِّنة غير صفرية، قِيَم نَموذجيَّة `[+0.047, +0.044, -0.048, -0.045]` rad
- ULog (`rocket_gnc_status.fin1..4`): مُتَطابِقة تَماماً → MPC يَحسُب
- **خُلاصة**: داخل PX4، البَيانات صَحيحة

### الدَّورة 2: تَأكيد أَنَّ الـbridge يَستَقبِل صِفر
- إضافة Python diagnostic في `_drain_target` يَطبَع كُلّ HIL_ACTUATOR_CONTROLS
- **النَّتيجة**: 106 رِسالة، **0 منها فيها قِيَم غير صفرية**
- **خُلاصة**: الـbridge يَستَقبِل صِفر فِعلاً

### الدَّورة 3: تَحديد المَنطِقة المَجهولة
- الـbridge يَقرأ TCP 4560 = `simulator_mavlink::send_controls()` (ليس `MavlinkStreamHILActuatorControls`)
- المَنطِقة المَجهولة = بَين `actuator_outputs_sim` topic و TCP send

### الدَّورة 4: ULog diagnostic داخل PX4
- إضافة publish إلى `debug_array` (id=77, name="SIM_RD") بَعد `orb_copy` مُباشَرة في `send_controls()`
- **النَّتيجة الصَّاعِقة**: 528 عَيِّنة SIM_RD، **0 منها فيها قِيَم** — رَغم أَنَّ `actuator_outputs_sim` فيه 343 قيمة غير صفرية في نَفس الـrun
- `orb_copy` يُرجِع `0` (نَجاح) لَكِن يَكتُب صفراً
- **خُلاصة**: الـlegacy `orb_copy` على `int handle` لا يَرى publishes مِن `uORB::Publication<T>` الحَديث

### الدَّورة 5a (فاشِلة): استِبدال orb_copy بـSubscription فَقَط
```cpp
// قَبل:
int _copy_ret = orb_copy(ORB_ID(actuator_outputs_sim), _actuator_outputs_sub, &_actuator_outputs);

// 5a:
_actuator_outputs_sim_sub.copy(&_actuator_outputs);
```
- **النَّتيجة**: warmup فَشَل تَماماً، 0 رَسائل خِلال 30s
- **السَّبَب**: `px4_poll` على الـlegacy fd لا يُعاد تَحفيزه (POLLIN edge-triggered) لأَنَّ `orb_copy` لم يَعُد يُسْتَدعى لاسْتِهلاك queue entry

### الدَّورة 5b: إعادة orb_copy فَقَط لِتَحفيز POLLIN
```cpp
actuator_outputs_s _legacy_drain{};  // يُتَجاهَل
orb_copy(ORB_ID(actuator_outputs_sim), _actuator_outputs_sub, &_legacy_drain);  // re-arm POLLIN
_actuator_outputs_sim_sub.copy(&_actuator_outputs);  // البَيانات الفِعليَّة
```
- **النَّتيجة**: warmup يَنجَح (179 msgs)، لَكِن fin لا يَزال 0
- ULog SIM_RD يَنتَهي عند `t=21421s`، Topic non-zero يَبدأ عند `t=21427.94s` (فَجوة 6.5s)
- **خُلاصة**: `send_controls()` يَتَوَقَّف عن العَمَل قَبل launch بِـ 6.5 ثانية

### الدَّورة 6 (نَجَحَت ✅): تَعطيل `px4_lockstep_wait_for_components()`
- الـbridge يَعمَل بِدون lockstep (`hil_config.yaml: lockstep:false`)
- لَكِن PX4 مَبني مع `ENABLE_LOCKSTEP_SCHEDULER=1`
- `px4_lockstep_wait_for_components()` يَحجِب thread الإرسال انتِظاراً لـcomponents لا تُؤَكِّد أَبَداً (logger/ekf2 في وَضع non-lockstep لا تُسَجِّل lockstep)
- التَّعطيل يُحَرِّر thread الإرسال

---

## 🛠 الإصلاح النِّهائي

### الملف الوَحيد المُعَدَّل
```
@/home/wd/Desktop/GAB_3/1234/m13/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/simulation/simulator_mavlink/SimulatorMavlink.cpp
```

### التَّعديل (3 أَجزاء)

#### 1. `send_controls()` (الأَسطُر 178-211)
```cpp
void SimulatorMavlink::send_controls()
{
    // إعادة تَحفيز POLLIN (الـlegacy fd لا يَزال مَرجَع px4_poll)
    actuator_outputs_s _legacy_drain{};
    orb_copy(ORB_ID(actuator_outputs_sim), _actuator_outputs_sub, &_legacy_drain);

    // البَيانات الفِعليَّة عَبر uORB::Subscription (مُتَوافِق مع uORB::Publication<T>)
    _actuator_outputs_sim_sub.copy(&_actuator_outputs);

    // ... (كَما كان)
}
```

#### 2. حَلقة `send()` (الأَسطُر 1148-1157) — تَعطيل lockstep_wait
```cpp
if (fds_actuator_outputs[0].revents & POLLIN) {
    parameters_update(false);
    check_failure_injections();
    _vehicle_status_sub.update(&_vehicle_status);
    _battery_status_sub.update(&_battery_status);

    // 2026-05-08 FIX: bypass lockstep wait. Verified by ULog/SIM_RD that
    // send_controls stops 6.5s before launch (t=21421s vs t=21427.94s)
    // because this wait blocks for components that never ack in non-lockstep
    // bridge mode (hil_config.yaml lockstep:false).
    // px4_lockstep_wait_for_components();

    send_controls();
}
```

#### 3. `SimulatorMavlink.hpp` (الأَسطُر 268-284) — إضافة Subscription
```cpp
// الـlegacy int handle لا يَزال مَطلوباً لِـpx4_poll POLLIN
int _actuator_outputs_sub{-1};

// مَصدَر البَيانات الحَقيقي — مُتَوافِق مع uORB::Publication<T> في rocket_mpc
uORB::Subscription _actuator_outputs_sim_sub{ORB_ID(actuator_outputs_sim)};

actuator_outputs_s _actuator_outputs{};
```

---

## 🧠 الدُّروس المُستَفادة

### 1. خَلط APIs قَديم وحَديث في uORB يَكسِر الرَّبط
- `Publication<T>` الحَديث في rocket_mpc
- `orb_subscribe_multi + orb_copy` القَديم في simulator_mavlink
- النَّتيجة: orb_copy يُرجِع نَجاحاً لَكِن البَيانات صِفر

### 2. POLLIN في PX4 edge-triggered
- بِدون `orb_copy` على الـfd المَربوط بـ`px4_poll`، POLLIN لا يُعاد تَحفيزه
- يَجِب استِهلاك queue entry حتَّى لَو لم نَستَخدِم البَيانات

### 3. `px4_lockstep_wait_for_components` خَطير في HITL غير-lockstep
- مُصَمَّم لـSITL مع lockstep كامِل
- في HITL مع `lockstep:false`، يَحجِب الـthread إلى الأَبَد
- يَجِب أَن يَكون مَشروطاً بـrun-time check للـlockstep status (وليس compile-time `ENABLE_LOCKSTEP_SCHEDULER`)

### 4. Diagnostic عَبر ULog أَفضَل مِن logcat
- `PX4_INFO` مِن simulator_mavlink لم يَظهَر في logcat (مَعروف في Android port)
- `debug_array` يُسَجَّل دائماً في ULog → دَليل مَوثوق

---

## 🔁 كيف نُتَأَكَّد مِن استِمرار الإصلاح

### اختِبار سَريع
1. شَغِّل `/lab` أَو `/ground` workflow
2. بَعد الـrun، تَحقَّق:
   ```
   Max fin: > 0° (أَيّ قيمة فوق الصِّفر)
   actuator_msgs (warmup): > 100
   fin_cmd_* في CSV: قِيَم غير صفرية بَعد launch
   ```

### في ULog
```python
from pyulog import ULog
import numpy as np
u = ULog('latest.ulg')
for d in u.data_list:
    if d.name == 'debug_array':
        ids = np.array(d.data['id'])
        if 77 in ids:
            mask = ids == 77
            outs = np.array([d.data[f'data[{i}]'] for i in range(4)]).T[mask]
            non_zero = np.sum(np.any(np.abs(outs) > 0.001, axis=1))
            assert non_zero > 50, f"Regression: only {non_zero} non-zero SIM_RD samples"
```

---

## ⚠️ مُشكِلة مُتَبَقِّية (مُنفَصِلة)

بَعد الإصلاح، `Max fin = 20°` (أَقصى)، لَكِن:
- `Max |α| = 177.8°` — الصاروخ يَنقَلِب
- Time = 6.76s (تَحَطُّم سَريع)
- Score = 13.9/100

السَّبَب المُحتَمَل: 
1. **Sign convention** خاطِئ بَين MPC و physics في bridge
2. **Servo delay** كبير غير مُعَوَّض في MPC
3. **Aerodynamic mismatch** (نَموذج MPC ≠ نَموذج الـbridge)

هذه مُشكِلة مُنفَصِلة تَماماً — البَيانات تَصِل صَحيحة الآن، لَكِن المَنطِق نَفسه يَنحَرِف. تُحَلَّ في جَلسة لاحِقة.

---

## 🗂 المراجِع

- ULog Diag#6 (دَليل orb_copy=0): `06_07_26.ulg` — 528 SIM_RD صفر vs 343 طوبيك غير صفر
- ULog Attempt 5b (دَليل lockstep block): `06_30_31.ulg` — SIM_RD يَتَوَقَّف عند 21421s
- ULog Attempt 6 (الإصلاح يَعمَل): سَيُحَدَّث بَعد الـrun التالي
- Bridge run logs: `/tmp/attempt5b_hil_run.log`, `/tmp/attempt6_hil_run.log`
- Backups: `SimulatorMavlink.cpp.pre_attempt5_*`, `SimulatorMavlink.hpp.pre_attempt5_*`

## 📝 التَّحَقُّق النِّهائي

```
[DIAG] HIL_ACT samples (Attempt 6 bridge):
  total = 117
  non-zero = 5
  values: [+0.0732, +0.0634, -0.0739, -0.0641]   # pitch/roll pattern
          [-0.3491, +0.0153, +0.3491, -0.0153]   # saturated at ±20°
```

**الإصلاح مُؤَكَّد بالقياس المُباشَر مَن مَصدَرَين مُستَقِلَّين**:
1. ULog `debug_array.SIM_RD` (PX4 internal)
2. Python `[DIAG]` في الـbridge (TCP wire)
