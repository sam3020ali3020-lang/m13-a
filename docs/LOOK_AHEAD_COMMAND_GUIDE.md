# Look-Ahead Command — الشرح الكامل والدليل العملي

> دليل تعويض تأخير النقل (CAN + Servo MCU) في حلقة MPC للصاروخ M130
> بدون تعديل نموذج الـ MPC وبدون زيادة في `NX`.

---

## 1. الفكرة باختصار

بدلاً من تعديل نموذج الـ MPC ليحوي حالات تأخير (وهو ما سبّب لنا عدم استقرار في الحل)،
نستفيد من أن MPC أصلاً يحسب **مسار تحكم كامل للمستقبل** (`u_future` عبر `N` مراحل).

- **التقليدي**: نُرسل أمر المرحلة الأولى `u[0]` للمشغّل (servo).
- **Look-Ahead**: نُرسل أمر مرحلة لاحقة `u[k]` حيث `k * dt ≈ تأخير النقل الكلي`.

وهكذا حين يصل الأمر فعلياً إلى الدفّة بعد التأخير، يكون هو نفس الأمر الذي خطّط له MPC
لتلك اللحظة بالضبط.

---

## 2. لماذا هذا أنسب حل لحالتنا

| البديل | المشكلة في حالتنا |
|---|---|
| **Smith Predictor** | يتطلب نموذج تنبؤ منفصل وضبط دقيق؛ حسّاس لعدم تطابق النموذج |
| **توسيع نموذج MPC بـ delay buffers** (cascade / shift register) | جرّبناه: `N=4` سبّب `MINSTEP errors` وعدم استقرار في الـ QP بسبب الصلابة (stiffness) مقارنة بفترة التحكم. أيضاً يضخّم `NX` من 18 إلى 24/30 ويُبطئ الحل |
| **Look-Ahead Command** | مجاني حسابياً، لا يغيّر النموذج، لا يزيد `NX`، لا يكسر الاستقرار العددي، ويستفيد مباشرة من المسار المتوقع الذي يحسبه MPC أصلاً |

ولأن MPC في حالتنا يحلّ trajectory كاملة (`N=40` stage) كل دورة،
فالأوامر المستقبلية متوفرة **مجاناً** في ذاكرة الحل.

---

## 3. نموذج التأخير في نظامنا

تأخير النقل الكلي من خرج MPC حتى تحرك الدفّة فعلياً:

```
t_total ≈ t_CAN_TX + t_servo_MCU_processing + t_servo_mech_lag
        ≈ 110 ms (مقاس)
```

عند `dt = 20 ms` في MPC على الهاتف:

```
lookahead_stage = round(t_total / dt) = round(110 / 20) = 6   →   ~120 ms
```

---

## 4. كيف يعمل عملياً

دورة MPC تحسب:

```
u[0], u[1], u[2], ..., u[N-1]      (الأزمنة: t, t+dt, t+2dt, ...)
```

**السلوك التقليدي**: نُرسل `u[0]` ← يصل بعد ‎110ms ← متأخر عن نية MPC.

**سلوك Look-Ahead**: نُرسل `u[6]` ← يصل بعد ~120ms ← يتزامن مع نية MPC للحظة `t + 120ms`.

```
الزمن الحقيقي:   t ────────────────► t+120ms
MPC خطّط:                              u[6]
نُرسل الآن u[6] ──CAN──servo──► يصل عند t+120ms ✓
```

---

## 5. التطبيق في الكود

### 5.1 إعدادات التحكم (config struct)

`AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/mpc_controller.h`

```cpp
struct MpcConfig {
    // ...
    int lookahead_stage = 1;  // 1 = legacy; >1 = compensate transport delay
};
```

### 5.2 استخراج أمر المرحلة المستقبلية

`AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/mpc_controller.cpp`

```cpp
int stage = config_.lookahead_stage;
if (stage < 1)             stage = 1;
if (stage >= N_HORIZON)    stage = N_HORIZON - 1;

double u_out[NU];
ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_,
                stage, "u", u_out);   // بدل stage=1 الثابت
```

### 5.3 التهيئة في `RocketMPC.cpp`

`AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/RocketMPC.cpp`

```cpp
MpcConfig cfg;
// ...
cfg.lookahead_stage = 6;   // 6 stages × 20ms = 120ms
```

> ملاحظة: مؤقتاً تم تثبيت القيمة في الكود (hard-coded) لأن
> `px4_parameters.hpp` المُولَّد ليس متزامناً مع تعريف بارامتر `ROCKET_MPC_LA`.
> لاحقاً يمكن تحويلها إلى parameter قابل للضبط في وقت التشغيل.

---

## 6. كيفية ضبط القيمة (Tuning)

### الخطوة 1: قِس التأخير الفعلي

من تحليل ULog في HIL، ارسم:

- خرج MPC (`actuator_controls_0.control[*]`)
- تغذية راجعة من السيرفو (`servo_feedback`)

التأخير = الفارق الزمني بين بداية الأمر وبداية حركة الدفّة
(مع طرح الـ rise time الميكانيكي للسيرفو).

### الخطوة 2: احسب المرحلة

```
lookahead_stage = round(measured_delay_ms / mpc_dt_ms)
```

### الخطوة 3: تحقّق

- إن كان `lookahead_stage` صغيراً جداً → ستبقى الدفّات متأخرة عن الأمر، تذبذب وتجاوز (overshoot)
- إن كان كبيراً جداً → ستتحرك الدفّات قبل الحاجة، تذبذب عالي التردد

### الخطوة 4: التحقق العملي

| مؤشر النجاح | القيمة المستهدفة |
|---|---|
| Servo tracking RMSE | ينخفض بنسبة >50% مقارنة بـ `stage=1` |
| HIL score | ≥ 85/100 |
| Standalone score (بدون تأخير) | يبقى ≥ 95/100 (لأن المحاكاة بلا CAN) |

---

## 7. القيود والمحاذير

1. **افتراض أن المسار المستقبلي صحيح**
   إذا ساء النموذج جداً، فأمر المرحلة 6 قد لا يعكس الواقع. يخفّف من هذا أن MPC يُعاد حلّه
   كل دورة (receding horizon)، لكن في الحالات الديناميكية الشديدة قد يحتاج تخفيض
   `lookahead_stage`.

2. **التأخير المتغيّر**
   إذا تذبذب التأخير الفعلي (±20ms) فالأفضل اختيار قيمة متوسطة، أو تطبيق
   **interpolation** بين `u[k]` و `u[k+1]`.

3. **الاستقرار الحدّي**
   في طائرات/صواريخ شديدة الحساسية، تعويض كامل للتأخير قد يقترب من حدّ الاستقرار.
   ابدأ بـ ~70% من القيمة المحسوبة (مثلاً 4 بدل 6) ثم ارفع.

4. **شرط `lookahead_stage < N_HORIZON`**
   ضروري؛ عندنا `N=40` و `stage=6` فهامش الأمان كبير.

---

## 8. الفروق عن البدائل

### 8.1 مقابل توسيع النموذج بـ delay buffers

| الجانب | Look-Ahead | Delay-Augmented Model |
|---|---|---|
| تغيير `NX` | لا | نعم (18 → 24 أو 30) |
| تغيير في `m130_acados_model.py` | لا | نعم (cascade states) |
| استقرار QP | محفوظ | معرض لـ MINSTEP |
| زمن الحل | لا يتأثر | يزداد |
| دقة التعويض | جيدة (رهينة بدقة المسار المتوقع) | أعلى نظرياً |
| سهولة الضبط | عالية (رقم واحد) | منخفضة (نموذج كامل) |

### 8.2 مقابل Smith Predictor

| الجانب | Look-Ahead | Smith Predictor |
|---|---|---|
| نموذج إضافي | لا | نعم (predictor) |
| حساسية لعدم تطابق النموذج | متوسطة | عالية |
| التعقيد | منخفض جداً | متوسط |
| التكامل مع MPC الحالي | سطر واحد | إعادة هيكلة |

---

## 9. ملخص تنفيذي

- التأخير الكلي ≈ **110 ms** (CAN + servo MCU)
- `mpc_dt = 20 ms`  →  `lookahead_stage = 6`
- التطبيق: استخراج `u[6]` من حل acados بدل `u[1]`
- **بدون** تعديل نموذج MPC، **بدون** زيادة `NX`، **بدون** عدم استقرار عددي
- التكلفة الحسابية: **صفر** (الحل موجود أصلاً في `nlp_out`)

---

## 10. مراجع داخل المستودع

- `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/RocketMPC.cpp`
- `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/mpc_controller.cpp`
- `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/mpc_controller.h`
- `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/rocket_mpc_params.c`
- `6DOF_v4_pure/config/6dof_config_advanced.yaml`
- `6DOF_v4_pure/hil/hil_runner.py`
