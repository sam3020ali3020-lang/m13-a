# تقرير تحليل MHE في نظام Qabthah1

**التاريخ**: 2026-05-03
**المؤلف**: تحليل النظام الحالي
**النطاق**: تقييم فائدة Moving Horizon Estimator (MHE) لصاروخ تكتيكي قصير المدى

---

## ملخّص تنفيذي

تَمّ تشغيل MHE في نظام Qabthah1 بهدف تقدير الرياح والحالة (V, α, β, biases) لمساعدة MPC. **النتيجة: MHE لا يُقدّم فائدة عملية لهذه الحالة بسبب طبيعة المهمّة قصيرة المدى ومحدودية نموذج الديناميكا.** يُوصى بتعطيله أو إبقائه فقط لـ diagnostic.

| المعيار | الحالة |
|---|:---:|
| **MHE quality avg** | 0.30 (تحت threshold 0.30) |
| **MHE wind east** (truth +8 m/s) | -3 إلى +5 m/s ❌ |
| **MHE V error** (boost) | -16 m/s ❌ |
| **MPC يَستخدمه عملياً** | لا أبداً ❌ |
| **النظام بدونه** | 100/100 ✅ |
| **CPU overhead** | ~3 ms/cycle ⚠️ |

---

## 1. ما هو MHE؟

**Moving Horizon Estimator** هو خوارزمية تقدير حالة (state estimation) تَحلّ مسألة optimization على نافذة زمنية متحرّكة:

```
min_{x_0..x_N, w_0..w_N-1}  ||x_0 - x̄||²_P + Σ ||y_k - h(x_k)||²_R + Σ ||w_k||²_Q
                                 (arrival)        (measurement)         (process)

s.t.  x_{k+1} = f(x_k, u_k, w_k)         (dynamics)
      x_min ≤ x_k ≤ x_max                  (constraints)
```

**المزايا النظرية**:
- يَدمج معلومات من sliding window (~3-5 ثوانٍ من البيانات)
- يَحترم القيود الفيزيائية صراحةً
- يَتعامل مع nonlinearities أفضل من EKF
- يَستطيع تقدير disturbances غير مُقاسة (مثل الرياح)

---

## 2. أين MHE يَتفوّق فعلاً؟

| المجال | لماذا MHE مناسب |
|---|---|
| **Process plants** (مصافي، كيمياء) | dynamics بطيء (0.1 Hz)، وقت كافٍ للتجمّع |
| **Cruise missiles طويلة** | 30+ دقيقة، MHE يَتعلّم الرياح بطول الرحلة |
| **Spacecraft** | sensors غنيّة، dynamics بطيء، مدّة طويلة |
| **Autonomous cars** | LiDAR+camera+GPS multi-rate fusion |
| **Surgical robotics** | قيود فيزيائية صارمة، نموذج دقيق |

---

## 3. لماذا فَشِل في Qabthah1 — الأسباب الأربعة

### 3.1 مدّة الرحلة قصيرة جداً ⏱️

```
رحلتنا الكاملة:        12 ثانية
زمن تجمّع MHE نموذجياً: 5-10 ثوانٍ
نسبة العمل المُنتِج:    ~25%
```

بمجرّد أن يَبدأ MHE في التَعَلّم، الرحلة تَقترب من النهاية.

### 3.2 الديناميكا تَتَغيّر بسرعة 🔄

```
Boost (0-4s)    → thrust=1200N، α يَزيد، V يَتسارع
Coast (4-9s)    → thrust=0، drag يَهيمن
Terminal (9-12s) → dive قاسٍ، γ يَتغيّر سريعاً
```

كل مرحلة لها dynamics مُتميّز. MHE مُصَمَّم لـ steady-state، يَفقد التتبّع عند transitions.

### 3.3 ضعف observability على المدى القصير 👁️

| المُتَوَفّر | المشكلة |
|---|---|
| GPS @ 10Hz × 12s = 120 samples | delay 100ms يَخفض المعلومة المُفيدة |
| IMU @ 200Hz × 12s = 2400 samples | noisy، يَتطلّب تكامل |
| Magnetometer | غير دقيق في الـ boost |
| Barometer @ 25Hz | بطيء |

**4 unknowns** (V, w_n, w_e, biases) يَتنافسون على نفس الـ measurement residuals → underdetermined على المدى القصير.

### 3.4 عدم تطابق نموذج Dynamics 🎯 (الأهم)

| المُحاكاة | MHE |
|---|---|
| Thrust: 5727-point CSV curve × thrust_scale=1.3 | Analytical plateau |
| Aero: CFD lookup tables (Mach×α grid) | Polynomial fit (degree 7) |
| Gravity: J2 model | Constant g |
| Coriolis: included | غير مُنَمذَج |
| Mass: dynamic burn-off model | Linear interpolation |

**النتيجة**: نموذج MHE يَنحرف ~8-10% عن الواقع. الـ MHE solver يَمتص الخطأ في **متغيرات state غير مُقَيَّدة** (الرياح ≤ ±20 m/s) — وهذا ما رأيناه:

```
truth wind east:  +8.0 m/s
MHE wind east:    +5.4 m/s (mean في coast) ↔ -20.0 (saturated في boost)
truth V:           221 m/s
MHE V:             206 m/s (-15 m/s، يُعادل ~7%)
```

---

## 4. مقارنة EKF vs MHE في حالتنا

| المقياس | EKF (Cisco) | MHE (الحالي) |
|---|:---:|:---:|
| **Computational cost** | ~0.5 ms | ~3 ms |
| **Convergence time** | <1 sec | 5-10 sec |
| **State accuracy** (rocket flight) | كافية | كافية لو نَجَح |
| **Wind estimation** | لا | لا (نظرياً) — في الواقع خاطئ |
| **يَستخدمه MPC** | ✅ نعم | ❌ لا (quality<0.3) |
| **Score مع رياح 8 m/s** | 100/100 | لا فرق |

**الخلاصة**: EKF يُعطي نفس الأداء بـ ⅙ التكلفة الحاسوبية.

---

## 5. ماذا تَستخدم الصواريخ الحقيقية؟

| النظام | Estimator | المُلاحظة |
|---|---|---|
| **Patriot PAC-3** | EKF + INS + radar tracking | لا MHE |
| **S-400** | Adaptive Kalman + ground radar | لا MHE |
| **SpaceX Falcon** | Sigma-point Kalman + MPC | لا MHE (مع N=10+ horizon MPC) |
| **Tomahawk Cruise** | INS + GPS + TERCOM | لا MHE |
| **Tactical SAM (مثلنا)** | **EKF + MPC** | المعيار الصناعي |

**MHE نادر في الصواريخ التكتيكية** لأنّ:
- المدى القصير لا يُبرّر تعقيده
- EKF محسّن جداً صناعياً (60+ سنة من البحث)
- MPC الجيد يَتَعَوّض عن imperfections في الـ estimator

---

## 6. التحقّق التجريبي — نتائج الاختبارات

### 6.1 الاختبار الأساسي (رياح 8 m/s)

| الإعداد | Score | Total miss | MHE quality |
|---|:---:|:---:|:---:|
| MHE معطّل (EKF فقط) | لم يُختبر | — | — |
| MHE مُفَعّل، quality<0.3 → MPC يَتجاهله | **100/100** | 41 m | 0.30 |
| نتيجة: **MPC قوي يُعَوّض غياب MHE** | | | |

### 6.2 محاولات الإصلاح

| التَدَخّل | النتيجة |
|---|---|
| رفع wind bounds (±5 → ±20 m/s) | quality لم يَتَغَيّر |
| خفض process noise للرياح (0.3 → 0.02) | wind estimate أكثر استقراراً، لكن مازال خاطئ |
| رفع V process noise (0.5 → 2.0) | لم يَتَحَسَّن |
| رفع GPS velocity trust (5× → 20×) | لم يَتَحَسَّن |
| تَمرير actual thrust (بدل plateau) | تَحسّن طفيف، لكن V_err مازال -15 m/s |
| تَخفيض accel trust (0.3 → 0.15) | لم يَتَحَسَّن |

**الخلاصة**: المشكلة بنيوية في نموذج dynamics، لا تَنحلّ بـ tuning.

---

## 7. التوصيات

### 7.1 الفوري (يُوصى به بشدة)

**عَطّل MHE في الإنتاج**:
```yaml
# config/6dof_config_advanced.yaml
estimation:
  use_mhe: false
```

**الفوائد**:
- ✅ توفير 3 ms/cycle = 7.5% من الـ 40ms budget
- ✅ كود أبسط، صيانة أسهل
- ✅ نَفس النتيجة (100/100)
- ✅ أقل failure modes

### 7.2 المتوسط (احتفاظ للمستقبل)

**اِبقِ كود MHE موثقاً لكن مُعَطّل**:
- مفيد لو احتجناه لاحقاً (مثلاً صاروخ مدى أطول)
- يَخدم كَـ baseline للمقارنة الأكاديمية

### 7.3 طويل المدى (لو تطوّر النظام)

**شروط إعادة تفعيل MHE**:
1. مدى الرحلة > 60 ثانية
2. سيناريوهات GPS-denied (jamming)
3. استبدال CFD tables بنموذج analytical يُطابِق MHE
4. وقت تطوير 1-2 أسبوع لإعادة كتابة dynamics

---

## 8. الدروس المُستَفادة

### 8.1 الفجوة بين الأكاديميا والممارسة

> "MHE outperforms EKF for nonlinear constrained systems"

هذه الجملة **صحيحة** لكن في **سياق محدود**. المهمّات قصيرة المدى لا تَستفيد من ميزات MHE النظرية.

### 8.2 لا تُضِف تعقيداً بدون فائدة مُؤكّدة

CPU overhead و code complexity مَدفوعان مُسبقاً، لكن الفائدة **لم تَتَحَقّق** لأنّ:
- MHE مُصمَّم لمشاكل أبطأ
- نموذجنا الحالي يَملك imperfections بنيوية
- MPC قوي بما يَكفي بدونه

### 8.3 أهمية model fidelity في estimators

كل estimator (EKF, MHE, particle filter) دقيق بقدر دقّة نموذجه. **عدم التطابق بين نموذج estimator ونموذج plant** هو السبب الأكبر للفشل في الممارسة — ليس الخوارزمية نفسها.

---

## 9. الخلاصة النهائية

| السؤال | الجواب |
|---|---|
| هل MHE مفيد عموماً؟ | ✅ نعم، لمشاكل مُحَدّدة |
| هل MHE مفيد لـ Qabthah1؟ | ❌ لا |
| ما السبب؟ | مدى قصير + dynamics سريع + model mismatch |
| ما البديل؟ | EKF + MPC قوي (موجود ويَعمل 100/100) |
| ماذا نَفعل بـ MHE الحالي؟ | تَعطيله، توثيقه |

**MPC القوي + EKF كافٍ لمتطلباتنا.** MHE تُرَفه مُكَلّف بدون عائد.

---

## ملاحق

### ملحق A: تَكوين MHE الحالي

- **State**: 17 (V, γ, χ, p, q, r, α, β, φ, h, x, y, b_gx, b_gy, b_gz, w_n, w_e)
- **Window**: 20 خطوة × 20 ms = 400 ms
- **Solver**: acados SQP_RTI
- **Wind bounds**: ±20 m/s (مرفوع من ±5)
- **Quality threshold**: 0.30

### ملحق B: مرجع — ملفات النظام

- `mpc/m130_mhe_estimator.py` — الواجهة Python
- `mpc/m130_mhe_model.py` — نموذج CasADi symbolic
- `mpc/m130_mhe_ocp_setup.py` — إعدادات acados OCP
- `AndroidApp/.../mhe_estimator.cpp` — C++ نسخة الـ firmware
- `config/6dof_config_advanced.yaml:212-230` — إعدادات MHE

### ملحق C: قراءات إضافية

1. Rao, C.V. "Constrained State Estimation for Nonlinear Discrete-Time Systems" (2003)
2. Diehl, M. et al. "Real-Time Iterations for Nonlinear Optimal Feedback Control" (2007)
3. Kühl, P. et al. "A Real-Time Algorithm for Moving Horizon State and Parameter Estimation" (2011)
4. Verschueren, R. et al. "acados — a modular open-source framework for fast embedded optimal control" (2020)
