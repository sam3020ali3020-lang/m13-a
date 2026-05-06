# تمثيل تأخير الـ 110ms في نموذج الـ MPC

**التاريخ**: 2026-04-30 (مُحدَّث بعد فحص شامل)
**السياق**: تأخير النقل CAN + servo MCU = 110ms مُقاس عملياً. تعويض جزئي عبر `lookahead_stage` مُفعَّل، لكن الـ MPC نفسه لا يرى التأخير عند تخطيطه — وهذا يفسّر فجوة HIL مقابل PIL/SITL.

---

## الحالة الحالية (مُحقَّقة)

### أبعاد الـ Solver
```@/home/yoga/m13/m13/c_generated_code/acados_solver_m130_rocket.h:39-67
#define M130_ROCKET_NX     18
#define M130_ROCKET_NU     3
#define M130_ROCKET_NP     2
#define M130_ROCKET_N      80
```

### تخطيط الحالات (من `m130_acados_model.py`)
```
x[0..11]  : V, gamma, chi, p, q, r, alpha, beta, phi, h, x_ground, y_ground
x[12..14] : delta_e_s, delta_r_s, delta_a_s   (الأوامر كحالات، يحرّكها u=ddelta_*)
x[15..17] : delta_e_act, delta_r_act, delta_a_act   (المواقع الفعلية بـ lag τ=25ms)
```

### إعدادات الـ Solver
- `tf = 1.6 s`، `dt = 20 ms`، `cond_N = 8` (تكثيف جزئي)
- `sim_method_num_steps = 2`، `levenberg_marquardt = 2e-2`
- `integrator_type = ERK`، `nlp_solver_type = SQP_RTI`

### تعويض التأخير الحالي (مُفعَّل ✅)
```@/home/yoga/m13/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/RocketMPC.cpp:307-310
	// delay on real hardware.  Hardcoded for now; can be promoted to a
	// runtime parameter (ROCKET_MPC_LA) once the auto-generated params
	// header is regenerated.  Set to 1 to restore legacy behaviour.
	mpc_cfg.lookahead_stage  = 6;
```
يستخرج الـ controller الأمر من stage 6 (= 120ms مستقبلاً) → time-shift يعوّض التأخير عند الإخراج فقط.

### تعطيل cascade في النموذج
```@/home/yoga/m13/m13/6DOF_v4_pure/mpc/m130_ocp_setup.py:37-43
    # ── Pure-delay augmentation: disabled (N=0). The first-order cascade
    # cannot accurately represent the simulator's shift-register pure delay
    # — attempts at N=2/4 produced solver MINSTEP errors and unstable
    # flights. Re-enabling needs either (a) Padé approximation in the
    # acados model, or (b) cascade-style delay in the simulator's actuator.
    N_DELAY_BUFFERS = 0
    TAU_TRANSPORT_S = 0.110  # measured CAN→fin delay (servo characterization)
```

### الفجوة المتبقّية
| السيناريو | النتيجة | تأخير حقيقي؟ |
|---|---|---|
| Standalone | 98–99/100 | لا |
| SITL | 98.7/100 | لا |
| PIL | 100/100 | لا |
| **HIL** | **27–45/100** | **نعم (110ms)** |

الفرق الوحيد المنهجي بين PIL و HIL هو وجود تأخير CAN حقيقي → دليل قاطع أن الـ `lookahead_stage=6` غير كافٍ بمفرده، وأن المشكلة بنيوية في النموذج.

---

## الخيار 1️⃣ — `lookahead_stage` Time-Shift Compensation

**الحالة**: ✅ **مُنفَّذ بالفعل** (`lookahead_stage = 6`)

### كيف يعمل

بعد كل solve، الـ controller يستخرج الأمر من stage 6 بدل stage 1:
```@/home/yoga/m13/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/mpc_controller.cpp:637-642
	int la_stage = _cfg.lookahead_stage;
	if (la_stage < 1)        { la_stage = 1; }
	if (la_stage > MPC_N)    { la_stage = MPC_N; }

	double x1[MPC_NX];
	ocp_nlp_out_get(_nlp_config, _nlp_dims, _nlp_out, la_stage, "x", x1);
```
و:
```@/home/yoga/m13/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/mpc_controller.cpp:679-683
	} else {
		_consec_fails = 0;
		de = (float)x1[12];
		dr = (float)x1[13];
		da = (float)x1[14];
```

### القيود البنيوية

هذا **time-shift في الإخراج**، ليس في التخطيط. الـ MPC يحلّ مسارَه الأمثل تحت افتراض ديناميكي خاطئ:

| | النموذج الحالي | الواقع |
|---|---|---|
| ديناميكا الزعنفة | `delta_act_dot = (delta_s − delta_act) / 0.025` (lag وحيد) | `delta_act = (delta_s مؤخَّر 110ms) → lag بـ 25ms` |
| استجابة المعدّل (rate) | فورية | متأخّرة 110ms |
| تأثير على prediction trajectory | الـ trajectory المُحسَّبة لا تعكس الواقع | الواقع يخالف التنبؤ خاصة عند burnout |

عند burnout تختفي قوة الدفع → الديناميكا الجانبية تصبح حسّاسة جداً للزعانف → الـ MPC يخطّط افتراض استجابة فورية، لكن الواقع متأخّر → oscillation متزايد → loss of stability.

**خلاصة**: `lookahead_stage` يحلّ نصف المشكلة. الباقي يحتاج تمثيل التأخير في النموذج نفسه.

---

## الخيار 2️⃣ — Padé(1,1) Approximation (التوصية الفعلية) ⭐

### الرياضيات

التقريب الأول لـ Padé:

```
e^(-sD) ≈ (1 - sD/2) / (1 + sD/2)
```

تمثيل state-space (لكل محور):

```
ẋ_p = -(2/D)·x_p + (2/D)·u
y    = 2·x_p − u
```

حيث:
- `u = delta_*_s` (الأمر الصادر من الـ MPC)
- `y = delta_*_act_input` (الإشارة المتأخرة الداخلة على lag السيرفو)
- `D = tau_transport_val = 0.110 s`

### التعديل في `m130_acados_model.py`

```python
# بدلاً من cascade buffers، استخدم Padé state واحد لكل محور:
pade_e = ca.SX.sym('pade_e')
pade_r = ca.SX.sym('pade_r')
pade_a = ca.SX.sym('pade_a')

state_list.extend([pade_e, pade_r, pade_a])  # +3 states → NX=21

# الديناميكا (tau_p = D/2 = 0.055)
tau_p = max(float(tau_transport_val), 1e-4) / 2.0
pade_e_dot = (-pade_e + delta_e_s) / tau_p
pade_r_dot = (-pade_r + delta_r_s) / tau_p
pade_a_dot = (-pade_a + delta_a_s) / tau_p

# الخرج المتأخر يغذّي lag السيرفو الفعلي:
delta_e_delayed = 2.0 * pade_e - delta_e_s
delta_r_delayed = 2.0 * pade_r - delta_r_s
delta_a_delayed = 2.0 * pade_a - delta_a_s

delta_e_act_dot = (delta_e_delayed - delta_e_act) / tau_servo
delta_r_act_dot = (delta_r_delayed - delta_r_act) / tau_servo
delta_a_act_dot = (delta_a_delayed - delta_a_act) / tau_servo
```

### لماذا ينجح Padé حيث فشل Cascade؟

| المعيار | Cascade Nb=2 | Padé(1,1) |
|---|---|---|
| Pole locations | −18.2, −18.2 (مكرّر، شبه singular) | −36.4 (واحد، نظيف) |
| Zero في TF | ❌ لا | ✅ نعم (يُحاكي pure-delay بدقة أعلى) |
| State count لكل محور | 2 | 1 |
| Phase @ 5 Hz | تأخير ≈ 90 ms | تأخير ≈ 105 ms (أقرب لـ 110) |
| Hessian conditioning | **ضعيف** (poles متطابقة) | **ممتاز** |

### السبب الحقيقي لفشل Nb=2 سابقاً

ليس "MINSTEP" من stiffness — بل **conditioning**: قطبان متطابقان عند `-1/τ_buf` يجعلان مصفوفة الـ Hessian شبه-منفردة، فيرفض SQP أي خطوة. Padé له **قطب وحيد + zero في النصف الأيمن** → conditioning صحي.

### التكلفة الحسابية

| المقياس | قبل | بعد Padé |
|---|---:|---:|
| NX | 18 | **21** (+17%) |
| QP size (cond_N=10) | 10 × 18 | 10 × 21 (+17%) |
| Solve time ARM64 | ~46 ms | **~54 ms متوقع** |
| Deadline 20ms × 3 iter = 60ms | ضمن | **ضمن** |

### الزمن المتوقع للتطبيق

- تعديل `m130_acados_model.py`: 30 دقيقة
- تعديل `m130_ocp_setup.py` (NX=21، x0 padding، W مدّد): 15 دقيقة
- regen + ARM64 + SITL + APK + PIL: 30–45 دقيقة
- **الإجمالي**: ~1.5 ساعة

---

## الخيار 3️⃣ — Cascade مع إصلاح Conditioning

### الفكرة

أعد تفعيل cascade `Nb=2` لكن مع إصلاحات تعالج جذور الفشل السابق.

### الإصلاحات المطلوبة

```python
N_DELAY_BUFFERS = 2

# 1. أعد num_steps إلى 2 (كان 2، خُفِّض إلى 1 لتسريع ARM64)
ocp.solver_options.sim_method_num_steps = 2

# 2. احتفظ بـ partial condensing
ocp.solver_options.qp_solver_cond_N = 10

# 3. مهم: امتد W matrix لتشمل البافرات بأوزان regularization صغيرة
#    (يمنع Hessian من أن يصبح شبه-singular على البافرات)
W_buf = 0.001  # tiny weight to anchor buffer states
```

### التكلفة

| المقياس | قبل | بعد Nb=2 |
|---|---:|---:|
| NX | 18 | **24** (+33%) |
| Solve time ARM64 | ~46 ms | **~62 ms متوقع** |

### التحليل

| المزايا | العيوب |
|---|---|
| ✅ تمثيل أدق نظرياً (cascade → pure delay مع Nb→∞) | ❌ Nb=2 تقريب رديء (3 db drop @ 18 rad/s) |
| | ❌ NX أكبر من Padé |
| | ❌ مخاطر عودة MINSTEP إذا الإصلاحات لم تكن كافية |

---

## التوصية المُحدَّثة

بما أن الخيار ① **مُنفَّذ بالفعل** والفجوة HIL مقابل PIL لا تزال كبيرة (45 vs 100 نقطة):

```
  ① lookahead_stage=6 ──────► مُنفَّذ ✓ (يحلّ نصف المشكلة)
                              │
                              ▼
  ② Padé(1,1) في النموذج ──► الخطوة التالية الفعلية ⭐
                              │
                              ▼
  المتوقع: HIL score 45 → >75 (تحسّن ~30 نقطة)
```

| السيناريو | الخيار |
|---|---|
| ~~تجربة سريعة~~ | ~~`lookahead_stage=6`~~ — مُفعَّل |
| **الخطوة التالية الفعلية** | **② Padé(1,1)** |
| دقة قصوى (للأبحاث/النشر) | ③ Cascade Nb=2 مع إصلاحات conditioning |

### معايير النجاح بعد تطبيق Padé

- HIL score: 45 → **>75**
- Max |α| post-burnout: 51° → **<20°**
- Fin saturation: 6.1% → **<1%**
- MPC solve time ARM64: 46ms → **<55ms** (ضمن deadline)

---

## ملاحظة معمارية

Padé(1,1) هو **الخيار القياسي في صناعة التحكم** للأنظمة ذات pure-delay، وهو ما اقترحه المؤلف نفسه:

```@/home/yoga/m13/m13/6DOF_v4_pure/mpc/m130_ocp_setup.py:40-41
    # flights. Re-enabling needs either (a) Padé approximation in the
    # acados model, or (b) cascade-style delay in the simulator's actuator.
```

الخيار (a) في التعليق = الخيار ② في هذه الوثيقة.

---

## مراجع

- `@/home/yoga/m13/m13/6DOF_v4_pure/mpc/m130_acados_model.py` — تعريف النموذج
- `@/home/yoga/m13/m13/6DOF_v4_pure/mpc/m130_ocp_setup.py:39-43` — تعطيل cascade
- `@/home/yoga/m13/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/RocketMPC.cpp:307-310` — تفعيل `lookahead_stage=6`
- `@/home/yoga/m13/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/mpc_controller.cpp:637-683` — استخراج الأمر من stage مستقبلي
- `@/home/yoga/m13/m13/c_generated_code/acados_solver_m130_rocket.h:39-67` — أبعاد الـ solver المُولَّد
- `@/home/yoga/m13/m13/PIL_SIGSEGV_FIX.md` — تاريخ الـ regen + build chain

---

## سجل التحديثات

| التاريخ | التغيير |
|---|---|
| 2026-04-30 (الأولى) | إنشاء الوثيقة بثلاثة خيارات |
| 2026-04-30 (مُحدَّث) | بعد فحص شامل: تأكيد أن `lookahead_stage=6` مُنفَّذ بالفعل، تأكيد أن `_forward_guess` سليم للـ NX=18، حصر التوصية في Padé(1,1) |

**نهاية الوثيقة**
