# ربط MHE بـ MPC وآلية عمل acados في النظام
**MHE ↔ MPC Integration & acados Internals**

> Version 1.0 — April 2026

---

## فهرس المحتويات

1. [مقدمة: لماذا MHE + MPC؟](#1-مقدمة-لماذا-mhe--mpc)
2. [دورة حياة البيانات الكاملة](#2-دورة-حياة-البيانات-الكاملة)
3. [MHE → MPC: آلية الربط بالكود](#3-mhe--mpc-آلية-الربط-بالكود)
4. [بوابة الجودة والحماية من الفشل](#4-بوابة-الجودة-والحماية-من-الفشل)
5. [تقدير الرياح وتعويض الاتجاه](#5-تقدير-الرياح-وتعويض-الاتجاه)
6. [acados: من التعريف الرمزي إلى الحل الآني](#6-acados-من-التعريف-الرمزي-إلى-الحل-الآني)
7. [acados لـ MPC: البنية الداخلية](#7-acados-لـ-mpc-البنية-الداخلية)
8. [acados لـ MHE: البنية الداخلية](#8-acados-لـ-mhe-البنية-الداخلية)
9. [التفاعل بين الحلّين في الزمن الحقيقي](#9-التفاعل-بين-الحلين-في-الزمن-الحقيقي)
10. [الدروس المستفادة والقيود](#10-الدروس-المستفادة-والقيود)

---

## 1. مقدمة: لماذا MHE + MPC؟

### المشكلة

الصاروخ M130 يحتاج نظام تحكم يعمل في الزمن الحقيقي مع:
- **حساسات مشوّشة:** الجيروسكوب والتسارعميتر فيها ضوضاء وانحياز
- **بيئة غير معروفة:** رياح مجهولة السرعة والاتجاه
- **قيود فيزيائية:** زعانف محدودة المدى (±20°) والسرعة (250°/s)
- **ديناميكا سريعة:** سرعة الصاروخ تتغير من 30 م/ث إلى 800+ م/ث خلال ثوانٍ

### الحل: MHE + MPC

```
الواقع الحقيقي          ←    لا يمكن قياسه مباشرة
        │
        ▼
  ┌──────────────┐        ┌──────────────┐
  │  حساسات      │ y[13]  │   MHE        │
  │  (مشوّشة)    │──────►│  (acados)    │
  │              │        │  "ماذا حدث؟" │
  └──────────────┘        └──────┬───────┘
                                 │ x̂[17] + wind + bias
                                 ▼
                          ┌──────────────┐
                          │   MPC        │
                          │  (acados)    │
                          │  "ماذا أفعل?"│
                          └──────┬───────┘
                                 │ fins[4]
                                 ▼
                          ┌──────────────┐
                          │  الصاروخ     │
                          │  (6DOF)      │
                          └──────────────┘
```

| المكون | السؤال الذي يُجيب عليه | الأداة | المعدل |
|--------|------------------------|--------|--------|
| **MHE** | "ما هي حالة الصاروخ الحقيقية؟" | acados SQP_RTI | 50 Hz |
| **MPC** | "ما هي أوامر الزعانف المثلى؟" | acados SQP_RTI | 50 Hz |

---

## 2. دورة حياة البيانات الكاملة

### التسلسل الزمني لخطوة واحدة

```
الزمن t                                      الزمن t + dt (10ms)
════════                                      ════════════════════

 ┌─────────────────────────────────────────────────────────────┐
 │  rocket_6dof_sim.py :: simulate()                           │
 │                                                             │
 │  ❶ التحكم: _mhe_control_fn(state_dict, t)                  │
 │     │   ◄── يحقن state_dict['mhe_output'] من الخطوة السابقة│
 │     │                                                       │
 │     └──► MpcController.control_function(state_dict, t)      │
 │          │                                                  │
 │          ├── يستخرج x_mpc[18] من state_dict                │
 │          ├── يقرأ mhe_output (إن وُجد)                     │
 │          ├── يمزج الموقع (h, x, y) من MHE                  │
 │          ├── يحسب التوجيه (γ_ref, χ_ref)                   │
 │          ├── يحل acados MPC → u_opt[3]                     │
 │          └── يُرجع fins[4]                                 │
 │                                                             │
 │  ❷ التكامل: _integrate_one_step(state, t, dt, ctrl)        │
 │     └── RK4 مع 4 مراحل → next_state[14], snapshot         │
 │                                                             │
 │  ❸ تحديث MHE (بعد التكامل):                                │
 │     ├── sensor_bus.update(snapshot, t+dt) → y_meas[13]     │
 │     ├── mhe.push_measurement(t+dt, y_meas)                 │
 │     ├── mhe.push_control_and_params(t+dt, fins, params)    │
 │     ├── mhe.update(t+dt)                                   │
 │     │     └── acados MHE solve → x_hat[17]                 │
 │     └── self._mhe_output = mhe_out  ◄── للخطوة التالية    │
 │                                                             │
 │  ❹ تخزين السجل وانتقال للخطوة التالية                      │
 └─────────────────────────────────────────────────────────────┘
```

### الترتيب الحرج

```
الخطوة k:   MPC يستخدم mhe_out(k-1)  →  يُصدر fins(k)
             ↓
             RK4 integration  →  snapshot(k)
             ↓
             SensorBus  →  y_meas(k)
             ↓
             MHE solve  →  mhe_out(k)    ◄── يُخزَّن للخطوة k+1

تأخر خطوة واحدة: MPC في الخطوة k يستخدم تقدير MHE من الخطوة k-1
```

هذا التأخر (one-step delay) مقبول عند 100 Hz (dt=10ms) — تغيّر الحالة بين خطوتين صغير جداً.

---

## 3. MHE → MPC: آلية الربط بالكود

### 3.1 حقن MHE في state_dict

في `rocket_6dof_sim.py` سطر ~3445، يُـلف دالة التحكم الأصلية بغلاف يحقن تقدير MHE:

```python
# الإعداد (مرة واحدة عند بدء المحاكاة):
_original_ctrl = mpc.control_function  # الأصلي
_sim_ref = self                         # مرجع للمحاكاة

def _mhe_control_fn(state_dict, t):
    # حقن آخر تقدير MHE في القاموس
    if _sim_ref._mhe_output is not None:
        state_dict['mhe_output'] = _sim_ref._mhe_output
    return _original_ctrl(state_dict, t)

control_function = _mhe_control_fn  # يُستخدم بدلاً من الأصلي
```

### 3.2 استقبال MHE في متحكم MPC

في `m130_mpc_autopilot.py` سطر ~476، يستقبل المتحكم التقدير:

```python
# داخل MpcController.control_function():

# أولاً: استخراج الحالة من الحقيقة (truth)
x_mpc = self._extract_mpc_state(state_dict, t)  # [18] من state_dict

# ثانياً: قراءة تقدير MHE
mhe_out = state_dict.get('mhe_output')

if mhe_out is not None and mhe_out.valid:
    if self._use_estimation and mhe_out.quality >= self._quality_gate_thr:
        xh = mhe_out.x_hat          # [17] تقدير الحالة
        wind_ne = mhe_out.d_hat.get('wind_ne', [0.0, 0.0])
        # ... مزج وتعويض ...
```

### 3.3 المزج: ما الذي يُؤخذ من MHE وما الذي يُترك؟

```
  حالة MPC [18]          مصدر البيانات
  ─────────────          ──────────────
  V     (0)              ✗ الحقيقة (truth) — انحياز MHE ~18 م/ث
  γ     (1)              ✗ الحقيقة — V bias يُفسد gamma
  χ     (2)              ✗ الحقيقة — V bias يُفسد chi
  p     (3)              ✗ الحقيقة — مباشر من الجيروسكوب
  q     (4)              ✗ الحقيقة
  r     (5)              ✗ الحقيقة
  α     (6)              ✗ الحقيقة — محسوب من V, γ
  β     (7)              ✗ الحقيقة — محسوب من V, χ
  φ     (8)              ✗ الحقيقة — مباشر من quaternion
  h     (9)              ✓ مزج MHE — GPS+baro مُصحَّح
  x     (10)             ✓ مزج MHE — GPS مُصحَّح
  y     (11)             ✓ مزج MHE — GPS مُصحَّح
  δe_s  (12)             ✗ ذاكرة المتحكم
  δr_s  (13)             ✗ ذاكرة المتحكم
  δa_s  (14)             ✗ ذاكرة المتحكم
  δe_act(15)             ✗ من نموذج المشغّل
  δr_act(16)             ✗ من نموذج المشغّل
  δa_act(17)             ✗ من نموذج المشغّل
```

**السبب:**  MHE يستخدم متعدد حدود تقريبي لمعامل السحب $C_D$، بينما المحاكاة تستخدم جداول CFD كاملة. هذا يُنتج فرقاً ~18 م/ث في تقدير V. لأن $\gamma = \arctan(-v_z/v_h)$ و $\chi = \arctan(v_y/v_x)$ يعتمدان مباشرة على V، الخطأ ينتشر. أما الموقع (h, x, y) فمُصحَّح بواسطة GPS ودقيق.

### 3.4 منحنى المزج (Blend Curve)

```
    blend
    1.0 ┤                    ╔═══════════════════
        │                   ╔╝
        │                  ╔╝
        │                 ╔╝
    0.5 ┤                ╔╝
        │               ╔╝
        │              ╔╝
        │             ╔╝
    0.0 ┤════════════╝
        └──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──→ t(s)
           0  1  2  3  4  5  5.5 6  7  8

        ◄─── burnout ──►◄ settle ►◄ ramp ►◄ full blend ►
```

```python
# المزج الزمني: خطي من 0 إلى 1
t_ramp_start = 5.5   # بعد انتهاء الاحتراق (~4.5s) + استقرار
t_ramp_end   = 8.0   # مزج كامل

if t <= 5.5:      blend = 0.0          # لا مزج أثناء الاحتراق
elif t >= 8.0:    blend = 1.0          # مزج كامل
else:             blend = (t - 5.5) / 2.5   # خطي

# إخماد عند الارتفاعات المنخفضة:
#  150m → 50m AGL: يتناقص blend خطياً إلى صفر
alt_agl = x_mpc[9] * 100.0   # إلغاء التطبيع
if alt_agl < 150:
    blend *= max(0, (alt_agl - 50) / 100)  # صفر تحت 50m
```

**لماذا الإخماد عند الأرض؟**  
عند alt < 150m، الصاروخ قريب من الاصطدام. أي خطأ في تقدير الموقع من MHE يمكن أن يتسبب في مناورة مفاجئة. من الأفضل الاعتماد على الحقيقة في اللحظات الأخيرة.

### 3.5 المعادلة الكاملة

$$x_{mpc,i} = (1 - \beta_{blend}) \cdot x_{truth,i} + \beta_{blend} \cdot x_{MHE,i} \quad \text{for } i \in \{9, 10, 11\}$$

حيث:

$$\beta_{blend} = \underbrace{\text{clamp}\left(\frac{t - 5.5}{2.5}, 0, 1\right)}_{\text{زمني}} \times \underbrace{\text{clamp}\left(\frac{h_{AGL} - 50}{100}, 0, 1\right)}_{\text{ارتفاعي}}$$

---

## 4. بوابة الجودة والحماية من الفشل

### 4.1 مقياس الجودة (Quality Metric)

يُحسب في `MheEstimator.update()` بعد كل حل:

```python
quality = 1.0

# 1. حالة الحل: فشل QP يخفض الجودة 50%
if solver_status in (3, 4):    # QP failure
    quality *= 0.5

# 2. حجم ضوضاء العملية: كبيرة = النموذج لا يطابق القياسات
w_norm = 0.0
for k in range(N):
    wk = solver.get(k, "u")   # ضوضاء العملية
    w_norm += dot(wk, wk)
w_norm = sqrt(w_norm / N)

if w_norm > 5.0:
    quality *= max(0.1, 1.0 - (w_norm - 5.0) / 15.0)
#                              ↑
# w_norm = 5  → quality × 1.0    (لا تأثير)
# w_norm = 10 → quality × 0.67
# w_norm = 15 → quality × 0.33
# w_norm = 20 → quality × 0.1    (أدنى حد)
```

### 4.2 بوابة الجودة في MPC

```python
# في MpcController:
self._quality_gate_thr = 0.3   # الحد الأدنى من الإعدادات

if mhe_out.quality >= 0.3:
    # ← استخدام تقدير MHE ✓
else:
    # ← تجاهل MHE، استخدام الحقيقة ✗
    logger.debug("MHE quality too low, using truth")
```

### 4.3 آلية التجميد (Frozen Output)

إذا فشل MHE لعدة خطوات متتالية:

```python
# في MheEstimator:
self._max_consec_fails = 10

if NaN_in_x_hat:
    self._consec_fails += 1
    if self._consec_fails >= 10:
        return frozen_output   # quality=0, valid=False
        # → MPC لن يستخدمه (quality < 0.3)
    else:
        return self._last_valid  # آخر تقدير صالح
```

### 4.4 مخطط القرار الكامل

```
                    mhe_out موجود؟
                         │
                    ┌────┴────┐
                   نعم       لا → استخدم الحقيقة كاملةً
                    │
               mhe_out.valid؟
                    │
               ┌────┴────┐
              نعم       لا → استخدم الحقيقة كاملةً
               │
          quality ≥ 0.3؟
               │
          ┌────┴────┐
         نعم       لا → استخدم الحقيقة + log تحذير
          │
     أحسب blend (زمني × ارتفاعي)
          │
     ┌────┴────┐
   blend>0     blend=0 → لا تعديل (أثناء الاحتراق أو قرب الأرض)
     │
     ▼
  امزج (h, x, y)
  + تعويض اتجاه (إن مُفعّل)
  + تحديث _cur_alt, _cur_x, _cur_y
```

---

## 5. تقدير الرياح وتعويض الاتجاه

### 5.1 كيف يُقدّر MHE الرياح؟

MHE لا يقيس الرياح مباشرة. يستنتجها من الفرق بين:
- **السرعة الهوائية** (من نموذج الديناميكا): $V_{air}$ 
- **السرعة الأرضية** (من GPS): $V_{ground}$

$$\vec{V}_{ground} = \vec{V}_{air} + \vec{w}$$

في نموذج القياس (h function):
```python
h[10] = V·cos(γ)·cos(χ) + w_n    # GPS velocity north
h[11] = V·cos(γ)·sin(χ) + w_e    # GPS velocity east
```

MHE يُحسّن $V$, $\gamma$, $\chi$ و $w_n$, $w_e$ معاً بحيث يتطابق $h(x)$ مع قياسات GPS.

### 5.2 مساران لاستخدام تقدير الرياح

```
                wind_ne من MHE
                      │
           ┌──────────┴──────────┐
           │                     │
    ┌──────▼──────┐      ┌──────▼──────┐
    │ Wind FF     │      │ Heading     │
    │ (معطّل)     │      │ Compensation│
    │             │      │ (مُفعّل)    │
    │ يحوّل V,γ,χ │      │ يُعدّل χ_ref│
    │ من أرضي    │      │ لتعويض drift│
    │ إلى هوائي  │      │             │
    └─────────────┘      └─────────────┘
```

### المسار 1: Wind Feedforward (معطّل)

```python
# في _extract_mpc_state():
if self._enable_wind_ff:  # ← False بالافتراضي
    self._wind_est[0] = wind_ne[0]   # w_N
    self._wind_est[1] = wind_ne[1]   # w_E

# يُطرح من السرعة الأرضية:
vx = v_ned[0] - self._wind_est[0]   # → سرعة هوائية
vy = v_ned[1] - self._wind_est[1]
V = sqrt(vx² + vy² + vz²)           # → V_air
```

**لماذا مُعطّل؟** نموذج MPC لا يتضمن قوى الرياح في معادلات $\dot{V}$, $\dot{\gamma}$, $\dot{\chi}$. تحويل V إلى هوائي مع بقاء الديناميكا بدون قوى الرياح يُنشئ تناقضاً → عدم استقرار (roll divergence عند MPC step ~350).

### المسار 2: Heading Compensation (مُفعّل ✓)

يعدّل فقط **مرجع الاتجاه** $\chi_{ref}$ لتعويض الانجراف بسبب الرياح:

```python
def _apply_wind_compensation(d_hat, V_est, t):
    w_n, w_e = d_hat['wind_ne']
    V_fwd = max(V_est, 50.0)
    
    # ❶ الزاوية الخام: كم يجب أن يميل الاتجاه؟
    chi_raw = atan2(w_e, V_fwd)
    #   مثال: w_e=8, V=300 → chi_raw ≈ 1.5°
    
    # ❷ إشباع: |correction| ≤ 8°
    chi_raw = clamp(chi_raw, -8°, +8°)
    
    # ❸ تحديد معدل: |Δ/dt| ≤ 15°/s
    max_step = 15°/s × dt
    delta = chi_raw - chi_comp_filt
    delta = clamp(delta, -max_step, +max_step)
    
    # ❹ مرشح تمرير منخفض (2 Hz):
    α_f = min(1.0, 2π × 2.0 × dt)
    chi_comp_filt += α_f × delta
```

**الاستخدام في yref:**
```python
# عند تعيين مراجع MPC:
yref[2] = chi_ref_los - chi_comp_filt
#         ↑ الاتجاه    ↑ تعويض الرياح
#         الهندسي       (مُلطّف ومُشبَع)
```

**لماذا هذا آمن؟** لأنه يُعدّل فقط **المرجع** الذي يتتبعه MPC، ولا يغيّر حالة MPC أو نموذجه. MPC يظل يتعامل مع ديناميكا متسقة.

---

## 6. acados: من التعريف الرمزي إلى الحل الآني

### 6.1 ما هو acados؟

acados هي مكتبة مفتوحة المصدر لحل **مسائل التحكم الأمثل غير الخطية** (Nonlinear OCP) في **الزمن الحقيقي**. تجمع بين:
- **CasADi:** تعريف رمزي للنموذج الرياضي + اشتقاق تلقائي (AD)
- **HPIPM:** حل مسائل البرمجة التربيعية (QP) عالية الأداء  
- **توليد شيفرة C:** تحويل النموذج الرمزي إلى شيفرة C محسّنة

### 6.2 خط الإنتاج (Pipeline)

```
     ──── مرحلة البناء (مرة واحدة) ────         ──── زمن التشغيل (كل 20ms) ────

  ┌───────────┐
  │ CasADi    │
  │ رمزي     │    f(x,u,p) كتعبيرات رمزية
  │ (Python)  │
  └─────┬─────┘
        │  تعبيرات رياضية
        ▼
  ┌───────────┐
  │ acados    │    يُنشئ AcadosOcp مع:
  │ template  │    - التكلفة (LINEAR_LS / NONLINEAR_LS)
  │ (Python)  │    - القيود (صلبة / مرنة / polytopic)
  │           │    - خيارات الحل (SQP_RTI, HPIPM, ERK)
  └─────┬─────┘
        │  .json + .c files
        ▼
  ┌───────────┐
  │ Code Gen  │    يُولد ملفات C تلقائياً:
  │ (CasADi   │    • m130_rocket_expl_ode_fun.c     (ديناميكا)
  │  + acados)│    • m130_rocket_expl_ode_fun_jac.c  (Jacobians)
  │           │    • m130_rocket_cost_y_fun.c        (تكلفة)
  │           │    • acados_solver_m130_rocket.c      (الحل)
  └─────┬─────┘
        │  .c files
        ▼
  ┌───────────┐
  │  cc -O2   │    ترجمة إلى مكتبة مشتركة
  │ (compile) │
  └─────┬─────┘
        │  .so (shared library)
        ▼
  ┌───────────┐    في كل خطوة تحكم:
  │ ctypes    │    
  │ (Python   │    solver.set(0, "lbx", x_current)
  │  ↔ C)     │    solver.set(k, "yref", ref)
  │           │    solver.solve()            ← ~5ms C function
  │           │    u = solver.get(0, "u")
  └───────────┘
```

### 6.3 لماذا هذا سريع؟

| المرحلة | Python | C المُولد |
|---------|--------|-----------|
| حساب f(x,u,p) | ~5 ms | ~0.01 ms |
| حساب Jacobians | ~20 ms | ~0.05 ms |
| حل QP | ~50 ms | ~2 ms |
| **الإجمالي** | ~75 ms | ~3-5 ms |

acados يُزيل كل overhead الـ Python: لا حلقات، لا تخصيص ذاكرة، لا تفسير — كل شيء مُترجم مسبقاً إلى C محسّن.

### 6.4 آلية إعادة البناء الذكية

```python
# في MpcController._build_solver():

# حساب MD5 hash للملفات المصدرية
cur_hash = md5(m130_acados_model.py + m130_ocp_setup.py + m130_mpc_autopilot.py)

# مقارنة مع hash المُخزّن
if cur_hash == stored_hash:
    # لا تغيير → أعد استخدام .so المُترجم
    rebuild = False
else:
    # تغيير → أعد الترجمة
    os.remove('m130_mpc_autopilot.json')
    rebuild = True
    
solver = AcadosOcpSolver(ocp, json_file='m130_mpc_autopilot.json')
# إذا json موجود + .so موجود → تحميل مباشر (< 100ms)
# إذا لا → code gen + compile (~10s)
```

---

## 7. acados لـ MPC: البنية الداخلية

### 7.1 خوارزمية الحل

```
SQP_RTI (Sequential Quadratic Programming — Real-Time Iteration)
═══════════════════════════════════════════════════════════════

للخطوة k من المحاكاة:

  ❶ Linearization: حساب Jacobians حول المسار الحالي
     ∂f/∂x, ∂f/∂u (ديناميكا)    ← من C المُولد (ERK integrator)
     ∂cost/∂x, ∂cost/∂u         ← من Vx, Vu المُعرّفة
     
  ❷ QP Formation: بناء مسألة QP تربيعية
     min  Σ ||Δy||²_W
     s.t. Δx_{k+1} = A_k Δx_k + B_k Δu_k
          lbx ≤ x + Δx ≤ ubx
          lbu ≤ u + Δu ≤ ubu
          
  ❸ QP Solve: حل بواسطة HPIPM (interior-point method)
     → Δu*_0, Δu*_1, ..., Δu*_{N-1}
     
  ❹ Update: تحديث الحل
     u*_0 = u_prev + Δu*_0
     x*_k = x_prev_k + Δx*_k  (warm start للخطوة التالية)
```

**SQP_RTI vs SQP الكامل:**
- SQP الكامل: يُكرر ❶-❹ حتى التقارب (10-50 تكراراً)
- SQP_RTI: تكرار **واحد** فقط — يعتمد على أن الحل السابق قريب من الأمثل (warm start)
- في النظام: نُشغّل 3-8 تكرارات RTI (`_sqp_iter`) للحصول على دقة أفضل

### 7.2 المكامل (Integrator): ERK(4,2)

acados يحتاج مكامل داخلي للتنبؤ بالمستقبل خلال الأفق:

```
ERK = Explicit Runge-Kutta
(4,2) = 4 مراحل (stages), 2 خطوة (steps) لكل فترة

لكل فترة dt_h = 50ms (= tf/N = 4.0/80):
  ┌──────────────────────────────────────────────┐
  │  الخطوة 1: RK4 من t إلى t + 25ms            │
  │    k1 = f(x, u, p)                           │
  │    k2 = f(x + dt/2·k1, u, p)                │
  │    k3 = f(x + dt/2·k2, u, p)                │
  │    k4 = f(x + dt·k3, u, p)                  │
  │    x_mid = x + dt/6·(k1 + 2k2 + 2k3 + k4)  │
  │                                              │
  │  الخطوة 2: RK4 من t+25ms إلى t+50ms         │
  │    (نفس الشيء من x_mid)                      │
  │    → x_next                                  │
  └──────────────────────────────────────────────┘
  
  + حساب الحساسيات: ∂x_next/∂x, ∂x_next/∂u
  (مطلوبة لـ QP formation — تُحسب بالاشتقاق التلقائي)
```

### 7.3 كيف يُستخدم الحل كل 20ms

```python
# في MpcController.control_function():

# 1. تعيين الحالة الابتدائية (من حالة الصاروخ الحالية)
solver.set(0, "lbx", x_mpc)   # x_current = x_mpc
solver.set(0, "ubx", x_mpc)   # تثبيت الحالة الابتدائية

# 2. تعيين المعاملات لكل نقطة في الأفق
for k in range(N+1):
    t_k = t + k * dt_h
    solver.set(k, "p", _get_params(t_k))
    # → [mass(t_k), thrust(t_k), Ixx(t_k), ..., w_N, w_E]
    # الكتلة والدفع تتغير أثناء الاحتراق!

# 3. تعيين المراجع (ديناميكية)
for k in range(N):
    solver.set(k, "yref", [h_ref, γ_ref, χ_ref, 0,0,0, 0,0,0, 0,0,0])
solver.set(N, "yref", [h_ref, γ_ref, χ_ref, 0,0,0, 0,0,0, x_tgt, y_tgt])

# 4. تعيين الأوزان (ديناميكية — تتغير مع المرحلة واللف)
for k in range(N):
    solver.cost_set(k, "W", W_dynamic)
solver.cost_set(N, "W", W_e_dynamic)

# 5. حل!
for _ in range(sqp_iterations):
    status = solver.solve()
    
# 6. استخراج النتيجة
u_opt = solver.get(0, "u")     # [δ̇e, δ̇r, δ̇a] — معدلات الزعانف
x_next = solver.get(1, "x")   # الحالة المتنبأة في الخطوة التالية

# 7. استخراج مواقع الزعانف من الحالة المتنبأة
de = x_next[12]   # δe_s (مأمور)
dr = x_next[13]   # δr_s
da = x_next[14]   # δa_s
```

### 7.4 الأوزان الديناميكية

المتحكم يُعدّل الأوزان في كل خطوة حسب:

| العامل | كيف يؤثر | مثال |
|--------|----------|------|
| `pn_blend` | 0=altitude hold → 1=LOS | بعد الاحتراق يتحول تدريجياً |
| `roll_recovery` | يزيد أوزان اللف عند φ كبير | φ=20° → أوزان لف ×3 |
| `progress` | قرب الهدف | 95%+ → أوزان γ عالية جداً |
| `altitude` | إخماد χ تحت 100m | alt<100m → وزن χ أقل |

---

## 8. acados لـ MHE: البنية الداخلية

### 8.1 الفرق الجوهري عن MPC

| الجانب | MPC | MHE |
|--------|-----|-----|
| **الاتجاه** | يُخطط **للمستقبل** | يُعيد بناء **الماضي** |
| **متغير القرار u** | أوامر الزعانف | ضوضاء العملية |
| **الحالة الابتدائية** | مُثبتة (= القياس الحالي) | **حرة** (arrival cost) |
| **نوع التكلفة** | LINEAR_LS | NONLINEAR_LS |
| **الخرج المُستخدم** | u(0) — أول تحكم | x(N) — آخر حالة |

### 8.2 النافذة المنزلقة

```
 القياسات:  y₁  y₂  y₃  ...  y₂₃  y₂₄  y₂₅  y₂₆
            ════════════════════════════════════
            
 الزمن t₁:  [y₁───y₂₅]  N=25 خطوة = 500ms
             ↑            ↑
             arrival      تقدير
             cost         الحالي
             
 الزمن t₂:     [y₂───y₂₆]
                ↑            ↑
                x̄ جديد      تقدير
                = x̂(1)      الحالي
```

### 8.3 تكلفة الوصول (Arrival Cost)

تكلفة الوصول تُلخّص كل المعلومات من **قبل** النافذة:

$$J_{arrival} = \|x_0 - \bar{x}\|^2_{P_0^{-1}}$$

- $\bar{x}$ = أفضل تقدير للحالة عند بداية النافذة
- $P_0$ = عدم اليقين في $\bar{x}$

**التحديث بعد كل حل:**

```python
# بعد حل MHE في الزمن t:
self._x_bar = solver.get(1, "x")   # الحالة في الخطوة الثانية
# ← تُصبح arrival cost للنافذة التالية التي تبدأ بخطوة أبعد

# في yref_0:
yref_0 = [y_meas_0,     # قياسات المرحلة 0
          zeros(17),     # مرجع ضوضاء العملية = 0
          x_bar]         # مرجع arrival cost
```

### 8.4 بنية التكلفة في كل مرحلة

```
المرحلة 0 (بداية النافذة):
═══════════════════════
  cost_y_expr_0 = [ h(x,p)  ]   ← 13 قياس  → R⁻¹ (ثقة القياسات)
                  [   w     ]   ← 17 ضوضاء → Q⁻¹ (ثقة الديناميكا)
                  [   x     ]   ← 17 حالة  → P₀⁻¹ (arrival cost)
                  ─────────────
                  47 عنصر
                  
  yref_0 = [y_meas₀, 0₁₇, x̄₁₇]
  W_0 = block_diag(R, Q, P0)    ← 47×47

المراحل 1..N-1:
═══════════════
  cost_y_expr = [ h(x,p) ]   ← 13 قياس  → R⁻¹
                [   w    ]   ← 17 ضوضاء → Q⁻¹
                ───────────
                30 عنصر
  
  yref = [y_meas_k, 0₁₇]
  W = block_diag(R, Q)         ← 30×30

المرحلة النهائية N:
═════════════════
  cost_type_e = LINEAR_LS
  W_e = 0×0 (فارغة)             ← لا تكلفة نهائية
```

### 8.5 لماذا NONLINEAR_LS (وليس LINEAR_LS)؟

التكلفة في MHE تعتمد على $h(x, p)$ — دالة القياس — وهي **غير خطية**:
```python
h[3] = (T·cos(α)·cos(β) - D) / m    # تسارع محوري (غير خطي)
h[10] = V·cos(γ)·cos(χ) + w_n       # GPS velocity (غير خطي)
```

هذا يعني أن acados يحتاج Jacobian $\partial h / \partial x$ لبناء QP — يُحسب تلقائياً بواسطة CasADi AD.

في المقابل، MPC يستخدم LINEAR_LS لأن $y = V_x \cdot x + V_u \cdot u$ — تحويل خطي بسيط.

---

## 9. التفاعل بين الحلّين في الزمن الحقيقي

### 9.1 ميزانية الزمن

```
خطوة واحدة (10ms budget):
═══════════════════════════

MPC solve:   ██████████████░░░░░░  4-30 ms
MHE solve:   ████████░░░░░░░░░░░  5-15 ms
RK4 6DOF:    █░░░░░░░░░░░░░░░░░░  ~0.1 ms
Sensor Bus:  ░░░░░░░░░░░░░░░░░░░  ~0.01 ms
─────────────────────────────────────────
المجموع:                           9-45 ms
```

في الزمن الحقيقي (على معالج مدمج)، هذا يعني أن الحل يجب أن يكتمل في < 20ms (50 Hz). acados يُحقق ذلك بفضل:
1. شيفرة C المُولدة (بدون Python overhead)
2. SQP_RTI بتكرار واحد (بدلاً من 10+)
3. HPIPM: حل QP مُخصص لمسائل الأفق

### 9.2 تدفق المعاملات المشتركة

```
           rocket_6dof_sim.py
                   │
    ┌──────────────┼──────────────┐
    │              │              │
    ▼              ▼              ▼
  mass(t)      thrust(t)      Ixx(t)...
    │              │              │
    ├──────────────┼──────────────┤
    │                             │
    ▼                             ▼
  MPC params [9]:               MHE params [9]:
  [mass, T, Ixx,Iyy,Izz,       [δe, δr, δa,
   h_launch, τ_servo,            mass, T, Ixx,Iyy,Izz,
   w_N, w_E]                     h_launch]
```

**اتجاه تدفق البيانات بين MHE و MPC:**

```
                ┌────────────────────────────────────┐
                │          MHE                       │
                │                                    │
  y_meas[13] ──►│  solver → x_hat[17]               │
                │                    │               │
                │          ┌────────┘               │
                │          │                         │
                │          ▼                         │
                │  d_hat = {                         │
                │    'gyro_bias': x_hat[12:15]      │──── bias (لا يُستخدم حالياً)
                │    'wind_ne':   x_hat[15:17]      │──┐
                │  }                                 │  │
                │  quality = f(status, w_norm)       │──│── quality gate
                └────────────────────────────────────┘  │
                                                        │
                    ┌───────────────────────────────────┘
                    │ wind_ne
                    ▼
                ┌────────────────────────────────────┐
                │          MPC                       │
                │                                    │
                │  if heading_comp:                  │
                │    chi_comp = atan2(w_e, V)        │
                │    yref[χ] -= chi_comp_filt        │
                │                                    │
                │  x_mpc[h,x,y] ← blend(truth, MHE) │
                │                                    │
                │  solver → u_opt[3] → fins[4]       │
                └────────────────────────────────────┘
```

### 9.3 المعاملات التي يتبادلها النظامان

| البيان | المصدر | الوجهة | الاستخدام |
|--------|--------|--------|-----------|
| `x_hat[9:12]` (h,x,y) | MHE | MPC | مزج موقع GPS-مُصحَّح |
| `wind_ne` [2] | MHE | MPC | تعويض اتجاه χ |
| `quality` | MHE | MPC | بوابة: هل نثق بالتقدير؟ |
| `mass(t), T(t)` | المحاكاة | MPC + MHE | معاملات ديناميكية |
| `δe, δr, δa` | MPC | MHE | معاملات معروفة للنموذج |
| `gyro_bias` [3] | MHE | (مُخزَّن) | متاح لكن غير مُستخدم حالياً |

---

## 10. الدروس المستفادة والقيود

### 10.1 ما نجح

| الاستراتيجية | النتيجة |
|--------------|---------|
| مزج **الموقع فقط** من MHE | مستقر — GPS يُصلح الانجراف |
| **تعويض الاتجاه** عبر yref | آمن — لا يغير نموذج MPC |
| **بوابة جودة** 0.3 | يمنع MHE الفاشل من إفساد MPC |
| **إخماد عند الأرض** | يمنع مناورات مفاجئة قبل الاصطدام |
| **تأخر خطوة واحدة** | مقبول عند 100 Hz |

### 10.2 ما لم ينجح

| المحاولة | المشكلة | الحل |
|----------|---------|------|
| مزج V, γ, χ من MHE | انحياز V ~18 م/ث → MPC destabilizes | **لا تمزج** — استخدم truth |
| Wind feedforward (w_N, w_E → MPC params + air-rel V) | تناقض: نموذج MPC بدون قوى رياح في $\dot{V}$ | **معطّل** — heading comp فقط |
| β weight = 20 (بدلاً من 60) | لف -141° بدون رياح (regression) | **أُعيد** إلى 60 |
| chi_comp تراكمي (يُطرح كل خطوة) | يتراكم بلا حد → drift 120°+ → كارثي | **يُطبق كـ yref bias** |

### 10.3 القيود المتبقية

1. **S4 crosswind roll = 67° عند الأرض:**  
   الصاروخ يفقد السرعة والارتفاع → السيطرة الديناميكية الهوائية تضعف → الرياح تُمال الصاروخ. قيد فيزيائي لا حل برمجي له.

2. **انحياز V في MHE ~18 م/ث:**  
   فرق بين polynomial $C_D$ (في MHE) و CFD table (في المحاكاة). الحل: إما تحسين polynomial fit أو استخدام نفس الجداول في كلاهما.

3. **acados لا يدعم تطبيع الأوزان مع MHE:**  
   يجب تعيين `cost_scaling = ones(N+1)` يدوياً لمنع acados من قسمة الأوزان على خطوة الأفق.

### 10.4 مخطط القرار المستقبلي

```
هل لدى MHE polynomial أدق لـ CD؟
        │
   ┌────┴────┐
  نعم       لا
   │         │
   ▼         │
يمكن مزج     لا تمزج V, γ, χ
V, γ, χ      ──► موقع فقط + heading comp
   │
   ▼
هل V bias < 5 م/ث؟
   │
  ┌┴┐
 نعم لا → لا تمزج
  │
  ▼
فعّل wind FF + air-relative V
+ أضف قوى الرياح لنموذج MPC
```

---

## الملحق أ: المكتبات المطلوبة

```
acados-main/
├── lib/
│   ├── libacados.so       # المكتبة الرئيسية
│   ├── libhpipm.so        # حل QP (interior-point)
│   └── libblasfeo.so      # BLAS مُخصص لـ HPIPM
├── interfaces/
│   └── acados_template/   # Python API
│       ├── acados_template/
│       │   ├── acados_ocp.py          # AcadosOcp class
│       │   ├── acados_ocp_solver.py   # AcadosOcpSolver class
│       │   └── acados_model.py        # AcadosModel class
│       └── ...
└── include/acados/         # C headers
```

**متغيرات البيئة:**
```bash
export LD_LIBRARY_PATH=/path/to/acados-main/lib
export ACADOS_SOURCE_DIR=/path/to/acados-main
```

## الملحق ب: الملفات المُولدة تلقائياً

```
c_generated_code/
├── m130_rocket_model/
│   ├── m130_rocket_expl_ode_fun.c          # f(x,u,p) → ẋ
│   ├── m130_rocket_expl_ode_fun_jac.c      # ∂f/∂x, ∂f/∂u
│   ├── m130_rocket_expl_vde_forw.c         # forward sensitivities
│   └── m130_rocket_expl_vde_adj.c          # adjoint sensitivities
├── acados_solver_m130_rocket.c              # solver wrapper
├── acados_solver_m130_rocket.h              # header
├── acados_sim_solver_m130_rocket.c          # simulation solver
├── main_m130_rocket.c                       # standalone test
├── Makefile                                 # build rules
└── libacados_ocp_solver_m130_rocket.so      # compiled library
```

## الملحق ج: ملخص الأرقام

| المعامل | MPC | MHE |
|---------|-----|-----|
| الحالات (NX) | 18 | 17 |
| التحكم/الضوضاء (NU/NW) | 3 | 17 |
| المعاملات (NP) | 9 | 9 |
| الأفق (N) | 80 | 25 |
| الأفق الزمني | 4.0 s | 0.5 s |
| الخطوة الزمنية | 50 ms | 20 ms |
| نوع التكلفة | LINEAR_LS | NONLINEAR_LS |
| خرج المراقبة (ny) | 12 / 11(e) | 30 / 47(0) |
| الحل | SQP_RTI | SQP_RTI |
| QP | HPIPM | HPIPM |
| المكامل | ERK(4,2) | ERK(4,2) |
| LM regularization | 0.01 | — |
| Warm start | نعم | نعم |
| معدل الحل | 50 Hz | 50 Hz |
| زمن الحل النموذجي | 4-30 ms | 5-15 ms |
| JSON file | m130_mpc_autopilot.json | m130_mhe_ocp.json |
