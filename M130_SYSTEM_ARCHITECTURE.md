# M130 Missile GNC System — Architecture & acados Integration
**وثيقة معمارية نظام التوجيه والتحكم لصاروخ M130**

> Version 2.0 — April 2026  
> System: 6DOF Simulation + MPC Autopilot + MHE State Estimation

---

## Table of Contents

1. [نظرة عامة على النظام](#1-نظرة-عامة-على-النظام)
2. [محرك المحاكاة 6DOF](#2-محرك-المحاكاة-6dof)
3. [نموذج MPC في acados](#3-نموذج-mpc-في-acados)
4. [مسألة التحكم الأمثل OCP](#4-مسألة-التحكم-الأمثل-ocp)
5. [نظام التحكم MPC Autopilot](#5-نظام-التحكم-mpc-autopilot)
6. [نموذج MHE في acados](#6-نموذج-mhe-في-acados)
7. [مسألة التقدير MHE OCP](#7-مسألة-التقدير-mhe-ocp)
8. [مقدر الحالة MHE Estimator](#8-مقدر-الحالة-mhe-estimator)
9. [ناقل الحساسات Sensor Bus](#9-ناقل-الحساسات-sensor-bus)
10. [تدفق البيانات الكامل](#10-تدفق-البيانات-الكامل)
11. [الإعدادات والتكوين](#11-الإعدادات-والتكوين)
12. [الأداء ونتائج الاختبارات](#12-الأداء-ونتائج-الاختبارات)

---

## 1. نظرة عامة على النظام

النظام عبارة عن محاكاة كاملة لطيران صاروخ M130 بستة درجات حرية (6DOF) مع نظام تحكم تنبؤي (MPC) وتقدير حالة بأفق متحرك (MHE)، وكلاهما يعتمد على مكتبة **acados** لحل مسائل التحكم الأمثل في الزمن الحقيقي.

### البنية العامة

```
┌───────────────────────────────────────────────────────────┐
│                    rocket_6dof_sim.py                      │
│              محاكاة 6DOF (14 حالة، RK4)                    │
│                                                           │
│  state[14] = [pos(3), vel(3), quat(4), ω(3), mass]       │
│                                                           │
│  ┌─────────────┐  ┌──────────────┐  ┌───────────────┐    │
│  │ Propulsion  │  │ Aerodynamics │  │  Atmosphere    │    │
│  │ (thrust     │  │ (CFD68       │  │  (ISA, wind)   │    │
│  │  curve)     │  │  tables)     │  │               │    │
│  └─────────────┘  └──────────────┘  └───────────────┘    │
│                                                           │
│  ┌─────────────┐  ┌──────────────┐  ┌───────────────┐    │
│  │ Actuator    │  │ Mass Props   │  │  Frame Mgr    │    │
│  │ (servo      │  │ (time-       │  │  (NED↔ECEF,   │    │
│  │  dynamics)  │  │  varying)    │  │   WGS-84)     │    │
│  └─────────────┘  └──────────────┘  └───────────────┘    │
└───────────────┬───────────────────────────┬───────────────┘
                │ state_dict               │ snapshot
                ▼                           ▼
┌──────────────────────┐      ┌──────────────────────────┐
│  MPC Controller      │      │  MHE Estimator           │
│  (m130_mpc_autopilot)│◄────►│  (m130_mhe_estimator)    │
│                      │      │                          │
│  ┌────────────────┐  │      │  ┌────────────────────┐  │
│  │ acados OCP     │  │      │  │ acados OCP         │  │
│  │ Solver         │  │      │  │ Solver             │  │
│  │ (SQP_RTI)      │  │      │  │ (SQP_RTI)          │  │
│  └────────────────┘  │      │  └────────────────────┘  │
│                      │      │                          │
│  ┌────────────────┐  │      │  ┌────────────────────┐  │
│  │ CasADi Model   │  │      │  │ CasADi Model       │  │
│  │ (18 states)    │  │      │  │ (17 states)        │  │
│  └────────────────┘  │      │  └────────────────────┘  │
└──────────┬───────────┘      └──────────┬───────────────┘
           │ fins[4]                     │ x_hat, wind, bias
           ▼                             ▼
    ┌──────────────┐            ┌──────────────────┐
    │  Actuator    │            │  Sensor Bus      │
    │  Model       │            │  (virtual IMU,   │
    │  (servo lag) │            │   GPS, baro)     │
    └──────────────┘            └──────────────────┘
```

### الملفات الرئيسية

| الملف | الدور | الحجم |
|-------|-------|-------|
| `rocket_6dof_sim.py` | محرك المحاكاة الرئيسي | ~4200 سطر |
| `mpc/m130_acados_model.py` | نموذج CasADi لـ MPC | ~400 سطر |
| `mpc/m130_ocp_setup.py` | تهيئة OCP لـ MPC | ~250 سطر |
| `mpc/m130_mpc_autopilot.py` | متحكم MPC + توجيه LOS | ~850 سطر |
| `mpc/m130_mhe_model.py` | نموذج CasADi لـ MHE | ~450 سطر |
| `mpc/m130_mhe_ocp_setup.py` | تهيئة OCP لـ MHE | ~220 سطر |
| `mpc/m130_mhe_estimator.py` | غلاف MHE مع نافذة منزلقة | ~250 سطر |
| `mpc/m130_sensor_bus.py` | حساسات افتراضية | ~180 سطر |
| `config/6dof_config_advanced.yaml` | ملف الإعدادات | ~300 سطر |

---

## 2. محرك المحاكاة 6DOF

### الصنف الرئيسي: `Rocket6DOFSimulation`

**الملف:** `rocket_6dof_sim.py`

#### متجه الحالة (14 عنصراً)

```
state[0:3]   = [x, y, z]        الموقع (m) — إطار NED أو ECEF
state[3:6]   = [vx, vy, vz]     السرعة (m/s)
state[6:10]  = [q0, q1, q2, q3] الرباعية (quaternion) — scalar-first
state[10:13] = [ωx, ωy, ωz]     السرعة الزاوية (rad/s) — إطار الجسم
state[13]    = mass              الكتلة (kg)
```

#### التهيئة (`__init__`)

يُنشئ النماذج الفرعية التالية من ملف الإعدادات:

| النموذج الفرعي | الصنف | الوظيفة |
|---------------|-------|---------|
| الدفع | `AdvancedPropulsionModel` | منحنى الدفع مع الفراغ |
| الديناميكا الهوائية | `AdvancedAerodynamicsModel` | جداول CFD68 |
| الغلاف الجوي | `AtmosphereModel` | ISA + رياح |
| خصائص الكتلة | `MassPropertiesModel` | كتلة ومراكز ثقل وعزوم القصور الذاتي متغيرة مع الزمن |
| المشغّل | `ActuatorModel` | ديناميكا السيرفو (first-order lag) |
| المنصة | `LaunchRailModel` | قيد 1-DOF حتى مغادرة السكة |
| الإطار | `FrameManager` | تحولات NED↔ECEF (WGS-84, J2 gravity) |

#### حلقة المحاكاة (`simulate`)

```python
def simulate(self, duration, dt=0.01, control_function=None):
    # 1. اختيار مصدر التحكم (أولوية):
    #    - ملف انحرافات خارجي
    #    - MPC (control_type="mpc")
    #    - باليستي (انحرافات ثابتة)
    #    - دالة تحكم مخصصة

    # 2. تفعيل MHE (اختياري):
    if control_type == "mpc" and estimation.mode == "mhe":
        mpc = MpcController(self)
        sensor_bus = SensorBus(est_cfg)
        mhe_estimator = MheEstimator(est_cfg)
        # لف دالة التحكم لحقن تقدير MHE:
        control_function = _mhe_control_fn  # يحقن mhe_output في state_dict

    # 3. الحلقة الرئيسية:
    for step in range(total_steps):
        next_state, snapshot = _integrate_one_step(state, t, dt, control_function)

        # 4. تحديث MHE (بعد كل خطوة):
        if mhe_enabled and t > 0.3:
            pkt = sensor_bus.update(snapshot, t)
            mhe_estimator.push_measurement(t, pkt.y_meas)
            mhe_estimator.push_control_and_params(t, fins, params)
            mhe_out = mhe_estimator.update(t)
            self._mhe_output = mhe_out  # متاح للمتحكم في الخطوة التالية

        state = next_state
```

#### التكامل العددي (RK4)

يستخدم Runge-Kutta من الدرجة الرابعة (RK4 الكلاسيكي) مع أخذ عينة التحكم عند كل مرحلة:

```python
def _rk4_step(state, t, dt, control_function, k1):
    # k1: محسوب مسبقاً مع update_state=True
    # k2: t + dt/2, control_function(state2, t+dt/2)
    # k3: t + dt/2, control_function(state3, t+dt/2)
    # k4: t + dt,   control_function(state4, t+dt)
    next_state = state + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
```

#### تحويل الحالة للمتحكم

`_state_array_to_dict()` يحول متجه الحالة [14] إلى قاموس واجهة المتحكم:

```python
state_dict = {
    'position': [3],            # NED أو ECEF
    'velocity': [3],            # m/s
    'quaternion': [4],          # [q0,q1,q2,q3] مُطبَّع
    'angular_velocity': [3],    # rad/s (إطار الجسم)
    'mass': float,              # kg
    'actuator_positions': [4],  # انحرافات الزعانف الفعلية (rad)
    # وضع المدى الطويل يُضيف:
    'altitude_km': float,
    'ground_range_km': float,
    'vel_ned_launch': [3],      # سرعة NED في إطار الإطلاق
}
```

#### تطبيق الرياح

```python
# في _compute_dynamics():
wind_ned = atmosphere.get_wind(altitude, t)    # [w_N, w_E, w_D]
velocity_for_aero = vel_ned - wind_ned         # تحويل إلى سرعة هوائية
alpha = atan2(-v_aero_z, v_aero_x)            # زاوية الهجوم
beta  = asin(v_aero_y / |v_aero|)             # زاوية الانزلاق
```

---

## 3. نموذج MPC في acados

### الملف: `mpc/m130_acados_model.py`

يُعرّف نموذج CasADi رمزي لمسألة التحكم الأمثل. acados يستخدم هذا النموذج لتوليد شيفرة C محسّنة تُحل في الزمن الحقيقي.

### متجه الحالة (18 حالة)

```
x[0]  = V           السرعة الهوائية (m/s) — [ملاحظة: هوائية وليست أرضية]
x[1]  = γ (gamma)   زاوية مسار الطيران العمودي (rad)
x[2]  = χ (chi)     زاوية مسار الطيران الأفقي (rad)
x[3]  = p           معدل اللف (rad/s)
x[4]  = q           معدل الميل (rad/s)
x[5]  = r           معدل الانعراج (rad/s)
x[6]  = α (alpha)   زاوية الهجوم (rad)
x[7]  = β (beta)    زاوية الانزلاق الجانبي (rad)
x[8]  = φ (phi)     زاوية اللف (rad)
x[9]  = h           الارتفاع المُطبَّع = h_AGL / 100 (—)
x[10] = x_ground    المدى الأرضي المُطبَّع = x / 1000 (—)
x[11] = y_ground    الانحراف الجانبي المُطبَّع = y / 1000 (—)
x[12] = δe_s        انحراف زعنفة الميل المأمور (rad)
x[13] = δr_s        انحراف زعنفة الانعراج المأمور (rad)
x[14] = δa_s        انحراف زعنفة اللف المأمور (rad)
x[15] = δe_act      انحراف زعنفة الميل الفعلي (rad)
x[16] = δr_act      انحراف زعنفة الانعراج الفعلي (rad)
x[17] = δa_act      انحراف زعنفة اللف الفعلي (rad)
```

**ملاحظة معمارية مهمة:** الحالات [12-14] هي الأوامر و[15-17] هي المواضع الفعلية. هذا يُمكّن MPC من "رؤية" تأخير السيرفو والتخطيط وفقاً لذلك.

### متجه التحكم (3 أوامر)

```
u[0] = δ̇e   معدل تغيير زعنفة الميل (rad/s)
u[1] = δ̇r   معدل تغيير زعنفة الانعراج (rad/s)
u[2] = δ̇a   معدل تغيير زعنفة اللف (rad/s)
```

التحكم بالمعدلات (وليس المواضع) يُمكّن acados من فرض قيود سلسة على المواقع والمعدلات معاً.

### متجه المعاملات (9 معاملات)

```
p[0] = m         الكتلة (kg) — متغيرة أثناء الاحتراق
p[1] = T         الدفع (N) — صفر بعد الاحتراق
p[2] = Ixx       عزم القصور الذاتي حول محور اللف (kg·m²)
p[3] = Iyy       عزم القصور الذاتي حول محور الميل (kg·m²)
p[4] = Izz       عزم القصور الذاتي حول محور الانعراج (kg·m²)
p[5] = h_launch  ارتفاع الإطلاق فوق سطح البحر (m)
p[6] = τ_servo   ثابت زمن السيرفو (s)
p[7] = w_N       الرياح الشمالية من MHE (m/s)
p[8] = w_E       الرياح الشرقية من MHE (m/s)
```

### المعاملات الديناميكية الهوائية

يستخدم النموذج متعددات حدود من الدرجة 7 مع اعتماد ماخ مكعب، مُشتقة من 68 نقطة بيانات CFD:

$$C(\alpha, M) = C_{a1}(M) \cdot \alpha + C_{a3}(M) \cdot \alpha^3 + C_{a5}(M) \cdot \alpha^5 + C_{a7}(M) \cdot \alpha^7$$

حيث كل معامل يعتمد على ماخ بشكل مكعب:

$$C_{a1}(M) = k_0 + k_1 M + k_2 M^2 + k_3 M^3$$

**المعاملات المحسوبة:**

| المعامل | الوصف | المتغير |
|---------|-------|---------|
| $C_{N,pitch}(\alpha, M)$ | القوة العمودية (ميل) | `alpha` |
| $C_{m,pitch}(\alpha, M)$ | عزم الميل | `alpha` |
| $C_{md,pitch}(\delta_e, M)$ | عزم الميل من الزعنفة | `delta_e_act` |
| $C_{Nd,pitch}(\delta_e, M)$ | قوة الزعنفة العمودية | `delta_e_act` |
| $C_{mq}(\alpha, M)$ | تخميد الميل | `alpha²` (even) |
| $C_D(\alpha_{total}, M)$ | السحب | `α² + β²` (even) |

**التماثل المحوري:** الصاروخ متماثل → معاملات الانعراج تساوي معاملات الميل مع تبديل `β ↔ α` و `δr ↔ δe`.

**اللف:** تخميد `Clp` + فعالية `Cl_δa`.

### معادلات الحركة في CasADi

```python
# السرعة
V̇ = (T·cos(α)·cos(β) - D - m·g·sin(γ)) / m

# مسار الطيران العمودي
γ̇ = ((T·(sin(α)·cos(φ) + cos(α)·sin(β)·sin(φ))
       + L_pitch·cos(φ) - L_yaw·sin(φ)) / (m·V)
      - g·cos(γ)/V)

# مسار الطيران الأفقي
χ̇ = (T·(sin(α)·sin(φ) - cos(α)·sin(β)·cos(φ))
      + L_pitch·sin(φ) + L_yaw·cos(φ)) / (m·V·cos(γ))

# المعدلات الزاوية
ṗ = M_roll / Ixx
q̇ = M_pitch / Iyy
ṙ = M_yaw / Izz

# زاوية الهجوم والانزلاق
α̇ = q - γ̇·cos(φ) - χ̇·cos(γ)·sin(φ)
β̇ = -r - γ̇·sin(φ) + χ̇·cos(γ)·cos(φ)

# زاوية اللف
φ̇ = p + (q·sin(φ) + r·cos(φ))·tan(γ)

# الموقع (مع إضافة الرياح)
ḣ = V·sin(γ) / 100
ẋ_ground = (V·cos(γ)·cos(χ) + w_N) / 1000
ẏ_ground = (V·cos(γ)·sin(χ) + w_E) / 1000
```

**ملاحظة مهمة:** الرياح (w_N, w_E) تظهر **فقط** في معادلات الموقع. معادلات V̇, γ̇, χ̇ لا تتضمن تأثير الرياح المباشر — هذا تبسيط عن الديناميكا الكاملة.

### ديناميكا المشغّل (السيرفو)

```python
δ̇_e_act = (δe_s - δe_act) / τ_servo
δ̇_r_act = (δr_s - δr_act) / τ_servo
δ̇_a_act = (δa_s - δa_act) / τ_servo
```

نموذج first-order lag يُمكّن MPC من التنبؤ بتأخير تنفيذ الأوامر.

### بناء نموذج acados

```python
model = AcadosModel()
model.name = "m130_rocket"
model.x = ca.vertcat(V, gamma, chi, p, q, r, alpha, beta, phi,
                     h, x_ground, y_ground,
                     delta_e_s, delta_r_s, delta_a_s,
                     delta_e_act, delta_r_act, delta_a_act)
model.u = ca.vertcat(ddelta_e, ddelta_r, ddelta_a)
model.p = ca.vertcat(mass, thrust, Ixx, Iyy, Izz, launch_alt, tau_servo, w_N, w_E)
model.f_expl_expr = f_expl     # المعادلات الصريحة ẋ = f(x,u,p)
model.xdot = ca.SX.sym('xdot', 18)
model.f_impl_expr = xdot - f_expl  # الصيغة الضمنية (لطريقة IRK)
```

---

## 4. مسألة التحكم الأمثل OCP

### الملف: `mpc/m130_ocp_setup.py`

### الصياغة الرياضية

$$\min_{u_{0:N-1}} \sum_{k=0}^{N-1} \|y_k - y_{ref,k}\|^2_{W_k} + \|y_N - y_{ref,N}\|^2_{W_N}$$

خاضعة لـ:

$$x_{k+1} = f(x_k, u_k, p_k), \quad k = 0, \ldots, N-1$$
$$x_{lb} \leq x_k \leq x_{ub}$$
$$u_{lb} \leq u_k \leq u_{ub}$$
$$C_{poly} \cdot x_k \leq \delta_{max}$$

### الأفق (Horizon)

| المعامل | القيمة | الوصف |
|---------|--------|-------|
| N | 80 | عدد خطوات الأفق |
| tf | 4.0 s | زمن الأفق الكلي |
| dt | 50 ms | خطوة زمنية واحدة |

### دالة التكلفة

**نوع التكلفة:** `LINEAR_LS` — الأكثر كفاءة في acados لتتبع المرجع.

**متجه الخرج المتوسط (12 عنصراً):**

$$y_k = V_x \cdot x_k + V_u \cdot u_k$$

```
y = [h, γ, χ, p, q, r, α, β, φ, δ̇e, δ̇r, δ̇a]
```

**متجه الخرج النهائي (11 عنصراً):**

$$y_N = V_{x,e} \cdot x_N$$

```
y_e = [h, γ, χ, p, q, r, α, β, φ, x_ground, y_ground]
```

**الأوزان (تُستبدل ديناميكياً بالمتحكم):**

| الحالة | وزن أولي W | وزن نهائي W_e | الدور |
|--------|-----------|---------------|-------|
| h | 0 → 500 | 200 → 600 | تثبيت الارتفاع (مرحلة 1) |
| γ | 50 → 500 | 500 | تتبع مسار الهبوط |
| χ | 20 → 120 | 200 | تتبع الاتجاه |
| p | 50 → 180 | 100 | تخميد اللف |
| q | 40 → 30 | 80 | تخميد الميل |
| r | 40 → 20 | 80 | تخميد الانعراج |
| α | 5 → 40 | 10 → 30 | تحديد زاوية الهجوم |
| β | 40 → 60 | 40 | تقليل الانزلاق الجانبي |
| φ | 60 → 600 | 120 | تثبيت اللف عند الصفر |

### القيود

#### قيود التحكم (صلبة)

```
|δ̇e|, |δ̇r|, |δ̇a| ≤ 250°/s = 4.36 rad/s
```

#### قيود الحالة (مرنة — soft constraints)

| الحالة | الحد | النوع |
|--------|------|-------|
| p_rate | ±60°/s | مرن |
| q_rate | ±80°/s | مرن |
| r_rate | ±60°/s | مرن |
| α | ±12° | مرن |
| β | ±8° | مرن |
| φ | ±30° | مرن |
| h | ≥ -0.7 | مرن |

#### قيود الزعانف (صلبة)

```
|δe|, |δr|, |δa| ≤ ±20°   (المأمور والفعلي)
```

#### قيود خلط الزعانف (Polytopic)

الصاروخ يستخدم تكوين X بأربع زعانف:

```
fin₁ = -δe - δr + δa ≤ ±20°
fin₂ = -δe + δr + δa ≤ ±20°
fin₃ = +δe + δr + δa ≤ ±20°
fin₄ = +δe - δr + δa ≤ ±20°
```

هذا يمنع تشبع أي زعنفة فردية حتى لو كانت القيم المفردة ضمن الحدود.

### إعدادات الحل

```python
ocp.solver_options.nlp_solver_type       = 'SQP_RTI'
ocp.solver_options.qp_solver             = 'PARTIAL_CONDENSING_HPIPM'
ocp.solver_options.qp_solver_iter_max    = 200
ocp.solver_options.hessian_approx        = 'GAUSS_NEWTON'
ocp.solver_options.integrator_type       = 'ERK'
ocp.solver_options.sim_method_num_stages = 4
ocp.solver_options.sim_method_num_steps  = 2
ocp.solver_options.levenberg_marquardt   = 1e-2
```

| الإعداد | القيمة | السبب |
|---------|--------|-------|
| SQP_RTI | — | تقريب SQP بتكرار واحد — مثالي للزمن الحقيقي |
| HPIPM | — | حل QP داخلي عالي الأداء (interior-point) |
| GAUSS_NEWTON | — | تقريب Hessian من Jacobian — مستقر وسريع |
| ERK(4,2) | — | Runge-Kutta صريح، 4 مراحل، خطوتان — دقة كافية |
| LM=0.01 | — | تنظيم Levenberg-Marquardt — يمنع عدم اللياقة |

---

## 5. نظام التحكم MPC Autopilot

### الملف: `mpc/m130_mpc_autopilot.py`

### البنية: توجيه هندسي LOS + تتبع MPC

```
┌──────────────────────────────────────────────────────────┐
│                    MpcController                          │
│                                                          │
│  ┌────────────────────────────────────────────────────┐  │
│  │            توجيه هندسي LOS                        │  │
│  │                                                    │  │
│  │  γ_ref = atan2(Δh, Δx)    ← زاوية خط البصر       │  │
│  │  χ_ref = atan2(Δy, Δx)    ← اتجاه الهدف          │  │
│  │  + تشكيل زاوية الاصطدام (Hermite blend)           │  │
│  │  + تعويض الرياح (heading compensation)             │  │
│  └────────────────────────┬───────────────────────────┘  │
│                           │ γ_ref, χ_ref                 │
│                           ▼                              │
│  ┌────────────────────────────────────────────────────┐  │
│  │         acados MPC Solver (SQP_RTI)               │  │
│  │                                                    │  │
│  │  يتتبع γ_ref, χ_ref مع:                           │  │
│  │  • قيود الزعانف والمعدلات                          │  │
│  │  • تخميد المعدلات الزاوية                          │  │
│  │  • تثبيت اللف عند الصفر                           │  │
│  │  • ديناميكا سيرفو كاملة                           │  │
│  │  → يُخرج: δ̇e, δ̇r, δ̇a                            │  │
│  └────────────────────────┬───────────────────────────┘  │
│                           │                              │
│                           ▼                              │
│  ┌────────────────────────────────────────────────────┐  │
│  │              خلط الزعانف (X config)               │  │
│  │  fin₁ = δa - δe - δr                              │  │
│  │  fin₂ = δa - δe + δr                              │  │
│  │  fin₃ = δa + δe + δr                              │  │
│  │  fin₄ = δa + δe - δr                              │  │
│  └────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────┘
```

### مراحل التحكم

| المرحلة | الزمن | الوصف | γ_ref | التحكم |
|---------|-------|-------|-------|--------|
| **تثبيت ارتفاع** | 0 → ~5s | بعد الإطلاق وأثناء الاحتراق | 0° (أفقي) | h→120m AGL |
| **انتقال** | ~5 → ~7s | مزج سلس لمدة 2s | 0° → LOS | blend 0→1 |
| **توجيه LOS** | ~7 → ~11s | تتبع خط البصر للهدف | LOS angle | γ, χ tracking |
| **تشكيل الاصطدام** | >93% مدى | مزج نحو زاوية الاصطدام | LOS → -30° | impact blend |

### استخراج حالة MPC من محاكاة 6DOF

`_extract_mpc_state()` يحول حالة المحاكاة {14} إلى حالة MPC {18}:

```python
# السرعة والمسار (من سرعة NED):
vx, vy, vz = vel_ned - wind_est   # تحويل إلى سرعة هوائية
V = ||[vx, vy, vz]||
γ = atan2(-vz, sqrt(vx² + vy²))
χ = atan2(vy, vx)

# الزوايا الديناميكية الهوائية:
α = pitch_angle - γ
β = -(yaw_angle - χ)

# المعدلات الزاوية من المحاكاة مباشرة:
p, q, r = ω_body

# الموقع (مُطبَّع):
h = altitude_AGL / 100
x = x_ground / 1000    
y = y_ground / 1000

# الزعانف (مأمورة + فعلية من actuator model):
δe_s, δr_s, δa_s = last_commanded
δe_act, δr_act, δa_act = from_actuator_positions
```

### دمج تقدير MHE

عند تفعيل MHE، يُمزج المتحكم تقديرات الموقع مع القيم الحقيقية:

```python
if mhe_enabled and quality > 0.3:
    # مزج زمني: 0 عند t≤5.5s, 1 عند t≥8s
    blend = clamp((t - 5.5) / 2.5, 0, 1)
    
    # إخماد عند الارتفاعات المنخفضة (< 150m AGL)
    if alt_agl < 150:
        blend *= max(0, (alt_agl - 50) / 100)
    
    if blend > 0:
        # مزج الموقع فقط (h, x, y) — لا V, γ, χ
        x_mpc[9]  = (1-blend)*truth_h  + blend*mhe_h
        x_mpc[10] = (1-blend)*truth_x  + blend*mhe_x
        x_mpc[11] = (1-blend)*truth_y  + blend*mhe_y
        
        # تعويض الاتجاه من الرياح (اختياري)
        if heading_comp_enabled:
            _apply_wind_compensation(wind_estimate, V, t)
```

**ملاحظة مهمة:** لا يُمزج V, γ, χ من MHE لأن الفرق في نموذج السحب (polynomial vs CFD table) يُسبب انحياز ~18 m/s في V يُزعزع استقرار MPC.

### تعويض الاتجاه (Heading Compensation)

```python
def _apply_wind_compensation(d_hat, V, t):
    # الزاوية الخام: χ_corr = atan2(w_E, V_fwd)
    chi_raw = atan2(w_E, max(V, 50))
    
    # 1. إشباع: |χ_corr| ≤ 8°
    # 2. تحديد معدل: |Δχ| ≤ 15°/s × dt
    # 3. مرشح تمرير منخفض: f_c = 2 Hz
    
    # النتيجة تُطبق كانحياز في yref (وليس تراكمياً):
    yref[2] = chi_ref - chi_comp_filt
```

### معاملات متغيرة مع الزمن

```python
def _get_params(t):
    if t < burn_time:
        f = t / burn_time
        mass = mass_full - f*(mass_full - mass_dry)   # استهلاك خطي
        thrust = thrust_avg
        Ixx = Ixx_full - f*(Ixx_full - Ixx_dry)
    else:
        mass, thrust = mass_dry, 0.0
        Ixx, Iyy, Izz = dry values
    
    return [mass, thrust, Ixx, Iyy, Izz, launch_alt, τ_servo, w_N, w_E]
```

---

## 6. نموذج MHE في acados

### الملف: `mpc/m130_mhe_model.py`

### متجه الحالة (17 حالة)

```
x[0:12]  = [V, γ, χ, p, q, r, α, β, φ, h, x, y]    نفس حالات MPC (بدون زعانف)
x[12:15] = [b_gx, b_gy, b_gz]                        انحياز الجيروسكوب (rad/s)
x[15:17] = [w_n, w_e]                                 تقدير الرياح (m/s)
```

### متجه "التحكم" = ضوضاء العملية (17 عنصراً)

في MHE، متغيرات القرار هي ضوضاء العملية w على كل حالة:

$$\dot{x} = f(x, p) + w$$

### متجه المعاملات (9)

```
p[0:3] = [δe_act, δr_act, δa_act]    انحرافات الزعانف الفعلية (معروفة)
p[3:9] = [mass, thrust, Ixx, Iyy, Izz, launch_alt]
```

### الديناميكا الهوائية

**نفس متعددات الحدود CFD68 من الدرجة 7 المستخدمة في MPC** — هذا يضمن اتساق النموذجين.

### ديناميكا إضافية خاصة بـ MHE

```python
# الانحياز: مشي عشوائي
ḃ_gx = 0 + w[12]
ḃ_gy = 0 + w[13]
ḃ_gz = 0 + w[14]

# الرياح: مشي عشوائي
ẇ_n = 0 + w[15]
ẇ_e = 0 + w[16]
```

### نموذج القياس h(x, p) — 13 قياساً

يُعرّف في `build_m130_mhe_measurement_expr()`:

```python
# الجيروسكوب (3): ω_body + bias
h[0:3] = [p + b_gx, q + b_gy, r + b_gz]

# التسارعميتر (3): القوة النوعية في إطار الجسم
h[3] = (T·cos(α)·cos(β) - D) / m        # محوري
h[4] = -L_yaw / m                        # جانبي
h[5] = -L_pitch / m                      # عمودي

# البارومتر (1):
h[6] = h × 100 + launch_alt              # ارتفاع MSL

# GPS موقع (3):
h[7:10] = [x × 1000, y × 1000, h × 100 + launch_alt]

# GPS سرعة (3): سرعة أرضية (تتضمن الرياح!)
h[10] = V·cos(γ)·cos(χ) + w_n            # شمال
h[11] = V·cos(γ)·sin(χ) + w_e            # شرق
h[12] = -V·sin(γ)                        # عمودي
```

**العلاقة الحرجة:** GPS يقيس السرعة الأرضية = V_air + wind. MHE يُفصل بين V (هوائية) و w (رياح) من خلال مقارنة GPS بنموذج الديناميكا.

---

## 7. مسألة التقدير MHE OCP

### الملف: `mpc/m130_mhe_ocp_setup.py`

### الصياغة الرياضية

$$\min_{x_{0:N}, w_{0:N-1}} \underbrace{\|x_0 - \bar{x}\|^2_{P_0}}_{\text{تكلفة الوصول}} + \sum_{k=0}^{N-1} \left[ \underbrace{\|y_k - h(x_k)\|^2_{R^{-1}}}_{\text{ملاءمة القياسات}} + \underbrace{\|w_k\|^2_{Q^{-1}}}_{\text{ضوضاء العملية}} \right]$$

### معاملات النافذة المنزلقة

| المعامل | القيمة الافتراضية | الوصف |
|---------|-------------------|-------|
| N_mhe | 20 | عدد خطوات الأفق |
| dt | 20 ms | خطوة زمنية |
| أفق كلي | 400 ms | N_mhe × dt |
| معدل الحل | 50 Hz | كل 20 ms |

### أوزان القياسات R

| القياس | σ أصلي | عامل الثقة | σ فعلي | التأثير |
|--------|--------|------------|--------|---------|
| الجيروسكوب | 0.003 rad/s | 0.3 (أقل ثقة) | 0.010 | يُفضّل الديناميكا على القياس |
| التسارعميتر | 0.08 m/s² | 0.5 (أقل ثقة) | 0.16 | يتحمل خطأ النموذج |
| البارومتر | 1.2 m | 1.0 | 1.2 | كما هو |
| GPS موقع | 2.5 m | 1.5 (أكثر ثقة) | 1.67 | يُعزز دقة الموقع |
| GPS سرعة | 0.6 m/s | 2.0 (أكثر ثقة) | 0.30 | يُعزز تقدير الرياح |

### أوزان ضوضاء العملية Q

| الحالة | σ_w | التأثير |
|--------|-----|---------|
| V | 1.0 | **مرن جداً** — يمتص خطأ الديناميكا الهوائية |
| γ, χ | 0.015 | معتدل |
| p, q, r | 0.08 | محكم — يثق بالديناميكا |
| α, β, φ | 0.02 | محكم |
| h | 0.3 | معتدل |
| x, y | 0.5 | مرن |
| b_g | 5×10⁻⁴ | **بطيء جداً** — المشي العشوائي بطيء |
| w_wind | 0.05 | بطيء — الرياح لا تتغير سريعاً |

### تكلفة الوصول P₀

تُمثل عدم اليقين في الحالة عند بداية النافذة:

```python
P0_stds = [15.0, 0.15, 0.15,       # V, γ, χ — واسع
           0.8, 0.8, 0.8,           # p, q, r
           0.2, 0.2, 0.2,           # α, β, φ
           20.0, 40.0, 40.0,        # h, x, y — واسع جداً
           0.02, 0.02, 0.02,        # b_g — ضيق (يتغير ببطء)
           8.0, 8.0]                # wind — واسع (غير معروف مبدئياً)
```

### بناء OCP في acados

```python
ocp = AcadosOcp()
ocp.model = mhe_model

# المرحلة 0: قياسات + ضوضاء + تكلفة الوصول
ocp.cost.cost_type_0 = "NONLINEAR_LS"
ocp.model.cost_y_expr_0 = vertcat(h_meas, w, x)    # h(x) + w + arrival
ocp.cost.W_0 = block_diag(R, Q, P0)                 # أوزان مركبة
ocp.cost.yref_0 = [y_meas₀, 0, x̄]                  # قياسات + صفر + وصول

# المراحل 1..N-1: قياسات + ضوضاء (بدون arrival)
ocp.cost.cost_type = "NONLINEAR_LS"
ocp.model.cost_y_expr = vertcat(h_meas, w)
ocp.cost.W = block_diag(R, Q)
ocp.cost.yref = [y_meas_k, 0]

# المرحلة النهائية: فارغة
ocp.cost.cost_type_e = "LINEAR_LS"
ocp.cost.W_e = 0  # لا تكلفة نهائية
```

---

## 8. مقدّر الحالة MHE Estimator

### الملف: `mpc/m130_mhe_estimator.py`

### البنية الداخلية

```python
class MheEstimator:
    _meas_buf: list     # مخزن القياسات [(t, y[13])]
    _ctrl_buf: list     # مخزن التحكم [(t, fins[3])]
    _param_buf: list    # مخزن المعاملات [(t, p[9])]
    _x_bar: ndarray     # حالة تكلفة الوصول [17]
    _solver: AcadosOcpSolver
```

### دورة الحل

```
push_measurement(t, y_meas)  ──────►  مخزن القياسات
push_control_and_params(t, u, p) ──►  مخزن التحكم/المعاملات
                                          │
update(t) ◄───────────────────────────────┘
    │
    ├─ تحديد معدل: كل 20ms
    ├─ قص المخزن: الاحتفاظ بآخر N+1 قياس
    ├─ بناء yref لكل مرحلة
    ├─ حل acados → x_hat[17]
    ├─ تحديث تكلفة الوصول: x̄ = solver.get(1, "x")
    ├─ حساب مقياس الجودة
    └─ إرجاع MheOutput
```

### مقياس الجودة

```python
quality = 1.0
if solver_status in (3, 4):  # فشل QP
    quality *= 0.5
if w_norm > 5.0:             # ضوضاء عملية كبيرة
    quality *= max(0.1, 1 - (w_norm - 5)/15)

# يُستخدم كبوابة: إذا quality < 0.3 → لا مزج مع MPC
```

### هيكل البيانات المُخرجة

```python
@dataclass
class MheOutput:
    x_hat: ndarray         # [17] تقدير الحالة
    d_hat: dict            # {'gyro_bias': [3], 'wind_ne': [2]}
    quality: float         # 0.0→1.0
    status: int            # 0=OK, 2=maxiter, 4=QP_fail
    solve_time_ms: float
    valid: bool
```

---

## 9. ناقل الحساسات Sensor Bus

### الملف: `mpc/m130_sensor_bus.py`

### الحساسات المحاكاة

| الحساس | المعدل | الضوضاء σ | الانحياز | التأخير |
|--------|--------|-----------|----------|---------|
| الجيروسكوب | 200 Hz | 0.003 rad/s | N(0, 0.005) ثابت | 0 |
| التسارعميتر | 200 Hz | 0.08 m/s² | N(0, 0.1) ثابت | 0 |
| البارومتر | 25 Hz | 1.2 m | — | 30 ms |
| GPS موقع | 5 Hz | 2.5 m | — | 80 ms |
| GPS سرعة | 5 Hz | 0.6 m/s | — | 80 ms |

### ميزات خاصة

- **انقطاع GPS:** يُحاكي فقدان إشارة GPS خلال نافذة زمنية قابلة للتكوين
- **احتفاظ بآخر قيمة:** عند عدم تحديث GPS/بارومتر، يُعاد آخر قياس

### متجه القياس (13 عنصراً)

```python
y_meas = [
    ω_x + b_gx + noise,    # gyro_x
    ω_y + b_gy + noise,    # gyro_y
    ω_z + b_gz + noise,    # gyro_z
    a_body_x + b_ax + noise, # accel_x
    a_body_y + b_ay + noise, # accel_y
    a_body_z + b_az + noise, # accel_z
    alt_true + noise,       # baro_alt
    x_ground + noise,       # gps_x
    y_ground + noise,       # gps_y
    alt_true + noise,       # gps_z
    v_n + noise,            # gps_vn
    v_e + noise,            # gps_ve
    v_d + noise,            # gps_vd
]
```

---

## 10. تدفق البيانات الكامل

### الدورة الكاملة لخطوة واحدة

```
الزمن t
═══════

محاكاة 6DOF ──►  state_derivative(state, t, fins)
    │                    │
    │                    ├── الدفع: T(t), m(t)
    │                    ├── الغلاف الجوي: ρ(h), wind(h)
    │                    ├── الديناميكا الهوائية: Cn, Cm, CD ... (CFD tables)
    │                    ├── السيرفو: δ_act = f(δ_cmd, τ)
    │                    └── snapshot: forces, moments, aero angles
    │
    ├── RK4 ──► next_state[14]
    │
    ├── snapshot ──► SensorBus.update(snapshot, t)
    │                    │
    │                    └── y_meas[13] ──► MHE.push_measurement()
    │
    ├── MHE.update(t)
    │       │
    │       ├── acados solve (SQP_RTI, 25 خطوة, 500ms أفق)
    │       ├── x_hat[17] = solver.get(N, "x")
    │       ├── wind_ne = x_hat[15:17]
    │       ├── gyro_bias = x_hat[12:15]
    │       └── quality = f(status, w_norm)
    │
    └── MheOutput ──► مخزن مؤقت
    
الزمن t + dt
════════════

state_dict['mhe_output'] = MheOutput   حقن
    │
    ▼
MpcController.control_function(state_dict, t+dt)
    │
    ├── 1. استخراج حالة MPC (18) من state_dict
    │       └── تحويل سرعة NED → V, γ, χ هوائية
    │
    ├── 2. مزج MHE (اختياري):
    │       ├── مزج موقع (h, x, y) من MHE
    │       ├── استخراج رياح → _wind_est
    │       └── تعويض اتجاه → chi_comp_filt
    │
    ├── 3. حساب التوجيه:
    │       ├── γ_ref = LOS angle + impact shaping
    │       ├── χ_ref = atan2(Δy, Δx) - chi_comp
    │       └── rate limiting: 12°/s max
    │
    ├── 4. تهيئة acados MPC:
    │       ├── solver.set(0, "lbx/ubx", x_mpc)     ► حالة ابتدائية
    │       ├── solver.set(k, "p", params(t+k·dt))   ► معاملات متغيرة
    │       ├── solver.cost_set(k, "W", W_dynamic)    ► أوزان ديناميكية
    │       ├── solver.set(k, "yref", [h,γ,χ,...])    ► مراجع التتبع
    │       └── solver.set_new_time_steps(dt_h)       ► أفق قابل للتقلص
    │
    ├── 5. حل MPC:
    │       ├── عدة تكرارات SQP_RTI (3-8)
    │       ├── استخراج u_opt = solver.get(0, "u")     ► [δ̇e, δ̇r, δ̇a]
    │       └── استخراج x1 = solver.get(1, "x")        ► حالة متنبأة
    │
    └── 6. خلط زعانف + إرجاع:
            δe, δr, δa = x1[12:15]
            fins = [δa-δe-δr, δa-δe+δr, δa+δe+δr, δa+δe-δr]
            return fins[4]
```

### تدفق البيانات بين acados وبقية النظام

```
                    ┌──────────────────┐
 ┌─────────────────►│  acados MPC      │
 │  x0[18]         │  Solver          │
 │  p[9] per node  │                  │──► u_opt[3], x_plan[N+1, 18]
 │  yref[12/11]    │  SQP_RTI         │
 │  W[12×12]       │  + HPIPM         │
 └─────────────────│  + ERK(4,2)      │
                    └──────────────────┘

                    ┌──────────────────┐
 ┌─────────────────►│  acados MHE      │
 │  yref: y_meas   │  Solver          │
 │  p[9] per node  │                  │──► x_hat[17], quality
 │  arrival: x̄     │  SQP_RTI         │
 │                  │  + HPIPM         │
 └─────────────────│  + ERK(4,2)      │
                    └──────────────────┘
```

---

## 11. الإعدادات والتكوين

### الملف: `config/6dof_config_advanced.yaml`

### الأقسام الرئيسية

```yaml
# ── بيانات الصاروخ ──
rocket_data:
  name: "rocket_M130_v01_CFD68"    # 68 نقطة CFD
  multi_stage: false

# ── المحاكاة ──
simulation:
  duration: 500                     # ثانية
  dt: 0.01                         # خطوة زمنية (100 Hz)
  control_type: "mpc"              # "none" | "mpc"
  use_actuator_dynamics: true

# ── المدى الطويل ──
long_range:
  enabled: true
  gravity_model: "j2"             # تصحيح شكل الأرض
  include_coriolis: true
  use_launch_fixed_ned: true       # NED ثابت عند نقطة الإطلاق

# ── الإطلاق ──
launch:
  latitude: 16.457472              # °N
  longitude: 44.115361             # °E
  altitude: 1200.0                 # m MSL

# ── الهدف ──
target:
  range_m: 2600                    # m
  altitude: 1180.0                 # m MSL

# ── الطيار الآلي MPC ──
autopilot:
  mpc:
    N_horizon: 80                  # خطوات الأفق
    tf: 4.0                       # s — زمن الأفق
    t_ctrl: 0.5                   # s — بدء التحكم
    dt_solve: 0.02                # s — معدل الحل (50 Hz)
    delta_rate_limit_dps: 250     # °/s — حد معدل الزعانف

# ── التقدير MHE ──
estimation:
  mode: "mhe"                     # "off" | "mhe"
  use_estimated_state_for_mpc: true
  
  compensation:
    enable_heading_compensation: true    # تعويض الاتجاه من الرياح
    enable_wind_feedforward: false       # إيقاف تغذية أمامية للرياح (غير مستقر)
    chi_correction_max_deg: 8.0
    chi_correction_rate_limit_dps: 15.0
    chi_correction_lpf_hz: 2.0
  
  mhe:
    horizon_steps: 25
    horizon_dt: 0.02              # 20 ms
    solve_rate_hz: 50.0
    quality_gate_threshold: 0.3
    
    # أوزان الثقة
    gps_vel_trust_factor: 2.0     # ثقة أعلى في GPS
    gps_pos_trust_factor: 1.5
    accel_trust_factor: 0.5       # ثقة أقل (خطأ النموذج)
    gyro_trust_factor: 0.3        # ثقة أقل
  
  sensors:
    imu_rate_hz: 200
    gps_rate_hz: 5
    baro_rate_hz: 25
    gyro_noise_std: 0.003
    accel_noise_std: 0.08
    gps_pos_noise_std: 2.5
    gps_vel_noise_std: 0.6
    baro_noise_std: 1.2

# ── الغلاف الجوي ──
atmosphere:
  wind_enabled: false
  wind_speed: 0.0
  wind_direction: 0.0
```

### تحميل الإعدادات مع Overlay

يدعم دمج ملفات overlay فوق الإعدادات الأساسية:

```bash
python rocket_6dof_sim.py --config /tmp/crosswind.yaml
```

حيث `crosswind.yaml` يحتوي الحقول المراد تغييرها فقط:

```yaml
atmosphere:
  wind_enabled: true
  wind_speed: 8.0
  wind_direction: 90.0
estimation:
  compensation:
    enable_heading_compensation: true
```

---

## 12. الأداء ونتائج الاختبارات

### بيانات الصاروخ M130

| الخاصية | القيمة |
|---------|--------|
| القطر | 130 mm |
| المساحة المرجعية | 0.0133 m² |
| الكتلة (ممتلئ) | ~47 kg |
| الكتلة (جاف) | 12.74 kg |
| الدفع المتوسط | ~620 N |
| زمن الاحتراق | ~4.5 s |
| Ixx (جاف) | 0.0353 kg·m² |
| Iyy = Izz (جاف) | 1.0567 kg·m² |
| τ_servo | 15 ms |
| δ_max | ±20° |

### أوقات الحل النموذجية

| المكون | زمن الحل | المعدل |
|--------|----------|--------|
| MPC (SQP_RTI × 3) | 4-30 ms | 50 Hz |
| MHE (SQP_RTI × 1) | 5-15 ms | 50 Hz |
| محاكاة 6DOF (RK4) | ~0.1 ms | 100 Hz |

### نتائج اختبارات الإجهاد

| السيناريو | المدى | أقصى لف | NaN | الحكم |
|-----------|-------|---------|-----|-------|
| S1: بدون رياح, MHE OFF | 2593 m | 9.5° | 0 | ✅ |
| S2: بدون رياح, MHE ON | 2593 m | 9.5° | 0 | ✅ |
| S3: رياح 8 م/ث, MHE OFF | 2252 m | 172° | 0 | ✅ (مرجع) |
| S4: رياح 8 م/ث, MHE ON | 2395 m | 67° | 0 | ⚠️ |
| S5: انحياز 5×, MHE ON | 2593 m | 9.5° | 0 | ✅ |
| S6: انقطاع GPS 2s, MHE ON | 2593 m | 9.5° | 0 | ✅ |

**ملاحظة S4:** اللف 67° يحدث فقط تحت 70m AGL قبل الاصطدام. في S3 (بدون MHE) اللف 172° — MHE حسّن الأداء بشكل كبير.

### القيود المعروفة

1. **انحياز V في MHE ~18 m/s:** بسبب الفرق بين متعدد الحدود (في MHE) وجداول CFD (في المحاكاة) لمعامل السحب CD. يُخفف بعدم مزج V, γ, χ.

2. **Wind feedforward غير مستقر:** تحويل السرعة الأرضية→الهوائية يعتمد على دقة تقدير الرياح. أي خطأ يُضخّم عبر معادلات MPC. معطّل حالياً.

3. **لا تأثير رياح مباشر في ẋ الديناميكية:** معادلات V̇, γ̇, χ̇ في نموذج MPC لا تتضمن قوى الرياح — فقط معادلات الموقع تتضمنها.

---

## الملحق: ربط مكتبة acados

### التثبيت والمسارات

```
acados-main/
├── lib/
│   ├── libacados.so        ← المكتبة الرئيسية
│   ├── libhpipm.so         ← حل QP (interior-point)
│   └── libblasfeo.so       ← BLAS مُخصص لـ HPIPM
├── interfaces/
│   └── acados_template/    ← Python interface
│       ├── acados_template/
│       │   ├── acados_ocp.py
│       │   ├── acados_ocp_solver.py
│       │   └── acados_model.py
│       └── ...
└── include/
    └── acados/             ← C headers
```

### متطلبات التشغيل

```bash
export LD_LIBRARY_PATH=/path/to/acados-main/lib
export ACADOS_SOURCE_DIR=/path/to/acados-main
```

### دورة حياة الحل

```python
# 1. بناء النموذج (CasADi) — مرة واحدة
model = create_m130_model()          # → AcadosModel مع تعبيرات رمزية

# 2. صياغة OCP — مرة واحدة
ocp = create_m130_ocp()              # → AcadosOcp مع تكلفة وقيود

# 3. بناء الحل (يُولد C code + يُترجم) — مرة واحدة
solver = AcadosOcpSolver(ocp)        # → .c → .so قابل للربط

# 4. استخدام الحل — كل خطوة تحكم (50 Hz)
solver.set(0, "lbx", x_current)      # حالة ابتدائية
solver.set(k, "p", params_k)         # معاملات
solver.set(k, "yref", ref_k)         # مراجع
solver.cost_set(k, "W", weights)     # أوزان
status = solver.solve()              # ← هذا يُشغل C code المُولد
u_opt = solver.get(0, "u")           # الحل الأمثل
```

### توليد الشيفرة (Code Generation)

عند أول استدعاء لـ `AcadosOcpSolver()`:

1. acados يُحول تعبيرات CasADi → ملفات C
2. يُترجم إلى `*.so` باستخدام `cc -O2`
3. يحفظ hash للمصادر — لا يُعيد الترجمة إذا لم تتغير
4. عمليات الحل اللاحقة تستدعي C مباشرة (بدون Python overhead)

```
              ┌─────────────┐
CasADi ──────►│ Code Gen    │──► m130_rocket_model.c
expressions   │ (acados)    │──► m130_rocket_expl_ode_fun.c
              └──────┬──────┘    m130_rocket_impl_ode_fun.c
                     │
                     ▼
              ┌─────────────┐
              │  cc -O2     │──► libm130_mpc.so
              │  (compile)  │
              └──────┬──────┘
                     │
                     ▼
              ┌─────────────┐
   Python ───►│  ctypes     │   solver.solve() → C function call
              │  (FFI)      │   ~5ms per solve
              └─────────────┘
```
