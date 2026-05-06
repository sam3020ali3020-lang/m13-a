# 09 — تقدير الحالة بـ MHE (Moving Horizon Estimation)

> **MHE = Moving Horizon Estimator** — يُقدِّر الحالة الحقيقية للصاروخ من قياسات المستشعرات الخامة عبر **نافذة زمنية متحرّكة** بدلاً من filter مباشر.

## 🎯 لماذا MHE وليس EKF فقط؟

| الخاصية | EKF | MHE |
|---|---|---|
| نوع الحسابات | لحظية (point estimate) | تحسين OCP على نافذة |
| دقّة مع الضوضاء | متوسّطة | **أعلى** (يستفيد من تاريخ القياسات) |
| التعامل مع biases | يحتاج augmentation | **ضمنياً** في state vector |
| التكلفة الحاسوبية | منخفضة | **متوسّطة** (acados سريع) |
| الثبات عند التشبّع | يتشبّع | **أفضل** (constraints صريحة) |

**القرار:** النظام يستخدم **EKF2 + MHE على التوازي**:
- EKF2 (PX4 الداخلي) → تقدير أوّلي سريع
- MHE → تحسين على نافذة 20 خطوة × 20ms = 400ms

## 🏗️ أين يعمل MHE في المشروع؟

### 1) في محاكاة Python (6DOF_v4_pure)
**الملف:** [mpc/m130_mhe_estimator.py](../mpc/m130_mhe_estimator.py)
**التفعيل:** `config/6dof_config_advanced.yaml` → `estimation.mode: "mhe"`
**الاستخدام:** [rocket_6dof_sim.py:3312](../rocket_6dof_sim.py) — يغلِّف MPC control function

### 2) في الهاتف (Android APK)
**الملف:** [AndroidApp/.../rocket_mpc/mhe_estimator.cpp](../../AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/mhe_estimator.cpp)
**المُستدعي:** `RocketMPC.cpp:180` (init), `:504` (push_measurement), `:528` (push_params), `:531` (solve)
**كود acados:** [c_generated_code/acados_solver_m130_mhe.c](../../../c_generated_code/acados_solver_m130_mhe.c)

### 3) في SITL / HITL
- **SITL:** `sitl/run_sitl_qgc_udp.py` → يستخدم نفس Python estimator (وضع closed-loop)
- **HITL:** الهاتف يُشغِّل MHE الخاص به (مصدر الحقيقة: PC physics)

## 🧩 معمارية MHE

### حالة النموذج (17 × 1)
```
x = [V  γ  χ  p  q  r  α  β  φ  h  x_g  y_g  b_gx  b_gy  b_gz  w_N  w_E]
    │  │  │  │  │  │  │  │  │  │   │    │    │     │     │     │    │
    │  │  │  │  │  │  │  │  │  │   │    │    │     │     │     │    └─ wind East
    │  │  │  │  │  │  │  │  │  │   │    │    │     │     │     └────── wind North
    │  │  │  │  │  │  │  │  │  │   │    │    └─────┴─────┴──────────── gyro biases
    │  │  │  │  │  │  │  │  │  │   │    └─ position N
    │  │  │  │  │  │  │  │  │  │   └────── position E
    │  │  │  │  │  │  │  │  │  └────────── altitude
    │  │  │  │  │  │  │  │  └───────────── roll
    │  │  │  │  │  │  │  └──────────────── sideslip
    │  │  │  │  │  │  └─────────────────── angle of attack
    │  │  │  │  │  └──────────────────── body rates
    │  │  │  └──┴─────────────────────── ...
    │  │  └──────────────────────────── heading
    │  └─────────────────────────────── flight path angle
    └────────────────────────────────── velocity magnitude
```

### قياسات (13 × 1)
```
y = [a_x  a_y  a_z  ω_x  ω_y  ω_z  x_GPS  y_GPS  v_x  v_y  v_z  h_baro  ψ_mag]
```

### نموذج الديناميكا (rhs)
- معادلات 6DOF كاملة
- نماذج احتكاك + aerodynamic
- Runge-Kutta 4th order على كل خطوة

## ⚙️ معاملات MHE (من `config/6dof_config_advanced.yaml`)

```yaml
mhe:
  solve_rate_hz: 50.0              # 50Hz = كل 20ms
  horizon_steps: 20                # نافذة 20 خطوة
  horizon_dt: 0.02                 # كل خطوة 20ms → نافذة 400ms
  max_qp_iters: 100
  levenberg_marquardt: 0.1         # تنظيم Tikhonov
  max_consecutive_failures: 10
  quality_gate_threshold: 0.3      # رفض حلول أقل من 30% جودة
  process_noise_scale: 1.0
  arrival_cost_scale: 1.5          # وزن prior
  gyro_trust_factor: 0.3
  gps_vel_trust_factor: 5.0        # نثق جداً في GPS velocity
  gps_pos_trust_factor: 1.5
  accel_trust_factor: 0.3
  wind_abs_bound_mps: 0.1          # حد الرياح (صغير لأنّ طيراننا لا رياح)
  gyro_bias_abs_bound: 0.01
```

## 🔄 دورة حياة MHE (كل 20ms)

```
1) push_measurement(t, y)    ← إضافة قياس للنافذة
2) push_params(t, p)          ← إضافة Mach, mass, CG, …
3) حذف قياسات أقدم من (t - N·dt)
4) إذا لديك ≥ min_init_measurements:
     solve()  ← حلّ OCP عبر acados SQP-RTI
5) إذا status=0 و quality ≥ gate:
     x_hat = solver.get(N, "x")
     d_hat = { gyro_bias, wind_ne }
     quality *= ramp (أوّل 5 حلول)
   وإلا:
     fallback إلى آخر حل صالح
     _consec_fails += 1
6) إذا _consec_fails > 10:
     reset() ← أعد التهيئة بالكامل
```

## ✅ كيف تتأكّد أنّ MHE يعمل صحيحاً؟

### في Python sim:
```bash
python3 rocket_6dof_sim.py 2>&1 | grep -i "MHE\|estimator"
```
يجب أن ترى:
```
MheEstimator created: N=20, dt=0.02s, solve_rate=50.0Hz
MHE estimation enabled (تقدير MHE مفعّل)
```

### في HITL (الهاتف):
أثناء التشغيل راقب في لوغ PX4:
```bash
adb logcat | grep -i "mhe\|rocket_mpc"
```
يجب أن ترى:
```
rocket_mpc: MHE init OK, N=20, dt=20ms
rocket_mpc: MHE solve OK, q=0.95, t=2.1ms
```

### بنيوياً (offline):
بعد كل تشغيل HITL، CSV يحتوي على:
- أعمدة `mhe_v, mhe_gamma, …` ← تقدير MHE
- أعمدة `true_v, true_gamma, …` ← الحقيقة من physics
- أعمدة `ekf_v, ekf_gamma, …` ← تقدير PX4 EKF2

**مقياس الجودة:** `RMSE(mhe - true) < RMSE(ekf - true)` في حالة الضوضاء العالية.

### افتراضيات MHE (يجب أن تتطابق مع النموذج):
| المعامل | Physics | MHE |
|---|---|---|
| burn_time | 4.35s (Qabthah1) | يتم تمريره كـparameter |
| mass_t | mass integrator | يتم تمريره |
| gravity | 9.80665 | يفترض ثابت |
| CG | mass-properties | يتم تمريره |
| إن didn't match → MHE يتباعد! |

## ⚠️ مشاكل شائعة وحلولها

### 1) MHE status=2 (MAXITER)
- **السبب:** الضوضاء عالية أو horizon طويل
- **الحل:** قلّل `horizon_steps` إلى 15 أو زِد `levenberg_marquardt`

### 2) MHE quality < 0.3
- **السبب:** القياسات متناقضة أو GPS مفقود
- **الحل:** يرجع للحل السابق تلقائياً (fallback). راقب `_consec_fails`

### 3) MHE يُعطي biases ضخمة (> 0.05)
- **السبب:** `gyro_trust_factor` عالٍ أو نموذج خاطئ
- **الحل:** قلّل trust إلى 0.1، تحقّق من اتساق إشارات gyro

### 4) MHE ≠ EKF2 بفارق كبير
- **السبب:** طبيعي! MHE يستفيد من التاريخ، EKF2 لا.
- **يصبح مشكلة فقط إذا:** الفارق > 5m في الموقع أو > 2° في attitude

## 🧪 اختبار MHE منعزلاً (unit test)

```bash
cd ~/Desktop/nmn/px4/m13/6DOF_v4_pure
python3 -c "
from mpc.m130_mhe_estimator import MheEstimator
import yaml
cfg = yaml.safe_load(open('config/6dof_config_advanced.yaml'))
est = MheEstimator(cfg['estimation'])
print('MHE OK:', est._N, 'steps, dt =', est._dt, 's')
"
```

## 📊 مقارنة نتائج: مع MHE vs بدون MHE

يمكنك تشغيل sim مرّتين:
```bash
# مع MHE
python3 rocket_6dof_sim.py
# بدون MHE (عدّل estimation.mode: "off")
python3 rocket_6dof_sim.py --estimation-mode off
```

النتائج المتوقّعة:
- **مع MHE:** RMSE altitude < 3m, RMSE attitude < 0.5°
- **بدون MHE:** RMSE altitude 5-8m, RMSE attitude 1-2°

## 🔗 روابط

- النموذج الرياضي: [mpc/m130_mhe_model.py](../mpc/m130_mhe_model.py)
- إنشاء OCP: [mpc/m130_mhe_ocp_setup.py](../mpc/m130_mhe_ocp_setup.py)
- Wrapper Python: [mpc/m130_mhe_estimator.py](../mpc/m130_mhe_estimator.py)
- Implementation على الهاتف: [rocket_mpc/mhe_estimator.cpp](../../AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/mhe_estimator.cpp)
- Integration في المحاكاة: [rocket_6dof_sim.py:3300-3340](../rocket_6dof_sim.py)
