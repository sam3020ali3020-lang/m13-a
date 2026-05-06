# ما لم يُختبر بعد — M13 Pending Tests

> آخر تحديث: **2026-05-03**
> الهدف: قائمة كاملة بكل شيء **يحتاج اختباراً** قبل أول إطلاق حقيقي، مع
> سبب الأولوية والأمر المطلوب للتشغيل.

---

## 1. ملخص الحالة العامة

| الفئة | نُفّذ ✅ | معلّق ⬜ | حرج 🔴 |
|-------|:-------:|:--------:|:------:|
| Sensor characterization | 6/8 | 2 | 1 |
| End-to-end latency      | 0/3 | 3 | 3 |
| Thermal stress (MPC)    | 0/1 | 1 | 1 |
| HITL / PIL / Real flight| 0/3 | 3 | 3 |
| Sim config validation   | 2/6 | 4 | 1 |
| Firmware field-tuning   | 0/3 | 3 | 2 |

**المجموع: 8 اختبارات منجزة ، 16 معلّقة (7 منها حرجة قبل الإطلاق).**

---

## 2. ما تمّ اختباره فعلاً (للمرجع)

| الاختبار | التاريخ | النتيجة |
|----------|---------|---------|
| `/sensor static` (5 min) | 2026-05-03 | PASS — ضجيج accel ≈ 0.08 m/s² ، gyro ≈ 0.002 rad/s |
| `/sensor rates` | 2026-05-03 | PASS — IMU @ 100 Hz مستقر |
| `/sensor frame` | 2026-05-03 | PASS — بعد إصلاح إشارة `vertical_top_up` |
| `/sensor temperature` (10 min) | 2026-05-03 | PASS — 0.011 m/s²/°C (R²=0.22) بعد فلتر الحركة |
| `/sensor allan` (30 min) | 2026-05-03 | PASS — gyro BI **8.02 °/h** (gz)، accel BI **0.047 mg** (ay) |
| `/sensor dynamic_range` | 2026-05-03 | PASS |
| `/direct` repeatability | 2026-05-02 | PASS — pure_delay **31.72 ± 0.20 ms** (R²=0.96) |
| `/lab` PX4-SITL↔CAN | 2026-05-02 | PASS — servo delay 50–80 ms، backlash ≈ 3° |
| `/thermal_stress` (35 s، بدون MPC) | 2026-05-03 | PASS — skin 37.6°C max، لا throttling SEVERE |
| Simulation 6DOF (Qabthah1) | 2026-05-02 | **100/100** — range err −0.4% |

---

## 3. الاختبارات المعلّقة — مرتّبة حسب الأولوية

### 🔴 P0 — حرج جداً (لا يُطلق قبلها)

#### 3.1 `/e2e_latency` — passive + tap + sweep
- **الحالة:** شُغِّل مرة واحدة ⇒ جميع streams = 0 Hz ⇒ **NO-GO** (التطبيق لم يكن ينشر GNC).
- **لماذا حرج:** بدون قياس فعلي لـ `L_sensor + L_mpc + L_actuator` لا نعرف هامش الاستقرار مقابل `RKT_MPC_SVO_DLY = 100 ms` المفروض.
- **ما يحتاج قبل إعادة التشغيل:**
  1. التأكد أن PX4 على الهاتف ينشر `HIGHRES_IMU` و `ATTITUDE` و `DEBUG_FLOAT_ARRAY id=1 (SRV_FB)` و `id=2 (RktGNC)` عبر MAVLink TCP 5760.
  2. ربط CAN بسيرفو واحد على الأقل (للحصول على `SRV_FB`).
- **الأمر:**
  ```bash
  python3 e2e_latency/e2e_runner.py --preset passive    # أولاً (يجب Hz>90 و Hz>30)
  python3 e2e_latency/e2e_runner.py --preset tap        # قياس L_sensor فعلياً
  python3 e2e_latency/e2e_runner.py --preset sweep      # قياس L_actuator بسحب chirp
  ```
- **عتبات النجاح:** total transport ≤ 80 ms (المتوقّع ≈ 50–60 ms).

#### 3.2 `/thermal_stress` مع PX4 + MPC حقيقيّين (30 دقيقة)
- **الحالة:** شُغِّل 35 s فقط **بدون** MPC ⇒ `mpc_timing.csv` فارغ.
- **لماذا حرج:** هدف الاختبار هو قياس **deadline misses** في حلقة MPC تحت الحرارة.
  بدون MPC يصبح الاختبار مجرد قياس CPU temp لا أكثر.
- **متطلّبات الإعادة:**
  1. PX4 شغّال + rocket_mpc active (HITL أو PIL أو real sensors).
  2. `e2e_latency/e2e_reader.py` يستقبل `RktGNC` لتعبئة `mpc_timing.csv`.
  3. مدة ≥ 30 min (حتى يدخل الهاتف منطقة MODERATE/SEVERE throttling).
- **الأمر:**
  ```bash
  python3 thermal_stress/thermal_stress_runner.py --duration 1800
  ```
- **عتبات:** miss_ratio ≤ 1 % ، p99(cycle_us) ≤ 25 000 µs ، زمن بقاء في SEVERE ≤ 5 %.

#### 3.3 `/sensor vibration` — عربون ضدّ الاهتزاز (critical pre-flight)
- **الحالة:** لم يُشغَّل نهائياً.
- **لماذا حرج:** المقياس الوحيد لـ **clipping** و **rectification bias** (انحياز وهمي يظهر في الطيران ولا نراه في static). هذا تهديد مباشر لـ EKF2.
- **متطلّبات:** الهاتف مثبّت قرب محرّك يهتزّ (أو طاولة shaker).
- **الأمر:**
  ```bash
  python3 sensor/sensor_runner.py --tests vibration --duration 60
  ```
- **عتبات:** max_clipping_pct = 0 %، لا قمم spectral > 0.5 g تحت 100 Hz.

#### 3.3b `/watchdog` — صلاحية WatchdogManager على العتاد
- **الحالة:** البنية التحتية مكتملة (native + Kotlin + Python runner) — لم تُشغَّل على جهاز فعلي بعد.
- **لماذا حرج:** هذا هو أول خط دفاع إذا مات أيّ module (rocket_mpc، ekf2، native_sensor_reader، ...) داخل تطبيق الهاتف. بدون تأكيد زمني فعلي (detection / restart / recovery ضمن thresholds) لا نعرف إن كان WatchdogManager سيصحّح فشلاً حقيقياً قبل أن يفقد الصاروخ السيطرة.
- **متطلّبات قبل التشغيل:**
  1. بناء APK **debug** وتثبيته (release يرفض crash injection بتصميم).
  2. `adb devices` يرى الهاتف، و`Start PX4` مُفعَّل على الشاشة.
  3. `pip install -r 6DOF_v4_pure/watchdog/requirements.txt`.
- **الأوامر:**
  ```bash
  python3 6DOF_v4_pure/watchdog/watchdog_runner.py                    # quick (~2 min)
  python3 6DOF_v4_pure/watchdog/watchdog_runner.py --preset standard  # ~5 min
  python3 6DOF_v4_pure/watchdog/watchdog_runner.py --preset full      # ~15 min (شامل cascading)
  ```
- **عتبات PASS:**
  - detection ≤ 600 ms (stale_threshold + poll_period)
  - restart ≤ 1500 ms
  - recovery ≤ 2500 ms (أول publish جديد)
  - cascading: bystanders تتعافى ≤ 3000 ms

---

### 🟠 P1 — مهمّ جداً (قبل HITL الشامل)

#### 3.4 `/sensor gps` — أداء GPS الحقيقي في مكان مكشوف
- **الحالة:** لم يُشغَّل.
- **لماذا مهمّ:** قيم `estimation.sensors.gps_*` و `EKF2_GPS_*_NOISE` و `bridge.noise.gps_*` في
  `6dof_config_advanced.yaml` **placeholders ـ لم تُقَس**.
- **الأمر:**
  ```bash
  python3 sensor/sensor_runner.py --tests gps --duration 300
  ```
- **عتبات:** CEP ≤ 3 m ، HDOP ≤ 2.0 ، sats ≥ 8 ، vel_noise ≤ 0.3 m/s.

#### 3.5 `/sensor allan` طويل (ساعتان)
- **الحالة:** شُغِّلت 30 دقيقة فقط.
- **لماذا مهمّ:** 30 دقيقة تكفي لرؤية Bias Instability لكن **لا** تكفي لرؤية Rate-Random-Walk (τ > 1000 s).
  هذا يؤثّر مباشرة على قيمة `EKF2_GYR_B_NOISE` و `accel_bias_std` في MHE.
- **الأمر:**
  ```bash
  python3 sensor/sensor_runner.py --preset allan_long   # duration=7200
  ```

#### 3.6 Fault-detection patterns على العتاد الحقيقي
- **الحالة:** 3 patterns جديدة (`preflight_check`, `wiring_audit`, `fault_scan`) مسجّلة ومختبَرة على CSVs تركيبية — لم تُشغَّل يوماً على السيرفوهات الحقيقية.
- **الأوامر (الترتيب الموصى به قبل كلّ إطلاق):**
  ```bash
  python3 direct/direct_runner.py --pattern wiring_audit     # تأكيد عدم تبديل السلوك
  python3 direct/direct_runner.py --pattern preflight_check  # GO/NO-GO شامل 25 s
  python3 direct/direct_runner.py --pattern fault_scan       # مراقبة 30 s للأنواع الخمسة من anomalies
  ```

---

### 🟡 P2 — مطلوب لتوثيق التُّعديلات في السيم

#### 3.7 تحديث نموذج الضجيج/الانحياز في السيم من قياسات GPS/Baro/Mag الفعلية
بعد تشغيل 3.4، تحديث القيم التالية في `config/6dof_config_advanced.yaml`:

| قسم | متغيّرات معلّقة |
|-----|----------------|
| `error_injection` | `sig_gps_pos_noise`, `sig_gps_vel_noise` (حاليا placeholder) |
| `estimation.sensors` | `baro_noise_std`, `mag_noise_std`, `gps_pos_std`, `gps_vel_std` |
| `bridge.noise` | `baro_*`, `mag_*`, `gps_*` |

#### 3.8 إضافة Thermal-drift model للسيم
- **الحالة:** القياس موجود (0.011 m/s²/°C لـ accel)، لكن السيم لا يحقن هذا الانجراف.
- **المطلوب:** دالة في `dynamics/error_injection.py` تضيف `drift = k_T · (T − T_ref)` على كل محور accel/gyro حسب الوقت.
- **يرتبط بـ:** 3.2 (نحتاج رؤية أثر Thermal drift تحت MPC تحت الحرارة).

#### 3.9 إضافة Vibration model للسيم
- **الحالة:** معلّق على 3.3 (قياس الاهتزاز أولاً)، ثم تُحقن قمم spectral كـ band-limited white noise في bridge.

---

### 🟢 P3 — قبل Real Flight مباشرة

#### 3.10 `/lab` مع config محدّث + Preflight
- إعادة `/lab` مع قيم `EKF2` و `RKT_MPC_SVO_DLY = 0.100` المحدّثة، للتأكد أن الحلقة
  مقفولة ومستقرة مع lookahead_stage = 5.

#### 3.11 `run_stress_tests.py` — Monte Carlo robustness
- **الحالة:** لم يُشغَّل بعد تحديث قيم `error_injection.sig_accel_bias_*` من Allan.
- **الأمر:**
  ```bash
  python3 run_stress_tests.py --trials 200
  ```
- **عتبة:** success rate ≥ 95 % ، range σ ≤ 3 % من target.

#### 3.12 HITL كامل (phone + SITL + PX4)
- **الحالة:** بعض أجزاء منفصلة مشتغلة، لكن المسار الكامل
  `phone IMU → PX4 → MPC → servos → dynamics → PX4 → …` لم يُختبر end-to-end لمدة ≥ إطلاق كامل (12 s) بعد إصلاح `lockstep_scheduler`.
- **الشكل:** `hil/mavlink_bridge_hil.py` + Android PX4 + dynamics الخارجية.

#### 3.13 `system_audit.py` قبل الإطلاق
- **الحالة:** موجود كـ script لكن لم يُشغَّل بالتكوين الحالي.
- **الأمر:**
  ```bash
  python3 system_audit.py
  ```
- يتحقّق من: params متّسقة بين Python و PX4، قيم EKF2 داخل حدودها، `RKT_MPC_SVO_DLY` متزامن في الملفّين.

---

## 4. التّوطين الحالي (mid-step applied 2026-05-04)

طُبِّقت قيم **نصف‑المسافة** بين الدفاعي والمقاس في `@/home/yoga/m13/m13/AndroidApp/app/src/main/cpp/px4_jni.cpp:300-335`. الخطة الأصلية كانت "اتركها دفاعية حتى طلعتَين" — الآن أُخذت خطوة واحدة نحو المقاس مع احتفاظ بهامش أمان كبير:

| معلمة (PX4 on phone) | ما قبل (دفاعي) | **المُطبَّق الآن** | المقاس | هامش فوق المقاس |
|----------------------|:--------------:|:------------------:|:------:|:---------------:|
| `EKF2_ACC_NOISE`  | 1.0 m/s²    | **0.5 m/s²**   | 0.08 m/s² | ≈ 6 × |
| `EKF2_GYR_NOISE`  | 0.05 rad/s  | **0.025 rad/s**| 0.002 rad/s | ≈ 12 × |
| `EKF2_ACC_B_NOISE`| 0.02 m/s³   | **0.005 m/s³** | ~5e-5 m/s³ | ≈ 100 × |
| `EKF2_GYR_B_NOISE`| 0.005 rad/s²| **0.001 rad/s²**| ~2e-6 rad/s² | ≈ 500 × |
| `RKT_MPC_SVO_DLY` | 0.100 s     | 0.100 s (لم تُمسّ) | /direct: 0.032 s | قاعدة `max(+40ms, 100ms)` |

**المبدأ:** القفزة الأولى ×2–4 نحو المقاس. بعد طلعة ناجحة يمكن خفضها مرة أخرى إلى ما يقارب المقاس (×2 هامش فقط).

### خطوات التخفيض التالية (post-flight)

| معلمة | بعد الطلعة 1 | بعد الطلعة 2 | هدف نهائي |
|-------|:------------:|:------------:|:---------:|
| `EKF2_ACC_NOISE`  | 0.25 | 0.15 | 0.10 m/s² |
| `EKF2_GYR_NOISE`  | 0.012| 0.006| 0.004 rad/s |
| `EKF2_ACC_B_NOISE`| 0.001| 3e-4 | 1e-4 m/s³ (بعد Allan 2h) |
| `EKF2_GYR_B_NOISE`| 2e-4 | 5e-5 | 1e-5 rad/s² (بعد Allan 2h) |

**مبدأ الحذر:** خطوة واحدة فقط لكلّ طلعة ناجحة — لا قفزات.

---

## 5. ترتيب التنفيذ الموصى به (Runbook)

```
 يوم 1 :  3.4 (GPS)  →  3.5 (Allan 2h)  →  3.3 (Vibration)
 يوم 2 :  3.7 (sim update)  →  3.8 (thermal model)  →  3.9 (vibration model)
 يوم 3 :  3.6 (wiring/preflight/fault)  →  3.10 (/lab)
 يوم 4 :  3.1 (e2e passive+tap+sweep)  →  3.2 (thermal+MPC 30min)
 يوم 5 :  3.11 (Monte Carlo 200 runs)  →  3.12 (HITL كامل)  →  3.13 (system_audit)
 يوم 6 :  🚀  Field test
```

**لا طلعة إلا بعد إتمام جميع البنود P0 + P1 بنتائج PASS.**

---

## 6. معالم يجب تحديثها عند الإنجاز

عند إتمام كلّ بند، حدِّث:
- `docs/PENDING_TESTS.md` (هذا الملف) — انقل البند إلى §2.
- `config/6dof_config_advanced.yaml` — عدِّل القيم + comment بالتاريخ والمصدر.
- `progress.txt` (إن وُجد) — سطر واحد بالتاريخ.
