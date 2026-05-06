# 02 — التوقيت والتزامن

> **المبدأ:** كل مكوّن يعمل بتردّد معلوم ومتوافق مع البقية. أي عدم تطابق → اهتزاز أو تباعد.

## جدول التوقيت الكامل

| المكوّن | التردّد | الفترة | المكان |
|---|---|---|---|
| فيزياء 6DOF (PC) | 100 Hz | 10 ms | `config/6dof_config_advanced.yaml: simulation.dt` |
| HIL_SENSOR (PC→Phone) | 100 Hz | 10 ms | يطابق الفيزياء — `run_hitl_serial._send_hil` |
| HIL_GPS (PC→Phone) | 10 Hz | 100 ms | `run_hitl_serial.py` |
| PX4 EKF2 (Phone) | 100 Hz | 10 ms | يتبع HIL_SENSOR |
| acados MHE (Phone) | 50 Hz | 20 ms | داخل `rocket_mpc` |
| acados MPC (Phone) | 50 Hz | 20 ms | `autopilot.mpc.dt_solve: 0.02` |
| MPC horizon | — | 4.0 s | `tf: 4.0`, `N_horizon: 80` → dt_step=50ms |
| HIL_ACTUATOR_CONTROLS (Phone→PC) | ~50 Hz | 20 ms | يتبع MPC |
| QGC telemetry (PC→UDP) | 10 Hz | 100 ms | `QGCMavlinkProxy._qgc_tx_loop` |
| HEARTBEAT | 1 Hz | 1000 ms | تلقائي |
| طول تشغيل كامل | — | ~17 s | من الإطلاق حتى الاصطدام |

## العلاقات الرياضية (يجب أن تُحقَّق)

```
1) HIL_SENSOR_Hz × sim_dt = 1.0       ✓ 100 × 0.010 = 1.0
2) dt_solve ≥ sim_dt                   ✓ 20 ≥ 10 ms
3) dt_solve / sim_dt = integer         ✓ 20/10 = 2
4) tf = N_horizon × dt_step            ✓ 4.0 = 80 × 0.050
5) tf ≥ 80% × burn_time                ✓ 4.0 ≥ 3.84 (burn=4.8s)
6) tau_servo ≤ dt_solve                ✓ 15ms ≤ 20ms (MPC model)
```

التحقّق التلقائي:
```bash
python3 tools/verify_timing.py -v
```

## ثابت زمن السيرفو (τ)

| المكان | τ | السبب |
|---|---|---|
| نموذج الفيزياء (`dynamics/actuator.py`) | **50 ms** | واقع السيرفو الفعلي |
| نموذج MPC (`mpc/m130_mpc_autopilot.py`) | **15 ms** | أسرع من الواقع لمنح SQP هامش استقرار |

**تحذير مهم:** إذا غيّرت τ في أحد الملفّين، يجب:
1. تحديث الآخر أو عدم تحديثه (قرار مُتعَمَّد)
2. إعادة توليد OCP: `python3 mpc/m130_ocp_setup.py`
3. إعادة بناء APK: `cd ../AndroidApp && ./gradlew assembleDebug`
4. نسخ `c_generated_code/` الجديد إلى AndroidApp

## كيف تتأكّد من التزامن عند التشغيل

خلال أي تشغيل HITL، راقب هذه الرسائل:

```
[INFO]  t=0.99s  alt=3.3m   range=30m    fins=[...]  rx=218
[INFO]  t=1.99s  alt=14.0m  range=125m   fins=[...]  rx=286
```

**تحليل:**
- `rx` يزيد بحوالي **50 rx/s** → MPC يعمل @ 50Hz ✓
- إذا `rx` زيادة أقل من 30/s → MPC مُعلَّق (تنبيه خطير)
- إذا `t` يتقدّم أبطأ من الوقت الحقيقي → PC لا يواكب الفيزياء

## مصادر التباين المتبقّية (لا يمكن إلغاؤها 100%)

| المصدر | تأثير | مُعالَج؟ |
|---|---|---|
| ضوضاء المستشعرات | ±1-2 م في الذروة | ✅ `HITL_NOISE_SEED=20260424` |
| الانحيازات | ±1-2 م | ✅ نفس البذرة |
| إعدادات الفيزياء | — | ✅ `test_vectors.json` ثابت |
| jitter أندرويد (scheduler) | ±2-3 م | ❌ يحتاج RTOS |
| TCP buffering (Nagle) | <1 م | ❌ عيب بروتوكول |

**الخلاصة:** التباين المُلاحَظ 1% بين التشغيلات هو السقف الفيزيائي.
