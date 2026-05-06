# 01 — معمارية النظام

## 🏗️ نظرة عامة

النظام يعمل في ثلاثة أماكن متوازية:

```
┌──────────────────────────────┐      ┌────────────────────────┐      ┌──────────────┐
│    PC (Linux)                │      │   Phone (Android)      │      │   QGC (UDP)  │
│                              │      │                        │      │              │
│  ┌──────────────────────┐    │      │  ┌──────────────────┐  │      │              │
│  │ 6DOF Physics Sim     │    │ TCP  │  │ PX4 v1.17 Stack  │  │      │              │
│  │ (run_hitl_tcp.py)    │◄───┼──────┼─►│ + EKF2           │  │      │              │
│  │  - Dynamics @ 100Hz  │    │ 5760 │  │ + rocket_mpc     │  │      │              │
│  │  - Actuator τ=50ms   │    │      │  │   (acados @50Hz) │  │      │              │
│  │  - Aero coeffs       │    │      │  │ + rocket_mhe     │  │      │              │
│  │  - Thrust            │    │      │  │   (acados @50Hz) │  │      │              │
│  └──────────────────────┘    │      │  └──────────────────┘  │      │              │
│          │                   │      │                        │      │              │
│          ▼                   │      │                        │      │              │
│  ┌──────────────────────┐    │      │                        │      │              │
│  │ QGCMavlinkProxy      │    │ UDP  │                        │      │              │
│  │  @10Hz               │────┼──────┼────────────────────────┼─────►│ telemetry    │
│  │  (synth GPS, ATT)    │    │14550 │                        │      │  maps        │
│  └──────────────────────┘    │      │                        │      │              │
└──────────────────────────────┘      └────────────────────────┘      └──────────────┘
```

## 🔄 تدفّق البيانات

### من PC إلى الهاتف (@100Hz)
```
6DOF state  →  HIL_SENSOR    (IMU: accel, gyro, mag)
6DOF state  →  HIL_GPS       (@10Hz فقط: lat, lon, alt, vel)
6DOF state  →  HIL_STATE_QUATERNION (للمرجعية، ليس فعلياً للـEKF)
```

### من الهاتف إلى PC (@50Hz)
```
MPC output  →  HIL_ACTUATOR_CONTROLS   (fins[4] في [-1,1])
EKF2 state  →  (غير مستخدم — الفيزياء مرجعية)
```

### من PC إلى QGC (@10Hz)
```
ground_truth  →  HEARTBEAT
ground_truth  →  GLOBAL_POSITION_INT  (الموقع الحقيقي، ليس من الهاتف)
ground_truth  →  ATTITUDE
ground_truth  →  SYS_STATUS, BATTERY_STATUS
```

## 📦 المكوّنات الأساسية

### على PC (6DOF_v4_pure/)
| مكوّن | الملف | الوظيفة |
|---|---|---|
| Physics sim | `dynamics/six_dof.py` | تكامل Runge-Kutta 4 للحالة 6DOF |
| Actuator | `dynamics/actuator.py` | نموذج سيرفو 1st-order τ=50ms |
| Aero | `dynamics/aero.py` | قوى وعزوم airflow |
| HITL bridge | `hitl/run_hitl_tcp.py` | TCP server + MAVLink |
| QGC proxy | `hitl/qgc_proxy.py` | UDP بثّ موقع للـQGC |
| Runner | `hitl/run_phone_hitl.sh` | orchestration + cleanup |
| Analyzer | `analysis/compare_runs.py` | تحليل post-run |

### على الهاتف (AndroidApp/)
| مكوّن | المكان | الوظيفة |
|---|---|---|
| PX4 stack | `cpp/PX4-Autopilot/` | نواة PX4 1.17 |
| rocket_mpc | `cpp/PX4-Autopilot/src/modules/rocket_mpc/` | MPC controller |
| rocket_mhe | `cpp/PX4-Autopilot/src/modules/rocket_mhe/` | MHE estimator |
| acados | `jniLibs/.../libacados.so` | OCP solver runtime |
| MainActivity | `kotlin/.../MainActivity.kt` | UI + lifecycle |
| JNI bridge | `cpp/native-lib.cpp` | Kotlin ↔ PX4 |

## 🎯 مسارات التحكّم

### Guidance (داخل rocket_mpc)
```
reference trajectory  →  MPC  →  fin commands
     (from target)
```

### Estimation (داخل rocket_mhe)
```
HIL_SENSOR  →  EKF2  →  state estimate  →  MHE  →  refined state  →  MPC
                                                        (50Hz)
```

### Control (كامل)
```
MHE state (x̂)  →  MPC (SQP-RTI, 5/3/8 iters)  →  u* = [δ1 δ2 δ3 δ4]
                                                     │
                                                     ▼
                                           HIL_ACTUATOR_CONTROLS
                                                     │
                                    [PC] ◄───── TCP ◄┘
                                        │
                                        ▼
                                 actuator dynamics (τ=50ms)
                                        │
                                        ▼
                                  aero forces & moments
                                        │
                                        ▼
                                   6DOF integrator
                                        │
                                        ▼
                                  new state  ──┐
                                               │
                                  HIL_SENSOR ◄─┘
```

## 🛠️ حلقة التحكّم الرئيسية (@50Hz على الهاتف)

```cpp
// في mpc_controller.cpp
while (running) {
    x = mhe_estimator.get_state();           // من EKF+MHE
    target = guidance.get_reference(t);      // مسار مثالي
    
    // §24: حدّد إذا كنّا في نافذة burnout
    bool burnout_critical = (t > burn_time - 0.5) && (t < burn_time + 1.5);
    int n_rti = burnout_critical ? 8 : (warm_start ? 3 : 5);
    
    for (int i = 0; i < n_rti; i++) {
        acados_sqp_rti_preparation();
        acados_sqp_rti_feedback();
    }
    
    int status = acados_get_status();
    bool ok = burnout_critical ? (status == 0) : (status == 0 || status == 2);
    
    if (ok) {
        u = acados_get_u(0);                 // أوّل تحكّم
        apply_to_actuators(u);
    } else {
        // رفض — استخدم آخر قيمة جيّدة
        _consec_fails++;
        if (_consec_fails > 10) reinit_solver();
    }
    
    send_mavlink_actuator_controls(u);
    t += 1.0 / 50.0;
}
```

## 🧩 تفاعل المكوّنات (sequence)

```
  PC                   Phone                   QGC
  ─┬─                  ─┬─                    ─┬─
   │ HIL_SENSOR (10ms)  │                      │
   ├───────────────────►│                      │
   │                    │ EKF2 update          │
   │                    │ MHE refine (20ms)    │
   │                    │ MPC solve (20ms)     │
   │                    │                      │
   │ HIL_ACT_CTRL (20ms)│                      │
   │◄───────────────────┤                      │
   │ physics step       │                      │
   │                    │                      │
   │ GLOBAL_POS (100ms) │                      │
   ├─────────────────────────────────────────►│
   │ ATTITUDE (100ms)   │                      │
   ├─────────────────────────────────────────►│
```

## 🔐 لماذا PC يبعث GPS بدلاً من الهاتف؟

**السبب:** PX4 على الهاتف يتعامل مع HIL_GPS كدخل للـEKF.
النظام المُغلَق:
1. PC يحسب الفيزياء الحقيقية
2. يضيف ضوضاء (بذرة ثابتة)
3. يبعث كـHIL_SENSOR + HIL_GPS
4. EKF يُقدِّر الحالة
5. MPC يقرِّر
6. PC يستقبل التحكّم
7. يطبّق على الفيزياء
8. (عُد إلى 1)

هذا يحاكي الطيران الفعلي حيث الـGPS يأتي من السماء ويدخل PX4.
