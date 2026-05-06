# /e2e_latency — End-to-End Transport Delay Measurement

يقيس الـ **transport delay** الكامل من قراءة IMU الفيزيائية على الهاتف حتى تحرّك السيرفو الحقيقي.

## لماذا؟

اختبارات أخرى تغطي أجزاء فقط:

| اختبار | ما يقيسه | الجزء |
|--------|----------|-------|
| `/sensor` | Phone IMU → MAVLink | 1/3 الطريق |
| `/direct` | PC ↔ CAN ↔ Servo | جزء السيرفو فقط |
| `/lab` | EKF SITL → CAN → Servo | لا phone IMU حقيقي |

**الفجوة**: لا اختبار يربط Phone IMU الحقيقي بالسيرفو الحقيقي عبر pipeline كامل.

## ما الذي يُقاس

```
Phone IMU → NDK → uORB → EKF2 → MPC → CAN → Servo
   │           │       │       │       │       │
   └───────────┴───────┴───────┴───────┴───────┘
                  Total transport delay
```

تُقاس على مراحل من خلال timestamps موجودة في PX4 HRT:

| المرحلة | المقياس | المصدر MAVLink |
|---------|---------|----------------|
| `L_sensor` | IMU sample → vehicle_attitude ready | `HIGHRES_IMU.time_usec` ↔ `ATTITUDE.time_boot_ms` |
| `L_mpc` | EKF input → MPC fin command | `RktGNC.data[47]` = `mpc_solve_us` (μs) |
| `L_actuator` | Fin command → physical servo position | `SERVO_OUTPUT_RAW.time_usec` ↔ `SRV_FB.data[4..7]` |
| `L_total` | جمع المراحل | محسوب |

كل الأرقام في **PX4 HRT clock** الموحَّد — لا يحتاج clock alignment.

## المتطلبات

- هاتف يشغّل PX4 (m13/m13 AndroidApp)
- اتصال MAVLink TCP عبر `adb forward tcp:5760 tcp:5760` (أو wifi مباشر)
- `airframe = 22005` (Real flight) أو `22004` (HITL)
- (اختياري) USB-CAN + سيرفوهات XQPOWER لقياس `L_actuator` الحقيقي
- (اختياري) محاكي HITL يعمل لتفعيل MPC وتحريك الفينات

## الاستخدام

### تشغيل سريع

```bash
python3 6DOF_v4_pure/e2e_latency/e2e_runner.py
```

افتراضياً يشغّل preset `quick` (60 ثانية passive).

### Presets

```bash
python3 e2e_runner.py --preset quick      # 60s — sensor pipeline فقط
python3 e2e_runner.py --preset standard   # 5 min — sensor + actuator
python3 e2e_runner.py --preset full       # 15 min — يشمل tap test
```

### Sub-tests

```bash
python3 e2e_runner.py --test passive       # 60s قياس سلبي لكل المراحل
python3 e2e_runner.py --test tap           # tap-test بمحفّز يدوي (اختياري)
python3 e2e_runner.py --test sweep         # sweep إجباري عبر QGC mavlink command
```

### تحليل CSV قديم

```bash
python3 6DOF_v4_pure/e2e_latency/e2e_analysis.py results/20260503_HHMMSS/
```

## النتائج

في `results/<timestamp>/`:

| ملف | المحتوى |
|-----|---------|
| `imu.csv` | عينات HIGHRES_IMU |
| `attitude.csv` | عينات ATTITUDE |
| `servo_cmd.csv` | SERVO_OUTPUT_RAW (fin commands) |
| `servo_fb.csv` | SRV_FB من xqpower_can (feedback) |
| `gnc.csv` | RktGNC (timing diagnostics + state) |
| `latency.metrics.json` | إحصاءات الـ latency لكل مرحلة |
| `latency_report.txt` | تقرير نصي |
| `latency_plot.html` | رسم Plotly تفاعلي (اختياري) |

## أمثلة قيم متوقَّعة

```
L_sensor:   p50=15ms   p99=35ms     # IMU → ATTITUDE
L_mpc:      p50=4ms    p99=12ms     # MPC solve time
L_actuator: p50=40ms   p99=80ms     # CMD → fb match (depends on servo)
L_total:    p50=60ms   p99=120ms

Threshold pass:
  - L_total p50 < 100ms ✅ acceptable
  - L_total p99 < 200ms ✅ MPC tuning consistent مع RKT_MPC_SVO_DLY
```

## كيف تُحسَب الـ latency

### L_sensor

```python
for att_msg in attitude_stream:
    # Find latest IMU sample whose time_usec <= att_msg time
    t_att = att_msg.time_boot_ms * 1000  # → μs
    imu_at = max(imu.time_usec for imu in imu_stream if imu.time_usec <= t_att)
    latencies.append(t_att - imu_at)
```

### L_mpc

```python
# Direct from RktGNC
latencies = [gnc.data[47] for gnc in rktgnc_stream]  # mpc_solve_us
```

### L_actuator

```python
# For each command change in servo_cmd, find when fb stabilizes near it
for cmd in servo_cmd_stream:
    # Find next SRV_FB where |fb - cmd| < tolerance
    fb_arrived = next(fb for fb in srv_fb_stream
                      if fb.time_usec > cmd.time_usec
                      and abs(fb.fb_deg[0] - cmd.servo1_raw) < 0.5)
    latencies.append(fb_arrived.time_usec - cmd.time_usec)
```

## ملاحظات

1. **L_actuator على الطاولة قد يكون 0** إذا MPC لا يأمر بحركة (pre-launch idle).
   - استخدم HITL مع المحاكي لتفعيل MPC.
   - أو استخدم اختبار `/direct` للحصول على L_actuator_pure (servo only) ثم اجمع.

2. **L_sensor دائماً قابل للقياس** حتى لو لم يأمر MPC بشيء.

3. **الـ rates الموصى بها**:
   - HIGHRES_IMU: 100 Hz (max stable مع 40KB/s budget)
   - ATTITUDE: 50 Hz
   - DEBUG_FLOAT_ARRAY: 50 Hz (يعطي SRV_FB كل 20ms)
   - SERVO_OUTPUT_RAW: 50 Hz

4. **Clock**: كل الـ timestamps في PX4 HRT — لا حاجة لـ wall-clock alignment.

5. **الـ tap test** يحتاج إلى:
   - MPC نشط (post-launch أو HITL)
   - حركة سريعة محسوسة على الهاتف (tap قوي)

## مرجع: استخدام النتائج لـ tuning

| `L_total p99` | تأثير على `RKT_MPC_SVO_DLY` |
|---------------|----------------------------|
| < 80 ms | الإعداد الحالي 100ms كافٍ |
| 80-120 ms | اضبط `RKT_MPC_SVO_DLY = 0.150f` |
| 120-180 ms | اضبط `RKT_MPC_SVO_DLY = 0.200f` |
| > 200 ms | راجع pipeline — قد يكون thermal throttling |

راجع `~/SYSTEM-RETRIEVED-MEMORY` عن قاعدة `RKT_MPC_SVO_DLY = max(measured + 40ms, 100ms)`.
