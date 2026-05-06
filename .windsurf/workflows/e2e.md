---
description: تشغيل اختبار /e2e_latency (Phone IMU → Servo) — قياس transport delay كامل
---

مرجع كامل: `6DOF_v4_pure/e2e_latency/README.md`.

## 1) تأكد PX4 يعمل على الهاتف

// turbo
```bash
adb shell pidof com.ardophone.px4v17 || echo "App not running — start it manually"
```

اضغط `Start PX4` على شاشة الهاتف إذا لم تكن المنصة تعمل.

## 2) جسر MAVLink TCP من الهاتف للابتوب

// turbo
```bash
adb forward tcp:5760 tcp:5760
```

تأكد أن المنفذ يُجيب:
```bash
timeout 1 bash -c '< /dev/tcp/127.0.0.1/5760' && echo "TCP OK" || echo "TCP DOWN"
```

## 3) ثبّت المتطلبات (أول مرة فقط)

// turbo
```bash
pip install -r 6DOF_v4_pure/e2e_latency/requirements.txt
```

## 4) عدّل config حسب الحاجة (اختياري)

افتح `6DOF_v4_pure/e2e_latency/e2e_config.yaml`:
- `mavlink_streams.*.rate_hz`: زِد إذا كنت تحتاج دقة أعلى (انتبه لـ 40KB/s budget)
- `tests.passive.duration_s`: مدة التسجيل
- `thresholds.l_total.p99_ms_max`: حد الـ PASS/FAIL

## 5) شغّل الاختبار

```bash
python3 6DOF_v4_pure/e2e_latency/e2e_runner.py
```

أو خيارات مخصَّصة:

```bash
# سريع (60s passive فقط)
python3 6DOF_v4_pure/e2e_latency/e2e_runner.py --preset quick

# قياسي (5 دقائق passive + tap test)
python3 6DOF_v4_pure/e2e_latency/e2e_runner.py --preset standard

# شامل (15 دقيقة كل الاختبارات)
python3 6DOF_v4_pure/e2e_latency/e2e_runner.py --preset full

# اختبار محدّد بمدة مخصَّصة
python3 6DOF_v4_pure/e2e_latency/e2e_runner.py --test passive --duration 120
```

## 6) افتح النتائج

// turbo
```bash
ls -lt 6DOF_v4_pure/e2e_latency/results/ | head -5
```

أحدث مجلد يحوي:
- `latency_report.txt` — تقرير نصي مع PASS/FAIL
- `latency.metrics.json` — أرقام بصيغة JSON
- `latency_plot.html` — رسم Plotly (إن كان plotly مثبَّت)
- `imu.csv`, `attitude.csv`, `gnc.csv`, `servo_fb.csv`, `servo_cmd.csv` — البيانات الخام

افتح التقرير:
```bash
cat $(ls -dt 6DOF_v4_pure/e2e_latency/results/*/ | head -1)latency_report.txt
```

## 7) (اختياري) إعادة تحليل CSV قديم

```bash
python3 6DOF_v4_pure/e2e_latency/e2e_analysis.py \
    6DOF_v4_pure/e2e_latency/results/20260503_HHMMSS/ \
    --plot
```

## 8) ملاحظات

- **L_sensor** يُقاس دائماً من passive recording
- **L_mpc** يحتاج RktGNC stream (دائماً موجود في PX4 المُحدَّث)
- **L_actuator** يحتاج MPC ينشط ويأمر بحركة الفينات. على الطاولة في pre-launch:
  - شغّل HITL مع المحاكي لـ L_actuator حقيقي، أو
  - استخدم `/direct` لقياس actuator latency منفصلاً ثم اجمعها يدوياً

## 9) قاعدة tuning من النتيجة

| `L_total p99` | `RKT_MPC_SVO_DLY` |
|---------------|-------------------|
| < 80 ms | 0.100f (current floor) ✅ |
| 80–120 ms | 0.150f |
| 120–180 ms | 0.200f |
| > 200 ms | راجع pipeline (thermal? MPC tuning?) |
