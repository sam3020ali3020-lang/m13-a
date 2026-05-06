---
description: تشغيل اختبار /scheduler_jitter (قياس jitter MPC تحت حمل CPU متفاوت)
---

# اختبار Scheduler Jitter

يقيس هذا الاختبار تذبذب (jitter) توقيت control loop في PX4 تحت ثلاث حالات حمل:
- **baseline**: الهاتف ساكن
- **light_load**: تطبيقات خلفية خفيفة (WhatsApp/كاميرا)
- **heavy_load**: 8 عمليات `yes` تستهلك كل CPU cores

الغاية: إثبات (أو نفي) أن `nice=-20` + CPU affinity كافٍ بدل `SCHED_FIFO` (المحظور على Android بدون root).

## الخطوات

### 1. تحقّق من اتصال الهاتف
```bash
adb devices
adb forward tcp:5760 tcp:5760
```
يجب أن يظهر الهاتف كـ `device` (لا `unauthorized`).

### 2. شغّل PX4 على الهاتف
افتح التطبيق واضغط **Start PX4**. انتظر حتى تستقر الرسائل.

### 3. تحقّق من تدفّق MAVLink
```bash
python3 /tmp/probe_v2.py
```
ينبغي أن ترى `Array IDs: {2: N}` حيث N > 100.

### 4. شغّل السيناريوهات

#### السيناريو 1: baseline (30s)
اترك الهاتف ساكن، أغلق كل التطبيقات الخلفية.
// turbo
```bash
cd /home/yoga/m13/m13/6DOF_v4_pure/scheduler_jitter && python3 jitter_runner.py --scenario baseline
```

#### السيناريو 2: light_load (30s)
افتح WhatsApp أو الكاميرا، اتركهم يعملون في الخلفية.
// turbo
```bash
cd /home/yoga/m13/m13/6DOF_v4_pure/scheduler_jitter && python3 jitter_runner.py --scenario light_load
```

#### السيناريو 3: heavy_load (15s) — تلقائي بالكامل
السكريبت نفسه يُنشئ 8 عمليات `yes` عبر adb ثم ينظّفها بعد الاختبار.
// turbo
```bash
cd /home/yoga/m13/m13/6DOF_v4_pure/scheduler_jitter && python3 jitter_runner.py --scenario heavy_load
```

#### الكل مرّة واحدة (مع prompts)
```bash
cd /home/yoga/m13/m13/6DOF_v4_pure/scheduler_jitter && python3 jitter_runner.py --all
```

### 5. اقرأ النتائج
- `results/jitter_<scenario>.json` لكل سيناريو
- `results/comparison_report.md` مقارنة جدولية
- `results/jitter_histograms.png` رسومات overlay

### 6. الحكم
السيناريو ينجح إذا:
- `stddev_ms ≤ 8` (IMU) / `≤ 15` (RktGNC)
- `p99_ms ≤ 2× target`
- `dropped_est = 0`
- `late_3x ≤ 5`

إذا `heavy_load` ينجح → `SCHED_FIFO` غير ضروري. مثبت تجريبياً.

## استكشاف الأخطاء

- **ConnectionRefused**: PX4 لم يضغط Start. كرّر الخطوة 2.
- **unauthorized**: اضغط Allow على شاشة الهاتف واخرتها Always.
- **pymavlink error**: لا حاجة له — السكريبت يستخدم parser يدوي.
- **heavy_load يعلق**: عطّل stress عبر `adb shell pkill yes` يدوياً.
