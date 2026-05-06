# /ground — اختبار أرضي تكاملي (Ground Integration Test)

حساسات حقيقية + EKF2 حقيقي + MPC حقيقي — بدون طيران.

## لماذا هذا الاختبار؟

```
/sensor  → يختبر الحساسات فقط (بدون EKF2, بدون MPC)
/pil     → يختبر المعالج فقط (حساسات وهمية من الكمبيوتر)
/ground  → يختبر الحساسات + المعالج معاً (الثغرة المفقودة)
```

هذا الاختبار يكشف مشاكل **لا يمكن لأي اختبار آخر كشفها**:

| المشكلة | /sensor | /pil | /ground |
|---------|---------|------|---------|
| EKF2 لا يتقارب مع ضجيج الحساس الحقيقي | ❌ | ❌ | **✅** |
| MPC يتأخر بسبب حمل الحساسات | ❌ | ❌ | **✅** |
| الهاتف يحترق من sensor + MPC معاً | ❌ | ❌ | **✅** |
| Attitude drift مع bias حقيقي | ❌ | ❌ | **✅** |
| crash/freeze عند دمج الأنظمة | ❌ | ❌ | **✅** |

## تشغيل سريع

```bash
# 1) الهاتف يعمل بـ Airframe = 22002 (Real Flight)
adb forward tcp:5760 tcp:5760

# 2) اختبار قياسي (5 دقائق)
python3 ground_runner.py

# 3) فحص قبل الإطلاق (1 دقيقة)
python3 ground_runner.py --preset preflight

# 4) اختبار ممتد (15 دقيقة — حرارة)
python3 ground_runner.py --preset extended

# 5) مقارنة مع PIL
python3 ground_runner.py --compare-pil ../pil/results/pil_timing.csv

# 6) تحليل نتائج موجودة
python3 ground_analysis.py results/<timestamp>/
```

## ماذا يقيس؟

### 1. تقارب EKF2
- **الوقت**: كم ثانية حتى يتقارب EKF2 مع حساسات حقيقية؟
- **Innovation ratios**: هل vel/pos/mag أقل من 1.0 (صحي)؟
- **Accuracy**: دقة الموقع الأفقي/العمودي
- **Flags**: هل كل المكونات نشطة (attitude+velocity+position)؟

### 2. توقيت MPC/MHE
- **MPC solve time**: mean, p50, p95, p99, max
- **MHE solve time**: نفس المقاييس
- **Cycle time**: الدورة الكاملة
- **مقارنة مع PIL**: هل الحساسات الحقيقية تبطئ MPC؟

### 3. صحة النظام
- **CPU load** (%): من SYS_STATUS
- **حرارة**: من HIGHRES_IMU (sensor temperature)
- **ارتفاع الحرارة**: °C rise خلال الاختبار

### 4. استقرار الاتجاه
- **Roll/Pitch drift**: يجب ≈ 0 (الهاتف ثابت)
- **Yaw drift**: يُسمح ببعض الانجراف بدون compass جيد

## عتبات GO / NO-GO

| المعيار | PASS | FAIL |
|---------|------|------|
| EKF2 convergence | < 30 ثانية | > 30 ثانية أو لم يتقارب |
| Innovation ratios | mean < 1.0 | mean > 1.0 |
| Attitude drift | < 2° | > 2° |
| MPC solve p99 | < 15 ms | > 15 ms |
| MPC solve mean | < 8 ms | > 8 ms |
| Cycle p99 | < 25 ms | > 25 ms |
| CPU load | < 80% | > 80% |
| Temperature | < 55°C | > 55°C |
| Temp rise | < 15°C | > 15°C |
| vs PIL increase | < 30% | > 30% |

## المجموعات المسبقة

| Preset | المدة | الهدف |
|--------|-------|-------|
| `quick` | 2 min | تحقق سريع |
| `standard` | 5 min | الافتراضي |
| `extended` | 15 min | اختبار حرارة ممتد |
| `preflight` | 1 min | فحص قبل الإطلاق |

## المخرجات

```
results/<timestamp>/
├── ground_imu.csv              ← بيانات حساسات خام
├── ground_attitude.csv         ← EKF2 roll/pitch/yaw
├── ground_estimator.csv        ← EKF2 innovations + flags + accuracy
├── ground_timing.csv           ← MPC/MHE/cycle timing (µs)
├── ground_sys_status.csv       ← CPU load + battery
├── ground_gps.csv              ← GPS (إذا متصل)
├── ground_baro.csv             ← barometer
├── ground_metrics.json         ← كل النتائج مجمّعة
├── GO_NOGO.txt                 ← الحكم
├── ekf2.plot.html              ← رسم EKF2 تفاعلي
├── timing.plot.html            ← رسم توقيت MPC
├── system.plot.html            ← رسم CPU + حرارة
├── pil_comparison.plot.html    ← مقارنة مع PIL (اختياري)
└── summary.html                ← تقرير HTML شامل
```

## مسار البيانات

```
حساسات الهاتف (400 Hz)
  ↓ native_sensor_reader.cpp → phone_to_frd() → SharedSensorData
  ↓ android_uorb_publishers.cpp → uORB
  ↓
  ├→ EKF2 → ESTIMATOR_STATUS (msg 230) → /ground يقرأ
  ├→ rocket_mpc → MHE + MPC → DEBUG_FLOAT_ARRAY "RktGNC" (msg 350) → /ground يقرأ
  ├→ mavlink → ATTITUDE (msg 30) → /ground يقرأ
  ├→ mavlink → HIGHRES_IMU (msg 105) → /ground يقرأ
  └→ mavlink → SYS_STATUS (msg 1) → /ground يقرأ
```

## ملاحظة مهمة: MPC يحتاج arm

⚠️ **RktGNC timing لا يُبثّ إلا بعد تسلّح MPC (armed).**

في وضع Real Flight (22002) على الأرض، MPC لا يتسلّح تلقائياً.
خيارات:
1. **تسليح تلقائي** (مُوصى به): أضف `--arm` إلى الأمر
   ```bash
   python3 ground_runner.py --arm
   ```
   هذا يُرسل MAV_CMD_COMPONENT_ARM_DISARM (force-arm) كل 2 ثانية حتى يتسلّح PX4.
   يتتبّع COMMAND_ACK و HEARTBEAT base_mode لتأكيد التسلّح.

2. **تسليح يدوي**: استخدم QGC لتسلّح يدوي (إذا EKF2 متقارب)

3. أو اقبل أن قياس timing لن يكون متاحاً — الاختبار لا يزال مفيداً لـ EKF2 + system

يمكن أيضاً تفعيل التسليح التلقائي من الـ config:
```yaml
test:
  auto_arm: true
```

إذا لم يُستلم timing، الاختبار يُبلغ "No timing data" لكن لا يعتبره FAIL حتمياً.

## التسلسل الموصى به

```
1. /sensor  → هل الحساسات جيدة أصلاً؟
2. /direct  → هل السيرفوهات تستجيب؟
3. /ground  → هل EKF2 + MPC يعملان مع حساسات حقيقية؟ ← أنت هنا
4. /hil     → محاكاة طيران كاملة (بضجيج مُعاير من /sensor)
5. طيران    → الحقيقة
```

## الملفات

| الملف | الوظيفة |
|-------|---------|
| `ground_config.yaml` | إعدادات الاختبار + عتبات + presets |
| `ground_reader.py` | MAVLink reader (يرث من sensor_reader + يضيف EKF2/timing/CPU) |
| `ground_runner.py` | مشغّل الاختبار مع مراقبة حية وتحليل |
| `ground_analysis.py` | رسوم بيانية تفاعلية + تقرير HTML |
