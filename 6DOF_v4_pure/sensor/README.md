# /sensor — اختبار حساسات الهاتف (Phone Sensor Test Suite)

يختبر حساسات الهاتف (IMU + Baro + Mag + GPS) التي تُغذّي EKF2 و MPC.

## لماذا؟

```
الحساسات → EKF2 → MHE → MPC → السيرفوهات → الطيران
```

إذا الحساسات خاطئة، كل شيء بعدها خاطئ. هذا الاختبار يكشف مشاكل
الضجيج والانحياز والمعدلات **قبل** أن تظهر كأعراض غامضة في HIL أو الطيران.

## المتطلبات

- هاتف Android مع التطبيق (Airframe = 22002 Real Flight)
- كابل USB + `adb` على الكمبيوتر
- GPS USB (u-blox) — لاختبار GPS فقط
- Python 3.10+ مع: `pip install -r requirements.txt`

## تشغيل سريع

```bash
# 1) وصّل الهاتف وافتح التطبيق
adb forward tcp:5760 tcp:5760

# 2) اختبار سريع (3 دقائق — static + rates + dynamic_range)
python3 sensor_runner.py

# 3) اختبار قياسي (15 دقيقة — + frame + gps)
python3 sensor_runner.py --preset standard

# 4) اختبار كامل (45 دقيقة — كل الاختبارات)
python3 sensor_runner.py --preset full

# 5) Allan variance طويل (ساعة)
python3 sensor_runner.py --test allan --duration 3600

# 6) تحليل نتائج موجودة
python3 sensor_analysis.py results/<timestamp>/
```

## الاختبارات

| # | الاسم | المدة | الوصف |
|---|-------|-------|-------|
| 1 | `static` | 5 min | أرضية الضجيج والانحياز (الهاتف ثابت) |
| 2 | `allan` | 30-120 min | Allan Variance — المعيار الذهبي لتوصيف IMU |
| 3 | `rates` | 1 min | التحقق من معدلات العينات (200 Hz IMU, 25 Hz baro) |
| 4 | `frame` | يدوي | التحقق من `phone_to_frd()` — خطأ هنا = كارثة |
| 5 | `temperature` | 10 min | الانجراف الحراري (الهاتف يسخن من MPC) |
| 6 | `gps` | 5 min | أداء GPS (HDOP, CEP, سرعة, أقمار) |
| 7 | `vibration` | 1 min | تحليل طيفي للاهتزاز (FFT + clipping) |
| 8 | `dynamic_range` | 30 s | التحقق من نطاق الحساسات (±16g, ±2000°/s) |

## المجموعات المسبقة (Presets)

| Preset | المدة | الاختبارات |
|--------|-------|-----------|
| `quick` | ~3 min | static(60s), rates, dynamic_range |
| `standard` | ~15 min | static, rates, frame, gps, dynamic_range |
| `full` | ~45 min | الكل |
| `allan_long` | ~2 hr | allan(7200s) فقط |

## عتبات GO / NO-GO

| المعيار | PASS | FAIL |
|---------|------|------|
| IMU rate | ≥ 200 Hz | < 150 Hz |
| Gyro noise (std) | < 0.01 rad/s | > 0.02 rad/s |
| Accel noise (std) | < 0.5 m/s² | > 1.0 m/s² |
| Gyro bias instability | < 10 °/hr | > 50 °/hr |
| GPS HDOP | < 2.0 | > 4.0 |
| GPS fix | 3D | 2D or none |
| Frame check | gravity correct axis | wrong axis |
| Vibration clipping | 0% | any |

## المخرجات

```
results/<timestamp>/
├── static/
│   ├── sensor_imu.csv           ← بيانات خام
│   ├── static.metrics.json      ← قيم رقمية
│   └── static.plot.html         ← رسم تفاعلي
├── allan/
│   ├── sensor_imu.csv
│   ├── allan.metrics.json
│   └── allan.plot.html          ← منحنيات Allan deviation
├── rates/
│   └── rates.metrics.json
├── gps/
│   ├── sensor_gps.csv
│   ├── gps.metrics.json
│   └── gps.plot.html
├── vibration/
│   ├── sensor_imu.csv
│   └── vibration.plot.html      ← FFT / PSD
├── temperature/
│   └── temperature.plot.html
├── all_metrics.json             ← كل النتائج مجمّعة
├── config_suggestions.json      ← اقتراحات لتحديث 6dof_config
├── GO_NOGO.txt                  ← الحكم النهائي
└── summary.html                 ← تقرير HTML شامل
```

## الربط بباقي النظام

النتائج تُحدّث مباشرة معاملات `6dof_config_advanced.yaml`:

```yaml
# من static test:
estimation.sensors.accel_noise_std  →  accel RMS noise المقاس
estimation.sensors.gyro_noise_std   →  gyro RMS noise المقاس
bridge.noise.accel_std              →  نفس القيمة (لمحاكاة SITL واقعية)
bridge.noise.gyro_std               →  نفس القيمة

# من allan test:
estimation.sensors.gyro_bias_std    →  3 × bias instability المقاس
estimation.sensors.accel_bias_std   →  3 × bias instability المقاس

# من static test (bias):
error_injection.sig_gyro_bias_*     →  2 × mean gyro bias المقاس
error_injection.sig_accel_bias_*    →  2 × mean accel bias المقاس
```

## مسار البيانات

```
Android NDK ASensorManager (SENSOR_DELAY_FASTEST)
  ↓ UNCALIBRATED accel/gyro/mag + baro
  ↓ native_sensor_reader.cpp → phone_to_frd() → SharedSensorData
  ↓ android_uorb_publishers.cpp → sensor_accel/gyro/baro/mag → uORB
  ↓ PX4 mavlink module → MAVLink streams
  ↓ mavlink_tcp_bridge.cpp (TCP:5760)
  ↓ adb forward tcp:5760 tcp:5760
  ↓
PC: sensor_reader.py → HIGHRES_IMU + GPS_RAW_INT + SCALED_PRESSURE
```

## دقة القياسات وحدودها

### الحقيقة: downsampling عبر MAVLink

الحساس الأصلي يعمل بـ **~400 Hz** (gyro) / **~200 Hz** (accel) عبر `SENSOR_DELAY_FASTEST`.
لكن PX4 mavlink يبثّ `HIGHRES_IMU` بحد أقصى **~100 Hz** (محدود بـ budget = 40 KB/s
الذي ضُبط في `px4_jni.cpp`: `-r 40000`).

كل عينة MAVLink = **آخر قراءة حقيقية** من الحساس (وليست متوسط)، لكننا نفقد
~75% من العينات الأصلية.

### تأثير ذلك على كل اختبار:

| الاختبار | الدقة | التفسير |
|---------|-------|---------|
| `static` (ضجيج + bias) | ✅ **موثوق** | الضجيج white noise — الـ std لا يتأثر بـ downsampling. والـ bias = mean طويل = دقيق |
| `allan` (bias instability) | ✅ **موثوق** | المنطقة المهمة (BI) عند τ=1-100s لا تحتاج معدل عالٍ. 100 Hz كافٍ |
| `rates` | ⚠️ **محدود** | يقيس معدل MAVLink (100 Hz) وليس المعدل الأصلي (400 Hz). المعدل الأصلي يُسجّل في logcat: `adb logcat -s SensorReader:I` |
| `frame` | ✅ **موثوق** | اتجاه الجاذبية لا يحتاج معدل عالٍ |
| `temperature` | ✅ **موثوق** | ظاهرة بطيئة (دقائق) — 100 Hz أكثر من كافٍ |
| `gps` | ✅ **موثوق** | 5 Hz = المعدل الحقيقي لـ u-blox، لا downsampling |
| `vibration` | ❌ **غير موثوق** | Nyquist = 50 Hz فقط. اهتزاز المحرك الصاروخي (100-500 Hz) **غير مرئي** |
| `dynamic_range` | ⚠️ **تقريبي** | نستنتج من البيانات، لا نقرأ specs مباشرة |

### المشكلة الحرجة: اختبار الاهتزاز

**هذا أخطر اختبار** (اهتزاز المحرك يمكن أن يُشبّع الـ IMU ويُعمي EKF2)
وهو **الأضعف دقة** في التصميم الحالي:

- اهتزاز محرك صاروخي: **100-500 Hz**
- أقصى تردد يمكن كشفه عبر MAVLink بـ 100 Hz: **50 Hz**
- **لا يمكن كشف اهتزاز المحرك عبر MAVLink**

### الحل المطلوب: تسجيل مباشر على الهاتف

يجب إضافة وضع تسجيل محلي في `native_sensor_reader.cpp` يكتب CSV
مباشرة على تخزين الهاتف بمعدل الحساس الكامل (~400 Hz):

```
native_sensor_reader.cpp (400 Hz raw)
  ↓ يكتب مباشرة → /sdcard/sensor_log.csv  (بدون MAVLink)
  ↓ بعد الاختبار: adb pull /sdcard/sensor_log.csv
  ↓ sensor_analysis.py → FFT كامل حتى 200 Hz Nyquist
```

**التنفيذ**: إضافة flag (مثل JNI call أو system property) يُفعّل التسجيل
المحلي في `native_sensor_reader.cpp`. عند التفعيل:
1. فتح ملف CSV في `/sdcard/m130_sensor/`
2. كتابة كل `ASensorEvent` مباشرة (timestamp_ns, ax, ay, az, gx, gy, gz)
3. إيقاف التسجيل بعد المدة المطلوبة
4. سحب الملف: `adb pull /sdcard/m130_sensor/vibration.csv`
5. تحليل: `python3 sensor_analysis.py --vibration vibration.csv`

**حتى يُنفَّذ هذا الحل**، اختبار الاهتزاز الحالي (عبر MAVLink) صالح فقط
لكشف الاهتزاز المنخفض التردد (< 50 Hz) والتشبّع (clipping).

## مقارنة مع الاختبارات الأخرى

| الاختبار | الحساسات | السيرفوهات | الخوارزميات | الهاتف |
|----------|----------|-----------|-------------|--------|
| `/sensor` | **✅ المحور الرئيسي** | ❌ | ❌ | ✅ حقيقي |
| `/direct` | ❌ | ✅ baseline | ❌ | ❌ |
| `/lab` | ❌ | ✅ حقيقي | ✅ MPC | ❌ (SITL) |
| `/hil` | ❌ محاكاة | ✅ حقيقي | ✅ MPC+EKF | ✅ حقيقي |
| `/sitl` | ❌ محاكاة | ❌ محاكاة | ✅ MPC+EKF | ❌ |
| `/pil` | ❌ محاكاة | ❌ | ✅ MPC | ✅ حقيقي |

## الملفات

| الملف | الوظيفة |
|-------|---------|
| `sensor_config.yaml` | إعدادات الاختبارات، عتبات PASS/FAIL، MAVLink streams |
| `sensor_reader.py` | MAVLink v2 parser — يتصل بـ TCP:5760 ويقرأ بيانات الحساسات |
| `sensor_runner.py` | مشغّل الاختبارات الرئيسي — يُنفّذ الاختبارات ويُنتج CSV + metrics |
| `sensor_analysis.py` | تحليل Allan variance + FFT + plots + اقتراحات config |
