---
description: تشغيل اختبار /ground (حساسات حقيقية + EKF2 + MPC — بدون طيران)
---

مرجع كامل: `6DOF_v4_pure/ground/README.md`.

## 1) تأكد PX4 يعمل على الهاتف

// turbo
```bash
adb shell pidof com.ardophone.px4v17 || echo "App not running — start it manually"
```

اضغط `Start PX4` على شاشة الهاتف إذا لم تكن المنصة تعمل.

## 2) تحقّق من الـ Airframe (يجب Real Flight)

الـ airframe المطلوب: **`22005`** (Real flight الافتراضي) أو **`22002`** (alias قديم).
لا تستخدم `22003` (SITL) ولا `22004` (HITL) — ستأتي الحساسات وهمية.

تحقّق من QGC → Parameters → `SYS_AUTOSTART`، أو من شاشة التطبيق.

## 3) ضع الهاتف في الوضع الصحيح

- **ثابت** على سطح مستوٍ
- بطارية **≥ 50%** (خصوصاً لـ `extended` 15 دقيقة)
- (اختياري) وصّل **GPS USB** لتقارب EKF2 كامل الـ position flags
- بدون GPS: `pos_horiz_abs`/`pos_vert_abs` قد لا تنشط → عدّل `required_flags` في الـ config إلى `0x0007`

## 4) جسر MAVLink TCP

// turbo
```bash
adb forward tcp:5760 tcp:5760
```

تأكد أن المنفذ يُجيب:
```bash
timeout 1 bash -c '< /dev/tcp/127.0.0.1/5760' && echo "TCP OK" || echo "TCP DOWN"
```

## 5) ثبّت المتطلبات (أول مرة فقط)

// turbo
```bash
pip install numpy pyyaml plotly
```

`plotly` اختياري — بدونه الرسومات HTML لن تُولّد لكن الاختبار يكمل.

## 6) عدّل config حسب الحاجة (اختياري)

افتح `6DOF_v4_pure/ground/ground_config.yaml`:
- `test.duration_s` / `warmup_s`: مدة التسجيل والإحماء
- `thresholds.ekf2.required_flags`: `0x0F` مع GPS، `0x0007` بدونه
- `thresholds.ekf2.max_convergence_time_s`: أقصى زمن تقارب EKF2 (30s افتراضي)
- `thresholds.mpc.max_mpc_solve_p99_ms`: حد MPC timing
- `thresholds.system.max_temperature_C`: حد الحرارة

## 7) شغّل الاختبار

```bash
python3 6DOF_v4_pure/ground/ground_runner.py
```

أو مع تسليح تلقائي (للحصول على MPC timing):

```bash
python3 6DOF_v4_pure/ground/ground_runner.py --arm
```

أو presets:

```bash
# فحص سريع قبل الإطلاق (1 دقيقة) مع تسليح
python3 6DOF_v4_pure/ground/ground_runner.py --preset preflight --arm

# سريع (2 دقيقة)
python3 6DOF_v4_pure/ground/ground_runner.py --preset quick

# قياسي (5 دقائق — الافتراضي)
python3 6DOF_v4_pure/ground/ground_runner.py --preset standard

# ممتد (15 دقيقة — لاختبار thermal throttling)
python3 6DOF_v4_pure/ground/ground_runner.py --preset extended

# مدة مخصّصة
python3 6DOF_v4_pure/ground/ground_runner.py --duration 600

# مع مقارنة PIL
python3 6DOF_v4_pure/ground/ground_runner.py --compare-pil pil/results/pil_flight.csv
```

أثناء التشغيل سترى live monitor كل 5 ثوانٍ:
```
  120s | IMU=6000 EST=240 TIM=4800 | EKF=0x000F MPC=3.2ms CPU=42% T=38°C
```

## 8) افتح النتائج

// turbo
```bash
ls -lt 6DOF_v4_pure/ground/results/ | head -5
```

أحدث مجلد يحوي:
- `GO_NOGO.txt` — الحكم النهائي + قائمة failures
- `summary.html` — تقرير HTML شامل بلون GO/NO-GO
- `ground_metrics.json` — كل الأرقام JSON
- `ekf2.plot.html` — innovations + flags + accuracy (6 subplots)
- `timing.plot.html` — MPC/MHE time series + histograms
- `system.plot.html` — CPU + حرارة + attitude drift
- `pil_comparison.plot.html` — مقارنة مع PIL (إذا مرّرت `--compare-pil`)
- `ground_imu.csv`, `ground_attitude.csv`, `ground_estimator.csv`, `ground_timing.csv`, `ground_sys_status.csv` — بيانات خام

افتح الحكم:
```bash
cat $(ls -dt 6DOF_v4_pure/ground/results/*/ | head -1)GO_NOGO.txt
```

افتح التقرير الشامل في المتصفّح:
```bash
xdg-open $(ls -dt 6DOF_v4_pure/ground/results/*/ | head -1)summary.html
```

## 9) (اختياري) إعادة تحليل نتائج قديمة

```bash
python3 6DOF_v4_pure/ground/ground_analysis.py \
    6DOF_v4_pure/ground/results/20260503_HHMMSS/
```

يُعيد توليد الرسومات و `summary.html` فقط — لا يعيد التسجيل.

## 10) ملاحظات مهمة

- **MPC timing غالباً "No data"** على الأرض لأن MPC لا يتسلّح تلقائياً.
  - الحكم يصبح `passed=None` لقسم timing (لا يُفشل الاختبار حتمياً)
  - للحصول على timing: سلّح MPC يدوياً من QGC، أو استخدم `/lab` مع SITL
- **EKF2 convergence time** يعتمد على GPS. بدونه قد لا تكتمل flags الموقع.
- **الحرارة** لا ترتفع كفاية مع `quick`/`standard` — استخدم `extended` لاختبار thermal.
- **Attitude drift** يقاس من انحراف roll/pitch عن المتوسط (يجب < 2° لو ثابت).

## 11) ترتيب الاختبارات الموصى به

```
/sensor      → هل الحساسات جيدة أصلاً؟ (Allan)
/direct      → هل السيرفوهات تستجيب؟ (CAN)
/ground      ← هنا — تكامل EKF2 + MPC مع حساسات حقيقية
/e2e_latency → transport delay phone → servo
/lab         → SITL + سيرفوهات حقيقية
/thermal     → 30 دقيقة MPC تحت حرارة
/watchdog    → استجابة للأعطال
→ طيران حقيقي
```
