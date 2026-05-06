---
description: تشغيل اختبار /watchdog (حقن crash في modules PX4، قياس detection + restart + recovery)
---

مرجع كامل: `6DOF_v4_pure/watchdog/README.md`.

## 1) تأكد APK debug مثبَّت (release يرفض crash injection)

// turbo
```bash
adb shell dumpsys package com.ardophone.px4v17 | grep -E "versionName|debuggable" | head -5
```

إذا `debuggable` غير موجود، أعد البناء والتثبيت:
```bash
cd AndroidApp && ./gradlew installDebug
```

## 2) تأكد PX4 يعمل على الهاتف

// turbo
```bash
adb shell pidof com.ardophone.px4v17 || echo "التطبيق لا يعمل — افتحه واضغط Start PX4"
```

## 3) ثبِّت متطلبات Python (أول مرة فقط)

// turbo
```bash
pip install -r 6DOF_v4_pure/watchdog/requirements.txt
```

## 4) عدِّل config إذا لزم (اختياري)

افتح `6DOF_v4_pure/watchdog/watchdog_config.yaml`:
- `device.log_path`: مسار JSONL على الهاتف — غيِّره إذا getExternalFilesDir مختلف
- `modules`: قائمة الـ modules المُراقَبة (يجب تطابق `watchdog_native.cpp` registry)
- `scenarios.*`: المستهدفات + التكرار + الـ thresholds

## 5) شغّل الاختبار

```bash
# سريع (~2 دقيقة: solo crash لكل module)
python3 6DOF_v4_pure/watchdog/watchdog_runner.py

# قياسي (~5 دقائق: solo + manual restart)
python3 6DOF_v4_pure/watchdog/watchdog_runner.py --preset standard

# شامل (~15 دقيقة: solo + repeated + manual + cascading)
python3 6DOF_v4_pure/watchdog/watchdog_runner.py --preset full

# اختبار module محدَّد
python3 6DOF_v4_pure/watchdog/watchdog_runner.py --module rocket_mpc

# scenario محدَّد
python3 6DOF_v4_pure/watchdog/watchdog_runner.py --scenario cascading
```

## 6) افتح النتائج

// turbo
```bash
ls -lt 6DOF_v4_pure/watchdog/results/ | head -5
```

أحدث مجلد يحوي:
- `watchdog_report.md` — التقرير البشري مع PASS/FAIL وجداول لكل scenario
- `<scenario>_metrics.json` — أرقام detection/restart/recovery لكل iteration
- `<scenario>_events.jsonl` — log الـ watchdog الخام المسحوب من الهاتف
- `watchdog_plot.html` — رسم plotly (إذا plotly مثبَّت)

افتح التقرير:
```bash
cat $(ls -dt 6DOF_v4_pure/watchdog/results/*/ | head -1)watchdog_report.md
```

## 7) (اختياري) إعادة تحليل نتائج سابقة

```bash
python3 6DOF_v4_pure/watchdog/watchdog_runner.py \
    --analyze-only 6DOF_v4_pure/watchdog/results/20260503_230000
```

## 8) قياسات يجب فهمها

- **detection_ms**: من وصول أمر crash للهاتف → وقت تسجيل الـ watchdog لـ `dead` على الـ module. حد أعلى معقول ≈ `stale_threshold + poll_period` (مثلاً 200 + 50 = 250 ms لـ rocket_mpc). إذا أعلى بكثير → الـ watchdog بطيء في الاكتشاف.
- **restart_ms**: من `dead` → إكمال restart. يمثِّل زمن stop + start للـ PX4 module. لمعظم الـ modules ~50-300 ms. إذا > 1 s → يوجد تسرُّب (handles، advertisements، إلخ).
- **recovery_ms**: من `dead` → أول publish جديد (`alive` edge ثاني). يساوي تقريباً `restart_ms + 1/nominal_hz`. 
- **bystander recovery**: في scenario cascading، إذا ekf2 مات مؤقتاً لأن sensors أعيد تشغيله، يجب أن يتعافى ekf2 تلقائياً بدون تدخل. إذا لم يتعافَ → dependency غير مُنمَذَج.

## 9) طرق تصحيح فشل محتمل

| الأعراض | السبب المحتمل | التصرُّف |
|---|---|---|
| `REJECT_RELEASE_BUILD` في كل crash | APK release على الهاتف | `./gradlew installDebug` |
| `no events log on device` | Watchdog لم يبدأ (PX4 لم يكتمل؟) | انتظر 5s بعد Start PX4 |
| detection > 1 s دائماً | poll_period كبير أو CPU مثقَّل | راجع `/thermal` |
| restart_ms يتزايد مع reps | تسرُّب موارد | راجع logcat |
| bystander لا يتعافى | dependency غير معروف | أضف restart للـ bystander يدوياً في scenario |
