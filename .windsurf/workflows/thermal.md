---
description: تشغيل اختبار /thermal_stress (30 دقيقة MPC تحت حرارة) — قياس deadline misses + throttling
---

مرجع كامل: `6DOF_v4_pure/thermal_stress/README.md`.

## 1) تأكد PX4 يعمل على الهاتف (لاختبار MPC حقيقي)

// turbo
```bash
adb shell pidof com.ardophone.px4v17 || echo "App not running — start it manually"
```

اضغط `Start PX4` على شاشة الهاتف إذا لم تكن المنصة تعمل. لاختبار حرارة
فقط (بدون MPC) يمكن تخطّي هذه الخطوة.

## 2) جسر MAVLink + tunnel للـ HIL/PIL

// turbo
```bash
adb forward tcp:5760 tcp:5760
adb reverse tcp:4560 tcp:4560
```

تأكد أن المنفذ يُجيب:
```bash
timeout 1 bash -c '< /dev/tcp/127.0.0.1/5760' && echo "TCP OK" || echo "TCP DOWN"
```

## 3) ثبّت المتطلبات (أول مرة فقط)

// turbo
```bash
pip install -r 6DOF_v4_pure/thermal_stress/requirements.txt
```

## 4) عدّل العتبات (اختياري)

افتح `6DOF_v4_pure/thermal_stress/thermal_stress_config.yaml`:
- `mpc.deadline_us`: 40000 (= 25 Hz)
- `thresholds.mpc_solve_us.p99_max`: العتبة الفاشلة لـ p99
- `thresholds.deadline_miss_rate.fail`: 0.20 = 20%
- `presets.*`: 4 presets جاهزة (quick/standard/desert/extreme)

## 5) شغّل الاختبار

اختر سيناريو حسب الهدف:

```bash
# (أ) Smoke test سريع (5 دقائق passive — تأكد الـ pipeline يعمل)
python3 6DOF_v4_pure/thermal_stress/thermal_stress_runner.py --preset quick

# (ب) قياسي (30 دقيقة، أنت شغّل /hil في نافذة أخرى)
python3 6DOF_v4_pure/thermal_stress/thermal_stress_runner.py --preset standard

# (ج) محاكاة صحراء (30 دقيقة + preheat 65°C + cpu-stress)
python3 6DOF_v4_pure/thermal_stress/thermal_stress_runner.py --preset desert

# (د) متطرّف (60 دقيقة + preheat 72°C + stress مستمر)
python3 6DOF_v4_pure/thermal_stress/thermal_stress_runner.py --preset extreme

# (هـ) PIL تلقائي (لا يحتاج /hil يدوي — يُكرّر pil_runner طوال المدة)
python3 6DOF_v4_pure/thermal_stress/thermal_stress_runner.py \
    --duration 1800 --mode pil-loop

# (و) خيارات مخصَّصة
python3 6DOF_v4_pure/thermal_stress/thermal_stress_runner.py \
    --duration 600 --mode cpu-stress --heat-first 120
```

اضغط `Ctrl-C` مرّة لإيقاف الاختبار مبكّراً مع تشغيل التحليل، ومرّتين
للخروج الفوري.

## 6) افتح النتائج

// turbo
```bash
ls -lt 6DOF_v4_pure/thermal_stress/results/ | head -3
```

أحدث مجلد يحوي:
- `thermal_stress_report.txt` — تقرير نصي مع PASS/FAIL والمقارنة pre/post throttle
- `thermal_stress.metrics.json` — كل المقاييس JSON
- `thermal_stress_plot.html` — لوحة Plotly تفاعلية (3 محاور: حرارة + throttle + MPC)
- `thermal_log.csv` — telemetry حراري @ 1 Hz
- `mpc_timing.csv` — RktGNC samples (mpc_solve_us, cycle_us, dt_max)
- `config_used.yaml` — snapshot للإعدادات
- `pil_logs/` (إذا mode=pil-loop) — log لكل رحلة PIL

افتح التقرير:
```bash
cat $(ls -dt 6DOF_v4_pure/thermal_stress/results/*/ | head -1)thermal_stress_report.txt
```

افتح الـ HTML plot:
```bash
xdg-open $(ls -dt 6DOF_v4_pure/thermal_stress/results/*/ | head -1)thermal_stress_plot.html
```

## 7) (اختياري) إعادة تحليل run سابق

```bash
python3 6DOF_v4_pure/thermal_stress/thermal_stress_runner.py \
    --analyze-only 6DOF_v4_pure/thermal_stress/results/20260503_HHMMSS/
```

## 8) تفسير النتائج

### القرار التلقائي

```
VERDICT: ✓ PASS    أو    ✗ FAIL
[FAIL] MPC solve p99=22341μs > 20000μs        ← تجاوز عتبة p99
[FAIL] Post-throttle p99 solve 2.34× > pre-...  ← تدهور بسبب الحرارة
[WARN] Phone thermal status reached MODERATE   ← تنبيه HAL
```

### المقارنة pre/post throttle

في حال رصد throttling، يُقسَّم MPC على نقطة بداية أوّل throttle window
وتُقاس النسبة المباشرة:

```
Pre-throttle vs post-throttle (split at 487.3s):
    n:                  pre=  6132 post=  4218
    mpc_p99_us:         pre=  4892 post= 11241   ← 2.3× زيادة
    deadline_miss_pct:  pre= 0.012 post= 1.847   ← 150× زيادة
```

العتبات في `thresholds.throttle_degradation`:
- `solve_us_ratio_max: 2.0` ← post/pre p99 المسموح
- `miss_rate_delta_max: 0.10` ← زيادة miss rate المسموحة (10pp)

## 9) سيناريوهات شائعة

| الهدف | الأمر |
|------|-------|
| تأكُّد سريع أن النظام يعمل | `--preset quick` |
| Test قياسي قبل launch | `--preset standard` (شغّل HIL في نافذة أخرى) |
| محاكاة صحراء قاسية | `--preset desert` |
| اختبار stamina طويل (60 دقيقة) | `--preset extreme` |
| Tuning بدون يدوي | `--mode pil-loop --duration 1800` |
| فقط تحليل قياس قديم | `--analyze-only results/...` |

## 10) ملاحظات

- **adb واحد لكل العمليات**: الـ thermal poller و cpu-stress driver كلاهما
  يستخدمان `adb shell`. تم اختبارهما بالتوازي (36/36 sample بدون أخطاء).
- **Throttle ratio = 0.95** يعني الكيرنل خفّض `scaling_max` إلى 95% من
  `cpuinfo_max` — هذا "soft throttle" ويبدأ مبكّراً عند ~75°C.
- **Thermal status >= 2** (MODERATE) يحدث عند ~85°C ويبطئ MPC بشكل ملحوظ.
- **Status = 3+** (SEVERE/CRITICAL) خطر — Android قد يُغلق العمليات.
- **بدون MPC data**: الـ verdict سيُنبّه `[WARN] No MPC data captured`
  لأن RktGNC stream فارغ. شغّل HIL/PIL خارجياً أو استخدم `--mode pil-loop`.
