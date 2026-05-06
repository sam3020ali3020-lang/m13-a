# Thermal Stress Test

اختبار طويل المدّة (30 دقيقة افتراضياً) لقياس استقرار MPC على الهاتف تحت
الحرارة. الهدف: الكشف عن **deadline misses** و **thermal throttling**
في سيناريو يطابق منصّة الإطلاق في الصحراء.

## ماذا يقيس

| المقياس | المصدر | الفائدة |
|---------|--------|---------|
| `mpc_solve_us` p50/p95/p99/p99.9 | `RktGNC` data[47] | توزيع زمن حلّ MPC |
| `cycle_us` p50/p95/p99/p99.9    | `RktGNC` data[48] | الـ cycle الكامل |
| `dt_max` | `RktGNC` data[36] | أكبر فجوة جدولة |
| **deadline miss rate** | حسابي | نسبة `cycle > 40ms` |
| `cpu_silver_max_C`, `cpu_gold_max_C` | `/sys/class/thermal/*` | حرارة CPU clusters |
| `gpu_max_C`, `ddr_max_C`, `battery_C`, `skin_C` | adb dumpsys + sysfs | حرارة GPU/DDR/البطارية/الجلد |
| `thermal_status` (HAL) | `dumpsys thermalservice` | 0=NONE → 6=SHUTDOWN |
| `throttle_ratio_gold`, `_silver` | `scaling_max/cpuinfo_max` | كشف throttling السريع |

عند وجود throttling، يتم حساب **p99 و miss rate قبل وبعد بداية أوّل
window** للمقارنة المباشرة لتأثير حرارة CPU على MPC.

## الملفات

```
6DOF_v4_pure/thermal_stress/
├── thermal_stress_config.yaml   ← إعدادات + presets + thresholds
├── thermal_poller.py            ← قرّاء telemetry حراري عبر adb (1 Hz)
├── load_driver.py               ← مولّدات الحمل (passive/cpu-stress/pil-loop)
├── thermal_stress_runner.py     ← المنسّق الرئيسي
├── thermal_stress_analysis.py   ← التحليل + HTML plot + verdict
├── requirements.txt
└── results/
    └── YYYYMMDD_HHMMSS/
        ├── config_used.yaml          ← snapshot للإعدادات
        ├── thermal_log.csv           ← telemetry حراري @ 1 Hz
        ├── mpc_timing.csv            ← RktGNC samples (mpc_solve_us, cycle_us, dt_max)
        ├── thermal_stress.metrics.json   ← المقاييس المحسوبة
        ├── thermal_stress_report.txt    ← ملخّص نصّي + verdict
        ├── thermal_stress_plot.html     ← لوحة تفاعلية (plotly)
        └── pil_logs/ (إذا mode=pil-loop) ← لكل رحلة PIL stdout
```

## التشغيل

### 1) المتطلبات

```bash
pip install -r 6DOF_v4_pure/thermal_stress/requirements.txt
```

ضمان وجود:
- `adb` متصل بهاتف Android (`adb devices`)
- التطبيق مُثبَّت ويعمل
- لِـ MAVLink: `adb forward tcp:5760 tcp:5760`
- لِـ `pil-loop`: `adb reverse tcp:4560 tcp:4560` بالإضافة

### 2) أوضاع التشغيل

```bash
cd 6DOF_v4_pure/thermal_stress

# (أ) مراقبة فقط — أنت شغّل HIL/PIL في نافذة أخرى، نحن نسجّل الحرارة + MPC
python3 thermal_stress_runner.py --duration 1800 --mode passive

# (ب) سخِّن الهاتف ذاتياً عبر workers مع MPC مُشغَّل خارجياً
python3 thermal_stress_runner.py --duration 1800 --mode cpu-stress

# (ج) شغِّل MPC تلقائياً عبر تكرار pil_runner طوال المدّة
python3 thermal_stress_runner.py --duration 1800 --mode pil-loop

# (د) presets جاهزة
python3 thermal_stress_runner.py --preset quick     # 5 دقائق smoke test
python3 thermal_stress_runner.py --preset standard  # 30 دقيقة passive
python3 thermal_stress_runner.py --preset desert    # 30 دقيقة + preheat 65°C + cpu-stress
python3 thermal_stress_runner.py --preset extreme   # 60 دقيقة + preheat 72°C + cpu-stress

# (هـ) تخطّي القياس وأعِد التحليل لمجلد سابق
python3 thermal_stress_runner.py --analyze-only results/20260503_123000/
```

### 3) الـ live progress

أثناء التشغيل، كل 10 ثواني يُطبع سطر:

```
[0:05:23/0:30:00] CPU_S=72.4°C CPU_G=78.1°C batt=39.2°C throt=0.85 st=1 |
                  mav✓ mpc=  4521μs dt_max= 41.3ms rows=8123 errs=0
```

| الحقل | المعنى |
|-------|--------|
| `CPU_S` | حرارة LITTLE cluster (silver) |
| `CPU_G` | حرارة big cluster (gold) |
| `throt` | min throttle ratio على gold cores |
| `st`    | thermal_status من HAL (0..6) |
| `mav`   | حالة اتصال MAVLink |
| `mpc`   | آخر `mpc_solve_us` |
| `dt_max`| آخر فجوة جدولة |
| `rows`  | عدد عيّنات RktGNC المسجّلة |
| `errs`  | عدد أخطاء MPC solver |

## التفسير

### Verdict تلقائي

في نهاية الاختبار، `thermal_stress_report.txt` يحتوي:

```
VERDICT: ✓ PASS   (أو ✗ FAIL)
[FAIL] MPC solve p99=22341μs > 20000μs
[WARN] Deadline miss rate 7.2% >= 5%
```

العتبات قابلة للتعديل في `thermal_stress_config.yaml` تحت `thresholds:`.

### Pre vs Post Throttle

عند رصد أوّل throttle window، نقسم الـ MPC samples لقسمين ونقارن:

```
Pre-throttle vs post-throttle (split at 487.3s):
    n:                  pre=  6132 post=  4218
    mpc_p99_us:         pre=  4892 post= 11241   ← 2.3× زيادة
    deadline_miss_pct:  pre= 0.012 post= 1.847   ← 150× زيادة
```

يُفشَّل الاختبار إن `post_p99 / pre_p99 > 2.0` أو `post_miss - pre_miss > 10%`.

### الـ HTML plot

افتح `thermal_stress_plot.html` في المتصفّح — لوحة تفاعلية بثلاثة محاور
متزامنة:

1. **Temperatures** — كل clusters + battery + skin
2. **Throttle ratio** — gold/silver + thermal_status (×6)
3. **MPC timing** — `mpc_solve_ms`, `cycle_ms`, `dt_max_ms` مع خط
   deadline أحمر

النوافذ المُكتشفة من throttling تظهر كمستطيلات حمراء شفّافة عبر كل
المحاور.

## نصائح للحصول على بيانات MPC حقيقية

اختبار `passive` بدون PX4 يعطي 0 صفّ MPC ➜ verdict ينبّه لذلك. لكي تحصل
على RktGNC حقيقي تحتاج إحدى:

- **بأبسط طريقة (cpu-stress + HIL يدوي)**: شغِّل `/hil` في نافذة أخرى،
  ثم `python3 thermal_stress_runner.py --mode cpu-stress`. هذا يعطيك:
  + MPC نشط من HIL
  + حرارة عالية من cpu-stress
  + قياس مباشر لتأثير الحرارة

- **pil-loop**: يكرّر `pil_runner.py` تلقائياً. أبسط اعتماداً لكنه يحتاج
  PIL setup عاملاً سابقاً.

## Architecture

```
┌────────────────────────────────────────────────────────────────┐
│  thermal_stress_runner.py  (الـ main thread + signal handler)  │
└──────┬──────────┬───────────┬──────────────────────────────────┘
       │          │           │
       ▼          ▼           ▼
┌───────────┐  ┌────────┐  ┌──────────────┐
│ Thermal   │  │ MPC    │  │ Load Driver  │
│ Poller    │  │ Reader │  │ (optional)   │
│ (1 Hz)    │  │ (≈25Hz)│  │              │
│  thread   │  │ thread │  │ thread/none  │
└─────┬─────┘  └────┬───┘  └──────┬───────┘
      │            │              │
      ▼            ▼              ▼
   adb shell    TCP:5760       adb shell
                MAVLink        (cpu-stress)
                                or
                                subprocess
                                (pil-loop)
                                
       ┌─────────────────┬─────────────────┐
       ▼                 ▼                 ▼
 thermal_log.csv    mpc_timing.csv     pil_logs/*.log
       │                 │
       └────────┬────────┘
                ▼
   thermal_stress_analysis.py
                ▼
   ┌─────────────┬──────────────┬──────────────────┐
   ▼             ▼              ▼                  ▼
 metrics.json  report.txt   plot.html         exit code (0/1)
```

كل الخيوط تتشارك `signal.SIGINT` handler — اضغط Ctrl-C مرّة لإيقاف
الاختبار مبكّراً مع تشغيل التحليل، ومرّتين للخروج الفوري.

## استكشاف الأخطاء

| مشكلة | السبب الأكثر شيوعاً | الحل |
|-------|--------------------|------|
| `adb not reachable: timed out` | الهاتف غير متصل | `adb devices` |
| `mav✗ rows=0` طوال الوقت | PX4 ليس يعمل أو لا forward | `adb forward tcp:5760 tcp:5760` ثم `pidof px4` على الهاتف |
| `deadline miss rate=100%` | PX4 يعمل لكن الصاروخ لم ينطلق ➜ MPC في idle | تأكّد من ضغط START في التطبيق ووصول الـ launch |
| الحرارة لا ترتفع رغم cpu-stress | الهاتف أبرد من المتوقع | استخدم `--heat-first 180` و/أو ضع الهاتف في حقيبة |
| HTML plot فارغ في بعض المحاور | `plotly` غير مثبّت أو لا توجد بيانات | `pip install plotly` |

## الإصدارات

- 1.0 (مايو 2026): إصدار أوّل — passive/cpu-stress/pil-loop modes،
  HTML dashboard، throttle window detection، pre/post split analysis.
