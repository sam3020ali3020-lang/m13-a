# دليل اختيار الهاتف والأداء لمشروع M130 MPC

> آخر تحديث: مايو 2026  
> الإصدار: 2.0 — شامل: اختيار الهاتف + حلول Thermal Throttling الذكية + تشخيص

يعمل نظام M130 MPC على الهاتف عبر PX4 autopilot. المعالج يجب أن يُنهي حل المسألة الأمثل (SQP_RTI) ضمن المهلة الزمنية لكل دورة تحكم (20ms = 50Hz).

| القياس | الحد المطلوب | السبب |
|---|---|---|
| RTI iteration واحد | ≤ 8 ms | N=80, cond_N=8 على ARM64 |
| 3 warm iterations | ≤ 25 ms | ضمن deadline 20ms + عامل أمان |
| مدة الطيران بدون thermal throttling | ≥ 15 s | flight time ≈ 14s |
| ADPF (Android 13+, API 33+) | مطلوب | رفع تردد CPU تلقائياً أثناء MPC solve |
| USB-C OTG | مطلوب | CAN adapter + ADB |
| SCHED_FIFO + affinity | مطلوب | مُفعّل في `RocketMPC.cpp:576-690` |

### 1.1 لماذا ADPF ضروري

بدون Android Dynamic Performance Framework، حاكم التردد (governor) يرى أن MPC thread يستخدم < 25% من دورة CPU (لأن الـ solve متقطع: 10ms عمل + 10ms انتظار) فيُخفض التردد إلى ~600 MHz. هذا يُضاعف زمن الحل 5×.

مع ADPF (مُفعّل في `RocketMPC.cpp:663-690`):
- نُبلغ governor أننا نتوقع 10ms عمل لكل دورة
- عند تجاوز الفعلي (30-70ms)، governor يرفع التردد نحو الأقصى
- النتيجة: تردد مستقر عند 2.5-3.3 GHz بدل 600 MHz

### 1.2 لماذا التبريد حرج

طيران الصاروخ يستغرق ~14 ثانية. لكن:
- الاختناق الحراري يبدأ بعد 2-10 دقيقة من الحمل المستمر
- في HIL/PIL، هناك إحماء + اختبارات متكررة قبل الإطلاق الفعلي
- انخفاض التردد من 3.36 GHz إلى 2.23 GHz = زمن الحل يتضاعف 1.5×

**مثال مُؤكد**: Samsung S23 Ultra بعد ساعتين من PIL مستمر:
- cpu7: 3.36 GHz → 2.23 GHz (انخفاض 34%)
- MPC solve time: 25ms → 38ms (+52%)
- Deadline misses: 0% → 17%

---

## 2. تصنيف الهواتف حسب التوافق

### ✅ Tier 1: مُؤكّدة 100% (مُختبرة فعلياً)

| الهاتف | SoC | Prime Core | RAM | التبريد | النتيجة المعروفة |
|---|---|---|---|---|---|
| **OnePlus 13R** | Snapdragon 8 Gen 3 | Cortex-X4 @ 3.3 GHz | 12-16 GB | سلبي (graphite) | PIL **100/100** ✅ مُختبر فعلياً |

**ملاحظات OnePlus 13R**:
- سعر تقريبي: ~$500
- لا يُخنق خلال 14s flight حتى بدون تبريد خارجي
- ADPF يعمل بشكل ممتاز (Android 14+)
- تم اختباره مع N=80 و N=200 بنجاح

---

### ✅ Tier 2: مُتوقّعة 100% (نفس SoC أو مكافئ)

كل الهواتف في هذا Tier تستخدم Snapdragon 8 Gen 3 — نفس SoC كـ OnePlus 13R المُختبر.

| الهاتف | SoC | Prime Core | RAM | التبريد | ملاحظات |
|---|---|---|---|---|---|
| **OnePlus 12** | SD 8 Gen 3 | Cortex-X4 @ 3.3 GHz | 12-16 GB | سلبي | نفس SoC تماماً |
| **Samsung Galaxy S24 Ultra** | SD 8 Gen 3 (for Galaxy) | Cortex-X4 @ 3.39 GHz | 12 GB | Vapor Chamber | تردد أعلى قليلاً + تبريد أفضل |
| **Samsung Galaxy S24+** | SD 8 Gen 3 (for Galaxy) | Cortex-X4 @ 3.39 GHz | 12 GB | Vapor Chamber | أصغر من Ultra لكن نفس SoC |
| **Samsung Galaxy S24** | SD 8 Gen 3 (for Galaxy) | Cortex-X4 @ 3.39 GHz | 8 GB | سلبي | RAM أقل لكن كافية |
| **Xiaomi 14 Pro** | SD 8 Gen 3 | Cortex-X4 @ 3.3 GHz | 12-16 GB | Vapor Chamber | تبريد جيد |
| **Xiaomi 14** | SD 8 Gen 3 | Cortex-X4 @ 3.3 GHz | 8-12 GB | سلبي | اقتصادي |
| **Sony Xperia 1 VI** | SD 8 Gen 3 | Cortex-X4 @ 3.3 GHz | 12 GB | سلبي | مقاوم للماء IP65/68 |
| **Honor Magic6 Pro** | SD 8 Gen 3 | Cortex-X4 @ 3.3 GHz | 12-16 GB | سلبي | |

**ملاحظات Samsung S24 Ultra**:
- Vapor Chamber أفضل من graphite cooling في OnePlus 13R
- لكن Samsung governor (walt) أكثر عدوانية في تخفيض التردد
- قد يحتاج تبريد خارجي في الاختبارات المتكررة
- Exynos 2400 version (بعض الأسواق) — **غير مُختبر، يُنصح بتجنبه**

---

### 🚀 Tier 3: أقوى بـ 22% في Floating-Point (SD 8 Elite)

Snapdragon 8 Elite (Gen 4) يستخدم معمارية Oryon-2 المُخصصة من Qualcomm بدل Cortex ARM.

| الهاتف | SoC | Prime Core | RAM | التبريد | ملاحظات |
|---|---|---|---|---|---|
| **OnePlus 13** | SD 8 Elite | Oryon-2 @ 4.32 GHz | 12-24 GB | سلبي (upgrade) | أسرع 22% في FP من 8 Gen 3 |
| **Samsung Galaxy S25 Ultra** | SD 8 Elite (for Galaxy) | Oryon-2 @ 4.47 GHz | 12 GB | Vapor Chamber أكبر | أقوى prime core في السوق |
| **Samsung Galaxy S25+** | SD 8 Elite (for Galaxy) | Oryon-2 @ 4.47 GHz | 12 GB | Vapor Chamber | |
| **Samsung Galaxy S25** | SD 8 Elite (for Galaxy) | Oryon-2 @ 4.47 GHz | 8-12 GB | سلبي | |
| **Xiaomi 15 Pro** | SD 8 Elite | Oryon-2 @ 4.32 GHz | 12-16 GB | Vapor Chamber | |
| **Xiaomi 15** | SD 8 Elite | Oryon-2 @ 4.32 GHz | 8-12 GB | سلبي | اقتصادي |
| **Honor Magic7 Pro** | SD 8 Elite | Oryon-2 @ 4.32 GHz | 12-16 GB | سلبي | |

**مقارنة SD 8 Elite vs 8 Gen 3**:
- AnTuTu 11: 3336K vs 2341K (+43%)
- تردد Prime core: 4.32 GHz vs 3.3 GHz (+31%)
- **Floating-point: أسرع 22%** — الأهم لـ MPC
- عملية تصنيع: 3nm vs 4nm (أقل حرارة)
- عرض نطاق الذاكرة: 84.8 vs 76.8 GB/s (+10%)

**ملاحظة**: SD 8 Elite لم يُختبر فعلياً مع M130 بعد. الأرقام أعلاه مبنية على benchmarks. يُتوقع أن يعمل بشكل ممتاز نظراً للأداء الأعلى في كل المقاييس.

---

### 🏆 Tier 4: الأفضل مطلقاً (أداء + تبريد نشط)

هذه الهواتف تتميز بـ **مروحة تبريد مدمجة** تمنع الـ thermal throttling تماماً.

| الهاتف | SoC | Prime Core | التبريد | سعر تقريبي | لماذا هو الأفضل |
|---|---|---|---|---|---|
| **ASUS ROG Phone 9 Pro** | SD 8 Elite | Oryon-2 @ 4.32 GHz | **مروحة مدمجة + GameCool 9** | ~$1200 | أقوى SoC + لا يُخنق أبداً |
| **ASUS ROG Phone 8 Pro** | SD 8 Gen 3 | Cortex-X4 @ 3.3 GHz | **مروحة مدمجة + GameCool 8** | ~$900 | لا يُخنق أبداً + SoC مُختبر |
| **RedMagic 9S Pro** | SD 8 Gen 3 Leading Version | Cortex-X4 @ 3.36 GHz | **مروحة مدمجة + ICE 13** | ~$650 | أرخص هاتف بمروحة مدمجة |

**لماذا التبريد النشط مهم**:
- المروحة المدمجة تبقي SoC عند 35-40°C حتى تحت حمل 100%
- بدون مروحة: نفس SoC يصل 55-65°C بعد 5 دقائق → throttling
- في بيئة الإطلاق الحقيقي (صحراء، 40°C محيط): التبريد النشط هو الفرق بين نجاح وفشل

**ميزة إضافية لـ ROG Phone**:
- X Mode / Level 3: يُعطي CPU أولوية قصوى، يُعطل thermal throttling
- USB-C port إضافي (side port) لا يتداخل مع Aerodynamic Cooler
- شاشة 165Hz يمكن خفضها لـ 60Hz لتقليل الحرارة

---

## 3. الهواتف غير المتوافقة ❌

| الهاتف / SoC | السبب | التفاصيل |
|---|---|---|
| **Samsung S23 Ultra** (SD 8 Gen 2) | **Thermal throttling شديد** | مُؤكد بالاختبار: 3.36→2.23 GHz بعد ساعتين. MPC solve time +52%. Deadline misses 17% |
| **Samsung S22 Ultra** (SD 8 Gen 1) | أبطأ 3× من 8 Gen 3 | SD 8 Gen 1 به مشاكل حرارة مزمنة |
| **Samsung S23/S23+** (SD 8 Gen 2) | نفس مشكلة Ultra | أصغر حجم = تبريد أسوأ |
| **Pixel 8/8 Pro** (Tensor G3) | FP64 أبطأ بكثير | Tensor G3 مُحسّن لـ ML ليس FP64 عادي |
| **Pixel 9/9 Pro** (Tensor G4) | FP64 أبطأ من Snapdragon | نفس المشكلة |
| **أي هاتف SD 7xx / 7+ Gen x** | LITTLE cores فقط | solve time 75ms+ = خارج deadline |
| **أي هاتف Dimensity 7xxx / 8xxx** | أداء FP64 غير مُتحقق | لم يُختبر، قد يعمل لكن بدون ضمان |
| **أي هاتف Exynos 1380/1480/2200** | أداء ضعيف | لا يقارن بـ Snapdragon flagship |

---

## 4. مقارنة تفصيلية: SoC حسب SoC

### 4.1 Snapdragon 8 Gen 3 (مُختبر ✅)

```
المعمارية:
  1× Cortex-X4 prime    @ 3.30 GHz  (solve ~25ms)
  3× Cortex-A720 perf   @ 3.15 GHz  (solve ~35ms)
  2× Cortex-A720 eff    @ 2.96 GHz  (solve ~45ms)
  2× Cortex-A520 LITTLE @ 2.27 GHz  (solve ~75ms)

ذاكرة تخزين مؤقت:
  L1I/L1D: 64KB/64KB per core
  L2: 1MB (prime), 512KB (perf), 256KB (eff/LITTLE)
  L3: 8MB shared
  عملية: 4nm TSMC N4P

أداء FP64 (GeekBench 6):
  Single-core: ~2200
  Multi-core:  ~7000
```

**MPC Performance على OnePlus 13R**:
- N=80, cond_N=8: avg 3-8ms/iteration, 3 warm iters = 10-25ms ✅
- N=200, cond_N=10: avg 25ms/iteration, 3 warm iters = 46ms (24Hz) ⚠️
- PIL score: **100/100** (N=200)

### 4.2 Snapdragon 8 Elite (مُتوقع ✅)

```
المعمارية:
  2× Oryon-2 prime     @ 4.32 GHz  (solve ~18ms estimated)
  4× Oryon-2 perf     @ 3.53 GHz  (solve ~25ms estimated)
  2× Oryon-2 eff      @ 2.69 GHz  (solve ~40ms estimated)
  (لا LITTLE cores — كل الأنوية قوية)

ذاكرة تخزين مؤقت:
  L1I/L1D: 64KB/64KB per core
  L2: 1MB (prime), 512KB (perf)
  L3: 12MB shared (4MB أكبر من 8 Gen 3)
  عملية: 3nm TSMC N3E

أداء FP64 (GeekBench 6):
  Single-core: ~3100 (+41% vs 8 Gen 3)
  Multi-core:  ~10200 (+46% vs 8 Gen 3)
```

**MPC Performance المُتوقع**:
- N=80, cond_N=8: avg 2-5ms/iteration ✅✅
- N=200, cond_N=10: avg 15-18ms/iteration, 3 warm iters = 45-54ms
- FP64 أسرع 22% = MPC solve أسرع ~20%

### 4.3 Snapdragon 8 Gen 2 (مُختبر ❌)

```
المعمارية:
  1× Cortex-X3 prime    @ 3.36 GHz → 2.23 GHz (throttled)
  2× Cortex-A715 perf   @ 2.80 GHz
  2× Cortex-A710 perf   @ 2.50 GHz
  3× Cortex-A510 LITTLE @ 2.02 GHz

عملية: 4nm Samsung 4LPE
```

**المشكلة**: Samsung S23 Ultra يُخنق بشدة:
- بعد 2 ساعة PIL: 3.36 → 2.23 GHz (-34%)
- MPC solve time: 25ms → 38ms (+52%)
- Deadline misses: 0% → 17%
- السبب: Samsung governor (walt) عدواني + thermal design غير كافٍ

---

## 5. العوامل المؤثرة على الأداء

### 5.1 التبريد (الأهم)

| طريقة التبريد | مدة الأداء الأقصى | ملاحظات |
|---|---|---|
| سلبي (graphite) | 5-10 دقائق | OnePlus 13R, معظم الهواتف |
| Vapor Chamber | 10-20 دقيقة | Samsung S24/S25 Ultra |
| مروحة مدمجة | **غير محدود** | ROG Phone, RedMagic |
| مروحة خارجية (Peltier) | **غير محدود** | أي هاتف + ملحق |

### 5.2 Governor و CPU Affinity

الكود في `RocketMPC.cpp:576-690` يُفعّل تلقائياً:
1. **Affinity**: يُثبّت MPC thread على big cores (يقرأ `cpu_capacity` من sysfs)
2. **SCHED_FIFO priority 80**: يمنع Android scheduler من إزاحة MPC
3. **Nice = -20**: أعلى أولوية user-space
4. **Timer slack = 1ns**: يمنع kernel من دمج المؤقتات
5. **ADPF hint session**: يُبلغ governor بـ target 10ms → يرفع التردد

هذه الإعدادات **تعمل بدون root** على Android 13+.

### 5.3 Battery و Power Management

| العامل | التأثير | الحل |
|---|---|---|
| Battery Saver | تردد × 0.6-0.7 | تعطيل دائم |
| Battery < 20% | تردد مقيد | شحن كامل قبل الاختبار |
| الشحن السريع أثناء التشغيل | حرارة إضافية +3-5W | شحن بطيء 5W أو لا تشحن |
| Doze mode | يُبطئ/يوقف MAVLink | Wake lock في التطبيق |
| Adaptive Battery | يقيّد التطبيق | تعطيل لتطبيقنا |

### 5.4 الشاشة

| الإعداد | التأثير | التوصية |
|---|---|---|
| Refresh rate 120Hz | GPU + حرارة إضافية | خفض إلى 60Hz |
| السطوع 100% | حرارة panel | خفض إلى 30% |
| Screen off | أقل حرارة لكن Doze risk | استخدم wake lock |

---

## 6. قائمة تحضير ما قبل الإطلاق

```
قبل كل اختبار HIL/PIL أو إطلاق حقيقي:

الهاتف:
[ ] أعد تشغيل الهاتف
[ ] أغلق كل التطبيقات الخلفية (adb shell am kill-all)
[ ] عطّل Battery Saver
[ ] عطّل Adaptive Battery لتطبيقنا
[ ] خفّض refresh rate إلى 60Hz
[ ] خفّض السطوع إلى 30%
[ ] فعّل Do Not Disturb
[ ] فعّل Performance / Game mode إذا متاح
[ ] أزل الغلاف (case) عن الهاتف
[ ] وجّه مروحة / مبرد إذا متاح
[ ] تأكد البطارية > 50%
[ ] لا تشحن أثناء الاختبار (أو شحن بطيء 5W فقط)
[ ] تأكد أن USB-C كابل أصلي قصير (< 1m)

التحقق من الأداء:
[ ] adb shell cat /sys/devices/system/cpu/cpu7/cpufreq/scaling_cur_freq
    → يجب أن يكون قريباً من الأقصى (> 2.5 GHz)
[ ] adb shell cat /sys/class/thermal/thermal_zone*/temp
    → يجب أن يكون < 40°C قبل البدء
[ ] adb shell dumpsys battery | grep level
    → يجب أن يكون > 50%

الاتصال:
[ ] استخدم كابل USB أصلي قصير
[ ] وصّل مباشرة (بدون hub)
[ ] تحقق: adb reverse tcp:4560 tcp:4560 (PIL)
    أو adb forward tcp:5760 tcp:5760 (HIL)

المراقبة أثناء الاختبار:
[ ] راقب cpu7 frequency: يجب أن يبقى > 2.5 GHz
[ ] راقب درجة الحرارة: إذا > 45°C أوقف وبرّد
[ ] لاحظ MPC timing في التقرير: avg < 25ms مقبول
```

---

## 7. أوامر تشخيص شاملة

```bash
echo "=== M130 Phone Diagnostics ==="
echo ""
echo "--- Device ---"
adb shell getprop ro.product.model
adb shell getprop ro.build.version.release
adb shell getprop ro.hardware

echo ""
echo "--- CPU Frequencies ---"
for i in 0 1 2 3 4 5 6 7; do
  cur=$(adb shell cat /sys/devices/system/cpu/cpu$i/cpufreq/scaling_cur_freq 2>/dev/null)
  max=$(adb shell cat /sys/devices/system/cpu/cpu$i/cpufreq/scaling_max_freq 2>/dev/null)
  gov=$(adb shell cat /sys/devices/system/cpu/cpu$i/cpufreq/scaling_governor 2>/dev/null)
  cap=$(adb shell cat /sys/devices/system/cpu/cpu$i/cpu_capacity 2>/dev/null)
  echo "  cpu$i: cur=${cur}kHz  max=${max}kHz  gov=$gov  cap=$cap"
done

echo ""
echo "--- Thermal ---"
for tz in $(adb shell ls /sys/class/thermal/ | grep thermal_zone); do
  type=$(adb shell cat /sys/class/thermal/$tz/type 2>/dev/null)
  temp=$(adb shell cat /sys/class/thermal/$tz/temp 2>/dev/null)
  echo "  $tz ($type): ${temp}"
done

echo ""
echo "--- Battery ---"
adb shell dumpsys battery | grep -E "level|health|voltage|temperature|status"

echo ""
echo "--- Memory ---"
adb shell cat /proc/meminfo | head -3

echo ""
echo "--- Power Management ---"
adb shell settings get global low_power
adb shell dumpsys deviceidle | head -5

echo ""
echo "--- ADPF Check ---"
adb shell getprop ro.build.version.sdk  # must be >= 33

echo ""
echo "--- App Priority ---"
adb shell am get-standby-bucket com.ardophone.px4v17
```

---

## 8. التوصيات النهائية

### 8.1 حسب الميزانية

| الميزانية | الهاتف | السبب |
|---|---|---|
| **أقل من $600** | OnePlus 13R | مُختبر فعلياً، أداء كافٍ |
| **$600-$900** | OnePlus 13 أو RedMagic 9S Pro | SD 8 Elite أو مروحة مدمجة |
| **$900-$1200** | ROG Phone 8 Pro | SD 8 Gen 3 + مروحة مدمجة |
| **غير محدود** | **ROG Phone 9 Pro** | SD 8 Elite + مروحة = الأفضل مطلقاً |

### 8.2 حسب سيناريو الاستخدام

| السيناريو | الهاتف المُوصى به | السبب |
|---|---|---|
| **تطوير + اختبار مكثف** (ساعات PIL/HIL) | ROG Phone 8/9 Pro | مروحة = لا throttling أبداً |
| **إطلاق حقيقي واحد** (14 ثانية) | أي SD 8 Gen 3 / 8 Elite | 14s قصيرة جداً لا تُسبب throttling |
| **بيئة حارة** (صحراء 40°C+) | ROG Phone 9 Pro | مروحة + أقوى SoC |
| **ميزانية محدودة** | OnePlus 13R | مُختبر، رخيص، كافٍ |

### 8.3 تحذيرات مهمة

1. **لا تشترِ Samsung S23 Ultra** — مُختبر وثبت أنه يُخنق حرارياً بشكل كارثي
2. **تجنب Exynos** — إصدارات Samsung الأوروبية أحياناً تستخدم Exynos بدل Snapdragon
3. **تحقق من SoC قبل الشراء** — نفس الموديل قد يكون بمعالج مختلف حسب السوق
4. **لا تعتمد على benchmark وحده** — الأداء المستمر (sustained) أهم من burst
5. **المروحة الخارجية (Peltier cooler)** تحوّل أي هاتف من Tier 2-3 إلى أداء شبه Tier 4

---

## مراجع

- PIL 100/100 على OnePlus 13R: جلسة 25 أبريل 2026
- S23 Ultra throttling: جلسة 24 أبريل 2026 (cpu7: 3.36→2.23 GHz)
- HIL 27.2/100 (servo delay root cause): جلسة 25 أبريل 2026
- RT config (SCHED_FIFO + ADPF): `RocketMPC.cpp:576-690`
- PHONE_PERFORMANCE_FACTORS.md: تحليل شامل لعوامل أداء CPU
- Snapdragon 8 Elite vs 8 Gen 3 benchmarks: nanoreview.net (AnTuTu +43%, FP +22%)
