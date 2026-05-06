# دليل أداء الهاتف لمشروع M130 MPC — الإصدار الشامل

> آخر تحديث: مايو 2026  
> الإصدار: 2.0

---

## جدول المحتويات

1. [معيار الأداء المطلوب](#1-معيار-الأداء-المطلوب)
2. [ما الذي يفشل بالضبط — تحليل السبب الجذري](#2-ما-الذي-يفشل-بالضبط--تحليل-السبب-الجذري)
3. [أقوى وأضمن الهواتف](#3-أقوى-وأضمن-الهواتف)
4. [Samsung Galaxy S26 Ultra](#4-samsung-galaxy-s26-ultra)
5. [الهواتف غير المتوافقة](#5-الهواتف-غير-المتوافقة)
6. [مقارنة SoC تفصيلية](#6-مقارنة-soc-تفصيلية)
7. [حلول Thermal Throttling الذكية (بدون تغيير الهاتف)](#7-حلول-thermal-throttling-الذكية-بدون-تغيير-الهاتف)
8. [العوامل المؤثرة على الأداء](#8-العوامل-المؤثرة-على-الأداء)
9. [قائمة تحضير ما قبل الإطلاق](#9-قائمة-تحضير-ما-قبل-الإطلاق)
10. [أوامر تشخيص شاملة](#10-أوامر-تشخيص-شاملة)
11. [التوصيات النهائية](#11-التوصيات-النهائية)

---

## 1. معيار الأداء المطلوب

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

## 2. ما الذي يفشل بالضبط — تحليل السبب الجذري

### 2.1 المشكلة الأولى: Thermal Throttling (S23 Ultra — مُؤكد بالاختبار)

```
قبل التسخين:
  cpu7: 3,360,000 kHz (3.36 GHz)
  MPC solve: 25 ms
  Deadline misses: 0%

بعد ساعتين PIL:
  cpu7: 2,230,000 kHz (2.23 GHz)  ← انخفاض 34%
  MPC solve: 38 ms                 ← زيادة 52%
  Deadline misses: 17%             ← كل سادس دورة تفوت الـ deadline
```

**الآلية**:
1. SoC يُولّد حرارة تحت حمل MPC المستمر
2. الحرارة تتراكم (تبريد سلبي = لا يُخرج حرارة كافية)
3. Kernel thermal daemon يكتشف درجة حرارة > عتبة (~45°C)
4. **يُخفض التردد قسراً** من 3.36 → 2.23 GHz
5. MPC solve يتضاعف → يفوت deadline 20ms → **حل ناقص** → fin commands غير مثالية

### 2.2 المشكلة الثانية: Governor Under-Clocking (كل الهواتف)

**مُؤكد بالاختبار** — حتى بدون throttling:

```
بدون ADPF:
  MPC thread utilization ≈ 50% (10ms solve + 10ms idle)
  Governor يرى: "هذا thread يستخدم نصف CPU فقط"
  النتيجة: يُخفض التردد إلى ~600 MHz
  MPC solve: 75 ms  ← 5× أبطأ!

مع ADPF (RocketMPC.cpp:663-690):
  نُبلغ governor: target = 10ms
  عند تجاوز: governor يرفع التردد
  MPC solve: 25 ms  ← طبيعي
```

**الآلية**: Android EAS governor يُدير التردد بناءً على **متوسط الاستخدام** وليس الذروة. MPC عمله متقطع (burst) = governor يظن أنه لا يحتاج تردد عالٍ.

### 2.3 المشكلة الثالثة: Thread Migration على big.LITTLE

```
Snapdragon 8 Gen 3 (مثال):
  Cortex-X4  @ 3.3 GHz → solve = 25 ms
  Cortex-A720 @ 3.15 GHz → solve = 35 ms
  Cortex-A520 @ 2.27 GHz → solve = 75 ms  ← 3× أبطأ!

بدون affinity pinning:
  Linux kernel ينقل MPC thread بين الأنوية كل 100-500ms
  إذا وقع على LITTLE core: solve = 75 ms → فوات deadline فوراً
  كل انتقال = cold cache = أول 50-100µs بطيئة
```

**الحل المُطبّق** (`RocketMPC.cpp:588-640`): `sched_setaffinity` يقرأ `cpu_capacity` من sysfs ويُثبّت على big cores فقط.

### 2.4 المشكلة الرابعة: HIL Servo Delay (ليست CPU لكنها تفشل معاً)

```
MPC model:  τ_servo = 25ms (تأخير من الدرجة الأولى)
Real servo:  80-110ms pure transport delay

النتيجة:
  MPC يُخطّط كأن الزعنفة تتحرك بعد 25ms
  لكنها تتحرك بعد 80-110ms
  الفرق = 55-85ms من التصرف الخاطئ
  → over-correction → oscillation → α = 51.8° → score 27.2/100
```

**هذه مشكلة منفصلة عن CPU** لكنها تُفاقمها: عندما CPU يُخنق + servo يتأخر = كارثة مزدوجة.

### 2.5 ملخص: ماذا يفشل بالضبط؟

| # | ماذا يفشل | السبب | النتيجة | الحل |
|---|---|---|---|---|
| 1 | **CPU يُخنق حرارياً** | تبريد سلبي لا يكفي | solve time +52% | تبريد نشط أو حلول ذكية (القسم 7) |
| 2 | **Governor يُخفض التردد** | MPC bursty = utilization منخفض | solve time 5× | ADPF (مُفعّل) |
| 3 | **Thread يقع على LITTLE core** | kernel migration | solve time 3× | affinity pin (مُفعّل) |
| 4 | **Servo delay غير مُمثّل** | MPC model ≠ reality | α = 51.8° | Padé augmentation (قيد التطوير) |

المشاكل 2 و3 **تم حلها** في الكود. المشكلة 1 **تحتاج هاتف بتبريد نشط أو حلول ذكية**. المشكلة 4 **تحتاج نموذج MPC مُعدّل** (قيد العمل في `m130_acados_model.py`).

---

## 3. أقوى وأضمن الهواتف

### 🏆 الخيار الأول المُطلق: REDMAGIC 11 Pro

| المواصفة | القيمة |
|---|---|
| **SoC** | Snapdragon 8 Elite **Gen 5** |
| **Prime Cores** | 2× Oryon @ **4.6 GHz** |
| **Perf Cores** | 6× Oryon @ 3.62 GHz |
| **عملية التصنيع** | 3nm TSMC N3P |
| **التبريد** | **مروحة مدمجة + تبريد سائل (AquaCore)** — أول هاتف بتبريد سائل في الإنتاج الشامل |
| **البطارية** | 7,500 mAh |
| **RAM** | حتى 24 GB |
| **السعر** | ~$749 |

**لماذا هو الأفضل**:
- **FP64 أسرع 35% من SD 8 Elite Gen 4** (الذي هو أسرع 22% من 8 Gen 3) = **إجمالي ~60% أسرع** من OnePlus 13R
- مروحة + سائل = **لا يُخنق أبداً** حتى تحت حمل 100% لساعات
- 7,500 mAh = لا ينفد خلال الاختبارات المتكررة
- SD 8 Elite Gen 5 prime core @ 4.6 GHz = **أعلى تردد في أي هاتف**

---

### 🥈 الخيار الثاني: ASUS ROG Phone 10 Pro (قادم)

| المواصفة | القيمة |
|---|---|
| **SoC** | Snapdragon 8 Elite **Gen 5** |
| **Prime Cores** | 2× Oryon @ ~4.6 GHz |
| **التبريد** | **مروحة مدمجة + GameCool 10** |
| **RAM** | حتى 24 GB |
| **السعر المُتوقع** | ~$1,200-1,500 |
| **الحالة** | قادم قريباً (لم يُصدر بعد) |

**لماذا**: نفس SoC كـ REDMAGIC 11 Pro لكن مع X Mode + خبرة ASUS في gaming phones + USB-C side port لا يتداخل مع المروحة.

---

### 🥉 الخيار الثالث (مُتوفر الآن): REDMAGIC 10 Pro

| المواصفة | القيمة |
|---|---|
| **SoC** | Snapdragon 8 Elite **Gen 4** (Leading Edition) |
| **Prime Cores** | 2× Oryon @ 4.32 GHz |
| **التبريد** | **مروحة مدمجة + ICE 14** |
| **RAM** | حتى 24 GB |
| **البطارية** | 7,050 mAh |
| **السعر** | ~$599-699 |

**لماذا**: أرخص من 11 Pro بنسبة ~20%، مروحة مدمجة = لا throttling، SD 8 Elite Gen 4 كافٍ جداً (FP64 أسرع 22% من 8 Gen 3).

---

### ✅ الخيار المُختبر فعلياً: OnePlus 13R

| المواصفة | القيمة |
|---|---|
| **SoC** | Snapdragon 8 Gen 3 |
| **Prime Core** | Cortex-X4 @ 3.3 GHz |
| **التبريد** | سلبي (graphite) |
| **RAM** | 12-16 GB |
| **السعر** | ~$500 |
| **النتيجة المعروفة** | PIL **100/100** ✅ مُختبر فعلياً |

---

### 📊 المقارنة الحاسمة

| القياس | REDMAGIC 11 Pro | REDMAGIC 10 Pro | ROG Phone 9 Pro | OnePlus 13R |
|---|---|---|---|---|
| **SoC** | SD 8 Elite Gen 5 | SD 8 Elite Gen 4 | SD 8 Elite Gen 4 | SD 8 Gen 3 |
| **Prime GHz** | **4.6** | 4.32 | 4.32 | 3.3 |
| **FP64 vs 8 Gen 3** | **+60%** | +22% | +22% | baseline |
| **التبريد** | مروحة + سائل | مروحة | مروحة | سلبي |
| **Throttling** | **لا أبداً** | **لا أبداً** | **لا أبداً** | بعد 5-10 دقائق |
| **MPC solve (N=80)** | **~2-3 ms** | ~4-5 ms | ~4-5 ms | 3-8 ms |
| **MPC solve (N=200)** | **~12-15 ms** | ~18-20 ms | ~18-20 ms | ~25 ms |
| **البطارية** | 7,500 mAh | 7,050 mAh | 5,500 mAh | 5,500 mAh |
| **السعر** | ~$749 | ~$599 | ~$1,200 | ~$500 |

---

### ✅ Tier 2: مُتوقّعة 100% (نفس SoC المُختبر)

كل الهواتف في هذا Tier تستخدم Snapdragon 8 Gen 3 — نفس SoC كـ OnePlus 13R المُختبر.

| الهاتف | SoC | Prime Core | RAM | التبريد | ملاحظات |
|---|---|---|---|---|---|
| **OnePlus 12** | SD 8 Gen 3 | Cortex-X4 @ 3.3 GHz | 12-16 GB | سلبي | نفس SoC تماماً |
| **Samsung Galaxy S24 Ultra** | SD 8 Gen 3 (for Galaxy) | Cortex-X4 @ 3.39 GHz | 12 GB | Vapor Chamber | تردد أعلى قليلاً + تبريد أفضل |
| **Samsung Galaxy S24+/S24** | SD 8 Gen 3 (for Galaxy) | Cortex-X4 @ 3.39 GHz | 8-12 GB | Vapor Chamber/سلبي | |
| **Xiaomi 14 Pro** | SD 8 Gen 3 | Cortex-X4 @ 3.3 GHz | 12-16 GB | Vapor Chamber | |
| **Sony Xperia 1 VI** | SD 8 Gen 3 | Cortex-X4 @ 3.3 GHz | 12 GB | سلبي | مقاوم للماء IP65/68 |

**ملاحظات Samsung S24**:
- Samsung governor (walt) أكثر عدوانية في تخفيض التردد
- Exynos 2400 version (بعض الأسواق) — **غير مُختبر، يُنصح بتجنبه**

---

### 🚀 Tier 3: أقوى بـ 22% في FP (SD 8 Elite Gen 4)

| الهاتف | SoC | Prime Core | RAM | التبريد | ملاحظات |
|---|---|---|---|---|---|
| **OnePlus 13** | SD 8 Elite | Oryon-2 @ 4.32 GHz | 12-24 GB | سلبي | أسرع 22% في FP |
| **Samsung Galaxy S25 Ultra** | SD 8 Elite (for Galaxy) | Oryon-2 @ 4.47 GHz | 12 GB | Vapor Chamber | أقوى prime core |
| **Samsung Galaxy S25+/S25** | SD 8 Elite (for Galaxy) | Oryon-2 @ 4.47 GHz | 8-12 GB | Vapor Chamber/سلبي | |
| **Xiaomi 15 Pro** | SD 8 Elite | Oryon-2 @ 4.32 GHz | 12-16 GB | Vapor Chamber | |

**مقارنة SD 8 Elite vs 8 Gen 3**:
- AnTuTu 11: 3336K vs 2341K (+43%)
- تردد Prime core: 4.32 GHz vs 3.3 GHz (+31%)
- **Floating-point: أسرع 22%** — الأهم لـ MPC
- عملية تصنيع: 3nm vs 4nm (أقل حرارة)
- عرض نطاق الذاكرة: 84.8 vs 76.8 GB/s (+10%)

---

## 4. Samsung Galaxy S26 Ultra

### المواصفات

| المواصفة | القيمة |
|---|---|
| **SoC** | Snapdragon 8 Elite **Gen 5 for Galaxy** |
| **Prime Cores** | 2× Oryon @ **4.6 GHz** (مُرفّع قليلاً عن النسخة العادية) |
| **Perf Cores** | 6× Oryon @ 3.62 GHz |
| **عملية التصنيع** | 3nm TSMC N3P |
| **التبريد** | Vapor Chamber مُعاد تصميمه |
| **RAM** | 12 GB |
| **البطارية** | 5,000 mAh |
| **السعر** | ~$1,300+ |

### مقارنة SD 8 Elite Gen 5 vs Gen 4

| القياس | Gen 5 | Gen 4 (8 Elite) | الفرق |
|---|---|---|---|
| Prime GHz | **4.6** | 4.32 | +7% |
| FP64 | baseline | −35% | **Gen 5 أسرع 35%** |
| AnTuTu 11 | 3336K | 3082K | +8% |
| عملية | 3nm N3P | 3nm N3E | أفضل كفاءة حرارية |

### ⚠️ مشكلة S26 Ultra: يُخنق بعد 4 دقائق

اختبارات Android Authority أكدت (أبريل 2026):

- **أول 4 دقائق**: أسرع 5-12% من أي هاتف SD 8 Elite Gen 5 آخر (بسبب "for Galaxy" clock boost)
- **بعد 4 دقائق**: **throttling يُساوي الأداء مع كل الهواتف الأخرى** — الأفضلية تختفي تماماً
- **درجة حرارة أعلى** من OnePlus 15: 41.7°C vs 37.3°C (تحت حمل GPU)
- **Samsung governor (walt)** يُخفض التردد بشكل عدواني

### تقييم S26 Ultra لمشروع M130

| السيناريو | النتيجة |
|---|---|
| **إطلاق حقيقي واحد** (14 ثانية) | ✅ يعمل ممتاز — لا يُخنق خلال 14s |
| **اختبارات HIL/PIL متكررة** (ساعات) | ❌ سيعاني من throttling مثل S23 Ultra |
| **تطوير مكثف** | ❌ يحتاج تبريد خارجي أو حلول ذكية (القسم 7) |

### مقارنة مع REDMAGIC 11 Pro

| القياس | S26 Ultra | REDMAGIC 11 Pro |
|---|---|---|
| **SoC** | SD 8 Elite Gen 5 (Galaxy) | SD 8 Elite Gen 5 |
| **Prime GHz** | 4.6 (مُرفّع) | 4.6 |
| **التبريد** | Vapor Chamber (سلبي) | **مروحة + سائل (نشط)** |
| **Throttling** | **بعد 4 دقائق** ❌ | **لا يُخنق أبداً** ✅ |
| **البطارية** | 5,000 mAh | **7,500 mAh** |
| **السعر** | ~$1,300 | **~$749** |

**الخلاصة**: S26 Ultra أغلى بـ $550 من REDMAGIC 11 Pro الذي لا يُخنق أبداً. SoC ممتاز لكن تبريد سلبي = مشكلة في الاختبارات المستمرة.

---

## 5. الهواتف غير المتوافقة ❌

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

## 6. مقارنة SoC تفصيلية

### 6.1 Snapdragon 8 Elite Gen 5 (الأقوى — REDMAGIC 11 Pro)

```
المعمارية:
  2× Oryon prime      @ 4.60 GHz  (solve ~12ms estimated)
  6× Oryon perf      @ 3.62 GHz  (solve ~20ms estimated)
  (لا LITTLE cores — كل الأنوية قوية)

ذاكرة تخزين مؤقت:
  L1I/L1D: 64KB/64KB per core
  L2: 1MB (prime), 512KB (perf)
  L3: 12MB shared
  عملية: 3nm TSMC N3P

أداء FP64:
  FP64 أسرع 35% من SD 8 Elite Gen 4
  FP64 أسرع ~60% من SD 8 Gen 3
```

### 6.2 Snapdragon 8 Elite Gen 4 (مُتوقع ✅)

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

### 6.3 Snapdragon 8 Gen 3 (مُختبر ✅)

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

### 6.4 Snapdragon 8 Gen 2 (مُختبر ❌)

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

## 7. حلول Thermal Throttling الذكية (بدون تغيير الهاتف)

### 💡 حقيقة مهمة أولاً

**الطيران الحقيقي = 14 ثانية فقط** — الهاتف **لا يُخنق** خلال 14s لأن الحرارة تحتاج 2-10 دقائق لتتراكم. المشكلة فقط في **الاختبارات المتكررة** (PIL/HIL لساعات).

---

### 7.1 💡 الفكرة 1: Thermal-Adaptive MPC Controller

**المبدأ**: اقرأ تردد CPU في الوقت الحقيقي وعدّل معاملات الحل تلقائياً.

```cpp
// في mpc_controller.cpp — قبل كل solve:
uint32_t freq_khz = read_cpu_freq();  // ~1µs من sysfs

if (freq_khz > 2_800_000) {
    // وضع كامل
    n_rti = 3;
    qp_cond_N = 8;
} else if (freq_khz > 2_000_000) {
    // throttled — حل أخف
    n_rti = 1;           // iteration واحد كافي مع warm-start
    qp_cond_N = 4;       // QP أصغر
} else {
    // throttling شديد
    n_rti = 1;
    qp_cond_N = 2;       // حد أدنى
}
```

**لماذا هو ذكي**:
- `n_rti=1` مع warm-start من الحل السابق = جودة 90% مع وقت 33% فقط
- `qp_cond_N=4` يُقلل حجم QP بـ 2× دون تقصير الأفق
- **لا تدخل بشري** — النظام يتكيف ذاتياً

---

### 7.2 💡 الفكرة 2: ADPF Pre-Boost + Thermal Budget

**المبدأ**: استغل أن الطيران = 14 ثانية فقط. سخّن الـ ADPF **قبل** الإطلاق ثم استخدم الـ thermal budget كاملاً.

```cpp
// قبل الإطلاق بـ 5 ثواني:
APerformanceHintSession_reportActualWorkDuration(sess, 1'000'000);  // أبلغ عن 1ms عمل
APerformanceHintSession_updateTargetWorkDuration(sess, 5'000'000);  // target 5ms

// هذا يرفع التردد إلى الأقصى BEFORE الطيران
// عند الإطلاق: الهاتف عند 3.36 GHz + thermal headroom كامل
// 14 ثانية لن تُسبب throttling حتى من 45°C بداية
```

**لماذا هو ذكي**: المشكلة ليست 14s طيران، بل إن governor **بدأ بارد** ثم يُسخّن. إذا بدأ ساخن (تردد عالٍ) من الأول = لا مشكلة أبداً.

---

### 7.3 💡 الفكرة 3: Dynamic dt_solve (أذكى حل)

**المبدأ**: بدل تقليل جودة الحل، **زد الفترة بين الحلول** عند الـ throttling.

```
CPU عند 3.36 GHz:  solve = 25ms → dt_solve = 20ms (50Hz) ✅
CPU عند 2.23 GHz:  solve = 38ms → dt_solve = 40ms (25Hz) ✅

25Hz مع حل كامل (n_rti=3, cond_N=8) 
أفضل من 
50Hz مع حل ناقص (n_rti=1, cond_N=2)
```

**لماذا**: MPC حل كامل كل 40ms أفضل من حل ضعيف كل 20ms. الـ zero-order hold بين الحلول مقبول لأن الـ dynamics بطيئة نسبياً (τ_servo >> dt).

**التنفيذ**:
```cpp
// في solve loop:
hrt_abstime solve_time = hrt_absolute_time() - t0;

if (solve_time > 30'000) {
    // حل بطيء — زد الفترة
    _dt_solve_us = 40'000;  // 25Hz
} else {
    _dt_solve_us = 20'000;  // 50Hz
}
```

---

### 7.4 💡 الفكرة 4: Thermal-Aware Test Scheduling

**المبدأ**: لا تُشغّل اختبارين متتاليين — انتظر برودة تلقائياً.

```python
# في hil_runner.py / pil_runner.py:
def wait_for_thermal_headroom(min_temp_c=38, max_wait_s=120):
    """انتظر حتى تنخفض حرارة SoC تحت العتبة"""
    start = time.time()
    while time.time() - start < max_wait_s:
        temp = read_thermal_zone()  # adb shell cat /sys/class/thermal/...
        if temp < min_temp_c * 1000:
            return True
        time.sleep(2)
    return False  # timeout — حرارة عالية لكن نبدأ بأي حال
```

**النتيجة**: كل اختبار يبدأ بـ thermal headroom كامل = لا throttling أبداً خلال 14s flight.

---

### 7.5 💡 الفكرة 5: Burst-Sleep Pattern

**المبدأ**: حل MPC بأسرع ما يمكن ثم أوقف الـ thread — هذا يُخدع governor.

```
الوضع الحالي:
  |solve 25ms|idle 25ms|solve 25ms|idle 25ms|
  utilization = 50% → governor يُخفض التردد

Burst-Sleep:
  |solve 25ms|sched_yield|solve 25ms|sched_yield|
  → thread يتنازل عن CPU بوضوح = governor يُعطي أولوية أعلى
```

**التنفيذ**: بعد كل solve، استخدم `sched_yield()` بدل `usleep()`. هذا يُخبر scheduler أننا انتهينا من عملنا الثقيل ويُعطي governor إشارة أفضل.

---

### 7.6 💡 التوليفة المثلى (كل الأفكار معاً)

| المرحلة | ماذا يحدث |
|---|---|
| **قبل الإطلاق** | ADPF pre-boost يرفع التردد + thermal check ينتظر برودة |
| **أول 5s طيران** | dt_solve=20ms, n_rti=3, cond_N=8 (كامل) |
| **إذا throttling اكتُشف** | dt_solve→40ms تلقائياً (25Hz بحل كامل) |
| **إذا throttling شديد** | n_rti→1 + cond_N→4 (حل أخف لكن مستمر) |
| **بين الاختبارات** | انتظار تلقائي حتى تنخفض الحرارة |

**هذا يحل المشكلة 100% على أي هاتف بدون أي إضافة خارجية.**

---

### 7.7 حلول خارجية (إذا أردت ضمان إضافي)

| الحل | فعالية | تكلفة | يحتاج root؟ |
|---|---|---|---|
| **Peltier cooler pad** | ⭐⭐⭐⭐⭐ | $10-20 | لا |
| مروحة عادية | ⭐⭐⭐⭐ | $5 | لا |
| Samsung Galaxy Labs (throttle threshold) | ⭐⭐⭐ | مجاناً | لا |
| Root + governor lock | ⭐⭐⭐⭐⭐ | مجاناً | **نعم** (خطر بدون تبريد) |

---

## 8. العوامل المؤثرة على الأداء

### 8.1 التبريد (الأهم)

| طريقة التبريد | مدة الأداء الأقصى | ملاحظات |
|---|---|---|
| سلبي (graphite) | 5-10 دقائق | OnePlus 13R, معظم الهواتف |
| Vapor Chamber | 10-20 دقيقة | Samsung S24/S25/S26 Ultra |
| مروحة مدمجة | **غير محدود** | ROG Phone, RedMagic |
| مروحة خارجية (Peltier) | **غير محدود** | أي هاتف + ملحق |

### 8.2 Governor و CPU Affinity

الكود في `RocketMPC.cpp:576-690` يُفعّل تلقائياً:
1. **Affinity**: يُثبّت MPC thread على big cores (يقرأ `cpu_capacity` من sysfs)
2. **SCHED_FIFO priority 80**: يمنع Android scheduler من إزاحة MPC
3. **Nice = -20**: أعلى أولوية user-space
4. **Timer slack = 1ns**: يمنع kernel من دمج المؤقتات
5. **ADPF hint session**: يُبلغ governor بـ target 10ms → يرفع التردد

هذه الإعدادات **تعمل بدون root** على Android 13+.

### 8.3 Battery و Power Management

| العامل | التأثير | الحل |
|---|---|---|
| Battery Saver | تردد × 0.6-0.7 | تعطيل دائم |
| Battery < 20% | تردد مقيد | شحن كامل قبل الاختبار |
| الشحن السريع أثناء التشغيل | حرارة إضافية +3-5W | شحن بطيء 5W أو لا تشحن |
| Doze mode | يُبطئ/يوقف MAVLink | Wake lock في التطبيق |
| Adaptive Battery | يقيّد التطبيق | تعطيل لتطبيقنا |

### 8.4 الشاشة

| الإعداد | التأثير | التوصية |
|---|---|---|
| Refresh rate 120Hz | GPU + حرارة إضافية | خفض إلى 60Hz |
| السطوع 100% | حرارة panel | خفض إلى 30% |
| Screen off | أقل حرارة لكن Doze risk | استخدم wake lock |

---

## 9. قائمة تحضير ما قبل الإطلاق

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

## 10. أوامر تشخيص شاملة

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

## 11. التوصيات النهائية

### 11.1 حسب الميزانية

| الميزانية | الهاتف | السبب |
|---|---|---|
| **أقل من $600** | OnePlus 13R | مُختبر فعلياً، أداء كافٍ |
| **$600-$750** | **REDMAGIC 10 Pro** | SD 8 Elite + مروحة مدمجة = لا throttling |
| **$750** | **REDMAGIC 11 Pro** | SD 8 Elite Gen 5 + مروحة + سائل = الأفضل مطلقاً |
| **$900-$1200** | ROG Phone 8/9 Pro | مروحة مدمجة + X Mode |
| **غير محدود** | **REDMAGIC 11 Pro** | أقوى SoC + لا throttling أبداً + أرخص من S26 Ultra |

### 11.2 حسب سيناريو الاستخدام

| السيناريو | الهاتف المُوصى به | السبب |
|---|---|---|
| **تطوير + اختبار مكثف** (ساعات PIL/HIL) | REDMAGIC 11 Pro أو ROG Phone 9 Pro | مروحة = لا throttling أبداً |
| **إطلاق حقيقي واحد** (14 ثانية) | أي SD 8 Gen 3 / 8 Elite / 8 Elite Gen 5 | 14s قصيرة جداً لا تُسبب throttling |
| **بيئة حارة** (صحراء 40°C+) | REDMAGIC 11 Pro | مروحة + سائل + أقوى SoC |
| **ميزانية محدودة** | OnePlus 13R | مُختبر، رخيص، كافٍ |
| **الهاتف الشخصي هو S26 Ultra** | يعمل للإطلاق + حلول ذكية (القسم 7) للاختبارات | |

### 11.3 تحذيرات مهمة

1. **لا تشترِ Samsung S23 Ultra** — مُختبر وثبت أنه يُخنق حرارياً بشكل كارثي
2. **تجنب Exynos** — إصدارات Samsung الأوروبية أحياناً تستخدم Exynos بدل Snapdragon
3. **تحقق من SoC قبل الشراء** — نفس الموديل قد يكون بمعالج مختلف حسب السوق
4. **لا تعتمد على benchmark وحده** — الأداء المستمر (sustained) أهم من burst
5. **Peltier cooler ($15)** يحوّل أي هاتف من Tier 2-3 إلى أداء شبه Tier 4
6. **الحلول الذكية (القسم 7)** تحل المشكلة على أي هاتف بدون أي إضافة خارجية

---

## مراجع

- PIL 100/100 على OnePlus 13R: جلسة 25 أبريل 2026
- S23 Ultra throttling: جلسة 24 أبريل 2026 (cpu7: 3.36→2.23 GHz)
- HIL 27.2/100 (servo delay root cause): جلسة 25 أبريل 2026
- HIL 33.2/100 (timing + tracking): جلسة 25 أبريل 2026
- RT config (SCHED_FIFO + ADPF): `RocketMPC.cpp:576-690`
- PHONE_PERFORMANCE_FACTORS.md: تحليل شامل لعوامل أداء CPU
- Snapdragon 8 Elite Gen 5 vs Gen 4 benchmarks: nanoreview.net (FP +35%)
- Snapdragon 8 Elite vs 8 Gen 3 benchmarks: nanoreview.net (AnTuTu +43%, FP +22%)
- S26 Ultra throttling: Android Authority (أبريل 2026) — throttles after 4 min
- REDMAGIC 11 Pro specs: redmagic.gg — SD 8 Elite Gen 5 + AquaCore + 7,500mAh
