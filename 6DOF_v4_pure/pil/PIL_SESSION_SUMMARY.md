# PIL Roadmap — ملخّص الجلسة الكامل

> **التاريخ:** 2026-05-05 → 2026-05-06  
> **الهدف:** تحقيق SITL parity في PIL (Processor-In-the-Loop) مع PX4 على Android  
> **النتيجة العامة:** 7 إصلاحات برمجيّة منجزة. الجذر النهائي = حد أداء الهاتف (يحتاج قرار خارجي).

---

## 1. المنهجيّة المتّبعة

```
Diagnose → Hypothesize → Verify → Fix → Test → Validate → Commit gate
قاعدة: لا ننتقل لنقطة جديدة قبل تأكيد ✅ السابقة.
```

---

## 2. الإصلاحات المنجزة (✅ موثّقة في الكود)

| # | الموضوع | الملف | التشخيص | الإصلاح |
|---|---|---|---|---|
| **F1** | EKF2 estimator path | `px4_jni.cpp` (HITL block) | `ROCKET_USE_GT=1` كان يستخدم ground-truth → false-pass | 1→0 |
| **F2** | Yaw alignment | `px4_jni.cpp` (HITL block) | `EKF2_MAG_TYPE=5` → magnetic deviation cause yaw drift | 5→6 (init-only) |
| **F3** | Gravity double-subtraction | `mavlink_bridge_pil.py:_body_specific_force` | كان `-C·g_ned` يطرح الجاذبية مرّتين | إزالة الطرح الإضافي |
| **F4** | Warmup pad forces | `mavlink_bridge_pil.py` (warmup) | pad forces خاطئة → EKF تتأخّر للاستقرار | تصحيح القوى لإطار pad الحقيقي |
| **F5** | Warmup launch quaternion | `mavlink_bridge_pil.py` (warmup) | identity quat بدل launch quat → EKF gravity check يفشل | استخدام launch quat + إعادة ضوضاء كل tick |
| **F6** | MPC rate budget | `RocketMPC.cpp:947`, `pil_config.yaml` | mpc avg 40ms مع deadline 20ms (50Hz) → 78% over-deadline | gate 19ms→39ms (50→25Hz) |
| **F7** | LOS rate-limiter bug | `los_guidance.h:set_gamma_natural` | تحديث `_gamma_natural` بدون `_gamma_ref_prev` → stale gref=1.504 (86°) | تحديث `_gamma_ref_prev` معاً |

### تفاصيل F7 (الأهم — لم يكن مُكتشَفاً قبل هذه الجلسة)
- **العَرَض:** run9 أعطى gref=1.504 rad (86° vertical climb) في أوّل solve → apogee=266m + post-apogee tumble.
- **الجذر:** عند ARM، EKF2 attitude transient → pitch≈86° → `gamma_natural=1.5` → `_los.reset()` → `_gamma_ref_prev=1.5`. لاحقاً تصحيح `gamma_natural=0.096` لكن `_gamma_ref_prev` محبوس على 1.5.
- **التأكيد:** run10-12 → gref=0.246 ✓ (15° launch pitch).

---

## 3. النقاط المُشخَّصة (NP) — توثيق التحقيق

### ✅ NP1: yaw alignment
- **الفرضيّة:** EKF2 لا يحسم yaw بدون magnetometer جيد.
- **التحقّق:** estimator flags `0x22e24000043` → tilt=1, yaw=1 at ARM (run6 logcat 22:59:19). ✓

### ✅ NP2: SITL vs PIL state divergence
- **النتيجة:** متطابقان عند t=0.31s. يتباعدان عند t=0.45s (ω_y -2.7→-8.9).
- **الجذر الأوّلي:** `inject_compute_delay_max_ms=80` + phone MPC avg=40ms vs deadline=20ms.
- **التأكيد:** run7 (delay=0): apogee 8m→74m (9× تحسّن).

### ✅ NP3: Decision على MPC rate
- **القرار:** Option A — تخفيض MPC rate (50→25Hz). gate 19→39ms.

### ✅ NP4: Verification بعد F6
- **run9** (25Hz, inject=80): apogee=266m, max α=14°, max fin=11°, NO saturation. لكن post-apogee tumble (NP5).

### ✅ NP5: post-apogee tumble
- لم يكن MPC. كان F7 (LOS bug) أعلى — gref=86° يُجبر تسلّقاً عمودياً يهلك V بسرعة → tumble طبيعي.

### ✅ NP7: fin sign verification (PIL vs SITL)
- استخراج apples-to-apples من CSV الملفّين:
  - SITL t=0.5: fins=[0.094, 0.080, -0.095, -0.081] → δe=-0.087 (-5.0°)
  - PIL  t=0.5: fins=[0.030, 0.021, -0.030, -0.021] → δe=-0.025 (-1.5°)
- **النتيجة:** الإشارات متطابقة ✓ — الفرق في **المقدار** (3×)، ليس bug في mixing.

### ✅ NP8: state divergence root cause
- **الاكتشاف الحاسم:** PIL.csv و SITL.csv **متطابقان حرفيّاً** في t=0.00→0.30 (V, γ, α, alt, mass).
  ```
  t=0.00 → 0.30: IDENTICAL
  t=0.40: SITL fins arrive first (PIL still [0,0,0,0])
  t=0.50: PIL fins arrive but 3× smaller magnitude
  ```
- **الجذر:** PIL→TCP-USB→Phone→PX4(EKF2+MPC ~50ms)→Phone→USB→PIL = **100-200ms loop**.
- **SITL** نفس الـloop على Linux محلّي = <5ms.
- **الأثر:** PIL MPC يحسب أوامر لحالة قديمة → δe صغير → استجابة ضعيفة → ballistic.

### ⚠️ NP6: Phone CPU throttling (جذر NP8)
- **التشخيص:**
  - `cpu7` (Cortex-X, 3.36 GHz, capacity=1024) محبوس على **864 MHz** (26%)
  - `cpu3-6` (A720, 2.8 GHz, capacity=811) على 2.19 GHz (78%)
  - Affinity to prime cores ✓ + ADPF session ✓ — **لكن `SCHED_FIFO=FAIL`** (no root)
- **السلسلة السببيّة:**
  ```
  app بدون root → CAP_SYS_NICE غير متاح → SCHED_FIFO يفشل
  → governor لا يُجبَر على boost cpu7 → cpu7 على 864 MHz
  → MPC على cpu3 (2.19 GHz) → 47-58ms solve > 40ms deadline
  ```
- **محاولات الإصلاح:**
  - `cmd power set-fixed-performance-mode-enabled true` → تأثير جزئي
  - sched_setaffinity to prime cores → نُفِّذ لكن لم يحرّك cpu7
- **الحلول الممكنة (تتطلّب قرار):**
  - **A.** Root الجهاز → SCHED_FIFO + governor=performance → cpu7@3.36GHz → MPC ~25-30ms
  - **B.** Regenerate solver N=80→40 (acados rebuild ~30 min) → MPC ~25ms
  - **C.** Smith-predictor في الـbridge (معقّد، قد يكسر sensor consistency)

---

## 4. جدول النتائج (Run table) — كل التجارب

| Run | Score | Apogee | Range | Max α | Notes |
|-----|-------|--------|-------|-------|-------|
| baseline (GT path) | 70 | 3m | 88m | 14° | false-pass via GT |
| run3 (after F3) | 70 | 4m | 89m | 11° | EKF2 path, gravity OK |
| run4 (after F5) | 70 | 3m | 88m | 7° | warmup parity |
| run5 (HIL_STATE) | 25 | 12m | 262m | 180° | tumbling (no yaw) |
| run6 (after F2) | 70 | 8m | 290m | 5° | stable, ballistic missing |
| run7 (delay=0, 50Hz) | 59 | 74m | 1044m | 8° | 9× improvement, MPC catches up |
| run8 (delay=40) | 70 | 4m | 124m | crash | high run-to-run variance |
| run9 (25Hz/d=80) | 25 | 266m | 1334m | 180° | LOS bug → vertical climb + tumble |
| run10 (LOS-fix) | 70 | 13m | 473m | 6° | gref correct now (0.246 vs 1.504) |
| run11 (fixed-perf) | 67 | 13m | 716m | 5° | cpu3@2.19GHz, cpu7@864MHz (idle) |
| run12 (inject=0) | 70 | 6m | 247m | 5° | even idealized phone too slow |
| run13 (inject=80,wind) | 70 | 4m | 107m | 6° | confirms NP8: lag dominates |

### المقارنة الحاسمة (SITL parity target)
| المقياس | SITL | run9 (LOS bug) | run10 (LOS fix) | run12 (idealized) |
|---|---|---|---|---|
| Apogee | ~112m | 266m | 13m | 6m |
| Range | ~2400m | 1334m | 473m | 247m |
| Trajectory | ✓ tracks gref | عمودي زائد | تتبّع جزئي | ballistic |

---

## 5. ما تبقّى من خارطة الطريق

### 🔴 المتبقّي الفعلي (يحتاج قرار خارجي):
1. **NP6 — حلّ حدّ أداء الهاتف** (مسار A أو B أو C)
2. **NP9 — post-apogee `attitude_mode: velocity_aligned`** ← **مفهوم لكن غير ذي معنى الآن** لأن apogee=4-13m فقط.

### 🟢 الجاهز للتنفيذ بعد NP6:
3. baseline run جديد بعد الإصلاح المختار → التحقّق من SITL parity
4. NP9: تنفيذ post-apogee attitude (إذا apogee >100m بعد NP6)
5. **MPC tracking RMSE quantitative** — قياس دقيق لـ |gamma - gref| على المدى الكامل
6. سلسلة Monte Carlo (5-10 runs مختلف seeds) للتحقّق من repeatability

### 🟡 خارج الـPIL (لكن مرتبط):
- HITL على عتاد حقيقي (السيرفوهات الفعلية + CAN bus) — يحتاج اختبار `/direct` و `/lab` أولاً
- Property-based tests (Hypothesis) لقوانين الإطار والرباعي
- TSan/ASan على SharedSensorData

---

## 6. الملفات المهمّة للجلسة القادمة

| الملف | الغرض |
|---|---|
| `pil/PIL_PROGRESS.txt` | متابعة لحظيّة لكل نقطة |
| `pil/PIL_SESSION_SUMMARY.md` | **هذا الملف — نقطة استئناف الجلسة** |
| `pil/PIL_GRAVITY_BUG_REPORT.md` | تقرير F3 المنفصل |
| `pil/RUN_COMMANDS.md` | أوامر التشغيل اليدويّة |
| `pil/results/pil_flight_*.csv` | بيانات الرحلة الخام |
| `pil/results/plots/pil_analysis_*.html` | تقارير HTML تحليليّة |
| `/tmp/pil_logcat_run*.log` | logcat لكل run |
| `/tmp/pil_run_test*.log` | stdout لكل run |

---

## 7. كيف نعود في الجلسة القادمة

```bash
# 1. اقرأ هذا الملف ثم PIL_PROGRESS.txt
cat /home/wd/Desktop/GAB_3/1234/m13/m13/6DOF_v4_pure/pil/PIL_SESSION_SUMMARY.md
cat /home/wd/Desktop/GAB_3/1234/m13/m13/6DOF_v4_pure/pil/PIL_PROGRESS.txt

# 2. اختر مسار NP6 (A/B/C)
# 3. نفّذ الأوامر من RUN_COMMANDS.md
# 4. سجّل النتيجة في الجدول أعلاه (run14, run15, ...)
```

---

## 8. القرارات المعلّقة الواجب اتّخاذها

- [ ] هل نُجرّب root للهاتف (مسار A)؟
- [ ] هل نُولّد solver N=40 (مسار B)؟ (آمن، لا يُعدّل الجهاز)
- [ ] هل نَكتفي بـPIL لتأكيد code paths فقط ونمضي لـHITL (مسار C)؟
- [ ] هل نُجرّب Smith-predictor في الـbridge؟ (تجريبي — قد يُخفي مشاكل أخرى)
