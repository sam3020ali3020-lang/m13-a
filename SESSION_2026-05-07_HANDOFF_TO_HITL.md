# SESSION 2026-05-07 — Handoff: حالة المشروع + الانتقال لـ HITL

> **هذا الملف تقرير handoff** للذكاء التالي. يقرأه فور بدء الجلسة (STEP 4 من
> القواعد). يلخّص أحدث ما وصلنا إليه وأين نريد المتابعة.

---

## 1. الحالة الراهنة لكل طبقة (آخر runs)

| الطبقة | آخر score | ملاحظة | المسار/الـCSV |
|---|---|---|---|
| **Stage 1: Python 6DOF** | **100/100** | range 2586 m، err -0.5%. مرجع علمي مستقر. | `6DOF_v4_pure/results/6dof_results.npz` |
| **Stage 2: SITL** | **89.8/100** ✅ | baseline x86_64 PX4. يطابق `tag: baseline-pil83-sitl89`. | `6DOF_v4_pure/sitl/results/sitl_20260506_180727.csv` |
| **Stage 3: PIL** | **83/100** baseline → آخر تجربة 70/100 ⚠️ | نزل إلى 70 في run اليوم 01:57. range=159 m، err -93.9%، MPC over-deadline 100% (avg=1000ms). transport=USB tunnel (127.0.0.1) — يجب التحقق إن كان السبب. | `6DOF_v4_pure/pil/results/pil_flight_20260507_015723.csv` |
| **Stage 4: HITL** | **58-63/100** ❌ | لم نُكمل بعد. آخر run 06-05 23:30 توقّف عند t=2.24s, range=156 m. ملاحظ: `fin_3=20°` bug + MPC=0 commands. servos تعمل عبر CAN. | `6DOF_v4_pure/hil/results/hil_flight_20260506_232913.csv` |

**git tag الحالي**: `baseline-pil83-sitl89` (commit `318f8c89`).

---

## 2. ما تم إنجازه قبل هذه الجلسة

- Stage 1 → Stage 3 جاهزة بـscores مقبولة (مع اختلاف باقٍ بين باسلاين 83 و run الأخير 70).
- bug PIL `position_lla` في warmup snapshot — تم إصلاحه (LESSONS_LEARNED.md).
- bug `skip_imu` race على instance 0 — تم إصلاحه (LESSONS_LEARNED.md).
- نظام الحوكمة AI_GOVERNANCE/ مُنشأ مع تحميل تلقائي عبر `.windsurf/rules/`.
- airframe 22003 (SITL) يستخدم `ROCKET_USE_GT=1` (groundtruth) → score مستقر.
- airframe 22004 (HITL) محدّد لكن لم نُكمل تطبيقه.

---

## 3. المهمة الحالية — Stage 4: HITL

### 3.1 الهدف
الوصول بـHITL إلى parity مع SITL/PIL ضمن tolerance المحدّدة في
`AI_GOVERNANCE/AI_OPERATING_RULES.md` §7.

### 3.2 الأعراض المعلومة من آخر HITL run
- toggle `fin_3=20°` ثابت (يُشير لخطأ في mapping/scaling).
- MPC commands = 0 (controller لا يصدر أوامر فعلية).
- CSV ينتهي عند t=2.24s — early termination (الصاروخ لم يطر).
- range = 156 m (مقابل 2586 m المتوقَّع من Python).
- servos تستجيب على CAN (أعمدة `δe_act_a/b/c/d, δr_act, δa_act` تتحرك).

### 3.3 الفجوة المتوقّعة (Stage 1 vs Stage 4)
| Metric | Python (ref) | آخر HITL | الفجوة |
|---|---|---|---|
| Peak range | ~2586 m | 156 m | -93.9% |
| Apogee time | ~14 s | لم يصل (t<2.24s) | لم يطر |
| Servo command | non-zero | يبدو 0 من MPC | فجوة في glue |

### 3.4 ملفات المُتوقَّع التحقيق فيها
- `6DOF_v4_pure/hil/mavlink_bridge_hil.py` — bridge HITL
- `6DOF_v4_pure/hil/hil_runner.py` — runner
- `6DOF_v4_pure/hil/hil_config.yaml` — config (IP الجوّال 10.42.0.145)
- `AndroidApp/.../airframes/22004_m130_rocket_mpc_hitl` — airframe HITL
- `AndroidApp/.../rc.rocket_defaults` — يفرض EKF2_MAG_TYPE=6
- وحدة CAN/PWM المستخدمة (servos mapping)

### 3.5 الجوّال الحالي
- Samsung S23 Ultra (SM-S918U).
- serial: `R5CW22JQ4GE` (تحقّق بـ`adb devices`).
- IP الجوّال على Ethernet: 10.42.0.145.
- IP اللابتوب: 10.42.0.1 (set via `setprop debug.m130.target_ip`).

---

## 4. الـbaselines التي يجب الإبقاء عليها

- **SITL 89.8/100**: لا تكسرها أثناء العمل على HITL.
- **PIL 83/100**: حاول استعادتها إن لزم (آخر run 70 — قد يكون transport USB tunnel).
- **Python 100/100**: مرجع مقدّس — لا يُعدَّل.

---

## 5. الخطوات المقترحة للذكاء التالي

1. **اقرأ القواعد الكاملة** (`AI_GOVERNANCE/AI_OPERATING_RULES.md`).
2. **أعلِن دورك** كـ مُشخِّص أولاً (لا مُصلِح بعد).
3. **اطلب إذن** قبل بدء الـHITL run الجديد.
4. **شغّل HITL مرة** (بعد التحقق من البناء + توصيل CAN + servos في صفر آمن).
5. **اجمع**: logcat + bridge stdout + flight CSV + servo CSV + timing CSV.
6. **شخّص** السبب الجذري لـ`fin_3=20°` و MPC=0.
7. **اعرض parity matrix** Python ↔ SITL ↔ PIL ↔ HITL.
8. **انتظر إذناً** قبل أي تعديل.

---

## 6. تذكير بالقواعد الحرجة

- ❌ ممنوع تعديل: `PX4-Autopilot/src/lib/**`، `ekf2/**`، `commander/**`، `sensors/**`، `acados-main/**`.
- ❌ ممنوع تعديل airframe HITL بدون إذن (قد يكون مختوماً).
- ❌ ممنوع تشغيل CAN/servos بدون التحقق من الـzero positions أولاً.
- ✅ كل ادعاء = رقم من log/csv.
- ✅ كل تعديل = backup قبله.
- ✅ كل فشل = revert فوري.

---

## 7. الأمل

نتمنى تجاوز:
- HITL: من 58-63 → ≥80
- PIL: من 70 → استعادة 83+ (إن كان طبيعياً تذبذبه)
- الوصول إلى parity matrix كاملة جاهزة للترقية لـReal Flight.
