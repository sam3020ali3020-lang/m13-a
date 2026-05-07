# HITL Recovery Roadmap (2026-05-07)

**الهدف**: HITL ≥80/100 ثلاث runs متتالية + Max fin >0° ثابت + Range ≥2330m + لا failsafe قبل launch.

**المرجعية**: AI_OPERATING_RULES.md + LESSONS_LEARNED.md + هذه الخارطة.

---

## المبادئ غير القابلة للتفاوض

1. **5-step diagnosis** قبل أي تعديل (Observation → Expectation → Gap → Hypothesis → Proof).
2. **Backup قبل أي تعديل** (`cp <file> <file>.pre_<reason>_<timestamp>`).
3. **fix واحد في كل دورة** (لا تجميع، لا تخمين).
4. **Revert فوري عند الفشل** + تحديث LESSONS_LEARNED بالفشل.
5. **لا غش معايير**: لا تعديل threshold، لا كتم rejection، root-cause حصراً.
6. **ملف محظور = إذن صريح من المالك** قبل التعديل، مع توثيق كامل.
7. **لا ادعاء نجاح بدون أرقام** مُثبَتة من run.

---

## الفرضيات (مُرتَّبة بالاحتمال + الدليل)

| # | الفرضية | الدليل | احتمال |
|---|---|---|---|
| H1 | `simulator_mavlink` لا يُنشَر لـinstance 0 عند فراغها | Run #6 (Fix #4): native blocked → no sensors → preflight fail | عالٍ |
| H2 | EKF2 يتجاهل GPS (`gnss=0`) رغم وصول GPS صحيح | ARM-DIAG: fix=3 sats=12 لكن flags=`gnss=0` | عالٍ |
| H3 | MPC يُخرج commands صغيرة (de=±0.027) — saturation أو خطأ | Run #5 logcat: تكرار قيم صغيرة | متوسط |
| H4 | Failsafe يُفعَّل خلال 10ms من arm فيكتم MPC | Run #6: arm @03:25:49.366, failsafe @03:25:49.387 | عالٍ |
| H5 | Lockstep timeouts 46% يكسر MPC من 25Hz | LESSONS_LEARNED سابقاً | متوسط |

---

## المراحل

### قبل P1 — اختبار A/B (طلب المالك 2026-05-07 05:09)
- **A**: HIL run بدون EKF2 (skip ekf2 start في px4_jni).
- **B**: HIL run بـEKF2 default (الحالة الحالية).
- مخرَج: مقارنة Score / Max fin / Range / Failsafe بين A و B.
- بعدها: revert EKF2-skip لـحالة B.

### P1 — تشخيص (مُشخِّص فقط، read-only)
- D1: قراءة `simulator_mavlink.cpp` — instance allocation.
- D2: قراءة EKF2 GPS gating — لماذا `gnss=0`.
- D3: تتبُّع `actuator_outputs_sim` → mixer → `HIL_ACTUATOR_CONTROLS`.
- D4: مُحفِّزات failsafe في commander.
- D5: قياس lockstep rate الفعلي.
- مخرَج: تقرير `P1_DIAGNOSIS.md` + 5 hypotheses مُؤكَّدة بأرقام.
- **🛑 نقطة قرار**: أعرض على المالك. إذن fix واحد فقط.

### P2 — إصلاحات مُتسلسلة (مُصلِح + مُختبِر)
- **Loop لكل fix**:
  1. اقتراح: ملف، سطر، root-cause، كيفية القياس → 🛑 إذن المالك.
  2. backup → تعديل → clean build → reinstall.
  3. HIL run → استخراج score/max_fin/range.
  4. تحسّن مُثبَت (≥+5 score أو Max fin ثابت >0°)؟ → احتفظ + LESSONS.
  5. لا تحسّن أو تراجع؟ → revert فوري + LESSONS بالفشل.
- **حد أقصى 7 fixes**. بعدها: اعتراف بالعجز إن لم نصل ≥75.

#### ترتيب fixes (الأقل مخاطرة أولاً):
| # | المحتوى | الملف | المخاطرة |
|---|---|---|---|
| F1 | EKF2 GPS aiding params | param فقط | منخفضة |
| F2 | failsafe params | param فقط | منخفضة |
| F3 | bridge config | `mavlink_bridge_hil.py` | منخفضة |
| F4 | airframe config | `airframes_*.json/yaml` | متوسطة |
| F5 | native publishers | `android_uorb_publishers.cpp` | متوسطة |
| F6 | rocket_mpc logic | `RocketMPC.cpp` | عالية |
| F7 | ملف محظور (lib/, ekf2/, commander/, sensors/) | **إذن صريح + توثيق root-cause** | عالية |

### P3 — تثبيت (3 runs متتالية)
- شرط النجاح: 3 runs متتالية، كلّ منها ≥80/100 + Max fin >0° + Range ≥2330m + لا failsafe pre-launch.
- إن فشل واحد → عودة لـP2 أو إعلان impasse.

### P4 — توثيق وإغلاق
- تحديث `LESSONS_LEARNED.md` بكل bug + fix (نجح/فشل).
- تحديث `BASELINES.md` بـHITL baseline الجديد.
- كتابة `SESSION_2026-05-07_HITL_RECOVERY.md`.
- git tag: `baseline-hitl{score}-{date}`.
- اقتراح `LAUNCH_CHECKLIST.md` (لـpre-Real-Flight) — يحتاج توقيع المالك يدوياً.

---

## شروط الانتهاء

### ✅ نجاح حصري عند:
1. P3 مكتمل (3 runs متتالية ≥80).
2. كل التعديلات في git، tag جديد.
3. LESSONS + BASELINES + SESSION_REPORT مُحدَّثة.

### ❌ عجز (إعلان صريح بلا تجميل):
- استنفاد 7 fixes في P2 بدون score ≥75.
- root-cause في ملف محظور رفض المالك تعديله.
- مشكلة هاردوير (لا برمجية).
- في كل الحالات: تقرير شامل بالأدلة.

---

## ما خارج قدرة الـAI

- ضغط START على الهاتف (المالك يضغط).
- توصيل/فصل CAN adapter (المالك).
- توقيع Real Flight checklist (المالك).
- تعديل ملف محظور (إذن صريح للمالك مع root-cause موثَّق).

---

## السجلّ

| التاريخ | الحدث | المرجع |
|---|---|---|
| 2026-05-07 | كتابة الخارطة + بدء A/B test (no-EKF2 vs EKF2) | هذا الملف |
