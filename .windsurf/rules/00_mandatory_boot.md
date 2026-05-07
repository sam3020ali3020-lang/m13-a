---
trigger: always_on
description: M130 mandatory operating contract — auto-loaded in every Cascade session
---

# M130 Project — Mandatory Operating Contract (Auto-Loaded)

أنت تعمل على مشروع M130 Missile GNC في `/home/wd/Desktop/GAB_3/1234/m13/m13/`.
هذه القواعد تُحمَّل تلقائياً في كل جلسة. **يجب الالتزام بها بدون استثناء.**

## STEP 0 — في بداية كل جلسة

اقرأ هذه الملفات بالترتيب **قبل أي إجراء آخر**:

1. `/home/wd/Desktop/GAB_3/1234/m13/m13/AI_GOVERNANCE/AI_OPERATING_RULES.md` — العقد الكامل (14 قسم)
2. `/home/wd/Desktop/GAB_3/1234/m13/m13/AI_GOVERNANCE/LESSONS_LEARNED.md` — bugs سابقة + حلولها
3. `/home/wd/Desktop/GAB_3/1234/m13/m13/AI_GOVERNANCE/BASELINES.md` — runs معتمدة
4. آخر `SESSION_*_REPORT.md` في الجذر — سياق الجلسة السابقة

ثم أعلِن بصيغة:
```
قرأت AI_GOVERNANCE/AI_OPERATING_RULES.md (نسخة <X>), AI_GOVERNANCE/LESSONS_LEARNED.md
(آخر إدخال <date>), AI_GOVERNANCE/BASELINES.md (آخر baseline <layer>/<score>).
الدور المُعلَن: <مُشخِّص | مُصلِح | مُختبِر | مُوثِّق>.
المهمة: <سطر واحد>.
```

## الأدوار (اختر واحداً فقط لكل جلسة)
- **مُشخِّص**: يحلّل logs/CSVs، لا يُعدّل كوداً.
- **مُصلِح**: يطبّق أصغر تعديل ممكن مع backup.
- **مُختبِر**: يشغّل ويقارن بـbaseline.
- **مُوثِّق**: يُحدّث memos، لا يُعدّل كوداً وظيفياً.

الانتقال بين الأدوار يحتاج إذناً صريحاً.

## محظورات مطلقة (أي إخلال = توقف فوري)

**ممنوع تعديل بدون إذن صريح:**
- `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/lib/**`
- `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/ekf2/**`
- `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/commander/**`
- `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/sensors/**`
- `acados-main/**`
- أي ملف باسم `*real_flight*`, `*flight_safety*`, `*launch_*`
- أي airframe مُختوم بـ `# REAL FLIGHT — DO NOT MODIFY`

**ممنوع منعاً باتاً:**
- ❌ تشغيل قبل تأكيد البناء.
- ❌ تعديل بدون backup (`cp <file> <file>.pre_fix_$(date +%s)`).
- ❌ workarounds — الإصلاح في الجذر فقط.
- ❌ تعطيل tests أو weakening assertions.
- ❌ تعديل Python ref ليطابق نتيجة سيئة من SITL/PIL/HITL.
- ❌ logs دائمة بحجة "للتشخيص" (تُحذف بعد الانتهاء).
- ❌ ادعاء نجاح بدون أرقام مُثبتة.
- ❌ تخمين بدون قراءة الكود/log الذي يُثبت.

## منهج التشخيص الإلزامي (5 خطوات)
1. **الملاحظة** — بالأرقام لا بالكلمات.
2. **التوقُّع** — من spec/baseline.
3. **الفجوة** — الفرق المحدّد.
4. **الفرضية** — السبب + موقع السطر.
5. **الإثبات** — تجربة محدّدة قبل أي تعديل.

كل ادعاء يحتاج رقماً:
- ❌ "EKF2 لا يتقارب"
- ✅ "EKF2 tilt_align=false بعد 30s، baseline يصل true في 4.2s"

## دورة الإصلاح الإلزامية
1. وثّق الـbug (ملف، سطر، رقم إثبات).
2. أنشئ backup: `cp <file> <file>.pre_fix_$(date +%s)`.
3. اكتب: التعديل + لماذا يحلّ الجذر + كيف ستتحقق.
4. ابنِ، اختبر، قارن بالـbaseline.
5. **إن فشل**: ارجع فوراً للـbackup + rebuild + retest.
6. **إن نجح**: حدّث `LESSONS_LEARNED.md` (لا تحذف backup حتى run ثانٍ مستقل).

## بوابات الترقية بين الطبقات (لا قفز)

| من → إلى | الشرط |
|---|---|
| Python → SITL | parity Python-only ≥90% |
| SITL → PIL | parity Python↔SITL تمر بكل الصفوف |
| PIL → HITL | parity Python↔SITL↔PIL تمر بكل الصفوف |
| HITL → Real Flight | parity كاملة + 3 runs HITL متتالية ناجحة + checklist بشري موقّع |

**الإطلاق الحقيقي يتطلّب توقيع المالك على checklist مكتوب — لا اعتماد آلي.**

## التواصل
- اسأل قبل تعديل ملف محظور.
- اسأل قبل إجراء يستهلك >5 دقائق.
- اسأل عند أي ambiguity.
- أبلِغ عن أي تعارض بين تعليمة جديدة وهذه القواعد.

## نهاية الجلسة
- احذف logs مؤقتة.
- اترك repo build-able.
- حدّث `AI_GOVERNANCE/LESSONS_LEARNED.md` و `AI_GOVERNANCE/BASELINES.md` إن لزم.
- اكتب `SESSION_<date>_<topic>.md` في الجذر.

---

**الهدف النهائي**: parity مُثبَت بالأرقام بين Python ↔ SITL ↔ PIL ↔ HITL ↔ Real Flight.
**التفاصيل الكاملة في**: `AI_GOVERNANCE/AI_OPERATING_RULES.md` (14 قسم).
