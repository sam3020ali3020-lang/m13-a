# SESSION 2026-05-07 — HITL Lockstep Fix + CHANGELOG Deferral
**Role**: مُشخِّص → مُصلِح → مُختبِر → مُوثِّق
**Duration**: 09:00 — 10:44 (~1h 45min)
**Owner approval**: مُعطى صراحة ("اعمل ماشئت يا مهندس")

---

## 1. الهدف الأصلي (من الجلسة)
1. تشخيص "regression" المُلاحَظ بعد تطبيق 8 تَعديلات من CHANGELOG_M130
2. استرداد HITL إلى حالة عاملة
3. تَوثيق ما حَدث وما يَجب فعله

---

## 2. الإنجازات الرئيسية ✅

### A. اكتشاف bug صامت في `adb install -r`
- **العَرض**: تعديلات الكود لا تَأثر رغم rebuild + install ناجحَين
- **السبب الجذري**: `adb install -r` خَرج بـ 0 لكن APK لم يَتغيّر (signature/storage/session leak)
- **الإصلاح**: `pm uninstall` + `adb install` (بدون `-r`) + التَحقق من `dumpsys package | grep lastUpdateTime`
- **الأثر**: ساعات من تشخيص خاطئ تَجنُّبت لاحقاً

### B. اكتشاف أن الـ regression كان وَهماً
بعد revert كامل لـ 8 تَعديلات + clean install:
- **Run #1** (09:43): score 68.5، t=3.01s، Range 292m
- لكن **wall=3.1× sim** → غير مُستقر، **Run #2 أعطى score 25 + tumbling α=178°**
- النتيجة: **infrastructure problem في الأصل**، لا CHANGELOG bugs

### C. الـ Root cause الحقيقي = `lockstep=true`
- 100% timeouts في كل الـ runs (20ms و 50ms)
- PX4 يُرسل `HIL_ACTUATOR_CONTROLS` في bursts (TCP buffering / Nagle)
- bursts لا تَتناغم مع نوافذ steps → كل step يَهدر timeout كاملاً
- النتيجة: المُحاكاة 3-6× أبطأ من realtime، MPC على بيانات قديمة

### D. الإصلاح المُعتَمَد: `lockstep: false`
**ملف وحيد، سطر وحيد**: `6DOF_v4_pure/hil/hil_config.yaml:74`

| Metric | قبل | بعد |
|---|---|---|
| wall/sim | 3-6× ⛔ | **1.0×** ✓ |
| Peak Alt | 3-7m | **16-61m** |
| Range | 32-797m | **1112-1241m** |
| Tumbling | yes | **no** |
| Score | 25-68 | 28-66 |

### E. تَوثيق شامل
- ✅ `AI_GOVERNANCE/BASELINES.md`: Baseline جديد HITL (Run #5 = 66.5/100)
- ✅ `AI_GOVERNANCE/LESSONS_LEARNED.md`: 4 دروس جديدة:
  1. Lockstep timing pathology
  2. `adb install -r` failure mode
  3. `_debug_array_pub` causes MAVLink saturation
  4. **4 شُروط** لإعادة تَطبيق أيّ CHANGELOG item
- ✅ `CHANGELOG_M130.md`: شارة "حالة التطبيق — مُؤجَّل بالكامل" + جدول 25 شَرط للعودة

---

## 3. القرارات المُتّخَذة

### قرار 1: تَأجيل كامل لـ CHANGELOG #1-#15
- 6 تَعديلات **no-op في HITL** (EKF2 مُعطَّل بـ `ROCKET_USE_GT=1`)
- 2 تَعديل **مُؤكَّد ضارّ** (#10c CPU contention، #12.3 MAVLink saturation)
- لا واحد منها يُعالج المشكلة المُتبقّية (Range -52%، Peak Alt 61m)
- ⟹ **تأجيل بقرار مُراجع**، لا revert نهائي

### قرار 2: إبقاء `lockstep=false` كإصلاح وَحيد مُعتَمَد
- backup: `hil_config.yaml.pre_lockstep_fix_1778138419`
- موَثَّق في BASELINES.md و LESSONS_LEARNED.md

### قرار 3: 4 شُروط إلزامية قبل أيّ CHANGELOG item مُستقبلاً
1. سبب مَلموس بالأرقام
2. A/B test مُسجَّل
3. revert فوري إن ساء أيّ metric
4. single change at a time

---

## 4. التَعديلات النشطة في الكود الآن
- `hil_config.yaml:74`: `lockstep: false` ✅ مُعتَمَد
- لا تَعديلات أخرى من الجلسة (جميع backups من الجلسة الحالية أُعيدت)

## 5. Backups المحفوظة
```
hil_config.yaml.pre_lockstep_fix_1778138419   (قبل lockstep=false)
px4_jni.cpp.pre_round1_12.5_1778139280        (قبل #12.5، rolled back)
px4_jni.cpp.pre_changelog_apply_1778131533    (من بداية الجلسة)
```

---

## 6. المشكلة المُتبقّية (للجلسة القادمة)

**Range −52% (1241m vs target 2600m)، Peak Alt 61m فقط في 7s**

### فرضيات (لم يُختبَر أيّ منها بعد):
1. **Score variability**: Run #4=28.5، Run #5=66.5 — يَحتاج 5 runs لتحديد نطاق ثابت
2. **Trajectory مسطّح بطبيعته**: target عند نفس الارتفاع (1200m)، الصاروخ horizontal
3. **MPC gamma_ref يَنخفض**: من 0.26 إلى 0.10 خلال أول ثانية — مُلفِت
4. **Scoring criteria مُلائمة لـ SITL وليس HITL**: SITL score=89/100، HITL ≤ 70
5. **Battery/thermal**: لم يُختبَر طوال الجلسة بشكل مَنهجي

---

## 7. للجلسة القادمة — Action Items

1. **5 runs مُتتالية** بنفس الإعدادات، سَجّل score variability
2. **مُقارنة trajectory CSV** بين HITL و SITL (89/100) للجلسة المُؤرَّخة
3. **فحص MPC gamma_ref**: لماذا يَنخفض من 0.26 إلى 0.10؟
4. **مُراجعة scoring criteria** في `hil_analysis.py` — هل تَحاكي SITL؟
5. **توثيق reproducibility test** كـ baseline اختبار قبل أيّ تَغيير
6. **بعد استقرار baseline**: العودة إلى CHANGELOG حسب الـ 4 شُروط

---

## 8. ما يَجب أن يَبقى محفوظاً

- ✅ `lockstep=false` في `hil_config.yaml` — لا تَعكس
- ✅ Backups كلها — لا تَحذف حتى run ثاني مُستقل
- ✅ `LESSONS_LEARNED.md` — ادرسه قبل لمس CHANGELOG
- ✅ `BASELINES.md` — Run #5 = baseline HITL الحالي

---

## 9. Lessons من جانبي (AI) للمُراجَعة الذاتية

- **بدأت بـ 8 تَعديلات معاً → خطأ**. كان يَجب 1-by-1.
- **ضَيَّعت ساعات في تشخيص "regression" خاطئ** قبل اكتشاف `adb install -r` bug.
- **افترضت أن `lockstep=true` صحيح** — كان السبب الجذري الحقيقي.
- **لم أُحقّق من `lastUpdateTime`** بعد كل install — نظام إنذار غائب.
