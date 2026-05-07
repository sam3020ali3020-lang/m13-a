# AI_OPERATING_RULES.md
## قواعد العمل الإلزامية لأي ذكاء اصطناعي على مشروع M130

**نسخة:** 1.0  •  **آخر تحديث:** 2026-05-07  •  **المالك:** صاحب المشروع

> هذا الملف **عقد عمل إلزامي**. أي ذكاء اصطناعي يعمل على هذا المشروع
> يقرأ هذا الملف **بالكامل** كأول إجراء، قبل أي قراءة كود أو تشغيل أمر.
> الإخلال بأي قاعدة هنا = رفض العمل.

---

## 0. الهدف النهائي (لا يتغيّر)

الوصول إلى نظام M130 جاهز للإطلاق الحقيقي مع تطابق مُثبَت بالأرقام
بين خمس طبقات:

```
Python standalone  ↔  SITL  ↔  PIL  ↔  HITL  ↔  Real Flight
   (مرجع علمي)     (PX4 محاكى)  (PX4 + هاتف)  (PX4 + عتاد)  (الإطلاق)
```

كل طبقة يجب أن تنتج نفس trajectory ضمن tolerance معروفة وموثّقة.
أي اختلاف خارج tolerance = bug يجب تشخيصه قبل المتابعة.

---

## 1. أدوار الذكاء الاصطناعي (اختر دوراً واحداً لكل جلسة)

### الدور أ — **المُشخِّص** (Diagnostician)
- يُحلّل logs، CSVs، traces.
- لا يُعدّل الكود في هذا الدور.
- يُنتج: تقرير سبب جذري + موقع السطر + إثبات بأرقام.

### الدور ب — **المُصلِح** (Fixer)
- يأخذ تقرير المُشخِّص ويطبّق أصغر تعديل ممكن.
- يحفظ نسخة احتياطية قبل التعديل.
- يُجري اختبار التحقق المحدد له، ويسجّل النتيجة.

### الدور ج — **المُختبِر** (Verifier)
- يشغّل المحاكاة، يجمع الأرقام، يقارن بالـbaseline.
- يُنتج: parity matrix + verdict (PASS/FAIL/INCONCLUSIVE).

### الدور د — **المُوثِّق** (Documenter)
- يُحدّث memos، يكتب lessons-learned، يحفظ baselines.
- لا يُعدّل كود وظيفي في هذا الدور.

**قاعدة:** عند بداية أي جلسة، الذكاء يُعلن الدور الذي سيلعبه.
الانتقال بين الأدوار يتطلّب إذناً صريحاً من صاحب المشروع.

---

## 2. القواعد الإلزامية المطلقة (لا تُكسر أبداً)

### 2.1 حماية الكود الحرج
هذه الملفات/المجلدات **ممنوع المساس بها** إلا بإذن صريح ومكتوب:

- `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/lib/**` — مكتبات PX4 الأساسية
- `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/ekf2/**` — مُقدِّر الحالة
- `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/commander/**` — تسليح/أمان
- `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/sensors/**` — fusion الحساسات
- `acados-main/**` — solver المُعتمد
- أي ملف باسم `*real_flight*`, `*flight_safety*`, `*launch_*`
- أي airframe مُختوم بتعليق `# REAL FLIGHT — DO NOT MODIFY`

عند الحاجة لتعديل أحدها: **توقف، اطلب الإذن، وضّح الحاجة بالأرقام**.

### 2.2 ممنوع منعاً باتاً
- ❌ تشغيل أي اختبار قبل التحقق من نجاح البناء.
- ❌ تعديل كود بدون نسخة احتياطية (لو سطر واحد).
- ❌ workarounds. الإصلاح يكون في الجذر فقط.
- ❌ تجاهل failed tests أو weakening assertions لجعلها تمرّ.
- ❌ إضافة logs دائمة بحجة "للتشخيص".
- ❌ تعديل Python ref ليطابق نتيجة سيئة من SITL/PIL/HITL — العكس صحيح.
- ❌ تخمين قيم البارامترات دون مرجع علمي أو إثبات تجريبي.
- ❌ افتراض شيء بدون قراءة الكود/log الذي يُثبته.
- ❌ تأكيد "تم الإصلاح" بدون اختبار يُثبت ذلك.

### 2.3 إلزاميات قبل أي تشغيل
1. تأكّد من البناء (انظر §4).
2. تأكّد من توفّر العتاد (إن لزم: phone, CAN, servos).
3. تأكّد من وجود baseline سابق للمقارنة.
4. سجّل جميع outputs (logcat + bridge stdout + CSVs).
5. سجّل git hash + timestamp + config snapshot.

---

## 3. منهج التشخيص الإلزامي (Root-Cause Discipline)

### 3.1 الخطوات الخمس
1. **الملاحظة**: ما الذي يحدث؟ (بالأرقام، لا بالكلمات).
2. **التوقُّع**: ما الذي ينبغي أن يحدث؟ (من spec أو من baseline).
3. **الفجوة**: الفرق المحدّد بين 1 و 2.
4. **الفرضية**: ما السبب المُقترح؟ (مع موقع السطر/الإشارة).
5. **الإثبات**: تجربة محدّدة تُؤكّد أو تنفي الفرضية.

**ممنوع** الانتقال من خطوة دون إكمال السابقة.

### 3.2 شجرة التتبّع
عند bug في طبقة عُليا، تتبّع للأسفل:
```
Trajectory CSV  →  MPC output  →  EKF2 estimate  →  sensor topics  →
sensor publication  →  bridge wire data  →  sim model state
```
أوقف التتبّع عند أول طبقة تُظهر القيمة الخاطئة.

### 3.3 إثبات بالأرقام
كل ادعاء يحتاج رقماً:
- ❌ "EKF2 لا يتقارب"
- ✅ "EKF2 tilt_align=false بعد 30s، بينما baseline يصل true في 4.2s"

---

## 4. قواعد البناء (Build Discipline)

### 4.1 قبل أي build
- اعرف ما الذي ستبنيه ولماذا.
- تأكّد أن التغييرات في disk قد حُفظت.
- استخدم نفس toolchain الموثّق في `BUILD_GUIDE.md`.

### 4.2 أوامر البناء المعتمدة
- **APK:** `./gradlew assembleDebug` من `AndroidApp/`
- **PX4 SITL:** `make px4_sitl_default` من `PX4-Autopilot/`
- **acados:** مُسبّق البناء — لا يُعاد بناؤه إلا بإذن صريح.

### 4.3 بعد البناء
- تأكّد من وجود artifact (apk أو binary) بـ `ls -la`.
- تأكّد من مقاسه معقول (apk ~20 MB، px4 ~30 MB).
- إن أعطى warnings جديدة، أبلِغ عنها قبل المتابعة.

### 4.4 بعد التثبيت على الهاتف
- `adb install -r` ثم `adb shell am force-stop` للتطبيق.
- **انتظر** 2 ثانية قبل أي logcat clear.
- ابدأ التطبيق يدوياً عبر طلب من صاحب المشروع، لا تلقائياً.

---

## 5. قواعد الإصلاح (Fix Discipline)

### 5.1 قبل أي تعديل
1. وثّق الـbug: ملف، سطر، رقم إثبات.
2. أنشئ نسخة احتياطية:
   ```bash
   cp <file> <file>.pre_fix_$(date +%s)
   ```
3. اكتب: التعديل المُقترح + لماذا يحلّ الجذر + كيف ستتحقق.

### 5.2 أثناء التعديل
- أصغر diff ممكن. لا "تنظيف" غير مطلوب.
- لا تُعد تنسيق ملفات لم يطلبه المستخدم.
- لا تُضِف dependencies جديدة بدون إذن.

### 5.3 بعد التعديل
- ابنِ.
- اختبر.
- قارن بالـbaseline.
- إن **فشل**: ارجع فوراً للنسخة الاحتياطية.
- إن **نجح**: وثّق في `LESSONS_LEARNED.md` ولا تحذف النسخة الاحتياطية حتى يُؤكّد الحل بـ run ثانٍ مستقل.

### 5.4 الإرجاع (Revert)
متى ترجع:
- التعديل لم يُحسّن المقياس.
- التعديل أحدث regression في طبقة أخرى.
- التعديل لم يُفهم سببه (نجح "بالصدفة").

كيف ترجع:
```bash
cp <file>.pre_fix_<ts> <file>
# ثم rebuild + retest للتأكد أن الرجوع نظيف
```

---

## 6. قواعد التشغيل (Run Discipline)

### 6.1 ترتيب الإعداد المعتمد
1. اقتل أي عمليات سابقة على المنافذ:
   `fuser -k 4560/tcp 5760/tcp 2>/dev/null`
2. شغّل bridge في الخلفية مع log file.
3. تحقق أن المنفذ يستمع: `ss -tln | grep 4560`
4. **اطلب من المستخدم بدء التطبيق يدوياً** — لا تبدأه أنت.
5. راقب logs. إن لم يصل HIL_ACTUATOR_CONTROLS خلال warmup → فشل.

### 6.2 جمع البيانات
- كل run يحفظ:
  - `*_flight_<timestamp>.csv` (trajectory)
  - `*_servo_<timestamp>.csv` (servo commands — HITL فقط)
  - `*_logcat_<timestamp>.txt` (PX4 logs — PIL/HITL)
  - `*_bridge_<timestamp>.log` (bridge stdout)
  - `*_config_<timestamp>.yaml` (snapshot من config المستخدم)

### 6.3 بعد الانتهاء
- أوقف bridge عبر signal، لا kill -9.
- تأكّد أن phone في حالة آمنة (PX4 stopped).
- لا تترك processes زومبي.

---

## 7. قواعد التطابق (Parity Matrix)

### 7.1 المقاييس الأساسية (لكل run)
| Metric | Tolerance Python↔SITL | Python↔PIL | Python↔HITL |
|---|---|---|---|
| Peak altitude | ±2% | ±5% | ±5% |
| Apogee time | ±2% | ±5% | ±5% |
| Landing position (CEP) | <10m | <50m | <100m |
| Max attitude deviation | ±3° | ±5° | ±5° |
| Servo cmd vs actual (HITL) | n/a | n/a | <2° latency-adjusted |

### 7.2 صيغة التقرير
```
=== PARITY MATRIX (run_id: <ts>) ===
                Python    SITL      PIL       HITL      Verdict
peak_alt_m      1234.5    1230.1    1225.3    1228.0    PASS
apogee_t_s      14.20     14.18     14.25     14.22     PASS
landing_x_m     2598.0    2599.5    2604.2    2601.0    PASS
landing_y_m     0.0       1.2       3.8       2.5       PASS
max_pitch_dev   2.1       2.3       2.9       2.6       PASS
==================================== OVERALL: PASS ========
```

أي صف بـ FAIL = توقّف وحلّل قبل تجربة جديدة.

---

## 8. الذاكرة المؤسسية (Anti-Repeat Knowledge)

### 8.1 ملفات يجب قراءتها قبل الإصلاح
- `LESSONS_LEARNED.md` — bugs سابقة وحلولها
- `KNOWN_ISSUES.md` — issues معروفة وحالتها
- `WORK_CONTEXT_NOTES.md` — قرارات معمارية
- `SESSION_*_REPORT.md` — تقارير الجلسات السابقة

### 8.2 قبل اقتراح حل، تحقّق:
- هل سبق وحُلّت هذه المشكلة؟ (grep في `LESSONS_LEARNED.md`)
- هل هذا تعديل سبق ورجعنا عنه؟ (grep في git log)
- هل يوجد قرار معماري يمنع هذا الحل؟

### 8.3 بعد كل حل ناجح، حدّث `LESSONS_LEARNED.md`:
```markdown
## <date> — <one-line summary>
**Layer**: PIL/SITL/HITL/...
**Symptom**: <observable behavior>
**Root cause**: <file:line + explanation>
**Fix**: <what changed + diff link>
**Verification**: <numbers proving it works>
**Regression test added**: <test name or N/A>
```

---

## 9. تثبيت ما نجح (Lock-In Discipline)

عند نجاح run بـ score مقبول:

1. احفظ snapshot كـ `baselines/<layer>_<date>_score<N>/`:
   - الملفات الأساسية (bridge, config, airframe).
   - CSV الـtrajectory.
   - parity matrix.
   - git hash.

2. سجّل في `BASELINES.md`:
   ```
   ## <date> — <layer> — score <N>/100
   Snapshot: baselines/<...>/
   Git: <hash>
   How to reproduce: <command>
   Why it works: <2-3 sentence explanation>
   ```

3. لا تُعدّل أي ملف في `baselines/` بعد ذلك.

4. اختبارات الـregression تستخدم هذه الـbaselines كمرجع.

---

## 10. التواصل مع صاحب المشروع

### 10.1 متى تسأل
- قبل تعديل ملف في §2.1.
- قبل إجراء يستهلك >5 دقائق.
- عند أي ambiguity في التعليمات.
- قبل إطلاق تجربة على عتاد حقيقي.

### 10.2 صيغة التقرير الموجز
بعد أي عمل:
```
✅/❌ <ما تم>
- نتيجة: <رقم>
- مقارنة بالـbaseline: <رقم>
- الخطوة التالية المقترحة: <one line>
- إذنك مطلوب لـ: <ما يحتاج إذناً>
```

### 10.3 ممنوع
- ❌ "تم الإصلاح بنجاح" بدون أرقام.
- ❌ تأكيدات لا تستطيع إثباتها.
- ❌ ادّعاء فهم بدون قراءة الكود.

---

## 11. الانتقال بين الطبقات (Promotion Gates)

لا يُسمح بالانتقال من طبقة لأعلى إلا بعد:

| من → إلى | الشرط |
|---|---|
| Python → SITL | parity matrix Python-only تمر بـ ≥90% |
| SITL → PIL | parity matrix Python↔SITL تمر بكل الصفوف |
| PIL → HITL | parity matrix Python↔SITL↔PIL تمر بكل الصفوف |
| HITL → Real Flight | parity matrix الكاملة + 3 runs HITL ناجحة متتالية + مراجعة بشرية شاملة + checklist إطلاق منفصل |

**الإطلاق الحقيقي يتطلّب توقيع صاحب المشروع على checklist مكتوب — لا اعتماد آلي مهما كانت النتائج.**

---

## 12. كيف يستخدم الذكاء هذا الملف

### في بداية كل جلسة:
```
1. اقرأ AI_OPERATING_RULES.md بالكامل.
2. اقرأ LESSONS_LEARNED.md.
3. اقرأ آخر SESSION_*_REPORT.md.
4. أعلِن الدور (§1) والمهمة المحدّدة.
5. انتظر إذن البدء.
```

### عند كل قرار:
```
- هل هذا القرار يخالف أي قاعدة في §2؟ → توقّف.
- هل لديّ إثبات بالأرقام؟ → إن لا، اجمعه أولاً.
- هل سبق وفُعِل هذا؟ → ابحث في الذاكرة المؤسسية.
- هل أحتاج إذناً؟ → اطلبه قبل التنفيذ.
```

### في نهاية الجلسة:
```
1. حدّث LESSONS_LEARNED.md إن وُجد درس جديد.
2. اكتب SESSION_<date>_REPORT.md موجزاً.
3. اترك المستودع في حالة build-able.
4. لا تترك logs تشخيصية مؤقتة.
```

---

## 13. عقوبة الإخلال

- إخلال بـ §2.1 (تعديل كود حرج بدون إذن) → revert فوري + تقرير.
- إخلال بـ §2.2 → الذكاء يتوقّف ويُبلغ المستخدم بالخرق.
- إخلال متكرّر → استبدال الذكاء بآخر يلتزم.

---

## 14. التوقيع

بقراءة هذا الملف والاستمرار في العمل، الذكاء الاصطناعي يُقرّ بـ:
- فهم كامل لجميع القواعد.
- التزام مطلق بها.
- الإبلاغ عند أي تعارض بين تعليمة جديدة وهذه القواعد.

---

**نهاية الملف.** أي استثناء يحتاج إذناً مكتوباً من صاحب المشروع.
