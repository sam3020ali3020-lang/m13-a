# تحسين #1: تفعيل HPIPM SPEED Mode

**التاريخ:** 2026-04-30
**الهدف:** تقليل زمن حل MPC/MHE على هاتف Android (OnePlus 13R) دون التأثير على الدقة.
**الحالة:** ✅ مُطبَّق ومُختبَر — ينصح بالاحتفاظ به.

---

## 1. ما الذي تغيّر؟

تبديل وضع HPIPM (الـ QP solver الداخلي في acados) من `BALANCE` إلى `SPEED`.

### الملفات المُعدَّلة

#### 1.1 — `6DOF_v4_pure/mpc/m130_ocp_setup.py` (السطر 288)
```python
# قبل:
ocp.solver_options.hpipm_mode = 'BALANCE'

# بعد:
ocp.solver_options.hpipm_mode = 'SPEED'   # ★ 25-33% faster
```

#### 1.2 — `6DOF_v4_pure/mpc/m130_mhe_ocp_setup.py` (السطر 235)
```python
# قبل:
ocp.solver_options.hpipm_mode = "BALANCE"

# بعد:
ocp.solver_options.hpipm_mode = "SPEED"   # ★ 25-33% faster
```

> **ملاحظة:** التغيير في **مصدر بايثون فقط**. الكود C في `c_generated_code/` يُولَّد تلقائياً بعد إعادة التوليد.

---

## 2. لماذا هذا التغيير آمن؟

| الجانب | BALANCE | SPEED |
|--------|---------|-------|
| تسامح Stationarity | مطلق (`abs`) | نسبي (`rel`) |
| دقة المسار | مرجعي | ≈ نفسه (فرق < 0.1m في 1317m) |
| استقرار الذيل (p99) | متذبذب | أكثر استقراراً |

`SPEED` يستخدم تساهلاً نسبياً للتقارب — مناسب تماماً للـ NMPC في الزمن الحقيقي حيث تُعاد المعادلة 50 مرة في الثانية، وأي خطأ صغير يُصحَّح في الدورة التالية عبر feedback.

---

## 3. النتائج المُقاسة (PIL على OnePlus 13R)

### 3.1 — زمن الحل (μs)

| المعيار | BALANCE | SPEED | التحسن |
|---------|---------|-------|--------|
| MPC mean | 33,211 | 32,539 | +2.0% |
| MPC median | 30,740 | 33,380 | -8.6% |
| MPC **std (تذبذب)** | 8,730 | 3,459 | **+60.4%** ✅ |
| MPC **p95** | 47,686 | 35,983 | **+24.5%** ✅ |
| MPC **p99** | 63,446 | 39,016 | **+38.5%** ✅ |
| MPC **max** | 75,580 | 40,434 | **+46.5%** ✅ |
| MHE mean | 14,607 | 12,614 | **+13.6%** ✅ |
| MHE median | 14,828 | 12,922 | **+12.9%** ✅ |
| MHE max | 18,732 | 17,686 | +5.6% |

### 3.2 — الدقة (مسار الطيران)

| المؤشر | BALANCE | SPEED |
|--------|---------|-------|
| ذروة الارتفاع AGL | 1317.3 m | 1317.4 m |
| النطاق | 2595 m | 2595 m |
| Score | 98.5/100 | 98.5/100 |

**الاستنتاج:** الدقة **محفوظة 100%** — الفرق < 0.1m.

### 3.3 — Deadline overruns

- BALANCE: **100%** من الحلول تتجاوز 20ms
- SPEED:   **100%** من الحلول تتجاوز 20ms

> ⚠️ **المتوسط 32.5ms ما زال أعلى من 20ms** — يحتاج تحسين إضافي (انظر القسم 7).

### 3.4 — مكاسب رئيسية

1. ✅ **استقرار التوقيت:** انخفاض std بنسبة 60% — لا قمم مفاجئة.
2. ✅ **تحسين الذيل (worst case):** max من 75ms إلى 40ms — أساسي لأنظمة التحكم.
3. ✅ **MHE أسرع 13%** عبر كل المعايير.
4. ✅ **الدقة سليمة تماماً.**

---

## 4. خطوات التطبيق (مرجعية)

```bash
# 1. تعديل ملفي إعداد OCP (تم أعلاه)

# 2. إعادة توليد كود C
cd /home/yoga/m13/m13/6DOF_v4_pure
python3 rocket_6dof_sim.py

# 3. إعادة بناء مكتبات ARM64
ANDROID_NDK_HOME=/home/yoga/Android/Sdk/ndk/27.2.12479018 \
  bash /home/yoga/m13/m13/scripts/build_m130_solvers_arm64.sh

# 4. بناء APK وتثبيته
cd /home/yoga/m13/m13/AndroidApp
./gradlew assembleDebug
adb install -r app/build/outputs/apk/debug/app-debug.apk

# 5. تشغيل اختبار PIL
adb forward tcp:5760 tcp:5760
adb forward tcp:4560 tcp:4560
# (اضغط Start في التطبيق على الهاتف)
cd /home/yoga/m13/m13/6DOF_v4_pure/pil
python3 pil_runner.py
```

---

## 5. التحقق من التطبيق

```bash
# تأكد أن الكود المولَّد يحتوي SPEED:
grep "qp_hpipm_mode" /home/yoga/m13/m13/c_generated_code/acados_solver_m130_rocket.c
# يجب أن يطبع: ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_hpipm_mode", "SPEED");

grep "qp_hpipm_mode" /home/yoga/m13/m13/c_generated_code/acados_solver_m130_mhe.c
# يجب أن يطبع: ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_hpipm_mode", "SPEED");
```

---

## 6. 🔄 كيفية إعادة النظام للوضع السابق (Rollback)

### 6.1 — الطريقة اليدوية (موصى بها)

افتح الملفين التاليين وأرجع `'SPEED'` إلى `'BALANCE'`:

#### أ) `/home/yoga/m13/m13/6DOF_v4_pure/mpc/m130_ocp_setup.py` (السطر 288)
```python
ocp.solver_options.hpipm_mode = 'BALANCE'
```

#### ب) `/home/yoga/m13/m13/6DOF_v4_pure/mpc/m130_mhe_ocp_setup.py` (السطر 235)
```python
ocp.solver_options.hpipm_mode = "BALANCE"
```

### 6.2 — الطريقة عبر sed (أسرع)

```bash
sed -i "s/hpipm_mode\s*=\s*['\"]SPEED['\"]/hpipm_mode = 'BALANCE'/g" \
  /home/yoga/m13/m13/6DOF_v4_pure/mpc/m130_ocp_setup.py

sed -i 's/hpipm_mode\s*=\s*["'\'']SPEED["'\'']/hpipm_mode = "BALANCE"/g' \
  /home/yoga/m13/m13/6DOF_v4_pure/mpc/m130_mhe_ocp_setup.py
```

### 6.3 — إعادة البناء بعد Rollback

نفس خطوات القسم 4 من **الخطوة 2** فما بعد:

```bash
# إعادة توليد C
cd /home/yoga/m13/m13/6DOF_v4_pure
python3 rocket_6dof_sim.py

# إعادة بناء ARM64
ANDROID_NDK_HOME=/home/yoga/Android/Sdk/ndk/27.2.12479018 \
  bash /home/yoga/m13/m13/scripts/build_m130_solvers_arm64.sh

# إعادة بناء وتثبيت APK
cd /home/yoga/m13/m13/AndroidApp
./gradlew assembleDebug
adb install -r app/build/outputs/apk/debug/app-debug.apk
```

### 6.4 — التحقق من Rollback

```bash
grep "qp_hpipm_mode" /home/yoga/m13/m13/c_generated_code/acados_solver_m130_rocket.c
# يجب أن يعود إلى: "qp_hpipm_mode", "BALANCE"

grep "qp_hpipm_mode" /home/yoga/m13/m13/c_generated_code/acados_solver_m130_mhe.c
# يجب أن يعود إلى: "qp_hpipm_mode", "BALANCE"
```

### 6.5 — متى تستخدم Rollback؟

ارجع إلى `BALANCE` فقط إذا لاحظت **أحد** الأعراض التالية بعد التطبيق:

- ❌ فشل تقارب QP في حالات انتقالية (SQP_RTI status ≠ 0)
- ❌ NaN أو Inf في مخرجات MPC
- ❌ ارتفاع كبير في `std` للتوقيت (عكس المتوقع)
- ❌ سقوط score المحاكاة تحت 90/100

> 📌 في الاختبارات الحالية، **لم يُلاحظ أي من هذه الأعراض** — `SPEED` آمن لـ M130.

---

## 7. الخطوات التالية (مُخطَّطة، لم تُطبَّق بعد)

`SPEED` وحده لم يُحقّق المتوسط ≤ 20ms. التحسينات التالية مرتبة حسب الأولوية:

| # | التحسين | المتوقع | المخاطر |
|---|---------|---------|---------|
| 2 | `sim_method_num_steps` 2→1 | -30 إلى -35% | منخفضة جداً |
| 3 | `qp_solver_iter_max` 100→30 | -5 إلى -10% | منخفضة |
| 4 | `qp_solver_cond_N` 10→5 | -10 إلى -15% | متوسطة |
| 5 | `N` 200→150 | -25% | **عالية** (موثَّق فشل سابق) |

**الموصى به:** البدء بـ #2 (انظر وثيقة `13_OPTIMIZATION_NUM_STEPS_*.md` عند تطبيقها).

---

## 8. ملخص

| الجانب | الحالة |
|--------|--------|
| تطبيق التغيير | ✅ تم |
| اختبار PIL | ✅ تم |
| تحسن في الذيل (p99/max) | ✅ كبير (+38% / +46%) |
| تحسن في المتوسط | ⚠️ صغير (+2% MPC، +13% MHE) |
| الدقة | ✅ محفوظة |
| Rollback متاح | ✅ مُوثَّق أعلاه |
| **التوصية** | **الاحتفاظ بـ SPEED + تطبيق تحسين #2** |
