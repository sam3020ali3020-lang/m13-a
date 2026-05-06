# Property-Based Tests — اختبارات القوانين

اختبارات تُثبت **قوانين رياضية/فيزيائية** بدلاً من عيّنات محددة.
باستخدام [Hypothesis](https://hypothesis.readthedocs.io) نُولّد آلاف المدخلات
(بما فيها الحدود العائمة و NaN/Inf/ε) ونتحقق من بقاء القانون صحيحًا.

## البنية

```
tests/property/
├── conftest.py                       # يُضيف 6DOF_v4_pure إلى sys.path
├── strategies.py                     # Hypothesis strategies مشتركة
├── test_frame_transforms.py          # phone_to_frd + NED↔FUR
├── test_quaternion_properties.py     # جبر الرباعي الكامل
└── README.md                         # هذا الملف
```

## القوانين المُختبَرة

### `phone_to_frd` (13 اختبارًا)
مرجعه C++ في `AndroidApp/app/src/main/cpp/native_sensor_reader.cpp:34-41`.
النسخة Python هنا **مرآة bit-for-bit** — لو تغيّر C++ يجب مزامنتها.

- **Isometry** — يحافظ على norm² (القانون الأهم: المستشعر يقرأ نفس الإشارة)
- **Involution** — تطبيقه مرّتين = identity
- **Linearity** — `T(a·u + b·v) = a·T(u) + b·T(v)`
- **Dot product preservation** — الزوايا محفوظة
- **Proper rotation on cross** — `T(u×v) = T(u)×T(v)` ⇒ `det = +1`
- **Matrix ≡ scalar form** — الدالة C++ = ضرب مصفوفة الدوران

### NED ↔ FUR (4 اختبارات)
- Round-trip exact (`C · Cᵀ · v = v`)
- Norm preservation
- Matrix orthogonality + `det = +1`
- Dot product preservation

### جبر الرباعي (25 اختبارًا)

**Conjugate/Inverse:**
- `(q*)* = q`
- `|q*| = |q|`
- `q ⊗ q* = |q|²` (الجزء التخيّلي يُلغى — قانون التبرير)

**الضرب:**
- **Associative** — `(q1⊗q2)⊗q3 = q1⊗(q2⊗q3)`
- **Identity** — `1⊗q = q⊗1 = q`
- **Norm multiplicative** — `|q1⊗q2| = |q1|·|q2|`
- **Reversal law** — `(q1⊗q2)* = q2* ⊗ q1*`
- **Non-commutative when axes differ** — counter-check

**مصفوفة الدوران R(q):**
- Orthogonal (`R·Rᵀ = I`)
- `det(R) = +1`
- `R(q) = R(-q)` (double cover of SO(3))
- يحافظ على norm المتجهات + الضرب القياسي
- `R(q1⊗q2) = R(q1)·R(q2)` (تركيب = ضرب)
- `R(q*) = R(q)ᵀ`

**Normalization:**
- `|normalize(q)| = 1`
- `normalize(normalize(q)) = normalize(q)` (idempotent)
- Scale invariance للعوامل الموجبة
- Degenerate (norm < threshold) → `DegenerateQuaternionError`

**SLERP:**
- Endpoints: `slerp(q1, q2, 0) = q1` و `slerp(q1, q2, 1) = q2`
- Unit quaternion طوال الطريق
- Fixed point: `slerp(q, q, t) = q`

**Euler ↔ quaternion:**
- Round-trip بعيدًا عن gimbal lock
- `euler_to_quaternion` يُنتج unit quaternion

**Edge cases:**
- NaN/Inf/zero quaternion → `DegenerateQuaternionError`

## التشغيل

```bash
cd 6DOF_v4_pure
python3 -m pytest tests/property/ -v
```

### زيادة عدد العيّنات (CI mode)

```bash
# افتراضيًا: ~100 عيّنة/اختبار
# 1000 عيّنة/اختبار (أطول، كشف أعمق)
python3 -m pytest tests/property/ --hypothesis-profile=ci
```

لتفعيل profile `ci`، أضف في `conftest.py`:
```python
from hypothesis import settings
settings.register_profile("ci", max_examples=1000, deadline=None)
```

### تشغيل اختبار واحد

```bash
python3 -m pytest tests/property/test_frame_transforms.py::test_phone_to_frd_preserves_norm -v
```

### إعادة إنتاج فشل (Hypothesis يحفظ البذور)

```bash
python3 -m pytest tests/property/ --hypothesis-seed=<seed>
```

## متى تضيف اختبارًا جديدًا؟

- عند إضافة دالة رياضية/هندسية جديدة (دوران، تحويل إطار، تصفية)
- عند **اكتشاف bug** — أضف property يُعبّر عن الشرط المكسور
- عند تعديل `phone_to_frd` في C++ → حدّث النسخة المرآة هنا

## لماذا property-based؟

- يكشف حالات حدّية (NaN، subnormal، overflow) لن يفكّر بها مبرمج عيّنات.
- يُثبت **قانونًا** (ينطبق دائمًا) بدلاً من **حالة** (قد تمر بالصدفة).
- مثال: `assert phone_to_frd(1,2,3) == (2,1,-3)` لا يكشف لو كتب المبرمج
  `nz = z` بدلاً من `-z` طالما أن الحالة المُختبَرة تستخدم `z=0`.
  بينما `test_phone_to_frd_preserves_norm` يكشف هذا فورًا لأي `z≠0`.
