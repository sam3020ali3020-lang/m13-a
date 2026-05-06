# Differential Tests — مرجعان يتنافسان

اختبارات تقارن **تنفيذَين للمسألة نفسها**. أي انحراف = bug في أحدهما.

> "هذه تكشف انحرافات الترجمة Python↔C++ في MPC/MHE — أخطر مكان للانحراف الصامت."

## البنية

```
tests/differential/
├── conftest.py                         # sys.path + Hypothesis profiles
├── test_quaternion_vs_scipy.py         # Python quats ↔ scipy.Rotation
├── test_mpc_python_vs_cpp.py           # Python gtv.* ↔ ./validate (C++)
└── README.md                           # هذا الملف
```

## المرجعان في كل ملف

### `test_quaternion_vs_scipy.py` (10 اختبارات)

| دالة كودنا | المرجع | الاختبار |
|-----------|--------|---------|
| `quaternion_to_rotation_matrix(q)` | `Rotation.from_quat(q).as_matrix()` | `test_rotation_matrix_matches_scipy` |
| `R(q) @ v` | `Rotation.apply(v)` | `test_rotate_vector_matches_scipy` |
| `quaternion_multiply(q1, q2)` | `r1 * r2` | `test_quaternion_multiply_matches_scipy_composition` |
| `quaternion_conjugate(q)` | `Rotation.inv()` | `test_conjugate_matches_scipy_inverse_for_unit_quaternion` |
| `euler_to_quaternion(r, p, y)` | `Rotation.from_euler('xyz', [r, p, y])` | `test_euler_to_quaternion_matches_scipy` |
| `quaternion_to_euler(q)` | `Rotation.as_euler('xyz')` | `test_quaternion_to_euler_matches_scipy` |
| `quaternion_slerp(q1, q2, t)` | `Slerp([0, 1], [q1, q2])(t)` | `test_slerp_matches_scipy_slerp` |
| `normalize_quaternion(q)` | `q / ||q||` | `test_normalize_matches_manual_division` |

**اتفاقيات التحويل:**
- كودنا: scalar-first `[w, x, y, z]`
- SciPy: scalar-last `[x, y, z, w]`
- Euler: `'xyz'` extrinsic = `Rz(yaw) · Ry(pitch) · Rx(roll)` ⇔ كودنا

**اكتشاف قام به الاختبار نفسه:**
- `quaternion_slerp` عندنا يستخدم **NLERP** (linear+normalize) عند `|dot|>0.9995`.
- SciPy يستخدم **SLERP** الدقيق دائمًا.
- الفرق `O(1e-7)` للزوايا الصغيرة — مقبول، موثَّق في التعليقات.
- اختباران: أحدهما relaxed (1e-5) يقبل NLERP، والآخر strict (1e-9) للزوايا الكبيرة حيث كلاهما SLERP.

### `test_mpc_python_vs_cpp.py` (5 اختبارات)

| دالة Python | دالة C++ | الاختبار |
|-------------|----------|---------|
| `gtv.compute_params(...)` | `compute_params(...)` في `validate_cpp.cpp` | `test_compute_params_python_matches_cpp` ✅ |
| `gtv.compute_los(...)` | `compute_los(...)` | `test_compute_los_python_matches_cpp` ✅ |
| `gtv.compute_fin_mixing(...)` | `compute_fin_mixing(...)` | `test_compute_fin_mixing_python_matches_cpp` ✅ |
| `gtv.compute_weights(...)` | `compute_weights(...)` | `test_compute_weights_python_matches_cpp` ✅ |
| Fixed flight points | Fixed flight points | `test_all_fixed_flight_points_still_valid` ✅ |

**البروتوكول لكل @given:**
1. Hypothesis يولّد `flight_state` عشوائي (t, x_pos, gamma, ...)
2. Python يحسب `expected` باستدعاء `generate_test_vectors.py` functions
3. نكتب JSON مؤقّت مُطابق لـ schema `validate`
4. نشغّل `./validate tmp.json` كـ subprocess
5. نتحقّق أن stdout يحوي `PASS <name>` للحقول المطلوبة

**ملاحظة تاريخية:** في البداية ظهر فشل في `W[4]` أثناء boost (got=200, expected=488).
التشخيص كشف أن السبب **binary قديم** (`validate` مبني في Apr 28) من نسخة سابقة
من `validate_cpp.cpp`. إعادة البناء من المصدر الحالي ⇒ 9/9 PASS.
الدرس: **دائمًا أعد بناء `validate` قبل تشغيل differential tests** —
`run_validation.sh` يقوم بذلك تلقائيًا.

## التشغيل

```bash
cd 6DOF_v4_pure

# default (30 examples لـ MPC/C++, 100 لـ SciPy) ~6s
python3 -m pytest tests/differential/ -v

# فقط SciPy (سريع، 100 examples)
python3 -m pytest tests/differential/test_quaternion_vs_scipy.py -v

# فقط MPC/C++ (يحتاج ./validate مبني)
python3 -m pytest tests/differential/test_mpc_python_vs_cpp.py -v

# profile مخصص: 30 examples مضمون مع deadline=None
HYPOTHESIS_PROFILE=diff_cpp python3 -m pytest tests/differential/ -v
```

### بناء `validate` binary إن لم يكن موجودًا

```bash
cd 6DOF_v4_pure/validation
bash run_validation.sh                  # يبني ويشغّل الحالات الثابتة
# أو يدويًا:
g++ -std=c++17 -O2 -o validate validate_cpp.cpp -lm
```

الاختبارات ستُتجاوَز تلقائيًا (`pytest.mark.skipif`) إذا كان الـ binary مفقودًا.

## متى تضيف اختبار differential جديد؟

1. **عند كتابة دالة رياضية جديدة:** ابحث عن مرجع ذهبي (SciPy/NumPy) واكتب differential test فوراً.
2. **عند ترجمة Python → C++:** كل دالة في كلا اللغتَين يجب أن تُختبر مع بعضها.
3. **عند اكتشاف bug انحراف:** أضف اختبار يُثبت المساواة، حتى لا يعود الـ bug.

## لماذا differential > unit?

| Unit test | Differential test |
|-----------|-------------------|
| `assert f(3) == 7` — قد يكون `f` خطأ و 7 خطأ نفسه | `assert f(x) == g(x)` لآلاف `x` |
| تختبر _قيمة_ محددة | تختبر _تكافؤ_ مرجعين |
| يمرّ حتى لو نسخنا bug من C++ إلى Python | يفشل فوراً عند أي انحراف |
| يعتمد على ذاكرة المبرمج | يعتمد على مرجع موثوق خارجي |
