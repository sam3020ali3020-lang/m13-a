---
description: تشغيل اختبارات differential (Python vs SciPy، Python vs C++ MPC)
---

# Differential Tests

اختبارات تُقارن مرجعَين يحلاّن نفس المسألة. أي انحراف = bug في الترجمة.

## المكوّنات

- **`test_quaternion_vs_scipy.py`** — دوال الرباعي في `quaternion_utils.py` ضد `scipy.Rotation`
- **`test_mpc_python_vs_cpp.py`** — دوال MPC في `generate_test_vectors.py` ضد `./validate` binary (C++)

## خطوات التشغيل

1. تثبيت التبعيّات:
// turbo
```bash
pip install --quiet hypothesis pytest scipy
```

2. بناء C++ validator (إن لم يكن مبنيًّا):
// turbo
```bash
cd /home/yoga/m13/m13/6DOF_v4_pure/validation && g++ -std=c++17 -O2 -o validate validate_cpp.cpp -lm
```

3. تشغيل كل الـ differential tests:
// turbo
```bash
python3 -m pytest /home/yoga/m13/m13/6DOF_v4_pure/tests/differential/ -v
```

4. SciPy فقط (سريع):
```bash
python3 -m pytest /home/yoga/m13/m13/6DOF_v4_pure/tests/differential/test_quaternion_vs_scipy.py -v
```

5. MPC/C++ فقط (أبطأ لأنه subprocess لكل مثال):
```bash
python3 -m pytest /home/yoga/m13/m13/6DOF_v4_pure/tests/differential/test_mpc_python_vs_cpp.py -v
```

## النتيجة المتوقَّعة

```
test_quaternion_vs_scipy.py ..........                   [10 passed]
test_mpc_python_vs_cpp.py   ....x.                       [4 passed, 1 xfailed]
================= 14 passed, 1 xfailed in ~2s =================
```

## التعامل مع الفشل

- **SciPy test فشل:** انحراف في `dynamics/quaternion_utils.py` عن الرياضيات القياسية.
- **MPC/C++ test فشل (مختلف عن xfail المعروف):** انحراف بين `generate_test_vectors.py` و `validate_cpp.cpp` — bug ترجمة خفي.
- **`test_compute_weights_python_matches_cpp` فشل كـ XPASS:** تم إصلاح الـ bug الموثَّق — أزل `@pytest.mark.xfail` وحوّله إلى اختبار عادي.
