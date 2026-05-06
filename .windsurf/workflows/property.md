---
description: تشغيل اختبارات property-based (Hypothesis) للتحقق من قوانين الإطار والرباعي
---

# Property-Based Tests

اختبارات تُثبت قوانين رياضية/فيزيائية باستخدام Hypothesis (آلاف المدخلات، حالات حرجة).

## المكوّنات

- `phone_to_frd` — isometry, involution, linearity, det=+1
- NED ↔ FUR — round-trip, orthogonality, norm preservation
- جبر الرباعي — associativity, conjugate, R(q) properties, SLERP, Euler round-trip

## خطوات التشغيل

1. تأكد من تثبيت التبعيّات:
// turbo
```bash
pip install --quiet hypothesis pytest
```

2. شغّل كل الاختبارات (افتراضي ~100 عيّنة لكل اختبار):
// turbo
```bash
python3 -m pytest /home/yoga/m13/m13/6DOF_v4_pure/tests/property/ -v
```

3. للتشغيل المكثّف في CI (1000 عيّنة/اختبار):
```bash
HYPOTHESIS_PROFILE=ci python3 -m pytest /home/yoga/m13/m13/6DOF_v4_pure/tests/property/ -v
```

4. لإعادة إنتاج فشل بعد تحديث البذرة:
```bash
python3 -m pytest /home/yoga/m13/m13/6DOF_v4_pure/tests/property/ --hypothesis-seed=<SEED>
```

## النتيجة المتوقَّعة

```
============================== 42 passed in ~6s ==============================
```

أي فشل يعني إما:
- bug حقيقي في دوال الرباعي/الإطار
- `phone_to_frd` في `AndroidApp/.../native_sensor_reader.cpp` تغيّر ولم تُحدَّث النسخة المرآة في `tests/property/test_frame_transforms.py`
