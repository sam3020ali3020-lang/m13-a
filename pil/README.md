# PIL Test Runner

مُشغِّل PIL مستقل تماماً عن اختبار SITL.

يتصل بجهاز ARM64 خارجي يُشغّل PX4 في HITL mode عبر TCP MAVLink ويُصدِر CSV للرحلة والتوقيت. **لا توجد محاكاة مرجعية ولا مقارنة** داخل هذا المُشغِّل — إن احتجت المقارنة يمكن تشغيل `rocket_6dof_sim` منفصلاً ومقارنة CSV يدوياً.

## الملفات

```
pil/
├── mavlink_bridge_pil.py   ← جسر MAVLink (لا يستورد من sitl/)
├── pil_runner.py           ← يُطلق الجسر فقط
├── pil_config.yaml         ← إعدادات الهدف والتوقيت
└── results/
    ├── pil_flight.csv      ← من جهاز ARM64 الفعلي
    └── pil_timing.csv      ← إحصائيات توقيت من الهدف
```

## تشغيل

```bash
# 1) شغّل PX4 على الجهاز الهدف مع SYS_HITL=1 و`adb reverse tcp:4560 tcp:4560`
# 2) ثم على الـ PC:
cd 6DOF_v4_pure/pil
python pil_runner.py
# أو تحديد ملفات مخصّصة:
python pil_runner.py --config my_cfg.yaml --pil-csv out.csv --timing-csv t.csv
```

مجلد `results/` يُنشَأ تلقائياً (لا يُحفظ في الريپو).

## آلية warm-up

الجسر ينتظر أول `HIL_ACTUATOR_CONTROLS` كـ proxy لاستعداد PX4 (SYS_HITL مفعّل + EKF2 متقارب)، ثم ينتظر `settle_after_arm_s` إضافية قبل بدء حلقة الرحلة. إن لم يصل شيء خلال `warmup.duration_s` يُفشِل صراحةً مع تعليمات تشخيصية. لتعطيل الفحص استخدم `warmup.abort_on_no_actuator: false` في `pil_config.yaml`.

## متطلبات الهدف لقياس التوقيت

يجب أن ينشر `rocket_mpc` على الجهاز إحدى الطريقتين:

1. **`DEBUG_VECT`** باسم `"TIMING"` حيث `x=mhe_us`, `y=mpc_us`, `z=cycle_us`.
2. أو **`NAMED_VALUE_FLOAT`** بالأسماء: `mhe_us`, `mpc_us`, `cycle_us`.

إن لم يُنشر شيء، يُنشَأ `pil_timing.csv` فارغاً.
