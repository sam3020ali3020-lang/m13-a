# PIL Bridge — مشكلة حساب التسارع (Specific Force) عند استخدام EKF2

## الملف المعني
`pil/mavlink_bridge_pil.py` — السطر 388

## الكود الحالي
```python
def _body_specific_force(f_body, mass, g_ned, quat):
    ...
    return f_body / max(mass, 0.1) - C @ g_ned
```

## المشكلة
`snapshot['forces']` من `rocket_6dof_sim.py` = **thrust + aero فقط** (بدون جاذبية — السطر 2045).

EKF2 يحسب التسارع الحقيقي كالتالي:
```
a_inertial = R_body_to_ned × a_imu + g_ned
```

مع الكود الحالي (`F/m - C·g`):
```
a_inertial = R × (F/m - C·g) + g = F_ned/m - g + g = F_ned/m    ← الجاذبية مفقودة!
```

المعادلة الصحيحة من 6DOF: `a_inertial = F_ned/m + g_ned`

**النتيجة**: EKF2 يفقد 9.81 m/s² في كل خطوة ← تقدير السرعة يتراكم خطأ ~9.5 m/s/ثانية ← MPC ينهار (NaN) بعد ~5 ثوانٍ.

## لماذا لا تظهر المشكلة حالياً؟
`ROCKET_USE_GT` الافتراضي = 1 → MPC يقرأ groundtruth → لا يمر عبر EKF2.
المشكلة تظهر **فقط** عند `ROCKET_USE_GT=0` (مسار EKF2 الكامل).

## الإصلاح المقترح

### 1. تعديل `_body_specific_force` (سطر 388):
```python
# قبل:
return f_body / max(mass, 0.1) - C @ g_ned

# بعد:
return f_body / max(mass, 0.1)
```

### 2. معالجة مرحلة المنصة (warm-up)
بعد الإصلاح، warm-up يرسل `forces=[0,0,0]` → accel = `[0,0,0]` (خطأ — EKF2 يحتاج `-g_body` لمحاذاة الميل).

يجب تعديل warm-up snapshot (حوالي سطر 893-894) لحقن قوة التثبيت:
```python
# قبل:
init_sensors = self._sensors(
    {"forces": [0, 0, 0], "vel_ned": [0, 0, 0]}, state
)

# بعد (نفس أسلوب SITL):
q0, q1, q2, q3 = state[6:10]
C_ned2b = np.array([
    [1-2*(q2*q2+q3*q3), 2*(q1*q2+q0*q3), 2*(q1*q3-q0*q2)],
    [2*(q1*q2-q0*q3), 1-2*(q1*q1+q3*q3), 2*(q2*q3+q0*q1)],
    [2*(q1*q3+q0*q2), 2*(q2*q3-q0*q1), 1-2*(q1*q1+q2*q2)],
])
pad_mass = state[13] if len(state) > 13 else 30.0
pad_forces = -pad_mass * (C_ned2b @ np.array([0, 0, 9.80665]))
init_sensors = self._sensors(
    {"forces": pad_forces, "vel_ned": [0, 0, 0]}, state
)
```

## كيف تتحقق
1. احفظ نسخة احتياطية: `cp mavlink_bridge_pil.py mavlink_bridge_pil.py.backup`
2. طبّق التعديلين أعلاه
3. شغّل PIL مع `ROCKET_USE_GT=0` في airframe HITL (22004)
4. إذا نجح → الإصلاح صحيح
5. إذا فشل → `cp mavlink_bridge_pil.py.backup mavlink_bridge_pil.py`

## المرجع
SITL bridge (`sitl/mavlink_bridge.py`) يستخدم `F/m` بالفعل ويسجّل **89.5/100** مع EKF2.
