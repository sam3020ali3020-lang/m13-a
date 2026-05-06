# PIL Critical Patch — EKF2/IMU Parameter Tuning

> **التعديل الوحيد الذي حقّق القفزة من فشل → 82/100 في PIL**.
> كل التعديل في ملف واحد فقط: `AndroidApp/app/src/main/cpp/px4_jni.cpp`.

---

## السبب الجذري

كان الـPX4 على الهاتف يشغّل:
- **EKF2 prediction @ 200 Hz**
- **IMU integration @ 200 Hz**
- **Inner-loop gyro publication @ 400 Hz**

بينما SITL على الكمبيوتر يشغّلهم على **100 Hz**. النتيجة:
- الهاتف يستهلك ضعف CPU في EKF2/IMU
- MPC لا يحصل على وقت كافٍ → over-deadline > 95%
- Lockstep يفشل → "Connection reset by peer"

**الحل**: مطابقة معدلات الهاتف مع SITL (100 Hz).

---

## الملف المعدّل

`AndroidApp/app/src/main/cpp/px4_jni.cpp` — داخل block الـHITL initialization (تقريباً سطر 600–770).

---

## الـPatch الكامل (Copy-Paste Ready)

استبدِل هذا البلوك بالكامل في `px4_jni.cpp`:

### قبل ⛔
```cpp
// EKF2 sensor noise
p = param_find("EKF2_ACC_NOISE");
if (p != PARAM_INVALID) { float v = 3.0f; param_set(p, &v); }

p = param_find("EKF2_GPS_V_NOISE");
if (p != PARAM_INVALID) { float v = 1.5f; param_set(p, &v); }

p = param_find("EKF2_GPS_P_NOISE");
if (p != PARAM_INVALID) { float v = 2.0f; param_set(p, &v); }

p = param_find("EKF2_GPS_DELAY");
if (p != PARAM_INVALID) { float v = 200.0f; param_set(p, &v); }

p = param_find("EKF2_ABL_LIM");
if (p != PARAM_INVALID) { float v = 1.0f; param_set(p, &v); }

p = param_find("EKF2_GPS_V_GATE");
if (p != PARAM_INVALID) { float v = 10.0f; param_set(p, &v); }

p = param_find("EKF2_PREDICT_US");
if (p != PARAM_INVALID) { int32_t v = 5000; param_set(p, &v); }

// (الـ params التالية لم تكن موجودة)
```

### بعد ✅
```cpp
// =========================================================================
// EKF2/IMU UNIFIED TUNING — matches SITL airframe 22003 (verified 91/100)
// Critical fix: phone CPU bottleneck due to EKF2@200Hz + IMU@400Hz
// =========================================================================

// --- EKF2 sensor noise (matches bridge SensorNoise std values) ---
p = param_find("EKF2_ACC_NOISE");
if (p != PARAM_INVALID) { float v = 2.0f; param_set(p, &v); }    // 3.0 → 2.0

p = param_find("EKF2_GPS_V_NOISE");
if (p != PARAM_INVALID) { float v = 0.5f; param_set(p, &v); }    // 1.5 → 0.5 (bridge gps_vel_std=0.1)

p = param_find("EKF2_GPS_P_NOISE");
if (p != PARAM_INVALID) { float v = 2.5f; param_set(p, &v); }    // 2.0 → 2.5 (bridge gps_pos_std=2.5)

p = param_find("EKF2_GPS_DELAY");
if (p != PARAM_INVALID) { float v = 100.0f; param_set(p, &v); }  // 200 → 100 ms (bridge gps_delay_ms=100)

p = param_find("EKF2_ABL_LIM");
if (p != PARAM_INVALID) { float v = 0.8f; param_set(p, &v); }    // 1.0 → 0.8 (PX4 max bound)

// --- High-G burn tolerance ---
p = param_find("EKF2_GPS_V_GATE");
if (p != PARAM_INVALID) { float v = 50.0f; param_set(p, &v); }   // 10 → 50 σ (7G acceleration tolerance)

// =========================================================================
// THE CRITICAL FIX — CPU rate budget (200Hz → 100Hz)
// =========================================================================

// EKF2 prediction rate: 100 Hz instead of 200 Hz → halves EKF2 CPU.
p = param_find("EKF2_PREDICT_US");
if (p != PARAM_INVALID) { int32_t v = 10000; param_set(p, &v); } // 5000 → 10000 µs

// IMU integration must match EKF2 rate (otherwise frames are lost/duplicated).
p = param_find("IMU_INTEG_RATE");
if (p != PARAM_INVALID) { int32_t v = 100; param_set(p, &v); }   // NEW = 100 Hz

// Inner-loop gyro publication rate (was implicitly 400 Hz).
p = param_find("IMU_GYRO_RATEMAX");
if (p != PARAM_INVALID) { int32_t v = 100; param_set(p, &v); }   // NEW = 100 Hz

// =========================================================================
// Warmup convergence helpers
// =========================================================================

// Faster tilt alignment during warmup (rocket on stationary launch rail).
p = param_find("EKF2_ANGERR_INIT");
if (p != PARAM_INVALID) { float v = 0.01f; param_set(p, &v); }   // NEW = 0.01 rad

// Heading fusion gates for MAG_TYPE=6 (init-only at ARM, no continuous mag fusion).
p = param_find("EKF2_HDG_GATE");
if (p != PARAM_INVALID) { float v = 10.0f; param_set(p, &v); }   // NEW = 10 σ

p = param_find("EKF2_HEAD_NOISE");
if (p != PARAM_INVALID) { float v = 0.7f; param_set(p, &v); }    // NEW = 0.7 rad
```

---

## جدول القيم النهائية

| Param | قبل | بعد | الأثر |
|---|---|---|---|
| `EKF2_PREDICT_US` | 5000 (200Hz) | **10000 (100Hz)** | ⭐ خفض CPU بالنصف |
| `IMU_INTEG_RATE` | 200 | **100** | ⭐ مطابق لـEKF2 |
| `IMU_GYRO_RATEMAX` | 400 | **100** | ⭐ إلغاء loop زائد |
| `EKF2_GPS_V_GATE` | 10 | **50** | بوابة GPS أثناء 7G |
| `EKF2_ANGERR_INIT` | 0.1 | **0.01** | tilt_align أسرع 10x |
| `EKF2_HDG_GATE` | 2.6 | **10.0** | heading fusion |
| `EKF2_HEAD_NOISE` | 0.3 | **0.7** | mag init noise |
| `EKF2_GPS_V_NOISE` | 1.5 | **0.5** | بـ bridge std |
| `EKF2_GPS_P_NOISE` | 2.0 | **2.5** | بـ bridge std |
| `EKF2_GPS_DELAY` | 200 | **100** | بـ bridge delay |
| `EKF2_ACC_NOISE` | 3.0 | **2.0** | تخفيف ACC |
| `EKF2_ABL_LIM` | 1.0 | **0.8** | PX4 max |

---

## خطوات التطبيق

```bash
# 1) عدّل px4_jni.cpp بحسب الـpatch أعلاه

# 2) ابنِ APK
cd <repo>/AndroidApp
JAVA_HOME=<jdk17_path> PATH=<jdk17_path>/bin:$PATH ./gradlew assembleDebug

# 3) ثبّت
~/Android/Sdk/platform-tools/adb install -r app/build/outputs/apk/debug/app-debug.apk

# 4) شغّل PIL — يجب أن تحصل على ~82/100
cd <repo>/6DOF_v4_pure/pil
python3 -u pil_runner.py
```

---

## النتيجة المتوقّعة

| المقياس | القيمة |
|---|---|
| Score | **~82/100** ✅ |
| Range | ~2461m (target 2600m) |
| MPC avg | ~24 ms |
| Lockstep ratio | ~1.03x (شبه real-time) |
| EKF2 active | ✅ tilt=1 yaw=1 baro=1 |

---

**التاريخ**: 2026-05-06
**Hardware**: Samsung S23 Ultra (Snapdragon 8 Gen 2)
**Reference run**: `pil_flight_20260506_022444.csv` — 82.0/100
