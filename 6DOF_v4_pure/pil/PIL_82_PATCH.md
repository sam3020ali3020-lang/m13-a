# PIL 82/100 — Patch Document (Code-Level Diff)

> **الغاية**: نسخة code-level من كل التعديلات التي أعطت 82/100 في PIL.
> أعطِ هذا الملف لأي شخص ليُطبّق التعديلات على نسخته بدقّة سطر-سطر.

---

## ملخّص الملفات المعدّلة

| # | الملف | عدد التعديلات |
|---|---|---|
| 1 | `AndroidApp/app/src/main/cpp/px4_jni.cpp` | 12 سطر params + 1 سطر default IP |
| 2 | `AndroidApp/app/src/main/cpp/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/22003_m130_rocket_mpc` | إضافة 18 سطر params |
| 3 | `AndroidApp/app/src/main/cpp/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/22004_m130_rocket_mpc_hitl` | تعديل قيمتين |
| 4 | `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/RocketMPC.cpp` | MPC rate gate |
| 5 | `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/los_guidance.h` | bug fix |
| 6 | `6DOF_v4_pure/pil/mavlink_bridge_pil.py` | 3 إصلاحات |
| 7 | `6DOF_v4_pure/pil/pil_config.yaml` | mpc_cycle_hz + inject_compute_delay |

---

# 1) `AndroidApp/app/src/main/cpp/px4_jni.cpp`

## 1.A — EKF2 / IMU params (HITL block)

موقع التعديل: **داخل block `is_hitl == true`** (تقريباً سطر 600-770 في الإصدار الحالي).

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

// (لم تكن هذه الـparams موجودة:)
// IMU_INTEG_RATE, IMU_GYRO_RATEMAX, EKF2_ANGERR_INIT, EKF2_HDG_GATE, EKF2_HEAD_NOISE
```

### بعد ✅
```cpp
// EKF2 sensor noise (matches SITL airframe 22003 — verified parity)
p = param_find("EKF2_ACC_NOISE");
if (p != PARAM_INVALID) { float v = 2.0f; param_set(p, &v); }    // 3.0 → 2.0

p = param_find("EKF2_GPS_V_NOISE");
if (p != PARAM_INVALID) { float v = 0.5f; param_set(p, &v); }    // 1.5 → 0.5 (bridge gps_vel_std=0.1)

p = param_find("EKF2_GPS_P_NOISE");
if (p != PARAM_INVALID) { float v = 2.5f; param_set(p, &v); }    // 2.0 → 2.5 (bridge gps_pos_std=2.5)

p = param_find("EKF2_GPS_DELAY");
if (p != PARAM_INVALID) { float v = 100.0f; param_set(p, &v); }  // 200 → 100 (bridge gps_delay_ms=100)

p = param_find("EKF2_ABL_LIM");
if (p != PARAM_INVALID) { float v = 0.8f; param_set(p, &v); }    // 1.0 → 0.8 (PX4 max bound)

p = param_find("EKF2_GPS_V_GATE");
if (p != PARAM_INVALID) { float v = 50.0f; param_set(p, &v); }   // 10 → 50 (high-G burn)

// === الأهم: تخفيض حمل CPU بالنصف (200Hz → 100Hz) ===
p = param_find("EKF2_PREDICT_US");
if (p != PARAM_INVALID) { int32_t v = 10000; param_set(p, &v); } // 5000 → 10000

// IMU integration rate must match EKF2_PREDICT_US
p = param_find("IMU_INTEG_RATE");
if (p != PARAM_INVALID) { int32_t v = 100; param_set(p, &v); }

// Inner-loop gyro publication rate
p = param_find("IMU_GYRO_RATEMAX");
if (p != PARAM_INVALID) { int32_t v = 100; param_set(p, &v); }

// EKF2 initial tilt uncertainty: fast tilt_align (matches SITL)
p = param_find("EKF2_ANGERR_INIT");
if (p != PARAM_INVALID) { float v = 0.01f; param_set(p, &v); }

// Heading fusion gates (for MAG_TYPE=6 init-only at ARM)
p = param_find("EKF2_HDG_GATE");
if (p != PARAM_INVALID) { float v = 10.0f; param_set(p, &v); }

p = param_find("EKF2_HEAD_NOISE");
if (p != PARAM_INVALID) { float v = 0.7f; param_set(p, &v); }
```

## 1.B — Default Target IP (للـWiFi-ADB)

موقع التعديل: **داخل block `start_px4_modules`** (تقريباً سطر 1814).

### قبل ⛔
```cpp
if (target_ip[0] == '\0') {
    strncpy(target_ip, "127.0.0.1", sizeof(target_ip) - 1);
}
```

### بعد ✅
```cpp
if (target_ip[0] == '\0') {
    // Default for WiFi/USB-tethered ADB: laptop's IP on shared 10.42.0.0/24 network.
    // For USB ADB with `adb reverse`, "127.0.0.1" works too.
    strncpy(target_ip, "10.42.0.1", sizeof(target_ip) - 1);
}
```

> **ملاحظة لصديقك**: غيّر `10.42.0.1` لـIP لابتوبه إذا اختلف. أو يضع IP لابتوبه عبر:
> ```bash
> adb shell setprop persist.m130.target_ip <LAPTOP_IP>
> ```
> (يحتاج root). البديل الأبسط: تعديل السطر مباشرة وإعادة بناء APK.

---

# 2) `AndroidApp/app/src/main/cpp/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/22003_m130_rocket_mpc`

أضِف هذه الكتل **قبل** السطر `param set-default RKT_MPC_SVO_DLY`:

```sh
# Use EKF2 path (ROCKET_USE_GT=0) for apples-to-apples parity with PIL/HITL.
# PIL airframe (22004) also uses USE_GT=0, so SITL must match for fair comparison.
# Set to 1 only to isolate MPC from EKF2 errors during specific debugging.
param set-default ROCKET_USE_GT   0

# HITL: init-only magnetometer alignment — required for ROCKET_USE_GT=0.
# Mirrors PIL airframe (22004) for parity.
param set EKF2_MAG_TYPE   6

# EKF2 initial tilt uncertainty: default 0.1 rad (5.7°) causes tilt_align
# to barely miss the 3° threshold during warm-up. 0.01 rad (0.57°) matches
# the actual accuracy of accelerometer-based tilt init on a stationary rail.
param set EKF2_ANGERR_INIT 0.01

# EKF2 GPS delay: match bridge's SensorNoise.gps_delay_ms = 100ms.
param set EKF2_GPS_DELAY   100

# SITL EKF2 tuning to prevent attitude corruption during high-G burn:
# Root cause: GPS velocity innovations corrupt attitude through Kalman
# cross-terms. High ACC_NOISE → velocity covariance grows fast → GPS
# innovations are attributed to velocity error (not attitude).
param set EKF2_ACC_NOISE    2.0
param set EKF2_GPS_V_NOISE  0.5
param set EKF2_GPS_V_GATE   50
param set EKF2_GPS_P_NOISE  2.5
param set EKF2_TAU_VEL      0.25
# Freeze gyro bias: prevent GPS innovations from corrupting gyro bias.
param set EKF2_GYR_B_NOISE  0.0001

# CPU/IMU rate budget — match PIL (Android) for parity.
# EKF2 prediction at 100Hz reduces CPU 50% vs default 200Hz; phone needs this.
param set EKF2_PREDICT_US   10000
param set IMU_INTEG_RATE    100
param set IMU_GYRO_RATEMAX  100

# Accel bias clamp — must be <= PX4 max (0.8).
param set EKF2_ABL_LIM      0.8

# Heading fusion gates — for MAG_TYPE=6 (init-only) at ARM.
param set EKF2_HDG_GATE     10.0
param set EKF2_HEAD_NOISE   0.7
```

---

# 3) `AndroidApp/app/src/main/cpp/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/22004_m130_rocket_mpc_hitl`

تأكّد أن هذه القيم موجودة (إن لم تكن، أضِفها):

```sh
# Use EKF2 instead of ground-truth (full-stack test path).
param set-default ROCKET_USE_GT   0

# Init-only magnetometer alignment (one-shot at ARM, no continuous fusion).
param set-default EKF2_MAG_TYPE   6
```

---

# 4) `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/RocketMPC.cpp`

موقع التعديل: **MPC solve rate-limit gate** (تقريباً سطر 1158-1187).

### قبل ⛔
```cpp
// Rate-limit MPC solves to 50 Hz (20 ms gate)
const hrt_abstime kMpcGate_us = 20'000;
```

### بعد ✅
```cpp
// Rate-limit MPC solves to 25 Hz (40 ms gate) — phone solver capability.
// Higher rates cause queue buildup → phase lag → fin command staleness.
const hrt_abstime kMpcGate_us = 40'000;
```

---

# 5) `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/los_guidance.h`

موقع التعديل: **`set_gamma_natural()`** (تقريباً سطر 34-46).

### قبل ⛔
```cpp
void set_gamma_natural(float gamma_natural_rad) {
    _gamma_natural = gamma_natural_rad;
    // BUG: _gamma_ref_prev not updated → rate-limiter uses stale value
}
```

### بعد ✅
```cpp
void set_gamma_natural(float gamma_natural_rad) {
    _cfg.gamma_natural_rad = gamma_natural_rad;
    // CRITICAL FIX: must update _gamma_ref_prev too, otherwise rate-limiter
    // uses stale value and gamma_ref drifts from natural arc → vertical climb.
    _gamma_ref_prev = gamma_natural_rad;
}
```

---

# 6) `6DOF_v4_pure/pil/mavlink_bridge_pil.py`

## 6.A — Gravity subtraction bug

موقع التعديل: **`_body_specific_force()`** (تقريباً سطر 372-399).

### قبل ⛔
```python
def _body_specific_force(f_body, mass, g_ned, quat):
    ...
    # BUG: subtracts gravity, but PX4 EKF2 expects spec-force WITH gravity component
    return f_body / max(mass, 0.1) - C @ g_ned
```

### بعد ✅
```python
def _body_specific_force(f_body, mass, g_ned, quat):
    ...
    # FIX: do NOT subtract gravity — EKF2 expects raw specific force.
    # Bridge sends accel = f_body/mass (which already includes gravity reaction).
    return f_body / max(mass, 0.1)
```

## 6.B — Re-noise sensors per tick during warmup

موقع التعديل: **داخل warmup loop في `run()`** (تقريباً سطر 943-958).

### قبل ⛔
```python
# warmup يرسل نفس init_sensors لكل tick → DataValidator يرى 100+ عيّنة متطابقة → STALE
self._send(build_hil_sensor(
    self._sim_t_us,
    init_sensors["accel_body"], init_sensors["gyro_body"],
    init_sensors["mag_body"], init_sensors["baro_p"],
    init_sensors["diff_p"], init_sensors["pressure_alt"],
))
```

### بعد ✅
```python
# Re-noise per tick: ضوضاء جديدة كل tick → يمنع DataValidator STALE
wu_noisy_accel, wu_noisy_gyro = self.noise.add_imu_noise(
    init_sensors["accel_body_true"], np.zeros(3)
)
wu_noisy_mag = self.noise.add_mag_noise(init_sensors["mag_body"])
wu_noisy_alt = self.noise.add_baro_noise(self.launch_alt)
wu_noisy_baro_p = 1013.25 * (1.0 - 2.25577e-5 * wu_noisy_alt) ** 5.25588
self._send(build_hil_sensor(
    self._sim_t_us,
    wu_noisy_accel, wu_noisy_gyro,
    wu_noisy_mag, wu_noisy_baro_p,
    init_sensors["diff_p"], wu_noisy_alt,
))
```

## 6.C — Use actual launch quaternion in warmup HIL_STATE

موقع التعديل: **`run()` warmup HIL_STATE_QUATERNION** (تقريباً سطر 933-938).

### قبل ⛔
```python
# Identity quaternion → EKF2 yaw alignment fails (rocket pitch=15° not 0°)
self._send(build_hil_state_quat(
    self._sim_t_us,
    np.array([1.0, 0.0, 0.0, 0.0]),  # ← BUG
    np.zeros(3),
    self.launch_lat, self.launch_lon, self.launch_alt,
    0, 0, 0, init_sensors["accel_body_true"], 0.0,
))
```

### بعد ✅
```python
# Use the real launch quaternion so EKF2 sees consistent attitude during warmup.
self._send(build_hil_state_quat(
    self._sim_t_us,
    state[6:10],  # ← FIX: actual launch quaternion (pitch=15°, yaw=0°)
    np.zeros(3),
    ...
))
```

---

# 7) `6DOF_v4_pure/pil/pil_config.yaml`

## 7.A — MPC cycle rate (must match RocketMPC.cpp gate)

### قبل ⛔
```yaml
clock:
  mpc_cycle_hz: 50.0
```

### بعد ✅
```yaml
clock:
  mpc_cycle_hz: 25.0    # يجب مطابقة RocketMPC.cpp gate=39_ms (25Hz)
```

## 7.B — Compute delay injection (lockstep realism)

### قبل ⛔
```yaml
clock:
  inject_compute_delay_max_ms: 0   # disabled
```

### بعد ✅
```yaml
clock:
  inject_compute_delay_max_ms: 80  # يحقن MPC solve time في الفيزياء (واقعي)
```

---

# 8) خطوات التطبيق على نسخة جديدة

```bash
# 1) طبّق كل التعديلات أعلاه (نسخ/لصق أو git apply patch).

# 2) ابنِ SITL (إذا غيّرت airframe 22003):
cd <repo>/AndroidApp/app/src/main/cpp/PX4-Autopilot
make px4_sitl_default

# 3) ابنِ Android APK (إذا غيّرت px4_jni.cpp / RocketMPC.cpp / los_guidance.h):
cd <repo>/AndroidApp
JAVA_HOME=<jdk17_path> PATH=<jdk17_path>/bin:$PATH ./gradlew assembleDebug

# 4) ثبّت APK على الهاتف:
~/Android/Sdk/platform-tools/adb install -r app/build/outputs/apk/debug/app-debug.apk

# 5) اختبار SITL:
cd <repo>/6DOF_v4_pure/sitl
bash run_sitl_test.sh --px4-bin ../../AndroidApp/app/src/main/cpp/PX4-Autopilot/build/px4_sitl_default/bin/px4

# 6) اختبار PIL:
cd <repo>/6DOF_v4_pure/pil
python3 -u pil_runner.py
# عندما تظهر: "Listening on TCP 0.0.0.0:4560" → افتح التطبيق على الهاتف واضغط START
```

---

# 8) القيم النهائية الموثّقة (Reference Table)

| Param | القيمة النهائية | السبب |
|---|---|---|
| `EKF2_ACC_NOISE` | 2.0 | Kalman cross-term protection |
| `EKF2_GPS_V_NOISE` | 0.5 | match bridge std=0.1 + margin |
| `EKF2_GPS_P_NOISE` | 2.5 | match bridge std=2.5 |
| `EKF2_GPS_V_GATE` | 50 | high-G burn tolerance |
| `EKF2_GPS_DELAY` | 100 | match bridge=100ms |
| `EKF2_ABL_LIM` | 0.8 | PX4 max (avoid silent clip) |
| `EKF2_PREDICT_US` | 10000 (100Hz) | **CPU bottleneck fix** ⭐ |
| `IMU_INTEG_RATE` | 100 | match EKF2 |
| `IMU_GYRO_RATEMAX` | 100 | match EKF2 |
| `EKF2_ANGERR_INIT` | 0.01 | tilt_align speed |
| `EKF2_HDG_GATE` | 10.0 | MAG_TYPE=6 init |
| `EKF2_HEAD_NOISE` | 0.7 | MAG_TYPE=6 init |
| `EKF2_TAU_VEL` | 0.25 | fast velocity tracking |
| `EKF2_GYR_B_NOISE` | 0.0001 | freeze gyro bias |
| `EKF2_MAG_TYPE` | 6 | init-only at ARM |
| `ROCKET_USE_GT` | 0 | EKF2 full-stack |
| `MPC rate gate` | 40ms (25Hz) | phone solver capability |

---

**النتيجة المتوقّعة بعد تطبيق كل ما فوق**:
- SITL: ~91/100
- PIL (Snapdragon 8 Gen 2 phone, WiFi-ADB): ~82/100

**التاريخ**: 2026-05-06
**Run reference**: `pil_flight_20260506_022444.csv` — Score 82.0/100, Range 2461m, ratio 1.03x
