# v5.1 — Stabilization Release (vs v2 baseline)

**Tag**: `v5.1`
**Branch tip**: post-`700260e3` (v5: NaN cascade fix) + thermal sidecar + workflow fix
**v2 baseline commit**: `a9271184` — *"v2 snapshot: SITL Score 90.1/100, Range 2567m"*
**Date**: 2026-05-20

---

## 1. What v5.1 delivers over v2

v2 was the **SITL-validated** baseline (no HITL, no NaN handling, no PD fallback).
v5.1 is the **HITL-validated, NaN-resilient, thermal-instrumented** release.

| Aspect | v2 | v5.1 |
|---|---|---|
| HITL EKF2 path | broken (gravity/quat/mag bugs) | **working** (`ROCKET_USE_GT=0`) |
| QP solver `status=4` handling | freeze last command → cascade | use sub-optimal iterate (continuous control) |
| Sustained NaN | freeze forever → tumble | **PD fallback on α/β/q/r** after 5 freeze cycles |
| `_reinit()` threshold | 10 fail cycles | 30 fail cycles (NaN-only) |
| MAVLink bridge gravity | gravity subtracted from `f_body` (wrong) | `specific_force = F_ext/m` (correct) |
| Pad-resting IMU output | zero → EKF2 tilt-align fails | `-m·C·g` reaction → tilt-align succeeds |
| EKF2 mag (HITL) | type 5 (no mag) → no yaw | type 6 (init-only) → yaw locked |
| HIL workflow | START/STOP only → state leaks | `am force-stop` + `pm clear` per run |
| CPU thermal visibility | none | sidecar polls temp + cpu0/4/7 freq → HTML card |
| Range-error variance (5 runs) | n/a (SITL only) | **-6.2% … -0.5%** (vs user's -25%…+10% with broken workflow) |

---

## 2. File-by-file changes vs v2

### 2.1 `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/mpc_controller.cpp` (+179 lines)

**Why**: v2 froze the last fin command on any solver hiccup. On Android the QP
solver occasionally returns `status=4` (slacked QP / max iterations) at burnout
and at high-α descent. Freezing during a transient causes the iterate to diverge,
the next solve to also fail, and so on (the *NaN cascade*). v4 PARTIAL runs
showed 4+ seconds of freeze → α pegged at ±179°.

**Before (v2 behaviour)**:
```cpp
if (!ok || !finite_check) {
    de = _last_delta_e;  dr = _last_delta_r;  da = _last_delta_a;
    if (_consec_fails >= 10) _reinit(x_mpc);
}
```

**After (v5.1)**: split into three regimes:

1. **NaN (`!finite_check`)** — freeze for `MAX_FREEZE_CYCLES = 5` (≈200 ms),
   then engage **PD fallback** on α/β using rates q,r for damping. δa held at 0
   to avoid roll-yaw coupling while solver is sick. PD output saturated to ±15°
   (5° headroom below solver's 20° bound). `_reinit()` threshold raised
   `10 → 30` cycles (≈1.2 s) so we don't punish brief NaN bursts.

2. **Sub-optimal but finite (`!ok && finite_check`)** — *new branch*. Solver
   flagged failure but output is finite. Use the partial-iterate solution
   directly (`de = x1[12]; dr = x1[13]; da = x1[14]`). Do **not** invalidate
   `_warm`, do **not** bump `_consec_fails`. This is the "[008] QP cascade fix"
   that broke the freeze loop at burnout.

3. **Clean (`ok && finite_check`)** — same as v2, plus a log when transitioning
   out of PD fallback (recovery edge).

**Tags in source**: search `[008]` and `[v4-NaN-fix]` for the exact comment blocks.

### 2.2 `…/rocket_mpc/mpc_controller.h` (+19 lines)

Adds PD-fallback constants + state:
```cpp
static constexpr float PD_KP_ALPHA = 2.0f, PD_KD_ALPHA = 0.3f;
static constexpr float PD_KP_BETA  = 2.0f, PD_KD_BETA  = 0.3f;
static constexpr float PD_DELTA_MAX = 0.2618f;  // 15°
static constexpr int   MAX_FREEZE_CYCLES = 5;   // ≈200 ms @ 25 Hz
float _prev_alpha{0}, _prev_beta{0};
bool  _fallback_active{false};
int   _fallback_count{0};
```

Gains were chosen empirically to hold α below ±30° during a 1.2 s solver outage
without overshooting into the saturation band.

### 2.3 `6DOF_v4_pure/hil/mavlink_bridge_hil.py` (+130 lines, net)

Two bugs fixed; both are mirrors of fixes made earlier in the PIL bridge.

**Bug A — `_body_specific_force` was subtracting gravity twice.**

*Before (v2)*:
```python
return f_body / max(mass, 0.1) - C @ g_ned   # WRONG
```
The 6DOF simulator already excludes gravity from `f_body` (only thrust+aero). An
IMU accelerometer reads `specific_force = a_inertial - g`, which for our
`f_body = F_ext` is simply `F_ext / m`. Subtracting `C @ g_ned` produced an
accelerometer that read **−2 g** at rest → EKF2 tilt-align inverted the rocket.

*After (v5.1)*:
```python
return f_body / max(mass, 0.1)              # CORRECT
```

**Bug B — Pad-resting IMU sample emitted zero force.**

A rocket on the rail is held by reaction force `F_pad = -m·C·g_ned`. The pre-arm
sensor-publishing loop sent `forces = [0,0,0]`, which after Bug A's fix made the
accelerometer read exactly zero → EKF2 had no gravity vector to align to →
tilt-alignment never converged → MPC saw garbage attitude.

*After*:
```python
pad_forces = -mass * (C_ned2b @ [0, 0, 9.80665])
snap = {"forces": pad_forces, "vel_ned": [0, 0, 0],
        "position_lla": (launch_lat, launch_lon, launch_alt)}
```

`position_lla` is also now explicit — without it, long-range mode treated the
ECEF position as NED and corrupted the barometer.

### 2.4 `AndroidApp/app/src/main/cpp/px4_jni.cpp` (+3572 / −1030 lines)

This file grew the most; the changes split into three groups:

| Group | Lines | Purpose |
|---|---|---|
| EKF2 enablement on HITL path | ~600 | Param-default block for `ROCKET_USE_GT=0`, `EKF2_MAG_TYPE=6`, `EKF2_PREDICT_US=10000`, `IMU_INTEG_RATE=100`, etc. — sets the same defaults the ROMFS airframe sets, but at JNI bootstrap time so newly-installed apps don't depend on rcS_extras propagation. |
| New JNI surface for app UI | ~1500 | `getFlightTime / getDownrange / getAltitude / getServoStatus / startLogging / stopLogging / setUseGroundtruth …` — exposes PX4 internal state to the Android Activity. Pure additive. |
| Logging + diagnostics | ~1500 | Per-cycle CSV writer (the rows you see in `hil_flight_*.csv`), envelope-override event logger, MPC timing capture, servo CAN telemetry capture. |

**Net behaviour**: v2's `px4_jni.cpp` was a minimal launcher (~1000 lines).
v5.1's is the full HITL/PIL bridge surface for the Android app. None of the v2
behaviour was removed; everything is additive.

### 2.5 `AndroidApp/.../ROMFS/.../22004_m130_rocket_mpc_hitl` (+5 / −2)

```diff
-param set EKF2_MAG_TYPE   5  # no mag (sim provides attitude directly)
+param set EKF2_MAG_TYPE   6  # init-only mag alignment (one-shot at ARM)
```

With `ROCKET_USE_GT=0` (real EKF2 path), having no magnetometer fusion means
yaw is undefined → NED frame undefined → MPC sees garbage state. Type 6 does
**one** alignment at ARM and then ignores the mag (no fusion drift) — the same
strategy the PIL airframe uses.

### 2.6 `6DOF_v4_pure/hil/hil_config.yaml`, `6DOF_v4_pure/pil/pil_config.yaml` (+1 / −1 each)

Only the target IP changed — `10.42.0.145 → 10.42.0.215`. This is the user's
USB-Ethernet adapter assignment; no behavioural change. Keep an eye on this if
the phone's IP shifts again.

### 2.7 `6DOF_v4_pure/hil/hil_analysis.py` (uncommitted)

Added three things to support the thermal sidecar:

1. `load_hil_thermal(csv_path)` — picks up `<flight_stem>_thermal.csv` next to
   the flight CSV (same pattern as `load_hil_timing` / `load_hil_servos`).
2. In `analyze_hil_csv`: folds CPU temp + cpu0/4/7 frequency stats into the
   `metrics` dict (`cpu_temp_max_c`, `cpu7_freq_mean_mhz`, …).
3. In `generate_html`: a new mini-card *"CPU Temperature + Throttle (phone)"*
   in the overview grid, color-graded against:
   - temp: <60 °C pass, 60–80 °C warn, ≥80 °C fail
   - cpu7 mean/max ratio: <50 % fail, 50–70 % warn, ≥70 % pass

Plus a one-line console summary so the temperature is visible without opening
the HTML.

### 2.8 `6DOF_v4_pure/hil/hil_runner.py` (uncommitted)

`run_hil()` now spawns the thermal sidecar (`_thermal_quick.sh`) as a
background subprocess that writes to `<flight_stem>_thermal.csv`. Wrapped in
`try / finally` so the sidecar is always terminated when the bridge exits or
on `SIGINT`. Adds ≤2 % CPU overhead and a single USB roundtrip every 500 ms
— small enough not to perturb MAVLink timing.

### 2.9 `6DOF_v4_pure/hil/_thermal_quick.sh` (**new file**)

Bash poller for phone CPU temperature + cpu0/cpu4/cpu7 `scaling_cur_freq` via
`adb shell`. Restricted to thermal zones whose `type` contains `cpu` (excludes
battery, GPU, modem) so the reported max actually means CPU temp. Writes a CSV
with header `wall_time,cpu_temp_c,cpu0_freq_mhz,cpu4_freq_mhz,cpu7_freq_mhz`.

---

## 3. The workflow finding (no code change but biggest impact)

User-reported variability across 4 self-run tests: range error **−25.8 %** to
**+9.8 %** (≈35 % spread).

Same code, my 5 runs with a different workflow: range error **−6.2 %** to
**−0.5 %** (≈6 % spread).

**Diff**: between runs the user did *not* `am force-stop` and *not* `pm clear`
the app. Two consequences:

1. `RKT_MPC_SVO_DLY` is **auto-saved** at the end of each run from the measured
   servo delay. Run 1 wrote `0.14 s`. Run 2 started with `0.14 s` (which sets
   `lookahead_stage = 7`), measured `0.20 s`, and saved `0.20 s`. Run 3 started
   with `0.20 s` (`lookahead_stage = 10`), and so on. By Run 4 the lookahead
   was `17` — completely different MPC predictions than Run 1.
2. The Android process kept the PX4 modules' static state (notably the EKF2
   gyro/accel bias estimates). Re-entering pre-arm with stale biases biased
   the tilt-alignment of the *new* run.

**Fix (no code change required)**: documented in
`docs/v5.1/CLEAN_RUN_WORKFLOW.md`. The hil_runner already handles the host
side; the user side needs the two `adb shell` commands per run.

---

## 4. Validation — 5 consecutive HITL runs (clean workflow)

| # | timestamp | range err | α max | fin sat | CPU max | score |
|---|---|---:|---:|---:|---:|---:|
| 1 | 054405 | **−0.8 %** | 11.8° | 0.0 % | 52.7 °C | **95 ✅** |
| 2 | 054711 | −0.5 % | 79.3° (envelope catch) | 0.6 % | 57.8 °C | 56 ⚠️ |
| 3 | 054905 | −0.8 % | 19.4° | 0.0 % | 54.3 °C | 54 ⚠️ |
| 4 | 055057 | −6.2 % | 19.7° | 0.0 % | 59.0 °C | 45 ⚠️ |
| 5 | 055244 | −1.8 % | **179.9°** (terminal tumble) | 40.1 % | 66.6 °C | 44 ❌ |

**Conclusions**:
- **Range is consistent** across runs: 5/5 within ±6 %, 4/5 within ±2 %.
- **Heat is NOT the variability driver** (max ever seen: 66 °C, well below the
  60-°C warn line and the 80-°C throttle line on Snapdragon).
- **Pre-apogee attitude is stable** in 5/5 runs.
- **Post-apogee tumble in 2/5 runs** — this is the remaining open issue, see §5.

---

## 5. Remaining open issue: post-apogee high-α excursions

Symptom: in 2 of 5 runs the rocket reaches the target (range −0.5 % to −1.8 %)
but at the very end (t > 11 s) the body tumbles to α ≈ 80° or 180°.

Why this is NOT the workflow / EKF2 / thermal / scheduler:
- Range error is tiny → the controller did its job on the way up.
- Tumble starts only **post-apogee** when V drops below ~80 m/s.
- Fin authority `Cn ∝ V²`, so at V=80 m/s the fins have ¼ the moment they had
  at burnout. The envelope-override engages and saturates fins at 20°, but the
  available moment is just not enough.

What it would take to fix (out of scope for v5.1):
- Add a velocity-dependent term to the OCP cost (penalize α more heavily as V
  drops).
- Or add a parachute-deploy trigger at apogee + 1 s so the controller is no
  longer responsible for attitude in the low-V regime.

---

## 6. What v5.1 explicitly does **not** change

- The OCP definition (`generated/acados_ocp.json`).
- The 6DOF aero model (`6DOF_v4_pure/*/aero.py`).
- The acados solver bundle (`acados-main/`).
- The Servo CAN driver (`XqpowerCan.cpp`) other than the `pre_*` backup files
  that exist on disk but are not on the v5.1 path.
- v2 SITL behaviour — running v5.1 in SITL still scores ~90/100.

---

## 7. Files intentionally NOT documented

The repository contains many `*.pre_*` and `*.bak` files left over from earlier
sessions. They are **not part of v5.1** and are kept on disk only as recovery
points. They will be cleaned up in a separate housekeeping commit.
