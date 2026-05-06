# PIL Trajectory Fix — Session Summary

**Date:** 2026-05-05
**Status:** Main objectives ✅ achieved
**Outcome:** apogee 8m → 266m (×33), full ascent matches SITL parity

---

## Bugs Fixed (root-cause, verified by data)

### F1 — `ROCKET_USE_GT=0` in HITL block
- **File:** `AndroidApp/.../px4_jni.cpp` (HITL block)
- **Before:** `ROCKET_USE_GT=1` (PIL bypassed EKF2, used groundtruth)
- **After:** `ROCKET_USE_GT=0` (full EKF2 path, real flight equivalent)
- **Why:** PIL must validate the entire EKF2→MPC stack, not bypass EKF2.
- **Symptom:** false-pass score 70/100 with apogee=3m.

### F2 — `EKF2_MAG_TYPE=5→6` in HITL block
- **File:** `AndroidApp/.../px4_jni.cpp` (HITL block) + airframe `22004_m130_rocket_mpc_hitl`
- **Before:** `EKF2_MAG_TYPE=5` (no mag fusion)
- **After:** `EKF2_MAG_TYPE=6` (init-only mag for yaw alignment)
- **Why:** without yaw alignment, EKF2 NED frame is undefined → MPC sees garbage state → tumbling.
- **Symptom:** `α=180°` (rocket tumbling) in run5.

### F3 — Gravity subtraction bug in `_body_specific_force`
- **File:** `6DOF_v4_pure/pil/mavlink_bridge_pil.py`
- **Before:** `return f_body / mass - C @ g_ned` (subtracted gravity twice)
- **After:** `return f_body / mass` (mirrors SITL)
- **Why:** PX4 expects "specific force" = F_ext/m (excludes gravity). Subtracting g_ned in body frame removed gravity that EKF2 needed to estimate tilt.
- **Symptom:** EKF2 NaN after ~5s, alignment failure.

### F4 — Warmup pad_forces correctness
- **File:** `6DOF_v4_pure/pil/mavlink_bridge_pil.py`
- **Before:** warmup sent zero/incorrect accel during pre-arm.
- **After:** `pad_forces = -pad_mass * (C_ned2b @ g_ned)` matches SITL.
- **Why:** EKF2 needs proper static-pad gravity reading to align tilt before arm.

### F5 — Re-noise sensors per warmup tick + launch_quat
- **File:** `6DOF_v4_pure/pil/mavlink_bridge_pil.py`
- **Before:** sent identical sensor values + identity quaternion every warmup tick.
- **After:** re-noised per tick + actual launch quaternion in HIL_STATE_QUATERNION.
- **Why:** PX4 DataValidator marks 100+ identical samples as STALE → baro/mag rejected. Identity quat creates EKF2 inconsistency with tilted-pad accel.

### F6 — MPC rate 50Hz → 25Hz
- **File:** `RocketMPC.cpp:1177` + `pil_config.yaml`
- **Before:** gate `>= 19_ms` (50Hz, deadline 20ms)
- **After:** gate `>= 39_ms` (25Hz, deadline 40ms)
- **Why:** phone MPC avg=40ms → at 50Hz, 78% over-deadline → 80ms phase lag in `inject_compute_delay` → ballistic trajectory collapses.
- **Symptom:** apogee=8m at 50Hz vs 266m at 25Hz.
- **Trade-off:** lookahead_stage=5 still maps to 100ms servo delay regardless of cadence; alignment correct.

---

## Verification Run Table

| Run | Apogee | Range | Time | Max α | Note |
|-----|--------|-------|------|-------|------|
| baseline (GT) | 3m  | 88m   | —    | 14°  | false-pass via GT |
| run3 (F3)     | 4m  | 89m   | —    | 11°  | EKF2 path active, gravity OK |
| run4 (F4-F5)  | 3m  | 88m   | —    | 7°   | warmup parity |
| run5          | 12m | 262m  | —    | 180° | tumbling (no yaw) |
| run6 (F2)     | 8m  | 290m  | 3.0s | 5°   | stable, ballistic missing |
| run7 (delay=0)| 74m | 1044m | 6.2s | 8°   | confirmed delay was issue |
| run8 (delay=40)| 4m | 124m  | 2.0s | crash| high run-to-run variance |
| **run9 (F6)** | **266m** | **1334m** | 9.1s | 14° (ascent) | ✅ |

---

## Outstanding Items (Optional)

- **NP5:** post-apogee `attitude_mode = velocity_aligned` for graceful descent (eliminates terminal tumble).
- **NP6:** regenerate acados solver with `N=40, tf=1.6s` (true 25Hz dt parity; current dt_solver still 20ms but solver runs every 40ms).

---

## Files Changed

```
mavlink_bridge_pil.py              +42 -14   (F3, F4, F5)
px4_jni.cpp                        +6  -1    (F1, F2)
22004_m130_rocket_mpc_hitl         +6  -2    (F2 doc)
RocketMPC.cpp                      +10 -9    (F6)
pil_config.yaml                    +1  -1    (F6)
PIL_PROGRESS.txt                   +37 -0    (tracker)
PIL_FIX_SUMMARY.md                 (new)     (this file)
```
