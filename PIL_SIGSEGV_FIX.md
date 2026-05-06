# PIL SIGSEGV Fix — Root Cause & Solution

**Date:** Apr 24, 2026
**Status:** ✅ RESOLVED — Full PIL flight completes successfully (1072 steps, 10.7s) with MPC(N=200) + MHE(N=20) both active.

## Symptom

PIL crashed with SIGSEGV at the first `m130_rocket_acados_update_params` call on Android ARM64. Debug showed:
```
config->N=80   dims->N=80   plan->N=80
Ts→parameter_values gap = 640 bytes = 80*8  (expected 1600 = 200*8)
```
despite every `M130_ROCKET_N` macro in the sources being `200`.

## Root Cause

`scripts/build_m130_solvers_arm64.sh` used `ar rcs` to create the static archive. The `rcs` flags **append** objects to an existing archive rather than replacing it:

```bash
$AR rcs "$OUT_DIR/libm130_solvers.a" "${OBJS[@]}"
```

Because the same directory had previously been used as the output of a CMake build (inside Android Studio), it contained stale objects with a different naming convention (`<file>.c.o` from CMake vs `<file>.o` from our script). These stale objects were **never removed** by subsequent script runs, so `libm130_solvers.a` kept accumulating both:

- New objects (built from current c_generated_code/): `acados_solver_m130_rocket.o`
- Old objects (from an earlier CMake build with a different N / struct layout): `acados_solver_m130_rocket.c.o`

Both `.o` and `.c.o` defined the same global `T` symbols (e.g. `m130_rocket_acados_create_with_discretization`). When the linker resolved those symbols for `libpx4phone_native.so`, it took the **first** definition it encountered in the archive — which happened to be the stale `.c.o` built with different code.

That stale code allocated memory for a different `N` (whatever value it was compiled with in the old build), producing the 80-vs-200 mismatch visible at runtime and the SIGSEGV inside `ocp_nlp_in_set`.

## Fix

One-line change in `scripts/build_m130_solvers_arm64.sh`: delete any pre-existing archive before creating the new one, so `ar rcs` cannot append to stale state.

```diff
+ rm -f "$OUT_DIR/libm130_solvers.a"  # ensure clean archive (ar rcs appends if file exists)
  $AR rcs "$OUT_DIR/libm130_solvers.a" "${OBJS[@]}"
```

Effect: archive size dropped from 2,492,040 bytes (contaminated: 16 new `.o` + 16 stale `.c.o`) to 526,464 bytes (clean: 16 `.o` only).

## Verification

After the fix, with **zero additional code changes** (same C generator, same solvers, same PX4 code):

| Check | Before | After |
|-------|--------|-------|
| `plan->N` at init | 80 | **200** ✓ |
| `config->N` at init | 80 | **200** ✓ |
| `dims->N` at init | 80 | **200** ✓ |
| Gap Ts→param_values | 640 bytes | **1600 bytes** ✓ |
| SIGSEGV on first solve | crash | **no crash** ✓ |
| PIL flight completion | crash | **1072 steps, ground impact at 10.7s** ✓ |
| MHE enabled simultaneously | N/A (disabled to isolate crash) | **MHE(N=20) + MPC(N=200) both OK** ✓ |

## Files Restored / Cleaned After Debug Session

- `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/mpc_controller.cpp`
  - Removed init-time `MPC DEBUG` logging block (~20 lines)
  - Removed solve-time raw-pointer dump block (~25 lines)
- `AndroidApp/app/src/main/cpp/PX4-Autopilot/src/modules/rocket_mpc/RocketMPC.cpp`
  - Re-enabled `_mhe.init(mhe_cfg)` (was commented out to isolate the MPC crash)
- `c_generated_code/acados_solver_m130_rocket.c`
  - Removed temporary `M130_DBG` instrumentation (regenerated each run of `rocket_6dof_sim.py` anyway)

## Lessons

1. `ar rcs` creates archives but does not *clear* them — always `rm -f` the target first in build scripts.
2. When two build systems write to the same archive file with different object-naming conventions (`.o` vs `.c.o`), the linker silently picks one. Mixed-origin archives are a footgun; treat them as build-system bugs.
3. The "impossible" symptoms (`plan->N` readable as both 200-from-allocation and 80-from-library-code simultaneously) are the fingerprint of two different versions of the same translation unit coexisting in one binary.
