# Current SITL Build State (2026-04-21)

## User Request (Arabic)
"ابني وشغل اختبار sitl وانظر الى النتائج" — Build and run SITL test, show results.

## What's Done
1. **Build 1 completed successfully**: `make px4_sitl_default` → 420/420 (incl. `modules__rocket_mpc.a`, `bin/px4`). No errors.
2. **Removed `ROCKET_MAX_DEFL` references from 3 airframes** (were using deleted param):
   - `ROMFS/px4fmu_common/init.d-posix/airframes/22003_m130_rocket_mpc`
   - `ROMFS/px4fmu_common/init.d/airframes/22004_m130_rocket_mpc_hitl`
   - `ROMFS/px4fmu_common/init.d/airframes/22005_m130_rocket_mpc_real`
3. **Kept** `ROCKET_MAX_DEFL` in airframes 22000, 22001, 22002 (PID controllers, NOT rocket_mpc — but need to verify PID uses this param too; if not, these will also fail).

## What's Pending
1. **REBUILD** to incorporate airframe changes: `cd /home/yoga/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot && make px4_sitl_default`
2. **Check if airframes 22000/22001/22002 also need cleanup** — they reference `ROCKET_MAX_DEFL` but no other code defines it. Run: `grep_content "PARAM_DEFINE.*ROCKET_MAX_DEFL" /home/yoga/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/src` — returned **No matches**. All airframes using `ROCKET_MAX_DEFL` will get "unknown parameter" warnings.
3. **Run SITL**: `cd build/px4_sitl_default && ./bin/px4` with airframe 22003 (M130 MPC SITL). Use env var `PX4_SIM_MODEL=none` or similar.
4. **Verify behavior**:
   - `rocket_mpc start` loads module
   - No "ROCKET_MAX_DEFL not found" errors
   - MPC solver clamps to hardcoded `SOLVER_DELTA_MAX_RAD = 0.3490658503988659f`
   - `xqpower_can status` or logs show no limit mismatch warning

## Key Files Modified (Session)
### rocket_mpc
- `src/modules/rocket_mpc/rocket_mpc_params.c` — deleted `ROCKET_MAX_DEFL` definition
- `src/modules/rocket_mpc/RocketMPC.hpp:200` — deleted `_param_max_defl`
- `src/modules/rocket_mpc/RocketMPC.cpp:62` — added `static constexpr float SOLVER_DELTA_MAX_RAD = 0.3490658503988659f;`
- `src/modules/rocket_mpc/RocketMPC.cpp:1059, 1104` — `_param_max_defl.get()` → `SOLVER_DELTA_MAX_RAD`

### xqpower_can
- `src/drivers/xqpower_can/XqpowerCan.cpp:109-117` — removed `param_find("ROCKET_MAX_DEFL")`, now compares `_angle_limit` to hardcoded 20° solver limit
- `src/drivers/xqpower_can/xqpower_can_params.c` — updated doc comment

### ROMFS airframes
- `22003_m130_rocket_mpc` — removed `param set-default ROCKET_MAX_DEFL 0.349`
- `22004_m130_rocket_mpc_hitl` — removed `param set ROCKET_MAX_DEFL 0.349`
- `22005_m130_rocket_mpc_real` — removed `param set-default ROCKET_MAX_DEFL 0.349`

## Context (from prior session — all bugs fixed)
All 7 critical bugs from deep audit were fixed + #8 (s_consumed_fd race condition atomic):
1. send_test multiplier (removed send_test entirely)
2. actuator_outputs_sim HITL guard (SYS_HITL check)
3. sensor_gps instead of vehicle_gps_position for MHE
4. tau_servo NaN guard (2 locations)
5. MHE GPS origin guards (3 checks in sensor_bridge.cpp)
6. Runtime limit mismatch check (now obsolete — param removed)
7. atomic override fields + setters (move/zero/release use setters)
8. s_consumed_fd atomic race fix

## Build Directory
`/home/yoga/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/build/px4_sitl_default/`
Binary: `./bin/px4`

## SITL Airframe 22003
M130 Rocket MPC SITL — uses 6DOF_v4_pure/rocket_6dof_sim.py Python physics bridge via MAVLink HIL. SYS_HITL=1.

## Next Commands to Run
```bash
cd /home/yoga/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot
# Rebuild after airframe changes
make px4_sitl_default 2>&1 | tail -30

# Run SITL (airframe 22003 = M130 Rocket MPC)
# Airframes in init.d-posix are auto-discovered by airframe_number
cd build/px4_sitl_default
PX4_SIM_MODEL=none_iris ./bin/px4 -i 0 -s etc/init.d-posix/rcS -d 2>&1 | head -100 &
sleep 15
kill %1 2>/dev/null
```

## Known Ignored
- lint-diagnostics about `rclcpp/rclcpp.hpp` — IDE include path issue, NOT a build error (confirmed: full build 420/420 succeeded).
