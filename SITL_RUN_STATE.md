# SITL Run State — Current Point (2026-04-21)

## Build Status
✅ PX4 builds successfully (420/420) after all fixes including ROCKET_MAX_DEFL removal.
✅ Incremental rebuild after airframe changes (3/3) works.
✅ Binary ready: `/home/yoga/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/build/px4_sitl_default/bin/px4`

## SITL Run Results So Far
- PX4 starts correctly
- Loads default airframe (10016 iris) because no `PX4_SYS_AUTOSTART` set
- Hangs on `simulator_mavlink: Waiting for simulator to accept connection on TCP port 4560`
- Needs either:
  - External physics simulator connected on TCP 4560 (e.g., `rocket_6dof_sim.py`)
  - OR setting `PX4_SYS_AUTOSTART=22003` AND providing simulator

## Key Finding: NO ROCKET_MAX_DEFL ERRORS!
Previous test output showed:
- "Parameter UXRCE_DDS_KEY not found" (unrelated)
- "Parameter UXRCE_DDS_AG_IP not found" (unrelated)
- **NO mention of ROCKET_MAX_DEFL** — confirms clean deletion

## To Test rocket_mpc Airframe Specifically
```bash
cd /home/yoga/m13/AndroidApp/app/src/main/cpp/PX4-Autopilot/build/px4_sitl_default
rm -rf rootfs_test && mkdir rootfs_test && cd rootfs_test
PX4_SYS_AUTOSTART=22003 PX4_SIM_MODEL=none_iris timeout 10 ../bin/px4 -i 0 -d ../etc 2>&1 | grep -iE "rocket|mpc|ROCKET_MAX|error|warn|failed" | head -50
```

This will:
1. Force airframe 22003 (M130 Rocket MPC)
2. Run for 10 sec
3. Filter output for rocket/mpc/error/warning messages
4. Simulator will still hang waiting for 4560 connection — that's OK, we just want init logs

## Pending Final Steps
1. Run SITL with airframe 22003 and capture output
2. Verify:
   - No "ROCKET_MAX_DEFL not found" warnings
   - rocket_mpc module starts
   - xqpower_can doesn't warn about limit mismatch (since solver limit = 20° = XQCAN_LIMIT default)
3. Report findings to user

## Files Modified This Session (all bugs #1-#8 fixed)
See /home/yoga/m13/SITL_BUILD_STATE.md for complete list.
