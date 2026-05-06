# `_dev_tools/` — SITL development helpers

Personal developer scripts used to investigate PX4 ↔ MAVLink lockstep issues.
**Not part of the production SITL path.** `run_sitl_test.py` does not depend on
anything in this directory.

## Contents

| Script | Purpose |
|---|---|
| `debug_lockstep.py` | Hand-rolled MAVLink v2 frame builder; sends one `HIL_SENSOR` at a time and prints PX4's reply. Useful to isolate framing bugs. |
| `debug_pymavlink.py` | Same as above but uses `pymavlink` so framing is guaranteed correct. |
| `test_tcp_mavlink.py` | Minimal portable TCP listener that speaks the `HIL_SENSOR` handshake. |

## Environment

All scripts resolve paths in this order (first hit wins):

1. Environment variables (explicit override):
   - `PX4_SITL_BIN` — path to the `px4` binary.
   - `PX4_SITL_CWD` — PX4 working directory (defaults to `dirname(dirname(PX4_SITL_BIN))`).
   - `ACADOS_LIB`   — acados shared-library directory.
2. Defaults derived from `__file__`:
   - `<repo>/AndroidApp/app/src/main/cpp/PX4-Autopilot/build/px4_sitl_default/bin/px4`
   - `<repo>/acados-main/lib`

Example:

```sh
export PX4_SITL_BIN=/opt/PX4-Autopilot/build/px4_sitl_default/bin/px4
export ACADOS_LIB=/opt/acados/lib
python3 debug_pymavlink.py
```
