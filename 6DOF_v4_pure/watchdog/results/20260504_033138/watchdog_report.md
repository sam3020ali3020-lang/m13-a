# Watchdog Test Report
- results dir: `/home/yoga/m13/m13/6DOF_v4_pure/watchdog/results/20260504_033138`
- scenarios run: 1
- overall: **FAIL**

## Thresholds
| metric | max |
|---|---|
| `detection_ms_max` | 600 |
| `restart_ms_max` | 1500 |
| `recovery_ms_max` | 2500 |
| `bystander_recovery_ms_max` | 3000 |
| `max_failures_per_scenario` | 0 |

## Scenario: `solo_crash` — FAIL
**Failures:**
- commander#1: detection 911ms > 600ms
- mavlink_tcp_bridge#1: detection 1020ms > 600ms

| module | det_med (ms) | det_max | restart_med | recovery_med | recovery_max | N |
|---|---|---|---|---|---|---|
| `native_sensor_reader` | 106 | 106 | 52 | 52 | 52 | 1 |
| `sensors` | 70 | 70 | 54 | 102 | 102 | 1 |
| `ekf2` | 76 | 76 | 52 | 505 | 505 | 1 |
| `rocket_mpc` | 174 | 174 | 83 | 101 | 101 | 1 |
| `commander` | 911 | 911 | 53 | 101 | 101 | 1 |
| `control_allocator` | 469 | 469 | 52 | 101 | 101 | 1 |
| `mavlink_tcp_bridge` | 1020 | 1020 | 52 | 102 | 102 | 1 |
