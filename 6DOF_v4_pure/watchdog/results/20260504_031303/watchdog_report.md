# Watchdog Test Report
- results dir: `/home/yoga/m13/m13/6DOF_v4_pure/watchdog/results/20260504_031303`
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
- rocket_mpc: missing crash_complete event (iter 1)
