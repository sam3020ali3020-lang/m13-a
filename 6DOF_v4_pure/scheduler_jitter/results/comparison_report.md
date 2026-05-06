# Scheduler Jitter — Scenario Comparison

Generated from internal PX4 timestamps (HRT). Wall/TCP figures are not used for pass/fail (they are affected by TCP buffering).


## HIGHRES_IMU (50 Hz, target 20 ms)

| Metric | baseline | light_load | heavy_load |
|---|---|---|---|
| count | 523 | 523 | 1524 |
| mean (ms) | 20.00 | 19.99 | 20.00 |
| stddev (ms) | 3.76 | 3.64 | 3.15 |
| p50 (ms) | 19.94 | 20.03 | 20.03 |
| p95 (ms) | 26.23 | 25.95 | 25.10 |
| p99 (ms) | 29.16 | 29.19 | 27.68 |
| p99.9 (ms) | 30.24 | 33.12 | 38.11 |
| max (ms) | 30.24 | 33.12 | 40.09 |
| late>2× (%) | 0.00 | 0.00 | 0.07 |
| late>3× (n) | 0 | 0 | 0 |
| dropped (n) | 0 | 0 | 0 |

Thresholds: stddev ≤ 8.0 ms, p99 ≤ 40.0 ms, late>2× ≤ 1.0%, late>3× ≤ 5, dropped ≤ 0.

## RktGNC (target 40 ms)

| Metric | baseline | light_load | heavy_load |
|---|---|---|---|
| count | 261 | 264 | 828 |
| mean (ms) | 40.09 | 39.69 | 36.81 |
| stddev (ms) | 4.84 | 5.19 | 5.40 |
| p50 (ms) | 40.53 | 40.42 | 38.92 |
| p95 (ms) | 47.19 | 47.34 | 43.25 |
| p99 (ms) | 49.41 | 50.87 | 46.33 |
| p99.9 (ms) | 56.39 | 55.47 | 53.81 |
| max (ms) | 56.39 | 55.47 | 53.81 |
| late>2× (%) | 0.00 | 0.00 | 0.00 |
| late>3× (n) | 0 | 0 | 0 |
| dropped (n) | 0 | 0 | 0 |

Thresholds: stddev ≤ 15.0 ms, p99 ≤ 80.0 ms, late>2× ≤ 2.0%, late>3× ≤ 5, dropped ≤ 0.

## Verdict

| Scenario | IMU | RktGNC | Overall |
|---|---|---|---|
| baseline | ✅ | ✅ | ✅ PASS |
| light_load | ✅ | ✅ | ✅ PASS |
| heavy_load | ✅ | ✅ | ✅ PASS |
