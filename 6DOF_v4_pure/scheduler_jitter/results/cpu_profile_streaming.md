# CPU Profile — streaming

PX4 PID: 2155, samples: 20, duration: 20s

## Total PX4 CPU usage (sum of all threads)

- mean: **83.9%**
- max:  92.3%
- stddev: 3.3%
- trend: **falling**

## Top threads by mean CPU

| Rank | TID | Name | mean% | max% | stddev% | trend |
|---|---|---|---|---|---|---|
| 1 | 2751 | `wq:manager` | 16.6 | 18.0 | 0.9 | steady |
| 2 | 2155 | `rdophone.px4v17` | 9.1 | 13.7 | 1.5 | steady |
| 3 | 2753 | `wq:INS0` | 9.0 | 10.3 | 0.6 | steady |
| 4 | 2713 | `rdophone.px4v17` | 8.6 | 10.3 | 1.0 | steady |
| 5 | 2762 | `mavlink_if0` | 8.4 | 9.0 | 0.6 | steady |
| 6 | 2752 | `wq:rate_ctrl` | 5.3 | 6.8 | 0.5 | steady |
| 7 | 2768 | `logger` | 5.2 | 6.8 | 0.7 | steady |
| 8 | 2723 | `px4` | 4.3 | 5.0 | 0.5 | steady |
| 9 | 2717 | `wkr_hrt` | 3.4 | 4.0 | 0.5 | steady |
| 10 | 2763 | `px4` | 2.8 | 3.4 | 0.4 | steady |
| 11 | 2759 | `wq:rocket_mpc` | 2.7 | 3.4 | 0.5 | steady |
| 12 | 2770 | `log_writer_file` | 2.2 | 3.4 | 0.5 | steady |
| 13 | 2758 | `wq:hp_default` | 1.3 | 2.0 | 0.6 | steady |
| 14 | 2755 | `commander` | 1.2 | 2.0 | 0.5 | steady |
| 15 | 2675 | `RenderThread` | 1.2 | 3.4 | 0.9 | oscillating |

## Oscillating threads (CV ≥ 0.7, mean ≥ 1%)

These show unstable CPU consumption — investigate scheduling/contention.

- TID 2675 `RenderThread` — mean 1.2%, stddev 0.9%
