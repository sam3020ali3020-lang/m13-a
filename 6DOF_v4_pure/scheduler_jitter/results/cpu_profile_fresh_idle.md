# CPU Profile — fresh_idle

PX4 PID: 2155, samples: 20, duration: 20s

## Total PX4 CPU usage (sum of all threads)

- mean: **83.1%**
- max:  89.0%
- stddev: 2.7%
- trend: **steady**

## Top threads by mean CPU

| Rank | TID | Name | mean% | max% | stddev% | trend |
|---|---|---|---|---|---|---|
| 1 | 2751 | `wq:manager` | 16.7 | 18.0 | 1.0 | steady |
| 2 | 2155 | `rdophone.px4v17` | 9.6 | 12.0 | 1.1 | steady |
| 3 | 2713 | `rdophone.px4v17` | 9.3 | 11.0 | 0.8 | steady |
| 4 | 2753 | `wq:INS0` | 8.8 | 10.3 | 0.6 | steady |
| 5 | 2762 | `mavlink_if0` | 8.1 | 10.3 | 0.6 | steady |
| 6 | 2768 | `logger` | 5.3 | 6.0 | 0.7 | steady |
| 7 | 2752 | `wq:rate_ctrl` | 5.2 | 6.0 | 0.6 | steady |
| 8 | 2723 | `px4` | 4.2 | 6.0 | 0.7 | steady |
| 9 | 2717 | `wkr_hrt` | 3.3 | 4.0 | 0.4 | steady |
| 10 | 2759 | `wq:rocket_mpc` | 2.7 | 3.4 | 0.5 | steady |
| 11 | 2770 | `log_writer_file` | 2.0 | 3.0 | 0.6 | steady |
| 12 | 2758 | `wq:hp_default` | 1.4 | 2.0 | 0.6 | steady |
| 13 | 2755 | `commander` | 1.3 | 3.4 | 0.6 | steady |
| 14 | 2675 | `RenderThread` | 1.3 | 3.4 | 0.7 | steady |
| 15 | 2652 | `Jit thread pool` | 0.8 | 3.0 | 0.9 | oscillating |
