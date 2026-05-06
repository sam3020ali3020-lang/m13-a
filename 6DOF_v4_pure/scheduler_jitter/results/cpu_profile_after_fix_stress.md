# CPU Profile — after_fix_stress

PX4 PID: 2315, samples: 15, duration: 15s

## Total PX4 CPU usage (sum of all threads)

- mean: **24.3%**
- max:  28.0%
- stddev: 2.4%
- trend: **steady**

## Top threads by mean CPU

| Rank | TID | Name | mean% | max% | stddev% | trend |
|---|---|---|---|---|---|---|
| 1 | 2315 | `rdophone.px4v17` | 4.2 | 5.0 | 0.8 | steady |
| 2 | 5066 | `wq:nav_ctrl` | 3.7 | 4.0 | 0.5 | steady |
| 3 | 5068 | `wq:INS0` | 2.9 | 3.1 | 0.3 | steady |
| 4 | 5077 | `mavlink_if0` | 2.7 | 3.1 | 0.5 | steady |
| 5 | 5024 | `rdophone.px4v17` | 1.8 | 3.1 | 0.6 | steady |
| 6 | 5067 | `wq:rate_ctrl` | 1.5 | 2.0 | 0.6 | steady |
| 7 | 5048 | `px4` | 1.3 | 2.0 | 0.6 | steady |
| 8 | 5084 | `logger` | 1.3 | 2.0 | 0.6 | steady |
| 9 | 5035 | `wkr_hrt` | 0.9 | 1.0 | 0.3 | steady |
| 10 | 5074 | `wq:rocket_mpc` | 0.9 | 3.1 | 0.8 | oscillating |
| 11 | 4995 | `RenderThread` | 0.7 | 1.0 | 0.5 | oscillating |
| 12 | 5086 | `log_writer_file` | 0.6 | 1.0 | 0.5 | oscillating |
| 13 | 5069 | `commander` | 0.5 | 1.0 | 0.5 | oscillating |
| 14 | 5080 | `mavlink_rcv_if0` | 0.2 | 1.0 | 0.4 | oscillating |
| 15 | 5072 | `wq:hp_default` | 0.2 | 1.0 | 0.4 | oscillating |
