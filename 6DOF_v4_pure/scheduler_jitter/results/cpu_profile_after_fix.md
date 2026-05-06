# CPU Profile — after_fix

PX4 PID: 2315, samples: 20, duration: 20s

## Total PX4 CPU usage (sum of all threads)

- mean: **83.2%**
- max:  95.0%
- stddev: 10.0%
- trend: **falling**

## Top threads by mean CPU

| Rank | TID | Name | mean% | max% | stddev% | trend |
|---|---|---|---|---|---|---|
| 1 | 5066 | `wq:nav_ctrl` | 15.1 | 18.0 | 2.3 | steady |
| 2 | 2315 | `rdophone.px4v17` | 12.5 | 17.0 | 2.7 | falling |
| 3 | 5024 | `rdophone.px4v17` | 8.6 | 10.3 | 1.6 | steady |
| 4 | 5068 | `wq:INS0` | 8.5 | 10.3 | 1.0 | steady |
| 5 | 5077 | `mavlink_if0` | 7.7 | 9.0 | 0.7 | steady |
| 6 | 5067 | `wq:rate_ctrl` | 4.9 | 6.0 | 0.9 | steady |
| 7 | 5084 | `logger` | 4.7 | 6.0 | 0.9 | steady |
| 8 | 5048 | `px4` | 4.0 | 5.0 | 0.6 | steady |
| 9 | 5035 | `wkr_hrt` | 3.3 | 4.0 | 0.6 | steady |
| 10 | 5074 | `wq:rocket_mpc` | 2.7 | 5.0 | 0.8 | steady |
| 11 | 4986 | `Jit thread pool` | 2.1 | 5.0 | 1.4 | steady |
| 12 | 5086 | `log_writer_file` | 1.9 | 3.0 | 0.6 | steady |
| 13 | 5072 | `wq:hp_default` | 1.6 | 2.0 | 0.7 | steady |
| 14 | 4995 | `RenderThread` | 1.4 | 3.0 | 0.8 | steady |
| 15 | 5069 | `commander` | 1.1 | 2.0 | 0.5 | steady |
