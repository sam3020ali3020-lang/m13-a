# CPU Profile — idle

PX4 PID: 23025, samples: 15, duration: 15s

## Total PX4 CPU usage (sum of all threads)

- mean: **126.6%**
- max:  130.0%
- stddev: 1.7%
- trend: **steady**

## Top threads by mean CPU

| Rank | TID | Name | mean% | max% | stddev% | trend |
|---|---|---|---|---|---|---|
| 1 | 26281 | `wq:manager` | 99.5 | 101.0 | 0.7 | steady |
| 2 | 26292 | `mavlink_if0` | 9.7 | 10.0 | 0.5 | steady |
| 3 | 26244 | `rdophone.px4v17` | 6.2 | 7.0 | 0.4 | steady |
| 4 | 23025 | `rdophone.px4v17` | 5.2 | 6.0 | 0.7 | steady |
| 5 | 26254 | `px4` | 2.4 | 3.3 | 0.5 | steady |
| 6 | 26284 | `commander` | 1.4 | 3.3 | 0.7 | steady |
| 7 | 26294 | `mavlink_rcv_if0` | 0.7 | 1.0 | 0.5 | oscillating |
| 8 | 26214 | `DefaultExecutor` | 0.4 | 1.0 | 0.5 | oscillating |
| 9 | 26256 | `DefaultDispatch` | 0.3 | 1.0 | 0.5 | oscillating |
| 10 | 26255 | `DefaultDispatch` | 0.3 | 1.0 | 0.5 | oscillating |
| 11 | 26313 | `px4_watchdog` | 0.1 | 1.0 | 0.3 | oscillating |
| 12 | 26247 | `lpwork` | 0.1 | 1.0 | 0.3 | oscillating |
| 13 | 26291 | `px4` | 0.1 | 1.0 | 0.3 | oscillating |
| 14 | 26290 | `px4` | 0.1 | 1.0 | 0.3 | oscillating |
| 15 | 26286 | `navigator` | 0.1 | 1.0 | 0.3 | oscillating |
