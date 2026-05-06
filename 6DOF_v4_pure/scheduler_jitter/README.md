# Scheduler Jitter Test — `/scheduler_jitter`

**Purpose:** empirically validate that the PX4 real-time priority setup on
Android (`nice=-20` + CPU affinity to big cores + `timerslack=1ns`) provides
sufficient scheduling determinism for MPC, given that `SCHED_FIFO` is
blocked on non-rooted Android.

The test measures **inter-arrival jitter** of two MAVLink streams under
three load scenarios and compares against pass/fail thresholds.

---

## Why this test exists

PX4's `rocket_mpc` module tries to set `SCHED_FIFO` priority, which fails on
Android userspace (no `CAP_SYS_NICE`). The fallback is `nice=-20`. Logs
show:

```
PX4.rocket_mpc: RT config: affinity(prime)=OK, SCHED_FIFO=FAIL, nice=-20
```

The question: **is `nice=-20` enough, or is the loss of `SCHED_FIFO` a
real problem for flight?**

This test answers it quantitatively by measuring jitter under:

1. **baseline** — idle phone
2. **light_load** — typical background apps (WhatsApp, camera)
3. **heavy_load** — all CPU cores pegged at 100% by `yes` processes

If jitter stays within thresholds even in `heavy_load`, then `SCHED_FIFO`
is empirically not needed on this hardware.

---

## What is measured

Two streams, captured for the configured duration:

| Stream | msg ID | requested rate | target interval |
|---|---|---|---|
| HIGHRES_IMU | 105 | 50 Hz | 20 ms |
| DEBUG_FLOAT_ARRAY (RktGNC, array_id 2) | 350 | 40 Hz | 40 ms (PX4 module caps at ~25 Hz) |

For each stream, we use the **internal PX4 timestamp** (`time_usec` inside
the payload) to compute inter-arrival intervals. This reflects PX4's
actual publishing cadence, independent of TCP buffering. Wall-clock
arrivals are also recorded for diagnostic purposes but are **not** used
for pass/fail (TCP bursts make them unreliable).

Statistics computed per stream:

- **mean**, **stddev** (jitter)
- **min**, **max**
- **p50**, **p95**, **p99**, **p99.9**
- **late >2× target** count and percentage
- **late >3× target** count
- **dropped estimate** (intervals > 5× target)

---

## Directory layout

```
scheduler_jitter/
├── README.md                     (this file)
├── jitter_config.yaml            (streams, scenarios, thresholds)
├── jitter_reader.py              (MAVLink TCP reader, captures raw timestamps)
├── jitter_analysis.py            (statistics, thresholds, comparison report)
├── jitter_runner.py              (CLI entry point)
├── requirements.txt
└── results/                      (output: json + md + png per scenario)
```

---

## Running

### Preconditions

- Phone connected via USB with `adb` authorized.
- PX4 app (`com.ardophone.px4v17`) running with "Start PX4" pressed.
- TCP port forwarded: `adb forward tcp:5760 tcp:5760`.

### Single scenario

```bash
python3 jitter_runner.py --scenario baseline
python3 jitter_runner.py --scenario heavy_load      # auto-spawns stress
```

### All scenarios sequentially

```bash
python3 jitter_runner.py --all
```

The runner prompts before `baseline` and `light_load` so you can prepare
the phone (close apps / open apps). `heavy_load` is fully automatic: it
spawns 8 `yes` processes on the phone via `adb shell`, runs the capture,
then kills them (`pkill yes`).

### Custom duration

```bash
python3 jitter_runner.py --scenario baseline --duration 60
```

---

## Pass/Fail criteria

Thresholds from `jitter_config.yaml` (per stream):

| Check | HIGHRES_IMU | RktGNC |
|---|---|---|
| stddev_ms | ≤ 8.0 | ≤ 15.0 |
| p99_ms | ≤ 40.0 (2× target) | ≤ 80.0 (2× target) |
| p99_9_ms | ≤ 60.0 (3× target) | ≤ 120.0 (3× target) |
| late_2x_pct | ≤ 1% | ≤ 2% |
| late_3x | ≤ 5 events total | ≤ 5 events total |
| dropped_est | 0 | 0 |

A scenario passes only if **both streams** pass all checks.

---

## Outputs

Per scenario → `results/jitter_<scenario>.json`:

```json
{
  "scenario": "heavy_load",
  "duration_s": 15.0,
  "overall_pass": true,
  "imu_stats": { "stddev_ms": 3.33, "p99_ms": 27.18, ... },
  "rkt_stats": { "stddev_ms": 5.55, "p99_ms": 47.18, ... },
  "imu_verdict": { "overall_pass": true, "checks": {...} },
  "rkt_verdict": { "overall_pass": true, "checks": {...} }
}
```

When multiple scenarios are run (`--all` or manual), also produces:

- `results/comparison_report.md` — side-by-side Markdown table
- `results/jitter_histograms.png` — overlayed histograms (if matplotlib
  available)

---

## Interpreting results

### ✅ Expected outcome on a healthy phone

Across all three scenarios, **internal PX4 timing stays within thresholds**:

- stddev ≈ 3–6 ms for both streams
- p99 ≈ 1.2–1.5× target
- zero dropped packets

Example from a Snapdragon 845 reference device:

| Metric (IMU internal) | baseline | light_load | heavy_load |
|---|---|---|---|
| stddev (ms) | 3.99 | 3.91 | 3.33 |
| p99 (ms) | 30.49 | 29.59 | 27.18 |
| late >2× | 1 | 3 | 1 |

Counter-intuitively, `heavy_load` was **better** than `baseline` on that
device — because the CPU governor boosted frequency under load while idle
saw conservative power-save scaling. This is a strong signal that
`nice=-20 + affinity` is sufficient.

### ❌ If a scenario fails

- **stddev_ms exceeded** → consider tighter CPU pinning or disable a
  competing thread.
- **p99_ms exceeded but late_3x = 0** → borderline; check thermal status
  (`adb shell cat /sys/class/thermal/thermal_zone*/temp`).
- **dropped > 0** → PX4 is losing cycles; inspect `logcat -s PX4`.
- **heavy_load worse than baseline by a large margin** → the RT fallback
  is leaking priority; re-check `RocketMPC.cpp` priority-setup code path.

---

## Relationship to other tests

- **`/direct`** measures pure hardware delay (servo + CAN). No scheduler
  interaction.
- **`/e2e_latency`** measures end-to-end latency including MPC solve time,
  but averages out jitter.
- **`/ground`** integrates EKF2 + MPC behavior but does not isolate
  scheduler timing.
- **`/scheduler_jitter`** (this test) isolates the scheduler question.

---

## Caveats

1. `heavy_load` uses `yes` processes which run at default Android nice
   value (0). The test validates that `nice=-20` protects MPC from
   default-priority interference — this is the realistic scenario on a
   phone. It does **not** validate against other SCHED_FIFO threads
   (there are none in a standard Android image besides Audio/UI, which
   run on fixed CPUs anyway).
2. Thermal throttling can degrade timing late in a long test. The default
   durations (30 s / 15 s) are short enough to avoid steady-state thermal
   events. For thermal characterization use `/thermal_stress`.
3. TCP buffering artifacts appear in the "wall/TCP" stats — these are
   recorded for reference but **not** used in pass/fail decisions.
