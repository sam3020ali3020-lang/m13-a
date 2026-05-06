# /watchdog — Module Liveness Test Suite

Tests the on-phone `WatchdogManager` by injecting deliberate module crashes,
measuring detection / restart / recovery timing, and verifying that
dependent modules survive (no cascading failures).

## What this test exercises

| Layer | File |
|---|---|
| Native poll + restart dispatcher | `AndroidApp/app/src/main/cpp/watchdog_native.cpp` |
| JNI bindings | `AndroidApp/app/src/main/cpp/px4_jni.cpp` |
| Kotlin policy + broadcast receiver | `AndroidApp/app/src/main/java/.../watchdog/WatchdogManager.kt` |
| FlightService integration | `AndroidApp/app/src/main/java/.../service/FlightService.kt` |
| Python orchestrator + metrics | this directory |

The watchdog monitors the liveness of:

- `rocket_mpc`, `ekf2`, `sensors`, `commander`, `control_allocator`, `navigator`
  (PX4 modules — liveness inferred from their uORB publications).
- `native_sensor_reader` (Android IMU reader — counter heartbeat).
- `mavlink_tcp_bridge` (TCP↔UDP plumbing — heartbeat per poll iteration).

## Prerequisites

1. **Debug-signed APK installed.** Release builds refuse crash injection
   — see `WatchdogManager::isDebuggable()` check. Build with:
   ```bash
   cd AndroidApp && ./gradlew installDebug
   ```

2. **Phone connected via adb.**
   ```bash
   adb devices   # your device should appear as "device" (not "unauthorized")
   ```

3. **PX4 running.** Tap *Start PX4* on the phone. Verify:
   ```bash
   adb shell pidof com.ardophone.px4v17   # non-empty
   ```

4. **Python requirements.**
   ```bash
   pip install -r 6DOF_v4_pure/watchdog/requirements.txt
   ```

## Running

```bash
# preset: quick (~2 min, solo crash of every module once)
python3 6DOF_v4_pure/watchdog/watchdog_runner.py

# preset: standard (~5 min, adds manual_restart path)
python3 6DOF_v4_pure/watchdog/watchdog_runner.py --preset standard

# preset: full (~15 min, adds repeated + cascading)
python3 6DOF_v4_pure/watchdog/watchdog_runner.py --preset full

# one-shot: crash a single module and observe recovery
python3 6DOF_v4_pure/watchdog/watchdog_runner.py --module rocket_mpc

# run a specific scenario
python3 6DOF_v4_pure/watchdog/watchdog_runner.py --scenario cascading

# re-analyse an existing results dir (no device needed)
python3 6DOF_v4_pure/watchdog/watchdog_runner.py \
    --analyze-only 6DOF_v4_pure/watchdog/results/20260503_230000
```

## Outputs

Each run creates `results/YYYYMMDD_HHMMSS/` containing:

- `config.yaml` — copy of the config used
- `<scenario>_events.jsonl` — raw event log pulled from the phone
- `<scenario>_metrics.json` — per-iteration metrics + pass/fail
- `watchdog_report.md` — human-readable Markdown summary
- `watchdog_plot.html` — plotly bar chart (if `plotly` is installed)

## Pass/fail criteria

See `watchdog_config.yaml → thresholds`. Defaults:

| metric | threshold |
|---|---|
| `detection_ms_max` | 600 ms (stale + 1 poll period) |
| `restart_ms_max` | 1500 ms (stop + start round-trip) |
| `recovery_ms_max` | 2500 ms (first fresh signal after restart) |
| `bystander_recovery_ms_max` | 3000 ms (for cascading scenario) |

## Troubleshooting

- **`REJECT_RELEASE_BUILD`**: The APK on the phone is a release build;
  install debug (`./gradlew installDebug`).
- **`no iterations executed`**: The receiver didn't register — check that
  FlightService is running and PX4 was started. Re-open the app if needed.
- **`no events log on device`**: The watchdog's JSONL file doesn't exist
  at the expected path. Verify with:
  ```bash
  adb shell ls -la /sdcard/Android/data/com.ardophone.px4v17/files/px4/
  ```
  The app writes the log to `getExternalFilesDir(null)`; if your phone
  routes this differently, update `watchdog_config.yaml → device.log_path`.
- **Broadcast always returns `result=-1`**: `am broadcast` didn't wait
  for ordered delivery. This usually means the receiver isn't registered
  yet (FlightService hasn't finished `onStartCommand`), or the action
  doesn't match. Wait a few seconds after tapping *Start PX4*, then retry.

## Interpreting the report

- **solo_crash**: Each module must die and come back within thresholds.
  Expected behaviour is recovery_ms ≈ restart_ms + ~1 publish period.
- **repeated_crash**: All N repetitions should pass uniformly. Increasing
  recovery_ms across reps indicates leaked state (file handles, uORB
  advertisements, etc).
- **manual_restart**: With auto-restart off, the recovery_ms is dominated
  by the operator's `manual_restart_delay_s`. The metric of interest is
  `restart_ms` alone (should match solo_crash's restart timing).
- **cascading**: `victim` must recover, but each `bystander` must EITHER
  stay alive the whole time OR die briefly and recover within
  `bystander_recovery_ms_max`. A bystander that dies and doesn't recover
  indicates an unmodeled dependency (e.g. ekf2 keeping stale subscription
  handles to a restarted sensors).
