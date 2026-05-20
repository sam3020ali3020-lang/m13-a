# v5.1 — Clean HIL Run Workflow

This is the **only** workflow that produced consistent range error across runs.
Skipping any of the marked CRITICAL steps causes parameter / state leakage
between runs and explains the ±25 % range-error spread observed earlier.

---

## Per-run procedure

```bash
# ── before each run ──────────────────────────────────────────
pkill -9 -f hil_runner                                         # ① kill any previous bridge
pkill -9 -f _thermal_quick                                     # ② kill any previous thermal sidecar
fuser -k 4560/tcp                                              # ③ free the MAVLink port
adb shell am force-stop com.ardophone.px4v17                   # CRITICAL — really kills the app process,
                                                              #             not just the UI activity
adb shell pm clear      com.ardophone.px4v17                   # CRITICAL — wipes EEPROM / persistent params
                                                              #             so RKT_MPC_SVO_DLY etc. start fresh
adb reverse tcp:4560 tcp:4560                                  # MAVLink HIL data
adb forward tcp:5760 tcp:5760                                  # MPC timing samples (else "no timing data")
adb logcat -c                                                  # ④ optional — clean log for diagnostics

# ── start the run ────────────────────────────────────────────
nohup python3 -u 6DOF_v4_pure/hil/hil_runner.py > /tmp/hil_run.log 2>&1 & disown
sleep 4                                                        # let the bridge bind to 4560
adb shell am start -n com.ardophone.px4v17/.MainActivity       # launch the UI
# → press START in the app — flight is ~14 s
```

The thermal sidecar starts itself from inside `run_hil()` and writes to
`<flight_csv_stem>_thermal.csv`, so the HTML report picks it up automatically.

---

## Why each "CRITICAL" step matters

### `am force-stop`
Pressing **STOP** in the app's UI only stops the simulation thread. The Android
process keeps running, which means PX4 modules keep their static state:

* `Ekf2` keeps the gyro / accel bias estimates from the previous flight. On the
  *next* `START`, tilt-alignment converges to the *previous* run's bias instead
  of re-converging from scratch — biases the new run's NED frame.
* `MhEstimator` (when active) holds its sliding window from the previous run.

`am force-stop` actually kills the process so all module constructors run again
on the next launch.

### `pm clear`
At the end of each run the on-target code calls `param save`, which persists a
handful of auto-tuned parameters. The most consequential one is:

```text
RKT_MPC_SVO_DLY  ← auto-tuned from the measured servo delay
```

This parameter sets the MPC `lookahead_stage`. Run 1 might measure 0.14 s and
save it. Run 2 starts with 0.14 s already loaded, measures 0.20 s, and saves
0.20 s. By Run 4 the lookahead is twice what Run 1 used → MPC predicts the
plant *much* further out → completely different fin commands for the same
flight phase.

`pm clear` deletes `/data/data/com.ardophone.px4v17/files/params.bin` (and the
rest of the app's data dir), forcing a fresh boot from the ROMFS defaults.

### `adb forward tcp:5760`
The MPC timing samples are streamed over a separate MAVLink connection on
port 5760. Without the forward, `hil_analysis.py` reports
*"MPC timing: no timing samples"* and the timing card in the HTML is empty.

---

## Symptom-to-cause table

| Symptom across runs | Likely missing step |
|---|---|
| Range error wanders by >5 % run-to-run | `pm clear` (`RKT_MPC_SVO_DLY` drift) |
| First-arm tilt-align takes 8 s (normally 3–4 s) | `am force-stop` (stale EKF2 biases) |
| HTML report has no MPC timing card | `adb forward tcp:5760` |
| HTML report has no CPU-temp card | `_thermal_quick.sh` not on disk or sidecar crashed (check `nohup` log) |
| `am start` says *"Activity not started, intent has been delivered to currently running top-most instance"* | The previous app process is still alive — re-run `am force-stop` |

---

## One-shot script (optional)

If you want to bake the whole sequence into a single command:

```bash
cat > /tmp/run_one.sh <<'SH'
#!/usr/bin/env bash
set -euo pipefail
APP=com.ardophone.px4v17
pkill -9 -f hil_runner    || true
pkill -9 -f _thermal_quick || true
fuser  -k 4560/tcp        || true
adb shell am force-stop "$APP"
adb shell pm clear        "$APP"
adb reverse tcp:4560 tcp:4560
adb forward tcp:5760 tcp:5760
adb logcat -c
nohup python3 -u 6DOF_v4_pure/hil/hil_runner.py > /tmp/hil_run.log 2>&1 &
disown
sleep 4
adb shell am start -n "$APP"/.MainActivity
echo "READY — press START in the app"
SH
chmod +x /tmp/run_one.sh
/tmp/run_one.sh
```
