/**
 * watchdog_native.h — Per-module liveness watchdog API.
 *
 * See watchdog_native.cpp for the full design notes.  This header exposes
 * the C-ABI surface used by both px4_jni.cpp (JNI glue) and direct C++
 * callers such as FlightService initialisation.
 */
#pragma once

#include <cstddef>
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

// Snapshot returned by wd_get_module_status().  All *_us timestamps are in
// the hrt_absolute_time() domain (microseconds since boot on NuttX, or the
// lockstep sim clock on Android when the simulator is active).
typedef struct {
    bool      alive;            // fresh signal within stale_us
    bool      was_alive_ever;   // latch — has this module ever produced a signal?
    bool      auto_restart;     // current policy (read-only snapshot)
    uint64_t  last_alive_us;    // most recent observed signal
    uint64_t  last_tick_us;     // time watchdog last polled this module
    uint64_t  last_crash_us;    // last wd_crash_module() success
    uint64_t  last_restart_us;  // last wd_restart_module() or auto-restart completion
    uint64_t  last_recover_us;  // first alive-again time after a crash/restart
    uint32_t  crash_count;      // total crashes injected (test) + detected
    uint32_t  restart_count;    // total (manual + auto) restarts
    uint64_t  stale_us;         // configured threshold for "dead"
    uint64_t  now_us;           // hrt_absolute_time() at read
    uint64_t  age_us;           // now_us - last_alive_us (UINT64_MAX if never alive)
} wd_status_t;

// Initialise the watchdog: opens the JSONL log file and spawns the poll
// thread.  `log_file_path` may be NULL or empty to disable file logging
// (in-memory ring buffer still active).  `poll_period_ms == 0` selects the
// default (50 ms).  Returns false if already initialised or on setup error.
bool   wd_init(const char* log_file_path, uint32_t poll_period_ms);

// Stop the poll thread and close the log.  Safe to call at Android
// onDestroy or stopPX4.  Not idempotent — calling wd_init again after
// shutdown is unsupported (module registry rebuild path not implemented).
void   wd_shutdown();

// Fill *out with the latest status snapshot for the named module.  Returns
// false if not initialised or the module is not registered.
bool   wd_get_module_status(const char* name, wd_status_t* out);

// List registered module names into names_out[0..n).  Returns the number
// of names written (may be less than max_names).  The returned pointers
// are owned by the watchdog and valid until wd_shutdown().
size_t wd_list_modules(const char** names_out, size_t max_names);

// Enable/disable auto-restart for a single module.  Kotlin sets this per
// module according to config + flight state (pre-launch vs armed).
bool   wd_set_auto_restart(const char* name, bool enable);

// Test-only: stop the module synchronously (simulates a crash).  The next
// poll cycle will observe the liveness signal going stale, mark the
// module dead, and — if auto_restart is on — restart it.
bool   wd_crash_module(const char* name);

// Manual restart (stop+start).  Logs "restart_requested" + "restart_*".
bool   wd_restart_module(const char* name);

// Copy the last <= max_events ring-buffer entries into json_out as a JSON
// array (null-terminated).  Returns bytes written (including the closing
// ']' and NUL).  Truncates gracefully if max_bytes is insufficient.
// Use max_events == 0 to copy all available.
size_t wd_copy_recent_events(char* json_out, size_t max_bytes,
                             size_t max_events);

// Clear the on-device JSONL log by close + reopen with O_TRUNC.  This
// must be used instead of `adb shell rm` because the log file is held
// open by the poll thread: unlinking from the shell leaves the fd
// writing to an unreachable inode and the file never reappears.
// Returns false if not initialised, or reopen failed (in which case
// the in-memory ring buffer is still available via wd_copy_recent_events).
// Intended for use by the /watchdog test runner between scenarios.
bool   wd_truncate_log();

#ifdef __cplusplus
}
#endif
