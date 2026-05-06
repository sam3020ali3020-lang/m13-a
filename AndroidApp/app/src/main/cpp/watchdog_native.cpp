/**
 * watchdog_native.cpp — Per-module liveness watchdog + soft-restart.
 *
 * On NuttX the RTOS kernel restarts crashed tasks.  On Android the PX4
 * modules live inside our own process and a silent crash leaves the
 * remaining modules running on stale data — which on a rocket means
 * uncontrolled flight.  This file provides:
 *
 *   1.  A background poll thread that checks a liveness signal for each
 *       registered module every `poll_period_ms` (default 50 ms).
 *
 *   2.  Per-module status cache (atomic) readable lock-free from JNI or
 *       from Kotlin.
 *
 *   3.  A JSONL event log written to a file that the /watchdog test can
 *       pull with `adb pull`.  In-memory ring buffer also retained for
 *       quick inspection via JNI.
 *
 *   4.  A public API for the test harness / FlightService:
 *         wd_crash_module(name)        — invoke "<name> stop" (simulated crash)
 *         wd_restart_module(name)      — invoke "<name> start" with proper argv
 *         wd_set_auto_restart(name, b) — enable/disable per-module auto-restart
 *
 * LIVENESS SIGNALS
 *   rocket_mpc   : rocket_gnc_status.timestamp       (publishes @ ~50 Hz)
 *   ekf2         : vehicle_attitude.timestamp         (publishes @ 200 Hz)
 *   sensors      : sensor_combined.timestamp          (publishes @ 200 Hz)
 *   commander    : vehicle_status.timestamp           (publishes @ 5 Hz)
 *   navigator    : position_setpoint_triplet.timestamp (publishes on demand)
 *   control_allocator : actuator_servos.timestamp     (publishes @ 400 Hz)
 *   native_sensor_reader : g_sensor_counts.imu counter (should tick @ ~200 Hz)
 *   mavlink      : mavlink_tcp_bridge_alive_us() heartbeat (set by bridge TX)
 *
 * If the signal hasn't advanced in stale_threshold_us, the module is DEAD.
 *
 * SAFETY NOTE
 *   wd_crash_module() is a TEST-ONLY backdoor.  In production flight code we
 *   only ever call wd_restart_module() automatically (and only for modules
 *   whose config has auto_restart enabled).  The BroadcastReceiver that
 *   dispatches "crash" commands MUST be gated by a debuggable-build check
 *   at the Kotlin layer — see WatchdogManager.kt.
 */

#include "watchdog_native.h"
#include "native_sensor_reader.h"
#include "mavlink_tcp_bridge.h"
#include "android_uorb_publishers.h"  // start/stop_uorb_publishers

#include <android/log.h>
#include <pthread.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <drivers/drv_hrt.h>
#include <uORB/Subscription.hpp>
// The topic headers are needed for ORB_ID(name) macro expansion.  We don't
// access any struct fields directly — probe_uorb_timestamp() treats the
// buffer as opaque bytes and only reads the leading uint64_t timestamp
// common to every PX4 uORB message.
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/rocket_gnc_status.h>

#define WD_TAG "PX4Watchdog"
#define WD_LOGI(...) __android_log_print(ANDROID_LOG_INFO,  WD_TAG, __VA_ARGS__)
#define WD_LOGW(...) __android_log_print(ANDROID_LOG_WARN,  WD_TAG, __VA_ARGS__)
#define WD_LOGE(...) __android_log_print(ANDROID_LOG_ERROR, WD_TAG, __VA_ARGS__)

// =============================================================================
// PX4 module entry points (same symbols used by px4_jni.cpp)
// =============================================================================
extern "C" {
    int rocket_mpc_main(int argc, char *argv[]);
    int ekf2_main(int argc, char *argv[]);
    int sensors_main(int argc, char *argv[]);
    int commander_main(int argc, char *argv[]);
    int navigator_main(int argc, char *argv[]);
    int control_allocator_main(int argc, char *argv[]);
    int mavlink_main(int argc, char *argv[]);
    int land_detector_main(int argc, char *argv[]);
    int load_mon_main(int argc, char *argv[]);
    int logger_main(int argc, char *argv[]);
}

// =============================================================================
// Module registry — each entry describes how to observe and restart a module
// =============================================================================

namespace {

// Per-module runtime state — protected by global s_mod_mutex; atomics for
// fields read lock-free from JNI.
struct ModuleState {
    // --- static (filled by register_modules) ---
    std::string  name;
    uint64_t     stale_us       = 0;     // missed-heartbeat threshold
    bool         is_native      = false; // true = uses native_start/stop, not PX4 main()
    int (*px4_main)(int, char**) = nullptr; // for is_native == false

    // For uORB-based liveness: keep a subscription alive inside the watchdog
    // thread and pull timestamp on each poll.  Pointer so we can defer
    // construction until after uORB is initialised.
    uORB::Subscription *uorb_sub = nullptr;
    const orb_metadata *uorb_meta = nullptr;
    // For counter-based liveness (native_sensor_reader)
    std::atomic<uint64_t>      *counter = nullptr;
    uint64_t                    last_counter_value = 0;
    // For mavlink bridge heartbeat
    uint64_t (*bridge_alive_fn)() = nullptr;

    // --- runtime ---
    std::atomic<uint64_t> last_alive_us{0};      // most recent observed signal
    std::atomic<uint64_t> last_tick_us{0};       // time we last polled
    std::atomic<bool>     alive{false};
    std::atomic<bool>     was_alive_ever{false}; // first-time-started latch
    std::atomic<bool>     auto_restart{false};
    std::atomic<uint32_t> crash_count{0};
    std::atomic<uint32_t> restart_count{0};
    std::atomic<uint64_t> last_crash_us{0};
    std::atomic<uint64_t> last_restart_us{0};
    std::atomic<uint64_t> last_recover_us{0};    // when state flipped back to alive
};

// Module lookup is rare (crash/restart commands) and we stay well under 20
// modules so a flat vector + linear search is the right data structure.
static std::vector<ModuleState*> s_modules;
static std::mutex s_mod_mutex;  // guards vector ownership, NOT atomics within

static ModuleState* find_module(const char* name) {
    if (!name) return nullptr;
    std::lock_guard<std::mutex> lock(s_mod_mutex);
    for (auto* m : s_modules) {
        if (m->name == name) return m;
    }
    return nullptr;
}

// =============================================================================
// Event log — JSONL file + in-memory ring buffer
// =============================================================================

struct LogEntry {
    uint64_t    t_us;
    std::string event;   // "alive" | "dead" | "restart_requested" | "restart_complete" | ...
    std::string module;
    std::string note;    // extra data serialized as "key=val key2=val2"
};

static constexpr size_t kLogRingSize = 512;
static LogEntry              s_log_ring[kLogRingSize];
static std::atomic<uint64_t> s_log_ring_head{0};
static std::mutex            s_log_file_mutex;
static FILE*                 s_log_fp = nullptr;
static std::string           s_log_path;

static void log_event(const char* event, const char* module,
                      const char* note_fmt, ...) {
    char note_buf[256] = {0};
    if (note_fmt) {
        va_list ap;
        va_start(ap, note_fmt);
        vsnprintf(note_buf, sizeof(note_buf) - 1, note_fmt, ap);
        va_end(ap);
    }
    const uint64_t t_us = hrt_absolute_time();

    // 1) In-memory ring
    const uint64_t idx = s_log_ring_head.fetch_add(1) % kLogRingSize;
    s_log_ring[idx].t_us   = t_us;
    s_log_ring[idx].event  = event ? event : "";
    s_log_ring[idx].module = module ? module : "";
    s_log_ring[idx].note   = note_buf;

    // 2) JSONL file — best effort (non-fatal on failure)
    {
        std::lock_guard<std::mutex> lock(s_log_file_mutex);
        if (s_log_fp) {
            // Escape quotes in note (simple — we control all call-sites)
            std::string esc_note;
            for (char c : std::string(note_buf)) {
                if (c == '"' || c == '\\') esc_note += '\\';
                if (c == '\n') { esc_note += "\\n"; continue; }
                esc_note += c;
            }
            fprintf(s_log_fp,
                "{\"t_us\":%llu,\"event\":\"%s\",\"module\":\"%s\",\"note\":\"%s\"}\n",
                (unsigned long long)t_us,
                event ? event : "",
                module ? module : "",
                esc_note.c_str());
            fflush(s_log_fp);
        }
    }

    // 3) Android log for quick adb logcat debugging
    WD_LOGI("[%s] %s%s%s",
            event ? event : "",
            module ? module : "",
            note_buf[0] ? " " : "",
            note_buf);
}

// =============================================================================
// Liveness probe helpers
// =============================================================================

// Pull the latest timestamp for a uORB-backed module.  Returns 0 if the
// topic has never published, i.e. module never started.
//
// Sized for the largest topic we monitor.  rocket_gnc_status is the
// biggest (~500 bytes at the time of writing — many float fields for the
// MPC/MHE diagnostic snapshot).  2 kB is comfortably large for any PX4
// topic; sensor_combined, vehicle_status etc. are all well below 1 kB.
static uint64_t probe_uorb_timestamp(ModuleState* m) {
    if (!m->uorb_sub) return 0;
    alignas(8) uint8_t buf[2048];
    // copy() is non-advancing: repeated calls on the same generation
    // just re-read the latest sample without marking it consumed, so
    // we do not interfere with any other subscriber to the same topic.
    if (!m->uorb_sub->copy(buf)) {
        return 0;
    }
    // All PX4 uORB messages begin with `uint64_t timestamp;` — this is a
    // documented convention enforced by the message generator.  Read it
    // with memcpy to avoid aliasing UB.
    uint64_t ts = 0;
    std::memcpy(&ts, buf, sizeof(ts));
    return ts;
}

// Returns "signal time" for counter-based modules — a virtual timestamp
// that advances in HRT microseconds whenever the counter increments.
static uint64_t probe_counter(ModuleState* m, uint64_t now_us) {
    if (!m->counter) return 0;
    const uint64_t v = m->counter->load(std::memory_order_relaxed);
    if (v != m->last_counter_value) {
        m->last_counter_value = v;
        return now_us;          // signal "seen now"
    }
    // No change — return previous signal time
    return m->last_alive_us.load(std::memory_order_relaxed);
}

// =============================================================================
// Start / stop dispatch
// =============================================================================

// Build argv for each PX4 module.  Note some modules need specific args
// (land_detector fixedwing, logger -f -t) but none of those are in the
// watchdog-monitored set so we use simple "module start" / "module stop".
// Land_detector etc. are started from px4_jni.cpp and if they crash we
// log it but don't touch them (their failure is not flight-critical).
static int invoke_main(ModuleState* m, const char* verb) {
    if (m->is_native || !m->px4_main) return -1;
    const char* argv[] = { m->name.c_str(), verb, nullptr };
    return m->px4_main(2, (char**)argv);
}

// =============================================================================
// Public API — crash / restart
// =============================================================================

static std::atomic<bool> s_initialized{false};

static int wd_start_module_locked(ModuleState* m) {
    // Reset counter baseline so the first increment after restart is seen
    // as "alive" (otherwise the counter might not move for 1 poll).
    if (m->counter) {
        m->last_counter_value = m->counter->load(std::memory_order_relaxed);
    }
    if (m->is_native) {
        // native: dispatch by module name to the right start hook
        if (m->name == "native_sensor_reader") {
            native_sensor_start();
            return 0;
        }
        if (m->name == "mavlink_tcp_bridge") {
            // Port & forward are fixed in px4_jni.cpp; re-use the same
            // values if ever restarted by the watchdog.  Leaving them
            // hard-coded here is intentional — the watchdog is not the
            // right place to centralise configuration.  start() is
            // void-returning; treat completion of the call as success.
            mavlink_tcp_bridge_start(5760, 14550);
            return 0;
        }
        return -1;
    }
    return invoke_main(m, "start");
}

static int wd_stop_module_locked(ModuleState* m) {
    if (m->is_native) {
        if (m->name == "native_sensor_reader") {
            native_sensor_stop();
            return 0;
        }
        if (m->name == "mavlink_tcp_bridge") {
            mavlink_tcp_bridge_stop();
            return 0;
        }
        return -1;
    }
    return invoke_main(m, "stop");
}

} // namespace

// =============================================================================
// Public API implementations
// =============================================================================

extern "C" {

bool wd_crash_module(const char* name) {
    if (!s_initialized.load()) return false;
    ModuleState* m = find_module(name);
    if (!m) {
        WD_LOGW("crash: unknown module '%s'", name ? name : "(null)");
        return false;
    }
    log_event("crash_requested", m->name.c_str(), "test_backdoor");
    const int rc = wd_stop_module_locked(m);
    m->crash_count.fetch_add(1);
    m->last_crash_us.store(hrt_absolute_time());
    log_event(rc == 0 ? "crash_complete" : "crash_failed",
              m->name.c_str(), "rc=%d", rc);
    return rc == 0;
}

bool wd_restart_module(const char* name) {
    if (!s_initialized.load()) return false;
    ModuleState* m = find_module(name);
    if (!m) {
        WD_LOGW("restart: unknown module '%s'", name ? name : "(null)");
        return false;
    }
    log_event("restart_requested", m->name.c_str(), "caller=manual");
    const uint64_t t0 = hrt_absolute_time();
    // Stop + small sleep + start.  The stop may be a no-op if the module
    // already died, but it's idempotent for all PX4 modules we monitor.
    wd_stop_module_locked(m);
    usleep(50000); // 50ms — give uORB publishers time to finalise
    const int rc = wd_start_module_locked(m);
    const uint64_t dt = hrt_absolute_time() - t0;
    m->restart_count.fetch_add(1);
    m->last_restart_us.store(hrt_absolute_time());
    log_event(rc == 0 ? "restart_complete" : "restart_failed",
              m->name.c_str(), "rc=%d duration_us=%llu",
              rc, (unsigned long long)dt);
    return rc == 0;
}

bool wd_set_auto_restart(const char* name, bool enable) {
    if (!s_initialized.load()) return false;
    ModuleState* m = find_module(name);
    if (!m) return false;
    m->auto_restart.store(enable);
    log_event("auto_restart_set", m->name.c_str(),
              "enabled=%d", enable ? 1 : 0);
    return true;
}

bool wd_get_module_status(const char* name, wd_status_t* out) {
    if (!s_initialized.load() || !out) return false;
    ModuleState* m = find_module(name);
    if (!m) return false;
    const uint64_t now = hrt_absolute_time();
    out->alive            = m->alive.load();
    out->was_alive_ever   = m->was_alive_ever.load();
    out->auto_restart     = m->auto_restart.load();
    out->last_alive_us    = m->last_alive_us.load();
    out->last_tick_us     = m->last_tick_us.load();
    out->last_crash_us    = m->last_crash_us.load();
    out->last_restart_us  = m->last_restart_us.load();
    out->last_recover_us  = m->last_recover_us.load();
    out->crash_count      = m->crash_count.load();
    out->restart_count    = m->restart_count.load();
    out->stale_us         = m->stale_us;
    out->now_us           = now;
    // Guard against the clamp being bypassed elsewhere (e.g. a reader
    // racing with a freshly-updated last_alive_us).  Treat "future"
    // timestamps as age=0 rather than letting uint64 underflow.
    out->age_us           = out->last_alive_us
                              ? (now >= out->last_alive_us
                                   ? now - out->last_alive_us
                                   : 0)
                              : UINT64_MAX;
    return true;
}

size_t wd_list_modules(const char** names_out, size_t max_names) {
    if (!s_initialized.load()) return 0;
    std::lock_guard<std::mutex> lock(s_mod_mutex);
    const size_t n = s_modules.size() < max_names ? s_modules.size() : max_names;
    for (size_t i = 0; i < n; ++i) {
        names_out[i] = s_modules[i]->name.c_str();
    }
    return n;
}

bool wd_truncate_log() {
    if (!s_initialized.load()) return false;
    std::lock_guard<std::mutex> lock(s_log_file_mutex);
    if (s_log_path.empty()) return false;

    // Close current fd first; if we skip this step and fopen(path, "w")
    // succeeds on a different inode (because an earlier `rm` unlinked
    // the visible path), the stale fd would keep writing to the ghost
    // inode until wd_shutdown().
    if (s_log_fp) {
        fclose(s_log_fp);
        s_log_fp = nullptr;
    }

    // "w" truncates to zero length if the path resolves to an existing
    // file, or creates a fresh one otherwise — in both cases we end up
    // with an empty, visible file that subsequent log_event() calls
    // will populate.
    s_log_fp = fopen(s_log_path.c_str(), "w");
    if (!s_log_fp) {
        WD_LOGE("wd_truncate_log: reopen failed for '%s'", s_log_path.c_str());
        return false;
    }
    WD_LOGI("log truncated and reopened: %s", s_log_path.c_str());
    return true;
}

} // extern "C"

// =============================================================================
// Poll thread + lifecycle
// =============================================================================

namespace {

static std::atomic<bool> s_running{false};
static std::atomic<uint32_t> s_poll_period_ms{50};
static pthread_t s_thread{};

static void poll_once() {
    const uint64_t now = hrt_absolute_time();

    std::vector<ModuleState*> snapshot;
    {
        std::lock_guard<std::mutex> lock(s_mod_mutex);
        snapshot = s_modules; // copy pointers (cheap)
    }

    for (ModuleState* m : snapshot) {
        uint64_t signal_us = 0;
        if (m->uorb_sub) {
            signal_us = probe_uorb_timestamp(m);
        } else if (m->counter) {
            signal_us = probe_counter(m, now);
        } else if (m->bridge_alive_fn) {
            signal_us = m->bridge_alive_fn();
        }

        m->last_tick_us.store(now, std::memory_order_relaxed);
        const bool was_alive = m->alive.load(std::memory_order_relaxed);

        // Update last_alive_us monotonically — signal_us from uORB may
        // stutter backwards if an older sample is re-read, which would
        // spuriously flip alive=false.  We also clamp to `now` to guard
        // against a publisher thread that wrote its timestamp AFTER we
        // sampled `now` at the top of the poll cycle: such a timestamp
        // would be (now + ε) and the subsequent (now - last_alive)
        // subtraction would underflow in uint64 — producing a bogus
        // "dead" edge with age_us ≈ UINT64_MAX for a perfectly healthy
        // module.  Clamping here is the cheapest fix and preserves the
        // intended semantics ("last time we observed the module alive").
        if (signal_us > 0) {
            const uint64_t clamped = (signal_us > now) ? now : signal_us;
            uint64_t prev = m->last_alive_us.load(std::memory_order_relaxed);
            if (clamped > prev) {
                m->last_alive_us.store(clamped, std::memory_order_relaxed);
                m->was_alive_ever.store(true, std::memory_order_relaxed);
            }
        }

        const uint64_t last_alive = m->last_alive_us.load(std::memory_order_relaxed);
        // Use addition (last_alive + stale_us >= now) rather than
        // subtraction (now - last_alive <= stale_us) for safety: the
        // addition form cannot underflow even if a future clamp is
        // skipped.  Both are equivalent when last_alive <= now.
        const bool is_alive = (last_alive != 0) && (last_alive + m->stale_us >= now);
        m->alive.store(is_alive, std::memory_order_relaxed);

        // Edge detection
        if (!was_alive && is_alive) {
            // alive edge
            if (m->last_crash_us.load() > 0 || m->restart_count.load() > 0) {
                m->last_recover_us.store(now, std::memory_order_relaxed);
            }
            log_event("alive", m->name.c_str(),
                      "signal_us=%llu", (unsigned long long)last_alive);
        } else if (was_alive && !is_alive) {
            // dead edge — age_us is guaranteed non-negative by the
            // clamp above (last_alive <= now), so the subtraction is safe.
            const uint64_t age = (now >= last_alive) ? (now - last_alive) : 0;
            log_event("dead", m->name.c_str(),
                      "age_us=%llu stale_thr_us=%llu",
                      (unsigned long long)age,
                      (unsigned long long)m->stale_us);

            if (m->auto_restart.load()) {
                log_event("auto_restart_triggered", m->name.c_str(), nullptr);
                // Do restart OUTSIDE the critical section — the restart
                // itself may take >100ms and we don't want to block other
                // modules' polling.  We detach a helper thread to run it
                // asynchronously; next poll will observe the result.
                std::thread([m]() {
                    wd_stop_module_locked(m);
                    usleep(50000);
                    const int rc = wd_start_module_locked(m);
                    m->restart_count.fetch_add(1);
                    m->last_restart_us.store(hrt_absolute_time());
                    log_event(rc == 0 ? "auto_restart_complete" : "auto_restart_failed",
                              m->name.c_str(), "rc=%d", rc);
                }).detach();
            }
        }
    }
}

// Defense-in-depth: every `kLogRecheckPeriodUs` we stat() the log path
// and reopen if it disappeared.  Protects against external processes
// (adb shell rm, Android storage GC, etc.) deleting the file while we
// hold it open — in that state our fd keeps writing to an orphan inode
// that nobody can adb-pull.  Reopen is cheap and only happens if the
// file is truly gone.
static constexpr uint64_t kLogRecheckPeriodUs = 5ULL * 1000 * 1000; // 5s

static void maybe_reopen_log_if_deleted() {
    std::string path_copy;
    {
        std::lock_guard<std::mutex> lock(s_log_file_mutex);
        if (!s_log_fp || s_log_path.empty()) return;
        path_copy = s_log_path;
    }
    struct stat st;
    if (stat(path_copy.c_str(), &st) == 0) return;  // exists, nothing to do

    // File vanished.  Close the stale fd (so the orphan inode is freed)
    // and reopen in append mode to keep accumulating events.
    WD_LOGW("log file '%s' missing — reopening", path_copy.c_str());
    std::lock_guard<std::mutex> lock(s_log_file_mutex);
    if (s_log_fp) {
        fclose(s_log_fp);
        s_log_fp = nullptr;
    }
    s_log_fp = fopen(s_log_path.c_str(), "a");
    if (!s_log_fp) {
        WD_LOGE("reopen after external delete failed: '%s'", s_log_path.c_str());
    }
}

static void* poll_thread_fn(void*) {
    // Rename for profilers / thread dumps — helps when diagnosing ANRs.
    pthread_setname_np(pthread_self(), "px4_watchdog");
    WD_LOGI("watchdog poll thread running (period=%u ms)",
            s_poll_period_ms.load());
    uint64_t last_recheck_us = 0;
    while (s_running.load()) {
        poll_once();
        // File-existence check runs on a separate cadence (5s) to avoid
        // stat() syscall overhead on every 50ms tick.
        const uint64_t now = hrt_absolute_time();
        if (now - last_recheck_us >= kLogRecheckPeriodUs) {
            last_recheck_us = now;
            maybe_reopen_log_if_deleted();
        }
        usleep(s_poll_period_ms.load() * 1000u);
    }
    WD_LOGI("watchdog poll thread exiting");
    return nullptr;
}

// ----- module registration helpers ---------------------------------------

static ModuleState* new_px4_module(const char* name,
                                    int (*main_fn)(int, char**),
                                    const orb_metadata* meta,
                                    uint32_t stale_ms) {
    auto* m = new ModuleState();
    m->name         = name;
    m->stale_us     = (uint64_t)stale_ms * 1000u;
    m->is_native    = false;
    m->px4_main     = main_fn;
    m->uorb_meta    = meta;
    m->uorb_sub     = new uORB::Subscription(meta);
    return m;
}

static ModuleState* new_native_module(const char* name, uint32_t stale_ms) {
    auto* m = new ModuleState();
    m->name      = name;
    m->stale_us  = (uint64_t)stale_ms * 1000u;
    m->is_native = true;
    return m;
}

} // namespace

extern "C" bool wd_init(const char* log_file_path, uint32_t poll_period_ms) {
    if (s_initialized.load()) {
        WD_LOGW("wd_init called twice — ignoring");
        return false;
    }

    s_poll_period_ms.store(poll_period_ms == 0 ? 50 : poll_period_ms);

    // ---- open log file (JSONL) ----
    {
        std::lock_guard<std::mutex> lock(s_log_file_mutex);
        if (log_file_path && log_file_path[0]) {
            s_log_path = log_file_path;
            s_log_fp = fopen(log_file_path, "a");
            if (!s_log_fp) {
                WD_LOGW("cannot open watchdog log '%s' — in-memory only",
                        log_file_path);
            } else {
                WD_LOGI("watchdog log: %s", log_file_path);
            }
        }
    }

    // ---- register modules ----
    {
        std::lock_guard<std::mutex> lock(s_mod_mutex);
        s_modules.clear();

        // FLIGHT-CRITICAL (monitored tightly)
        //   rocket_mpc — MPC runs at ~50 Hz.  200ms = 10 missed cycles.
        s_modules.push_back(new_px4_module(
            "rocket_mpc", rocket_mpc_main,
            ORB_ID(rocket_gnc_status), 200));

        //   ekf2 — runs on sensor_combined callback @ 200 Hz.  100ms = 20 cycles.
        s_modules.push_back(new_px4_module(
            "ekf2", ekf2_main,
            ORB_ID(vehicle_attitude), 100));

        //   sensors — vehicle_* topics at 200 Hz.  100ms = 20 cycles.
        s_modules.push_back(new_px4_module(
            "sensors", sensors_main,
            ORB_ID(sensor_combined), 100));

        //   commander — vehicle_status at 5 Hz (200ms period).  The
        //   original 500ms threshold was only 2.5× the nominal cycle
        //   which produced spurious "dead" edges whenever commander
        //   was slightly delayed (seen as e.g. age_us=506985).  1000ms
        //   (5× cycle) keeps detection latency reasonable while
        //   tolerating the occasional scheduler hiccup.  commander is
        //   flight_critical=false so widening the window does not
        //   compromise safety.
        s_modules.push_back(new_px4_module(
            "commander", commander_main,
            ORB_ID(vehicle_status), 1000));

        //   control_allocator — actuator_servos at 400 Hz when MPC is running.
        //   But can be silent pre-launch — allow 500ms.
        s_modules.push_back(new_px4_module(
            "control_allocator", control_allocator_main,
            ORB_ID(actuator_servos), 500));

        //   navigator — publishes only on demand; give it 5s.
        s_modules.push_back(new_px4_module(
            "navigator", navigator_main,
            ORB_ID(position_setpoint_triplet), 5000));

        // NATIVE components (counter/heartbeat based)
        //   native_sensor_reader — counter ticks at ~200 Hz (IMU).  100ms.
        {
            auto* m = new_native_module("native_sensor_reader", 100);
            m->counter = &g_sensor_counts.imu;
            s_modules.push_back(m);
        }
        //   mavlink_tcp_bridge — has its own heartbeat function.
        {
            auto* m = new_native_module("mavlink_tcp_bridge", 2000);
            m->bridge_alive_fn = mavlink_tcp_bridge_alive_us;
            s_modules.push_back(m);
        }

        // Default auto_restart: OFF.  Kotlin WatchdogManager decides policy
        // based on config + flight state.
        for (auto* m : s_modules) m->auto_restart.store(false);
    }

    // ---- spawn poll thread ----
    s_running.store(true);
    s_initialized.store(true);
    if (pthread_create(&s_thread, nullptr, poll_thread_fn, nullptr) != 0) {
        WD_LOGE("pthread_create failed for watchdog");
        s_running.store(false);
        s_initialized.store(false);
        return false;
    }

    log_event("init", "", "modules=%zu period_ms=%u",
              s_modules.size(), s_poll_period_ms.load());
    return true;
}

extern "C" void wd_shutdown() {
    if (!s_initialized.load()) return;
    log_event("shutdown", "", nullptr);
    s_running.store(false);
    // Don't join — thread is daemon-ish; detach on destroy to avoid
    // hanging on app exit if a poll cycle is running a slow restart.
    pthread_detach(s_thread);

    // Note: we intentionally do not `delete` the ModuleStates to avoid
    // races with polling thread that may still be running for up to one
    // more period.  Process exit reclaims the small amount of memory.
    std::lock_guard<std::mutex> lock(s_log_file_mutex);
    if (s_log_fp) { fclose(s_log_fp); s_log_fp = nullptr; }
    s_initialized.store(false);
}

extern "C" size_t wd_copy_recent_events(char* json_out, size_t max_bytes,
                                         size_t max_events) {
    if (!s_initialized.load() || !json_out || max_bytes < 16) return 0;
    const uint64_t head = s_log_ring_head.load();
    const size_t available = head < kLogRingSize ? head : kLogRingSize;
    const size_t want = max_events == 0 ? available : (max_events < available ? max_events : available);

    size_t written = 0;
    json_out[0] = '[';
    written = 1;

    // iterate oldest-first of the last `want` entries
    for (size_t i = 0; i < want; ++i) {
        const uint64_t idx = (head - want + i) % kLogRingSize;
        const LogEntry& e = s_log_ring[idx];
        char buf[512];
        int n = snprintf(buf, sizeof(buf),
            "%s{\"t_us\":%llu,\"event\":\"%s\",\"module\":\"%s\",\"note\":\"%s\"}",
            (i == 0 ? "" : ","),
            (unsigned long long)e.t_us,
            e.event.c_str(), e.module.c_str(), e.note.c_str());
        if (n <= 0) break;
        if (written + (size_t)n + 2 >= max_bytes) break; // +2 for closing ']' and NUL
        std::memcpy(json_out + written, buf, (size_t)n);
        written += (size_t)n;
    }
    json_out[written++] = ']';
    json_out[written] = '\0';
    return written;
}
