package com.ardophone.px4v17.watchdog

import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.ApplicationInfo
import android.util.Log
import com.ardophone.px4v17.bridge.PX4Bridge

/**
 * WatchdogManager — thin Kotlin shell around the native watchdog (see
 * cpp/watchdog_native.h).
 *
 * The native side already runs the 50 ms poll loop and writes a JSONL
 * event log; this class adds two things:
 *
 *   1. Policy dispatch: per-module auto-restart gating based on flight
 *      state or the operator's pre-flight checklist.  Pushed into the
 *      native layer via PX4Bridge.nativeSetAutoRestart.
 *
 *   2. BroadcastReceiver surface for the /watchdog test runner.  The
 *      Python harness triggers scenarios over adb with:
 *
 *        adb shell am broadcast -a com.ardophone.px4v17.WATCHDOG_TEST \
 *          --es action crash --es module rocket_mpc
 *
 *      For flight safety, crash injection is gated by isDebuggable() — a
 *      release-signed APK silently ignores those commands even if the
 *      broadcast reaches the receiver.  Restart and policy commands are
 *      allowed regardless; those are operator tools, not attack vectors.
 */
object WatchdogManager {
    private const val TAG = "WatchdogManager"

    // Broadcast surface — the /watchdog runner sends this action over adb.
    const val ACTION_TEST_CMD = "com.ardophone.px4v17.WATCHDOG_TEST"
    const val EXTRA_ACTION    = "action"        // crash | restart | set_autorestart | dump | policy
    const val EXTRA_MODULE    = "module"        // module name, e.g. "rocket_mpc"
    const val EXTRA_ENABLE    = "enable"        // "true"/"false" for set_autorestart
    const val EXTRA_MAX       = "max_events"    // int, for dump

    // A pre-launch policy profile: which modules auto-restart.  EKF2 and
    // rocket_mpc default OFF because restarting them mid-flight loses the
    // state that makes them useful; operator should enable explicitly from
    // the checklist if the pad is configured for a hot-restart campaign.
    data class Policy(
        val autoRestartPreLaunch: Set<String> = setOf(
            "native_sensor_reader",   // stateless — counter-driven
            "mavlink_tcp_bridge",     // pure plumbing
            "sensors",                // no private state beyond uORB
        ),
        val autoRestartInFlight: Set<String> = emptySet(),
    )

    data class Status(
        val module: String,
        val alive: Boolean,
        val wasAliveEver: Boolean,
        val autoRestart: Boolean,
        val lastAliveUs: Long,
        val lastTickUs: Long,
        val lastCrashUs: Long,
        val lastRestartUs: Long,
        val lastRecoverUs: Long,
        val crashCount: Long,
        val restartCount: Long,
        val staleUs: Long,
        val nowUs: Long,
        val ageUs: Long,
    )

    @Volatile private var receiver: BroadcastReceiver? = null
    @Volatile private var registeredContext: Context? = null

    /**
     * Install the broadcast receiver and apply the given pre-launch policy.
     *
     * Safe to call from FlightService.onStartCommand.  Idempotent: calling
     * twice in the same process is a no-op.  The receiver is unregistered
     * on [release].
     */
    fun init(context: Context, policy: Policy = Policy()) {
        if (receiver != null) {
            Log.i(TAG, "init: already registered")
            applyPolicy(policy.autoRestartPreLaunch)
            return
        }
        val r = WatchdogReceiver()
        val filter = IntentFilter(ACTION_TEST_CMD)
        // RECEIVER_EXPORTED is required on Android 13+ (API 33) for
        // receivers that accept broadcasts from other apps / adb shell.
        // Pre-33 the flag is ignored.  We need exported so the Python
        // harness can broadcast from outside our UID.
        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.TIRAMISU) {
            context.registerReceiver(r, filter, Context.RECEIVER_EXPORTED)
        } else {
            @Suppress("UnspecifiedRegisterReceiverFlag")
            context.registerReceiver(r, filter)
        }
        receiver = r
        registeredContext = context.applicationContext
        Log.i(TAG, "WatchdogManager initialised (testable=${isDebuggable(context)})")
        applyPolicy(policy.autoRestartPreLaunch)
    }

    fun release() {
        val r = receiver ?: return
        try {
            registeredContext?.unregisterReceiver(r)
        } catch (e: IllegalArgumentException) {
            // Race with onDestroy on some OEM builds — already gone.
            Log.w(TAG, "unregisterReceiver: $e")
        }
        receiver = null
        registeredContext = null
    }

    /** Snapshot status for a single module; null if unknown/uninitialised. */
    fun status(module: String): Status? {
        val a = PX4Bridge.nativeGetModuleStatus(module)
        if (a.size != 13) return null
        return Status(
            module        = module,
            alive         = a[0] == 1L,
            wasAliveEver  = a[1] == 1L,
            autoRestart   = a[2] == 1L,
            lastAliveUs   = a[3],
            lastTickUs    = a[4],
            lastCrashUs   = a[5],
            lastRestartUs = a[6],
            lastRecoverUs = a[7],
            crashCount    = a[8],
            restartCount  = a[9],
            staleUs       = a[10],
            nowUs         = a[11],
            ageUs         = a[12],
        )
    }

    /** List module names registered with the native watchdog. */
    fun modules(): List<String> = PX4Bridge.nativeListWatchdogModules().toList()

    private fun applyPolicy(enabled: Set<String>) {
        for (m in PX4Bridge.nativeListWatchdogModules()) {
            val on = enabled.contains(m)
            PX4Bridge.nativeSetAutoRestart(m, on)
        }
        Log.i(TAG, "policy applied: auto_restart=$enabled")
    }

    private fun isDebuggable(context: Context): Boolean =
        (context.applicationInfo.flags and ApplicationInfo.FLAG_DEBUGGABLE) != 0

    // ---------------------------------------------------------------------
    // Broadcast surface
    //
    // The native crash/restart paths can block up to ~1 s (TCP bridge
    // join, module work-queue drain) which would ANR if dispatched on the
    // main thread — BroadcastReceiver.onReceive runs on the main thread
    // by default.  We use goAsync() to extend the delivery window to 10 s
    // and offload the native call to a background Thread.
    // ---------------------------------------------------------------------
    private class WatchdogReceiver : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            val action = intent.getStringExtra(EXTRA_ACTION) ?: run {
                Log.w(TAG, "received broadcast without 'action' extra")
                return
            }
            val module = intent.getStringExtra(EXTRA_MODULE) ?: ""
            val appContext = context.applicationContext

            val pending = goAsync()
            Thread(Runnable {
                try {
                    handle(appContext, action, module, intent, pending)
                } catch (t: Throwable) {
                    Log.e(TAG, "handler threw", t)
                    setResultSafely(pending, 1, "EXCEPTION:${t.javaClass.simpleName}")
                } finally {
                    pending.finish()
                }
            }, "WatchdogReceiver-$action").start()
        }

        private fun handle(context: Context, action: String, module: String,
                           intent: Intent, pending: PendingResult) {
            when (action) {
                "crash" -> {
                    if (!isDebuggable(context)) {
                        Log.w(TAG, "crash injection rejected — release build")
                        setResultSafely(pending, 1, "REJECT_RELEASE_BUILD")
                        return
                    }
                    val ok = PX4Bridge.nativeCrashModule(module)
                    Log.i(TAG, "crash '$module' -> $ok")
                    setResultSafely(pending, if (ok) 0 else 1, if (ok) "OK" else "FAIL")
                }
                "restart" -> {
                    val ok = PX4Bridge.nativeRestartModule(module)
                    Log.i(TAG, "restart '$module' -> $ok")
                    setResultSafely(pending, if (ok) 0 else 1, if (ok) "OK" else "FAIL")
                }
                "set_autorestart" -> {
                    val enable = intent.getStringExtra(EXTRA_ENABLE)?.equals("true", true) == true
                    val ok = PX4Bridge.nativeSetAutoRestart(module, enable)
                    Log.i(TAG, "set_autorestart '$module'=$enable -> $ok")
                    setResultSafely(pending, if (ok) 0 else 1, if (ok) "OK" else "FAIL")
                }
                "dump" -> {
                    val maxN = intent.getStringExtra(EXTRA_MAX)?.toIntOrNull() ?: 0
                    val json = PX4Bridge.nativeGetWatchdogEvents(maxN)
                    Log.i(TAG, "dump ($maxN): ${json.length} bytes")
                    // Echo into logcat so the runner can pull via `adb logcat`
                    // without needing a file permission — split by chunks
                    // because logcat truncates each line around 4 KB.
                    logLong("WatchdogDump", json)
                    setResultSafely(pending, 0, "OK")
                }
                "clear_log" -> {
                    // Must NOT be implemented as `adb shell rm` because the
                    // native poll thread holds the file open — unlinking
                    // leaves the fd pointing at an orphan inode that no
                    // one can pull.  nativeTruncateWatchdogLog closes and
                    // reopens the fd in-process so the path stays visible.
                    val ok = PX4Bridge.nativeTruncateWatchdogLog()
                    Log.i(TAG, "clear_log -> $ok")
                    setResultSafely(pending, if (ok) 0 else 1, if (ok) "OK" else "FAIL")
                }
                "policy" -> {
                    // Switch between pre-launch and in-flight policy profiles.
                    // NB: we reuse EXTRA_MODULE as the profile name to avoid
                    // inventing another extra key; accepted values are
                    // "prelaunch" and "inflight".
                    val profile = module.ifEmpty { "prelaunch" }
                    val policy = Policy()
                    val enabled = if (profile == "inflight")
                        policy.autoRestartInFlight else policy.autoRestartPreLaunch
                    applyPolicy(enabled)
                    setResultSafely(pending, 0, "OK")
                }
                else -> {
                    Log.w(TAG, "unknown action '$action'")
                    setResultSafely(pending, 1, "UNKNOWN_ACTION")
                }
            }
        }

        private fun isDebuggable(context: Context): Boolean =
            (context.applicationInfo.flags and ApplicationInfo.FLAG_DEBUGGABLE) != 0

        private fun setResultSafely(p: PendingResult, code: Int, data: String) {
            try {
                p.setResultCode(code)
                p.setResultData(data)
            } catch (e: IllegalStateException) {
                // Not an ordered broadcast — shell ignored result, that's fine.
                Log.d(TAG, "setResult ignored: $e")
            }
        }

        private fun logLong(tag: String, s: String) {
            // Logcat line limit is ~4000 bytes.  Slice slightly smaller
            // to leave room for tag/metadata.
            val chunk = 3500
            var i = 0
            while (i < s.length) {
                val end = (i + chunk).coerceAtMost(s.length)
                Log.i(tag, s.substring(i, end))
                i = end
            }
        }
    }
}
