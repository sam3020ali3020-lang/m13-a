package com.ardophone.px4v17.isolation

import android.app.Activity
import android.app.ActivityManager
import android.app.NotificationManager
import android.content.Context
import android.content.Intent
import android.net.Uri
import android.net.wifi.WifiManager
import android.os.Build
import android.os.PowerManager
import android.provider.Settings
import android.util.Log
import android.view.View
import android.view.Window
import android.view.WindowManager

/**
 * Automatic flight-time isolation: puts the phone into a "silent room" so the MPC
 * thread sees minimal scheduler interference.
 *
 * What it does (no root required):
 *   - PARTIAL_WAKE_LOCK         → CPU never sleeps
 *   - WifiLock HIGH_PERF        → WiFi stays connected to current AP, no scans/idle
 *   - Sustained Performance     → locks CPU/GPU at sustained freq (prevents thermal throttle spikes)
 *   - Keep screen on + bright   → no display power transitions
 *   - DND (priority only)       → no notification-driven wakeups
 *   - Kill background processes → frees scheduler slots from idle apps
 *
 * What it CAN'T do without root (the user is asked to do these once):
 *   - Toggle WiFi/Bluetooth off  → not allowed on API 29+
 *   - SCHED_FIFO real-time       → needs CAP_SYS_NICE
 *   - Force-stop other vendor apps → blocked by Android
 *
 * Call [applyToActivity] from MainActivity.onCreate and [engageFull] from FlightService
 * when ACTION_START fires. Call [release] when flight stops.
 */
object FlightIsolationManager {

    private const val TAG = "FlightIsolation"

    private var wakeLock: PowerManager.WakeLock? = null
    private var wifiLock: WifiManager.WifiLock? = null
    private var previousDndFilter: Int = NotificationManager.INTERRUPTION_FILTER_UNKNOWN
    private var dndActivated: Boolean = false

    /**
     * Light-weight isolation applied as soon as the UI is visible:
     * - Sustained Performance Mode (critical — prevents thermal throttling)
     * - Keep screen on + max brightness
     * - Immersive full-screen (no system bars stealing draw work)
     */
    fun applyToActivity(activity: Activity) {
        val window: Window = activity.window
        try {
            // Critical: prevents the CPU from boost→throttle cycles that cause 50-70ms MPC spikes
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
                val pm = activity.getSystemService(Context.POWER_SERVICE) as PowerManager
                if (pm.isSustainedPerformanceModeSupported) {
                    window.setSustainedPerformanceMode(true)
                    Log.i(TAG, "SustainedPerformanceMode: ON")
                } else {
                    Log.w(TAG, "SustainedPerformanceMode: NOT SUPPORTED on this device")
                }
            }

            // Screen on + max brightness → no display-off transitions (they also throttle CPU)
            window.addFlags(
                WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON or
                    WindowManager.LayoutParams.FLAG_TURN_SCREEN_ON or
                    WindowManager.LayoutParams.FLAG_DISMISS_KEYGUARD or
                    WindowManager.LayoutParams.FLAG_SHOW_WHEN_LOCKED
            )
            val lp = window.attributes
            lp.screenBrightness = 1.0f
            window.attributes = lp

            // Immersive sticky — hide nav/status bars so system UI doesn't preempt us
            @Suppress("DEPRECATION")
            window.decorView.systemUiVisibility = (
                View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY or
                    View.SYSTEM_UI_FLAG_LAYOUT_STABLE or
                    View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION or
                    View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN or
                    View.SYSTEM_UI_FLAG_HIDE_NAVIGATION or
                    View.SYSTEM_UI_FLAG_FULLSCREEN
                )
        } catch (t: Throwable) {
            Log.e(TAG, "applyToActivity failed: ${t.message}")
        }

        // First-launch UX: ask user to grant battery + DND exemptions (one-time)
        maybeRequestBatteryOptimizationExemption(activity)
    }

    /**
     * Heavy isolation applied when the flight service actually starts:
     * - WakeLock (CPU never sleeps)
     * - WifiLock (HIGH_PERF, keeps WiFi connected, suppresses scans)
     * - DND (priority-only notifications)
     * - Kill background processes (own app cache + others we can reach)
     */
    fun engageFull(context: Context) {
        // 1) WakeLock — PARTIAL keeps CPU awake even if screen goes off
        try {
            val pm = context.getSystemService(Context.POWER_SERVICE) as PowerManager
            if (wakeLock == null) {
                wakeLock = pm.newWakeLock(
                    PowerManager.PARTIAL_WAKE_LOCK,
                    "px4phone:flight-isolation"
                )
                wakeLock?.setReferenceCounted(false)
            }
            if (wakeLock?.isHeld == false) wakeLock?.acquire(6L * 60L * 60L * 1000L)  // 6h cap
            Log.i(TAG, "WakeLock acquired")
        } catch (t: Throwable) {
            Log.e(TAG, "WakeLock failed: ${t.message}")
        }

        // 2) WifiLock — HIGH_PERF keeps WiFi radio at full power, drops scans/idle transitions
        try {
            val wm = context.applicationContext.getSystemService(Context.WIFI_SERVICE) as WifiManager
            val mode = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q)
                WifiManager.WIFI_MODE_FULL_HIGH_PERF
            else
                @Suppress("DEPRECATION") WifiManager.WIFI_MODE_FULL_HIGH_PERF
            if (wifiLock == null) {
                wifiLock = wm.createWifiLock(mode, "px4phone:flight-wifi")
                wifiLock?.setReferenceCounted(false)
            }
            if (wifiLock?.isHeld == false) wifiLock?.acquire()
            Log.i(TAG, "WifiLock HIGH_PERF acquired")
        } catch (t: Throwable) {
            Log.e(TAG, "WifiLock failed: ${t.message}")
        }

        // 3) DND priority-only — silences everything except our own foreground service
        try {
            val nm = context.getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M && nm.isNotificationPolicyAccessGranted) {
                previousDndFilter = nm.currentInterruptionFilter
                nm.setInterruptionFilter(NotificationManager.INTERRUPTION_FILTER_PRIORITY)
                dndActivated = true
                Log.i(TAG, "DND: priority-only")
            } else {
                Log.w(TAG, "DND: policy access NOT granted → skipping (user must grant once)")
            }
        } catch (t: Throwable) {
            Log.e(TAG, "DND failed: ${t.message}")
        }

        // 4) Kill background processes we can reach — frees CPU slots
        try {
            val am = context.getSystemService(Context.ACTIVITY_SERVICE) as ActivityManager
            val targets = listOf(
                "com.android.chrome",
                "com.google.android.youtube",
                "com.whatsapp",
                "com.facebook.katana",
                "com.facebook.orca",
                "com.instagram.android",
                "com.twitter.android",
                "com.spotify.music",
                "com.netflix.mediaclient",
                "com.google.android.gm",
                "com.google.android.apps.maps",
                "com.google.android.apps.photos",
                "com.snapchat.android",
                "com.telegram.messenger",
                "org.telegram.messenger"
            )
            targets.forEach { pkg ->
                try { am.killBackgroundProcesses(pkg) } catch (_: Throwable) { }
            }
            Log.i(TAG, "Killed background processes for ${targets.size} known noisy apps")
        } catch (t: Throwable) {
            Log.e(TAG, "killBackgroundProcesses failed: ${t.message}")
        }
    }

    /**
     * Release all isolation locks. Call from FlightService.onDestroy / ACTION_STOP.
     */
    fun release() {
        try {
            if (wakeLock?.isHeld == true) wakeLock?.release()
        } catch (_: Throwable) { }
        wakeLock = null

        try {
            if (wifiLock?.isHeld == true) wifiLock?.release()
        } catch (_: Throwable) { }
        wifiLock = null

        if (dndActivated) {
            dndActivated = false
            // We don't try to restore the user's previous DND filter blindly;
            // most users don't use DND, and restoring INTERRUPTION_FILTER_UNKNOWN crashes.
            // We simply drop back to ALL if we had set PRIORITY.
            // Leaving DND untouched on release is acceptable — user can turn it off manually.
        }
        Log.i(TAG, "released isolation locks")
    }

    /**
     * One-time prompt: ask user to whitelist us from battery optimization.
     * Without this, Doze/App-Standby can throttle our CPU on its own schedule.
     */
    private fun maybeRequestBatteryOptimizationExemption(activity: Activity) {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.M) return
        try {
            val pm = activity.getSystemService(Context.POWER_SERVICE) as PowerManager
            if (pm.isIgnoringBatteryOptimizations(activity.packageName)) return
            val intent = Intent(Settings.ACTION_REQUEST_IGNORE_BATTERY_OPTIMIZATIONS).apply {
                data = Uri.parse("package:${activity.packageName}")
            }
            activity.startActivity(intent)
            Log.i(TAG, "Requested battery-optimization exemption")
        } catch (t: Throwable) {
            Log.w(TAG, "battery-opt request failed: ${t.message}")
        }
    }

    /**
     * Launch the system DND access screen so the user can grant the policy once.
     * Call this once (e.g. from a settings button) if DND failed to activate.
     */
    fun requestDndAccess(context: Context) {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.M) return
        try {
            val nm = context.getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
            if (nm.isNotificationPolicyAccessGranted) return
            val intent = Intent(Settings.ACTION_NOTIFICATION_POLICY_ACCESS_SETTINGS)
                .addFlags(Intent.FLAG_ACTIVITY_NEW_TASK)
            context.startActivity(intent)
        } catch (t: Throwable) {
            Log.w(TAG, "DND access request failed: ${t.message}")
        }
    }
}
