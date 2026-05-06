package com.ardophone.px4v17.service

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.app.Service
import android.content.Intent
import android.content.pm.ServiceInfo
import android.os.Build
import android.os.IBinder
import androidx.core.app.NotificationCompat
import com.ardophone.px4v17.MainActivity
import com.ardophone.px4v17.bridge.PX4Bridge
import com.ardophone.px4v17.isolation.FlightIsolationManager
import com.ardophone.px4v17.sensor.SensorReader
import com.ardophone.px4v17.state.BridgeState
import com.ardophone.px4v17.storage.BlackBoxPreferences
import com.ardophone.px4v17.storage.SafBlackBoxMirror
import com.ardophone.px4v17.usb.UsbServoManager
import com.ardophone.px4v17.watchdog.WatchdogManager
import java.io.File

class FlightService : Service() {

    private lateinit var sensorReader: SensorReader
    private lateinit var usbServoManager: UsbServoManager

    override fun onCreate() {
        super.onCreate()
        sensorReader = SensorReader(this)
        usbServoManager = UsbServoManager(this)
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        when (intent?.action) {
            ACTION_START -> {
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.UPSIDE_DOWN_CAKE) {
                    startForeground(NOTIFICATION_ID, createNotification(), ServiceInfo.FOREGROUND_SERVICE_TYPE_LOCATION)
                } else {
                    startForeground(NOTIFICATION_ID, createNotification())
                }

                // Full flight isolation: WakeLock + WifiLock HIGH_PERF + DND + kill bg apps
                FlightIsolationManager.engageFull(this)

                val storagePath = getExternalFilesDir(null)!!.absolutePath + "/px4"
                PX4Bridge.startPX4(storagePath)

                // Watchdog broadcast surface + default policy.  Must come
                // AFTER startPX4 because modules register themselves during
                // start_px4_modules() and the receiver calls into native
                // APIs that would otherwise return "not initialised".
                WatchdogManager.init(applicationContext)

                sensorReader.start()
                usbServoManager.start()

                // Start USB mirror if configured
                val treeUri = BlackBoxPreferences.getTreeUri(this)
                if (treeUri != null) {
                    val px4Root = File(storagePath)
                    SafBlackBoxMirror.schedule(this, px4Root, treeUri)
                }
            }
            ACTION_STOP -> {
                // Final sync to USB before stopping
                val stopTreeUri = BlackBoxPreferences.getTreeUri(this)
                if (stopTreeUri != null) {
                    val px4Root = File(getExternalFilesDir(null)!!.absolutePath + "/px4")
                    SafBlackBoxMirror.syncOnce(this, px4Root, stopTreeUri)
                }
                SafBlackBoxMirror.cancelSchedule()

                // Signal all native threads to stop (non-blocking flags only)
                WatchdogManager.release()
                BridgeState.clear()
                FlightIsolationManager.release()
                stopForeground(STOP_FOREGROUND_REMOVE)
                stopSelf()

                // Run blocking cleanup in background, then kill process
                Thread {
                    usbServoManager.stop()
                    sensorReader.stop()
                    PX4Bridge.stopPX4()
                    android.os.Process.killProcess(android.os.Process.myPid())
                }.start()

                // Failsafe: if cleanup hangs, force kill after 2 seconds
                Thread {
                    Thread.sleep(2000)
                    android.os.Process.killProcess(android.os.Process.myPid())
                }.start()
            }
        }
        return START_NOT_STICKY
    }

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onDestroy() {
        try {
            WatchdogManager.release()
            if (::usbServoManager.isInitialized) usbServoManager.stop()
            if (::sensorReader.isInitialized) sensorReader.stop()
            if (PX4Bridge.isRunning()) PX4Bridge.stopPX4()
        } catch (_: Exception) { }
        BridgeState.clear()
        FlightIsolationManager.release()
        super.onDestroy()
    }

    private fun createNotification(): Notification {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            val channel = NotificationChannel(
                CHANNEL_ID,
                "PX4 Flight",
                NotificationManager.IMPORTANCE_LOW
            ).apply { setShowBadge(false) }
            (getSystemService(NOTIFICATION_SERVICE) as NotificationManager)
                .createNotificationChannel(channel)
        }
        val openIntent = Intent(this, MainActivity::class.java)
        val pending = PendingIntent.getActivity(
            this, 0, openIntent,
            PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_IMMUTABLE
        )
        return NotificationCompat.Builder(this, CHANNEL_ID)
            .setContentTitle("PX4 Flight Controller")
            .setContentText("PX4 is running — MAVLink on UDP:14550")
            .setSmallIcon(android.R.drawable.ic_dialog_info)
            .setContentIntent(pending)
            .setOngoing(true)
            .build()
    }

    companion object {
        const val ACTION_START = "com.ardophone.px4v17.START_FLIGHT"
        const val ACTION_STOP = "com.ardophone.px4v17.STOP_FLIGHT"
        private const val CHANNEL_ID = "px4_flight_channel"
        private const val NOTIFICATION_ID = 1001
    }
}
