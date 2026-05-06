package com.ardophone.px4v17.bridge

object PX4Bridge {
    init { System.loadLibrary("px4phone_native") }

    // Start / Stop
    external fun startPX4(storagePath: String): Boolean
    external fun stopPX4()
    external fun isRunning(): Boolean

    // Native sensor counts (for UI rate display)
    external fun getNativeImuCount(): Long
    external fun getNativeBaroCount(): Long
    external fun getNativeMagCount(): Long
    external fun getNativeGpsCount(): Long

    // Vehicle state from uORB
    external fun getRoll(): Float
    external fun getPitch(): Float
    external fun getYaw(): Float
    external fun getAltitude(): Float
    external fun isArmed(): Boolean
    external fun getEKFStatus(): Int
    external fun getAirframeId(): Int
    external fun setAirframeId(id: Int): Boolean

    // USB Servo output (Phase 11.2)
    external fun setServoUsbFd(fd: Int)

    // USB CAN adapter (Waveshare USB_CAN_A) for xqpower_can SLCAN
    external fun setCanUsbFd(fd: Int)

    // USB GPS UBX (u-blox binary protocol — reads directly in C++)
    external fun setGpsUsbFd(fd: Int)

    // CP210x telemetry radio (PTY bridge for mavlink -d)
    external fun setMavlinkTelemetryUsbFd(fd: Int)

    // =========================================================================
    // Watchdog (see cpp/watchdog_native.h)
    //
    // nativeGetModuleStatus returns an empty long[] if the module is unknown
    // or the watchdog isn't initialised yet; callers MUST handle size 0.
    // The 13-element layout is:
    //   [0] alive (0/1), [1] was_alive_ever, [2] auto_restart,
    //   [3] last_alive_us, [4] last_tick_us, [5] last_crash_us,
    //   [6] last_restart_us, [7] last_recover_us, [8] crash_count,
    //   [9] restart_count, [10] stale_us, [11] now_us, [12] age_us.
    //
    // nativeGetWatchdogEvents returns a JSON array string; caller parses.
    // Passing 0 for maxEvents means "return all buffered events".
    //
    // nativeCrashModule is TEST-ONLY.  FlightService guards it behind a
    // debuggable-build check before exposing via BroadcastReceiver.
    // =========================================================================
    external fun nativeListWatchdogModules(): Array<String>
    external fun nativeGetModuleStatus(module: String): LongArray
    external fun nativeCrashModule(module: String): Boolean
    external fun nativeRestartModule(module: String): Boolean
    external fun nativeSetAutoRestart(module: String, enable: Boolean): Boolean
    external fun nativeGetWatchdogEvents(maxEvents: Int): String

    // Truncate the on-device JSONL event log in-process (close + reopen
    // with O_TRUNC).  Required because the log is held open by the poll
    // thread: `adb shell rm` would unlink the path but leave us writing
    // to an orphan inode.  Returns false if the watchdog is not yet
    // initialised or reopen failed.
    external fun nativeTruncateWatchdogLog(): Boolean
}
