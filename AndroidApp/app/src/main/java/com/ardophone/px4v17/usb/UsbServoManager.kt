package com.ardophone.px4v17.usb

import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbDeviceConnection
import android.hardware.usb.UsbManager
import android.os.Build
import android.util.Log
import com.ardophone.px4v17.bridge.PX4Bridge

/**
 * UsbServoManager — Phase 11.2 + GPS UBX
 * يكتشف أجهزة USB (CP2102/CH340/STM32/u-blox) عبر Android UsbManager
 * ويمرر file descriptor لـ C++ عبر JNI.
 *
 * يدعم أجهزة متعددة متصلة في نفس الوقت عبر USB Hub:
 *   - CP2102/STM32 → servo_usb_output (سيرفوات XQPOWER)
 *   - CH340        → xqpower_can (SLCAN)
 *   - u-blox       → gps_usb_ubx (بروتوكول UBX الثنائي)
 */
class UsbServoManager(private val context: Context) {

    companion object {
        private const val TAG = "UsbDeviceMgr"
        private const val ACTION_USB_PERMISSION = "com.ardophone.px4v17.USB_PERMISSION"

        // Supported USB chips (VID)
        private const val VID_CP2102 = 0x10C4  // Silicon Labs CP2102
        private const val VID_CH340  = 0x1A86  // QinHeng CH340
        private const val VID_STM32  = 0x0483  // STMicroelectronics (CDC-ACM)
        private const val VID_UBLOX    = 0x1546  // u-blox AG (GPS direct USB)
        private const val VID_PROLIFIC = 0x067B  // Prolific PL2303 (USB-Serial for GPS)
        private const val VID_CAN_LIN = 0x2E3C  // CAN_LIN_Tool V6.0 (CDC-ACM)
    }

    private val usbManager: UsbManager =
        context.getSystemService(Context.USB_SERVICE) as UsbManager

    // دعم أجهزة متعددة — كل جهاز له connection خاص
    private val connections = mutableMapOf<Int, UsbDeviceConnection>()  // deviceId → connection
    private val devices = mutableMapOf<Int, UsbDevice>()               // deviceId → device
    private var started = false

    private val usbReceiver = object : BroadcastReceiver() {
        override fun onReceive(ctx: Context, intent: Intent) {
            when (intent.action) {
                UsbManager.ACTION_USB_DEVICE_ATTACHED -> {
                    val device = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                        intent.getParcelableExtra(UsbManager.EXTRA_DEVICE, UsbDevice::class.java)
                    } else {
                        @Suppress("DEPRECATION")
                        intent.getParcelableExtra(UsbManager.EXTRA_DEVICE)
                    }
                    device?.let { onDeviceAttached(it) }
                }

                UsbManager.ACTION_USB_DEVICE_DETACHED -> {
                    val device = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                        intent.getParcelableExtra(UsbManager.EXTRA_DEVICE, UsbDevice::class.java)
                    } else {
                        @Suppress("DEPRECATION")
                        intent.getParcelableExtra(UsbManager.EXTRA_DEVICE)
                    }
                    device?.let { onDeviceDetached(it) }
                }

                ACTION_USB_PERMISSION -> {
                    val granted = intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)
                    val device = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                        intent.getParcelableExtra(UsbManager.EXTRA_DEVICE, UsbDevice::class.java)
                    } else {
                        @Suppress("DEPRECATION")
                        intent.getParcelableExtra(UsbManager.EXTRA_DEVICE)
                    }
                    if (granted && device != null) {
                        Log.i(TAG, "USB permission granted for ${device.deviceName}")
                        openDevice(device)
                    } else {
                        Log.w(TAG, "USB permission denied")
                    }
                }
            }
        }
    }

    fun start() {
        if (started) return
        started = true

        val filter = IntentFilter().apply {
            addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED)
            addAction(UsbManager.ACTION_USB_DEVICE_DETACHED)
            addAction(ACTION_USB_PERMISSION)
        }

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            context.registerReceiver(usbReceiver, filter, Context.RECEIVER_NOT_EXPORTED)
        } else {
            context.registerReceiver(usbReceiver, filter)
        }

        Log.i(TAG, "UsbDeviceManager started — scanning for devices...")
        scanExistingDevices()
    }

    fun stop() {
        if (!started) return
        started = false

        try {
            context.unregisterReceiver(usbReceiver)
        } catch (_: Exception) {}

        closeAllDevices()
        Log.i(TAG, "UsbDeviceManager stopped")
    }

    private fun scanExistingDevices() {
        for ((_, device) in usbManager.deviceList) {
            if (isSupportedDevice(device)) {
                Log.i(TAG, "Found existing device: ${deviceInfo(device)}")
                onDeviceAttached(device)
            }
        }
    }

    private fun onDeviceAttached(device: UsbDevice) {
        if (!isSupportedDevice(device)) return
        if (devices.containsKey(device.deviceId)) {
            Log.w(TAG, "Device already connected: ${device.deviceName}")
            return
        }

        Log.i(TAG, "Device attached: ${deviceInfo(device)}")

        if (usbManager.hasPermission(device)) {
            openDevice(device)
        } else {
            Log.i(TAG, "Requesting USB permission for ${deviceInfo(device)}...")
            val flags = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_MUTABLE
            } else {
                PendingIntent.FLAG_UPDATE_CURRENT
            }
            val pi = PendingIntent.getBroadcast(context, device.deviceId,
                Intent(ACTION_USB_PERMISSION).setPackage(context.packageName), flags)
            usbManager.requestPermission(device, pi)
        }
    }

    private fun onDeviceDetached(device: UsbDevice) {
        if (!devices.containsKey(device.deviceId)) return
        Log.i(TAG, "Device detached: ${deviceInfo(device)}")
        closeDevice(device.deviceId)
    }

    private fun openDevice(device: UsbDevice) {
        val conn = usbManager.openDevice(device)
        if (conn == null) {
            Log.e(TAG, "Failed to open USB device: ${deviceInfo(device)}")
            return
        }

        val fd = conn.fileDescriptor
        if (fd < 0) {
            Log.e(TAG, "Invalid file descriptor for ${deviceInfo(device)}")
            conn.close()
            return
        }

        connections[device.deviceId] = conn
        devices[device.deviceId] = device

        // Route FD based on chip type:
        // CH340  (Waveshare USB_CAN_A) → xqpower_can driver
        // CAN_LIN (CAN_LIN_Tool V6.0) → xqpower_can driver (CDC protocol)
        // u-blox (GPS receiver)        → gps_usb_ubx (UBX binary protocol)
        // CP2102                       → MAVLink telemetry (radio via PTY bridge)
        // STM32                        → servo_usb_output (5-byte binary)
        when (device.vendorId) {
            VID_CH340, VID_CAN_LIN -> {
                PX4Bridge.setCanUsbFd(fd)
                Log.i(TAG, "CAN adapter opened: ${deviceInfo(device)}, fd=$fd")
            }
            VID_UBLOX, VID_PROLIFIC -> {
                PX4Bridge.setGpsUsbFd(fd)
                Log.i(TAG, "GPS UBX opened: ${deviceInfo(device)}, fd=$fd")
            }
            VID_CP2102 -> {
                PX4Bridge.setMavlinkTelemetryUsbFd(fd)
                Log.i(TAG, "CP210x → MAVLink telemetry, fd=$fd")
            }
            else -> {
                PX4Bridge.setServoUsbFd(fd)
                Log.i(TAG, "Servo USB opened: ${deviceInfo(device)}, fd=$fd")
            }
        }
    }

    private fun closeDevice(deviceId: Int) {
        val device = devices[deviceId]

        // Tell C++ to release the USB fd
        device?.let {
            when (it.vendorId) {
                VID_CH340, VID_CAN_LIN -> PX4Bridge.setCanUsbFd(-1)
                VID_UBLOX, VID_PROLIFIC -> PX4Bridge.setGpsUsbFd(-1)
                VID_CP2102              -> PX4Bridge.setMavlinkTelemetryUsbFd(-1)
                else                    -> PX4Bridge.setServoUsbFd(-1)
            }
        }

        connections[deviceId]?.close()
        connections.remove(deviceId)
        devices.remove(deviceId)
    }

    private fun closeAllDevices() {
        val ids = devices.keys.toList()
        for (id in ids) {
            closeDevice(id)
        }
    }

    private fun isSupportedDevice(device: UsbDevice): Boolean {
        return device.vendorId == VID_CP2102 ||
               device.vendorId == VID_CH340 ||
               device.vendorId == VID_STM32 ||
               device.vendorId == VID_UBLOX ||
               device.vendorId == VID_PROLIFIC ||
               device.vendorId == VID_CAN_LIN
    }

    private fun deviceInfo(device: UsbDevice): String {
        val chip = when (device.vendorId) {
            VID_CP2102 -> "CP2102"
            VID_CH340  -> "CH340"
            VID_STM32  -> "STM32"
            VID_UBLOX    -> "u-blox"
            VID_PROLIFIC -> "PL2303 (GPS)"
            VID_CAN_LIN -> "CAN_LIN_Tool"
            else -> "Unknown"
        }
        return "$chip (VID=0x${device.vendorId.toString(16)}, PID=0x${device.productId.toString(16)})"
    }
}
