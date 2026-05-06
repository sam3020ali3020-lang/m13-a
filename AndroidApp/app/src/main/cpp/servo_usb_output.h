#pragma once
/**
 * servo_usb_output.h — Phase 11.2
 * جسر بين PX4 actuator_servos (uORB) و سيرفوهات XQPOWER عبر USB libusb
 */

// تشغيل/إيقاف thread الإخراج (يُستدعى من px4_jni.cpp)
void servo_usb_output_start();
void servo_usb_output_stop();

// يُستدعى من JNI عند توصيل/فصل USB — يمرر file descriptor
// fd > 0 = USB متصل، fd < 0 = USB مفصول
void servo_usb_set_fd(int fd);
