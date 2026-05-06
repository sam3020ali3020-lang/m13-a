#pragma once

void start_uorb_publishers();
void stop_uorb_publishers();

// للقراءة من JNI (حالة المركبة من uORB)
float get_vehicle_roll();
float get_vehicle_pitch();
float get_vehicle_yaw();
float get_vehicle_altitude();
bool  get_vehicle_armed();
int   get_vehicle_nav_state();
int   get_ekf_status();
int   get_airframe_id();
bool  set_airframe_id(int id);
