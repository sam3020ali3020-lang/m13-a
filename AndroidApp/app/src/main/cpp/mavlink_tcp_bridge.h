#pragma once

#include <cstdint>

// TCP-to-UDP bridge for MAVLink over USB (ADB forward)
// TCP server on port 5760, forwards to/from MAVLink UDP on localhost:14550

void mavlink_tcp_bridge_start(int tcp_port, int udp_port);
void mavlink_tcp_bridge_stop();

// Watchdog heartbeat: returns hrt_absolute_time() of the last successful
// bridge_loop iteration.  0 if never started.  The bridge updates this at
// least once per second (poll timeout = 1s) so a reader can detect the
// bridge thread hanging by noticing the value stops advancing.
uint64_t mavlink_tcp_bridge_alive_us();
