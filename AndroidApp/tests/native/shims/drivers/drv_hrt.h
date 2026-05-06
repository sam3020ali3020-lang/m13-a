/**
 * Minimal shim for PX4 <drivers/drv_hrt.h> when building host tests.
 *
 * The real header lives in PX4-Autopilot and pulls in many dependencies.
 * For race-detection tests we only need `hrt_abstime` and a stub
 * `hrt_absolute_time()` to compile `shared_sensor_data.h`.
 */
#pragma once

#include <cstdint>
#include <chrono>

typedef uint64_t hrt_abstime;

static inline hrt_abstime hrt_absolute_time() {
    using namespace std::chrono;
    return (hrt_abstime)duration_cast<microseconds>(
        steady_clock::now().time_since_epoch()
    ).count();
}
