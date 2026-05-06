#pragma once

#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_local_position.h>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>

static constexpr int SENSOR_NMEAS = 13;

// GPS staleness timeout: if no GPS update within this window, treat
// the cached position/velocity as invalid so stale values are not
// pushed into the MHE measurement stream.
static constexpr hrt_abstime GPS_STALE_TIMEOUT_US  = 500'000; // 500 ms
static constexpr hrt_abstime BARO_STALE_TIMEOUT_US = 500'000; // 500 ms

struct SensorMeasurement {
	double y[SENSOR_NMEAS];
	// y[0..2]  = gyro_rad[0..2]          (raw IMU)
	// y[3..5]  = accelerometer_m_s2[0..2] (raw IMU)
	// y[6]     = baro_alt_meter           (raw baro)
	// y[7..8]  = GPS position north, east (m, relative to arm origin)
	// y[9]     = GPS altitude MSL (m)
	// y[10..12] = GPS velocity N, E, D (m/s)
	bool valid;
	bool baro_valid;  // false when baro data is stale (> BARO_STALE_TIMEOUT_US)
	bool gps_fresh;   // true only when GPS was actually updated since last build_measurement()
	bool gps_vel_valid;  // true only when GPS velocity is valid (vel_ned_valid + finite)
};

class SensorBridge
{
public:
	SensorBridge() = default;

	/** Set GPS reference origin in WGS84 (call at arm / pre-launch).
	 *  Delegates the lat/lon → local NED projection to PX4's MapProjection
	 *  (azimuthal equidistant) so the frame used here matches the one used
	 *  by EKF2, navigator and commander. This keeps the MHE position state
	 *  consistent with vehicle_local_position / lpos.ref_lat/ref_lon, which
	 *  matters when the two streams are mixed (see update_from_lpos and
	 *  RocketMPC's set_ned_origin). */
	void set_gps_origin(double lat_deg, double lon_deg, double alt_msl_m)
	{
		_ref_proj.initReference(lat_deg, lon_deg, hrt_absolute_time());
		_ref_alt_msl = alt_msl_m;
	}

	/** Set the NED origin (arm position in EKF2 lpos frame) used by
	 *  update_from_lpos() to produce arm-relative measurements.  This keeps
	 *  the MHE position state in the same frame as RocketMPC's Xm/Ym/Zm
	 *  (which always subtract _arm_origin_*), so that the EKF↔MHE cooperative
	 *  blend and XVAL reset do not inject a constant (_arm_origin) offset
	 *  when the EKF2 origin differs from the arm position. Must be called
	 *  before update_from_lpos() whenever _arm_origin_* is captured/updated. */
	void set_ned_origin(float ned_x, float ned_y, float ned_z)
	{
		_ned_origin_x = ned_x;
		_ned_origin_y = ned_y;
		_ned_origin_z = ned_z;
		_ned_origin_set = true;
	}

	/** Feed raw GPS reading — converts to local NED internally */
	void update_gps(const sensor_gps_s &gps);

	/** SITL fallback: use EKF2 local position as GPS substitute.
	 *  Only use when real sensor_gps is unavailable (SITL circular dep).
	 *  Requires set_gps_origin() to have been called first (for alt ref). */
	void update_from_lpos(const vehicle_local_position_s &lpos);

	/** Feed baro reading — marks baro as valid and records update time.
	 *  Must be called every cycle that fresh vehicle_air_data arrives so
	 *  the staleness detector in build_measurement() can work. */
	void update_baro(const vehicle_air_data_s &air);

	/** Build MHE measurement vector from raw sensors (bypasses EKF2) */
	SensorMeasurement build_measurement(
		const sensor_combined_s &sc,
		const vehicle_air_data_s &air);

	/** Reference altitude MSL (set at arm). Used as MHE launch_alt parameter. */
	double ref_alt_msl() const { return _ref_alt_msl; }
	bool   gps_origin_set() const { return _ref_proj.isInitialized(); }
	bool   gps_valid() const { return _gps_valid; }
	// Staleness is handled internally by build_measurement(): once a sample
	// is older than GPS_STALE_TIMEOUT_US it drops _gps_valid and the caller
	// sees .valid==false on the next measurement.

	/** True when last baro update is newer than BARO_STALE_TIMEOUT_US. */
	bool   baro_fresh(hrt_abstime now) const {
		return _baro_valid && _last_baro_update_us > 0
		       && (now - _last_baro_update_us) < BARO_STALE_TIMEOUT_US;
	}

	hrt_abstime last_gps_update_us() const { return _last_gps_update_us; }

private:
	// GPS reference origin. Lat/lon are held inside _ref_proj (PX4's
	// azimuthal equidistant MapProjection, same one used by EKF2) so
	// GPS→NED stays consistent with the rest of the PX4 frame stack.
	// _ref_alt_msl is kept separately because MapProjection is 2D only.
	MapProjection _ref_proj{};
	double        _ref_alt_msl{0.0};

	// NED origin (arm position in EKF2 lpos frame). Used by update_from_lpos
	// to subtract the arm-time offset so HITL/SITL MHE measurements share a
	// frame with RocketMPC's Xm/Ym/Zm. Initialised to (0,0,0) so pre-arm calls
	// to update_from_lpos() pre-set-ned behave as if EKF2 origin is the
	// reference — same as the previous behaviour.
	float _ned_origin_x{0.0f};
	float _ned_origin_y{0.0f};
	float _ned_origin_z{0.0f};
	bool  _ned_origin_set{false};

	// Last GPS reading converted to local NED
	double _gps_north{0.0};
	double _gps_east{0.0};
	double _gps_alt_msl{0.0};
	double _gps_vn{0.0};
	double _gps_ve{0.0};
	double _gps_vd{0.0};
	bool   _gps_valid{false};
	bool   _gps_vel_valid{false};  // velocity specifically valid (vel_ned_valid + finite)
	hrt_abstime _last_gps_update_us{0};

	// Baro staleness tracking (mirrors GPS pattern)
	bool   _baro_valid{false};
	hrt_abstime _last_baro_update_us{0};

	// GPS freshness: detect whether _last_gps_update_us changed between
	// consecutive build_measurement() calls.  MHE uses this to avoid
	// counting stale (repeated) GPS samples as independent measurements.
	hrt_abstime _prev_gps_update_us{0};
};
