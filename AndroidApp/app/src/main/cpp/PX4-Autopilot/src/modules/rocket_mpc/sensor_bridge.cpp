#include "sensor_bridge.h"
#include <px4_platform_common/defines.h>   // PX4_ISFINITE
#include <cmath>                           // fabs

void SensorBridge::update_gps(const sensor_gps_s &gps)
{
	if (gps.fix_type < sensor_gps_s::FIX_TYPE_3D) { return; }

	// Reject non-finite values (NaN / Inf) before they poison the projection,
	// the origin latch, or the MHE measurement vector. A NaN on the first fix
	// would permanently corrupt the MapProjection reference via
	// set_gps_origin().
	if (!PX4_ISFINITE(gps.latitude_deg)
	    || !PX4_ISFINITE(gps.longitude_deg)
	    || !PX4_ISFINITE(gps.altitude_msl_m)) {
		return;
	}

	if (fabs(gps.latitude_deg) > 90.0 || fabs(gps.longitude_deg) > 180.0) {
		return;
	}

	// Auto-set origin on first valid 3D fix if not yet set
	if (!_ref_proj.isInitialized()) {
		set_gps_origin(gps.latitude_deg, gps.longitude_deg, gps.altitude_msl_m);
	}

	// WGS84 → local NED relative to arm origin via PX4's azimuthal
	// equidistant MapProjection — same projection used by EKF2 / navigator /
	// commander, so GPS-fed MHE measurements share a frame with lpos-fed
	// ones in update_from_lpos().
	float north_m = 0.0f;
	float east_m  = 0.0f;
	_ref_proj.project(gps.latitude_deg, gps.longitude_deg, north_m, east_m);
	_gps_north   = (double)north_m;
	_gps_east    = (double)east_m;
	_gps_alt_msl = gps.altitude_msl_m;

	_gps_vel_valid = gps.vel_ned_valid
		&& PX4_ISFINITE(gps.vel_n_m_s)
		&& PX4_ISFINITE(gps.vel_e_m_s)
		&& PX4_ISFINITE(gps.vel_d_m_s);

	if (_gps_vel_valid) {
		_gps_vn = (double)gps.vel_n_m_s;
		_gps_ve = (double)gps.vel_e_m_s;
		_gps_vd = (double)gps.vel_d_m_s;
	}

	_gps_valid = true;
	_last_gps_update_us = hrt_absolute_time();
}

void SensorBridge::update_from_lpos(const vehicle_local_position_s &lpos)
{
	// SITL fallback: treat EKF2 local position as GPS measurements.
	// Position is NED relative to EKF2 origin (≈ arm origin).

	// Guard 1: reject until GPS origin is set — otherwise _ref_alt_msl=0
	// and _gps_alt_msl would be a relative value mistaken for MSL,
	// causing MHE to diverge trying to fit physics at wrong altitude.
	if (!_ref_proj.isInitialized()) { return; }

	// Guard 2: reject if EKF2 has not yet produced valid position estimates.
	if (!lpos.xy_valid || !lpos.z_valid) { return; }

	// Guard 3: reject non-finite values (NaN / Inf) that would poison MHE.
	if (!PX4_ISFINITE(lpos.x) || !PX4_ISFINITE(lpos.y) || !PX4_ISFINITE(lpos.z)) { return; }

	// Subtract the arm-time NED origin so MHE's position measurements are
	// arm-relative, matching the real-flight path where set_gps_origin() is
	// called with the arm-time WGS84 fix. Without this, _gps_north = lpos.x
	// would be relative to the EKF2 origin (lpos.ref_lat/lon) while the MPC
	// state's Xm = lpos.x - _arm_origin_x is relative to the arm position,
	// creating a constant (_arm_origin) offset that the EKF↔MHE blend
	// injects into cur_x as the blend ramps from 0 to 1.
	// When set_ned_origin() has not been called yet (pre-arm), _ned_origin_*
	// default to zero and this reduces to the previous behaviour.
	_gps_north   = (double)(lpos.x - _ned_origin_x);
	_gps_east    = (double)(lpos.y - _ned_origin_y);
	// Convert NED-down to altitude MSL for MHE measurement model. The arm
	// altitude offset (relative to EKF2 ref) is folded into _ref_alt_msl by
	// RocketMPC's set_gps_origin() call (which uses arm-position MSL), so the
	// resulting y[9] stays consistent with the MHE launch_alt parameter.
	_gps_alt_msl = _ref_alt_msl + (double)(-(lpos.z - _ned_origin_z));

	_gps_vel_valid = lpos.v_xy_valid && lpos.v_z_valid
		&& PX4_ISFINITE(lpos.vx) && PX4_ISFINITE(lpos.vy) && PX4_ISFINITE(lpos.vz);

	if (_gps_vel_valid) {
		_gps_vn = (double)lpos.vx;
		_gps_ve = (double)lpos.vy;
		_gps_vd = (double)lpos.vz;
	}

	_gps_valid = true;

	// Rate-limit GPS freshness to match the real GPS cadence (~5-10 Hz).
	// update_from_lpos() is called at IMU rate (100+ Hz) but the underlying
	// EKF2 position only changes meaningfully when a new GPS sample is fused.
	// Without this gate, every call stamps a new _last_gps_update_us, making
	// build_measurement() see gps_fresh=true at 100 Hz instead of ~10 Hz.
	// MHE then assigns full GPS weight to ALL 20 horizon stages (instead
	// of ~2), over-constraining the position fit and inflating w_norm
	// → quality stuck at ~0.30 → blend=0 for the entire flight.
	static constexpr hrt_abstime MIN_GPS_INTERVAL_US = 80'000;  // 80ms → ~12 Hz max
	const hrt_abstime now_lpos = hrt_absolute_time();

	if (_last_gps_update_us == 0 ||
	    (now_lpos - _last_gps_update_us) >= MIN_GPS_INTERVAL_US) {
		_last_gps_update_us = now_lpos;
	}
}

void SensorBridge::update_baro(const vehicle_air_data_s &air)
{
	if (air.timestamp == 0) { return; }

	if (!PX4_ISFINITE(air.baro_alt_meter)) { return; }

	_baro_valid = true;
	_last_baro_update_us = hrt_absolute_time();
}

SensorMeasurement SensorBridge::build_measurement(
	const sensor_combined_s &sc,
	const vehicle_air_data_s &air)
{
	SensorMeasurement m{};

	// Drop GPS validity if no update arrived within the staleness window.
	// Prevents pushing frozen position/velocity into the MHE when the GPS
	// feed dies mid-flight.
	const hrt_abstime now = hrt_absolute_time();
	if (_gps_valid && _last_gps_update_us > 0
	    && (now - _last_gps_update_us) > GPS_STALE_TIMEOUT_US) {
		_gps_valid = false;
	}

	// Drop baro validity if no update arrived within the staleness window.
	// Unlike GPS (which gates m.valid and blocks the entire measurement),
	// stale baro only sets m.baro_valid=false so the caller can warn/log.
	// The measurement is still pushed — losing baro alone should not
	// discard the valuable IMU+GPS data in y[0..5] and y[7..12]. The stale
	// y[6] slot is filled with GPS altitude MSL below so the MHE baro model
	// (y[6] = h*H_SCALE + launch_alt) sees a sane residual instead of a
	// frozen value pulling the h-state toward the last pre-failure altitude.
	if (_baro_valid && _last_baro_update_us > 0
	    && (now - _last_baro_update_us) > BARO_STALE_TIMEOUT_US) {
		_baro_valid = false;
	}

	m.valid = (sc.timestamp > 0) && _gps_valid;
	m.baro_valid = _baro_valid;

	// Gyroscope (body frame, rad/s) — raw from IMU
	m.y[0] = (double)sc.gyro_rad[0];
	m.y[1] = (double)sc.gyro_rad[1];
	m.y[2] = (double)sc.gyro_rad[2];

	// Accelerometer (body frame, m/s²) — raw from IMU
	m.y[3] = (double)sc.accelerometer_m_s2[0];
	m.y[4] = (double)sc.accelerometer_m_s2[1];
	m.y[5] = (double)sc.accelerometer_m_s2[2];

	// Barometric altitude (m). When baro is fresh use the raw reading; when
	// stale fall back to GPS altitude MSL (already in the same frame as the
	// MHE baro measurement model). The caller-side gate on m.valid ensures
	// this fallback is only reached when _gps_valid is true (otherwise the
	// whole measurement is dropped), so _gps_alt_msl is guaranteed fresh
	// within GPS_STALE_TIMEOUT_US. Without this fallback the last pre-
	// failure baro reading would stay frozen on y[6] and pull MHE's h-state
	// toward the failure altitude, producing large altitude errors if the
	// rocket keeps climbing or descends after baro failure.
	m.y[6] = _baro_valid ? (double)air.baro_alt_meter : _gps_alt_msl;

	// GPS position relative to arm origin (local NED, meters)
	m.y[7]  = _gps_north;
	m.y[8]  = _gps_east;
	m.y[9]  = _gps_alt_msl;

	// GPS velocity (NED frame, m/s). When velocity is not valid, zero
	// the slots so stale readings don't leak into the MHE measurement.
	// NOTE: when !_gps_vel_valid these are zero, but MHE must NOT trust
	// them — see gps_vel_valid flag below which tells MHE to zero the
	// velocity rows in its weight matrix.
	m.y[10] = _gps_vel_valid ? _gps_vn : 0.0;
	m.y[11] = _gps_vel_valid ? _gps_ve : 0.0;
	m.y[12] = _gps_vel_valid ? _gps_vd : 0.0;
	m.gps_vel_valid = _gps_vel_valid;

	// GPS freshness: true only if the underlying GPS timestamp actually
	// changed since the previous build_measurement() call. At 50 Hz MPC
	// cadence vs 10 Hz GPS, only 1 in 5 calls will see a fresh sample.
	// MHE uses this to zero the GPS rows of W on stale stages, preventing
	// repeated samples from inflating GPS confidence by √(stage_count).
	m.gps_fresh = (_last_gps_update_us != _prev_gps_update_us);
	_prev_gps_update_us = _last_gps_update_us;

	return m;
}
