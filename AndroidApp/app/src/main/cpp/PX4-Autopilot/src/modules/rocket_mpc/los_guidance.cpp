#include "los_guidance.h"

static constexpr float DEG2RAD = (float)M_PI / 180.0f;

static inline float hermite_s(float s)
{
	s = (s < 0.0f) ? 0.0f : ((s > 1.0f) ? 1.0f : s);
	return 3.0f * s * s - 2.0f * s * s * s;
}

void LosGuidance::configure(const LosConfig &cfg)
{
	_cfg = cfg;
	_gamma_ref_prev = cfg.gamma_natural_rad;
	_chi_ref_prev   = 0.0f;
	_cruise_alt_set = false;
	_last_t = -1.0f;
}

LosResult LosGuidance::compute(float x_pos, float y_pos, float altitude,
			       float t_flight, float dt)
{
	LosResult res;

	float dx = _cfg.target_x - x_pos;
	float dh = _cfg.target_h - altitude;
	float dy = -y_pos;

	float range_to_target = sqrtf(dx * dx + dy * dy);
	float dx_safe = (dx > 0.5f) ? dx : 0.5f;

	res.dx_to_target = dx_safe;

	// Pure LOS gamma
	float range_safe = (range_to_target > 1.0f) ? range_to_target : 1.0f;
	float gamma_los = atan2f(dh, range_safe);

	// When we have overshot the target (dx < 0), clamp the dive angle so
	// the guidance doesn't demand an extreme gamma during flyby.
	if (dx < 0.0f && gamma_los < -60.0f * DEG2RAD) {
		gamma_los = -60.0f * DEG2RAD;
	}

	// Impact angle blending
	float impact_rad = _cfg.impact_angle_deg * DEG2RAD;
	float progress = 1.0f - dx_safe / ((_cfg.target_x > 1.0f) ? _cfg.target_x : 1.0f);
	progress = (progress < 0.0f) ? 0.0f : ((progress > 1.0f) ? 1.0f : progress);

	float p0 = _cfg.impact_blend_start;
	float p1 = _cfg.impact_blend_end;
	if (p1 <= p0 + 1e-3f) { p1 = p0 + 0.05f; }  // guard: prevent div-by-zero
	float k_impact;

	if (progress < p0) {
		k_impact = 0.0f;

	} else if (progress > p1) {
		k_impact = 1.0f;

	} else {
		k_impact = hermite_s((progress - p0) / (p1 - p0));
	}

	float gamma_ref_los = (1.0f - k_impact) * gamma_los + k_impact * impact_rad;

	// Clamp gamma_ref
	const float gamma_min = fminf(-45.0f, _cfg.impact_angle_deg) * DEG2RAD;
	if (gamma_ref_los < gamma_min) { gamma_ref_los = gamma_min; }

	if (gamma_ref_los >  15.0f * DEG2RAD) { gamma_ref_los =  15.0f * DEG2RAD; }

	// Chi reference — use actual dx (with tiny guard to avoid atan2 singularity
	// when directly over the target). Using dx_safe (clamped to 0.5) would point
	// chi_ref sideways after overshoot (dx<0) instead of pointing back toward
	// the target. Example: dx=-100m, dy=-50m:
	//   wrong (dx_safe=0.5):  atan2(-50, 0.5)  = -89 deg   (points west)
	//   right (dx=-100):      atan2(-50,-100)  = -153 deg  (points at target)
	float dx_chi = dx;
	if (fabsf(dx_chi) < 0.5f) {
		dx_chi = (dx >= 0.0f) ? 0.5f : -0.5f;
	}
	float chi_ref_los = atan2f(dy, dx_chi);

	// Phase blending: burn → cruise → dive
	float t_rel_burn = t_flight - (_cfg.burn_time - 0.5f);
	float s_burn = 0.5f * (1.0f + tanhf(t_rel_burn / 0.8f));

	// Dive transition
	float s_dive;
	{
		float cp0 = _cfg.cruise_progress;
		float cp1 = cp0 + 0.10f;

		if (cp1 > 0.95f) { cp1 = 0.95f; }

		if (progress < cp0) { s_dive = 0.0f; }
		else if (progress > cp1) { s_dive = 1.0f; }
		else { s_dive = hermite_s((progress - cp0) / (cp1 - cp0)); }
	}

	// Capture cruise altitude
	if (s_burn > 0.95f && !_cruise_alt_set) {
		_cruise_alt_target = altitude;
		_cruise_alt_set = true;
	}

	float gamma_cruise = s_dive * gamma_ref_los;
	float gamma_ref_raw = (1.0f - s_burn) * _cfg.gamma_natural_rad + s_burn * gamma_cruise;
	float chi_ref_raw   = chi_ref_los;

	// Rate limiting (10 deg/s)
	// Use the wall time *between LOS calls*, NOT the caller's per-cycle dt.
	// The caller may run at IMU rate (e.g. 200–500 Hz) while LOS is rate-gated
	// to ~50 Hz upstream; passing the IMU dt would shrink max_dg 4–10x and
	// make gamma_ref lag the true LOS target by seconds during dive onset.
	float dt_internal;
	if (_last_t < 0.0f) {
		// First call: trust caller's dt, or fall back to 20 ms
		dt_internal = (dt > 1e-3f) ? dt : 0.02f;
	} else {
		dt_internal = t_flight - _last_t;
		if (dt_internal < 1e-3f) { dt_internal = 1e-3f; }
		if (dt_internal > 0.05f) { dt_internal = 0.05f; } // cap post-idle jump
	}
	_last_t = t_flight;

	// gamma_ref rate limit: 18°/s (was 10°/s). The lower limit was tuned for
	// MHE-blended γ which was biased steep; with EKF2-only state, 10°/s
	// starves terminal pitch-down. 18°/s gives γ_ref enough headroom to
	// track impact_angle without exciting pitch-axis oscillation.
	static constexpr float MAX_REF_RATE = 18.0f * DEG2RAD;  // gamma & chi rate limit
	float max_dg = MAX_REF_RATE * dt_internal;

	float dg = gamma_ref_raw - _gamma_ref_prev;
	float dc = chi_ref_raw   - _chi_ref_prev;

	// Clamp rate
	if (dg >  max_dg) { dg =  max_dg; }

	if (dg < -max_dg) { dg = -max_dg; }

	if (dc >  max_dg) { dc =  max_dg; }

	if (dc < -max_dg) { dc = -max_dg; }

	res.gamma_ref = _gamma_ref_prev + dg;
	res.chi_ref   = _chi_ref_prev   + dc;

	_gamma_ref_prev = res.gamma_ref;
	_chi_ref_prev   = res.chi_ref;

	return res;
}
