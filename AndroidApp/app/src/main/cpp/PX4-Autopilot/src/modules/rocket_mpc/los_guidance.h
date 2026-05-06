#pragma once

#include <cmath>

struct LosConfig {
	float target_x          = 2600.0f;
	float target_h          = 0.0f;     // target_alt - launch_alt
	float impact_angle_deg  = -30.0f;
	float impact_blend_start = 0.90f;  // تحسين بدون MHE: بدء impact-shaping مبكراً (kept Python parity)
	float impact_blend_end   = 0.99f;
	float cruise_progress   = 0.65f;
	float gamma_natural_rad = 0.0f;
	float burn_time         = 0.0f;
};

struct LosResult {
	float gamma_ref{0.0f};
	float chi_ref{0.0f};
	float dx_to_target{0.0f};
};

class LosGuidance {
public:
	LosGuidance() = default;

	void configure(const LosConfig &cfg);

	void configure_target(float target_x, float target_h, float impact_angle_deg) {
		_cfg.target_x = target_x;
		_cfg.target_h = target_h;
		_cfg.impact_angle_deg = impact_angle_deg;
	}

	void set_gamma_natural(float gamma_natural_rad) {
		_cfg.gamma_natural_rad = gamma_natural_rad;
		// Pre-launch updates: keep the rate-limiter seed in sync with the
		// latest natural gamma, so a transient EKF2 attitude error at arm
		// (e.g., 86° spike) that produces a wrong gamma_natural=1.5 rad
		// does not leave _gamma_ref_prev stuck at 1.5 even after the
		// estimate corrects to 15°. Without this, the rate-limit (18°/s)
		// keeps gamma_ref tracking the stale value for ~5s post-launch.
		// The caller (RocketMPC) only invokes this pre-launch and at arm,
		// so overwriting _gamma_ref_prev here is safe — any in-flight
		// rate-limited transition is handled by compute() not this setter.
		_gamma_ref_prev = gamma_natural_rad;
	}

	void set_burn_time(float burn_time) {
		_cfg.burn_time = burn_time;
	}

	// Keep the LOS copy of cruise_progress in sync with RocketMPC's config
	// when ROCKET_CRUISE_P is edited at runtime. Without this setter,
	// parameters_update() only updates the MPC side and LOS keeps the value
	// captured at init(), so the dive onset in the guidance reference and
	// the dive-window weights in the solver can drift out of sync.
	void set_cruise_progress(float cruise_progress) {
		_cfg.cruise_progress = cruise_progress;
	}

	LosResult compute(float x_pos, float y_pos, float altitude,
			  float t_flight, float dt);

	float gamma_ref_prev() const { return _gamma_ref_prev; }
	float chi_ref_prev()   const { return _chi_ref_prev; }

	void set_gamma_prev(float g) { _gamma_ref_prev = g; }
	void set_chi_prev(float c)   { _chi_ref_prev = c; }

	void reset() {
		_gamma_ref_prev = _cfg.gamma_natural_rad;
		_chi_ref_prev = 0.0f;
		_cruise_alt_target = 0.0f;
		_cruise_alt_set = false;
		_last_t = -1.0f;
	}

	bool cruise_alt_set() const { return _cruise_alt_set; }
	float cruise_alt_target() const { return _cruise_alt_target; }

private:
	LosConfig _cfg;

	float _gamma_ref_prev{0.0f};
	float _chi_ref_prev{0.0f};
	float _cruise_alt_target{0.0f};
	bool  _cruise_alt_set{false};
	float _last_t{-1.0f};   // last t_flight seen by compute() — used to
	                        // derive dt between LOS calls, independent of
	                        // the caller's per-cycle dt (which may be the
	                        // IMU tick, not the LOS tick).
};
