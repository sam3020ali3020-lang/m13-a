#include "mpc_controller.h"

#include <cstring>
#include <cstdio>
#include <drivers/drv_hrt.h>

#ifdef MODULE_NAME
#  undef MODULE_NAME
#endif
#define MODULE_NAME "rocket_mpc"
#include <px4_platform_common/log.h>

// Helper: Hermite smooth step  3s² - 2s³
static inline float hermite(float s)
{
	s = (s < 0.0f) ? 0.0f : ((s > 1.0f) ? 1.0f : s);
	return 3.0f * s * s - 2.0f * s * s * s;
}

// ===================================================================
MpcController::~MpcController()
{
	destroy();
}

void MpcController::destroy()
{
	if (_capsule) {
		m130_rocket_acados_free(_capsule);
		m130_rocket_acados_free_capsule(_capsule);
		_capsule = nullptr;
	}

	_destroy_sim_solver();
	_solver_ready = false;
}

bool MpcController::init(const MpcConfig &cfg)
{
	_cfg = cfg;

	_capsule = m130_rocket_acados_create_capsule();

	if (!_capsule) {
		PX4_ERR("MPC create_capsule failed");
		return false;
	}

	int status = m130_rocket_acados_create(_capsule);

	if (status != 0) {
		PX4_ERR("MPC acados_create returned %d", status);
		m130_rocket_acados_free_capsule(_capsule);
		_capsule = nullptr;
		return false;
	}

	_nlp_config = m130_rocket_acados_get_nlp_config(_capsule);
	_nlp_dims   = m130_rocket_acados_get_nlp_dims(_capsule);
	_nlp_in     = m130_rocket_acados_get_nlp_in(_capsule);
	_nlp_out    = m130_rocket_acados_get_nlp_out(_capsule);
	_nlp_solver = m130_rocket_acados_get_nlp_solver(_capsule);

	_solver_ready = true;

	// Initialize sim solver for Smith predictor (only if delay > 0)
	if (_cfg.servo_delay_s > 1e-4f) {
		if (!_init_sim_solver()) {
			PX4_WARN("MPC sim solver init failed — delay compensation disabled");
		}
	}

	PX4_INFO("MPC solver init N=%d tf=%.1f delay=%.3f", _cfg.N_horizon, (double)_cfg.tf, (double)_cfg.servo_delay_s);
	return true;
}

// ===================================================================
// Runtime parameters: [mass, thrust]
// ===================================================================
void MpcController::get_params(float t, double p[MPC_NP]) const
{
	float mass, thrust;

	if (t < _cfg.burn_time) {
		float f = t / _cfg.burn_time;
		mass = _cfg.mass_full - f * (_cfg.mass_full - _cfg.mass_dry);
		float t_tailoff_start = _cfg.burn_time - _cfg.t_tail;

		if (t > t_tailoff_start) {
			float s = (t - t_tailoff_start) / _cfg.t_tail;
			thrust = _cfg.thrust_plateau * (1.0f - s) * (1.0f - s) * (1.0f - s);

		} else {
			thrust = _cfg.thrust_plateau;
		}

	} else {
		mass = _cfg.mass_dry;
		thrust = 0.0f;
	}

	p[0] = (double)mass;
	p[1] = (double)thrust;
}

// ===================================================================
// Weight scheduling — exact port from Python _update_weights_for_phase
// ===================================================================
void MpcController::_compute_weights(float t, float gamma, float gamma_ref_prev,
				     float phi, float p_rate,
				     float alpha, float q_rate, float x_pos,
				     bool cruise_alt_set, float h_ref_scaled,
				     double W[MPC_NY], double W_e[MPC_NYN])
{
	const bool is_boost = (t < _cfg.burn_time);
	// Weight-blend window scales with t_tail so the transition tracks the
	// thrust curve in get_params() (thrust tail-off spans exactly t_tail).
	// Post-burnout the window is 2*t_tail to let weights settle smoothly.
	const float t_tailoff_start = _cfg.burn_time - _cfg.t_tail;
	const float t_tailoff_end   = _cfg.burn_time + 2.0f * _cfg.t_tail;
	const bool in_tailoff = (t_tailoff_start < t) && (t < t_tailoff_end);

	float gamma_base, q_w, r_w, de_rate_w, dr_rate_w, alpha_w_boost;

	if (is_boost && !in_tailoff) {
		gamma_base   = 250.0f;
		q_w          = 200.0f;
		r_w          = 80.0f;
		de_rate_w    = 120.0f;
		dr_rate_w    = 60.0f;
		alpha_w_boost = 120.0f;

	} else if (in_tailoff) {
		float s_to = (t - t_tailoff_start) / (t_tailoff_end - t_tailoff_start);
		s_to = (s_to < 0.0f) ? 0.0f : ((s_to > 1.0f) ? 1.0f : s_to);
		const float s_peak = 0.67f;
		float s_up = s_to / s_peak;
		s_up = (s_up > 1.0f) ? 1.0f : s_up;

		float gamma_act = 250.0f * (1.0f - s_up) * (1.0f - s_up) + 2.0f;
		float q_act     = 200.0f + 800.0f * s_up;
		float r_act     = 80.0f + 200.0f * s_up;
		float de_act    = 120.0f * (1.0f - s_up) + 200.0f * s_up;
		float dr_act    = 60.0f + 100.0f * s_up;
		float alpha_act = 120.0f + 800.0f * s_up;

		if (s_to > s_peak) {
			float s_f = (s_to - s_peak) / (1.0f - s_peak);
			float h = hermite(s_f);
			gamma_base    = gamma_act + (100.0f - gamma_act) * h;
			q_w           = q_act     + (60.0f  - q_act)     * h;
			r_w           = r_act     + (40.0f  - r_act)     * h;
			de_rate_w     = de_act    + (20.0f  - de_act)    * h;
			dr_rate_w     = dr_act    + (20.0f  - dr_act)    * h;
			alpha_w_boost = alpha_act + (40.0f  - alpha_act) * h;

		} else {
			gamma_base    = gamma_act;
			q_w           = q_act;
			r_w           = r_act;
			de_rate_w     = de_act;
			dr_rate_w     = dr_act;
			alpha_w_boost = alpha_act;
		}

	} else {
		// Coast/terminal
		gamma_base    = 100.0f;
		q_w           = 60.0f;
		r_w           = 40.0f;
		de_rate_w     = 20.0f;
		dr_rate_w     = 20.0f;
		alpha_w_boost = 80.0f;  // higher alpha penalty during coast (was 40)
	}

	// Adaptive gamma error boost
	// NOTE: This must be the tracking error |gamma - gamma_ref|, NOT |gamma|.
	// Python reference: m130_mpc_autopilot.py:644
	//   gamma_err_deg = abs(math.degrees(gamma - self._gamma_ref_prev))
	float gamma_err_deg = fabsf((gamma - gamma_ref_prev) * 180.0f / (float)M_PI);
	float alpha_deg_abs = fabsf(alpha * 180.0f / (float)M_PI);
	float q_rate_abs    = fabsf(q_rate * 180.0f / (float)M_PI);

	if (is_boost && !in_tailoff) {
		if (gamma_err_deg < 5.0f && q_rate_abs > 3.0f) {
			q_w = q_w + 300.0f * (1.0f - gamma_err_deg / 5.0f);
		}

		if (alpha_deg_abs > 1.0f) {
			float a_sc = alpha_deg_abs / 3.0f;
			alpha_w_boost = alpha_w_boost + 200.0f * ((a_sc > 3.0f) ? 3.0f : a_sc);
		}
	} else {
		// Coast/terminal: boost alpha penalty when alpha grows (delay makes this worse)
		if (alpha_deg_abs > 2.0f) {
			float a_sc = alpha_deg_abs / 5.0f;
			alpha_w_boost = alpha_w_boost + 400.0f * ((a_sc > 4.0f) ? 4.0f : a_sc);
		}
	}

	// Altitude cruise weight: active after burnout when cruise alt is captured,
	// fades out during dive phase (matches Python baseline).
	float h_cruise_w = 0.0f;
	if (!is_boost && cruise_alt_set) {
		float t_since_burn = t - _cfg.burn_time;
		float early_boost = (1.0f - t_since_burn / 3.0f);
		if (early_boost < 0.0f) { early_boost = 0.0f; }

		// Compute s_dive from progress (same as LOS guidance)
		float dx_safe = _cfg.target_x - x_pos;
		if (dx_safe < 0.5f) { dx_safe = 0.5f; }
		float progress = 1.0f - dx_safe / ((_cfg.target_x > 1.0f) ? _cfg.target_x : 1.0f);
		progress = (progress < 0.0f) ? 0.0f : ((progress > 1.0f) ? 1.0f : progress);

		// Python reference: m130_mpc_autopilot.py:530
		//   p1 = min(p0 + 0.10, 0.95)  # 10% window, clipped at 95%
		float cp0 = _cfg.cruise_progress;
		float cp1 = cp0 + 0.10f;
		if (cp1 > 0.95f) { cp1 = 0.95f; }
		float s_dive;
		if (progress < cp0) { s_dive = 0.0f; }
		else if (progress > cp1) { s_dive = 1.0f; }
		else {
			float sf = (progress - cp0) / (cp1 - cp0);
			s_dive = hermite(sf);
		}

		h_cruise_w = (15.0f + 35.0f * early_boost) * (1.0f - s_dive);
	}

	// Roll recovery weighting — triggered on EITHER angle OR rate.
	// Rationale: roll mode has a fast time-constant (~75 ms for low Ixx).
	// Waiting until phi exceeds 5 deg leaves no horizon for the solver to
	// recover when the disturbance enters as a body rate (motor thrust
	// misalignment, gust, etc.) — by the time |phi|>5° the rate may have
	// grown well past the solver's authority. We therefore trigger
	// recovery as soon as |p| crosses a small threshold and use the max
	// of the angle-based and rate-based scalings.
	const float roll_recovery_start = 5.0f * (float)M_PI / 180.0f;
	const float roll_recovery_full  = 20.0f * (float)M_PI / 180.0f;
	float roll_abs = fabsf(phi);
	float roll_recovery_angle;

	if (roll_abs <= roll_recovery_start) {
		roll_recovery_angle = 0.0f;

	} else if (roll_abs >= roll_recovery_full) {
		roll_recovery_angle = 1.0f;

	} else {
		roll_recovery_angle = (roll_abs - roll_recovery_start) /
				      (roll_recovery_full - roll_recovery_start);
	}

	// Rate-based recovery: 0 when |p| <= 0.05 rad/s (~3 deg/s, sensor noise
	// floor), full at |p| >= 0.5 rad/s (~29 deg/s, clearly disturbed).
	const float p_rate_recovery_start = 0.05f;  // rad/s
	const float p_rate_recovery_full  = 0.50f;  // rad/s
	float p_abs = fabsf(p_rate);
	float roll_recovery_rate;

	if (p_abs <= p_rate_recovery_start) {
		roll_recovery_rate = 0.0f;

	} else if (p_abs >= p_rate_recovery_full) {
		roll_recovery_rate = 1.0f;

	} else {
		roll_recovery_rate = (p_abs - p_rate_recovery_start) /
				     (p_rate_recovery_full - p_rate_recovery_start);
	}

	float roll_recovery = (roll_recovery_angle > roll_recovery_rate) ?
			      roll_recovery_angle : roll_recovery_rate;

	// Baseline weights raised so MPC treats roll on equal footing with
	// pitch/yaw from t=0 (previously p_w=20 vs q_w=200 left roll under-
	// damped in nominal flight, exposing the system to roll runaway when
	// any disturbance entered before phi crossed the recovery threshold).
	// chi tracking — مرفوع 3× لمواجهة انجراف الرياح بقوة (matches Python sim).
	// Old: max(25, 120*(1-0.55*r)) — was leaving wind-induced cross-range
	// drift uncompensated. With wind 4 m/s east: SITL CR drift was -95m
	// vs Python -37m. Bumping chi_w by 3× brings them into parity.
	float chi_w     = 75.0f > (360.0f * (1.0f - 0.55f * roll_recovery)) ?
			  75.0f : (360.0f * (1.0f - 0.55f * roll_recovery));
	float p_w       = 120.0f + 100.0f * roll_recovery;   // was 20 + 160*r
	float phi_w     = 200.0f + 400.0f * roll_recovery;   // was 80 + 520*r
	float da_rate_w = 10.0f - 7.0f * roll_recovery;

	// W: diagonal for [h, gamma, chi, p, q, r, alpha, beta, phi, de_rate, dr_rate, da_rate]
	W[0]  = (double)h_cruise_w;
	W[1]  = (double)(gamma_base + 400.0f);
	W[2]  = (double)chi_w;
	W[3]  = (double)p_w;
	W[4]  = (double)q_w;
	W[5]  = (double)r_w;
	W[6]  = (double)alpha_w_boost;
	W[7]  = 400.0;  // beta — مرفوع 6.6× (was 60) لقمع سوائل ترنح ياو تحت رياح عرضية (توافق مع Python sim)
	W[8]  = (double)phi_w;
	W[9]  = (double)de_rate_w;
	W[10] = (double)dr_rate_w;
	W[11] = (double)da_rate_w;

	// W_e: diagonal for [h, gamma, chi, p, q, r, alpha, beta, phi]
	W_e[0] = (double)(h_cruise_w * 0.5f);
	W_e[1] = 300.0;
	W_e[2] = (double)((40.0f > chi_w) ? 40.0f : chi_w);
	W_e[3] = (double)((40.0f > p_w) ? 40.0f : p_w);
	W_e[4] = 60.0;
	W_e[5] = 40.0;
	W_e[6] = 30.0;
	W_e[7] = 150.0;  // تغليظ للـterminal yaw alignment (was 40)
	W_e[8] = (double)((40.0f > phi_w) ? 40.0f : phi_w);
}

// ===================================================================
// Forward guess for initial trajectory
// ===================================================================
void MpcController::_forward_guess(const double x0[MPC_NX], float t0)
{
	const int N = _cfg.N_horizon;
	const float dt_h = _cfg.tf / (float)N;

	memcpy(&_x_traj[0], x0, MPC_NX * sizeof(double));

	for (int k = 0; k < N; k++) {
		const double *x = &_x_traj[k * MPC_NX];
		double *xn = &_x_traj[(k + 1) * MPC_NX];
		float t_k = t0 + k * dt_h;

		double p[MPC_NP];
		get_params(t_k, p);
		float mass = (float)p[0];
		float thrust = (float)p[1];

		float V_k = (float)x[0];

		if (V_k < 10.0f) { V_k = 10.0f; }

		float gam = (float)x[1];
		float sg = sinf(gam);
		float cg = cosf(gam);
		float al = (float)x[6];

		float V_dot = (thrust * cosf(al) - 0.5f * 1.225f * V_k * V_k * 0.0133f * 0.30f
			       - mass * G_ACCEL * sg) / mass;
		float gam_dot = -G_ACCEL * cg / V_k + thrust * sinf(al) / (mass * V_k);

		memcpy(xn, x, MPC_NX * sizeof(double));
		xn[0] = (double)(V_k + V_dot * dt_h);

		if (xn[0] < 10.0) { xn[0] = 10.0; }

		xn[1] = (double)(gam + gam_dot * dt_h);

		for (int j = 3; j < 9; j++) { xn[j] *= 0.95; }

		xn[9]  = x[9]  + (double)(V_k * sg * 0.01f * dt_h);
		xn[10] = x[10] + (double)(V_k * cg * cosf((float)x[2]) * 0.001f * dt_h);
		xn[11] = x[11] + (double)(V_k * cg * sinf((float)x[2]) * 0.001f * dt_h);

		// No servo dynamics in NX=15 model: delta_s_dot = u (rate command)
	}
}

// ===================================================================
// Full reset for re-arm — wipe all flight state
// ===================================================================
void MpcController::reset()
{
	if (_capsule) {
		m130_rocket_acados_reset(_capsule, 1);
	}

	_warm = false;
	_consec_fails = 0;
	_consec_ok = 0;
	_solve_count = 0;
	_last_delta_e = 0.0f;
	_last_delta_r = 0.0f;
	_last_delta_a = 0.0f;
	_last_solve_t = -1.0f;
	_last_solve_time_ms = 0.0f;
	_cmd_buf_head = 0;
	_cmd_buf_count = 0;
	memset(_x_traj, 0, sizeof(_x_traj));

	PX4_INFO("MPC reset for re-arm");
}

// ===================================================================
// Reinitialize solver after consecutive failures
// ===================================================================
void MpcController::_reinit(const double x_mpc[MPC_NX])
{
	m130_rocket_acados_reset(_capsule, 1);
	_forward_guess(x_mpc, _last_solve_t > 0.0f ? _last_solve_t : 0.0f);

	for (int k = 0; k <= _cfg.N_horizon; k++) {
		ocp_nlp_out_set(_nlp_config, _nlp_dims, _nlp_out, _nlp_in, k, "x",
				&_x_traj[k * MPC_NX]);
	}

	double u_zero[MPC_NU] = {0.0, 0.0, 0.0};

	for (int k = 0; k < _cfg.N_horizon; k++) {
		ocp_nlp_out_set(_nlp_config, _nlp_dims, _nlp_out, _nlp_in, k, "u", u_zero);
	}

	_warm = false;
	_consec_fails = 0;
	_consec_ok = 0;
	PX4_WARN("MPC solver reinitialized");
}

// ===================================================================
// Main solve
// ===================================================================
MpcSolveResult MpcController::solve(const double x_mpc_in[MPC_NX],
				    float gamma_ref, float chi_ref, float phi_ref,
				    float h_ref_scaled, bool cruise_alt_set,
				    float t_flight, float dt,
				    float x_pos, float altitude)
{
	MpcSolveResult result{};
	result.delta_e = _last_delta_e;
	result.delta_r = _last_delta_r;
	result.delta_a = _last_delta_a;
	result.status = -1;
	result.valid = false;

	if (!_solver_ready) { return result; }

	// --- Forward prediction for servo delay compensation ---
	// Predict the state that will exist when the servo actually executes
	// our command (τ_d seconds in the future). MPC then plans for that
	// future state instead of the current one.
	double x_mpc[MPC_NX];
	if (_cfg.servo_delay_s > 1e-4f) {
		_predict_ahead(x_mpc_in, t_flight, _cfg.servo_delay_s, x_mpc);
		_prune_cmd_buffer(t_flight);
	} else {
		memcpy(x_mpc, x_mpc_in, MPC_NX * sizeof(double));
	}

	const int N = _cfg.N_horizon;
	float V = (float)x_mpc[0];  // uses predicted state if delay > 0

	// Adaptive time horizon
	float dx_to_target = _cfg.target_x - x_pos;

	if (dx_to_target < 0.5f) { dx_to_target = 0.5f; }

	float tf_use = _cfg.tf;

	if (V > 30.0f && dx_to_target > 0.0f) {
		float t_to_target = dx_to_target / V;
		tf_use = t_to_target * 0.8f;

		if (tf_use < 2.0f) { tf_use = 2.0f; }

		if (tf_use > _cfg.tf) { tf_use = _cfg.tf; }
	}

	// dt_h is the MPC's internal prediction stage (tf/N = 50ms).
	// The actual control cadence is 20ms (see RocketMPC.cpp:947).
	// This mismatch is a known design compromise — see docs/mpc_design.md.
	float dt_h = tf_use / (float)N;

	// Set initial state constraint
	if (_solve_count < 2) { PX4_DEBUG("MPC solve: N=%d V=%.1f tf=%.1f dt=%.4f", N, (double)V, (double)tf_use, (double)dt_h); }
	ocp_nlp_constraints_model_set(_nlp_config, _nlp_dims, _nlp_in, _nlp_out, 0,
				      "lbx", (void *)x_mpc);
	ocp_nlp_constraints_model_set(_nlp_config, _nlp_dims, _nlp_in, _nlp_out, 0,
				      "ubx", (void *)x_mpc);
	if (_solve_count < 2) { PX4_DEBUG("MPC constraints set"); }

	// Compute weight schedule
	double W_diag[MPC_NY];
	double W_e_diag[MPC_NYN];
	_compute_weights(t_flight, (float)x_mpc[1], gamma_ref, (float)x_mpc[8],
			 (float)x_mpc[3],
			 (float)x_mpc[6], (float)x_mpc[4], x_pos,
			 cruise_alt_set, h_ref_scaled,
			 W_diag, W_e_diag);

	// Build full diagonal weight matrices
	double W_mat[MPC_NY * MPC_NY];
	memset(W_mat, 0, sizeof(W_mat));

	for (int i = 0; i < MPC_NY; i++) { W_mat[i * MPC_NY + i] = W_diag[i]; }

	double W_e_mat[MPC_NYN * MPC_NYN];
	memset(W_e_mat, 0, sizeof(W_e_mat));

	for (int i = 0; i < MPC_NYN; i++) { W_e_mat[i * MPC_NYN + i] = W_e_diag[i]; }

	float h_ref = h_ref_scaled;   // cruise altitude from LOS (scaled)
	if (_solve_count < 2) { PX4_DEBUG("MPC weights computed"); }

	// Set stage references and parameters
	for (int k = 0; k < N; k++) {
		float t_k = t_flight + k * dt_h;
		double p[MPC_NP];
		get_params(t_k, p);
		m130_rocket_acados_update_params(_capsule, k, p, MPC_NP);

		// Cost weight
		ocp_nlp_cost_model_set(_nlp_config, _nlp_dims, _nlp_in, k, "W", W_mat);

		// yref = [h_ref, gamma_ref, chi_ref, 0,0,0, 0,0, phi_ref, 0,0,0]
		double yref[MPC_NY] = {};
		yref[0] = (double)h_ref;
		yref[1] = (double)gamma_ref;
		yref[2] = (double)chi_ref;
		yref[8] = (double)phi_ref;
		ocp_nlp_cost_model_set(_nlp_config, _nlp_dims, _nlp_in, k, "yref", yref);
	}

	// Terminal stage
	{
		double p_term[MPC_NP];
		get_params(t_flight + tf_use, p_term);
		m130_rocket_acados_update_params(_capsule, N, p_term, MPC_NP);

		ocp_nlp_cost_model_set(_nlp_config, _nlp_dims, _nlp_in, N, "W", W_e_mat);

		double yref_e[MPC_NYN] = {};
		yref_e[0] = (double)h_ref;
		yref_e[1] = (double)gamma_ref;
		yref_e[2] = (double)chi_ref;
		yref_e[8] = (double)phi_ref;
		ocp_nlp_cost_model_set(_nlp_config, _nlp_dims, _nlp_in, N, "yref", yref_e);
	}

	if (_solve_count < 2) { PX4_DEBUG("MPC params+costs set for all stages"); }

	// Update time steps
	double new_time_steps[MPC_N];

	for (int k = 0; k < N; k++) { new_time_steps[k] = (double)dt_h; }

	int ts_status = m130_rocket_acados_update_time_steps(_capsule, N, new_time_steps);

	if (ts_status != 0) {
		// Silent failure would leave the integrator on stale dt values
		// while upstream logic (cost scaling, trajectory prediction) uses
		// the new dt_h — the solver reports success on a physically
		// inconsistent problem. Count as a fail and log with throttling.
		_consec_fails++;

		if (_consec_fails <= 3 || (_consec_fails % 50) == 0) {
			PX4_WARN("MPC update_time_steps failed status=%d dt_h=%.4f N=%d",
				 ts_status, (double)dt_h, N);
		}
	}

	if (_solve_count < 2) { PX4_DEBUG("MPC time_steps updated"); }

	// Warm start / initialization
	if (!_warm) {
		_forward_guess(x_mpc, t_flight);

		for (int k = 0; k <= N; k++) {
			ocp_nlp_out_set(_nlp_config, _nlp_dims, _nlp_out, _nlp_in, k, "x",
					&_x_traj[k * MPC_NX]);
		}

		double u_zero[MPC_NU] = {0.0, 0.0, 0.0};

		for (int k = 0; k < N; k++) {
			ocp_nlp_out_set(_nlp_config, _nlp_dims, _nlp_out, _nlp_in, k, "u", u_zero);
		}

	} else {
		// Shift trajectory one step
		double x_buf[MPC_NX];

		for (int k = 0; k < N - 1; k++) {
			ocp_nlp_out_get(_nlp_config, _nlp_dims, _nlp_out, k + 1, "x", x_buf);
			ocp_nlp_out_set(_nlp_config, _nlp_dims, _nlp_out, _nlp_in, k, "x", x_buf);

			double u_buf[MPC_NU];
			ocp_nlp_out_get(_nlp_config, _nlp_dims, _nlp_out, k + 1, "u", u_buf);
			ocp_nlp_out_set(_nlp_config, _nlp_dims, _nlp_out, _nlp_in, k, "u", u_buf);
		}

		ocp_nlp_out_get(_nlp_config, _nlp_dims, _nlp_out, N, "x", x_buf);
		ocp_nlp_out_set(_nlp_config, _nlp_dims, _nlp_out, _nlp_in, N - 1, "x", x_buf);
	}

	// Set current state
	ocp_nlp_out_set(_nlp_config, _nlp_dims, _nlp_out, _nlp_in, 0, "x", (void *)x_mpc);
	if (_solve_count < 2) { PX4_DEBUG("MPC init guess set, warm=%d", _warm); }

	// RTI iterations
	int n_rti = _warm ? 3 : 5;

	if (_consec_fails > 3) { n_rti = 8; }

	// Time budget for the entire RTI loop.  Each m130_rocket_acados_solve()
	// is a blocking call with no internal timeout.  This budget is a safety
	// net against pathological spikes only; under normal conditions all
	// n_rti iterations should complete.
	// Sizing (current config N=80, tf=1.6s, cond_N=8 on ARM64 Cortex-X4 + ADPF):
	// each warm RTI iteration ≈ 3–8 ms, so 3 warm iterations ≈ 10–25 ms nominal.
	// Budget = 90 ms is a generous safety net against pathological spikes
	// (cache miss, scheduler preemption, MHE re-init); under normal load the
	// loop completes in well under 30 ms with full headroom.
	// History: previous configs (N=200/tf=4 → ~25 ms/iter; N=100/tf=2 → ~8 ms/iter)
	// also fit comfortably under this same 90 ms budget.
	// A partial solve (fewer iterations) is safe: SQP_RTI produces a valid
	// (if suboptimal) control action after every completed iteration.
	static constexpr hrt_abstime SOLVE_BUDGET_US = 90'000;  // 90 ms

	hrt_abstime t0 = hrt_absolute_time();
	bool ok = true;
	int status = 0;
	int iters_done = 0;
	bool timed_out = false;

	for (int i = 0; i < n_rti; i++) {
		if (_solve_count < 2) { PX4_DEBUG("MPC RTI iter %d/%d", i, n_rti); }
		status = m130_rocket_acados_solve(_capsule);
		if (_solve_count < 2) { PX4_DEBUG("MPC RTI iter %d done, status=%d", i, status); }

		iters_done++;

		// [008] QP cascade fix: only break on status=1 (NaN — unrecoverable).
		// status=4 (QP didn't fully converge in HPIPM) is RECOVERABLE: HPIPM
		// retains the partial iterate, and the next RTI iteration builds on
		// it. Breaking out caused a cascade at burnout: cold-start next cycle
		// → harder QP → status=4 again → freeze fins → eventually _reinit(0)
		// → loss of control. Letting the loop continue lets the partial
		// iterate refine into a usable (sub-optimal) solution.
		if (status == 1) {
			ok = false;
			break;
		}

		if (status != 0 && status != 2) {
			ok = false;
			// Don't break — finish remaining iterations to refine the
			// partial iterate. A finite sub-optimal solution is far
			// better than freezing or zeroing the control.
		}

		// Budget check: don't start the next iteration if we're already
		// over the wall-clock budget.  The solution from the completed
		// iteration(s) is still usable.
		if ((hrt_absolute_time() - t0) >= SOLVE_BUDGET_US) {
			timed_out = (i + 1 < n_rti);  // true only if we skipped iterations

			if (timed_out) {
				PX4_WARN("MPC solve timeout after %d/%d iters (%.1f ms)",
					 iters_done, n_rti,
					 (double)((float)(hrt_absolute_time() - t0) * 1e-3f));
			}

			break;
		}
	}

	float solve_ms = (float)(hrt_absolute_time() - t0) * 1e-3f;

	_last_solve_time_ms = solve_ms;
	_solve_count++;

	// Read solution
	double u_opt[MPC_NU];
	ocp_nlp_out_get(_nlp_config, _nlp_dims, _nlp_out, 0, "u", u_opt);

	// ── Dead-time compensation: pull fin commands from a future MPC stage
	// matching the servo transport delay.  lookahead_stage is auto-computed
	// from servo_delay_s: round(servo_delay_s / dt_h).  This way, the
	// command the servo actually executes after the delay matches what MPC
	// planned for that future time.  servo_delay_s is disabled (0) — no
	// forward prediction with the NX=15 ideal-fin model.
	int la_stage = _cfg.lookahead_stage;
	if (la_stage < 1)        { la_stage = 1; }
	if (la_stage > MPC_N)    { la_stage = MPC_N; }

	double x1[MPC_NX];
	ocp_nlp_out_get(_nlp_config, _nlp_dims, _nlp_out, la_stage, "x", x1);

	bool finite_check = true;

	for (int i = 0; i < MPC_NU; i++) {
		if (!std::isfinite(u_opt[i])) { finite_check = false; }
	}

	for (int i = 0; i < MPC_NX; i++) {
		if (!std::isfinite(x1[i])) { finite_check = false; }
	}

	float de, dr, da;

	if (!finite_check) {
		// [008] QP cascade fix: NaN is the ONLY truly unrecoverable case.
		// Freeze on last known-good control and count toward _reinit threshold.
		_consec_fails++;
		_warm = false;       // force fresh forward_guess next solve — don't shift corrupted trajectory
		_consec_ok = 0;      // require 3 consecutive OK before re-enabling warm shift
		de = _last_delta_e;
		dr = _last_delta_r;
		da = _last_delta_a;

		// Diagnostic: dump input state on first few failures and every 50th after
		if (_consec_fails <= 3 || (_consec_fails % 50) == 0) {
			PX4_WARN("MPC NaN #%d status=%d t=%.2f V=%.1f gam=%.4f chi=%.4f",
				 _consec_fails, status, (double)t_flight,
				 x_mpc[0], x_mpc[1], x_mpc[2]);
			PX4_WARN("  alpha=%.4f beta=%.4f phi=%.4f h=%.4f xg=%.4f",
				 x_mpc[6], x_mpc[7], x_mpc[8], x_mpc[9], x_mpc[10]);
			PX4_WARN("  refs: gam=%.4f chi=%.4f phi=%.4f h=%.4f",
				 (double)gamma_ref, (double)chi_ref, (double)phi_ref, (double)h_ref);
		}

		// [008] Threshold raised 10 → 30 cycles (≈0.4s → ≈1.2s).
		// With sub-optimal control fallback in the !ok branch below, sustained
		// NaN is the only path here; 30 cycles is a safer panic threshold.
		if (_consec_fails >= 30) {
			_reinit(x_mpc);
		}

	} else if (!ok) {
		// [008] QP cascade fix: solver flagged failure but output is finite.
		// Use the partial-iterate solution (sub-optimal but continuous control)
		// instead of freezing. This breaks the cascade observed at burnout
		// where status=4 spikes during the propulsion→coast transition.
		// Crucially: do NOT invalidate _warm — the trajectory is still
		// close to the true optimum and warm-shift will help next cycle.
		// Do NOT increment _consec_fails — that counter is now NaN-only;
		// _reinit() must NEVER trigger from sub-optimal solves.
		de = (float)x1[12];
		dr = (float)x1[13];
		da = (float)x1[14];

		if (_solve_count <= 3 || (_solve_count % 250) == 0) {
			PX4_INFO("MPC sub-optimal status=%d t=%.2f de=%.4f dr=%.4f da=%.4f",
				 status, (double)t_flight,
				 (double)de, (double)dr, (double)da);
		}

	} else {
		_consec_fails = 0;
		de = (float)x1[12];
		dr = (float)x1[13];
		da = (float)x1[14];

		if (_solve_count <= 3) {
			PX4_DEBUG("MPC u_opt=[%.5f,%.5f,%.5f] x1_de=%.5f dr=%.5f da=%.5f",
				  u_opt[0], u_opt[1], u_opt[2],
				  x1[12], x1[13], x1[14]);
		}

		if (!_warm) {
			_consec_ok++;

			if (_consec_ok >= 3) {
				_warm = true;
			}

		}
	}

	_last_delta_e = de;
	_last_delta_r = dr;
	_last_delta_a = da;
	_last_solve_t = t_flight;

	// Add command to buffer for forward prediction
	_add_cmd_to_buffer(t_flight, de, dr, da);

	// X-fin mixing is intentionally NOT done here: RocketMPC.cpp owns the
	// per-fin saturation clamp and MHE back-solve, so it mixes the three
	// virtual channels right before publishing to the servos.
	result.delta_e = de;
	result.delta_r = dr;
	result.delta_a = da;
	result.status  = status;
	result.solve_time_ms = solve_ms;
	// Report real solver iteration count from acados, matching Python
	// m130_mpc_autopilot.py:776 which reads solver.get_stats("sqp_iter")
	// after the RTI loop.  Previously we reported n_rti (the *planned*
	// loop bound, 3/5/8), which stayed fixed even when the loop broke
	// on the very first iteration — misleading in failure logs.
	int sqp_iter_real = 0;
	ocp_nlp_get(_nlp_solver, "sqp_iter", &sqp_iter_real);
	result.sqp_iter = sqp_iter_real;
	result.valid   = ok && finite_check;

	return result;
}

// ===================================================================
// Smith predictor: acados sim solver for model-consistent forward prediction
// ===================================================================
bool MpcController::_init_sim_solver()
{
	_sim_capsule = m130_rocket_acados_sim_solver_create_capsule();

	if (!_sim_capsule) {
		PX4_ERR("MPC sim create_capsule failed");
		return false;
	}

	int status = m130_rocket_acados_sim_create(_sim_capsule);

	if (status != 0) {
		PX4_ERR("MPC sim acados_sim_create returned %d", status);
		m130_rocket_acados_sim_solver_free_capsule(_sim_capsule);
		_sim_capsule = nullptr;
		return false;
	}

	_sim_config = m130_rocket_acados_get_sim_config(_sim_capsule);
	_sim_in     = m130_rocket_acados_get_sim_in(_sim_capsule);
	_sim_out    = m130_rocket_acados_get_sim_out(_sim_capsule);
	_sim_dims   = m130_rocket_acados_get_sim_dims(_sim_capsule);

	// Set simulation step size to match MPC horizon step (dt_h = tf/N)
	double T_sim = (double)_cfg.tf / (double)_cfg.N_horizon;
	sim_in_set(_sim_config, _sim_dims, _sim_in, "T", &T_sim);

	PX4_INFO("MPC Smith predictor sim solver init (T_sim=%.4f)", T_sim);
	return true;
}

void MpcController::_destroy_sim_solver()
{
	if (_sim_capsule) {
		m130_rocket_acados_sim_free(_sim_capsule);
		m130_rocket_acados_sim_solver_free_capsule(_sim_capsule);
		_sim_capsule = nullptr;
		_sim_config = nullptr;
		_sim_in = nullptr;
		_sim_out = nullptr;
		_sim_dims = nullptr;
	}
}

// ===================================================================
// Forward prediction for servo delay compensation (Smith predictor)
// ===================================================================
void MpcController::_predict_ahead(const double x_cur[MPC_NX], float t, float dt_pred,
				   double x_pred[MPC_NX])
{
	if (dt_pred < 1e-4f) {
		memcpy(x_pred, x_cur, MPC_NX * sizeof(double));
		return;
	}

	// If sim solver not available, fall back to simple fin-state adjustment
	if (!_sim_capsule) {
		memcpy(x_pred, x_cur, MPC_NX * sizeof(double));
		// Smith predictor approx: set fin states to last commanded position
		// (by the time our command arrives, servo will have caught up)
		x_pred[12] = (double)_last_delta_e;
		x_pred[13] = (double)_last_delta_r;
		x_pred[14] = (double)_last_delta_a;
		return;
	}

	// Number of sim steps = delay / dt_h (each sim step = one MPC horizon step)
	const double dt_h = (double)_cfg.tf / (double)_cfg.N_horizon;
	const int n_steps = (int)((double)dt_pred / dt_h + 0.5);

	if (n_steps < 1) {
		memcpy(x_pred, x_cur, MPC_NX * sizeof(double));
		return;
	}

	// Set initial state
	sim_in_set(_sim_config, _sim_dims, _sim_in, "x", (void *)x_cur);

	// Propagate forward n_steps using the actual acados model
	double x_step[MPC_NX];
	memcpy(x_step, x_cur, MPC_NX * sizeof(double));

	for (int i = 0; i < n_steps; i++) {
		float t_i = t + (float)(i * dt_h);

		// Interpolate fin commands at t_i from buffer
		double de_i = (double)_last_delta_e;
		double dr_i = (double)_last_delta_r;
		double da_i = (double)_last_delta_a;

		if (_cmd_buf_count > 0) {
			for (int j = _cmd_buf_count - 1; j >= 0; j--) {
				int idx = (_cmd_buf_head - j - 1 + CMD_BUF_SIZE) % CMD_BUF_SIZE;

				if (_cmd_buf[idx].t <= t_i) {
					de_i = (double)_cmd_buf[idx].de;
					dr_i = (double)_cmd_buf[idx].dr;
					da_i = (double)_cmd_buf[idx].da;
					break;
				}
			}
		}

		// In NX=15 model: u = [ddelta_e, ddelta_r, ddelta_a] is the RATE
		// of fin change (rad/s), NOT the fin position. Compute rate from
		// the desired position change over one horizon step dt_h.
		double u[MPC_NU];
		u[0] = (de_i - x_step[12]) / dt_h;
		u[1] = (dr_i - x_step[13]) / dt_h;
		u[2] = (da_i - x_step[14]) / dt_h;

		// Clamp rate to physical limit
		double rate_lim = (double)_cfg.delta_rate_limit_dps * M_PI / 180.0;

		for (int k = 0; k < MPC_NU; k++) {
			if (u[k] > rate_lim) { u[k] = rate_lim; }
			if (u[k] < -rate_lim) { u[k] = -rate_lim; }
		}

		sim_in_set(_sim_config, _sim_dims, _sim_in, "x", (void *)x_step);
		sim_in_set(_sim_config, _sim_dims, _sim_in, "u", (void *)u);

		// Update parameters (mass, thrust) for this time
		double p[MPC_NP];
		get_params(t_i, p);
		m130_rocket_acados_sim_update_params(_sim_capsule, p, MPC_NP);

		// Solve one step
		int status = m130_rocket_acados_sim_solve(_sim_capsule);

		if (status != 0) {
			// Sim failed — fall back to last good state with fin adjustment
			PX4_WARN("MPC Smith predictor sim step %d failed (%d)", i, status);
			break;
		}

		// Get output state
		sim_out_get(_sim_config, _sim_dims, _sim_out, "x", (void *)x_step);
	}

	memcpy(x_pred, x_step, MPC_NX * sizeof(double));
}

// ===================================================================
// Command buffer management (ring buffer)
// ===================================================================
void MpcController::_add_cmd_to_buffer(float t, float de, float dr, float da)
{
	int idx = (_cmd_buf_head + _cmd_buf_count) % CMD_BUF_SIZE;
	_cmd_buf[idx].t  = t;
	_cmd_buf[idx].de = de;
	_cmd_buf[idx].dr = dr;
	_cmd_buf[idx].da = da;

	if (_cmd_buf_count < CMD_BUF_SIZE) {
		_cmd_buf_count++;
	} else {
		// Buffer full: advance head (overwrite oldest)
		_cmd_buf_head = (_cmd_buf_head + 1) % CMD_BUF_SIZE;
	}
}

void MpcController::_prune_cmd_buffer(float t)
{
	// Remove entries older than t - 2.0s
	const float cutoff = t - 2.0f;

	while (_cmd_buf_count > 0) {
		int oldest = _cmd_buf_head;
		if (_cmd_buf[oldest].t > cutoff) { break; }
		_cmd_buf_head = (_cmd_buf_head + 1) % CMD_BUF_SIZE;
		_cmd_buf_count--;
	}
}
