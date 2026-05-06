#pragma once

#include <cstdint>
#include <cmath>

// acados C interface — undefine PX4's UNUSED to avoid conflict with acados/utils/types.h
#ifdef UNUSED
#undef UNUSED
#endif
extern "C" {
#include "acados_solver_m130_rocket.h"
#include "acados_sim_solver_m130_rocket.h"
#include "acados_c/ocp_nlp_interface.h"
}

static constexpr int MPC_NX = M130_ROCKET_NX;   // 15 (ideal fins, no servo dynamics)
static constexpr int MPC_NU = M130_ROCKET_NU;    // 3
static constexpr int MPC_NP = M130_ROCKET_NP;    // 2
static constexpr int MPC_N  = M130_ROCKET_N;     // 80 (N=80, tf=1.6s, dt=20ms)
static constexpr int MPC_NY = M130_ROCKET_NY;    // 12
static constexpr int MPC_NYN = M130_ROCKET_NYN;  // 9

static constexpr float G_ACCEL = 9.80665f;
static constexpr float H_SCALE = 100.0f;
static constexpr float X_SCALE = 1000.0f;
static constexpr float Y_SCALE = 1000.0f;

struct MpcConfig {
	// Horizon — MUST match solver (c_generated_code M130_ROCKET_N)
	int   N_horizon       = MPC_N;   // uses solver's actual N (now 80)
	float tf              = 1.6f;   // matches solver tf (N=80 * dt=20ms = 1.6s)
	float t_ctrl          = 0.3f;

	// Target
	float target_x  = 2600.0f;
	float target_h  = 0.0f;      // target_alt - launch_alt

	// Mass/propulsion
	float mass_full      = 0.0f;
	float mass_dry       = 0.0f;
	float burn_time      = 0.0f;
	float thrust_plateau = 0.0f;
	float t_tail         = 1.0f;

	// Inertias
	float Ixx_full = 0.0f, Ixx_dry = 0.0f;
	float Iyy_full = 0.0f, Iyy_dry = 0.0f;
	float Izz_full = 0.0f, Izz_dry = 0.0f;

	// Servo delay compensation (not in model — handled by forward prediction)
	float servo_delay_s = 0.0f;   // pure transport delay (s). 0 = disabled.
	float delta_rate_limit_dps = 250.0f;  // deg/s - fin rate limit for Smith predictor

	// Guidance
	float impact_angle_deg     = -30.0f;
	float impact_blend_start   = 0.90f;
	float impact_blend_end     = 0.99f;
	float cruise_progress      = 0.65f;
	float gamma_natural_rad    = 0.0f;

	// MHE blend
	float quality_gate_thr = 0.3f;

	// ── Dead-time compensation: lookahead_stage pulls fin commands from
	// a future MPC stage matching the servo transport delay.
	// Auto-computed from servo_delay_s: round(servo_delay_s / dt_h).
	// servo_delay_s is set to 0 (disabled) — lookahead_stage handles delay.
	int   lookahead_stage = 1;
};

struct MpcSolveResult {
	// NOTE: the solver returns the three virtual-channel commands (delta_e,
	// delta_r, delta_a). The X-fin mixing + per-fin saturation clamp lives
	// in RocketMPC.cpp so that the MHE back-solve sees the deflections the
	// actuators actually executed, not the un-clamped mathematical mix.
	float delta_e;         // elevator (rad)
	float delta_r;         // rudder (rad)
	float delta_a;         // aileron (rad)
	int   status;
	float solve_time_ms;
	int   sqp_iter;
	bool  valid;
};

class MpcController {
public:
	MpcController() = default;
	~MpcController();

	bool init(const MpcConfig &cfg);
	void destroy();

	MpcSolveResult solve(const double x_mpc[MPC_NX],
			     float gamma_ref, float chi_ref, float phi_ref,
			     float h_ref_scaled, bool cruise_alt_set,
			     float t_flight, float dt,
			     float x_pos, float altitude);

	const MpcConfig &config() const { return _cfg; }
	MpcConfig &mutable_config() { return _cfg; }
	bool ready() const { return _solver_ready; }
	void reset();

	void get_params(float t, double p[MPC_NP]) const;
	int  solve_count() const { return _solve_count; }
	float last_solve_time_ms() const { return _last_solve_time_ms; }

private:
	void _compute_weights(float t_flight, float gamma, float gamma_ref_prev,
			      float phi, float p_rate,
			      float alpha, float q_rate, float x_pos,
			      bool cruise_alt_set, float h_ref_scaled,
			      double W[MPC_NY], double W_e[MPC_NYN]);
	void _forward_guess(const double x0[MPC_NX], float t0);
	void _reinit(const double x_mpc[MPC_NX]);

	// Forward prediction for servo delay compensation
	void _predict_ahead(const double x_cur[MPC_NX], float t, float dt_pred,
			    double x_pred[MPC_NX]);
	void _add_cmd_to_buffer(float t, float de, float dr, float da);
	void _prune_cmd_buffer(float t);

	// Smith predictor: acados sim solver for model-consistent forward prediction
	bool _init_sim_solver();
	void _destroy_sim_solver();
	m130_rocket_sim_solver_capsule *_sim_capsule{nullptr};
	sim_config  *_sim_config{nullptr};
	sim_in      *_sim_in{nullptr};
	sim_out     *_sim_out{nullptr};
	void        *_sim_dims{nullptr};

	MpcConfig _cfg;
	m130_rocket_solver_capsule *_capsule{nullptr};
	ocp_nlp_config *_nlp_config{nullptr};
	ocp_nlp_dims   *_nlp_dims{nullptr};
	ocp_nlp_in     *_nlp_in{nullptr};
	ocp_nlp_out    *_nlp_out{nullptr};
	ocp_nlp_solver *_nlp_solver{nullptr};

	bool  _solver_ready{false};
	bool  _warm{false};
	int   _consec_fails{0};
	int   _consec_ok{0};
	int   _solve_count{0};

	float _last_delta_e{0.0f};
	float _last_delta_r{0.0f};
	float _last_delta_a{0.0f};
	float _last_solve_t{-1.0f};
	float _last_solve_time_ms{0.0f};

	// Command buffer for forward prediction (ring buffer)
	static constexpr int CMD_BUF_SIZE = 128;  // ~2.5s at 50Hz
	struct CmdEntry { float t; float de; float dr; float da; };
	CmdEntry _cmd_buf[CMD_BUF_SIZE]{};
	int _cmd_buf_head{0};
	int _cmd_buf_count{0};

	// Temporary trajectory buffer for forward guess
	double _x_traj[(MPC_N + 1) * MPC_NX]{};
};
