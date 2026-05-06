/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef DEBUG_FLOAT_ARRAY_HPP
#define DEBUG_FLOAT_ARRAY_HPP

#include <uORB/topics/debug_array.h>
#include <uORB/topics/rocket_gnc_status.h>

class MavlinkStreamDebugFloatArray : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamDebugFloatArray(mavlink); }

	static constexpr const char *get_name_static() { return "DEBUG_FLOAT_ARRAY"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		unsigned size = 0;
		constexpr unsigned frame_size = MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;

		if (_debug_array_sub.advertised()) {
			size += frame_size;
		}

		if (_rocket_gnc_status_sub.advertised()) {
			// RktGNC is decimated in send(); expose average byte cost so stream
			// scheduler doesn't over-throttle SRV_FB.
			size += frame_size / _RKTGNC_DECIMATION;
		}

		return size;
	}

private:
	explicit MavlinkStreamDebugFloatArray(Mavlink *mavlink) : MavlinkStream(mavlink) {}
	static constexpr unsigned _RKTGNC_DECIMATION = 4; // stream at ~1/4 of DEBUG_FLOAT_ARRAY rate
	unsigned _rktgnc_decim_count{0};

	uORB::Subscription _debug_array_sub{ORB_ID(debug_array)};
	uORB::Subscription _rocket_gnc_status_sub{ORB_ID(rocket_gnc_status)};

	bool send() override
	{
		bool sent = false;

		// Forward debug_array (servo feedback id=1)
		debug_array_s debug;

		if (_debug_array_sub.update(&debug)) {
			mavlink_debug_float_array_t msg{};

			msg.time_usec = debug.timestamp;
			msg.array_id = debug.id;
			memcpy(msg.name, debug.name, sizeof(msg.name));
			msg.name[sizeof(msg.name) - 1] = '\0'; // enforce null termination

			for (size_t i = 0; i < debug_array_s::ARRAY_SIZE; i++) {
				msg.data[i] = debug.data[i];
			}

			mavlink_msg_debug_float_array_send_struct(_mavlink->get_channel(), &msg);
			sent = true;
		}

		// Pack rocket_gnc_status into DEBUG_FLOAT_ARRAY id=2
		rocket_gnc_status_s gnc;

		if (_rocket_gnc_status_sub.update(&gnc)) {
			_rktgnc_decim_count = (_rktgnc_decim_count + 1) % _RKTGNC_DECIMATION;

			if (_rktgnc_decim_count == 0) {
				mavlink_debug_float_array_t msg{};

				msg.time_usec = gnc.timestamp;
				msg.array_id = 2;
				strncpy(msg.name, "RktGNC", sizeof(msg.name));
				msg.name[sizeof(msg.name) - 1] = '\0';

				// Pack GNC fields into data array
				msg.data[0]  = (float)gnc.stage;
				msg.data[1]  = gnc.t_flight;
				msg.data[2]  = gnc.q_dyn;
				msg.data[3]  = gnc.q_kgf;
				msg.data[4]  = gnc.pitch_accel_cmd;
				msg.data[5]  = gnc.yaw_accel_cmd;
				msg.data[6]  = gnc.blend_alpha;
				msg.data[7]  = gnc.delta_roll;
				msg.data[8]  = gnc.delta_pitch;
				msg.data[9]  = gnc.delta_yaw;
				msg.data[10] = gnc.fin1;
				msg.data[11] = gnc.fin2;
				msg.data[12] = gnc.fin3;
				msg.data[13] = gnc.fin4;
				msg.data[14] = gnc.altitude;
				msg.data[15] = gnc.airspeed;
				msg.data[16] = gnc.rho;
				msg.data[17] = (float)gnc.servo_online_mask;

				// Solver / MHE diagnostics (renamed from repurposed legacy fields)
				msg.data[18] = gnc.phi;
				msg.data[19] = gnc.mpc_solve_ms;
				msg.data[20] = gnc.mhe_quality;
				msg.data[21] = (float)gnc.mpc_solve_count;

				// New fields - attitude angles
				msg.data[22] = gnc.theta;
				msg.data[23] = gnc.psi;
				msg.data[24] = gnc.alpha_est;
				msg.data[25] = gnc.gamma_rad;

				// New fields - guidance frame position
				msg.data[26] = gnc.pos_downrange;
				msg.data[27] = gnc.pos_crossrange;
				msg.data[28] = gnc.vel_downrange;
				msg.data[29] = gnc.vel_down;
				msg.data[30] = gnc.vel_crossrange;

				// Navigation
				msg.data[31] = gnc.bearing_deg;
				msg.data[32] = gnc.target_range_remaining;

				// Event flags
				msg.data[33] = gnc.launched ? 1.0f : 0.0f;

				// dt measurement
				msg.data[34] = gnc.dt_actual;
				msg.data[35] = gnc.dt_min;
				msg.data[36] = gnc.dt_max;

				// Solver diagnostics (previously dropped — see RocketGncStatus.msg)
				msg.data[37] = gnc.mhe_solve_ms;
				msg.data[38] = (float)gnc.mpc_solver_status;
				msg.data[39] = (float)gnc.mpc_sqp_iter;
				msg.data[40] = (float)gnc.mhe_valid;
				msg.data[41] = (float)gnc.mhe_status;

				// Cross-validation (EKF vs MHE)
				msg.data[42] = gnc.xval_gamma_err;
				msg.data[43] = gnc.xval_chi_err;
				msg.data[44] = gnc.xval_alt_err;
				msg.data[45] = gnc.xval_penalty;

				// Cycle timing (µs) — replaces the old DEBUG_VECT("TIMING")
				// channel. Ground tooling should read timing from here.
				msg.data[46] = (float)gnc.mhe_solve_us;
				msg.data[47] = (float)gnc.mpc_solve_us;
				msg.data[48] = (float)gnc.cycle_us;

				// Cumulative event counters (per-flight). Cast to float is
				// lossless up to 2^24 ≈ 16M events; well beyond any realistic
				// per-flight count for these states.
				msg.data[49] = (float)gnc.mpc_fail_count;
				msg.data[50] = (float)gnc.mpc_nan_skip_count;
				msg.data[51] = (float)gnc.mhe_fail_count;
				msg.data[52] = (float)gnc.fin_clamp_count;
				msg.data[53] = (float)gnc.xval_reset_count;
				msg.data[54] = (float)gnc.servo_offline_events;

				// GPS quality snapshot (raw feed consumed by MHE)
				msg.data[55] = (float)gnc.gps_fix_type;
				msg.data[56] = (float)gnc.gps_satellites_used;
				msg.data[57] = (float)gnc.gps_jamming_state;

				mavlink_msg_debug_float_array_send_struct(_mavlink->get_channel(), &msg);
				sent = true;
			}
		}

		return sent;
	}
};

#endif // DEBUG_FLOAT_ARRAY_HPP
