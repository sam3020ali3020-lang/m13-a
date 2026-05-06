/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

/**
 * @file rocket_mpc_params.c
 * Parameters for Rocket M130 MPC control module (EKF2-only, MHE removed).
 *
 * Mass/propulsion/inertia values must match the acados OCP model
 * used for code generation.
 */

#include <px4_platform_common/px4_config.h>
#include <parameters/param.h>

/* ===================================================================
 *  Timing
 * =================================================================== */

/**
 * Control activation delay after launch
 *
 * Time after launch detection before MPC starts commanding fins.
 * Should cover the launcher rail departure time.
 *
 * @unit s
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_T_CTRL, 0.3f);

/* ===================================================================
 *  Target
 * =================================================================== */

/**
 * Target downrange distance
 *
 * Distance along the bearing captured at arming.
 *
 * @unit m
 * @min 100.0
 * @max 300000.0
 * @decimal 0
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_XTRGT, 2600.0f);

/**
 * Target altitude AGL
 *
 * Target altitude above launch site (target_alt - launch_alt).
 * 0 = same elevation as launch site.
 *
 * @unit m
 * @min -5000.0
 * @max 50000.0
 * @decimal 0
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_HTRGT, 0.0f);

/**
 * Terminal impact angle
 *
 * Desired flight-path angle at impact. Negative = diving.
 *
 * @unit deg
 * @min -90.0
 * @max 0.0
 * @decimal 1
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_IMP_ANG, -30.0f);

/**
 * Cruise→dive transition progress
 *
 * Fraction of downrange-to-target at which the LOS guidance starts
 * blending from level cruise flight (γ=0) to the impact-angle dive
 * trajectory. The dive is fully active at progress = min(p+0.10, 0.95),
 * with a smooth Hermite blend in between.
 *
 * Lower values start the dive earlier (good for steep impact angles
 * and longer-range targets); higher values keep the vehicle in cruise
 * longer (good for shallow impact angles or shorter ranges).
 *
 * Must match autopilot.mpc.cruise_progress in the Python simulation
 * config to keep sim-vs-flight guidance timing consistent.
 *
 * @min 0.3
 * @max 0.95
 * @decimal 2
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_CRUISE_P, 0.65f);

/* ===================================================================
 *  Fin limits
 *
 *  NOTE: The maximum fin deflection is NOT a parameter — it is baked
 *  into the acados-generated solver at 0.3491 rad (20°). See
 *  m130_ocp_setup.py::delta_max and SOLVER_DELTA_MAX_RAD in RocketMPC.cpp.
 *  To change the limit, modify the Python OCP and regenerate the solver.
 * =================================================================== */

/* ===================================================================
 *  Mass / propulsion  (must match acados OCP model)
 * =================================================================== */

/**
 * Full (wet) mass at ignition
 *
 * @unit kg
 * @min 0.5
 * @max 10000.0
 * @decimal 3
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_MASS_F, 12.74f);

/**
 * Dry mass (burnout)
 *
 * @unit kg
 * @min 0.5
 * @max 10000.0
 * @decimal 3
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_MASS_D, 11.11f);

/**
 * Motor burn time
 *
 * @unit s
 * @min 0.1
 * @max 600.0
 * @decimal 3
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_TBURN, 4.772f);

/**
 * Total impulse
 *
 * @min 1.0
 * @max 10000000.0
 * @decimal 1
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_IMPULS, 3593.2f);

/**
 * Thrust plateau (manual override, advanced)
 *
 * Steady-state thrust during the propulsive phase.
 *
 * Leave at 0 (default) to auto-derive from impulse, burn time and tail-off:
 *     T_plateau = ROCKET_IMPULS / (ROCKET_TBURN - 0.75 * ROCKET_T_TAIL)
 * This keeps the plateau in lock-step with the propulsion parameter set and
 * matches the Python simulation reference.
 *
 * Any positive value is treated as an advanced override and bypasses the
 * derivation — use only when characterising a motor whose profile is not
 * captured by the impulse/burn-time/tail-off triplet. Mismatching the
 * override with the rest of the propulsion params will silently skew MPC
 * gamma tracking during boost.
 *
 * @unit N
 * @min 0.0
 * @max 5000000.0
 * @decimal 1
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_THRUST, 0.0f);

/**
 * Tail-off duration
 *
 * Time for thrust to ramp from plateau to zero.
 *
 * @unit s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_T_TAIL, 1.0f);

/* ===================================================================
 *  Inertias  (must match acados OCP model)
 * =================================================================== */

/**
 * Roll inertia at full mass (Ixx)
 *
 * @unit kg m^2
 * @min 0.001
 * @max 100000.0
 * @decimal 4
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_IXX_F, 0.0389f);

/**
 * Roll inertia at dry mass (Ixx)
 *
 * @unit kg m^2
 * @min 0.001
 * @max 100000.0
 * @decimal 4
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_IXX_D, 0.0356f);

/**
 * Pitch inertia at full mass (Iyy)
 *
 * @unit kg m^2
 * @min 0.001
 * @max 100000.0
 * @decimal 4
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_IYY_F, 1.1651f);

/**
 * Pitch inertia at dry mass (Iyy)
 *
 * @unit kg m^2
 * @min 0.001
 * @max 100000.0
 * @decimal 4
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_IYY_D, 1.0789f);

/**
 * Yaw inertia at full mass (Izz)
 *
 * @unit kg m^2
 * @min 0.001
 * @max 100000.0
 * @decimal 4
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_IZZ_F, 1.166f);

/**
 * Yaw inertia at dry mass (Izz)
 *
 * @unit kg m^2
 * @min 0.001
 * @max 100000.0
 * @decimal 4
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_IZZ_D, 1.0779f);

/* ===================================================================
 *  Servo delay compensation
 *
 *  The MPC model (NX=15) has ideal fins — no servo dynamics.
 *  The pure transport delay is compensated by pulling fin commands
 *  from a future MPC stage (lookahead_stage) matching the delay.
 *  Formula: lookahead_stage = round(servo_delay_s / dt_h)
 *  where dt_h = tf/N = 1.6/80 = 0.02s.
 *  Example: servo_delay_s=0.080 → lookahead_stage=4.
 *  Only this single parameter needs calibration for any servo type;
 *  no solver regeneration required.
 * =================================================================== */

/**
 * Servo pure transport delay
 *
 * Time from fin command output to servo execution.
 * Used to auto-compute lookahead_stage for MPC dead-time compensation.
 * 0 = no compensation (lookahead_stage=1).
 *
 * ════════════════════════════════════════════════════════════════════
 *  RULE  (read this before changing the value)
 * ════════════════════════════════════════════════════════════════════
 *
 *   RKT_MPC_SVO_DLY = max(pure_delay + tau_servo + MPC_margin, 100ms)
 *
 *   ┌───────────────────────────────────────────────────────────────┐
 *   │ TERM        MEANING                                           │
 *   ├───────────────────────────────────────────────────────────────┤
 *   │ pure_delay  Pure servo transport delay from /direct test:     │
 *   │             PC ↔ USB-CAN ↔ servo, aggregate fit on repeated   │
 *   │             step responses.  This captures ONLY the           │
 *   │             servo+CAN+wire transport delay.                   │
 *   │                                                               │
 *   │ tau_servo   Servo first-order time constant (rise time / 3).  │
 *   │             From /direct aggregate fit. The MPC's NX=15       │
 *   │             ideal-fin model has NO servo dynamics — τ acts    │
 *   │             as ADDITIONAL effective dead time to the plant.   │
 *   │             *** WITHOUT τ in compensation, the closed-loop    │
 *   │                 exhibits a 5 Hz LIMIT CYCLE in pitch rate.    │
 *   │                 (Verified in 6DOF sim 2026-05-02: ignoring τ  │
 *   │                  → la=2 → 5Hz amp 130; including τ → la=3    │
 *   │                  → range error 14m→0m, σ_pitch 4.93°→3.87°.) │
 *   │                                                               │
 *   │ +71ms       Safety margin for delays NOT captured by /direct  │
 *   │             but present in flight:                            │
 *   │               • MPC+MHE solve on ARM64 (avg 18, P95 57 ms)    │
 *   │               • PX4 scheduling jitter (1–5 ms)                │
 *   │               • Sensor pipeline gyro→EKF2→MPC (2–5 ms)        │
 *   │               • CAN bus contention with 4 actuators (1–3 ms) │
 *   │               • Future-planning safety (5–10 ms)              │
 *   │                                                               │
 *   │ 100ms       Absolute floor for MPC dead-time stability.       │
 *   │             Below ~5×dt_h, lookahead_stage<5 and MPC can      │
 *   │             oscillate near the dead-time boundary.            │
 *   └───────────────────────────────────────────────────────────────┘
 *
 *   How to set when changing servo:
 *     1. Run /direct repeatability test → aggregate fit
 *        Extract BOTH:  pure_delay (transport)  AND  tau_servo (τ).
 *     2. candidate = pure_delay + tau_servo + 40   (in ms)
 *     3. if candidate < 100  →  write 0.100f  (floor)
 *        else                →  write (candidate / 1000.0f)
 *     4. ALSO update the duplicate at:
 *        AndroidApp/app/src/main/cpp/px4_jni.cpp  (Real & HITL blocks)
 *
 * ════════════════════════════════════════════════════════════════════
 *  CURRENT MEASUREMENT
 * ════════════════════════════════════════════════════════════════════
 *
 *   Servo: XQPOWER XQ-BLS8145C (CAN, XQPOWER protocol, 4-axis fin set)
 *
 *   /direct test (2026-05-02, repeatability pattern, 2 runs of 100
 *   transitions/servo × 4 servos = 800 transitions total):
 *     pure_delay = 39.31 ms     (mean of 2 runs, aggregate fit R²=0.95)
 *     tau_servo  = 29.88 ms     (mean of 2 runs, aggregate fit R²=0.95)
 *
 *   Apply rule:
 *     candidate     = 39.31 + 29.88 + 71 = 140.19 ms
 *     140.19 ≥ 100  →  use computed value
 *     value written = 0.140 s
 *     lookahead_stage = roundf(0.140 / 0.020) = 7
 *
 *   PIL timing (ARM64, MPC+MHE, lockstep, 2026-05-03):
 *     MPC cycle: avg=18ms  P95=57ms  P99=61ms  max=79ms
 *     Total delay budget: servo(69ms) + MPC_P95(57ms) = 126ms < 140ms ✅
 *
 *   Previous servo (KST X20-7.4 via CAN): measured ~110 ms (pure delay only)
 *     If τ ≈ 30 ms: candidate = 110 + 30 + 40 = 180 ms  →  write 0.180 s
 *
 * @unit s
 * @min 0.0
 * @max 0.5
 * @decimal 3
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(RKT_MPC_SVO_DLY, 0.100f);

/* ===================================================================
 *  Launch site
 *
 *  Launch-site altitude and rail elevation are NOT parameters: they are
 *  captured directly from the sensor suite at arming / pre-launch:
 *    - altitude ASL  -> GPS 3D fix (real flight) / lpos.ref_alt (HITL),
 *                       with baro fallback at launch detection
 *    - rail pitch    -> attitude quaternion at arm + every pre-launch
 *                       cycle; see RocketMPC::_pitch_from_quat().
 *  Any "set once in params" design was a porting leftover from the
 *  simulator config and was the root cause of two silent-failure bugs
 *  (1150 m launch_alt mismatch saturating MHE's h state; wrong rail
 *  angle skewing LOS gamma_natural feedforward).
 * =================================================================== */

/* ===================================================================
 *  MPC solver tuning
 * =================================================================== */

/**
 * MPC prediction horizon time
 *
 * @unit s
 * @min 1.0
 * @max 20.0
 * @decimal 1
 * @group Rocket MPC
 */
PARAM_DEFINE_FLOAT(ROCKET_MPC_TF, 1.6f);

/* ===================================================================
 *  Test mode
 * =================================================================== */

/**
 * Fin mixing self-test
 *
 * When set to 1, rocket_mpc enters a mixing-test mode that commands
 * known pitch/yaw/roll fin patterns through xqpower_can and verifies
 * servo feedback matches the expected X-fin geometry.
 *
 * Sequence (~12 s): ZERO → PITCH → ZERO → YAW → ZERO → ROLL → ZERO.
 * Each command phase is 2 s; each zero phase is 1 s.
 * Test amplitude: ±5° (safe without arming).
 *
 * Safety: blocked if vehicle is armed.  Auto-resets to 0 on completion.
 * Result published via mavlink_log (visible in QGC / Android app).
 *
 * 0 = disabled (normal operation).
 * 1 = run mixing test on next Run() cycle.
 *
 * @min 0
 * @max 1
 * @group Rocket MPC
 */
PARAM_DEFINE_INT32(RKT_MIX_TEST, 0);

/* ===================================================================
 *  SITL workarounds
 * =================================================================== */

/**
 * SITL/HITL actuator routing
 *
 * Routes actuator commands to actuator_outputs_sim (lockstep) instead
 * of actuator_servos. ONLY enable in simulation — never on real hardware.
 *
 * 0 = disabled (real hardware default).
 * 1 = enabled (SITL only).
 *
 * @min 0
 * @max 1
 * @group Rocket MPC
 */
PARAM_DEFINE_INT32(ROCKET_SITL_GPS, 0);

/**
 * Use groundtruth topics in HITL mode
 *
 * When SYS_HITL=1 and this param=1, rocket_mpc subscribes to
 * vehicle_attitude_groundtruth and vehicle_local_position_groundtruth
 * (perfect state from the simulation bridge). This isolates MPC tuning
 * from EKF2 estimation errors.
 *
 * Set to 0 to use EKF2 output (vehicle_attitude / vehicle_local_position)
 * even in HITL mode — this tests the full sensor→EKF2→MPC flight path
 * that will be active during real flight (SYS_HITL=0).
 *
 * Has no effect when SYS_HITL=0 (real flight always uses EKF2).
 *
 * @min 0
 * @max 1
 * @group Rocket MPC
 */
PARAM_DEFINE_INT32(ROCKET_USE_GT, 1);

// NOTE: ROCKET_MPC_LA (lookahead stage) is now auto-computed from
// RKT_MPC_SVO_DLY (servo_delay_s). No separate param needed.
// Formula: lookahead_stage = round(servo_delay_s / dt_h).
