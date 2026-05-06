/**
 * @file xqpower_can_params.c
 * @brief XQPOWER CAN Servo Parameters
 */

/**
 * XQPOWER CAN Enable
 *
 * 0 = Disabled, 1 = Enabled
 *
 * @min 0
 * @max 1
 * @value 0 Disabled
 * @value 1 Enabled
 * @reboot_required true
 * @group XQPOWER CAN
 */
PARAM_DEFINE_INT32(XQCAN_ENABLE, 1);

/**
 * Servo 1 CAN Node ID
 *
 * @min 1
 * @max 127
 * @group XQPOWER CAN
 */
PARAM_DEFINE_INT32(XQCAN_NODE1, 1);

/**
 * Servo 2 CAN Node ID
 *
 * @min 1
 * @max 127
 * @group XQPOWER CAN
 */
PARAM_DEFINE_INT32(XQCAN_NODE2, 2);

/**
 * Servo 3 CAN Node ID
 *
 * @min 1
 * @max 127
 * @group XQPOWER CAN
 */
PARAM_DEFINE_INT32(XQCAN_NODE3, 3);

/**
 * Servo 4 CAN Node ID
 *
 * @min 1
 * @max 127
 * @group XQPOWER CAN
 */
PARAM_DEFINE_INT32(XQCAN_NODE4, 4);

/**
 * Servo angle limit (degrees)
 *
 * Maximum deflection angle in degrees. Servos will be clamped to ±limit.
 * XQPOWER supports ±100 deg.
 *
 * Must match the acados solver's delta_max (=20°) so that normalized
 * actuator_servos commands (fin / delta_max) scale back to the
 * intended physical angle. See m130_ocp_setup.py::delta_max.
 *
 * @min 1.0
 * @max 100.0
 * @unit deg
 * @decimal 1
 * @group XQPOWER CAN
 */
PARAM_DEFINE_FLOAT(XQCAN_LIMIT, 20.0f);


/**
 * Servo PDO auto-report interval (ms)
 *
 * How often each XQPOWER servo auto-reports its measured position over
 * CAN (PDO). Lower values give tighter timing data and tighter
 * closed-loop HIL injection, at the cost of more CAN bus load and more
 * USB-CH340 / PX4 parsing traffic.
 *
 * Guidance:
 *   50 ms (historical): proven value for open-loop. 4 servos × 20 Hz ≈ 2 %
 *                    of a 500 kbps CAN bus.
 *   10 ms:           fastest supported by the driver. 4 servos × 100 Hz
 *                    ≈ 11 % bus load. Recommended for closed-loop HIL,
 *                    and for real flight only after bench-validating
 *                    that the bus and USB-CH340 adapter keep up with
 *                    no dropped frames.
 *
 * The driver clamps values below 10 ms up to 10 ms. Applied during the
 * servo init sequence; change XQCAN_FB_MS then restart the driver
 * (`xqpower_can stop && xqpower_can start`) or reboot PX4 for the new
 * interval to reach the servos.
 *
 * @min 10
 * @max 255
 * @unit ms
 * @reboot_required true
 * @group XQPOWER CAN
 */
PARAM_DEFINE_INT32(XQCAN_FB_MS, 10);
