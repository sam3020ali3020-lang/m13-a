/**
 * @file XqpowerCan.hpp
 * @brief XQPOWER CAN Servo Driver for PX4
 *
 * NuttX:   Direct STM32H7 FDCAN register access (MicoAir H743)
 * Android: SLCAN via USB-Serial (Waveshare USB_CAN_A / CH340)
 *
 * Protocol: Shenzhen XQPOWER Technology Co., LTD (can-sd-v022)
 * CAN Bus: 500 kbps, Standard Frame (11-bit ID)
 */

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/log.h>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/debug_array.h>

#include <drivers/drv_hrt.h>

#ifdef __PX4_POSIX
#include <linux/usbdevice_fs.h>
#include <sys/ioctl.h>
#include <atomic>
#else
#include "fdcan_regs.h"
#endif

#define XQPOWER_MAX_SERVOS       4
#define XQPOWER_SDO_TX_BASE      0x600   // Master -> Servo
#define XQPOWER_SDO_RX_BASE      0x580   // Servo -> Master
#define XQPOWER_PDO_RX_BASE      0x180   // Auto-report
#define XQPOWER_SDO_WRITE        0x22
#define XQPOWER_SDO_READ         0x40
#define XQPOWER_DEFAULT_RATE_HZ  100

#ifndef __PX4_POSIX
/* FDCAN Message RAM configuration */
#define XQCAN_RX_FIFO0_ELEMENTS  16
#define XQCAN_TX_FIFO_ELEMENTS   8
#endif

struct xqpower_feedback_t {
	float    position_deg;
	uint16_t current_mA;
	int16_t  temperature_01C;
	bool     online;
	hrt_abstime last_update_us;   /* uint64_t — must match hrt_absolute_time() */
};

class XqpowerCan : public ModuleBase<XqpowerCan>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	XqpowerCan();
	~XqpowerCan() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

	/* Manual override API — thread-safe access from nsh (custom_command)
	 * while Run() reads on the work-queue thread.
	 * Atomics prevent torn reads/writes and ensure memory visibility across
	 * CPU cores without the overhead of a mutex. */
	void set_override_angle(int idx, float angle_deg) {
		if (idx < 0 || idx >= XQPOWER_MAX_SERVOS) { return; }
		_override_angles[idx].store(angle_deg, std::memory_order_relaxed);
	}
	void enable_override()  { _override_active.store(true,  std::memory_order_release); }
	void disable_override() { _override_active.store(false, std::memory_order_release); }

#ifdef __PX4_POSIX
	/** Set USB file descriptor from JNI (Android) */
	static void set_usb_fd(int fd);
#endif

private:
	void Run() override;
	void _servo_init_sequence();

	/* Override state (private, atomic) */
	std::atomic<bool>  _override_active{false};
	std::atomic<float> _override_angles[XQPOWER_MAX_SERVOS]{};

#ifdef __PX4_POSIX
	/* USB-Serial via direct ioctl (CH340 / Waveshare USB_CAN_A) */
	int _usb_fd{-1};
	int _bulk_ep_out{0x02};   /* bulk OUT endpoint */
	int _bulk_ep_in{0x82};    /* bulk IN endpoint  */
	bool _is_canlin_tool{false}; /* true = CAN_LIN_Tool V6.0 (CDC), false = Waveshare CH340 */

	/* SLCAN RX buffer */
	char _slcan_rxbuf[512];
	int  _slcan_rxpos{0};

	/* USB-Serial helpers */
	bool usb_serial_open(int fd);
	void usb_serial_close();
	bool ch340_set_baudrate(uint32_t baud);
	int  usb_control_msg(uint8_t reqtype, uint8_t req, uint16_t val, uint16_t idx,
			     void *data, uint16_t len, uint32_t timeout_ms);
	int  slcan_write(const char *data, int len);
	int  slcan_read(char *buf, int maxlen);

	/* Global USB FD (set from JNI) */
	static std::atomic<int> s_usb_fd;
	static std::atomic<int> s_consumed_fd;
#else
	/* FDCAN Hardware Access (direct register) */
	volatile FDCAN_Regs_t *_fdcan{nullptr};
	uint32_t _rx_fifo0_base{0};
	uint32_t _tx_fifo_base{0};
#endif

	bool     _can_initialized{false};

	bool can_open();
	void can_close();
	int  can_send(uint16_t id, const uint8_t *data, uint8_t dlc);
	int  can_receive();

	/* XQPOWER Protocol */
	void servo_set_position(int index, float angle_deg);
	void servo_set_report_interval(int index, uint8_t interval_ms);
	void servo_start_report(int index);
	void servo_stop_report(int index);
	void servo_read_position(int index);
	void servo_read_status(int index);
	void servo_process_rx(uint16_t id, const uint8_t *data, uint8_t dlc);

	/* Servo State */
	uint8_t            _node_ids[XQPOWER_MAX_SERVOS]{0x01, 0x02, 0x03, 0x04};
	xqpower_feedback_t _feedback[XQPOWER_MAX_SERVOS]{};
	bool               _report_started{false};
	float              _angle_limit{20.0f};

	/* PX4 Subscriptions */
	uORB::Subscription _actuator_servos_sub{ORB_ID(actuator_servos)};
	uORB::Subscription _actuator_outputs_sim_sub{ORB_ID(actuator_outputs_sim)};
	uORB::Subscription _actuator_outputs_sub{ORB_ID(actuator_outputs)};
	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};

	/* Servo feedback publication */
	uORB::Publication<debug_array_s> _servo_fb_pub{ORB_ID(debug_array)};
	float    _cmd_deg[XQPOWER_MAX_SERVOS]{};
	uint32_t _tx_fail_count{0};
	uint32_t _sdo_rx_count{0};
	uint32_t _pdo_rx_count{0};

	bool _armed{false};
	bool _sim_mode{false};   /* true  = HITL/SITL — listen to actuator_outputs_sim (radians)
				    false = real flight — IGNORE actuator_outputs_sim, use
					    actuator_servos only (normalized)
				    Set once at init() from SYS_HITL param. */

	/* Run() cycle counters */
	int _fb_cycle{0};
	int _pos_servo_idx{0};
	int _status_servo_idx{0};
	int _status_cycle_count{0};
	int _disarm_zero_count{0};
	int _tx_rate_div{0};
	int _rx_diag_count{0};
	int _total_rx{0};
	int _read_zero_count{0};   /* slcan_read returned 0 (timeout) */
	int _read_data_count{0};   /* slcan_read returned >0 (got data) */
	int _read_err_count{0};    /* slcan_read returned -1 (error) */

	/* Parameters */
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::XQCAN_ENABLE>)  _param_enable,
		(ParamInt<px4::params::XQCAN_NODE1>)   _param_node1,
		(ParamInt<px4::params::XQCAN_NODE2>)   _param_node2,
		(ParamInt<px4::params::XQCAN_NODE3>)   _param_node3,
		(ParamInt<px4::params::XQCAN_NODE4>)   _param_node4,
		(ParamFloat<px4::params::XQCAN_LIMIT>) _param_angle_limit,
		(ParamInt<px4::params::XQCAN_FB_MS>)   _param_fb_interval_ms
	)
};
