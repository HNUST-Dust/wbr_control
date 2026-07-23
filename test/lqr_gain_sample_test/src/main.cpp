/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>

#include <hpm_l1c_drv.h>

#include <modules/chassis/leg_kinematics.h>
#include <modules/chassis/leg_vmc.h>
#include <protocols/imu/hi91_protocol.h>
#include <protocols/motors/dji_motor_protocol.h>
#include <protocols/motors/dm_motor_protocol.h>

namespace {

constexpr bool kEnableDmMitMode = true;
constexpr bool kEnableDmTorqueOutput = false;
constexpr bool kPollDmFeedbackWithZeroTorque = true;
constexpr bool kEnableWheelOutput = false;
constexpr bool kEnableWheelPerturbation = false;
constexpr bool kEnableLegAnglePerturbation = false;

constexpr int kMinLegLengthMm = 160;
constexpr int kMaxLegLengthMm = 340;
constexpr int kStepLegLengthMm = 20;
constexpr uint32_t kControlPeriodMs = 1U;
constexpr uint32_t kMitEnterRepeatMs = 1000U;
constexpr uint32_t kSettleMs = 2500U;
constexpr uint32_t kSampleMs = 5000U;
constexpr uint32_t kPrintEveryMs = 5U;
constexpr double kLengthHoldKp = 250.0;
constexpr double kLengthHoldKd = 8.0;
constexpr double kJointTorqueLimitNm = 6.0;
constexpr double kWheelTorqueLimitNm = 1.0;
constexpr double kLegAnglePerturbNm = 1.0;
constexpr double kWheelPerturbNm = 0.6;
constexpr double kGravity = 9.80665;
constexpr double kDegToRad = 0.017453292519943295769;
constexpr double kDpsToRadPerSec = 0.017453292519943295769;
constexpr double kDjiRpmToRadPerSec = 0.10471975511965977;
constexpr double kDjiCurrentCommandPerNm = 1200.0;
constexpr double kWheelRadiusM = 0.05;
constexpr uint32_t kHi91Baudrate = 921600U;
constexpr int32_t kRxIdleTimeoutUs = 1000;
constexpr size_t kRxBufferSize = 256U;
constexpr size_t kRxBufferCount = 2U;
constexpr size_t kRxRingSize = 4096U;

constexpr uint8_t kLeftLegBus = 0U;
constexpr uint8_t kRightLegBus = 1U;
constexpr uint16_t kLeftJointBCanId = 0x00U;
constexpr uint16_t kLeftJointBMasterId = 0x10U;
constexpr uint16_t kLeftJointDCanId = 0x03U;
constexpr uint16_t kLeftJointDMasterId = 0x13U;
constexpr uint16_t kRightJointBCanId = 0x01U;
constexpr uint16_t kRightJointBMasterId = 0x11U;
constexpr uint16_t kRightJointDCanId = 0x02U;
constexpr uint16_t kRightJointDMasterId = 0x12U;
constexpr uint16_t kLeftWheelCanId = 0x201U;
constexpr uint16_t kRightWheelCanId = 0x201U;
constexpr uint16_t kDjiCurrentCommandCanId = 0x200U;
constexpr int kLeftLegKinematicBranch = 1;
constexpr int kRightLegKinematicBranch = -1;
constexpr double kLeftLegAngleOffset = -0.036063;
constexpr double kRightLegAngleOffset = -3.121010;
constexpr double kTwoPi = 6.28318530717958647692;

constexpr protocols::DmMitRange kDmJointMitRange = {
	.p_min = -12.56637f,
	.p_max = 12.56637f,
	.v_min = -45.0f,
	.v_max = 45.0f,
	.kp_min = 0.0f,
	.kp_max = 500.0f,
	.kd_min = 0.0f,
	.kd_max = 5.0f,
	.t_min = -54.0f,
	.t_max = 54.0f,
};

struct JointFeedback {
	protocols::DmMotorFeedbackNormal feedback = {};
	uint32_t sequence = 0U;
};

struct WheelFeedback {
	protocols::DjiMotorFeedback feedback = {};
	uint32_t sequence = 0U;
};

struct BusContext {
	uint8_t bus = 0U;
};

struct CanStats {
	uint32_t rx_count = 0U;
	uint32_t unknown_count = 0U;
	uint16_t last_unknown_id = 0U;
};

const struct device *g_can_devs[2] = {};
BusContext g_bus_context[2] = {{0U}, {1U}};
CanStats g_can_stats[2];
JointFeedback g_left_b;
JointFeedback g_left_d;
JointFeedback g_right_b;
JointFeedback g_right_d;
WheelFeedback g_left_wheel;
WheelFeedback g_right_wheel;

const struct device *const g_imu_uart = DEVICE_DT_GET(DT_NODELABEL(uart2));
K_SEM_DEFINE(g_rx_sem, 0, 1);
RING_BUF_DECLARE(g_rx_ring, kRxRingSize);
uint8_t g_rx_buffers[kRxBufferCount][kRxBufferSize];
uint8_t g_next_rx_buffer_index = 1U;
protocols::Hi91Parser g_parser;
protocols::Hi91Sample g_imu_sample = {};
bool g_imu_valid = false;

float UIntToFloat(uint16_t value, float min_value, float max_value, uint8_t bits)
{
	const float span = max_value - min_value;
	const float max_int = static_cast<float>((1U << bits) - 1U);
	return static_cast<float>(value) * span / max_int + min_value;
}

double DmPositionRad(const protocols::DmMotorFeedbackNormal &fb)
{
	return UIntToFloat(fb.angle, kDmJointMitRange.p_min, kDmJointMitRange.p_max, 16);
}

double DmVelocityRadPerSec(const protocols::DmMotorFeedbackNormal &fb)
{
	return UIntToFloat(fb.omega, kDmJointMitRange.v_min, kDmJointMitRange.v_max, 12);
}

void InvalidateDmaRxCache(const uint8_t *data, size_t len)
{
	if ((data == nullptr) || (len == 0U)) {
		return;
	}
	const uint32_t start = HPM_L1C_CACHELINE_ALIGN_DOWN(reinterpret_cast<uint32_t>(data));
	const uint32_t end = HPM_L1C_CACHELINE_ALIGN_UP(
		reinterpret_cast<uint32_t>(data) + static_cast<uint32_t>(len));
	l1c_dc_invalidate(start, end - start);
}

void ProcessImuBytes(const uint8_t *data, size_t size)
{
	for (size_t i = 0U; i < size; ++i) {
		protocols::Hi91Sample sample = {};
		if (g_parser.Feed(data[i], &sample) == protocols::Hi91ParseResult::kFrame) {
			g_imu_sample = sample;
			g_imu_valid = true;
		}
	}
}

void UartCallback(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);
	switch (evt->type) {
	case UART_RX_RDY: {
		const uint8_t *data = evt->data.rx.buf + evt->data.rx.offset;
		InvalidateDmaRxCache(data, evt->data.rx.len);
		(void)ring_buf_put(&g_rx_ring, data, evt->data.rx.len);
		k_sem_give(&g_rx_sem);
		break;
	}
	case UART_RX_BUF_REQUEST: {
		const uint8_t index = g_next_rx_buffer_index;
		g_next_rx_buffer_index = (g_next_rx_buffer_index + 1U) % kRxBufferCount;
		(void)uart_rx_buf_rsp(dev, g_rx_buffers[index], sizeof(g_rx_buffers[index]));
		break;
	}
	case UART_RX_DISABLED:
		g_next_rx_buffer_index = 1U;
		(void)uart_rx_enable(dev, g_rx_buffers[0], sizeof(g_rx_buffers[0]), kRxIdleTimeoutUs);
		break;
	default:
		break;
	}
}

int ConfigureImuUart()
{
	if (!device_is_ready(g_imu_uart)) {
		return -ENODEV;
	}
	struct uart_config config = {};
	config.baudrate = kHi91Baudrate;
	config.parity = UART_CFG_PARITY_NONE;
	config.stop_bits = UART_CFG_STOP_BITS_1;
	config.data_bits = UART_CFG_DATA_BITS_8;
	config.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
	int rc = uart_configure(g_imu_uart, &config);
	if (rc != 0) {
		return rc;
	}
	rc = uart_callback_set(g_imu_uart, UartCallback, nullptr);
	if (rc != 0) {
		return rc;
	}
	g_parser.SetStrictCrc(false);
	return uart_rx_enable(g_imu_uart, g_rx_buffers[0], sizeof(g_rx_buffers[0]),
			      kRxIdleTimeoutUs);
}

void PumpImu()
{
	uint8_t buffer[kRxBufferSize];
	(void)k_sem_take(&g_rx_sem, K_NO_WAIT);
	for (;;) {
		const uint32_t count = ring_buf_get(&g_rx_ring, buffer, sizeof(buffer));
		if (count == 0U) {
			break;
		}
		ProcessImuBytes(buffer, count);
	}
}

const struct device *CanDeviceForBus(uint8_t bus)
{
	if (bus == 0U) {
		return DEVICE_DT_GET(DT_NODELABEL(can0));
	}
	return DEVICE_DT_GET(DT_NODELABEL(can1));
}

void WriteJointFeedback(JointFeedback &slot, const uint8_t data[8], uint8_t dlc)
{
	if (protocols::DecodeDmFeedbackNormal(data, dlc, &slot.feedback) == 0) {
		++slot.sequence;
	}
}

void WriteWheelFeedback(WheelFeedback &slot, const uint8_t data[8], uint8_t dlc)
{
	if (protocols::DecodeDjiFeedback(data, dlc, &slot.feedback) == 0) {
		++slot.sequence;
	}
}

uint8_t DmPayloadCanId(const struct can_frame *frame)
{
	if ((frame == nullptr) || (frame->dlc < 1U)) {
		return 0xffU;
	}
	return frame->data[0] & 0x0fU;
}

void OnCanRx(const struct device *dev, struct can_frame *frame, void *user_data)
{
	ARG_UNUSED(dev);
	if ((frame == nullptr) || ((frame->flags & CAN_FRAME_IDE) != 0U)) {
		return;
	}
	const auto *context = static_cast<const BusContext *>(user_data);
	if (context == nullptr) {
		return;
	}
	if (context->bus < 2U) {
		++g_can_stats[context->bus].rx_count;
	}
	bool routed = false;
	if (context->bus == kLeftLegBus) {
		if (frame->id == kLeftWheelCanId) {
			WriteWheelFeedback(g_left_wheel, frame->data, frame->dlc);
			routed = true;
		} else if (DmPayloadCanId(frame) == kLeftJointBCanId) {
			WriteJointFeedback(g_left_b, frame->data, frame->dlc);
			routed = true;
		} else if (DmPayloadCanId(frame) == kLeftJointDCanId) {
			WriteJointFeedback(g_left_d, frame->data, frame->dlc);
			routed = true;
		} else if (frame->id == kLeftJointBMasterId) {
			WriteJointFeedback(g_left_b, frame->data, frame->dlc);
			routed = true;
		} else if (frame->id == kLeftJointDMasterId) {
			WriteJointFeedback(g_left_d, frame->data, frame->dlc);
			routed = true;
		}
	} else if (context->bus == kRightLegBus) {
		if (frame->id == kRightWheelCanId) {
			WriteWheelFeedback(g_right_wheel, frame->data, frame->dlc);
			routed = true;
		} else if (DmPayloadCanId(frame) == kRightJointBCanId) {
			WriteJointFeedback(g_right_b, frame->data, frame->dlc);
			routed = true;
		} else if (DmPayloadCanId(frame) == kRightJointDCanId) {
			WriteJointFeedback(g_right_d, frame->data, frame->dlc);
			routed = true;
		} else if (frame->id == kRightJointBMasterId) {
			WriteJointFeedback(g_right_b, frame->data, frame->dlc);
			routed = true;
		} else if (frame->id == kRightJointDMasterId) {
			WriteJointFeedback(g_right_d, frame->data, frame->dlc);
			routed = true;
		}
	}
	if (!routed && (context->bus < 2U)) {
		++g_can_stats[context->bus].unknown_count;
		g_can_stats[context->bus].last_unknown_id = frame->id;
	}
}

int SendStdFrame(uint8_t bus, uint16_t can_id, const uint8_t data[8])
{
	if ((bus >= 2U) || (g_can_devs[bus] == nullptr)) {
		return -ENODEV;
	}
	struct can_frame frame = {};
	frame.flags = 0U;
	frame.id = can_id;
	frame.dlc = can_bytes_to_dlc(8U);
	for (uint8_t i = 0U; i < 8U; ++i) {
		frame.data[i] = data[i];
	}
	return can_send(g_can_devs[bus], &frame, K_MSEC(2), nullptr, nullptr);
}

void SendDmControl(uint8_t bus, uint16_t can_id,
		   protocols::DmControlCommand command)
{
	uint8_t data[8] = {};
	if (protocols::GetDmControlCommandFrame(command, data) == 0) {
		(void)SendStdFrame(bus, can_id, data);
	}
}

void SendDmMitTorqueFrame(uint8_t bus, uint16_t can_id, double torque)
{
	protocols::DmMitCommand command = {};
	command.position = 0.0f;
	command.velocity = 0.0f;
	command.kp = 0.0f;
	command.kd = 0.0f;
	command.torque = static_cast<float>(std::clamp(
		torque, -kJointTorqueLimitNm, kJointTorqueLimitNm));
	uint8_t data[8] = {};
	if (protocols::PackDmMitCommand(&command, &kDmJointMitRange, data) == 0) {
		(void)SendStdFrame(bus, can_id, data);
	}
}

void SendDmTorque(uint8_t bus, uint16_t can_id, double torque)
{
	if (!kEnableDmTorqueOutput) {
		return;
	}
	SendDmMitTorqueFrame(bus, can_id, torque);
}

void PollDmFeedback()
{
	if (!kEnableDmMitMode || kEnableDmTorqueOutput || !kPollDmFeedbackWithZeroTorque) {
		return;
	}
	SendDmMitTorqueFrame(kLeftLegBus, kLeftJointBCanId, 0.0);
	SendDmMitTorqueFrame(kLeftLegBus, kLeftJointDCanId, 0.0);
	SendDmMitTorqueFrame(kRightLegBus, kRightJointBCanId, 0.0);
	SendDmMitTorqueFrame(kRightLegBus, kRightJointDCanId, 0.0);
}

void SendWheelTorque(uint8_t bus, uint16_t motor_id, double torque)
{
	if (!kEnableWheelOutput) {
		return;
	}
	const int16_t current = static_cast<int16_t>(std::clamp(
		torque * kDjiCurrentCommandPerNm,
		-static_cast<double>(INT16_MAX), static_cast<double>(INT16_MAX)));
	uint8_t data[8] = {};
	if (protocols::WriteDjiCurrentCommandToSlot(motor_id, current, data) == 0) {
		(void)SendStdFrame(bus, kDjiCurrentCommandCanId, data);
	}
}

int ConfigureCan()
{
	for (uint8_t bus = 0U; bus < 2U; ++bus) {
		g_can_devs[bus] = CanDeviceForBus(bus);
		if ((g_can_devs[bus] == nullptr) || !device_is_ready(g_can_devs[bus])) {
			return -ENODEV;
		}
		struct can_filter filter = {};
		filter.flags = 0U;
		filter.id = 0U;
		filter.mask = 0U;
		if (can_add_rx_filter(g_can_devs[bus], OnCanRx, &g_bus_context[bus],
				      &filter) < 0) {
			return -EIO;
		}
		int rc = can_start(g_can_devs[bus]);
		if ((rc != 0) && (rc != -EALREADY)) {
			return rc;
		}
	}
	return 0;
}

void SendAllEnter()
{
	if (!kEnableDmMitMode) {
		return;
	}
	SendDmControl(kLeftLegBus, kLeftJointBCanId, protocols::DmControlCommand::kEnter);
	SendDmControl(kLeftLegBus, kLeftJointDCanId, protocols::DmControlCommand::kEnter);
	SendDmControl(kRightLegBus, kRightJointBCanId, protocols::DmControlCommand::kEnter);
	SendDmControl(kRightLegBus, kRightJointDCanId, protocols::DmControlCommand::kEnter);
}

bool ComputeLegs(modules::LegKinematics &left,
		 modules::LegKinematics &right)
{
	if ((g_left_b.sequence == 0U) || (g_left_d.sequence == 0U) ||
	    (g_right_b.sequence == 0U) || (g_right_d.sequence == 0U)) {
		return false;
	}
	const bool left_valid = modules::ComputeLegKinematics(
		DmPositionRad(g_left_d.feedback), DmPositionRad(g_left_b.feedback),
		DmVelocityRadPerSec(g_left_d.feedback), DmVelocityRadPerSec(g_left_b.feedback),
		kLeftLegKinematicBranch, left);
	const bool right_valid = modules::ComputeLegKinematics(
		DmPositionRad(g_right_d.feedback), DmPositionRad(g_right_b.feedback),
		DmVelocityRadPerSec(g_right_d.feedback), DmVelocityRadPerSec(g_right_b.feedback),
		kRightLegKinematicBranch, right);
	if (left_valid) {
		left.angle = std::remainder(left.angle - kLeftLegAngleOffset, kTwoPi);
	}
	if (right_valid) {
		right.angle = std::remainder(right.angle - kRightLegAngleOffset, kTwoPi);
	}
	return left_valid && right_valid;
}

double Prbs(uint32_t tick, uint32_t period_ticks, double amplitude)
{
	if (period_ticks == 0U) {
		return 0.0;
	}
	uint32_t x = (tick / period_ticks) * 1103515245U + 12345U;
	x ^= x >> 16U;
	return ((x & 1U) == 0U) ? amplitude : -amplitude;
}

void ApplyLegLengthAndPerturbation(double target_length, double leg_angle_torque)
{
	modules::LegKinematics left = {};
	modules::LegKinematics right = {};
	if (!ComputeLegs(left, right)) {
		return;
	}
	const double left_force = std::clamp(
		kLengthHoldKp * (target_length - left.length) -
			kLengthHoldKd * left.length_rate,
		-80.0, 80.0);
	const double right_force = std::clamp(
		kLengthHoldKp * (target_length - right.length) -
			kLengthHoldKd * right.length_rate,
		-80.0, 80.0);
	auto vmc_from_force = [](const modules::LegKinematics &leg,
				 double axial_force,
				 double angle_torque) {
		modules::LegVmcOutput output = {};
		output.axial_force = axial_force;
		const double radial_x = leg.hx / leg.length;
		const double radial_z = leg.hz / leg.length;
		const double tangential_force = angle_torque / leg.length;
		const double force_x = output.axial_force * radial_x -
				       tangential_force * radial_z;
		const double force_z = output.axial_force * radial_z +
				       tangential_force * radial_x;
		output.joint_torque[0] =
			leg.jacobian[0][0] * force_x + leg.jacobian[1][0] * force_z;
		output.joint_torque[1] =
			leg.jacobian[0][1] * force_x + leg.jacobian[1][1] * force_z;
		return output;
	};
	const modules::LegVmcOutput left_vmc =
		vmc_from_force(left, left_force, 0.5 * leg_angle_torque);
	const modules::LegVmcOutput right_vmc =
		vmc_from_force(right, right_force, 0.5 * leg_angle_torque);
	SendDmTorque(kLeftLegBus, kLeftJointDCanId, left_vmc.joint_torque[0]);
	SendDmTorque(kLeftLegBus, kLeftJointBCanId, left_vmc.joint_torque[1]);
	SendDmTorque(kRightLegBus, kRightJointDCanId, right_vmc.joint_torque[0]);
	SendDmTorque(kRightLegBus, kRightJointBCanId, right_vmc.joint_torque[1]);
}

void PrintHeader()
{
	printk("lqr_sysid_csv,t_ms,point_mm,phase,sample,theta_mrad,theta_rate_mradps,x_mm,x_rate_mms,pitch_mrad,pitch_rate_mradps,u_wheel_mNm,u_leg_mNm,len_l_mm,len_r_mm,roll_mrad,yaw_rate_mradps,valid\n");
}

void PrintDiag(int point_mm, const char *phase)
{
	printk("lqr_diag,t_ms=%u,point_mm=%d,phase=%s,can0_rx=%u,can1_rx=%u,"
	       "can0_unknown=%u,last0=0x%x,can1_unknown=%u,last1=0x%x,"
	       "lb=%u,ld=%u,rb=%u,rd=%u,lw=%u,rw=%u,imu=%u\n",
	       static_cast<unsigned int>(k_uptime_get_32()),
	       point_mm,
	       phase,
	       static_cast<unsigned int>(g_can_stats[0].rx_count),
	       static_cast<unsigned int>(g_can_stats[1].rx_count),
	       static_cast<unsigned int>(g_can_stats[0].unknown_count),
	       g_can_stats[0].last_unknown_id,
	       static_cast<unsigned int>(g_can_stats[1].unknown_count),
	       g_can_stats[1].last_unknown_id,
	       static_cast<unsigned int>(g_left_b.sequence),
	       static_cast<unsigned int>(g_left_d.sequence),
	       static_cast<unsigned int>(g_right_b.sequence),
	       static_cast<unsigned int>(g_right_d.sequence),
	       static_cast<unsigned int>(g_left_wheel.sequence),
	       static_cast<unsigned int>(g_right_wheel.sequence),
	       g_imu_valid ? 1U : 0U);
}

void PrintSample(int point_mm, const char *phase, uint32_t sample_index,
		 double u_wheel, double u_leg, double &x_position)
{
	modules::LegKinematics left = {};
	modules::LegKinematics right = {};
	const bool legs_valid = ComputeLegs(left, right);
	const double pitch = g_imu_valid ? g_imu_sample.pitch_deg * kDegToRad : 0.0;
	const double roll = g_imu_valid ? g_imu_sample.roll_deg * kDegToRad : 0.0;
	const double pitch_rate = g_imu_valid ? g_imu_sample.gyro_dps[1] * kDpsToRadPerSec : 0.0;
	const double yaw_rate = g_imu_valid ? g_imu_sample.gyro_dps[2] * kDpsToRadPerSec : 0.0;
	const double left_wheel_speed =
		static_cast<double>(g_left_wheel.feedback.omega) * kDjiRpmToRadPerSec;
	const double right_wheel_speed =
		static_cast<double>(g_right_wheel.feedback.omega) * kDjiRpmToRadPerSec;
	const double x_rate = 0.5 * kWheelRadiusM * (left_wheel_speed + right_wheel_speed);
	x_position += x_rate * (static_cast<double>(kPrintEveryMs) / 1000.0);
	const double theta = legs_valid
		? std::remainder(0.5 * (left.angle + right.angle) + pitch, kTwoPi)
		: 0.0;
	const double theta_rate = legs_valid
		? 0.5 * (left.angle_rate + right.angle_rate) + pitch_rate
		: 0.0;
	printk("lqr_sysid_csv,%u,%d,%s,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u\n",
	       static_cast<unsigned int>(k_uptime_get_32()),
	       point_mm,
	       phase,
	       static_cast<unsigned int>(sample_index),
	       static_cast<int>(theta * 1000.0),
	       static_cast<int>(theta_rate * 1000.0),
	       static_cast<int>(x_position * 1000.0),
	       static_cast<int>(x_rate * 1000.0),
	       static_cast<int>(pitch * 1000.0),
	       static_cast<int>(pitch_rate * 1000.0),
	       static_cast<int>(u_wheel * 1000.0),
	       static_cast<int>(u_leg * 1000.0),
	       legs_valid ? static_cast<int>(left.length * 1000.0) : 0,
	       legs_valid ? static_cast<int>(right.length * 1000.0) : 0,
	       static_cast<int>(roll * 1000.0),
	       static_cast<int>(yaw_rate * 1000.0),
	       (legs_valid && g_imu_valid) ? 1U : 0U);
}

void RunPoint(int point_mm, double &x_position)
{
	const double target_length = static_cast<double>(point_mm) / 1000.0;
	uint32_t sample_index = 0U;
	const uint32_t settle_ticks = kSettleMs / kControlPeriodMs;
	const uint32_t sample_ticks = kSampleMs / kControlPeriodMs;
	const uint32_t print_ticks = std::max<uint32_t>(1U, kPrintEveryMs / kControlPeriodMs);
	for (uint32_t tick = 0U; tick < settle_ticks + sample_ticks; ++tick) {
		PumpImu();
		PollDmFeedback();
		const bool sampling = tick >= settle_ticks;
		const double u_wheel = (sampling && kEnableWheelPerturbation)
			? std::clamp(Prbs(tick - settle_ticks, 37U, kWheelPerturbNm),
				     -kWheelTorqueLimitNm, kWheelTorqueLimitNm)
			: 0.0;
		const double u_leg = (sampling && kEnableLegAnglePerturbation)
			? Prbs(tick - settle_ticks, 53U, kLegAnglePerturbNm)
			: 0.0;
		ApplyLegLengthAndPerturbation(target_length, u_leg);
		SendWheelTorque(kLeftLegBus, kLeftWheelCanId, 0.5 * u_wheel);
		SendWheelTorque(kRightLegBus, kRightWheelCanId, -0.5 * u_wheel);
		if ((tick % print_ticks) == 0U) {
			PrintSample(point_mm, sampling ? "sample" : "settle",
				    sample_index++, u_wheel, u_leg, x_position);
		}
		if ((tick % 1000U) == 0U) {
			PrintDiag(point_mm, sampling ? "sample" : "settle");
		}
		k_sleep(K_MSEC(kControlPeriodMs));
	}
}

}  // namespace

int main()
{
	printk("lqr_gain_sample_test physical sysid started\n");
	printk("dm_mit=%u dm_torque=%u dm_zero_poll=%u wheel_output=%u wheel_prbs=%u leg_prbs=%u\n",
	       kEnableDmMitMode ? 1U : 0U,
	       kEnableDmTorqueOutput ? 1U : 0U,
	       kPollDmFeedbackWithZeroTorque ? 1U : 0U,
	       kEnableWheelOutput ? 1U : 0U,
	       kEnableWheelPerturbation ? 1U : 0U,
	       kEnableLegAnglePerturbation ? 1U : 0U);
	int rc = ConfigureCan();
	if (rc != 0) {
		printk("CAN init failed: %d\n", rc);
		return 0;
	}
	rc = ConfigureImuUart();
	if (rc != 0) {
		printk("HI91 UART init failed: %d\n", rc);
		return 0;
	}
	for (uint32_t elapsed = 0U; elapsed < kMitEnterRepeatMs; elapsed += 10U) {
		SendAllEnter();
		k_sleep(K_MSEC(10));
	}
	PrintHeader();
	double x_position = 0.0;
	for (int point_mm = kMinLegLengthMm; point_mm <= kMaxLegLengthMm;
	     point_mm += kStepLegLengthMm) {
		RunPoint(point_mm, x_position);
	}
	printk("lqr_gain_sample_test physical sysid done\n");
	return 0;
}
