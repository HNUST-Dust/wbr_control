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
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <modules/chassis/leg_kinematics.h>
#include <protocols/motors/dm_motor_protocol.h>

namespace {

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

constexpr bool kEnableLeftLeg = true;
constexpr bool kEnableRightLeg = true;
constexpr int kLeftLegKinematicBranch = 1;
constexpr int kRightLegKinematicBranch = -1;

constexpr double kForceMaxN = 40.0;
constexpr double kForceStepN = 2.0;
constexpr uint32_t kDwellMs = 1500U;
constexpr uint32_t kZeroDwellMs = 300U;
constexpr uint32_t kControlPeriodMs = 1U;
constexpr uint32_t kEnterRepeatMs = 1000U;
constexpr uint32_t kPrintPeriodMs = 100U;
constexpr double kJointTorqueLimitNm = 8.0;
constexpr bool kExitBetweenSweepCycles = false;

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
	protocols::DmMotorFeedbackNormal feedback;
	uint32_t sequence = 0U;
};

struct BusContext {
	uint8_t bus = 0U;
};

struct LegCommand {
	bool valid = false;
	modules::LegKinematics leg = {};
	double joint_d_torque = 0.0;
	double joint_b_torque = 0.0;
};

const struct device *g_can_devs[2] = {};
BusContext g_bus_context[2] = {{0U}, {1U}};
JointFeedback g_left_b;
JointFeedback g_left_d;
JointFeedback g_right_b;
JointFeedback g_right_d;

float UIntToFloat(uint16_t value, float min_value, float max_value,
		  uint8_t bits)
{
	const float span = max_value - min_value;
	const float max_int = static_cast<float>((1U << bits) - 1U);
	return static_cast<float>(value) * span / max_int + min_value;
}

double DmPositionRad(const protocols::DmMotorFeedbackNormal &fb)
{
	return UIntToFloat(fb.angle, kDmJointMitRange.p_min,
			   kDmJointMitRange.p_max, 16);
}

double DmVelocityRadPerSec(const protocols::DmMotorFeedbackNormal &fb)
{
	return UIntToFloat(fb.omega, kDmJointMitRange.v_min,
			   kDmJointMitRange.v_max, 12);
}

const char *CanStateToString(enum can_state state)
{
	switch (state) {
	case CAN_STATE_ERROR_ACTIVE:
		return "error-active";
	case CAN_STATE_ERROR_WARNING:
		return "error-warning";
	case CAN_STATE_ERROR_PASSIVE:
		return "error-passive";
	case CAN_STATE_BUS_OFF:
		return "bus-off";
	case CAN_STATE_STOPPED:
		return "stopped";
	default:
		return "unknown";
	}
}

void PrintCanState(const struct device *dev, uint8_t bus, const char *tag)
{
	enum can_state state;
	struct can_bus_err_cnt err_cnt;
	const int rc = can_get_state(dev, &state, &err_cnt);
	if (rc == 0) {
		printk("[can] bus%u %s: %s tec=%u rec=%u\n",
		       static_cast<unsigned int>(bus), tag, CanStateToString(state),
		       static_cast<unsigned int>(err_cnt.tx_err_cnt),
		       static_cast<unsigned int>(err_cnt.rx_err_cnt));
	} else {
		printk("[can] bus%u %s: can_get_state rc=%d\n",
		       static_cast<unsigned int>(bus), tag, rc);
	}
}

const struct device *CanDeviceForBus(uint8_t bus)
{
	if (bus == 0U) {
#if DT_NODE_HAS_STATUS(DT_NODELABEL(can0), okay)
		return DEVICE_DT_GET(DT_NODELABEL(can0));
#else
		return nullptr;
#endif
	}
	if (bus == 1U) {
#if DT_NODE_HAS_STATUS(DT_NODELABEL(can1), okay)
		return DEVICE_DT_GET(DT_NODELABEL(can1));
#else
		return nullptr;
#endif
	}
	return nullptr;
}

uint32_t CanBitrateForBus(uint8_t bus)
{
	if (bus == 0U) {
		return DT_PROP_OR(DT_NODELABEL(can0), bitrate, 1000000);
	}
	if (bus == 1U) {
		return DT_PROP_OR(DT_NODELABEL(can1), bitrate, 1000000);
	}
	return 1000000U;
}

void WriteFeedback(JointFeedback &slot, const uint8_t data[8], uint8_t dlc)
{
	if (protocols::DecodeDmFeedbackNormal(data, dlc,
							&slot.feedback) == 0) {
		++slot.sequence;
	}
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

	if (context->bus == kLeftLegBus) {
		if (frame->id == kLeftJointBMasterId) {
			WriteFeedback(g_left_b, frame->data, frame->dlc);
		} else if (frame->id == kLeftJointDMasterId) {
			WriteFeedback(g_left_d, frame->data, frame->dlc);
		}
	} else if (context->bus == kRightLegBus) {
		if (frame->id == kRightJointBMasterId) {
			WriteFeedback(g_right_b, frame->data, frame->dlc);
		} else if (frame->id == kRightJointDMasterId) {
			WriteFeedback(g_right_d, frame->data, frame->dlc);
		}
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

void SendDmTorque(uint8_t bus, uint16_t can_id, double torque)
{
	protocols::DmMitCommand command = {};
	command.position = 0.0f;
	command.velocity = 0.0f;
	command.kp = 0.0f;
	command.kd = 0.0f;
	command.torque = static_cast<float>(std::clamp(
		torque, -kJointTorqueLimitNm, kJointTorqueLimitNm));

	uint8_t data[8] = {};
	if (protocols::PackDmMitCommand(
		    &command, &kDmJointMitRange, data) == 0) {
		(void)SendStdFrame(bus, can_id, data);
	}
}

void SendAllEnter()
{
	if (kEnableLeftLeg) {
		SendDmControl(kLeftLegBus, kLeftJointBCanId,
			      protocols::DmControlCommand::kEnter);
		SendDmControl(kLeftLegBus, kLeftJointDCanId,
			      protocols::DmControlCommand::kEnter);
	}
	if (kEnableRightLeg) {
		SendDmControl(kRightLegBus, kRightJointBCanId,
			      protocols::DmControlCommand::kEnter);
		SendDmControl(kRightLegBus, kRightJointDCanId,
			      protocols::DmControlCommand::kEnter);
	}
}

void SendAllExit()
{
	if (kEnableLeftLeg) {
		SendDmControl(kLeftLegBus, kLeftJointBCanId,
			      protocols::DmControlCommand::kExit);
		SendDmControl(kLeftLegBus, kLeftJointDCanId,
			      protocols::DmControlCommand::kExit);
	}
	if (kEnableRightLeg) {
		SendDmControl(kRightLegBus, kRightJointBCanId,
			      protocols::DmControlCommand::kExit);
		SendDmControl(kRightLegBus, kRightJointDCanId,
			      protocols::DmControlCommand::kExit);
	}
}

LegCommand ComputeAxialForceCommand(const JointFeedback &joint_b,
				    const JointFeedback &joint_d,
				    int branch, double axial_force)
{
	LegCommand command = {};
	if ((joint_b.sequence == 0U) || (joint_d.sequence == 0U)) {
		return command;
	}

	command.valid = modules::ComputeLegKinematics(
		DmPositionRad(joint_d.feedback),
		DmPositionRad(joint_b.feedback),
		DmVelocityRadPerSec(joint_d.feedback),
		DmVelocityRadPerSec(joint_b.feedback),
		branch, command.leg);
	if (!command.valid) {
		return command;
	}

	const double radial_x = command.leg.hx / command.leg.length;
	const double radial_z = command.leg.hz / command.leg.length;
	const double force_x = axial_force * radial_x;
	const double force_z = axial_force * radial_z;
	command.joint_d_torque =
		command.leg.jacobian[0][0] * force_x +
		command.leg.jacobian[1][0] * force_z;
	command.joint_b_torque =
		command.leg.jacobian[0][1] * force_x +
		command.leg.jacobian[1][1] * force_z;
	command.joint_d_torque = std::clamp(command.joint_d_torque,
					    -kJointTorqueLimitNm,
					    kJointTorqueLimitNm);
	command.joint_b_torque = std::clamp(command.joint_b_torque,
					    -kJointTorqueLimitNm,
					    kJointTorqueLimitNm);
	return command;
}

void ApplyForce(double axial_force)
{
	const LegCommand left = ComputeAxialForceCommand(
		g_left_b, g_left_d, kLeftLegKinematicBranch, axial_force);
	const LegCommand right = ComputeAxialForceCommand(
		g_right_b, g_right_d, kRightLegKinematicBranch, axial_force);

	if (kEnableLeftLeg && left.valid) {
		SendDmTorque(kLeftLegBus, kLeftJointBCanId, left.joint_b_torque);
		SendDmTorque(kLeftLegBus, kLeftJointDCanId, left.joint_d_torque);
	} else if (kEnableLeftLeg) {
		SendDmTorque(kLeftLegBus, kLeftJointBCanId, 0.0);
		SendDmTorque(kLeftLegBus, kLeftJointDCanId, 0.0);
	}

	if (kEnableRightLeg && right.valid) {
		SendDmTorque(kRightLegBus, kRightJointBCanId, right.joint_b_torque);
		SendDmTorque(kRightLegBus, kRightJointDCanId, right.joint_d_torque);
	} else if (kEnableRightLeg) {
		SendDmTorque(kRightLegBus, kRightJointBCanId, 0.0);
		SendDmTorque(kRightLegBus, kRightJointDCanId, 0.0);
	}
}

void PrintForceState(const char *phase, double axial_force)
{
	const LegCommand left = ComputeAxialForceCommand(
		g_left_b, g_left_d, kLeftLegKinematicBranch, axial_force);
	const LegCommand right = ComputeAxialForceCommand(
		g_right_b, g_right_d, kRightLegKinematicBranch, axial_force);

	printk("[leg_ff] phase=%s force_N=%d ", phase,
	       static_cast<int>(axial_force));
	if (left.valid) {
		printk("L len_mm=%d rate_mms=%d tq_mNm B=%d D=%d ",
		       static_cast<int>(left.leg.length * 1000.0),
		       static_cast<int>(left.leg.length_rate * 1000.0),
		       static_cast<int>(left.joint_b_torque * 1000.0),
		       static_cast<int>(left.joint_d_torque * 1000.0));
	} else {
		printk("L invalid seq B=%u D=%u ",
		       static_cast<unsigned int>(g_left_b.sequence),
		       static_cast<unsigned int>(g_left_d.sequence));
	}
	if (right.valid) {
		printk("R len_mm=%d rate_mms=%d tq_mNm B=%d D=%d\n",
		       static_cast<int>(right.leg.length * 1000.0),
		       static_cast<int>(right.leg.length_rate * 1000.0),
		       static_cast<int>(right.joint_b_torque * 1000.0),
		       static_cast<int>(right.joint_d_torque * 1000.0));
	} else {
		printk("R invalid seq B=%u D=%u\n",
		       static_cast<unsigned int>(g_right_b.sequence),
		       static_cast<unsigned int>(g_right_d.sequence));
	}
}

void HoldForce(const char *phase, double axial_force, uint32_t dwell_ms)
{
	const uint32_t loops = dwell_ms / kControlPeriodMs;
	const uint32_t print_loops = std::max(1U, kPrintPeriodMs / kControlPeriodMs);
	for (uint32_t i = 0U; i < loops; ++i) {
		ApplyForce(axial_force);
		if ((i % print_loops) == 0U) {
			PrintForceState(phase, axial_force);
		}
		k_sleep(K_MSEC(kControlPeriodMs));
	}
}

int InitCanBus(uint8_t bus)
{
	const struct device *dev = CanDeviceForBus(bus);
	if (dev == nullptr || !device_is_ready(dev)) {
		printk("[can] bus%u device not ready\n", static_cast<unsigned int>(bus));
		return -ENODEV;
	}

	g_can_devs[bus] = dev;
	printk("[can] bus%u using %s\n", static_cast<unsigned int>(bus), dev->name);
	int rc = can_set_mode(dev, CAN_MODE_NORMAL);
	if (rc != 0) {
		printk("[can] bus%u can_set_mode rc=%d\n",
		       static_cast<unsigned int>(bus), rc);
		return rc;
	}
	rc = can_set_bitrate(dev, CanBitrateForBus(bus));
	if (rc != 0) {
		printk("[can] bus%u can_set_bitrate rc=%d\n",
		       static_cast<unsigned int>(bus), rc);
		return rc;
	}
	rc = can_start(dev);
	if ((rc != 0) && (rc != -EALREADY)) {
		printk("[can] bus%u can_start rc=%d\n",
		       static_cast<unsigned int>(bus), rc);
		return rc;
	}

	const struct can_filter all_standard_frames = {
		.id = 0U,
		.mask = 0U,
		.flags = 0U,
	};
	const int filter_id = can_add_rx_filter(
		dev, OnCanRx, &g_bus_context[bus], &all_standard_frames);
	if (filter_id < 0) {
		printk("[can] bus%u add rx filter rc=%d\n",
		       static_cast<unsigned int>(bus), filter_id);
		return filter_id;
	}
	PrintCanState(dev, bus, "after start");
	return 0;
}

}  // namespace

int main()
{
	printk("leg_feedforward_test started\n");
	printk("[leg_ff] sweep: 0 -> -%dN -> short 0 -> +%dN -> short 0, step=%dN dwell=%ums zero=%ums\n",
	       static_cast<int>(kForceMaxN), static_cast<int>(kForceMaxN),
	       static_cast<int>(kForceStepN), static_cast<unsigned int>(kDwellMs),
	       static_cast<unsigned int>(kZeroDwellMs));
	printk("[leg_ff] joint torque clamp=%d mNm\n",
	       static_cast<int>(kJointTorqueLimitNm * 1000.0));

	if (InitCanBus(kLeftLegBus) != 0 || InitCanBus(kRightLegBus) != 0) {
		printk("[leg_ff] CAN init failed\n");
		return -ENODEV;
	}

	printk("[leg_ff] entering DM MIT mode...\n");
	const uint32_t enter_loops = kEnterRepeatMs / kControlPeriodMs;
	for (uint32_t i = 0U; i < enter_loops; ++i) {
		SendAllEnter();
		k_sleep(K_MSEC(kControlPeriodMs));
	}

	while (true) {
		HoldForce("zero", 0.0, kZeroDwellMs);
		for (double force = -kForceStepN; force >= -kForceMaxN - 1e-9;
		     force -= kForceStepN) {
			HoldForce("retract", force, kDwellMs);
		}
		HoldForce("zero", 0.0, kZeroDwellMs);
		for (double force = kForceStepN; force <= kForceMaxN + 1e-9;
		     force += kForceStepN) {
			HoldForce("extend", force, kDwellMs);
		}
		HoldForce("zero", 0.0, kZeroDwellMs);
		if (kExitBetweenSweepCycles) {
			printk("[leg_ff] sweep cycle complete; sending exit for 500 ms\n");
			for (uint32_t i = 0U; i < 500U; ++i) {
				SendAllExit();
				k_sleep(K_MSEC(1));
			}
			printk("[leg_ff] re-entering DM MIT mode\n");
			for (uint32_t i = 0U; i < enter_loops; ++i) {
				SendAllEnter();
				k_sleep(K_MSEC(kControlPeriodMs));
			}
		}
	}
}
