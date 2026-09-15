/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "chassis_actuator.h"

#include <algorithm>
#include <cmath>

#include <platform/drivers/communication/can_dispatch.h>

#include "chassis_config.h"

namespace modules
{
namespace cfg = chassis_config;

bool ChassisActuator::arm_complete() const
{
	return dm_arm_ticks_ >= cfg::kDmArmTicks;
}

bool ChassisActuator::Apply(ChassisControlState state, uint32_t loop_ticks,
			    ChassisControlOutput &output)
{
	switch (state) {
	case ChassisControlState::kDisabled:
	case ChassisControlState::kSafetyStop:
	case ChassisControlState::kTiltFault:
		if ((loop_ticks % 100U) == 0U) {
			SendDmControl(protocols::DmControlCommand::kExit);
		}
		if ((loop_ticks % 10U) == 0U) {
			SendZeroWheels();
		}
		dm_arm_ticks_ = 0U;
		return true;

	case ChassisControlState::kDmArming:
		if ((loop_ticks % cfg::kDmModePeriodTicks) == 0U) {
			SendDmControl(dm_arm_ticks_ < cfg::kDmClearTicks
					      ? protocols::DmControlCommand::kClearError
					      : protocols::DmControlCommand::kEnter);
		}
		SendZeroWheels();
		if (dm_arm_ticks_ < cfg::kDmArmTicks) {
			++dm_arm_ticks_;
		}
		return true;

	case ChassisControlState::kWaitingFeedback:
		SendZeroOutput();
		return true;

	case ChassisControlState::kStool:
		SendJointOnly(output.joint_torque);
		return false;

	case ChassisControlState::kBalance:
		SendFullOutput(output);
		return false;
	}
	return true;
}

void ChassisActuator::SendDmControl(protocols::DmControlCommand command) const
{
	uint8_t data[8] = {};
	if (protocols::GetDmControlCommandFrame(command, data) != 0) {
		return;
	}
	const auto submit_joint = [&](Side side, Joint joint) {
		const cfg::MotorAddress &motor = cfg::kLegHardware.Get(side).joint.Get(joint);
		(void)platform::SubmitCanStandardFrame(motor.tx_slot, motor.bus, motor.can_id,
					       data, sizeof(data));
	};
	submit_joint(Side::kLeft, Joint::kB);
	submit_joint(Side::kLeft, Joint::kD);
	submit_joint(Side::kRight, Joint::kB);
	submit_joint(Side::kRight, Joint::kD);
}

void ChassisActuator::SendDmTorque(Side side, Joint joint, double torque) const
{
	protocols::DmMitCommand command = {};
	command.torque = static_cast<float>(torque);
	uint8_t data[8] = {};
	if (protocols::PackDmMitCommand(&command, &cfg::kDmRange, data) != 0) {
		return;
	}
	const cfg::MotorAddress &motor = cfg::kLegHardware.Get(side).joint.Get(joint);
	(void)platform::SubmitCanStandardFrame(motor.tx_slot, motor.bus, motor.can_id, data,
				       sizeof(data));
}

void ChassisActuator::SendWheelCurrent(Side side, int16_t current) const
{
	uint8_t data[8] = {};
	if (protocols::WriteDjiCurrentCommandToSlot(
		    cfg::kLegHardware.Get(side).wheel.can_id, current, data) != 0) {
		return;
	}
	const cfg::MotorAddress &wheel = cfg::kLegHardware.Get(side).wheel;
	(void)platform::SubmitCanStandardFrame(wheel.tx_slot, wheel.bus, cfg::kWheelCommandId,
				       data, sizeof(data));
}

void ChassisActuator::SendScheduledOutputs(
	const SidePair<JointPair<double>> &joint_torque,
	const SidePair<int16_t> &wheel_current) const
{
	SendDmTorque(Side::kLeft, Joint::kB, joint_torque.left.b);
	SendDmTorque(Side::kRight, Joint::kB, joint_torque.right.b);
	SendDmTorque(Side::kLeft, Joint::kD, joint_torque.left.d);
	SendDmTorque(Side::kRight, Joint::kD, joint_torque.right.d);
	SendWheelCurrent(Side::kLeft, wheel_current.left);
	SendWheelCurrent(Side::kRight, wheel_current.right);
}

void ChassisActuator::SendZeroWheels() const
{
	SendWheelCurrent(Side::kLeft, 0);
	SendWheelCurrent(Side::kRight, 0);
}

void ChassisActuator::SendZeroOutput() const
{
	SendScheduledOutputs({}, {});
}

void ChassisActuator::SendJointOnly(
	const SidePair<JointPair<double>> &joint_torque) const
{
	SendScheduledOutputs(joint_torque, {});
}

void ChassisActuator::SendFullOutput(ChassisControlOutput &output) const
{
	const auto torque_to_current = [](double torque) {
		const double current = std::isfinite(torque)
			? std::clamp(torque * cfg::kDjiCurrentPerNm,
				     -static_cast<double>(cfg::kDjiProtocolCurrentLimit),
				     static_cast<double>(cfg::kDjiProtocolCurrentLimit))
			: 0.0;
		return static_cast<int16_t>(current);
	};

	SidePair<int16_t> wheel_current;
	wheel_current.left = torque_to_current(
		cfg::kLegHardware.left.wheel_command_sign * output.physical_wheel_torque.left);
	wheel_current.right = torque_to_current(
		cfg::kLegHardware.right.wheel_command_sign * output.physical_wheel_torque.right);
	output.wheel_torque_saturated.left =
		std::abs(static_cast<int32_t>(wheel_current.left)) >= cfg::kDjiProtocolCurrentLimit;
	output.wheel_torque_saturated.right =
		std::abs(static_cast<int32_t>(wheel_current.right)) >= cfg::kDjiProtocolCurrentLimit;
	output.sent_wheel_torque.left = cfg::kLegHardware.left.wheel_command_sign *
		static_cast<double>(wheel_current.left) / cfg::kDjiCurrentPerNm;
	output.sent_wheel_torque.right = cfg::kLegHardware.right.wheel_command_sign *
		static_cast<double>(wheel_current.right) / cfg::kDjiCurrentPerNm;
	SendScheduledOutputs(output.joint_torque, wheel_current);
}

} // namespace modules
