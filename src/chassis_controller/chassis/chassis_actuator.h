/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <protocols/motors/dm_motor_protocol.h>

#include "chassis_types.h"

namespace modules
{

/** Converts physical chassis requests to bounded CAN motor commands. */
class ChassisActuator
{
public:
	/** Apply the state-dependent output policy; returns true when control state must reset. */
	bool Apply(ChassisControlState state, uint32_t loop_ticks,
		   ChassisControlOutput &output);
	bool arm_complete() const;
	void ResetArming() { dm_arm_ticks_ = 0U; }

	void SendDmControl(protocols::DmControlCommand command) const;
	void SendZeroWheels() const;
	void SendZeroOutput() const;
	void SendJointOnly(const SidePair<JointPair<double>> &joint_torque) const;
	void SendFullOutput(ChassisControlOutput &output) const;

private:
	uint32_t dm_arm_ticks_ = 0U;
	void SendDmTorque(Side side, Joint joint, double torque) const;
	void SendWheelCurrent(Side side, int16_t current) const;
	void SendScheduledOutputs(const SidePair<JointPair<double>> &joint_torque,
				  const SidePair<int16_t> &wheel_current) const;
};

} // namespace modules
