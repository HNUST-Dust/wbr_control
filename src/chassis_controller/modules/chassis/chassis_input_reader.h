/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "chassis_types.h"

namespace modules
{

/** Owns channel freshness history and produces one coherent physical snapshot. */
class ChassisInputReader
{
public:
	void Read(ChassisCycleInput &input, uint32_t now_ms, double dt,
		  bool arm_complete);

	const SidePair<protocols::DjiMotorFeedback> &wheel_feedback() const
	{
		return wheel_feedback_;
	}

	const SidePair<JointPair<protocols::DmMotorFeedbackNormal>> &joint_feedback() const
	{
		return joint_feedback_;
	}

private:
	uint32_t last_remote_sequence_ = 0U;
	uint32_t last_remote_update_ms_ = 0U;
	uint32_t last_imu_sequence_ = 0U;
	SidePair<protocols::DjiMotorFeedback> wheel_feedback_;
	SidePair<JointPair<protocols::DmMotorFeedbackNormal>> joint_feedback_;
};

} // namespace modules
