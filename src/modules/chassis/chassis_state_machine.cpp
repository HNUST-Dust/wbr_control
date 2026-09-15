/* SPDX-License-Identifier: Apache-2.0 */

#include "chassis_state_machine.h"

#include <cmath>

#include <zephyr/logging/log.h>

#include "chassis_config.h"

LOG_MODULE_DECLARE(chassis_module);

namespace modules
{

ChassisStateTransition ChassisStateMachine::Update(const ChassisCycleInput &input)
{
	ChassisStateTransition transition;

	// Only an explicit disable clears a tilt latch, preventing autonomous restart.
	if (!input.requested_enable && tilt_fault_latched_) {
		tilt_fault_latched_ = false;
	}

	if (input.requested_enable != last_requested_enable_) {
		transition.enable_changed = true;
		transition.enabled = input.requested_enable;
		balance_phase_reached_ = false;
		if (input.requested_enable) {
			arm_reset_reason_ = DmArmResetReason::kRemoteEnableEdge;
		}
		last_requested_enable_ = input.requested_enable;
		LOG_INF("chassis request %s", input.requested_enable ? "enabled" : "disabled");
	}

	if (input.requested_enable && input.imu_fresh) {
		const double pitch_deg = static_cast<double>(input.imu.pitch_deg);
		if (!tilt_fault_latched_ && std::abs(pitch_deg) >= chassis_config::kTiltCutoffDeg) {
			tilt_fault_latched_ = true;
			LOG_ERR("tilt fault pitch_mdeg=%d", static_cast<int>(pitch_deg * 1000.0));
		}
	}

	if (!input.requested_enable) {
		state_ = ChassisControlState::kDisabled;
	} else if (tilt_fault_latched_) {
		state_ = ChassisControlState::kTiltFault;
	} else if (!input.imu_fresh) {
		state_ = ChassisControlState::kSafetyStop;
	} else if (!input.arm_complete || !input.dm_ready) {
		state_ = ChassisControlState::kDmArming;
	} else if (!input.feedback_valid) {
		state_ = ChassisControlState::kWaitingFeedback;
	} else {
		state_ = balance_phase_reached_ ? ChassisControlState::kBalance
					       : ChassisControlState::kStool;
	}
	if (state_ == ChassisControlState::kSafetyStop) {
		arm_reset_reason_ = DmArmResetReason::kImuFreshness;
	} else if (state_ == ChassisControlState::kTiltFault) {
		arm_reset_reason_ = DmArmResetReason::kTiltFault;
	}
	return transition;
}

} // namespace modules
