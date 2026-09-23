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
	UpdateSafetyLatches(input);
	climb_request_edge_ = input.remote.climb_stairs && !last_climb_request_;
	last_climb_request_ = input.remote.climb_stairs;
	jump_request_edge_ = input.remote.jump && !last_jump_request_;
	last_jump_request_ = input.remote.jump;
	jump_cooldown_remaining_s_ = std::max(0.0, jump_cooldown_remaining_s_ - input.dt);

	if (input.requested_enable != last_requested_enable_) {
		transition.enable_changed = true;
		transition.enabled = input.requested_enable;
		if (input.requested_enable) {
			recovery_required_ = true;
			recovery_completed_ = false;
			recovery_failed_ = false;
			arm_reset_reason_ = DmArmResetReason::kRemoteEnableEdge;
		}
		last_requested_enable_ = input.requested_enable;
		LOG_INF("chassis request %s", input.requested_enable ? "enabled" : "disabled");
	}

	transition.from = state_;
	transition.to = DetermineNextState(input, transition.reason);
	transition.changed = transition.to != transition.from;
	state_ = transition.to;
	if (transition.changed && transition.to == ChassisControlState::kClimbStairs) {
		climb_completed_ = false;
		climb_failed_ = false;
	}
	if (transition.changed && transition.to == ChassisControlState::kFlight) {
		flight_completed_ = false;
		flight_failed_ = false;
	}
	if (transition.changed && transition.to == ChassisControlState::kJump) {
		jump_completed_ = false;
		jump_failed_ = false;
	}
	if (transition.changed && transition.from == ChassisControlState::kJump) {
		jump_cooldown_remaining_s_ =
			static_cast<double>(chassis_config::kJumpCooldownMs) * 0.001;
	}
	if (transition.changed && transition.from == ChassisControlState::kRecovery &&
	    transition.to == ChassisControlState::kBalance) {
		recovery_required_ = false;
		recovery_completed_ = false;
		tilt_fault_latched_ = false;
	}
	if (climb_failed_ || recovery_failed_ || flight_failed_ || jump_failed_) {
		action_fault_latched_ = true;
	}

	if (state_ == ChassisControlState::kSafetyStop) {
		arm_reset_reason_ = DmArmResetReason::kImuFreshness;
	} else if (state_ == ChassisControlState::kTiltFault) {
		arm_reset_reason_ = DmArmResetReason::kTiltFault;
	}
	return transition;
}

void ChassisStateMachine::UpdateSafetyLatches(const ChassisCycleInput &input)
{
	// Only an explicit, fresh remote disable clears the latch. A link timeout must not
	// silently arm the chassis for an automatic restart when the link returns.
	if (input.remote_fresh && !input.remote.robot_enable) {
		tilt_fault_latched_ = false;
		action_fault_latched_ = false;
		climb_completed_ = false;
		climb_failed_ = false;
		flight_detected_ = false;
		flight_completed_ = false;
		flight_failed_ = false;
		jump_completed_ = false;
		jump_failed_ = false;
		recovery_required_ = false;
		recovery_completed_ = false;
		recovery_failed_ = false;
	}

	if (input.requested_enable && input.imu_fresh &&
	    state_ != ChassisControlState::kRecovery && !recovery_required_) {
		const double pitch_deg = static_cast<double>(input.imu.pitch_deg);
		if (!tilt_fault_latched_ && std::abs(pitch_deg) >= chassis_config::kTiltCutoffDeg) {
			tilt_fault_latched_ = true;
			LOG_ERR("tilt fault pitch_mdeg=%d", static_cast<int>(pitch_deg * 1000.0));
		}
	}
}

ChassisControlState ChassisStateMachine::DetermineNextState(
	const ChassisCycleInput &input, ChassisTransitionReason &reason) const
{
	// Global transitions are evaluated before the state-specific nominal path.
	if (!input.requested_enable) {
		reason = input.remote_fresh ? ChassisTransitionReason::kRemoteDisabled
					    : ChassisTransitionReason::kRemoteTimeout;
		return ChassisControlState::kDisabled;
	}
	if (action_fault_latched_) {
		reason = recovery_failed_ ? ChassisTransitionReason::kRecoveryFailed
			: (flight_failed_ ? ChassisTransitionReason::kFlightFailed
			: (jump_failed_ ? ChassisTransitionReason::kJumpFailed
					: ChassisTransitionReason::kClimbFailed));
		return ChassisControlState::kActionFault;
	}
	if (!input.imu_fresh) {
		reason = ChassisTransitionReason::kImuLost;
		return ChassisControlState::kSafetyStop;
	}
	if (tilt_fault_latched_ && state_ != ChassisControlState::kRecovery &&
	    !recovery_required_) {
		reason = ChassisTransitionReason::kTiltExceeded;
		return ChassisControlState::kTiltFault;
	}

	switch (state_) {
	case ChassisControlState::kDisabled:
		reason = ChassisTransitionReason::kRemoteEnabled;
		return ChassisControlState::kDmArming;
	case ChassisControlState::kSafetyStop:
		reason = ChassisTransitionReason::kImuRecovered;
		return ChassisControlState::kDmArming;
	case ChassisControlState::kDmArming:
		if (!input.arm_complete || !input.dm_ready) {
			return state_;
		}
		if (!input.feedback_valid) {
			reason = ChassisTransitionReason::kArmingCompleted;
			return ChassisControlState::kWaitingFeedback;
		}
		reason = ChassisTransitionReason::kArmingCompleted;
		if (recovery_required_) {
			reason = ChassisTransitionReason::kRecoveryStarted;
			return ChassisControlState::kRecovery;
		}
		return ChassisControlState::kBalance;
	case ChassisControlState::kWaitingFeedback:
		if (!input.arm_complete || !input.dm_ready) {
			reason = ChassisTransitionReason::kDmNotReady;
			return ChassisControlState::kDmArming;
		}
		if (!input.feedback_valid) {
			return state_;
		}
		reason = ChassisTransitionReason::kFeedbackRecovered;
		if (recovery_required_) {
			reason = ChassisTransitionReason::kRecoveryStarted;
			return ChassisControlState::kRecovery;
		}
		return ChassisControlState::kBalance;
	case ChassisControlState::kRecovery:
	case ChassisControlState::kBalance:
	case ChassisControlState::kFlight:
	case ChassisControlState::kJump:
	case ChassisControlState::kClimbStairs:
		if (!input.arm_complete || !input.dm_ready) {
			reason = ChassisTransitionReason::kDmNotReady;
			return ChassisControlState::kDmArming;
		}
		if (!input.feedback_valid) {
			reason = ChassisTransitionReason::kFeedbackLost;
			return ChassisControlState::kWaitingFeedback;
		}
		if (state_ == ChassisControlState::kRecovery && recovery_failed_) {
			reason = ChassisTransitionReason::kRecoveryFailed;
			return ChassisControlState::kActionFault;
		}
		if (state_ == ChassisControlState::kRecovery && recovery_completed_) {
			reason = ChassisTransitionReason::kRecoveryCompleted;
			return ChassisControlState::kBalance;
		}
		if (state_ == ChassisControlState::kBalance && flight_detected_) {
			reason = ChassisTransitionReason::kFlightDetected;
			return ChassisControlState::kFlight;
		}
		if (state_ == ChassisControlState::kFlight && flight_failed_) {
			reason = ChassisTransitionReason::kFlightFailed;
			return ChassisControlState::kActionFault;
		}
		if (state_ == ChassisControlState::kFlight && flight_completed_) {
			reason = ChassisTransitionReason::kFlightLanded;
			return ChassisControlState::kBalance;
		}
		if (state_ == ChassisControlState::kBalance && jump_request_edge_ &&
		    jump_cooldown_remaining_s_ <= 0.0) {
			reason = ChassisTransitionReason::kJumpRequested;
			return ChassisControlState::kJump;
		}
		if (state_ == ChassisControlState::kJump && jump_failed_) {
			reason = ChassisTransitionReason::kJumpFailed;
			return ChassisControlState::kActionFault;
		}
		if (state_ == ChassisControlState::kJump && jump_completed_) {
			reason = ChassisTransitionReason::kJumpCompleted;
			return ChassisControlState::kBalance;
		}
		if (state_ == ChassisControlState::kBalance && climb_request_edge_) {
			reason = ChassisTransitionReason::kClimbRequested;
			return ChassisControlState::kClimbStairs;
		}
		if (state_ == ChassisControlState::kClimbStairs && climb_failed_) {
			reason = ChassisTransitionReason::kClimbFailed;
			return ChassisControlState::kActionFault;
		}
		if (state_ == ChassisControlState::kClimbStairs && climb_completed_) {
			reason = ChassisTransitionReason::kClimbCompleted;
			return ChassisControlState::kBalance;
		}
		return state_;
	case ChassisControlState::kActionFault:
	case ChassisControlState::kTiltFault:
		return state_;
	}

	// Corrupted or newly added unhandled states fail closed.
	return ChassisControlState::kSafetyStop;
}

} // namespace modules
