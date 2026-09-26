/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "chassis_types.h"

namespace modules
{

enum class ChassisTransitionReason : uint8_t {
	kNone = 0U,
	kRemoteDisabled,
	kRemoteTimeout,
	kRemoteEnabled,
	kTiltExceeded,
	kImuLost,
	kImuRecovered,
	kDmNotReady,
	kArmingCompleted,
	kFeedbackLost,
	kFeedbackRecovered,
	kRecoveryStarted,
	kRecoveryCompleted,
	kRecoveryFailed,
	kFlightDetected,
	kFlightLanded,
	kFlightFailed,
	kJumpRequested,
	kJumpCompleted,
	kJumpFailed,
	kClimbRequested,
	kClimbCompleted,
	kClimbFailed,
};

struct ChassisStateTransition {
	ChassisControlState from = ChassisControlState::kDisabled;
	ChassisControlState to = ChassisControlState::kDisabled;
	ChassisTransitionReason reason = ChassisTransitionReason::kNone;
	bool changed = false;
	bool enable_changed = false;
	bool enabled = false;
};

/** Owns safety latches and the chassis operating-state transition policy. */
class ChassisStateMachine
{
public:
	ChassisStateTransition Update(const ChassisCycleInput &input);
	void MarkRecoveryCompleted() { recovery_completed_ = true; }
	void MarkRecoveryFailed() { recovery_failed_ = true; }
	void MarkClimbCompleted() { climb_completed_ = true; }
	void MarkClimbFailed() { climb_failed_ = true; }
	void SetFlightDetected(bool detected) { flight_detected_ = detected; }
	void MarkFlightCompleted() { flight_completed_ = true; }
	void MarkFlightFailed() { flight_failed_ = true; }
	void MarkJumpCompleted() { jump_completed_ = true; }
	void MarkJumpFailed() { jump_failed_ = true; }
	void SetArmResetReason(DmArmResetReason reason) { arm_reset_reason_ = reason; }

	ChassisControlState state() const { return state_; }
	bool tilt_fault_latched() const { return tilt_fault_latched_; }
	DmArmResetReason arm_reset_reason() const { return arm_reset_reason_; }

private:
	ChassisControlState DetermineNextState(const ChassisCycleInput &input,
					       ChassisTransitionReason &reason) const;
	void UpdateSafetyLatches(const ChassisCycleInput &input);

	bool last_requested_enable_ = false;
	bool tilt_fault_latched_ = false;
	bool recovery_required_ = false;
	bool recovery_completed_ = false;
	bool recovery_failed_ = false;
	bool last_climb_request_ = false;
	bool climb_request_edge_ = false;
	bool climb_completed_ = false;
	bool climb_failed_ = false;
	bool flight_detected_ = false;
	bool flight_completed_ = false;
	bool flight_failed_ = false;
	bool last_jump_request_ = false;
	bool jump_request_edge_ = false;
	bool jump_completed_ = false;
	bool jump_failed_ = false;
	double jump_cooldown_remaining_s_ = 0.0;
	bool action_fault_latched_ = false;
	ChassisControlState state_ = ChassisControlState::kDisabled;
	DmArmResetReason arm_reset_reason_ = DmArmResetReason::kStartup;
};

} // namespace modules
