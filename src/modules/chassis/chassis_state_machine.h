/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "chassis_types.h"

namespace modules
{

struct ChassisStateTransition {
	bool enable_changed = false;
	bool enabled = false;
};

/** Owns safety latches and the chassis operating-state transition policy. */
class ChassisStateMachine
{
public:
	ChassisStateTransition Update(const ChassisCycleInput &input);
	void MarkBalanceReached() { balance_phase_reached_ = true; }
	void SetArmResetReason(DmArmResetReason reason) { arm_reset_reason_ = reason; }

	ChassisControlState state() const { return state_; }
	bool tilt_fault_latched() const { return tilt_fault_latched_; }
	DmArmResetReason arm_reset_reason() const { return arm_reset_reason_; }

private:
	bool last_requested_enable_ = false;
	bool tilt_fault_latched_ = false;
	bool balance_phase_reached_ = false;
	ChassisControlState state_ = ChassisControlState::kDisabled;
	DmArmResetReason arm_reset_reason_ = DmArmResetReason::kStartup;
};

} // namespace modules
