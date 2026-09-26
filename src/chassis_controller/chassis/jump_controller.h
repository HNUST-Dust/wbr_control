/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include "balance_controller.h"
#include "chassis_types.h"
#include "flight_controller.h"

namespace modules
{
enum class JumpPhase : uint8_t { kInit = 0U, kPreCompress, kThrust, kAirRetract, kAir, kReturn, kFailed };

struct JumpControllerOutput {
	ChassisControlOutput control;
	JumpPhase phase = JumpPhase::kInit;
	bool finished = false;
	bool failed = false;
};

class JumpController
{
public:
	void Reset();
	JumpControllerOutput Update(const ChassisCycleInput &input, BalanceController &balance);
	JumpPhase phase() const { return phase_; }

private:
	void EnterPhase(JumpPhase phase, const ChassisCycleInput &input);
	void ComputeAttitudeControl(const ChassisCycleInput &input, double actuator[4]) const;
	void ComputeVmcLeg(const ChassisCycleInput &input, Side side, double target_length,
			   double target_rate, double support_force, double leg_torque,
			   ChassisControlOutput &output) const;
	JumpPhase phase_ = JumpPhase::kInit;
	double phase_elapsed_s_ = 0.0;
	double total_elapsed_s_ = 0.0;
	double air_retract_hold_s_ = 0.0;
	SidePair<double> theta_reference_;
	SidePair<double> retract_reference_;
	FlightController airborne_controller_;
};
} // namespace modules
