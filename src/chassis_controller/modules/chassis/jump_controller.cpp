/* SPDX-License-Identifier: Apache-2.0 */
#include "jump_controller.h"

#include <algorithm>
#include <cmath>

#include "chassis_config.h"
#include "leg_vmc.h"
#include "unified_lqr_schedule.h"

namespace modules
{
namespace
{
using namespace chassis_config;
constexpr size_t kStateCount = 10U;
constexpr size_t kOutputCount = 4U;
constexpr size_t kLeftTheta = 4U;
constexpr size_t kLeftThetaRate = 5U;
constexpr size_t kRightTheta = 6U;
constexpr size_t kRightThetaRate = 7U;
constexpr size_t kLeftWheel = 0U;
constexpr size_t kRightWheel = 1U;
constexpr size_t kLeftLegTorque = 2U;
constexpr size_t kRightLegTorque = 3U;

double MoveToward(double value, double target, double maximum_delta)
{
	return value + std::clamp(target - value, -maximum_delta, maximum_delta);
}
} // namespace

void JumpController::Reset()
{
	phase_ = JumpPhase::kInit;
	phase_elapsed_s_ = 0.0;
	total_elapsed_s_ = 0.0;
	air_retract_hold_s_ = 0.0;
	theta_reference_ = {};
	retract_reference_ = {};
	airborne_controller_.Reset();
}

void JumpController::EnterPhase(JumpPhase phase, const ChassisCycleInput &input)
{
	phase_ = phase;
	phase_elapsed_s_ = 0.0;
	if (phase == JumpPhase::kAirRetract) {
		retract_reference_ = {input.leg.left.length, input.leg.right.length};
		air_retract_hold_s_ = 0.0;
	} else if (phase == JumpPhase::kAir) {
		airborne_controller_.Reset();
	}
}

void JumpController::ComputeAttitudeControl(const ChassisCycleInput &input,
					    double actuator[kOutputCount]) const
{
	double state_error[kStateCount] = {};
	state_error[kLeftTheta] = input.theta.left - theta_reference_.left;
	state_error[kLeftThetaRate] = input.theta_rate.left;
	state_error[kRightTheta] = input.theta.right - theta_reference_.right;
	state_error[kRightThetaRate] = input.theta_rate.right;
	double gain[kOutputCount][kStateCount] = {};
	EvaluateUnifiedLqrGain(0.5 * (input.leg.left.length + input.leg.right.length), gain);
	for (size_t row = 0U; row < kOutputCount; ++row) {
		actuator[row] = 0.0;
		for (size_t column = kLeftTheta; column <= kRightThetaRate; ++column) {
			actuator[row] -= gain[row][column] * state_error[column];
		}
	}
}

void JumpController::ComputeVmcLeg(const ChassisCycleInput &input, Side side,
				   double target_length, double target_rate,
				   double support_force, double leg_torque,
				   ChassisControlOutput &output) const
{
	const LegKinematics &leg = input.leg.Get(side);
	const LegHardwareMap &hardware = kLegHardware.Get(side);
	const LegVmcOutput vmc = ComputeLegVmc(leg, target_length, support_force, 0.0,
		hardware.vmc_torque_sign * leg_torque, leg.length_rate, target_rate);
	output.leg_axial_force.Get(side) = vmc.axial_force;
	output.body_on_leg_torque.Get(side) = leg_torque;
	JointPair<double> &torque = output.joint_torque.Get(side);
	torque.b = std::clamp(vmc.phi2_torque, -kJointTorqueLimit, kJointTorqueLimit);
	torque.d = std::clamp(vmc.phi1_torque, -kJointTorqueLimit, kJointTorqueLimit);
}

JumpControllerOutput JumpController::Update(const ChassisCycleInput &input,
					    BalanceController &balance)
{
	const double dt = std::clamp(input.dt, 0.0005, 0.01);
	if (phase_ == JumpPhase::kInit) {
		theta_reference_ = {input.theta.left, input.theta.right};
		total_elapsed_s_ = 0.0;
		EnterPhase(JumpPhase::kPreCompress, input);
	}
	total_elapsed_s_ += dt;
	phase_elapsed_s_ += dt;
	JumpControllerOutput output = {};

	if (phase_ == JumpPhase::kPreCompress) {
		balance.set_target_leg_length(kJumpPrecompressLegLength);
		balance.Update(input, output.control);
		const auto ready = [&](Side side) {
			const LegKinematics &leg = input.leg.Get(side);
			return std::abs(leg.length - kJumpPrecompressLegLength) <= kJumpPrecompressLengthTolerance &&
			       std::abs(leg.length_rate) <= kJumpPrecompressSpeedTolerance;
		};
		if ((ready(Side::kLeft) && ready(Side::kRight)) ||
		    phase_elapsed_s_ >= static_cast<double>(kJumpPrecompressTimeoutMs) * 0.001) {
			EnterPhase(JumpPhase::kThrust, input);
		}
	} else if (phase_ == JumpPhase::kThrust) {
		double actuator[kOutputCount] = {};
		ComputeAttitudeControl(input, actuator);
		output.control.physical_wheel_torque = {actuator[kLeftWheel], actuator[kRightWheel]};
		ComputeVmcLeg(input, Side::kLeft, input.leg.left.length, 0.0, 0.0,
			      actuator[kLeftLegTorque], output.control);
		ComputeVmcLeg(input, Side::kRight, input.leg.right.length, 0.0, 0.0,
			      actuator[kRightLegTorque], output.control);
		for (Side side : {Side::kLeft, Side::kRight}) {
			JointPair<double> &torque = output.control.joint_torque.Get(side);
			// The five-bar mechanisms are mirrored. Apply the configured joint
			// direction in the local kinematic branch so positive jump torque
			// extends both legs instead of extending only the left side.
			const double mirror_sign =
				static_cast<double>(kLegHardware.Get(side).kinematic_branch);
			torque.b = std::clamp(torque.b + mirror_sign * kJumpJointBThrustSign *
						       kJumpThrustTorque,
					      -kJointTorqueLimit, kJointTorqueLimit);
			torque.d = std::clamp(torque.d + mirror_sign * kJumpJointDThrustSign *
						       kJumpThrustTorque,
					      -kJointTorqueLimit, kJointTorqueLimit);
		}
		if (input.leg.left.length >= kJumpThrustExitLegLength &&
		    input.leg.right.length >= kJumpThrustExitLegLength) {
			EnterPhase(JumpPhase::kAirRetract, input);
		} else if (phase_elapsed_s_ >= static_cast<double>(kJumpThrustTimeoutMs) * 0.001) {
			EnterPhase(JumpPhase::kFailed, input);
		}
	} else if (phase_ == JumpPhase::kAirRetract) {
		double actuator[kOutputCount] = {};
		ComputeAttitudeControl(input, actuator);
		const double maximum_delta = kJumpAirRetractRate * dt;
		retract_reference_.left = MoveToward(retract_reference_.left, kJumpAirRetractLegLength, maximum_delta);
		retract_reference_.right = MoveToward(retract_reference_.right, kJumpAirRetractLegLength, maximum_delta);
		const double left_rate = retract_reference_.left > kJumpAirRetractLegLength
					 ? -kJumpAirRetractRate : 0.0;
		const double right_rate = retract_reference_.right > kJumpAirRetractLegLength
					  ? -kJumpAirRetractRate : 0.0;
		ComputeVmcLeg(input, Side::kLeft, retract_reference_.left, left_rate,
			      kJumpAirRetractGravityFeedforward, actuator[kLeftLegTorque], output.control);
		ComputeVmcLeg(input, Side::kRight, retract_reference_.right, right_rate,
			      kJumpAirRetractGravityFeedforward, actuator[kRightLegTorque], output.control);
		const bool retracted =
			std::abs(input.leg.left.length - kJumpAirRetractLegLength) <= kJumpAirRetractLengthTolerance &&
			std::abs(input.leg.right.length - kJumpAirRetractLegLength) <= kJumpAirRetractLengthTolerance &&
			std::abs(input.leg.left.length_rate) <= kJumpPrecompressSpeedTolerance &&
			std::abs(input.leg.right.length_rate) <= kJumpPrecompressSpeedTolerance;
		air_retract_hold_s_ = retracted ? air_retract_hold_s_ + dt : 0.0;
		if (air_retract_hold_s_ >= static_cast<double>(kJumpAirRetractHoldMs) * 0.001) {
			EnterPhase(JumpPhase::kAir, input);
		} else if (phase_elapsed_s_ >= static_cast<double>(kJumpAirRetractTimeoutMs) * 0.001) {
			EnterPhase(JumpPhase::kFailed, input);
		}
	} else if (phase_ == JumpPhase::kAir) {
		const FlightControllerOutput airborne = airborne_controller_.Update(input);
		output.control = airborne.control;
		if (airborne.failed) {
			EnterPhase(JumpPhase::kFailed, input);
		} else if (airborne.finished) {
			EnterPhase(JumpPhase::kReturn, input);
		}
	}

	if (phase_ != JumpPhase::kReturn && phase_ != JumpPhase::kFailed &&
	    total_elapsed_s_ >= static_cast<double>(kJumpTotalTimeoutMs) * 0.001) {
		EnterPhase(JumpPhase::kFailed, input);
	}
	output.phase = phase_;
	output.finished = phase_ == JumpPhase::kReturn;
	output.failed = phase_ == JumpPhase::kFailed;
	return output;
}
} // namespace modules
