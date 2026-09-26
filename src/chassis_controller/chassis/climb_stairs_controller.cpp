/* SPDX-License-Identifier: Apache-2.0 */

#include "climb_stairs_controller.h"

#include <algorithm>
#include <cmath>

#include "chassis_config.h"

namespace
{
using namespace modules::chassis_config;

constexpr int kBranch[2] = {kLeftKinematicBranch, kRightKinematicBranch};
constexpr double kLegAngleOffset[2] = {kLeftLegAngleOffset, kRightLegAngleOffset};

double MoveToward(double current, double target, double maximum_delta)
{
	return current + std::clamp(target - current, -maximum_delta, maximum_delta);
}

double MoveTowardAngle(double current, double target, double maximum_delta)
{
	const double error = std::remainder(target - current, kTwoPi);
	return std::remainder(current + std::clamp(error, -maximum_delta, maximum_delta),
			      kTwoPi);
}
} // namespace

namespace modules
{

void ClimbStairsController::Reset()
{
	phase_ = ClimbStairsPhase::kInit;
	phase_elapsed_s_ = 0.0;
	ready_elapsed_s_ = 0.0;
	reference_initialized_ = false;
	target_initialized_.fill(false);
	leg_length_reference_.fill(0.0);
	leg_angle_reference_.fill(0.0);
	joint_target_.fill(0.0);
	joint_pid_.fill({});
}

void ClimbStairsController::EnterPhase(ClimbStairsPhase phase)
{
	phase_ = phase;
	phase_elapsed_s_ = 0.0;
	ready_elapsed_s_ = 0.0;
}

ClimbStairsControllerOutput ClimbStairsController::Update(
	const ClimbStairsControllerInput &input)
{
	const double dt = std::clamp(input.dt, 0.0005, 0.01);
	phase_elapsed_s_ += dt;
	if (!reference_initialized_) {
		for (size_t side = 0U; side < input.leg.size(); ++side) {
			leg_length_reference_[side] = input.leg[side].length;
			leg_angle_reference_[side] = input.leg[side].angle;
		}
		reference_initialized_ = true;
		EnterPhase(ClimbStairsPhase::kMotion1);
	}

	UpdateReferences(input);
	const bool targets_valid = UpdateJointTargets(input);

	ClimbStairsControllerOutput output = {};
	output.phase = phase_;
	if (targets_valid) {
		for (uint8_t joint = 0U; joint < output.joint_torque.size(); ++joint) {
			output.joint_torque[joint] = ComputeJointTorque(
				joint, joint_target_[joint], input.joint_position[joint],
				input.joint_velocity[joint], dt);
		}
	}
	output.finished = phase_ == ClimbStairsPhase::kReturn;
	output.failed = phase_ == ClimbStairsPhase::kFailed;
	return output;
}

void ClimbStairsController::UpdateReferences(const ClimbStairsControllerInput &input)
{
	using namespace chassis_config;
	const double dt = std::clamp(input.dt, 0.0005, 0.01);
	const double ready_hold_s = static_cast<double>(kClimbReadyHoldMs) * 0.001;

	if (phase_ == ClimbStairsPhase::kMotion1) {
		for (size_t side = 0U; side < input.leg.size(); ++side) {
			leg_length_reference_[side] = MoveToward(
				leg_length_reference_[side], kClimbIntermediateLegLength,
				kClimbLegLengthRate * dt);
			leg_angle_reference_[side] = MoveTowardAngle(
				leg_angle_reference_[side], input.pitch + kClimbSwingTheta,
				kClimbMotion1AngleRateDeg * kDegToRad * dt);
		}
		const bool ready =
			std::abs(input.leg[0].length - kClimbIntermediateLegLength) <
				kClimbLengthTolerance &&
			std::abs(input.leg[1].length - kClimbIntermediateLegLength) <
				kClimbLengthTolerance &&
			std::abs(std::remainder(input.leg[0].angle - input.pitch -
						kClimbSwingTheta, kTwoPi)) <
				kClimbMotion1AngleToleranceDeg * kDegToRad &&
			std::abs(std::remainder(input.leg[1].angle - input.pitch -
						kClimbSwingTheta, kTwoPi)) <
				kClimbMotion1AngleToleranceDeg * kDegToRad;
		ready_elapsed_s_ = ready ? ready_elapsed_s_ + dt : 0.0;
		if (ready_elapsed_s_ >= ready_hold_s) {
			EnterPhase(ClimbStairsPhase::kMotion2);
		} else if (phase_elapsed_s_ >=
			   static_cast<double>(kClimbMotion1TimeoutMs) * 0.001) {
			EnterPhase(ClimbStairsPhase::kFailed);
		}
	} else if (phase_ == ClimbStairsPhase::kMotion2) {
		for (size_t side = 0U; side < input.leg.size(); ++side) {
			leg_length_reference_[side] = MoveToward(
				leg_length_reference_[side], kClimbRetractedLegLength,
				kClimbLegLengthRate * dt);
			leg_angle_reference_[side] = MoveTowardAngle(
				leg_angle_reference_[side], input.pitch,
				kClimbMotion2AngleRateDeg * kDegToRad * dt);
		}
		const bool ready =
			std::abs(input.leg[0].length - kClimbRetractedLegLength) <
				kClimbLengthTolerance &&
			std::abs(input.leg[1].length - kClimbRetractedLegLength) <
				kClimbLengthTolerance &&
			std::abs(std::remainder(input.leg[0].angle - input.pitch, kTwoPi)) <
				kClimbMotion2AngleToleranceDeg * kDegToRad &&
			std::abs(std::remainder(input.leg[1].angle - input.pitch, kTwoPi)) <
				kClimbMotion2AngleToleranceDeg * kDegToRad;
		ready_elapsed_s_ = ready ? ready_elapsed_s_ + dt : 0.0;
		if (ready_elapsed_s_ >= ready_hold_s) {
			EnterPhase(ClimbStairsPhase::kReturn);
		} else if (phase_elapsed_s_ >=
			   static_cast<double>(kClimbMotion2TimeoutMs) * 0.001) {
			EnterPhase(ClimbStairsPhase::kFailed);
		}
	}
}

bool ClimbStairsController::UpdateJointTargets(const ClimbStairsControllerInput &input)
{
	bool all_valid = true;
	for (uint8_t side = 0U; side < input.leg.size(); ++side) {
		const size_t base = 2U * side;
		const double raw_angle = RawKinematicAngle(side, leg_angle_reference_[side]);
		const double target_hx = leg_length_reference_[side] * std::sin(raw_angle);
		const double target_hz = -leg_length_reference_[side] * std::cos(raw_angle);
		const double seed_phi1 = target_initialized_[side]
						 ? joint_target_[base + 1U]
						 : input.joint_position[base + 1U];
		const double seed_phi2 = target_initialized_[side]
						 ? joint_target_[base]
						 : input.joint_position[base];
		double target_phi1 = 0.0;
		double target_phi2 = 0.0;
		if (!InverseKinematics(target_hx, target_hz, kBranch[side], seed_phi1,
				       seed_phi2, target_phi1, target_phi2)) {
			all_valid = false;
			continue;
		}
		joint_target_[base] = target_phi2;
		joint_target_[base + 1U] = target_phi1;
		target_initialized_[side] = true;
	}
	return all_valid && target_initialized_[0] && target_initialized_[1];
}

double ClimbStairsController::RawKinematicAngle(uint8_t side, double normalized_angle)
{
	return side == 0U
		       ? std::remainder(kLegAngleOffset[0] - normalized_angle, kTwoPi)
		       : std::remainder(normalized_angle + kLegAngleOffset[1], kTwoPi);
}

double ClimbStairsController::ComputeJointTorque(uint8_t index, double target,
						 double position, double velocity, double dt)
{
	using namespace chassis_config;
	if (index >= joint_pid_.size() || !std::isfinite(target) ||
	    !std::isfinite(position) || !std::isfinite(velocity)) {
		return 0.0;
	}
	JointPidState &pid = joint_pid_[index];
	const double position_error = std::remainder(target - position, kTwoPi);
	pid.position_integral = std::clamp(
		pid.position_integral + position_error * dt, -kJointPositionMaxSpeed,
		kJointPositionMaxSpeed);
	const double speed_target = std::clamp(
		kJointPositionKp * position_error + kJointPositionKi * pid.position_integral,
		-kJointPositionMaxSpeed, kJointPositionMaxSpeed);
	const double speed_error = speed_target - velocity;
	pid.speed_integral = std::clamp(
		pid.speed_integral + speed_error * dt, -kJointIntegralTorqueLimit,
		kJointIntegralTorqueLimit);
	return std::clamp(kJointSpeedKp * speed_error + kJointSpeedKi * pid.speed_integral,
			  -kJointTorqueMax, kJointTorqueMax);
}

} // namespace modules
