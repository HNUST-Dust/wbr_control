/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "stool_controller.h"

#include <algorithm>
#include <cmath>

namespace
{

constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kDegToRad = 0.01745329251994329577;
constexpr double kJointVelocityFilterCutoffHz = 35.0;
constexpr double kReadyThetaTolerance = 5.0 * kDegToRad;

constexpr double kJointPositionKp = 20.0;
constexpr double kJointPositionKi = 0.0;
constexpr double kJointPositionMaxSpeed = 44.0;
constexpr double kJointSpeedKp = 4.0;
constexpr double kJointSpeedKi = 0.0;
constexpr double kJointIntegralTorqueLimit = 20.0;
constexpr double kJointTorqueMax = 54.0;

constexpr int kBranch[2] = {1, -1};
constexpr double kLegAngleOffset[2] = {-0.036063, -3.121010};

} // namespace

namespace modules
{

void StoolController::Reset()
{
	target_initialized_.fill(false);
	velocity_initialized_ = false;
	joint_target_.fill(0.0);
	filtered_joint_velocity_.fill(0.0);
	joint_pid_.fill({});
}

StoolControllerOutput StoolController::Update(const StoolControllerInput &input)
{
	UpdateJointVelocity(input.joint_velocity, input.dt);
	UpdatePoseTarget(input);

	StoolControllerOutput output = {};
	output.target_initialized =
		std::all_of(target_initialized_.begin(), target_initialized_.end(),
			    [](bool value) { return value; });
	if (!output.target_initialized) {
		return output;
	}

	for (uint8_t joint = 0U; joint < output.joint_torque.size(); ++joint) {
		output.joint_torque[joint] =
			ComputeJointTorque(joint, joint_target_[joint], input.joint_position[joint],
					   filtered_joint_velocity_[joint], input.dt);
	}
	output.ready = std::all_of(
		input.leg.begin(), input.leg.end(), [pitch = input.pitch](const LegKinematics &leg) {
			const double theta = std::remainder(leg.angle - pitch, kTwoPi);
			return std::abs(theta) <= kReadyThetaTolerance;
		});
	return output;
}

void StoolController::UpdateJointVelocity(const std::array<double, 4> &velocity, double dt)
{
	const double time_constant = 1.0 / (kTwoPi * kJointVelocityFilterCutoffHz);
	const double alpha = std::clamp(dt / (time_constant + dt), 0.0, 1.0);
	for (size_t joint = 0U; joint < velocity.size(); ++joint) {
		if (!velocity_initialized_) {
			filtered_joint_velocity_[joint] = velocity[joint];
		} else {
			filtered_joint_velocity_[joint] +=
				alpha * (velocity[joint] - filtered_joint_velocity_[joint]);
		}
	}
	velocity_initialized_ = true;
}

void StoolController::UpdatePoseTarget(const StoolControllerInput &input)
{
	for (uint8_t side = 0U; side < input.leg.size(); ++side) {
		const size_t base = 2U * side;
		/*
		 * LQR 使用 theta = alpha - pitch。撑起阶段直接令 theta_target = 0，
		 * 因而腿相对机体的实时目标为 alpha_target = pitch。
		 */
		const double raw_angle = RawKinematicAngle(side, input.pitch);
		const double target_hx = kTargetLegLength * std::sin(raw_angle);
		const double target_hz = -kTargetLegLength * std::cos(raw_angle);
		const double seed_phi1 = target_initialized_[side]
						 ? joint_target_[base + 1U]
						 : input.joint_position[base + 1U];
		const double seed_phi2 = target_initialized_[side] ? joint_target_[base]
								   : input.joint_position[base];
		double target_phi1 = 0.0;
		double target_phi2 = 0.0;
		if (!InverseKinematics(target_hx, target_hz, kBranch[side], seed_phi1, seed_phi2,
				       target_phi1, target_phi2)) {
			continue;
		}

		joint_target_[base] = target_phi2;
		joint_target_[base + 1U] = target_phi1;
		target_initialized_[side] = true;
	}
}

double StoolController::RawKinematicAngle(uint8_t side, double normalized_angle)
{
	return side == 0U ? std::remainder(kLegAngleOffset[0] - normalized_angle, kTwoPi)
			  : std::remainder(normalized_angle + kLegAngleOffset[1], kTwoPi);
}

double StoolController::ComputeJointTorque(uint8_t index, double target, double position,
					   double velocity, double dt)
{
	if (index >= joint_pid_.size() || !std::isfinite(target) || !std::isfinite(position) ||
	    !std::isfinite(velocity)) {
		return 0.0;
	}

	JointPidState &pid = joint_pid_[index];
	const double bounded_dt = std::clamp(dt, 0.0005, 0.01);
	const double position_error = std::remainder(target - position, kTwoPi);
	pid.position_integral = std::clamp(pid.position_integral + position_error * bounded_dt,
					   -kJointPositionMaxSpeed, kJointPositionMaxSpeed);
	const double speed_target = std::clamp(kJointPositionKp * position_error +
						       kJointPositionKi * pid.position_integral,
					       -kJointPositionMaxSpeed, kJointPositionMaxSpeed);

	const double speed_error = speed_target - velocity;
	pid.speed_integral = std::clamp(pid.speed_integral + speed_error * bounded_dt,
					-kJointIntegralTorqueLimit, kJointIntegralTorqueLimit);
	return std::clamp(kJointSpeedKp * speed_error + kJointSpeedKi * pid.speed_integral,
			  -kJointTorqueMax, kJointTorqueMax);
}

} // namespace modules
