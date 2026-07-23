/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <array>
#include <cstdint>

#include <modules/chassis/leg_kinematics.h>

namespace modules {

struct StoolControllerInput {
	double pitch = 0.0;
	double dt = 0.001;
	std::array<LegKinematics, 2> leg{};
	std::array<double, 4> joint_position{};
	std::array<double, 4> joint_velocity{};
};

struct StoolControllerOutput {
	std::array<double, 4> joint_torque{};
	bool target_initialized = false;
	bool ready = false;
};

class StoolController {
public:
	static constexpr double kTargetLegLength = 0.150;

	void Reset();
	StoolControllerOutput Update(const StoolControllerInput &input);

private:
	struct JointPidState {
		double position_integral = 0.0;
		double speed_integral = 0.0;
	};

	void UpdateJointVelocity(const std::array<double, 4> &velocity, double dt);
	void UpdatePoseTarget(const StoolControllerInput &input);
	static double RawKinematicAngle(uint8_t side, double normalized_angle);
	double ComputeJointTorque(uint8_t index, double target, double position,
				  double velocity, double dt);

	std::array<bool, 2> target_initialized_{};
	bool reference_initialized_ = false;
	bool velocity_initialized_ = false;
	double filtered_pitch_ = 0.0;
	double alpha_reference_ = 0.0;
	std::array<double, 4> joint_target_{};
	std::array<double, 4> filtered_joint_velocity_{};
	std::array<JointPidState, 4> joint_pid_{};
};

}  // namespace modules
