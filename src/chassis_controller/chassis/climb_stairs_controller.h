/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <array>
#include <cstdint>

#include "leg_kinematics.h"

namespace modules
{

enum class ClimbStairsPhase : uint8_t {
	kInit = 0U,
	kMotion1,
	kMotion2,
	kReturn,
	kFailed,
};

struct ClimbStairsControllerInput {
	double dt = 0.001;
	double pitch = 0.0;
	std::array<LegKinematics, 2> leg{};
	std::array<double, 4> joint_position{};
	std::array<double, 4> joint_velocity{};
};

struct ClimbStairsControllerOutput {
	std::array<double, 4> joint_torque{};
	ClimbStairsPhase phase = ClimbStairsPhase::kInit;
	bool finished = false;
	bool failed = false;
};

/** Executes the atomic Init -> Motion1 -> Motion2 -> Return stair-climb action. */
class ClimbStairsController
{
public:
	void Reset();
	ClimbStairsControllerOutput Update(const ClimbStairsControllerInput &input);
	ClimbStairsPhase phase() const { return phase_; }

private:
	struct JointPidState {
		double position_integral = 0.0;
		double speed_integral = 0.0;
	};

	void EnterPhase(ClimbStairsPhase phase);
	void UpdateReferences(const ClimbStairsControllerInput &input);
	bool UpdateJointTargets(const ClimbStairsControllerInput &input);
	double ComputeJointTorque(uint8_t index, double target, double position,
				  double velocity, double dt);
	static double RawKinematicAngle(uint8_t side, double normalized_angle);

	ClimbStairsPhase phase_ = ClimbStairsPhase::kInit;
	double phase_elapsed_s_ = 0.0;
	double ready_elapsed_s_ = 0.0;
	bool reference_initialized_ = false;
	std::array<bool, 2> target_initialized_{};
	std::array<double, 2> leg_length_reference_{};
	std::array<double, 2> leg_angle_reference_{};
	std::array<double, 4> joint_target_{};
	std::array<JointPidState, 4> joint_pid_{};
};

} // namespace modules
