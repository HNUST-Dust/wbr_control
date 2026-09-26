/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <array>
#include <cstdint>

#include "leg_kinematics.h"

namespace modules
{

enum class RecoveryPhase : uint8_t {
	kInit = 0U,
	kTurnOver,
	kYawFront,
	kSwing,
	kDrawBack,
	kReturn,
	kFailed,
};

struct RecoveryGimbalStatus {
	bool available = false;
	bool safe_for_leg_motion = true;
	bool aligned = true;
};

struct RecoveryControllerInput {
	double dt = 0.001;
	double pitch = 0.0;
	double roll = 0.0;
	double vertical_accel_g = 0.0;
	std::array<LegKinematics, 2> leg{};
	std::array<double, 4> joint_position{};
	std::array<double, 4> joint_velocity{};
	RecoveryGimbalStatus gimbal{};
};

struct RecoveryControllerOutput {
	std::array<double, 4> joint_torque{};
	RecoveryPhase phase = RecoveryPhase::kInit;
	bool request_gimbal_disable = false;
	bool finished = false;
	bool failed = false;
};

class RecoveryController
{
public:
	void Reset();
	RecoveryControllerOutput Update(const RecoveryControllerInput &input);
	RecoveryPhase phase() const { return phase_; }

private:
	enum class SwingEscapePhase : uint8_t { kNone = 0U, kBackToVertical, kToFinal };
	struct JointPidState { double position_integral = 0.0; double speed_integral = 0.0; };

	void EnterPhase(RecoveryPhase phase, const RecoveryControllerInput &input);
	void UpdateReferences(const RecoveryControllerInput &input);
	bool UpdateJointTargets(const RecoveryControllerInput &input);
	double ComputeJointTorque(uint8_t index, double target, double position,
				  double velocity, double dt);
	static double RawKinematicAngle(uint8_t side, double normalized_angle);

	RecoveryPhase phase_ = RecoveryPhase::kInit;
	double phase_elapsed_s_ = 0.0;
	double ready_elapsed_s_ = 0.0;
	double upright_elapsed_s_ = 0.0;
	double turnover_stuck_elapsed_s_ = 0.0;
	double turnover_progress_ = 0.0;
	int turnover_direction_ = 1;
	uint32_t turnover_reversals_ = 0U;
	bool reference_initialized_ = false;
	std::array<double, 2> phase_start_angle_{};
	std::array<double, 2> leg_length_reference_{};
	std::array<double, 2> leg_angle_reference_{};
	std::array<double, 2> swing_stuck_elapsed_s_{};
	std::array<SwingEscapePhase, 2> swing_escape_phase_{};
	std::array<bool, 2> target_initialized_{};
	std::array<double, 4> joint_target_{};
	std::array<JointPidState, 4> joint_pid_{};
};

} // namespace modules
