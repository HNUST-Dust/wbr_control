/* SPDX-License-Identifier: Apache-2.0 */

#include "recovery_controller.h"

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

void RecoveryController::Reset()
{
	phase_ = RecoveryPhase::kInit;
	phase_elapsed_s_ = ready_elapsed_s_ = upright_elapsed_s_ = 0.0;
	turnover_stuck_elapsed_s_ = turnover_progress_ = 0.0;
	turnover_direction_ = 1;
	turnover_reversals_ = 0U;
	reference_initialized_ = false;
	phase_start_angle_.fill(0.0);
	leg_length_reference_.fill(0.0);
	leg_angle_reference_.fill(0.0);
	swing_stuck_elapsed_s_.fill(0.0);
	swing_escape_phase_.fill(SwingEscapePhase::kNone);
	target_initialized_.fill(false);
	joint_target_.fill(0.0);
	joint_pid_.fill({});
}

void RecoveryController::EnterPhase(RecoveryPhase phase,
				    const RecoveryControllerInput &input)
{
	phase_ = phase;
	phase_elapsed_s_ = ready_elapsed_s_ = 0.0;
	for (size_t side = 0U; side < input.leg.size(); ++side) {
		phase_start_angle_[side] = input.leg[side].angle;
	}
}

RecoveryControllerOutput RecoveryController::Update(const RecoveryControllerInput &input)
{
	const double dt = std::clamp(input.dt, 0.0005, 0.01);
	phase_elapsed_s_ += dt;
	if (!reference_initialized_) {
		for (size_t side = 0U; side < input.leg.size(); ++side) {
			leg_length_reference_[side] = input.leg[side].length;
			leg_angle_reference_[side] = input.leg[side].angle;
		}
		reference_initialized_ = true;
	}
	UpdateReferences(input);
	const bool targets_valid = UpdateJointTargets(input);
	RecoveryControllerOutput output = {};
	output.phase = phase_;
	output.request_gimbal_disable = phase_ != RecoveryPhase::kReturn;
	if (targets_valid) {
		for (uint8_t joint = 0U; joint < output.joint_torque.size(); ++joint) {
			output.joint_torque[joint] = ComputeJointTorque(
				joint, joint_target_[joint], input.joint_position[joint],
				input.joint_velocity[joint], dt);
		}
	}
	output.finished = phase_ == RecoveryPhase::kReturn;
	output.failed = phase_ == RecoveryPhase::kFailed;
	return output;
}

void RecoveryController::UpdateReferences(const RecoveryControllerInput &input)
{
	using namespace chassis_config;
	const double dt = std::clamp(input.dt, 0.0005, 0.01);
	const double ready_hold = static_cast<double>(kRecoveryReadyHoldMs) * 0.001;
	const double pitch_limit = 0.45;
	const double roll_limit = 20.0 * kDegToRad;
	// Normalize the installed IMU direction so a positive value always means
	// gravity points along the configured upright chassis direction.
	const double upright_accel_g = kRecoveryUprightAccelSign * input.vertical_accel_g;

	if (phase_ == RecoveryPhase::kInit) {
		if (!std::isfinite(input.vertical_accel_g) ||
		    std::abs(input.vertical_accel_g) < 0.05) {
			EnterPhase(RecoveryPhase::kFailed, input);
		} else if (upright_accel_g > kRecoveryUprightAccelThresholdG) {
			// Already upright: only retract and align the legs. Entering Swing here
			// can turn an ordinary enable operation into an unnecessary recovery kick.
			EnterPhase(RecoveryPhase::kDrawBack, input);
		} else {
			EnterPhase(RecoveryPhase::kTurnOver, input);
		}
	} else if (phase_ == RecoveryPhase::kTurnOver) {
		const bool upright_detected = upright_accel_g > kRecoveryUprightAccelThresholdG;
		for (size_t side = 0U; side < input.leg.size(); ++side) {
			leg_length_reference_[side] = MoveToward(
				leg_length_reference_[side], kRecoveryMaximumLegLength,
				kRecoveryExtendLegRate * dt);
		}
		// Freeze the sweep command as soon as upright is detected. The short
		// confirmation window then rejects a one-sample impact spike without
		// commanding another 18 degrees of motion as the previous 100 ms hold did.
		if (!upright_detected) {
			turnover_progress_ += turnover_direction_ *
				kRecoveryTurnoverSweepRateDeg * kDegToRad * dt;
		}
		for (size_t side = 0U; side < input.leg.size(); ++side) {
			leg_angle_reference_[side] = phase_start_angle_[side] + turnover_progress_;
		}
		const bool legs_extended = input.leg[0].length > 0.30 &&
			input.leg[1].length > 0.30;
		const bool stopped = !upright_detected && legs_extended &&
			(std::abs(input.leg[0].angle_rate) <
					     kRecoveryTurnoverStuckSpeedDeg * kDegToRad ||
				     std::abs(input.leg[1].angle_rate) <
					     kRecoveryTurnoverStuckSpeedDeg * kDegToRad);
		turnover_stuck_elapsed_s_ = stopped ? turnover_stuck_elapsed_s_ + dt : 0.0;
		if (turnover_stuck_elapsed_s_ >=
		    static_cast<double>(kRecoveryStuckTimeoutMs) * 0.001) {
			turnover_stuck_elapsed_s_ = 0.0;
			turnover_progress_ = 0.0;
			turnover_direction_ = -turnover_direction_;
			for (size_t side = 0U; side < input.leg.size(); ++side) {
				phase_start_angle_[side] = input.leg[side].angle;
			}
			if (++turnover_reversals_ > kRecoveryMaximumTurnoverReversals) {
				EnterPhase(RecoveryPhase::kFailed, input);
			}
		}
		upright_elapsed_s_ = upright_detected
					? upright_elapsed_s_ + dt
					: upright_elapsed_s_ * 0.5;
		if (upright_elapsed_s_ >= static_cast<double>(kRecoveryUprightHoldMs) * 0.001) {
			// Without a gimbal there is no additional orientation constraint to
			// satisfy; retracting immediately also arrests residual turnover motion.
			EnterPhase(input.gimbal.available ? RecoveryPhase::kYawFront
						 : RecoveryPhase::kDrawBack, input);
		} else if (phase_elapsed_s_ >=
			   static_cast<double>(kRecoveryTurnoverTimeoutMs) * 0.001) {
			EnterPhase(RecoveryPhase::kFailed, input);
		}
	} else if (phase_ == RecoveryPhase::kYawFront) {
		if (!input.gimbal.available ||
		    (input.gimbal.aligned && input.gimbal.safe_for_leg_motion)) {
			EnterPhase(RecoveryPhase::kSwing, input);
		}
	} else if (phase_ == RecoveryPhase::kSwing) {
		bool all_ready = true;
		for (size_t side = 0U; side < input.leg.size(); ++side) {
			const double swing_target = input.pitch + kRecoverySwingTheta;
			const bool stuck = input.leg[side].length > 0.30 &&
				std::abs(input.leg[side].angle_rate) <
					kRecoverySwingStuckSpeedDeg * kDegToRad &&
				std::abs(std::remainder(input.leg[side].angle - swing_target,
							kTwoPi)) > 10.0 * kDegToRad;
			swing_stuck_elapsed_s_[side] = stuck
				? swing_stuck_elapsed_s_[side] + dt : 0.0;
			if (swing_stuck_elapsed_s_[side] >=
			    static_cast<double>(kRecoveryStuckTimeoutMs) * 0.001) {
				swing_escape_phase_[side] = SwingEscapePhase::kBackToVertical;
				swing_stuck_elapsed_s_[side] = 0.0;
			}
			if (swing_escape_phase_[side] == SwingEscapePhase::kBackToVertical) {
				leg_length_reference_[side] = MoveToward(
					leg_length_reference_[side], kRecoveryMinimumLegLength,
					kRecoveryEscapeLegRate * dt);
				leg_angle_reference_[side] = MoveTowardAngle(
					leg_angle_reference_[side], input.pitch + 0.5 * kTwoPi,
					kRecoverySwingAngleRateDeg * kDegToRad * dt);
				if (std::abs(std::remainder(input.leg[side].angle - input.pitch -
							0.5 * kTwoPi, kTwoPi)) < 10.0 * kDegToRad) {
					swing_escape_phase_[side] = SwingEscapePhase::kToFinal;
				}
			} else {
				leg_length_reference_[side] = MoveToward(
					leg_length_reference_[side], kRecoveryMaximumLegLength,
					kRecoveryExtendLegRate * dt);
				leg_angle_reference_[side] = MoveTowardAngle(
					leg_angle_reference_[side], swing_target,
					kRecoverySwingAngleRateDeg * kDegToRad * dt);
			}
			const bool side_ready =
				std::abs(std::remainder(input.leg[side].angle - swing_target,
							kTwoPi)) < 15.0 * kDegToRad;
			all_ready = all_ready && side_ready;
		}
		const bool pose_ready = all_ready &&
			std::abs(input.pitch) < pitch_limit + 15.0 * kDegToRad &&
			std::abs(input.roll) < roll_limit + 10.0 * kDegToRad;
		ready_elapsed_s_ = pose_ready ? ready_elapsed_s_ + dt : 0.0;
		if (ready_elapsed_s_ >= ready_hold) {
			EnterPhase(RecoveryPhase::kDrawBack, input);
		} else if (phase_elapsed_s_ >=
			   static_cast<double>(kRecoverySwingTimeoutMs) * 0.001) {
			EnterPhase(RecoveryPhase::kFailed, input);
		}
	} else if (phase_ == RecoveryPhase::kDrawBack) {
		for (size_t side = 0U; side < input.leg.size(); ++side) {
			leg_length_reference_[side] = MoveToward(
				leg_length_reference_[side], kRecoveryMinimumLegLength,
				kRecoveryDrawbackLegRate * dt);
			leg_angle_reference_[side] = MoveTowardAngle(
				leg_angle_reference_[side], input.pitch,
				kRecoveryDrawbackAngleRateDeg * kDegToRad * dt);
		}
		const bool ready =
			std::abs(input.leg[0].length - kRecoveryMinimumLegLength) < 0.05 &&
			std::abs(input.leg[1].length - kRecoveryMinimumLegLength) < 0.05 &&
			std::abs(input.leg[0].length - input.leg[1].length) < 0.04 &&
			std::abs(input.leg[0].angle - input.pitch) < 15.0 * kDegToRad &&
			std::abs(input.leg[1].angle - input.pitch) < 15.0 * kDegToRad &&
			std::abs(input.pitch) < pitch_limit && std::abs(input.roll) < roll_limit;
		ready_elapsed_s_ = ready ? ready_elapsed_s_ + dt : 0.0;
		if (ready_elapsed_s_ >= ready_hold) {
			EnterPhase(RecoveryPhase::kReturn, input);
		} else if (phase_elapsed_s_ >=
			   static_cast<double>(kRecoveryDrawbackTimeoutMs) * 0.001) {
			EnterPhase(RecoveryPhase::kFailed, input);
		}
	}
}

bool RecoveryController::UpdateJointTargets(const RecoveryControllerInput &input)
{
	bool all_valid = true;
	for (uint8_t side = 0U; side < input.leg.size(); ++side) {
		const size_t base = 2U * side;
		const double raw_angle = RawKinematicAngle(side, leg_angle_reference_[side]);
		const double hx = leg_length_reference_[side] * std::sin(raw_angle);
		const double hz = -leg_length_reference_[side] * std::cos(raw_angle);
		const double seed1 = target_initialized_[side] ? joint_target_[base + 1U]
							      : input.joint_position[base + 1U];
		const double seed2 = target_initialized_[side] ? joint_target_[base]
							      : input.joint_position[base];
		double phi1 = 0.0, phi2 = 0.0;
		if (!InverseKinematics(hx, hz, kBranch[side], seed1, seed2, phi1, phi2)) {
			all_valid = false;
			continue;
		}
		joint_target_[base] = phi2;
		joint_target_[base + 1U] = phi1;
		target_initialized_[side] = true;
	}
	return all_valid && target_initialized_[0] && target_initialized_[1];
}

double RecoveryController::RawKinematicAngle(uint8_t side, double angle)
{
	return side == 0U ? std::remainder(kLegAngleOffset[0] - angle, kTwoPi)
			  : std::remainder(angle + kLegAngleOffset[1], kTwoPi);
}

double RecoveryController::ComputeJointTorque(uint8_t index, double target,
					       double position, double velocity, double dt)
{
	using namespace chassis_config;
	if (index >= joint_pid_.size() || !std::isfinite(target) ||
	    !std::isfinite(position) || !std::isfinite(velocity)) return 0.0;
	JointPidState &pid = joint_pid_[index];
	const double error = std::remainder(target - position, kTwoPi);
	pid.position_integral = std::clamp(pid.position_integral + error * dt,
					   -kJointPositionMaxSpeed, kJointPositionMaxSpeed);
	const double speed_target = std::clamp(
		kJointPositionKp * error + kJointPositionKi * pid.position_integral,
		-kJointPositionMaxSpeed, kJointPositionMaxSpeed);
	const double speed_error = speed_target - velocity;
	pid.speed_integral = std::clamp(pid.speed_integral + speed_error * dt,
					-kJointIntegralTorqueLimit, kJointIntegralTorqueLimit);
	return std::clamp(kJointSpeedKp * speed_error + kJointSpeedKi * pid.speed_integral,
			  -kJointTorqueMax, kJointTorqueMax);
}

} // namespace modules
