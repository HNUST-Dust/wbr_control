/* SPDX-License-Identifier: Apache-2.0 */

#include "balance_controller.h"

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

enum class UnifiedLqrState : uint8_t {
	kPosition = 0U, kSpeed, kYaw, kYawRate,
	kLeftLegAngle, kLeftLegAngularVelocity,
	kRightLegAngle, kRightLegAngularVelocity,
	kPitch, kPitchRate, kCount,
};

enum class UnifiedLqrOutput : uint8_t {
	kLeftWheelTorque = 0U, kRightWheelTorque,
	kLeftLegTorque, kRightLegTorque, kCount,
};

template <typename Enum> constexpr size_t ToIndex(Enum value)
{
	return static_cast<size_t>(value);
}

double MoveToward(double current, double target, double max_delta)
{
	return current + std::clamp(target - current, -max_delta, max_delta);
}

double ApplyRemoteDeadband(double value)
{
	if (!std::isfinite(value)) {
		return 0.0;
	}
	const double limited = std::clamp(value, -1.0, 1.0);
	const double magnitude = std::abs(limited);
	if (magnitude <= kRemoteDeadband) {
		return 0.0;
	}
	return std::copysign((magnitude - kRemoteDeadband) / (1.0 - kRemoteDeadband), limited);
}

static_assert(ToIndex(UnifiedLqrState::kCount) == 10U);
static_assert(ToIndex(UnifiedLqrOutput::kCount) == 4U);

} // namespace

BalanceController::BalanceController() : target_leg_length_(kStoolTargetLegLength)
{
}

void BalanceController::Reset()
{
	body_motion_estimator_.Reset();
	target_body_speed_ = 0.0;
	target_body_position_ = 0.0;
	target_yaw_rate_ = 0.0;
	target_yaw_angle_ = 0.0;
	yaw_reference_initialized_ = false;
	filtered_yaw_rate_ = 0.0;
	yaw_rate_filter_initialized_ = false;
	side_state_ = {};
}

void BalanceController::ResetForEnable()
{
	target_leg_length_ = kStoolTargetLegLength;
	balance_theta_reference_ = kThetaBalanceBias;
	balance_pitch_reference_ = 0.0;
	balance_roll_reference_ = 0.0;
	Reset();
}

void BalanceController::SynchronizeReferences(const ChassisCycleInput &input)
{
	target_leg_length_ = 0.5 * (input.leg.left.length + input.leg.right.length);
	balance_theta_reference_ = input.common_theta;
	balance_pitch_reference_ = input.pitch;
	balance_roll_reference_ = input.roll;
	Reset();
}

void BalanceController::Update(const ChassisCycleInput &input,
			       ChassisControlOutput &output)
{
	// 从交接姿态以受限速率回到稳态目标，角速度反馈始终保持完整控制权。
	balance_theta_reference_ = MoveToward(
		balance_theta_reference_, kThetaBalanceBias,
		kBalanceThetaReferenceRate * input.dt);
	balance_pitch_reference_ = MoveToward(
		balance_pitch_reference_, 0.0, kBalancePitchReferenceRate * input.dt);
	balance_roll_reference_ = MoveToward(
		balance_roll_reference_, 0.0,
		kRollReferenceRateDegPerSec * kDegToRad * input.dt);
	output.roll_reference = balance_roll_reference_;
	const double roll_error = balance_roll_reference_ - input.roll;
	output.roll_compensation_force = std::clamp(
		kRollCompensationSign * kRollCompensationKp * roll_error,
		-kRollCompensationForceLimit, kRollCompensationForceLimit);

	// 平衡控制第一步：结合轮速和腿部运动补偿估计机体前向速度。
	const SidePair<double> sin_theta = {
		.left = std::sin(input.theta.left),
		.right = std::sin(input.theta.right),
	};
	const SidePair<double> cos_theta = {
		.left = std::cos(input.theta.left),
		.right = std::cos(input.theta.right),
	};
	const SidePair<double> raw_wheel_omega = {
		.left = input.wheel_motor_speed_rpm.left *
			kRpmToRadPerSec /
			kWheelReduction,
		.right = input.wheel_motor_speed_rpm.right *
			 kRpmToRadPerSec /
			 kWheelReduction,
	};
	// 将轮速转换到世界坐标，再叠加髋部随腿运动产生的速度。
	const SidePair<double> wheel_world_omega = {
		.left = kLegHardware.left.wheel_feedback_sign * raw_wheel_omega.left +
			input.pitch_rate - input.leg.left.angle_rate,
		.right = kLegHardware.right.wheel_feedback_sign * raw_wheel_omega.right +
			 input.pitch_rate - input.leg.right.angle_rate,
	};
	SidePair<double> body_speed;
	const auto calculate_body_speed = [&](Side side) {
		const LegKinematics &leg = input.leg.Get(side);
		body_speed.Get(side) =
			-kWheelRadius * wheel_world_omega.Get(side) +
			leg.length_rate * sin_theta.Get(side) +
			leg.length * input.theta_rate.Get(side) * cos_theta.Get(side);
	};
	calculate_body_speed(Side::kLeft);
	calculate_body_speed(Side::kRight);
	const double raw_common_x_speed = 0.5 * (body_speed.left + body_speed.right);
	// 正值表示左轮向前、右轮向后，与遥控器横向通道的正方向一致。
	output.measured_turn_wheel_speed = 0.5 * (body_speed.left - body_speed.right);
	output.raw_body_speed = raw_common_x_speed;
	// IMU 前向加速度轴尚未标定，暂传 0，避免引入错误的轴向和符号。
	const BodyMotionState &motion =
		body_motion_estimator_.Update(raw_common_x_speed, 0.0, input.dt);
	output.estimated_body_speed = motion.speed;
	const bool motion_command_enabled = input.remote.run && input.requested_enable;
	const double body_command =
		motion_command_enabled ? ApplyRemoteDeadband(input.remote.chassis_x) : 0.0;
	const double turn_command =
		motion_command_enabled ? ApplyRemoteDeadband(input.remote.chassis_rotate) : 0.0;
	const double leg_length_command =
		motion_command_enabled ? ApplyRemoteDeadband(input.remote.leg_length_delta) : 0.0;
	const double previous_target_leg_length = target_leg_length_;
	target_leg_length_ = std::clamp(
		target_leg_length_ + leg_length_command * kLegLengthCommandRate * input.dt,
		kMinCommandedLegLength, kMaxCommandedLegLength);
	const double target_leg_length_rate = input.dt > 0.0
					      ? (target_leg_length_ - previous_target_leg_length) /
							input.dt
					      : 0.0;
	target_body_speed_ = MoveToward(target_body_speed_, body_command * kMaxBodySpeed,
				       kBodyAccelerationLimit * input.dt);
	/*
	 * 当前遥控正转向使左轮向前、右轮向后，对应顺时针 yaw。
	 * IMU 坐标以逆时针为正，因此目标 yaw-rate 在这里取反号，
	 * 保持操作手现有的摇杆方向不变。
	 */
	target_yaw_rate_ = MoveToward(target_yaw_rate_, -turn_command * kMaxYawRate,
				      kYawAccelerationLimit * input.dt);
	const double measured_yaw = static_cast<double>(input.imu.yaw_deg) * kDegToRad;
	if (!yaw_reference_initialized_) {
		target_yaw_angle_ = measured_yaw;
		yaw_reference_initialized_ = true;
	}
	target_yaw_angle_ = std::remainder(target_yaw_angle_ + target_yaw_rate_ * input.dt,
					  kTwoPi);
	const double raw_yaw_rate = static_cast<double>(input.imu.yaw_rate_rad_s);
	if (!yaw_rate_filter_initialized_) {
		filtered_yaw_rate_ = raw_yaw_rate;
		yaw_rate_filter_initialized_ = true;
	} else {
		const double yaw_rate_filter_alpha =
			1.0 - std::exp(-kTwoPi * kYawRateFilterCutoffHz * input.dt);
		filtered_yaw_rate_ += yaw_rate_filter_alpha * (raw_yaw_rate - filtered_yaw_rate_);
	}
	const double yaw_rate_error =
		filtered_yaw_rate_ - target_yaw_rate_;
	const double yaw_tracking_error = std::clamp(
		std::remainder(measured_yaw - target_yaw_angle_, kTwoPi),
		-kYawTrackingErrorLimit, kYawTrackingErrorLimit);
	target_body_position_ += target_body_speed_ * input.dt;
	const double position_error =
		std::clamp(motion.position - target_body_position_, -kPositionErrorLimit,
			   kPositionErrorLimit);

	// 第二步：完整十状态、四输入 LQR 在左右物理坐标中一次生成四路力矩。
	const double common_leg_length = 0.5 * (input.leg.left.length + input.leg.right.length);
	constexpr size_t kUnifiedStateCount = ToIndex(UnifiedLqrState::kCount);
	constexpr size_t kUnifiedOutputCount = ToIndex(UnifiedLqrOutput::kCount);
	double state_error[kUnifiedStateCount] = {};
	state_error[ToIndex(UnifiedLqrState::kPosition)] =
		kHoldLqrTranslationErrorAtZero ? 0.0 : position_error;
	state_error[ToIndex(UnifiedLqrState::kSpeed)] =
		kHoldLqrTranslationErrorAtZero ? 0.0 : motion.speed - target_body_speed_;
	state_error[ToIndex(UnifiedLqrState::kYaw)] = yaw_tracking_error;
	state_error[ToIndex(UnifiedLqrState::kYawRate)] = yaw_rate_error;
	state_error[ToIndex(UnifiedLqrState::kLeftLegAngle)] =
		input.theta.left - balance_theta_reference_;
	state_error[ToIndex(UnifiedLqrState::kLeftLegAngularVelocity)] =
		input.theta_rate.left;
	state_error[ToIndex(UnifiedLqrState::kRightLegAngle)] =
		input.theta.right - balance_theta_reference_;
	state_error[ToIndex(UnifiedLqrState::kRightLegAngularVelocity)] =
		input.theta_rate.right;
	state_error[ToIndex(UnifiedLqrState::kPitch)] =
		input.pitch - balance_pitch_reference_;
	state_error[ToIndex(UnifiedLqrState::kPitchRate)] = input.pitch_rate;
	double gain[kUnifiedOutputCount][kUnifiedStateCount] = {};
	EvaluateUnifiedLqrGain(common_leg_length, gain);
	double actuator_torque[kUnifiedOutputCount] = {};
	for (size_t actuator = 0U; actuator < kUnifiedOutputCount; ++actuator) {
		for (size_t state = 0U; state < kUnifiedStateCount; ++state) {
			actuator_torque[actuator] -= gain[actuator][state] * state_error[state];
		}
	}
	const double left_wheel = actuator_torque[ToIndex(UnifiedLqrOutput::kLeftWheelTorque)];
	const double right_wheel = actuator_torque[ToIndex(UnifiedLqrOutput::kRightWheelTorque)];
	const double left_leg = actuator_torque[ToIndex(UnifiedLqrOutput::kLeftLegTorque)];
	const double right_leg = actuator_torque[ToIndex(UnifiedLqrOutput::kRightLegTorque)];
	const double common_wheel_torque = std::isfinite(left_wheel + right_wheel)
						? 0.5 * (left_wheel + right_wheel)
						: 0.0;
	output.requested_turn_torque = std::isfinite(left_wheel - right_wheel)
					 ? std::clamp(0.5 * (left_wheel - right_wheel),
						      -kTurnTorqueLimit, kTurnTorqueLimit)
					 : 0.0;
	output.differential_leg_torque = std::isfinite(left_leg - right_leg)
					     ? std::clamp(0.5 * (left_leg - right_leg),
							  -kDifferentialLegTorqueLimit,
							  kDifferentialLegTorqueLimit)
					     : 0.0;
	// 差动力矩只使用共模平衡力矩之外的余量，避免转向命令削弱平衡控制权。
	const double turn_torque_available =
		std::max(0.0, kWheelTorqueLimit - std::abs(common_wheel_torque));
	output.allocated_turn_torque = std::clamp(
		output.requested_turn_torque, -turn_torque_available, turn_torque_available);
	// 正轮差模力矩仍是左加右减，与既有执行器坐标完全一致。
	output.physical_wheel_torque.left =
		common_wheel_torque + output.allocated_turn_torque;
	output.physical_wheel_torque.right =
		common_wheel_torque - output.allocated_turn_torque;
	const double common_leg_torque = std::isfinite(left_leg + right_leg)
					       ? 0.5 * (left_leg + right_leg)
					       : 0.0;
	output.requested_body_on_leg_torque.left =
		common_leg_torque + output.differential_leg_torque;
	output.requested_body_on_leg_torque.right =
		common_leg_torque - output.differential_leg_torque;
	output.body_on_leg_torque = output.requested_body_on_leg_torque;

	// 第三步：VMC 将轴向力和腿部姿态力矩转换为四个关节力矩。
	const auto calculate_vmc_output = [&](Side side) {
		const LegHardwareMap &hardware = kLegHardware.Get(side);
		const LegKinematics &leg = input.leg.Get(side);
		SideState &state = side_state_.Get(side);
		const double length_error = target_leg_length_ - leg.length;
		state.leg_length_integral_force = UpdateLegLengthIntegral(
			state.leg_length_integral_force, length_error, input.dt);
		const double nominal_support_force =
			0.5 * kRobotMass * kGravity / std::max(cos_theta.Get(side), 0.5);
		const double roll_force = side == Side::kLeft
					  ? output.roll_compensation_force
					  : -output.roll_compensation_force;
		const double support_force = nominal_support_force + roll_force;
		// 安装映射负责把“机体对腿”力矩转换成各侧 VMC 输入方向。
		const double vmc_angle_torque =
			hardware.vmc_torque_sign * output.body_on_leg_torque.Get(side);
		const LegVmcOutput vmc = ComputeLegVmc(leg, target_leg_length_, support_force,
						       state.leg_length_integral_force,
						       vmc_angle_torque, leg.length_rate,
						       target_leg_length_rate);
		output.leg_axial_force.Get(side) = vmc.axial_force;
		JointPair<double> &joint_torque = output.joint_torque.Get(side);
		joint_torque.b =
			std::isfinite(vmc.phi2_torque)
				? std::clamp(vmc.phi2_torque, -kJointTorqueLimit, kJointTorqueLimit)
				: 0.0;
		joint_torque.d =
			std::isfinite(vmc.phi1_torque)
				? std::clamp(vmc.phi1_torque, -kJointTorqueLimit, kJointTorqueLimit)
				: 0.0;
	};
	calculate_vmc_output(Side::kLeft);
	calculate_vmc_output(Side::kRight);
}

} // namespace modules
