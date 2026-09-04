/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file src/modules/chassis/chassis_module.cpp
 * @ingroup wbr_modules
 * @brief 实现轮腿底盘控制模块及周期控制流程。
 * @details 实现运行在模块自有 Zephyr 线程或其驱动回调中。回调路径只完成有界的数据搬运和通知，耗时解析与控制计算留在线程上下文执行。
 */

#include "chassis_module.h"

#include <algorithm>
#include <cmath>

#include <zephyr/logging/log.h>

#include <channels/chassismotors_feedback_raw.hpp>
#include <channels/hi91_imu_sample.hpp>
#include <channels/oscilloscope_sample.hpp>
#include <channels/remote_input_state.hpp>
#include <platform/drivers/communication/can_dispatch.h>
#include <scheduling/periodic_schedule.h>
#include <scheduling/thread_priorities.h>
#include "leg_vmc.h"
#include "lqr_schedule.h"

LOG_MODULE_REGISTER(chassis_module, LOG_LEVEL_INF);

namespace
{

K_THREAD_STACK_DEFINE(g_chassis_stack, 4096);

using modules::Joint;
using modules::JointPair;
using modules::Side;
using modules::SidePair;

enum class ChassisJoint : uint8_t {
	kLeftB = 0U,
	kLeftD,
	kRightB,
	kRightD,
	kCount,
};

enum class LqrState : uint8_t {
	kLegAngle = 0U,
	kLegAngularVelocity,
	kPosition,
	kSpeed,
	kPitch,
	kPitchRate,
	kCount,
};

enum class LqrOutput : uint8_t {
	kWheelTorque = 0U,
	kLegTorque,
	kCount,
};

enum class TelemetryChannel : uint8_t {
	kControlState = 0U,
	kCommonLegAngleDeg,
	kLeftSentWheelTorque,
	kRightSentWheelTorque,
	kPitchDeg,
	kLeftBodyOnLegTorque,
	kRightBodyOnLegTorque,
	kLegAngleWheelContribution,
	kPitchWheelContribution,
	kTargetLegLength,
	kLeftLegLength,
	kRightLegLength,
	kImuMaxSensorIntervalMs,
	kImuCrcErrorCount,
	kImuGapWithCrcCount,
	kImuGapWithoutCrcCount,
	kDmArmResetReason,
	kImuAgeUs,
	kImuMaxPublishIntervalUs,
	kImuTransportErrorCount,
	kCount,
};

template <typename Enum> constexpr size_t ToIndex(Enum value)
{
	return static_cast<size_t>(value);
}

struct MotorAddress {
	uint8_t bus;
	uint16_t can_id;
	platform::CanTxSlot tx_slot;
};

struct LegHardwareMap {
	MotorAddress wheel;
	JointPair<MotorAddress> joint;
	int kinematic_branch;
	double leg_angle_offset;
	double leg_coordinate_sign;
	double wheel_feedback_sign;
	double vmc_torque_sign;
	double wheel_command_sign;
};

// 左右腿的总线、ID、发送邮箱和安装方向集中定义，控制代码不再解释裸数字。
constexpr SidePair<LegHardwareMap> kLegHardware = {
	.left =
		{
			.wheel = {3U, 0x201U, platform::CanTxSlot::kLeftWheel},
			.joint =
				{
					.b = {3U, 0x00U, platform::CanTxSlot::kLeftJointB},
					.d = {3U, 0x03U, platform::CanTxSlot::kLeftJointD},
				},
			.kinematic_branch = 1,
			.leg_angle_offset = -0.036063,
			.leg_coordinate_sign = -1.0,
			.wheel_feedback_sign = 1.0,
			.vmc_torque_sign = -1.0,
			.wheel_command_sign = -1.0,
		},
	.right =
		{
			.wheel = {1U, 0x201U, platform::CanTxSlot::kRightWheel},
			.joint =
				{
					.b = {1U, 0x01U, platform::CanTxSlot::kRightJointB},
					.d = {1U, 0x02U, platform::CanTxSlot::kRightJointD},
				},
			.kinematic_branch = -1,
			.leg_angle_offset = -3.121010,
			.leg_coordinate_sign = 1.0,
			.wheel_feedback_sign = -1.0,
			.vmc_torque_sign = 1.0,
			.wheel_command_sign = 1.0,
		},
};

constexpr uint16_t kWheelCommandId = 0x200U;

// 机械结构、坐标变换和控制器参数。
constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kDegToRad = 0.01745329251994329577;
constexpr double kDpsToRadPerSec = kDegToRad;
constexpr double kRpmToRadPerSec = 0.10471975511965977;
constexpr double kWheelReduction = 268.0 / 17.0;
constexpr double kWheelRadius = 0.058;
constexpr double kGravity = 9.80665;
constexpr double kRobotMass = 12.054;
constexpr double kDefaultDt = 0.001;
constexpr uint32_t kControlPeriodMs = 1U;
constexpr double kTargetLegLengthMin = 0.15362;
constexpr double kTargetLegLengthMax = 0.31101;
constexpr double kTargetLegLengthRate = 0.06;

constexpr double kPerSideGainScale = 0.5;
constexpr double kPositionErrorLimit = 0.15;
// false 表示将受限的位置误差和估计速度送入完整六状态 LQR。
constexpr bool kHoldLqrTranslationErrorAtZero = false;
constexpr double kLegCoordinateKp = 6.0;
constexpr double kLegCoordinateKd = 0.6;
constexpr double kLegCoordinateTorqueLimit = 2.0;
constexpr double kDjiCurrentPerNm = 3450.0;
// ±16384 是 DJI 0x200 协议范围保护，不是底盘力矩限制。
constexpr int16_t kDjiProtocolCurrentLimit = 16384;
constexpr double kJointTorqueLimit = 54.0;

// 输入新鲜度、安全保护和 DM 电机使能时序。
constexpr uint32_t kRemoteTimeoutMs = 1000U;
constexpr uint32_t kImuTimeoutMs = 3U;
constexpr uint64_t kMotorTimeoutUs = 2000ULL;
// 撑起过程允许较大俯仰角，遥控使能仍是主要停机手段。
constexpr double kTiltCutoffDeg = 75.0;
constexpr uint32_t kDmClearTicks = 50U;
constexpr uint32_t kDmArmTicks = 300U;
constexpr uint32_t kDmModePeriodTicks = 10U;

// LQR 的腿部平衡角偏置。
constexpr double kThetaBalanceBias = 5.0 * kDegToRad;

constexpr protocols::DmMitRange kDmRange = {
	.p_min = -12.56637f,
	.p_max = 12.56637f,
	.v_min = -45.0f,
	.v_max = 45.0f,
	.kp_min = 0.0f,
	.kp_max = 500.0f,
	.kd_min = 0.0f,
	.kd_max = 5.0f,
	.t_min = -54.0f,
	.t_max = 54.0f,
};

static_assert(ToIndex(LqrState::kCount) == 6U);
static_assert(ToIndex(ChassisJoint::kCount) == 4U);
static_assert(ToIndex(TelemetryChannel::kCount) == channels::kOscilloscopeMaxChannels);

} // namespace

namespace modules
{

// 一个周期内只读的输入快照。读取完成后，后续阶段不再访问通信通道。
struct ChassisModule::CycleInput {
	SidePair<ChassisMotorFeedbackRawFrame> wheel_frame;
	SidePair<JointPair<ChassisMotorFeedbackRawFrame>> joint_frame;
	channels::Hi91ImuSample imu = {};
	channels::RemoteInputState remote = {};
	SidePair<LegKinematics> leg;
	SidePair<JointPair<double>> joint_position;
	SidePair<JointPair<double>> joint_velocity;
	double dt = kDefaultDt;
	double pitch = 0.0;
	double pitch_rate = 0.0;
	SidePair<double> theta;
	SidePair<double> theta_rate;
	double common_theta = 0.0;
	double common_theta_rate = 0.0;
	bool imu_fresh = false;
	uint64_t imu_age_us = UINT64_MAX;
	uint32_t remote_age_ms = UINT32_MAX;
	bool feedback_valid = false;
	bool requested_enable = false;
	bool dm_ready = false;
	bool arm_complete = false;
};

// 一个周期内的控制结果，同时作为执行器下发和遥测发布的数据源。
struct ChassisModule::CycleOutput {
	SidePair<double> physical_wheel_torque;
	SidePair<double> sent_wheel_torque;
	SidePair<bool> wheel_torque_saturated;
	SidePair<double> requested_body_on_leg_torque;
	SidePair<double> body_on_leg_torque;
	double theta_wheel_torque_contribution = 0.0;
	double pitch_wheel_torque_contribution = 0.0;
	SidePair<double> leg_axial_force;
	SidePair<JointPair<double>> joint_torque;
};

ChassisModule::ChassisModule()
{
	last_requested_enable_ = false;
	tilt_fault_latched_ = false;
	loop_ticks_ = 0U;
	dm_arm_ticks_ = 0U;
	last_remote_sequence_ = 0U;
	last_remote_update_ms_ = 0U;
	last_imu_sequence_ = 0U;
	last_loop_time_us_ = 0U;
	deadline_miss_count_ = 0U;
	control_state_ = ControlState::kDisabled;
	balance_phase_reached_ = false;
	stool_ready_ = false;
	target_leg_length_ = StoolController::kTargetLegLength;
	stool_controller_.Reset();
	ResetControlState();
}

int ChassisModule::Start()
{
	return CreateThread(
		g_chassis_stack, K_THREAD_STACK_SIZEOF(g_chassis_stack),
		K_PRIO_PREEMPT(wbr_control::scheduling::thread_priority::kChassis), "chassis");
}

void ChassisModule::ReadCycleInput(CycleInput &input, uint32_t now_ms, double dt)
{
	input.dt = dt;

	// 固化六个执行器的原始 CAN 快照，再统一解码。
	left_wheel_feedback_raw.read(input.wheel_frame.left);
	right_wheel_feedback_raw.read(input.wheel_frame.right);
	left_b_motor_feedback_raw.read(input.joint_frame.left.b);
	left_d_motor_feedback_raw.read(input.joint_frame.left.d);
	right_b_motor_feedback_raw.read(input.joint_frame.right.b);
	right_d_motor_feedback_raw.read(input.joint_frame.right.d);

	SidePair<bool> wheel_decoded;
	wheel_decoded.left = input.wheel_frame.left.valid &&
			     protocols::DecodeDjiFeedback(input.wheel_frame.left.data, 8U,
							  &wheel_feedback_.left) == 0;
	wheel_decoded.right = input.wheel_frame.right.valid &&
			      protocols::DecodeDjiFeedback(input.wheel_frame.right.data, 8U,
							   &wheel_feedback_.right) == 0;

	SidePair<JointPair<bool>> joint_decoded;
	joint_decoded.left.b = input.joint_frame.left.b.valid &&
			       protocols::DecodeDmFeedbackNormal(input.joint_frame.left.b.data, 8U,
								 &joint_feedback_.left.b) == 0;
	joint_decoded.left.d = input.joint_frame.left.d.valid &&
			       protocols::DecodeDmFeedbackNormal(input.joint_frame.left.d.data, 8U,
								 &joint_feedback_.left.d) == 0;
	joint_decoded.right.b =
		input.joint_frame.right.b.valid &&
		protocols::DecodeDmFeedbackNormal(input.joint_frame.right.b.data, 8U,
						  &joint_feedback_.right.b) == 0;
	joint_decoded.right.d =
		input.joint_frame.right.d.valid &&
		protocols::DecodeDmFeedbackNormal(input.joint_frame.right.d.data, 8U,
						  &joint_feedback_.right.d) == 0;

	// sequence 用于区分新样本和通道中残留的旧值。
	input.remote = {};
	latest_remote_state.read(input.remote);
	if (input.remote.sequence != 0U && input.remote.sequence != last_remote_sequence_) {
		last_remote_sequence_ = input.remote.sequence;
		last_remote_update_ms_ = now_ms;
	}
	input.remote_age_ms = last_remote_sequence_ != 0U
				      ? now_ms - last_remote_update_ms_
				      : UINT32_MAX;
	const bool remote_fresh = input.remote_age_ms <= kRemoteTimeoutMs;
	input.requested_enable = remote_fresh && input.remote.robot_enable;

	channels::Hi91ImuSample new_imu = {};
	if (channels::latest_hi91_imu_sample.read(new_imu) && new_imu.sequence != 0U) {
		input.imu = new_imu;
		if (input.imu.sequence != last_imu_sequence_) {
			last_imu_sequence_ = input.imu.sequence;
		}
	}

	// 快照读取期间 ISR 仍可能发布新帧，因此在全部读取完成后重新取时间基准。
	const uint64_t feedback_now_us = k_cyc_to_us_floor64(k_cycle_get_64());
	input.imu_age_us =
		input.imu.valid && input.imu.precise_timestamp_us <= feedback_now_us
			? feedback_now_us - input.imu.precise_timestamp_us
			: UINT64_MAX;
	input.imu_fresh = input.imu.valid && last_imu_sequence_ != 0U &&
			  input.imu_age_us <= static_cast<uint64_t>(kImuTimeoutMs) * 1000ULL;

	const auto frame_is_fresh = [feedback_now_us](const ChassisMotorFeedbackRawFrame &frame,
						      bool decoded) {
		return decoded && frame.IsPreciselyFresh(feedback_now_us, kMotorTimeoutUs);
	};
	const bool motors_fresh =
		frame_is_fresh(input.wheel_frame.left, wheel_decoded.left) &&
		frame_is_fresh(input.wheel_frame.right, wheel_decoded.right) &&
		frame_is_fresh(input.joint_frame.left.b, joint_decoded.left.b) &&
		frame_is_fresh(input.joint_frame.left.d, joint_decoded.left.d) &&
		frame_is_fresh(input.joint_frame.right.b, joint_decoded.right.b) &&
		frame_is_fresh(input.joint_frame.right.d, joint_decoded.right.d);

	// 正运动学输出统一采用校准后的左右腿坐标。
	input.leg = {};
	SidePair<bool> leg_valid;
	const auto update_leg_kinematics = [&](Side side) {
		const LegHardwareMap &hardware = kLegHardware.Get(side);
		const JointPair<bool> &decoded = joint_decoded.Get(side);
		const JointPair<protocols::DmMotorFeedbackNormal> &feedback =
			joint_feedback_.Get(side);
		LegKinematics &leg = input.leg.Get(side);
		bool &valid = leg_valid.Get(side);
		if (!decoded.b || !decoded.d) {
			return;
		}
		valid = ComputeLegKinematics(protocols::DmFeedbackPosition(feedback.d, kDmRange),
					     protocols::DmFeedbackPosition(feedback.b, kDmRange),
					     protocols::DmFeedbackVelocity(feedback.d, kDmRange),
					     protocols::DmFeedbackVelocity(feedback.b, kDmRange),
					     hardware.kinematic_branch, leg);
		if (valid) {
			leg.angle = hardware.leg_coordinate_sign *
				    std::remainder(leg.angle - hardware.leg_angle_offset, kTwoPi);
			leg.angle_rate *= hardware.leg_coordinate_sign;
		}
	};
	update_leg_kinematics(Side::kLeft);
	update_leg_kinematics(Side::kRight);

	input.feedback_valid = motors_fresh && input.imu_fresh && leg_valid.left && leg_valid.right;
	input.dm_ready = joint_feedback_.left.b.control_status == 1U &&
			 joint_feedback_.left.d.control_status == 1U &&
			 joint_feedback_.right.b.control_status == 1U &&
			 joint_feedback_.right.d.control_status == 1U;
	input.arm_complete = dm_arm_ticks_ >= kDmArmTicks;

	input.pitch = input.imu_fresh ? static_cast<double>(input.imu.pitch_deg) * kDegToRad : 0.0;
	input.pitch_rate = input.imu_fresh
				   ? static_cast<double>(input.imu.gyro_dps[0]) * kDpsToRadPerSec
				   : 0.0;
	input.theta = {};
	input.theta_rate = {};
	input.common_theta = 0.0;
	input.common_theta_rate = 0.0;
	if (input.feedback_valid) {
		// 论文坐标满足 alpha = theta + pitch，因此 theta = alpha - pitch。
		input.theta.left = std::remainder(input.leg.left.angle - input.pitch, kTwoPi);
		input.theta.right = std::remainder(input.leg.right.angle - input.pitch, kTwoPi);
		input.theta_rate.left = input.leg.left.angle_rate - input.pitch_rate;
		input.theta_rate.right = input.leg.right.angle_rate - input.pitch_rate;
		input.common_theta = std::remainder(
			0.5 * (input.leg.left.angle + input.leg.right.angle) - input.pitch, kTwoPi);
		input.common_theta_rate =
			0.5 * (input.leg.left.angle_rate + input.leg.right.angle_rate) -
			input.pitch_rate;
	}

	input.joint_position.left.b =
		protocols::DmFeedbackPosition(joint_feedback_.left.b, kDmRange);
	input.joint_position.left.d =
		protocols::DmFeedbackPosition(joint_feedback_.left.d, kDmRange);
	input.joint_position.right.b =
		protocols::DmFeedbackPosition(joint_feedback_.right.b, kDmRange);
	input.joint_position.right.d =
		protocols::DmFeedbackPosition(joint_feedback_.right.d, kDmRange);
	input.joint_velocity.left.b =
		protocols::DmFeedbackVelocity(joint_feedback_.left.b, kDmRange);
	input.joint_velocity.left.d =
		protocols::DmFeedbackVelocity(joint_feedback_.left.d, kDmRange);
	input.joint_velocity.right.b =
		protocols::DmFeedbackVelocity(joint_feedback_.right.b, kDmRange);
	input.joint_velocity.right.d =
		protocols::DmFeedbackVelocity(joint_feedback_.right.d, kDmRange);
}

void ChassisModule::UpdateControlState(const CycleInput &input)
{
	// 只有撤销使能后才解除倾倒锁存，防止机器人自行重新启动。
	if (!input.requested_enable && tilt_fault_latched_) {
		tilt_fault_latched_ = false;
	}

	// 每次使能沿都重新执行 DM 使能和撑起对齐流程。
	if (input.requested_enable != last_requested_enable_) {
		ResetControlState();
		dm_arm_ticks_ = 0U;
		if (input.requested_enable) {
			dm_arm_reset_reason_ = DmArmResetReason::kRemoteEnableEdge;
		}
		balance_phase_reached_ = false;
		stool_ready_ = false;
		stool_controller_.Reset();
		target_leg_length_ = StoolController::kTargetLegLength;
		last_requested_enable_ = input.requested_enable;
		LOG_INF("chassis request %s", input.requested_enable ? "enabled" : "disabled");
	}

	if (input.requested_enable && input.imu_fresh) {
		const double pitch_deg = static_cast<double>(input.imu.pitch_deg);
		if (!tilt_fault_latched_ && std::abs(pitch_deg) >= kTiltCutoffDeg) {
			tilt_fault_latched_ = true;
			LOG_ERR("tilt fault pitch_mdeg=%d", static_cast<int>(pitch_deg * 1000.0));
		}
	}

	// 状态优先级也是安全优先级：停机和故障必须覆盖所有控制状态。
	if (!input.requested_enable) {
		control_state_ = ControlState::kDisabled;
	} else if (tilt_fault_latched_) {
		control_state_ = ControlState::kTiltFault;
	} else if (!input.imu_fresh) {
		control_state_ = ControlState::kSafetyStop;
	} else if (!input.arm_complete || !input.dm_ready) {
		control_state_ = ControlState::kDmArming;
	} else if (!input.feedback_valid) {
		control_state_ = ControlState::kWaitingFeedback;
	} else {
		control_state_ =
			balance_phase_reached_ ? ControlState::kBalance : ControlState::kStool;
	}
}

void ChassisModule::ComputeControlOutput(const CycleInput &input, CycleOutput &output)
{
	output = {};

	if (control_state_ == ControlState::kStool) {
		// 撑起状态负责腿长收缩、腿角对齐和关节级联 PID。
		target_leg_length_ = StoolController::kTargetLegLength;
		StoolControllerInput stool_input = {};
		stool_input.dt = input.dt;
		stool_input.pitch = input.pitch;
		stool_input.leg = {input.leg.left, input.leg.right};
		stool_input.joint_position = {
			input.joint_position.left.b,
			input.joint_position.left.d,
			input.joint_position.right.b,
			input.joint_position.right.d,
		};
		stool_input.joint_velocity = {
			input.joint_velocity.left.b,
			input.joint_velocity.left.d,
			input.joint_velocity.right.b,
			input.joint_velocity.right.d,
		};
		const StoolControllerOutput stool = stool_controller_.Update(stool_input);
		output.joint_torque.left.b = stool.joint_torque[ToIndex(ChassisJoint::kLeftB)];
		output.joint_torque.left.d = stool.joint_torque[ToIndex(ChassisJoint::kLeftD)];
		output.joint_torque.right.b = stool.joint_torque[ToIndex(ChassisJoint::kRightB)];
		output.joint_torque.right.d = stool.joint_torque[ToIndex(ChassisJoint::kRightD)];
		stool_ready_ = stool.ready;
		if (stool.ready) {
			// 本周期保留撑起控制器的关节力矩，下周期开始运行完整 LQR。
			balance_phase_reached_ = true;
			control_state_ = ControlState::kBalance;
			target_leg_length_ = 0.5 * (input.leg.left.length + input.leg.right.length);
			ResetControlState();
			LOG_INF("stool theta within tolerance; entering full LQR");
		}
		return;
	}

	if (control_state_ != ControlState::kBalance) {
		return;
	}

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
		.left = static_cast<double>(wheel_feedback_.left.omega) * kRpmToRadPerSec /
			kWheelReduction,
		.right = static_cast<double>(wheel_feedback_.right.omega) * kRpmToRadPerSec /
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
	// IMU 前向加速度轴尚未标定，暂传 0，避免引入错误的轴向和符号。
	const BodyMotionState &motion =
		body_motion_estimator_.Update(raw_common_x_speed, 0.0, input.dt);
	const double position_error =
		std::clamp(motion.position, -kPositionErrorLimit, kPositionErrorLimit);

	// 第二步：按平均腿长调度 LQR 增益，得到轮毂和腿部姿态力矩。
	constexpr size_t kLqrStateCount = ToIndex(LqrState::kCount);
	double state_error[kLqrStateCount] = {};
	state_error[ToIndex(LqrState::kLegAngle)] = input.common_theta - kThetaBalanceBias;
	state_error[ToIndex(LqrState::kLegAngularVelocity)] = input.common_theta_rate;
	state_error[ToIndex(LqrState::kPosition)] =
		kHoldLqrTranslationErrorAtZero ? 0.0 : position_error;
	state_error[ToIndex(LqrState::kSpeed)] =
		kHoldLqrTranslationErrorAtZero ? 0.0 : motion.speed;
	state_error[ToIndex(LqrState::kPitch)] = input.pitch;
	state_error[ToIndex(LqrState::kPitchRate)] = input.pitch_rate;
	const double common_leg_length = 0.5 * (input.leg.left.length + input.leg.right.length);
	double common_gain[ToIndex(LqrOutput::kCount)][kLqrStateCount] = {};
	EvaluateLqrGain(common_leg_length, common_gain);
	double wheel_sum = 0.0;
	double leg_sum = 0.0;
	for (size_t state = 0U; state < kLqrStateCount; ++state) {
		const double wheel_contribution =
			common_gain[ToIndex(LqrOutput::kWheelTorque)][state] * state_error[state];
		const double leg_contribution =
			common_gain[ToIndex(LqrOutput::kLegTorque)][state] * state_error[state];
		wheel_sum += wheel_contribution;
		leg_sum += leg_contribution;
		if (state == ToIndex(LqrState::kLegAngle)) {
			output.theta_wheel_torque_contribution =
				-kPerSideGainScale * wheel_contribution;
		} else if (state == ToIndex(LqrState::kPitch)) {
			output.pitch_wheel_torque_contribution =
				-kPerSideGainScale * wheel_contribution;
		}
	}
	const double wheel_torque = -kPerSideGainScale * wheel_sum;
	const double leg_torque = -kPerSideGainScale * leg_sum;
	const auto assign_lqr_output = [&](Side side) {
		// 轮毂力矩不在控制器内截断，最终由 DJI 协议范围保护。
		output.physical_wheel_torque.Get(side) =
			std::isfinite(wheel_torque) ? wheel_torque : 0.0;
		// LQR 腿部姿态力矩保留完整控制权，最终由关节力矩上限保护。
		output.requested_body_on_leg_torque.Get(side) =
			std::isfinite(leg_torque) ? leg_torque : 0.0;
		output.body_on_leg_torque.Get(side) = output.requested_body_on_leg_torque.Get(side);
	};
	assign_lqr_output(Side::kLeft);
	assign_lqr_output(Side::kRight);

	// 左右腿协调项只在两侧之间重分配力矩，不改变总力矩。
	const double coordinate_torque_request =
		kLegCoordinateKp *
			std::remainder(input.leg.left.angle - input.leg.right.angle, kTwoPi) +
		kLegCoordinateKd * (input.leg.left.angle_rate - input.leg.right.angle_rate);
	const double coordinate_torque =
		std::isfinite(coordinate_torque_request)
			? std::clamp(coordinate_torque_request, -kLegCoordinateTorqueLimit,
				     kLegCoordinateTorqueLimit)
			: 0.0;
	output.body_on_leg_torque.left -= coordinate_torque;
	output.body_on_leg_torque.right += coordinate_torque;

	// 第三步：VMC 将轴向力和腿部姿态力矩转换为四个关节力矩。
	const auto calculate_vmc_output = [&](Side side) {
		const LegHardwareMap &hardware = kLegHardware.Get(side);
		const LegKinematics &leg = input.leg.Get(side);
		SideState &state = side_state_.Get(side);
		const double length_error = target_leg_length_ - leg.length;
		state.leg_length_integral_force = UpdateLegLengthIntegral(
			state.leg_length_integral_force, length_error, input.dt);
		const double support_force =
			0.25 * kRobotMass * kGravity / std::max(cos_theta.Get(side), 0.5);
		// 安装映射负责把“机体对腿”力矩转换成各侧 VMC 输入方向。
		const double vmc_angle_torque =
			hardware.vmc_torque_sign * output.body_on_leg_torque.Get(side);
		const LegVmcOutput vmc = ComputeLegVmc(leg, target_leg_length_, support_force,
						       state.leg_length_integral_force,
						       vmc_angle_torque, leg.length_rate, 0.0);
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

void ChassisModule::ApplyControlOutput(CycleOutput &output)
{
	switch (control_state_) {
	case ControlState::kDisabled:
		// 停机时周期性发送退出命令，并确保轮毂电流为零。
		if ((loop_ticks_ % 100U) == 0U) {
			SendDmControl(protocols::DmControlCommand::kExit);
		}
		if ((loop_ticks_ % 10U) == 0U) {
			SendWheelCurrent(Side::kLeft, 0);
			SendWheelCurrent(Side::kRight, 0);
		}
		dm_arm_ticks_ = 0U;
		ResetControlState();
		return;

	case ControlState::kSafetyStop:
		dm_arm_reset_reason_ = DmArmResetReason::kImuFreshness;
		if ((loop_ticks_ % 100U) == 0U) {
			SendDmControl(protocols::DmControlCommand::kExit);
		}
		if ((loop_ticks_ % 10U) == 0U) {
			SendWheelCurrent(Side::kLeft, 0);
			SendWheelCurrent(Side::kRight, 0);
		}
		dm_arm_ticks_ = 0U;
		ResetControlState();
		return;

	case ControlState::kTiltFault:
		dm_arm_reset_reason_ = DmArmResetReason::kTiltFault;
		if ((loop_ticks_ % 100U) == 0U) {
			SendDmControl(protocols::DmControlCommand::kExit);
		}
		if ((loop_ticks_ % 10U) == 0U) {
			SendWheelCurrent(Side::kLeft, 0);
			SendWheelCurrent(Side::kRight, 0);
		}
		dm_arm_ticks_ = 0U;
		ResetControlState();
		return;

	case ControlState::kDmArming:
		// DM 退出后可能不再反馈，因此使能阶段不能以反馈新鲜度为前提。
		if ((loop_ticks_ % kDmModePeriodTicks) == 0U) {
			SendDmControl(dm_arm_ticks_ < kDmClearTicks
					      ? protocols::DmControlCommand::kClearError
					      : protocols::DmControlCommand::kEnter);
		}
		SendWheelCurrent(Side::kLeft, 0);
		SendWheelCurrent(Side::kRight, 0);
		ResetControlState();
		if (dm_arm_ticks_ < kDmArmTicks) {
			++dm_arm_ticks_;
		}
		return;

	case ControlState::kWaitingFeedback: {
		// 已进入 MIT 模式后保持正常频率的零力矩帧，等待反馈链路恢复。
		const SidePair<JointPair<double>> zero_joint_torque = {};
		const SidePair<int16_t> zero_wheel_current = {};
		SendScheduledOutputs(zero_joint_torque, zero_wheel_current);
		ResetControlState();
		return;
	}

	case ControlState::kStool:
		// 撑起阶段只下发关节力矩，轮毂始终保持零电流。
		SendScheduledOutputs(output.joint_torque, {});
		return;

	case ControlState::kBalance:
		break;
	}

	// 平衡状态才允许将 LQR 轮毂力矩映射到左右 3508 电流。
	const auto torque_to_current = [](double torque) {
		const double current =
			std::isfinite(torque)
				? std::clamp(torque * kDjiCurrentPerNm,
					     -static_cast<double>(kDjiProtocolCurrentLimit),
					     static_cast<double>(kDjiProtocolCurrentLimit))
				: 0.0;
		return static_cast<int16_t>(current);
	};
	SidePair<int16_t> wheel_current;
	wheel_current.left = torque_to_current(kLegHardware.left.wheel_command_sign *
					       output.physical_wheel_torque.left);
	wheel_current.right = torque_to_current(kLegHardware.right.wheel_command_sign *
						output.physical_wheel_torque.right);
	output.wheel_torque_saturated.left =
		std::abs(static_cast<int32_t>(wheel_current.left)) >= kDjiProtocolCurrentLimit;
	output.wheel_torque_saturated.right =
		std::abs(static_cast<int32_t>(wheel_current.right)) >= kDjiProtocolCurrentLimit;

	// 遥测记录实际进入 CAN 的力矩，并还原为统一的物理前进方向。
	output.sent_wheel_torque.left = kLegHardware.left.wheel_command_sign *
					static_cast<double>(wheel_current.left) / kDjiCurrentPerNm;
	output.sent_wheel_torque.right = kLegHardware.right.wheel_command_sign *
					 static_cast<double>(wheel_current.right) /
					 kDjiCurrentPerNm;
	SendScheduledOutputs(output.joint_torque, wheel_current);
}


void ChassisModule::ResetControlState()
{
	body_motion_estimator_.Reset();
	side_state_ = {};
}

void ChassisModule::SendDmControl(protocols::DmControlCommand command)
{
	uint8_t data[8] = {};
	if (protocols::GetDmControlCommandFrame(command, data) != 0) {
		return;
	}
	const auto submit_joint = [&](Side side, Joint joint) {
		const MotorAddress &motor = kLegHardware.Get(side).joint.Get(joint);
		(void)platform::SubmitCanStandardFrame(motor.tx_slot, motor.bus, motor.can_id, data,
						       sizeof(data));
	};
	submit_joint(Side::kLeft, Joint::kB);
	submit_joint(Side::kLeft, Joint::kD);
	submit_joint(Side::kRight, Joint::kB);
	submit_joint(Side::kRight, Joint::kD);
}
void ChassisModule::SendDmTorque(Side side, Joint joint, double torque)
{
	protocols::DmMitCommand command = {};
	command.torque = static_cast<float>(torque);
	uint8_t data[8] = {};
	if (protocols::PackDmMitCommand(&command, &kDmRange, data) != 0) {
		return;
	}
	const MotorAddress &motor = kLegHardware.Get(side).joint.Get(joint);
	(void)platform::SubmitCanStandardFrame(motor.tx_slot, motor.bus, motor.can_id, data,
					       sizeof(data));
}

void ChassisModule::SendWheelCurrent(Side side, int16_t current)
{
	uint8_t data[8] = {};
	if (protocols::WriteDjiCurrentCommandToSlot(kLegHardware.Get(side).wheel.can_id, current,
						    data) != 0) {
		return;
	}
	const MotorAddress &wheel = kLegHardware.Get(side).wheel;
	(void)platform::SubmitCanStandardFrame(wheel.tx_slot, wheel.bus, kWheelCommandId, data,
					       sizeof(data));
}

void ChassisModule::SendScheduledOutputs(const SidePair<JointPair<double>> &joint_torque,
					 const SidePair<int16_t> &wheel_current)
{
	// CAN 工作线程优先级低于底盘线程，六个最新值邮箱会先完成同批更新。
	SendDmTorque(Side::kLeft, Joint::kB, joint_torque.left.b);
	SendDmTorque(Side::kRight, Joint::kB, joint_torque.right.b);
	SendDmTorque(Side::kLeft, Joint::kD, joint_torque.left.d);
	SendDmTorque(Side::kRight, Joint::kD, joint_torque.right.d);
	SendWheelCurrent(Side::kLeft, wheel_current.left);
	SendWheelCurrent(Side::kRight, wheel_current.right);
}


void ChassisModule::PublishTelemetry(const CycleInput &input, const CycleOutput &output)
{
	// 保持既有 VOFA 状态码，DM 重新使能原因由独立通道输出。
	float balance_state_x100 = 0.0F;
	const bool control_enabled = input.requested_enable && input.feedback_valid &&
				     !tilt_fault_latched_ && input.arm_complete && input.dm_ready;
	if (input.requested_enable) {
		balance_state_x100 =
			tilt_fault_latched_
				? 400.0F
				: (!input.arm_complete
					   ? 110.0F
					   : (!input.feedback_valid
						      ? 120.0F
						      : (!control_enabled
								 ? 130.0F
								 : (control_state_ == ControlState::
											      kStool
									    ? 200.0F
									    : 300.0F))));
	}

	static uint32_t sequence = 0U;
	channels::OscilloscopeSample sample = {};
	sample.sequence = ++sequence;
	sample.uptime_ms = k_uptime_get_32();
	sample.channel_count = static_cast<uint8_t>(ToIndex(TelemetryChannel::kCount));
	const auto set_channel = [&sample](TelemetryChannel channel, float value) {
		sample.value[ToIndex(channel)] = value;
	};
	set_channel(TelemetryChannel::kControlState, balance_state_x100);
	set_channel(TelemetryChannel::kCommonLegAngleDeg,
		    static_cast<float>(input.common_theta / kDegToRad));
	set_channel(TelemetryChannel::kLeftSentWheelTorque,
		    static_cast<float>(output.sent_wheel_torque.left));
	set_channel(TelemetryChannel::kRightSentWheelTorque,
		    static_cast<float>(output.sent_wheel_torque.right));
	set_channel(TelemetryChannel::kPitchDeg, static_cast<float>(input.pitch / kDegToRad));
	set_channel(TelemetryChannel::kLeftBodyOnLegTorque,
		    static_cast<float>(output.body_on_leg_torque.left));
	set_channel(TelemetryChannel::kRightBodyOnLegTorque,
		    static_cast<float>(output.body_on_leg_torque.right));
	set_channel(TelemetryChannel::kLegAngleWheelContribution,
		    static_cast<float>(output.theta_wheel_torque_contribution));
	set_channel(TelemetryChannel::kPitchWheelContribution,
		    static_cast<float>(output.pitch_wheel_torque_contribution));
	set_channel(TelemetryChannel::kTargetLegLength, static_cast<float>(target_leg_length_));
	set_channel(TelemetryChannel::kLeftLegLength, static_cast<float>(input.leg.left.length));
	set_channel(TelemetryChannel::kRightLegLength, static_cast<float>(input.leg.right.length));
	set_channel(TelemetryChannel::kImuMaxSensorIntervalMs,
		    static_cast<float>(input.imu.max_sensor_interval_ms));
	set_channel(TelemetryChannel::kImuCrcErrorCount,
		    static_cast<float>(input.imu.crc_error_count));
	set_channel(TelemetryChannel::kImuGapWithCrcCount,
		    static_cast<float>(input.imu.publish_gap_with_crc_count));
	set_channel(TelemetryChannel::kImuGapWithoutCrcCount,
		    static_cast<float>(input.imu.publish_gap_without_crc_count));
	set_channel(TelemetryChannel::kDmArmResetReason,
		    static_cast<float>(dm_arm_reset_reason_));
	set_channel(TelemetryChannel::kImuAgeUs,
		    static_cast<float>(std::min<uint64_t>(input.imu_age_us, UINT32_MAX)));
	set_channel(TelemetryChannel::kImuMaxPublishIntervalUs,
		    static_cast<float>(input.imu.max_publish_interval_us));
	const uint32_t imu_transport_error_count = input.imu.rx_drop_count +
						   input.imu.rx_stop_count +
						   input.imu.rx_buf_rsp_error_count;
	set_channel(TelemetryChannel::kImuTransportErrorCount,
		    static_cast<float>(imu_transport_error_count));
	channels::latest_oscilloscope_sample.write(sample);
}

void ChassisModule::RunLoop()
{
	LOG_INF("chassis started");

	CycleInput input = {};
	wbr_control::scheduling::AbsolutePeriodicSchedule release(
		kControlPeriodMs, wbr_control::scheduling::thread_phase_ms::kChassis);

	for (;;) {
		(void)release.WaitForNextRelease();
		deadline_miss_count_ = release.total_missed_releases();

		// 使用实际周期并限制异常调度延迟，避免冲击积分器和滤波器。
		const uint64_t now_us = k_cyc_to_us_floor64(k_cycle_get_64());
		const uint32_t now_ms = k_uptime_get_32();
		double dt = kDefaultDt;
		if (last_loop_time_us_ != 0U && now_us > last_loop_time_us_) {
			dt = std::clamp(static_cast<double>(now_us - last_loop_time_us_) * 1.0e-6,
					0.0005, 0.01);
		}
		last_loop_time_us_ = now_us;

			// 单周期流水线：输入快照 -> FSM -> 控制计算 -> 执行器 -> 遥测。
			CycleOutput output = {};
			ReadCycleInput(input, now_ms, dt);
			UpdateControlState(input);
			ComputeControlOutput(input, output);
			ApplyControlOutput(output);
			PublishTelemetry(input, output);
		++loop_ticks_;
	}
}
} // namespace modules
