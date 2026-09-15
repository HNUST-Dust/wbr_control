/* SPDX-License-Identifier: Apache-2.0 */

#include "chassis_input_reader.h"

#include <cmath>

#include <zephyr/kernel.h>

#include <msg/chassismotors_feedback_raw.hpp>
#include <msg/transport/freshness.hpp>
#include <msg/remote_input_state.hpp>

#include "chassis_config.h"
#include "chassis_imu_adapter.h"
#include "leg_kinematics.h"

namespace modules
{
using namespace chassis_config;

void ChassisInputReader::Read(ChassisCycleInput &input, uint32_t now_ms, double dt,
			      bool arm_complete)
{
	input.dt = dt;

	// 固化六个执行器的原始 CAN 快照，再统一解码。
	msg::left_wheel_feedback_raw.read(input.wheel_frame.left);
	msg::right_wheel_feedback_raw.read(input.wheel_frame.right);
	msg::left_b_motor_feedback_raw.read(input.joint_frame.left.b);
	msg::left_d_motor_feedback_raw.read(input.joint_frame.left.d);
	msg::right_b_motor_feedback_raw.read(input.joint_frame.right.b);
	msg::right_d_motor_feedback_raw.read(input.joint_frame.right.d);

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
	msg::latest_remote_state.read(input.remote);
	if (input.remote.sequence != 0U && input.remote.sequence != last_remote_sequence_) {
		last_remote_sequence_ = input.remote.sequence;
		last_remote_update_ms_ = now_ms;
	}
	input.remote_age_ms = last_remote_sequence_ != 0U
				      ? now_ms - last_remote_update_ms_
				      : UINT32_MAX;
	const bool remote_fresh = input.remote_age_ms <= kRemoteTimeoutMs;
	input.requested_enable = remote_fresh && input.remote.robot_enable;

	ChassisImuSample new_imu = {};
	(void)ReadChassisImuSample(new_imu);
	if (new_imu.sequence != 0U) {
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

	const auto frame_is_fresh = [feedback_now_us](const msg::ChassisMotorFeedbackRawFrame &frame,
						      bool decoded) {
		return decoded && msg::IsFresh(
			frame, &msg::ChassisMotorFeedbackRawFrame::precise_timestamp_us,
			feedback_now_us, kMotorTimeoutUs, false);
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
	input.wheel_motor_speed_rpm.left = static_cast<double>(wheel_feedback_.left.omega);
	input.wheel_motor_speed_rpm.right = static_cast<double>(wheel_feedback_.right.omega);
	input.dm_ready = joint_feedback_.left.b.control_status == 1U &&
			 joint_feedback_.left.d.control_status == 1U &&
			 joint_feedback_.right.b.control_status == 1U &&
			 joint_feedback_.right.d.control_status == 1U;
	input.arm_complete = arm_complete;

	input.pitch = input.imu_fresh ? static_cast<double>(input.imu.pitch_deg) * kDegToRad : 0.0;
	input.pitch_rate = input.imu_fresh
				   ? static_cast<double>(input.imu.pitch_rate_rad_s)
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

} // namespace modules
