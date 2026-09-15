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
#include <zephyr/sys/util.h>

#include <msg/chassis_realtime_status.hpp>
#include <msg/oscilloscope_sample.hpp>
#include <protocols/motors/dm_motor_protocol.h>
#include <scheduling/periodic_schedule.h>
#include <scheduling/thread_priorities.h>
#include "chassis_config.h"
#include "chassis_imu_adapter.h"

LOG_MODULE_REGISTER(chassis_module, LOG_LEVEL_INF);

namespace
{

K_THREAD_STACK_DEFINE(g_chassis_stack, 4096);

using namespace modules::chassis_config;

enum class ChassisJoint : uint8_t {
	kLeftB = 0U,
	kLeftD,
	kRightB,
	kRightD,
	kCount,
};

template <typename Enum> constexpr size_t ToIndex(Enum value)
{
	return static_cast<size_t>(value);
}

static_assert(ToIndex(ChassisJoint::kCount) == 4U);
static_assert(kTelemetryChannelCount <= msg::kOscilloscopeMaxChannels);
} // namespace

namespace modules
{

ChassisModule::ChassisModule()
{
	loop_ticks_ = 0U;
	last_loop_time_us_ = 0U;
	deadline_miss_count_ = 0U;
	realtime_status_sequence_ = 0U;
	max_loop_execution_us_ = 0U;
	loop_period_us_ = 0U;
	min_loop_period_us_ = UINT32_MAX;
	max_loop_period_us_ = 0U;
	stool_controller_.Reset();
	ResetControlState();
}

int ChassisModule::Start()
{
	InitializeChassisImuAdapter();
	return CreateThread(
		g_chassis_stack, K_THREAD_STACK_SIZEOF(g_chassis_stack),
		K_PRIO_PREEMPT(wbr_control::scheduling::thread_priority::kChassis), "chassis");
}

void ChassisModule::UpdateControlState(const ChassisCycleInput &input)
{
	const ChassisStateTransition transition = state_machine_.Update(input);
	if (transition.enable_changed) {
		ResetControlState();
		actuator_.ResetArming();
		stool_controller_.Reset();
		balance_controller_.ResetForEnable();
	}
}

void ChassisModule::ComputeControlOutput(const ChassisCycleInput &input,
					 ChassisControlOutput &output)
{
	output = {};
	if (state_machine_.state() == ChassisControlState::kStool) {
		// 撑起状态先收腿，再执行腿角对齐和关节级联 PID。
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
		balance_controller_.set_target_leg_length(stool.leg_length_reference);
		output.joint_torque.left.b = stool.joint_torque[ToIndex(ChassisJoint::kLeftB)];
		output.joint_torque.left.d = stool.joint_torque[ToIndex(ChassisJoint::kLeftD)];
		output.joint_torque.right.b = stool.joint_torque[ToIndex(ChassisJoint::kRightB)];
		output.joint_torque.right.d = stool.joint_torque[ToIndex(ChassisJoint::kRightD)];
		if (stool.ready) {
			// 本周期保留撑起控制器的关节力矩，下周期开始运行完整 LQR。
			state_machine_.MarkBalanceReached();
			balance_controller_.SynchronizeReferences(input);
			LOG_INF("stool ready; entering full LQR with synchronized attitude references");
		}
		return;
	}

	if (state_machine_.state() != ChassisControlState::kBalance) {
		return;
	}

	balance_controller_.Update(input, output);
}

void ChassisModule::ApplyControlOutput(ChassisControlOutput &output)
{
	if (actuator_.Apply(state_machine_.state(), loop_ticks_, output)) {
		ResetControlState();
	}
}

void ChassisModule::ResetControlState()
{
	balance_controller_.Reset();
}


void ChassisModule::PublishTelemetry(const ChassisCycleInput &input,
				     const ChassisControlOutput &output)
{
	// 保持既有 VOFA 状态码，DM 重新使能原因由独立通道输出。
	float balance_state_x10 = 0.0F;
	const bool control_enabled = input.requested_enable && input.feedback_valid &&
				     !state_machine_.tilt_fault_latched() && input.arm_complete &&
				     input.dm_ready;
	if (input.requested_enable) {
		balance_state_x10 =
			state_machine_.tilt_fault_latched()
				? 40.0F
				: (!input.arm_complete
					   ? 11.0F
					   : (!input.feedback_valid
						      ? 12.0F
						      : (!control_enabled
								 ? 13.0F
								 : (state_machine_.state() == ChassisControlState::
											      kStool
									    ? 20.0F
									    : 30.0F))));
	}

	msg::OscilloscopeSample sample = {};
	sample.sequence = ++telemetry_sequence_;
	sample.uptime_ms = k_uptime_get_32();
	sample.channel_count = kTelemetryChannelCount;

	sample.value[0] = balance_state_x10;
	sample.value[1] = input.common_theta / kDegToRad;
	sample.value[2] = input.common_theta_rate;
	sample.value[3] = input.pitch / kDegToRad;
	sample.value[4] = input.pitch_rate;
	sample.value[5] = input.leg.left.length;
	sample.value[6] = input.leg.left.length_rate;
	sample.value[7] = input.leg.right.length;
	sample.value[8] = input.leg.right.length_rate;
	sample.value[9] = balance_controller_.target_leg_length();
	sample.value[10] = output.physical_wheel_torque.left;
	sample.value[11] = output.physical_wheel_torque.right;
	sample.value[12] = output.sent_wheel_torque.left;
	sample.value[13] = output.sent_wheel_torque.right;
	sample.value[14] = input_reader_.wheel_feedback().left.current;
	sample.value[15] = input_reader_.wheel_feedback().right.current;
	sample.value[16] = output.joint_torque.left.b;
	sample.value[17] = output.joint_torque.left.d;
	sample.value[18] = output.joint_torque.right.b;
	sample.value[19] = output.joint_torque.right.d;
	sample.value[20] = protocols::DmFeedbackTorque(input_reader_.joint_feedback().left.b,
						 kDmRange);
	sample.value[21] = protocols::DmFeedbackTorque(input_reader_.joint_feedback().left.d,
						 kDmRange);
	sample.value[22] = protocols::DmFeedbackTorque(input_reader_.joint_feedback().right.b,
						 kDmRange);
	sample.value[23] = protocols::DmFeedbackTorque(input_reader_.joint_feedback().right.d,
						 kDmRange);
	sample.value[24] = output.raw_body_speed;
	sample.value[25] = output.estimated_body_speed;
	sample.value[26] = input.feedback_valid ? 1.0 : 0.0;
	// 27 以后只保留遥控运动和转向解耦调试所需的紧凑通道。
	sample.value[27] = input.remote.chassis_x;
	sample.value[28] = input.remote.chassis_rotate;
	sample.value[29] = balance_controller_.target_yaw_rate();
	// 发布控制器实际使用的滤波值，便于直接核对目标与反馈。
	sample.value[30] = balance_controller_.filtered_yaw_rate();
	sample.value[31] = std::remainder(static_cast<double>(input.imu.yaw_deg) * kDegToRad -
					 balance_controller_.target_yaw_angle(), kTwoPi) /
			   kDegToRad;
	sample.value[32] = output.measured_turn_wheel_speed;
	sample.value[33] = output.allocated_turn_torque;
	sample.value[34] =
		std::remainder(input.leg.left.angle - input.leg.right.angle, kTwoPi) /
		kDegToRad;
	sample.value[35] = input.leg.left.angle_rate - input.leg.right.angle_rate;
	sample.value[36] = output.differential_leg_torque;
	msg::latest_oscilloscope_sample.write(sample);
}

void ChassisModule::PublishRealtimeStatus(const ChassisCycleInput &input,
						  uint32_t loop_start_cycle)
{
	msg::ChassisRealtimeStatus status = {};
	status.sequence = ++realtime_status_sequence_;
	status.uptime_ms = k_uptime_get_32();
	status.deadline_miss_count = deadline_miss_count_;
	size_t unused_bytes = 0U;
	if (k_thread_stack_space_get(&thread_, &unused_bytes) == 0) {
		status.stack_unused_bytes = static_cast<uint32_t>(unused_bytes);
	}
	status.imu_age_us = input.imu_age_us;
	status.imu_source = static_cast<uint8_t>(SelectedChassisImuSource());
	status.imu_fresh = input.imu_fresh;
	/* Capture after the periodic stack query and status construction so the
	 * diagnostic cycle does not report an artificially optimistic WCET.  Only
	 * the final bounded channel copy remains outside this interval.
	 */
	status.loop_execution_us =
		k_cyc_to_us_floor32(k_cycle_get_32() - loop_start_cycle);
	max_loop_execution_us_ = MAX(max_loop_execution_us_, status.loop_execution_us);
	status.max_loop_execution_us = max_loop_execution_us_;
	status.loop_period_us = loop_period_us_;
	status.min_loop_period_us = min_loop_period_us_ == UINT32_MAX ? 0U : min_loop_period_us_;
	status.max_loop_period_us = max_loop_period_us_;
	msg::latest_chassis_realtime_status.write(status);
}

void ChassisModule::RunLoop()
{
	LOG_INF("chassis started: imu_source=%u timeout=%u ms",
		static_cast<unsigned int>(SelectedChassisImuSource()),
		static_cast<unsigned int>(kImuTimeoutMs));

	ChassisCycleInput input = {};
	wbr_control::scheduling::AbsolutePeriodicSchedule release(
		kControlPeriodMs, wbr_control::scheduling::thread_phase_ms::kChassis);

	for (;;) {
		(void)release.WaitForNextRelease();
		deadline_miss_count_ = release.total_missed_releases();
		const uint32_t loop_start_cycle = k_cycle_get_32();

		// 使用实际周期并限制异常调度延迟，避免冲击积分器和滤波器。
		const uint64_t now_us = k_cyc_to_us_floor64(k_cycle_get_64());
		if (last_loop_time_us_ != 0U && now_us > last_loop_time_us_) {
			loop_period_us_ = static_cast<uint32_t>(now_us - last_loop_time_us_);
			min_loop_period_us_ = MIN(min_loop_period_us_, loop_period_us_);
			max_loop_period_us_ = MAX(max_loop_period_us_, loop_period_us_);
		}
		const uint32_t now_ms = k_uptime_get_32();
		double dt = kDefaultDt;
		if (last_loop_time_us_ != 0U && now_us > last_loop_time_us_) {
			dt = std::clamp(static_cast<double>(now_us - last_loop_time_us_) * 1.0e-6,
					0.0005, 0.01);
		}
		last_loop_time_us_ = now_us;

		// 单周期流水线：输入快照 -> FSM -> 控制计算 -> 执行器 -> 遥测。
		ChassisControlOutput output = {};
		input_reader_.Read(input, now_ms, dt, actuator_.arm_complete());
		UpdateControlState(input);
		ComputeControlOutput(input, output);
		ApplyControlOutput(output);
		PublishTelemetry(input, output);
		++loop_ticks_;
		if ((loop_ticks_ % kRealtimeStatusPeriodTicks) == 0U) {
			PublishRealtimeStatus(input, loop_start_cycle);
		} else {
			const uint32_t loop_execution_us =
				k_cyc_to_us_floor32(k_cycle_get_32() - loop_start_cycle);
			max_loop_execution_us_ = MAX(max_loop_execution_us_, loop_execution_us);
		}
	}
}
} // namespace modules
