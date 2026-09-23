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
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <msg/chassis_realtime_status.hpp>
#include <msg/oscilloscope_sample.hpp>
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

const char *StateName(modules::ChassisControlState state)
{
	switch (state) {
	case modules::ChassisControlState::kDisabled: return "disabled";
	case modules::ChassisControlState::kSafetyStop: return "safety_stop";
	case modules::ChassisControlState::kDmArming: return "dm_arming";
	case modules::ChassisControlState::kWaitingFeedback: return "waiting_feedback";
	case modules::ChassisControlState::kRecovery: return "recovery";
	case modules::ChassisControlState::kBalance: return "balance";
	case modules::ChassisControlState::kFlight: return "flight";
	case modules::ChassisControlState::kJump: return "jump";
	case modules::ChassisControlState::kClimbStairs: return "climb_stairs";
	case modules::ChassisControlState::kActionFault: return "action_fault";
	case modules::ChassisControlState::kTiltFault: return "tilt_fault";
	}
	return "unknown";
}

const char *TransitionReasonName(modules::ChassisTransitionReason reason)
{
	switch (reason) {
	case modules::ChassisTransitionReason::kNone: return "none";
	case modules::ChassisTransitionReason::kRemoteDisabled: return "remote_disabled";
	case modules::ChassisTransitionReason::kRemoteTimeout: return "remote_timeout";
	case modules::ChassisTransitionReason::kRemoteEnabled: return "remote_enabled";
	case modules::ChassisTransitionReason::kTiltExceeded: return "tilt_exceeded";
	case modules::ChassisTransitionReason::kImuLost: return "imu_lost";
	case modules::ChassisTransitionReason::kImuRecovered: return "imu_recovered";
	case modules::ChassisTransitionReason::kDmNotReady: return "dm_not_ready";
	case modules::ChassisTransitionReason::kArmingCompleted: return "arming_completed";
	case modules::ChassisTransitionReason::kFeedbackLost: return "feedback_lost";
	case modules::ChassisTransitionReason::kFeedbackRecovered: return "feedback_recovered";
	case modules::ChassisTransitionReason::kRecoveryStarted: return "recovery_started";
	case modules::ChassisTransitionReason::kRecoveryCompleted: return "recovery_completed";
	case modules::ChassisTransitionReason::kRecoveryFailed: return "recovery_failed";
	case modules::ChassisTransitionReason::kFlightDetected: return "flight_detected";
	case modules::ChassisTransitionReason::kFlightLanded: return "flight_landed";
	case modules::ChassisTransitionReason::kFlightFailed: return "flight_failed";
	case modules::ChassisTransitionReason::kJumpRequested: return "jump_requested";
	case modules::ChassisTransitionReason::kJumpCompleted: return "jump_completed";
	case modules::ChassisTransitionReason::kJumpFailed: return "jump_failed";
	case modules::ChassisTransitionReason::kClimbRequested: return "climb_requested";
	case modules::ChassisTransitionReason::kClimbCompleted: return "climb_completed";
	case modules::ChassisTransitionReason::kClimbFailed: return "climb_failed";
	}
	return "unknown";
}
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
	recovery_controller_.Reset();
	flight_controller_.Reset();
	jump_controller_.Reset();
	climb_stairs_controller_.Reset();
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
	if (transition.enable_changed && transition.enabled) {
		// Preserve enable-edge initialization even when a simultaneous safety
		// condition sends the first enabled cycle directly to a stop state.
		balance_controller_.ResetForEnable();
	}
	if (!transition.changed) {
		return;
	}

	LOG_INF("chassis state %s -> %s reason=%s", StateName(transition.from),
		StateName(transition.to), TransitionReasonName(transition.reason));

	// State-entry actions live here; the state machine itself remains a pure
	// transition-policy owner and never touches controllers or hardware.
	switch (transition.to) {
	case ChassisControlState::kDisabled:
	case ChassisControlState::kSafetyStop:
	case ChassisControlState::kActionFault:
	case ChassisControlState::kTiltFault:
		ResetControlState();
		actuator_.ResetArming();
		recovery_controller_.Reset();
		flight_controller_.Reset();
		jump_controller_.Reset();
		climb_stairs_controller_.Reset();
		break;
	case ChassisControlState::kDmArming:
		ResetControlState();
		actuator_.ResetArming();
		recovery_controller_.Reset();
		flight_controller_.Reset();
		jump_controller_.Reset();
		climb_stairs_controller_.Reset();
		break;
	case ChassisControlState::kWaitingFeedback:
		ResetControlState();
		break;
	case ChassisControlState::kRecovery:
		ResetControlState();
		recovery_controller_.Reset();
		break;
	case ChassisControlState::kBalance:
		if (transition.from == ChassisControlState::kRecovery ||
		    transition.from == ChassisControlState::kClimbStairs ||
		    transition.from == ChassisControlState::kFlight ||
		    transition.from == ChassisControlState::kJump) {
			balance_controller_.SynchronizeReferences(input);
			recovery_controller_.Reset();
			climb_stairs_controller_.Reset();
			flight_controller_.Reset();
			jump_controller_.Reset();
		}
		break;
	case ChassisControlState::kFlight:
		flight_controller_.Reset();
		break;
	case ChassisControlState::kJump:
		balance_controller_.SynchronizeReferences(input);
		jump_controller_.Reset();
		break;
	case ChassisControlState::kClimbStairs:
		climb_stairs_controller_.Reset();
		break;
	}
}

void ChassisModule::ComputeControlOutput(const ChassisCycleInput &input,
					 ChassisControlOutput &output)
{
	output = {};
	if (state_machine_.state() == ChassisControlState::kRecovery) {
		RecoveryControllerInput recovery_input = {};
		recovery_input.dt = input.dt;
		recovery_input.pitch = input.pitch;
		recovery_input.roll = input.roll;
		recovery_input.vertical_accel_g = input.imu.vertical_accel_g;
		recovery_input.leg = {input.leg.left, input.leg.right};
		recovery_input.joint_position = {
			input.joint_position.left.b,
			input.joint_position.left.d,
			input.joint_position.right.b,
			input.joint_position.right.d,
		};
		recovery_input.joint_velocity = {
			input.joint_velocity.left.b,
			input.joint_velocity.left.d,
			input.joint_velocity.right.b,
			input.joint_velocity.right.d,
		};
		// 当前工程没有云台；这些默认值构成后续接入云台反馈的接口边界。
		recovery_input.gimbal.available = false;
		recovery_input.gimbal.safe_for_leg_motion = true;
		recovery_input.gimbal.aligned = true;
		const RecoveryControllerOutput recovery = recovery_controller_.Update(recovery_input);
		output.joint_torque.left.b = recovery.joint_torque[ToIndex(ChassisJoint::kLeftB)];
		output.joint_torque.left.d = recovery.joint_torque[ToIndex(ChassisJoint::kLeftD)];
		output.joint_torque.right.b = recovery.joint_torque[ToIndex(ChassisJoint::kRightB)];
		output.joint_torque.right.d = recovery.joint_torque[ToIndex(ChassisJoint::kRightD)];
		if (recovery.failed) {
			state_machine_.MarkRecoveryFailed();
		} else if (recovery.finished) {
			state_machine_.MarkRecoveryCompleted();
		}
		return;
	}

	if (state_machine_.state() != ChassisControlState::kBalance) {
		if (state_machine_.state() == ChassisControlState::kJump) {
			const JumpControllerOutput jump =
				jump_controller_.Update(input, balance_controller_);
			output = jump.control;
			if (jump.failed) {
				state_machine_.MarkJumpFailed();
			} else if (jump.finished) {
				state_machine_.MarkJumpCompleted();
			}
			return;
		}
		if (state_machine_.state() == ChassisControlState::kFlight) {
			const FlightControllerOutput flight = flight_controller_.Update(input);
			output = flight.control;
			if (flight.failed) {
				state_machine_.MarkFlightFailed();
			} else if (flight.finished) {
				state_machine_.MarkFlightCompleted();
			}
			return;
		}
		if (state_machine_.state() != ChassisControlState::kClimbStairs) {
			return;
		}
		ClimbStairsControllerInput climb_input = {};
		climb_input.dt = input.dt;
		climb_input.pitch = input.pitch;
		climb_input.leg = {input.leg.left, input.leg.right};
		climb_input.joint_position = {
			input.joint_position.left.b, input.joint_position.left.d,
			input.joint_position.right.b, input.joint_position.right.d,
		};
		climb_input.joint_velocity = {
			input.joint_velocity.left.b, input.joint_velocity.left.d,
			input.joint_velocity.right.b, input.joint_velocity.right.d,
		};
		const ClimbStairsControllerOutput climb =
			climb_stairs_controller_.Update(climb_input);
		output.joint_torque.left.b = climb.joint_torque[ToIndex(ChassisJoint::kLeftB)];
		output.joint_torque.left.d = climb.joint_torque[ToIndex(ChassisJoint::kLeftD)];
		output.joint_torque.right.b = climb.joint_torque[ToIndex(ChassisJoint::kRightB)];
		output.joint_torque.right.d = climb.joint_torque[ToIndex(ChassisJoint::kRightD)];
		if (climb.failed) {
			state_machine_.MarkClimbFailed();
		} else if (climb.finished) {
			state_machine_.MarkClimbCompleted();
		}
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
				     const ChassisControlOutput &output,
				     const ChassisSupportForceEstimate &support_force)
{
	// 保持既有 VOFA 状态码，DM 重新使能原因由独立通道输出。
	float balance_state_x10 = 0.0F;
	const bool control_enabled = input.requested_enable && input.feedback_valid &&
				     !state_machine_.tilt_fault_latched() && input.arm_complete &&
				     input.dm_ready;
	if (input.requested_enable) {
		if (state_machine_.tilt_fault_latched()) {
			balance_state_x10 = 40.0F;
		} else if (state_machine_.state() == ChassisControlState::kActionFault) {
			balance_state_x10 = 50.0F;
		} else if (!input.arm_complete) {
			balance_state_x10 = 11.0F;
		} else if (!input.feedback_valid) {
			balance_state_x10 = 12.0F;
		} else if (!control_enabled) {
			balance_state_x10 = 13.0F;
		} else if (state_machine_.state() == ChassisControlState::kRecovery) {
			balance_state_x10 = 20.0F;
		} else if (state_machine_.state() == ChassisControlState::kClimbStairs) {
			balance_state_x10 = 35.0F;
		} else if (state_machine_.state() == ChassisControlState::kFlight) {
			balance_state_x10 = 32.0F;
		} else if (state_machine_.state() == ChassisControlState::kJump) {
			balance_state_x10 = 33.0F;
		} else {
			balance_state_x10 = 30.0F;
		}
	}

	msg::OscilloscopeSample sample = {};
	sample.sequence = ++telemetry_sequence_;
	sample.uptime_ms = k_uptime_get_32();
	sample.channel_count = 20;

	sample.value[0] = balance_state_x10;
	// 腿长闭环。
	sample.value[1] = balance_controller_.target_leg_length();
	sample.value[2] = input.leg.left.length;
	sample.value[3] = input.leg.right.length;
	sample.value[4] = input.leg.left.length_rate;
	sample.value[5] = input.leg.right.length_rate;
	// 横滚补偿和俯仰姿态。
	sample.value[6] = input.roll / kDegToRad;
	sample.value[7] = output.roll_reference / kDegToRad;
	sample.value[8] = output.roll_compensation_force;
	sample.value[9] = input.pitch / kDegToRad;
	// 行驶状态、目标/实测 yaw 速度和物理输出。
	sample.value[10] = output.estimated_body_speed;
	sample.value[11] = balance_controller_.target_yaw_rate();
	sample.value[12] = balance_controller_.filtered_yaw_rate();
	sample.value[13] = output.physical_wheel_torque.left;
	sample.value[14] = output.physical_wheel_torque.right;
	sample.value[15] = output.leg_axial_force.left;
	sample.value[16] = output.leg_axial_force.right;
	sample.value[17] = support_force.filtered_force_n.left;
	sample.value[18] = support_force.filtered_force_n.right;
	// 动作阶段按十位区分：Flight=10+, Jump=20+, Climb=30+, Recovery=40+。
	float action_phase = 0.0F;
	if (state_machine_.state() == ChassisControlState::kFlight) {
		action_phase = 10.0F + static_cast<float>(flight_controller_.phase());
	} else if (state_machine_.state() == ChassisControlState::kJump) {
		action_phase = 20.0F + static_cast<float>(jump_controller_.phase());
	} else if (state_machine_.state() == ChassisControlState::kClimbStairs) {
		action_phase = 30.0F + static_cast<float>(climb_stairs_controller_.phase());
	} else if (state_machine_.state() == ChassisControlState::kRecovery) {
		action_phase = 40.0F + static_cast<float>(recovery_controller_.phase());
	}
	sample.value[19] = action_phase;
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
		const ChassisSupportForceEstimate support_force =
			support_force_estimator_.Update(input);
		const bool flight_detected = flight_controller_.UpdateLiftoffDetection(
			input, support_force,
			state_machine_.state() == ChassisControlState::kBalance);
		state_machine_.SetFlightDetected(flight_detected);
		UpdateControlState(input);
		ComputeControlOutput(input, output);
		ApplyControlOutput(output);
		PublishTelemetry(input, output, support_force);
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
