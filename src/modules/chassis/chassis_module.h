/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>

#include <zephyr/kernel.h>

#include "body_motion_estimator.h"
#include "leg_kinematics.h"
#include "stool_controller.h"
#include "../module_base.h"
#include <protocols/motors/dji_motor_protocol.h>
#include <protocols/motors/dm_motor_protocol.h>

namespace modules
{

enum class Side : uint8_t {
	kLeft = 0U,
	kRight,
};

enum class Joint : uint8_t {
	kB = 0U,
	kD,
};

template <typename T> struct SidePair {
	T left{};
	T right{};

	T &Get(Side side)
	{
		return side == Side::kLeft ? left : right;
	}

	const T &Get(Side side) const
	{
		return side == Side::kLeft ? left : right;
	}
};

template <typename T> struct JointPair {
	T b{};
	T d{};

	T &Get(Joint joint)
	{
		return joint == Joint::kB ? b : d;
	}

	const T &Get(Joint joint) const
	{
		return joint == Joint::kB ? b : d;
	}
};

class ChassisModule : public ModuleBase
{
public:
	ChassisModule();
	int Start() override;
	void RunLoop() override;

private:
	// 底盘控制有限状态机。状态同时决定本周期允许执行的控制算法和输出策略：
	// 停机 -> DM 使能 -> 等待反馈 -> 撑起对齐 -> 平衡控制。
	// 撤销使能、IMU 失效和倾倒故障具有更高优先级，可从任意状态切走。
	enum class ControlState : uint8_t {
		kDisabled = 0U,
		kSafetyStop,
		kDmArming,
		kWaitingFeedback,
		kStool,
		kBalance,
		kTiltFault,
	};

	struct SideState {
		double leg_length_integral_force = 0.0;
	};

	struct CycleInput;
	struct CycleOutput;

	// 以下函数严格按照 RunLoop() 中的调用顺序排列。
	void ReadCycleInput(CycleInput &input, uint32_t now_ms, double dt);
	void UpdateControlState(const CycleInput &input);
	void ComputeControlOutput(const CycleInput &input, CycleOutput &output);
	void ApplyControlOutput(CycleOutput &output);
	void PublishTelemetry(const CycleInput &input, const CycleOutput &output);

	void ResetControlState();
	void SendDmControl(protocols::DmControlCommand command);
	void SendScheduledOutputs(const SidePair<JointPair<double>> &joint_torque,
				  const SidePair<int16_t> &wheel_current);
	void SendDmTorque(Side side, Joint joint, double torque);
	void SendWheelCurrent(Side side, int16_t current);

	bool last_requested_enable_ = false;
	bool tilt_fault_latched_ = false;
	uint32_t loop_ticks_ = 0U;
	uint32_t dm_arm_ticks_ = 0U;
	uint32_t last_remote_sequence_ = 0U;
	uint32_t last_remote_update_ms_ = 0U;
	uint32_t last_imu_sequence_ = 0U;
	uint64_t last_loop_time_us_ = 0U;
	uint32_t deadline_miss_count_ = 0U;
	ControlState control_state_ = ControlState::kDisabled;
	bool balance_phase_reached_ = false;
	StoolController stool_controller_;
	BodyMotionEstimator body_motion_estimator_;
	bool stool_ready_ = false;
	double target_leg_length_ = StoolController::kTargetLegLength;
	SidePair<SideState> side_state_;
	SidePair<protocols::DjiMotorFeedback> wheel_feedback_;
	SidePair<JointPair<protocols::DmMotorFeedbackNormal>> joint_feedback_;
};

} // namespace modules
