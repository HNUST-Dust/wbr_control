/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file src/modules/chassis/chassis_module.h
 * @ingroup wbr_modules
 * @brief 实现轮腿底盘控制模块及周期控制流程。
 * @details 模块遵循 `ModuleBase` 生命周期：`Start()` 只负责一次性资源初始化和线程创建，`RunLoop()` 持有周期状态。跨线程数据通过 channels 层交换。
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

/** @brief 底盘左右侧标识。 */
enum class Side : uint8_t {
	kLeft = 0U, ///< 机器人左侧。
	kRight, ///< 机器人右侧。
};

/** @brief 单侧腿部关节标识。 */
enum class Joint : uint8_t {
	kB = 0U, ///< 五连杆机构的 B 关节。
	kD, ///< 五连杆机构的 D 关节。
};

/** @brief 按机器人左右侧保存一对同类型对象。 */
template <typename T> struct SidePair {
	T left{}; ///< 左侧对象。
	T right{}; ///< 右侧对象。

	/** @brief 按侧别取得可修改对象。 @param side 目标侧别。 @return 对应侧对象的引用。 */
	T &Get(Side side)
	{
		return side == Side::kLeft ? left : right;
	}

	/** @brief 按侧别取得只读对象。 @param side 目标侧别。 @return 对应侧对象的常量引用。 */
	const T &Get(Side side) const
	{
		return side == Side::kLeft ? left : right;
	}
};

/** @brief 按五连杆 B、D 关节保存一对同类型对象。 */
template <typename T> struct JointPair {
	T b{}; ///< B 关节对象。
	T d{}; ///< D 关节对象。

	/** @brief 按关节标识取得可修改对象。 @param joint 目标关节。 @return 对应关节对象的引用。 */
	T &Get(Joint joint)
	{
		return joint == Joint::kB ? b : d;
	}

	/** @brief 按关节标识取得只读对象。 @param joint 目标关节。 @return 对应关节对象的常量引用。 */
	const T &Get(Joint joint) const
	{
		return joint == Joint::kB ? b : d;
	}
};

/** @brief 执行轮腿底盘状态估计和闭环控制的应用模块。 */
class ChassisModule : public ModuleBase
{
public:
	/** @brief 构造底盘模块并保持执行器处于未使能状态。 */
	ChassisModule();
	/**
	 * @brief 初始化模块资源并创建工作线程。
	 * @return 成功返回 0；初始化或线程创建失败返回负 errno 错误码。
	 */
	int Start() override;
	/**
	 * @brief 执行模块线程的周期主循环。
	 */
	void RunLoop() override;

private:
	// 底盘控制有限状态机。状态同时决定本周期允许执行的控制算法和输出策略：
	// 停机 -> DM 使能 -> 等待反馈 -> 撑起对齐 -> 平衡控制。
	// 撤销使能、IMU 失效和倾倒故障具有更高优先级，可从任意状态切走。
	/** @brief 底盘控制流程当前所处的运行状态。 */
	enum class ControlState : uint8_t {
		kDisabled = 0U, ///< 操作员未使能，所有执行器输出为零。
		kSafetyStop, ///< 安全条件不满足，保持电机停机并等待恢复。
		kDmArming, ///< 正在向达妙关节电机发送进入控制模式命令。
		kWaitingFeedback, ///< 电机已使能，等待 IMU 与全部电机反馈变为新鲜。
		kStool, ///< 使用凳式控制器完成站起前的关节对齐。
		kBalance, ///< 使用轮腿平衡控制器执行正常运动控制。
		kTiltFault, ///< 机体倾角超过安全阈值，锁止输出直至重新使能。
	};

	/** @brief 达妙关节电机重新使能的触发原因。 */
	enum class DmArmResetReason : uint8_t {
		kStartup = 0U, ///< 上电启动后首次执行关节电机使能。
		kRemoteEnableEdge, ///< 检测到遥控器使能命令的上升沿。
		kImuFreshness, ///< IMU 数据由失效恢复为新鲜后重新使能。
		kTiltFault, ///< 倾倒故障解除并重新获得操作员使能。
	};

	/** @brief 单侧轮腿机构在一个控制周期内的聚合状态。 */
	struct SideState {
		double leg_length_integral_force = 0.0; ///< 腿长误差积分产生的轴向力，单位为牛。
	};

	/** @brief 单个底盘控制周期使用的输入快照。 */
	struct CycleInput;
	/** @brief 单个底盘控制周期产生的输出快照。 */
	struct CycleOutput;

	// 以下函数严格按照 RunLoop() 中的调用顺序排列。
	/**
	 * @brief 采集并冻结本控制周期使用的输入快照。
	 * @param[in,out] input 本周期使用的只读输入快照。
	 * @param now_ms 当前单调时钟时间戳，单位为毫秒。
	 * @param dt 相邻两次更新的时间间隔，单位为秒且必须为正。
	 */
	void ReadCycleInput(CycleInput &input, uint32_t now_ms, double dt);
	/**
	 * @brief 根据输入和安全条件更新底盘控制状态机。
	 * @param[in] input 本周期使用的只读输入快照。
	 */
	void UpdateControlState(const CycleInput &input);
	/**
	 * @brief 根据冻结输入计算本周期的全部执行器目标。
	 * @param[in] input 本周期使用的只读输入快照。
	 * @param[out] output 接收本周期计算结果的输出对象。
	 */
	void ComputeControlOutput(const CycleInput &input, CycleOutput &output);
	/**
	 * @brief 对控制输出执行限幅并写入执行器发送通道。
	 * @param[out] output 接收本周期计算结果的输出对象。
	 */
	void ApplyControlOutput(CycleOutput &output);
	/**
	 * @brief 发布本控制周期的状态和诊断遥测。
	 * @param[in] input 本周期使用的只读输入快照。
	 * @param[out] output 接收本周期计算结果的输出对象。
	 */
	void PublishTelemetry(const CycleInput &input, const CycleOutput &output);
	void PublishRealtimeStatus(const CycleInput &input, uint32_t loop_start_cycle);

	/**
	 * @brief 复位控制器、积分器和状态估计历史。
	 */
	void ResetControlState();
	/**
	 * @brief 向全部达妙关节电机发送特殊控制命令。
	 * @param command 待发送的电机特殊控制命令。
	 */
	void SendDmControl(protocols::DmControlCommand command);
	/**
	 * @brief 按照固定 CAN 槽位提交关节力矩和车轮电流。
	 * @param[in] joint_torque 左右腿各关节的目标力矩，单位为牛·米。
	 * @param[in] wheel_current 左右车轮的目标电流，单位为毫安。
	 */
	void SendScheduledOutputs(const SidePair<JointPair<double>> &joint_torque,
				  const SidePair<int16_t> &wheel_current);
	/**
	 * @brief 向指定达妙关节电机提交目标力矩。
	 * @param side 目标机构所在的左侧或右侧。
	 * @param joint 目标腿部关节。
	 * @param torque 目标关节力矩，单位为牛·米。
	 */
	void SendDmTorque(Side side, Joint joint, double torque);
	/**
	 * @brief 向指定侧车轮电机提交目标电流。
	 * @param side 目标机构所在的左侧或右侧。
	 * @param current 目标车轮电流，单位为毫安。
	 */
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
	uint32_t realtime_status_sequence_ = 0U;
	uint32_t max_loop_execution_us_ = 0U;
	ControlState control_state_ = ControlState::kDisabled;
	DmArmResetReason dm_arm_reset_reason_ = DmArmResetReason::kStartup;
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
