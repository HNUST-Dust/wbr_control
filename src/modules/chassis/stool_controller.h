/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file src/modules/chassis/stool_controller.h
 * @ingroup wbr_modules
 * @brief 实现凳式姿态下的底盘稳定控制。
 * @details 模块遵循 `ModuleBase` 生命周期：`Start()` 只负责一次性资源初始化和线程创建，`RunLoop()` 持有周期状态。跨线程数据通过 channels 层交换。
 */

#pragma once

#include <array>
#include <cstdint>

#include "leg_kinematics.h"

namespace modules
{

/** @brief 凳式姿态控制器的状态与目标输入。 */
struct StoolControllerInput {
	double dt = 0.001; ///< 控制周期，单位为秒且必须大于零。
	double pitch = 0.0; ///< 机体俯仰角，单位为弧度。
	std::array<LegKinematics, 2> leg{}; ///< 左、右腿当前五连杆正运动学结果。
	std::array<double, 4> joint_position{}; ///< 左 B、左 D、右 B、右 D 关节角，单位为弧度。
	std::array<double, 4> joint_velocity{}; ///< 同顺序的关节角速度，单位为弧度每秒。
};

/** @brief 凳式姿态控制器计算的执行器输出。 */
struct StoolControllerOutput {
	std::array<double, 4> joint_torque{}; ///< 同一关节顺序的目标力矩，单位为牛·米。
	bool target_initialized = false; ///< 平滑控制目标已经由当前状态对齐的标志。
	bool ready = false; ///< 模块已完成初始化并可提供有效数据的标志。
};

/** @brief 维持机器人凳式姿态稳定的控制器。 */
class StoolController
{
public:
	static constexpr double kTargetLegLength = 0.150; ///< `kTargetLegLength` 几何长度参数，单位为米。

	/**
	 * @brief 清空内部状态并恢复到初始条件。
	 */
	void Reset();
	/**
	 * @brief 使用当前采样更新内部状态并返回本周期结果。
	 * @param[in] input 本周期使用的只读输入快照。
	 * @return 四个关节的目标力矩及目标初始化、对齐完成标志。
	 */
	StoolControllerOutput Update(const StoolControllerInput &input);

private:
	/** @brief 单个关节 PID 控制器的积分与历史误差。 */
	struct JointPidState {
		double position_integral = 0.0; ///< 由前向速度连续积分得到的位置估计，单位为米。
		double speed_integral = 0.0; ///< 线速度，单位为米每秒。
	};

	/**
	 * @brief 由关节速度采样更新滤波状态。
	 * @param[in] velocity 四个关节的速度数组，单位为弧度每秒。
	 * @param dt 相邻两次更新的时间间隔，单位为秒且必须为正。
	 */
	void UpdateJointVelocity(const std::array<double, 4> &velocity, double dt);
	/**
	 * @brief 根据控制输入更新凳式姿态目标。
	 * @param[in] input 本周期使用的只读输入快照。
	 */
	void UpdatePoseTarget(const StoolControllerInput &input);
	/**
	 * @brief 将归一化关节角还原为机构原始角度。
	 * @param side 目标机构所在的左侧或右侧。
	 * @param normalized_angle 去除安装方向后的归一化关节角，单位为弧度。
	 * @return 考虑左右安装方向后的原始机构角，单位为弧度。
	 */
	static double RawKinematicAngle(uint8_t side, double normalized_angle);
	/**
	 * @brief 计算单个关节的 PID 目标力矩。
	 * @param index 关节在控制器数组中的索引。
	 * @param target 目标位置或角度。
	 * @param position 当前关节位置，单位为弧度。
	 * @param velocity 四个关节的速度数组，单位为弧度每秒。
	 * @param dt 相邻两次更新的时间间隔，单位为秒且必须为正。
	 * @return 经积分、微分和限幅后的目标关节力矩，单位为牛·米。
	 */
	double ComputeJointTorque(uint8_t index, double target, double position, double velocity,
				  double dt);

	std::array<bool, 2> target_initialized_{};
	bool velocity_initialized_ = false;
	std::array<double, 4> joint_target_{};
	std::array<double, 4> filtered_joint_velocity_{};
	std::array<JointPidState, 4> joint_pid_{};
};

} // namespace modules
