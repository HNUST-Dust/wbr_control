/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file src/modules/chassis/body_motion_estimator.h
 * @ingroup wbr_modules
 * @brief 根据底盘传感器数据估计机体运动状态。
 * @details 模块遵循 `ModuleBase` 生命周期：`Start()` 只负责一次性资源初始化和线程创建，`RunLoop()` 持有周期状态。跨线程数据通过 channels 层交换。
 */

#pragma once

#include <array>

namespace modules
{

/** @brief 机体运动估计器输出的速度和姿态状态。 */
struct BodyMotionState {
	double speed = 0.0; ///< 卡尔曼滤波后的车体前向速度，单位为米每秒。
	double acceleration = 0.0; ///< 当前线加速度，单位为米每二次方秒。
	double position = 0.0; ///< 由估计速度积分得到的车体前向位置，单位为米。
};

/*
 * SPR 风格的二状态卡尔曼估计器。
 *
 * 卡尔曼状态为 [前向速度, 前向加速度]。位置由估计速度独立连续积分，
 * 使用方负责限制送入控制器的位置误差。
 */
/** @brief 融合底盘反馈并估计机体运动状态。 */
class BodyMotionEstimator
{
public:
	/**
	 * @brief 清空内部状态并恢复到初始条件。
	 */
	void Reset();
	/**
	 * @brief 使用当前采样更新内部状态并返回本周期结果。
	 * @param speed_measurement 车体前向速度测量值，单位为米每秒。
	 * @param acceleration_measurement 车体前向加速度测量值，单位为米每二次方秒。
	 * @param dt 相邻两次更新的时间间隔，单位为秒且必须为正。
	 * @return 内部最新运动状态的常量引用，在下一次 `Update()` 前保持有效。
	 */
	const BodyMotionState &Update(double speed_measurement, double acceleration_measurement,
				      double dt);

private:
	BodyMotionState state_;
	std::array<double, 4> covariance_ = {1.0, 0.0, 0.0, 1.0};
};

} // namespace modules
