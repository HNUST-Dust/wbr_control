/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file src/modules/chassis/leg_vmc.h
 * @ingroup wbr_modules
 * @brief 实现腿部虚拟模型控制的力矩映射。
 * @details 模块遵循 `ModuleBase` 生命周期：`Start()` 只负责一次性资源初始化和线程创建，`RunLoop()` 持有周期状态。跨线程数据通过 channels 层交换。
 */

#pragma once

#include "leg_kinematics.h"

namespace modules
{

/** @brief 腿部虚拟力映射得到的关节力矩。 */
struct LegVmcOutput {
	double axial_force = 0.0; ///< 沿腿轴方向的控制力，单位为牛。
	double phi1_torque = 0.0; ///< 力矩，单位为牛·米。
	double phi2_torque = 0.0; ///< 力矩，单位为牛·米。
};

/**
 * @brief 更新带限幅的腿长误差积分项。
 * @param integral_force 上一周期累计的轴向力积分项。
 * @param length_error 目标腿长减实际腿长的误差，单位为米。
 * @param dt 相邻两次更新的时间间隔，单位为秒且必须为正。
 * @return 经过积分和限幅后的轴向力积分项。
 */
double UpdateLegLengthIntegral(double integral_force, double length_error, double dt);
/**
 * @brief 将期望轴向力和髋部力矩映射为关节力矩。
 * @param[in] leg 腿部运动学输入或输出状态。
 * @param target_leg_length 目标腿长，单位为米。
 * @param support_feedforward 抵消机体重力的腿部支撑前馈力，单位为牛。
 * @param integral_force 上一周期累计的轴向力积分项。
 * @param leg_angle_torque 绕髋部作用的腿角控制力矩，单位为牛·米。
 * @param filtered_leg_speed 经过滤波的实际腿长变化率，单位为米每秒。
 * @param target_leg_length_rate 目标腿长变化率，单位为米每秒。
 * @return 左右主动关节的目标力矩。
 */
LegVmcOutput ComputeLegVmc(const LegKinematics &leg, double target_leg_length,
			   double support_feedforward, double integral_force,
			   double leg_angle_torque, double filtered_leg_speed,
			   double target_leg_length_rate);

} // namespace modules
