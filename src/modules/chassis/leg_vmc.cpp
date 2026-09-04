/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file src/modules/chassis/leg_vmc.cpp
 * @ingroup wbr_modules
 * @brief 实现腿部虚拟模型控制的力矩映射。
 * @details 实现运行在模块自有 Zephyr 线程或其驱动回调中。回调路径只完成有界的数据搬运和通知，耗时解析与控制计算留在线程上下文执行。
 */

#include "leg_vmc.h"

#include <Eigen/Core>

#include <algorithm>

namespace
{

constexpr double kLegLengthKp = 600.0;
constexpr double kLegLengthKi = 100.0;
constexpr double kLegLengthKd = 120.0;
constexpr double kRetractFeedforward = 10.0;
constexpr double kRetractFeedforwardDeadband = 0.003;
constexpr double kExtendVelocityFeedforward = 60.0;
constexpr double kVelocityFeedforwardDeadband = 0.005;
constexpr double kIntegralForceLimit = 60.0;
constexpr double kForceLimit = 150.0;

} // namespace

namespace modules
{

double UpdateLegLengthIntegral(double integral_force, double length_error, double dt)
{
	return std::clamp(integral_force + kLegLengthKi * length_error * dt, -kIntegralForceLimit,
			  kIntegralForceLimit);
}

LegVmcOutput ComputeLegVmc(const LegKinematics &leg, double target_leg_length,
			   double support_feedforward, double integral_force,
			   double leg_angle_torque, double filtered_leg_speed,
			   double target_leg_length_rate)
{
	const double length_error = target_leg_length - leg.length;
	const double retract_feedforward =
		length_error < -kRetractFeedforwardDeadband ? -kRetractFeedforward : 0.0;
	const double velocity_feedforward =
		target_leg_length_rate > kVelocityFeedforwardDeadband
			? kExtendVelocityFeedforward * target_leg_length_rate
			: 0.0;

	LegVmcOutput output = {};
	output.axial_force =
		std::clamp(kLegLengthKp * length_error - kLegLengthKd * filtered_leg_speed +
				   support_feedforward + integral_force + retract_feedforward +
				   velocity_feedforward,
			   -kForceLimit, kForceLimit);
	const double radial_x = leg.hx / leg.length;
	const double radial_z = leg.hz / leg.length;
	const double tangential_force = leg_angle_torque / leg.length;
	// 轴向力与切向力合成末端力 F，再经雅可比转置映射到关节力矩：tau = J^T F。
	const Eigen::Vector2d radial(radial_x, radial_z);
	const Eigen::Vector2d tangential(-radial_z, radial_x);
	const Eigen::Vector2d force =
		output.axial_force * radial + tangential_force * tangential;
	Eigen::Matrix2d J;
	J << leg.jacobian[0][0], leg.jacobian[0][1],
	     leg.jacobian[1][0], leg.jacobian[1][1];
	const Eigen::Vector2d tau = J.transpose() * force;
	output.phi1_torque = tau[0];
	output.phi2_torque = tau[1];
	return output;
}

} // namespace modules
