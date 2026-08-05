/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <array>

namespace modules
{

struct BodyMotionState {
	double speed = 0.0;
	double acceleration = 0.0;
	double position = 0.0;
};

/*
 * SPR 风格的二状态卡尔曼估计器。
 *
 * 卡尔曼状态为 [前向速度, 前向加速度]。位置由估计速度独立连续积分，
 * 使用方负责限制送入控制器的位置误差。
 */
class BodyMotionEstimator
{
public:
	void Reset();
	const BodyMotionState &Update(double speed_measurement, double acceleration_measurement,
				      double dt);

private:
	BodyMotionState state_;
	std::array<double, 4> covariance_ = {1.0, 0.0, 0.0, 1.0};
};

} // namespace modules
