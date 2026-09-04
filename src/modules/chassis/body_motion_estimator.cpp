/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file src/modules/chassis/body_motion_estimator.cpp
 * @ingroup wbr_modules
 * @brief 根据底盘传感器数据估计机体运动状态。
 * @details 实现运行在模块自有 Zephyr 线程或其驱动回调中。回调路径只完成有界的数据搬运和通知，耗时解析与控制计算留在线程上下文执行。
 */

#include "body_motion_estimator.h"

#include <Eigen/Core>

namespace
{

using StateVector = Eigen::Matrix<double, 2, 1>;
using StateMatrix = Eigen::Matrix<double, 2, 2>;
using ObservationVector = Eigen::Matrix<double, 1, 2>;

constexpr double kProcessNoise = 0.1;
constexpr double kSpeedMeasurementNoise = 100.0;
// SPR 几乎不信任加速度测量，因此估计主要由轮腿运动学速度决定。
constexpr double kAccelerationMeasurementNoise = 1.0e12;

} // namespace

namespace modules
{

void BodyMotionEstimator::Reset()
{
	state_ = {};
	covariance_ = {1.0, 0.0, 0.0, 1.0};
}

const BodyMotionState &BodyMotionEstimator::Update(double speed_measurement,
						   double acceleration_measurement, double dt)
{
	Eigen::Map<StateMatrix> covariance(covariance_.data());

	// x = [前向速度, 前向加速度]。
	StateVector state;
	state << state_.speed, state_.acceleration;

	StateMatrix state_transition;
	state_transition << 1.0, dt,
			    0.0, 1.0;
	const StateMatrix process_noise = StateMatrix::Identity() * kProcessNoise;

	// 预测：x^- = F x，P^- = F P F^T + Q。
	state = state_transition * state;
	const StateMatrix predicted_covariance =
		state_transition * covariance * state_transition.transpose() + process_noise;
	covariance = predicted_covariance;

	// 依次进行两个一维观测校正：K = P H^T / (H P H^T + R)。
	// 顺序更新保留旧实现的计算结构，同时不需要构造或求逆通用观测矩阵。
	const auto correct = [&](const ObservationVector &observation, double measurement,
				 double measurement_noise) {
		const double innovation = measurement - (observation * state)(0);
		const double innovation_variance =
			(observation * covariance * observation.transpose())(0, 0) +
			measurement_noise;
		const StateVector kalman_gain =
			covariance * observation.transpose() / innovation_variance;
		state += kalman_gain * innovation;
		const StateMatrix corrected_covariance =
			(StateMatrix::Identity() - kalman_gain * observation) * covariance;
		covariance = corrected_covariance;
	};

	ObservationVector speed_observation;
	speed_observation << 1.0, 0.0;
	correct(speed_observation, speed_measurement, kSpeedMeasurementNoise);

	ObservationVector acceleration_observation;
	acceleration_observation << 0.0, 1.0;
	correct(acceleration_observation, acceleration_measurement,
		kAccelerationMeasurementNoise);

	state_.speed = state(0);
	state_.acceleration = state(1);

	// 连续保留位移历史；控制器限制位置误差，因此高速运动不会清空里程，
	// 估计偏置也不会产生无界 LQR 力矩请求。
	state_.position += state_.speed * dt;
	return state_;
}

} // namespace modules
