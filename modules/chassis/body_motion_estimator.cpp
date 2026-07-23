/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <modules/chassis/body_motion_estimator.h>

#include <cmath>

namespace {

constexpr double kProcessNoise = 0.1;
constexpr double kSpeedMeasurementNoise = 100.0;
/* SPR intentionally gives acceleration almost no measurement authority. The
 * estimator is therefore dominated by wheel/leg kinematic speed. */
constexpr double kAccelerationMeasurementNoise = 1.0e12;

}  // namespace

namespace modules {

void BodyMotionEstimator::Reset()
{
	state_ = {};
	covariance_ = {1.0, 0.0, 0.0, 1.0};
}

const BodyMotionState &BodyMotionEstimator::Update(
	double speed_measurement, double acceleration_measurement, double dt)
{
	// Predict x and P with x = [speed, acceleration].
	const double predicted_speed = state_.speed + state_.acceleration * dt;
	const double predicted_acceleration = state_.acceleration;
	const double p00 = covariance_[0] + dt * (covariance_[1] + covariance_[2]) +
		dt * dt * covariance_[3] + kProcessNoise;
	const double p01 = covariance_[1] + dt * covariance_[3];
	const double p10 = covariance_[2] + dt * covariance_[3];
	const double p11 = covariance_[3] + kProcessNoise;

	// Correct with the kinematic speed measurement.
	const double k00 = p00 / (p00 + kSpeedMeasurementNoise);
	const double k10 = p10 / (p00 + kSpeedMeasurementNoise);
	const double speed_innovation = speed_measurement - predicted_speed;
	state_.speed = predicted_speed + k00 * speed_innovation;
	state_.acceleration = predicted_acceleration + k10 * speed_innovation;
	const double q00 = (1.0 - k00) * p00;
	const double q01 = (1.0 - k00) * p01;
	const double q10 = p10 - k10 * p00;
	const double q11 = p11 - k10 * p01;

	// Correct with acceleration. Its very large noise preserves SPR's tuning.
	const double k01 = q01 / (q11 + kAccelerationMeasurementNoise);
	const double k11 = q11 / (q11 + kAccelerationMeasurementNoise);
	const double acceleration_innovation =
		acceleration_measurement - state_.acceleration;
	state_.speed += k01 * acceleration_innovation;
	state_.acceleration += k11 * acceleration_innovation;
	covariance_[0] = q00 - k01 * q10;
	covariance_[1] = q01 - k01 * q11;
	covariance_[2] = q10 - k11 * q10;
	covariance_[3] = q11 - k11 * q11;

	// Preserve displacement continuously.  The controller limits the position
	// error it consumes, so high speed no longer destroys odometry history and
	// estimator bias still cannot request unbounded LQR torque.
	state_.position += state_.speed * dt;
	return state_;
}

}  // namespace modules
