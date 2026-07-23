/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <array>

namespace modules {

struct BodyMotionState {
	double speed = 0.0;
	double acceleration = 0.0;
	double position = 0.0;
};

/* SPR-style two-state Kalman estimator.
 *
 * The Kalman state is [forward speed, forward acceleration]. Position is a
 * separate continuous integral of estimated speed; consumers are responsible
 * for limiting the position error used by their controller. */
class BodyMotionEstimator {
public:
	void Reset();
	const BodyMotionState &Update(double speed_measurement,
				      double acceleration_measurement, double dt);

private:
	BodyMotionState state_;
	std::array<double, 4> covariance_ = {1.0, 0.0, 0.0, 1.0};
};

}  // namespace modules
