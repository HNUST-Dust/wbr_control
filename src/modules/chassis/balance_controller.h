/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "body_motion_estimator.h"
#include "chassis_types.h"

namespace modules
{

/** Stateful balance algorithm, independent of msg and motor protocols. */
class BalanceController
{
public:
	BalanceController();
	void Reset();
	void ResetForEnable();
	void SynchronizeReferences(const ChassisCycleInput &input);
	void Update(const ChassisCycleInput &input, ChassisControlOutput &output);

	void set_target_leg_length(double value) { target_leg_length_ = value; }
	double target_leg_length() const { return target_leg_length_; }
	double target_yaw_rate() const { return target_yaw_rate_; }
	double target_yaw_angle() const { return target_yaw_angle_; }
	double filtered_yaw_rate() const { return filtered_yaw_rate_; }

private:
	struct SideState {
		double leg_length_integral_force = 0.0;
	};

	BodyMotionEstimator body_motion_estimator_;
	double target_leg_length_ = 0.0;
	double balance_theta_reference_ = 0.0;
	double balance_pitch_reference_ = 0.0;
	double balance_roll_reference_ = 0.0;
	double target_body_speed_ = 0.0;
	double target_body_position_ = 0.0;
	double target_yaw_rate_ = 0.0;
	double target_yaw_angle_ = 0.0;
	bool yaw_reference_initialized_ = false;
	double filtered_yaw_rate_ = 0.0;
	bool yaw_rate_filter_initialized_ = false;
	SidePair<SideState> side_state_;
};

} // namespace modules
