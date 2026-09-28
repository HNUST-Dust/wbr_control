/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "chassis_types.h"

namespace modules
{

/** Estimates per-wheel vertical ground reaction force from joint torque and IMU feedback. */
class SupportForceEstimator
{
public:
	void Reset();
	ChassisSupportForceEstimate Update(const ChassisCycleInput &input);

private:
	struct SideState {
		double previous_length_rate = 0.0;
		double previous_theta_rate = 0.0;
		double length_accel = 0.0;
		double theta_accel = 0.0;
		double filtered_force = 0.0;
	};

	/** Hook for the identified joint spring model; currently intentionally zero. */
	static JointPair<double> SpringTorque(Side side, const ChassisCycleInput &input);

	SidePair<SideState> side_state_;
	bool initialized_ = false;
};

} // namespace modules
