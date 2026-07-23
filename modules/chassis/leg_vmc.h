/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <modules/chassis/leg_kinematics.h>

namespace modules {

struct LegVmcOutput {
	double axial_force = 0.0;
	double joint_torque[2] = {};  // phi1, phi2
};

double UpdateLegLengthIntegral(double integral_force, double length_error,
				      double dt);
LegVmcOutput ComputeLegVmc(const LegKinematics &leg,
			   double target_leg_length,
			   double support_feedforward,
			   double integral_force,
			   double leg_angle_torque,
			   double filtered_leg_speed,
			   double target_leg_length_rate);

}  // namespace modules
