/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <modules/chassis/leg_vmc.h>

#include <algorithm>

namespace {

constexpr double kLegLengthKp = 600.0;
constexpr double kLegLengthKi = 100.0;
constexpr double kLegLengthKd = 120.0;
constexpr double kRetractFeedforward = 10.0;
constexpr double kRetractFeedforwardDeadband = 0.003;
constexpr double kExtendVelocityFeedforward = 60.0;
constexpr double kVelocityFeedforwardDeadband = 0.005;
constexpr double kIntegralForceLimit = 60.0;
constexpr double kForceLimit = 150.0;

}  // namespace

namespace modules {

double UpdateLegLengthIntegral(double integral_force, double length_error,
				      double dt)
{
	return std::clamp(integral_force + kLegLengthKi * length_error * dt,
		-kIntegralForceLimit, kIntegralForceLimit);
}

LegVmcOutput ComputeLegVmc(const LegKinematics &leg,
			   double target_leg_length,
			   double support_feedforward,
			   double integral_force,
			   double leg_angle_torque,
			   double filtered_leg_speed,
			   double target_leg_length_rate)
{
	const double length_error = target_leg_length - leg.length;
	const double retract_feedforward =
		length_error < -kRetractFeedforwardDeadband ? -kRetractFeedforward : 0.0;
	const double velocity_feedforward =
		target_leg_length_rate > kVelocityFeedforwardDeadband ?
			kExtendVelocityFeedforward * target_leg_length_rate : 0.0;

	LegVmcOutput output = {};
	output.axial_force = std::clamp(
		kLegLengthKp * length_error - kLegLengthKd * filtered_leg_speed +
			support_feedforward + integral_force + retract_feedforward +
			velocity_feedforward,
		-kForceLimit, kForceLimit);
	const double radial_x = leg.hx / leg.length;
	const double radial_z = leg.hz / leg.length;
	const double tangential_force = leg_angle_torque / leg.length;
	const double force_x =
		output.axial_force * radial_x - tangential_force * radial_z;
	const double force_z =
		output.axial_force * radial_z + tangential_force * radial_x;
	output.joint_torque[0] =
		leg.jacobian[0][0] * force_x + leg.jacobian[1][0] * force_z;
	output.joint_torque[1] =
		leg.jacobian[0][1] * force_x + leg.jacobian[1][1] * force_z;
	return output;
}

}  // namespace modules
