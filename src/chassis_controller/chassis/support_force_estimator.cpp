/* SPDX-License-Identifier: Apache-2.0 */

#include "support_force_estimator.h"

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <algorithm>
#include <cmath>

#include "chassis_config.h"

namespace modules
{
namespace
{
using namespace chassis_config;

double LowPassAlpha(double cutoff_hz, double dt)
{
	constexpr double kPi = 3.14159265358979323846;
	return 1.0 - std::exp(-2.0 * kPi * cutoff_hz * dt);
}
} // namespace

void SupportForceEstimator::Reset()
{
	side_state_ = {};
	initialized_ = false;
}

JointPair<double> SupportForceEstimator::SpringTorque(
	Side side, const ChassisCycleInput &input)
{
	ARG_UNUSED(side);
	ARG_UNUSED(input);
	// Replace this zero model after spring torque versus joint position has been identified.
	return {};
}

ChassisSupportForceEstimate SupportForceEstimator::Update(const ChassisCycleInput &input)
{
	using namespace chassis_config;
	ChassisSupportForceEstimate output = {};
	if (!input.feedback_valid || !input.imu_fresh) {
		Reset();
		return output;
	}

	const double dt = std::clamp(input.dt, 0.0005, 0.01);
	const double cr = std::cos(input.roll);
	const double sr = std::sin(input.roll);
	const double cp = std::cos(input.pitch);
	const double sp = std::sin(input.pitch);
	const double world_specific_force_g = kSupportWorldVerticalSpecificForceSign *
		(-sp * static_cast<double>(input.imu.accel_g[0]) +
		 sr * cp * static_cast<double>(input.imu.accel_g[1]) +
		 cr * cp * static_cast<double>(input.imu.accel_g[2]));
	output.world_vertical_specific_force_g = world_specific_force_g;
	output.body_vertical_accel_mps2 = (world_specific_force_g - 1.0) * kGravity;
	if (!std::isfinite(output.body_vertical_accel_mps2)) {
		Reset();
		return output;
	}

	const double derivative_alpha = LowPassAlpha(kSupportDerivativeFilterCutoffHz, dt);
	const double force_alpha = LowPassAlpha(kSupportForceFilterCutoffHz, dt);
	bool valid = true;
	for (Side side : {Side::kLeft, Side::kRight}) {
		const LegKinematics &leg = input.leg.Get(side);
		SideState &state = side_state_.Get(side);
		if (!initialized_) {
			state.previous_length_rate = leg.length_rate;
			state.previous_theta_rate = input.theta_rate.Get(side);
		}
		const double raw_length_accel =
			(leg.length_rate - state.previous_length_rate) / dt;
		const double raw_theta_accel =
			(input.theta_rate.Get(side) - state.previous_theta_rate) / dt;
		state.previous_length_rate = leg.length_rate;
		state.previous_theta_rate = input.theta_rate.Get(side);
		state.length_accel += derivative_alpha * (raw_length_accel - state.length_accel);
		state.theta_accel += derivative_alpha * (raw_theta_accel - state.theta_accel);

		Eigen::Matrix2d jacobian;
		jacobian << leg.jacobian[0][0], leg.jacobian[0][1],
			leg.jacobian[1][0], leg.jacobian[1][1];
		const JointPair<double> spring = SpringTorque(side, input);
		const JointPair<double> &feedback = input.joint_torque_feedback.Get(side);
		const double torque_sign = side == Side::kLeft
			? kSupportLeftJointTorqueSign : kSupportRightJointTorqueSign;
		// Kinematics columns are [joint D (phi1), joint B (phi2)].
		const Eigen::Vector2d joint_torque(
			torque_sign * feedback.d - spring.d,
			torque_sign * feedback.b - spring.b);
		const Eigen::Matrix2d normal = jacobian * jacobian.transpose() +
			kSupportJacobianDamping * Eigen::Matrix2d::Identity();
		const Eigen::Vector2d endpoint_force = normal.ldlt().solve(jacobian * joint_torque);
		const double leg_vertical_force = -endpoint_force.y();

		const double theta = input.theta.Get(side);
		const double theta_rate = input.theta_rate.Get(side);
		const double wheel_vertical_accel = output.body_vertical_accel_mps2 -
			state.length_accel * std::cos(theta) +
			2.0 * leg.length_rate * theta_rate * std::sin(theta) +
			leg.length * state.theta_accel * std::sin(theta) +
			leg.length * theta_rate * theta_rate * std::cos(theta);
		const double raw_force = leg_vertical_force +
			kSupportWheelMass * (kGravity + wheel_vertical_accel);
		if (!endpoint_force.allFinite() || !std::isfinite(raw_force)) {
			valid = false;
			continue;
		}
		const double bounded_force = std::clamp(
			raw_force, -kSupportMaximumAbsForce, kSupportMaximumAbsForce);
		if (!initialized_) {
			state.filtered_force = bounded_force;
		} else {
			state.filtered_force += force_alpha * (bounded_force - state.filtered_force);
		}
		output.raw_force_n.Get(side) = raw_force;
		output.filtered_force_n.Get(side) = state.filtered_force;
		output.leg_vertical_force_n.Get(side) = leg_vertical_force;
		output.wheel_vertical_accel_mps2.Get(side) = wheel_vertical_accel;
	}
	initialized_ = valid;
	output.valid = valid;
	return output;
}

} // namespace modules
