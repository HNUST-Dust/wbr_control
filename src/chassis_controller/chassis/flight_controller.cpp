/* SPDX-License-Identifier: Apache-2.0 */

#include "flight_controller.h"

#include <algorithm>
#include <cmath>

#include "chassis_config.h"
#include "leg_vmc.h"
#include "unified_lqr_schedule.h"

namespace modules
{
namespace
{
using namespace chassis_config;
constexpr size_t kStateCount = 10U;
constexpr size_t kOutputCount = 4U;
constexpr size_t kLeftTheta = 4U;
constexpr size_t kLeftThetaRate = 5U;
constexpr size_t kRightTheta = 6U;
constexpr size_t kRightThetaRate = 7U;
constexpr size_t kLeftWheel = 0U;
constexpr size_t kRightWheel = 1U;
constexpr size_t kLeftLegTorque = 2U;
constexpr size_t kRightLegTorque = 3U;
} // namespace

void FlightController::Reset()
{
	phase_ = FlightPhase::kInit;
	liftoff_elapsed_s_ = 0.0;
	flight_elapsed_s_ = 0.0;
	initial_leg_length_ = {};
	theta_reference_ = {};
	peak_extension_speed_ = {};
	touchdown_elapsed_s_ = {};
	touched_down_ = {};
}

bool FlightController::UpdateLiftoffDetection(
	const ChassisCycleInput &input, const ChassisSupportForceEstimate &support_force,
	bool detection_enabled)
{
	if (!detection_enabled || !support_force.valid) {
		liftoff_elapsed_s_ = 0.0;
		return false;
	}
	const double threshold = kFlightLiftoffForceRatio * 0.5 * kRobotMass * kGravity;
	const bool away_from_mechanical_stop =
		input.leg.left.length >= kFlightMinimumDetectionLegLength &&
		input.leg.right.length >= kFlightMinimumDetectionLegLength;
	const bool both_unloaded = support_force.filtered_force_n.left < threshold &&
		support_force.filtered_force_n.right < threshold;
	liftoff_elapsed_s_ = away_from_mechanical_stop && both_unloaded
		? liftoff_elapsed_s_ + std::clamp(input.dt, 0.0005, 0.01) : 0.0;
	return liftoff_elapsed_s_ >= static_cast<double>(kFlightLiftoffHoldMs) * 0.001;
}

FlightControllerOutput FlightController::Update(const ChassisCycleInput &input)
{
	const double dt = std::clamp(input.dt, 0.0005, 0.01);
	if (phase_ == FlightPhase::kInit) {
		initial_leg_length_ = {input.leg.left.length, input.leg.right.length};
		theta_reference_ = {input.theta.left, input.theta.right};
		peak_extension_speed_ = {};
		touchdown_elapsed_s_ = {};
		touched_down_ = {};
		flight_elapsed_s_ = 0.0;
		phase_ = FlightPhase::kAir;
	}
	flight_elapsed_s_ += dt;

	FlightControllerOutput output = {};
	output.phase = phase_;
	if (phase_ != FlightPhase::kAir) {
		output.finished = phase_ == FlightPhase::kReturn;
		output.failed = phase_ == FlightPhase::kFailed;
		output.touched_down = touched_down_;
		return output;
	}

	for (Side side : {Side::kLeft, Side::kRight}) {
		const LegKinematics &leg = input.leg.Get(side);
		double &peak = peak_extension_speed_.Get(side);
		peak = std::max(peak, leg.length_rate);
		const bool velocity_reversed = leg.length_rate < kFlightTouchdownRetractSpeed;
		const bool peak_dropped = leg.length <
				(kFlightMaximumLegLength - kFlightTouchdownLengthMargin) &&
			peak > kFlightTouchdownPeakMin &&
			peak - leg.length_rate > kFlightTouchdownPeakDrop;
		double &elapsed = touchdown_elapsed_s_.Get(side);
		elapsed = !touched_down_.Get(side) && (velocity_reversed || peak_dropped)
			? elapsed + dt : 0.0;
		if (elapsed >= static_cast<double>(kFlightTouchdownHoldMs) * 0.001) {
			touched_down_.Get(side) = true;
		}
	}

	double state_error[kStateCount] = {};
	state_error[kLeftTheta] = input.theta.left - theta_reference_.left;
	state_error[kLeftThetaRate] = input.theta_rate.left;
	state_error[kRightTheta] = input.theta.right - theta_reference_.right;
	state_error[kRightThetaRate] = input.theta_rate.right;
	double gain[kOutputCount][kStateCount] = {};
	EvaluateUnifiedLqrGain(0.5 * (input.leg.left.length + input.leg.right.length), gain);
	double actuator[kOutputCount] = {};
	for (size_t row = 0U; row < kOutputCount; ++row) {
		for (size_t column = kLeftTheta; column <= kRightThetaRate; ++column) {
			actuator[row] -= gain[row][column] * state_error[column];
		}
	}
	output.control.physical_wheel_torque.left = touched_down_.left ? actuator[kLeftWheel] : 0.0;
	output.control.physical_wheel_torque.right = touched_down_.right ? actuator[kRightWheel] : 0.0;

	const auto update_leg = [&](Side side, size_t leg_torque_index) {
		const LegKinematics &leg = input.leg.Get(side);
		const bool touched = touched_down_.Get(side);
		const double target_length = touched ? initial_leg_length_.Get(side) : leg.length;
		const bool near_maximum =
			leg.length >= kFlightMaximumLegLength - kFlightMaximumLengthMargin;
		const double axial_feedforward = touched
			? 0.25 * kRobotMass * kGravity /
				std::max(std::cos(input.theta.Get(side)), 0.5)
			: (near_maximum ? 0.0 : kFlightAirbornePushForce);
		const LegHardwareMap &hardware = kLegHardware.Get(side);
		const LegVmcOutput vmc = ComputeLegVmc(
			leg, target_length, axial_feedforward, 0.0,
			hardware.vmc_torque_sign * actuator[leg_torque_index],
			leg.length_rate, 0.0);
		output.control.leg_axial_force.Get(side) = vmc.axial_force;
		output.control.body_on_leg_torque.Get(side) = actuator[leg_torque_index];
		JointPair<double> &torque = output.control.joint_torque.Get(side);
		torque.b = std::clamp(vmc.phi2_torque, -kJointTorqueLimit, kJointTorqueLimit);
		torque.d = std::clamp(vmc.phi1_torque, -kJointTorqueLimit, kJointTorqueLimit);
	};
	update_leg(Side::kLeft, kLeftLegTorque);
	update_leg(Side::kRight, kRightLegTorque);

	if (touched_down_.left && touched_down_.right) {
		phase_ = FlightPhase::kReturn;
		output.finished = true;
	} else if (flight_elapsed_s_ >= static_cast<double>(kFlightTimeoutMs) * 0.001) {
		phase_ = FlightPhase::kFailed;
		output.failed = true;
	}
	output.phase = phase_;
	output.touched_down = touched_down_;
	return output;
}

} // namespace modules
