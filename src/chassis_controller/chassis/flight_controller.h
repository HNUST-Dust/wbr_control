/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "chassis_types.h"

namespace modules
{

enum class FlightPhase : uint8_t { kInit = 0U, kAir, kReturn, kFailed };

struct FlightControllerOutput {
	ChassisControlOutput control;
	FlightPhase phase = FlightPhase::kInit;
	SidePair<bool> touched_down;
	bool finished = false;
	bool failed = false;
};

class FlightController
{
public:
	void Reset();
	bool UpdateLiftoffDetection(const ChassisCycleInput &input,
				    const ChassisSupportForceEstimate &support_force,
				    bool detection_enabled);
	FlightControllerOutput Update(const ChassisCycleInput &input);
	FlightPhase phase() const { return phase_; }
	const SidePair<bool> &touched_down() const { return touched_down_; }

private:
	FlightPhase phase_ = FlightPhase::kInit;
	double liftoff_elapsed_s_ = 0.0;
	double flight_elapsed_s_ = 0.0;
	SidePair<double> initial_leg_length_;
	SidePair<double> theta_reference_;
	SidePair<double> peak_extension_speed_;
	SidePair<double> touchdown_elapsed_s_;
	SidePair<bool> touched_down_;
};

} // namespace modules
