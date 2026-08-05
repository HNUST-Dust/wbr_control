// Legacy controller stack; excluded from the firmware build.
#ifndef WBR_CONTROL_CORE_ROBOT_OBSERVER_H_
#define WBR_CONTROL_CORE_ROBOT_OBSERVER_H_

#include <cstdint>

#include "controller_io.h"
#include "ground_balance_controller.h"

namespace wbr::control {

struct RobotParameters {
  double total_mass = 10.1;
  double gravity = 9.80665;
  double wheel_radius = 0.058;
  double attitude_correction_time_constant = 0.30;
  double velocity_correction_time_constant = 0.08;
  double acceleration_norm_tolerance = 3.0;
  double contact_threshold = 0.5;
  double minimum_control_period = 0.0002;
  double maximum_control_period = 0.003;
  std::uint64_t maximum_sample_age_us = 5000;
};

struct ObservationResult {
  v2::GroundBalanceInput input;
  bool valid = false;
};

class RobotObserver {
 public:
  void Reset();
  ObservationResult Update(const ControlInput& sample,
                           const RobotParameters& parameters,
                           std::uint64_t now_us);

 private:
  bool initialized_ = false;
  double roll_ = 0.0;
  double pitch_ = 0.0;
  double x_ = 0.0;
  double x_speed_ = 0.0;
};

}  // namespace wbr::control

#endif  // WBR_CONTROL_CORE_ROBOT_OBSERVER_H_
