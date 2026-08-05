// Legacy controller stack; excluded from the firmware build.
#include "wheel_allocator.h"

#include <cmath>

#include "control_parameters.h"
#include "math_utils.h"

namespace wbr::v2 {

WheelAllocationOutput AllocateWheelTorque(const WheelAllocationInput& input) {
  WheelAllocationOutput output;
  const double common_torque = 0.5 * input.balance_torque;
  const double yaw_headroom =
      std::fmax(0.0, kPerWheelTorqueLimit - std::fabs(common_torque));
  const double differential_torque = Clamp(
      -0.5 * kDecoupledYawWheelInputScale * input.yaw_torque,
      -yaw_headroom, yaw_headroom);
  output.applied_yaw_torque = -2.0 * differential_torque;

  if (!input.grounded[0] && !input.grounded[1]) {
    output.applied_yaw_torque = 0.0;
    return output;
  }
  if (input.grounded[0] && !input.grounded[1]) {
    output.actuator_torque[0] = Clamp(
        -(input.contact_state == WbrContactSafetyState::kSingleSupportFirst
              ? input.balance_torque
              : common_torque),
        -kPerWheelTorqueLimit, kPerWheelTorqueLimit);
    output.applied_yaw_torque = 0.0;
    return output;
  }
  if (!input.grounded[0] && input.grounded[1]) {
    output.actuator_torque[1] = Clamp(
        input.contact_state == WbrContactSafetyState::kSingleSupportSecond
            ? input.balance_torque
            : common_torque,
        -kPerWheelTorqueLimit, kPerWheelTorqueLimit);
    output.applied_yaw_torque = 0.0;
    return output;
  }
  if (input.contact_state == WbrContactSafetyState::kSingleSupportFirst) {
    output.actuator_torque[0] = Clamp(
        -input.balance_torque, -kPerWheelTorqueLimit, kPerWheelTorqueLimit);
    output.applied_yaw_torque = 0.0;
    return output;
  }
  if (input.contact_state == WbrContactSafetyState::kSingleSupportSecond) {
    output.actuator_torque[1] = Clamp(
        input.balance_torque, -kPerWheelTorqueLimit, kPerWheelTorqueLimit);
    output.applied_yaw_torque = 0.0;
    return output;
  }
  if (input.contact_state == WbrContactSafetyState::kAirborne) {
    output.applied_yaw_torque = 0.0;
    return output;
  }
  // Direction probing established the chassis-forward raw-axis mapping as
  // left negative and right positive.  Keep the yaw component equal on both
  // mirrored motor axes, but map common balance torque with opposite signs.
  output.actuator_torque[0] = Clamp(
      -common_torque + differential_torque,
      -kPerWheelTorqueLimit, kPerWheelTorqueLimit);
  output.actuator_torque[1] = Clamp(
      common_torque + differential_torque,
      -kPerWheelTorqueLimit, kPerWheelTorqueLimit);
  return output;
}

}  // namespace wbr::v2
