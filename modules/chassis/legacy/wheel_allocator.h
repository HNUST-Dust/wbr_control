// Legacy controller stack; excluded from the firmware build.
#ifndef WBR_CONTROL_CORE_WHEEL_ALLOCATOR_H_
#define WBR_CONTROL_CORE_WHEEL_ALLOCATOR_H_

#include "controller_types.h"

namespace wbr::v2 {

struct WheelAllocationInput {
  double balance_torque = 0.0;
  double yaw_torque = 0.0;
  bool grounded[2] = {};
  WbrContactSafetyState contact_state = WbrContactSafetyState::kAirborne;
};

struct WheelAllocationOutput {
  double actuator_torque[2] = {};
  double applied_yaw_torque = 0.0;
};

WheelAllocationOutput AllocateWheelTorque(const WheelAllocationInput& input);

}  // namespace wbr::v2

#endif  // WBR_CONTROL_CORE_WHEEL_ALLOCATOR_H_
