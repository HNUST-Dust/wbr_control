// Legacy controller stack; excluded from the firmware build.
#ifndef WBR_CONTROL_CORE_CONTACT_SAFETY_H_
#define WBR_CONTROL_CORE_CONTACT_SAFETY_H_

#include "controller_types.h"

namespace wbr::v2 {

struct ContactSafetyOutput {
  WbrContactSafetyState state = WbrContactSafetyState::kAirborne;
  bool effective_grounded[2] = {};
  bool single_support = false;
  bool airborne = true;
  bool state_changed = false;
  double authority = 0.0;
  double state_elapsed = 0.0;
};

class ContactSafetyMachine {
 public:
  void Reset();
  ContactSafetyOutput Update(const bool grounded[2], double roll,
                             double pitch, double control_dt);

 private:
  double contact_grace_[2] = {};
  WbrContactSafetyState state_ = WbrContactSafetyState::kAirborne;
  WbrContactSafetyState candidate_ = WbrContactSafetyState::kAirborne;
  double state_elapsed_ = 0.0;
  double candidate_elapsed_ = 0.0;
};

}  // namespace wbr::v2

#endif  // WBR_CONTROL_CORE_CONTACT_SAFETY_H_
