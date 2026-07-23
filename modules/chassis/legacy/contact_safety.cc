// Legacy controller stack; excluded from the firmware build.
#include "contact_safety.h"

#include <cmath>

#include "control_parameters.h"
#include "math_utils.h"

namespace wbr::v2 {

void ContactSafetyMachine::Reset() {
  contact_grace_[0] = contact_grace_[1] = 0.0;
  state_ = WbrContactSafetyState::kAirborne;
  candidate_ = WbrContactSafetyState::kAirborne;
  state_elapsed_ = candidate_elapsed_ = 0.0;
}

ContactSafetyOutput ContactSafetyMachine::Update(
    const bool grounded[2], double roll, double pitch, double control_dt) {
  ContactSafetyOutput output;
  for (int wheel = 0; wheel < 2; ++wheel) {
    contact_grace_[wheel] = grounded[wheel]
        ? kContactGraceTime
        : std::fmax(0.0, contact_grace_[wheel] - control_dt);
    output.effective_grounded[wheel] = contact_grace_[wheel] > 0.0;
  }

  const WbrContactSafetyState previous_state = state_;
  WbrContactSafetyState observed = WbrContactSafetyState::kAirborne;
  if (grounded[0] && grounded[1]) {
    observed = WbrContactSafetyState::kDualSupport;
  } else if (grounded[0]) {
    observed = WbrContactSafetyState::kSingleSupportFirst;
  } else if (grounded[1]) {
    observed = WbrContactSafetyState::kSingleSupportSecond;
  }

  state_elapsed_ += control_dt;
  WbrContactSafetyState requested = observed;
  if (observed == WbrContactSafetyState::kDualSupport &&
      state_ != WbrContactSafetyState::kDualSupport) {
    requested = WbrContactSafetyState::kRecovery;
  }
  if (requested != candidate_) {
    candidate_ = requested;
    candidate_elapsed_ = 0.0;
  } else {
    candidate_elapsed_ += control_dt;
  }
  const double debounce = requested == WbrContactSafetyState::kRecovery
      ? kContactRecoveryDebounceTime
      : kContactLossDebounceTime;
  if (requested != state_ && candidate_elapsed_ >= debounce) {
    state_ = requested;
    state_elapsed_ = 0.0;
  }
  if (state_ == WbrContactSafetyState::kRecovery &&
      observed == WbrContactSafetyState::kDualSupport &&
      state_elapsed_ >= kContactRecoveryRampTime &&
      std::fabs(roll) <= kRecoveryMaxRoll &&
      std::fabs(pitch) <= kRecoveryMaxPitch) {
    state_ = WbrContactSafetyState::kDualSupport;
    candidate_ = WbrContactSafetyState::kDualSupport;
    state_elapsed_ = candidate_elapsed_ = 0.0;
  }

  output.state = state_;
  output.state_changed = state_ != previous_state;
  output.single_support =
      state_ == WbrContactSafetyState::kSingleSupportFirst ||
      state_ == WbrContactSafetyState::kSingleSupportSecond;
  output.airborne = state_ == WbrContactSafetyState::kAirborne;
  output.authority = 1.0;
  if (output.single_support) {
    output.authority = kSingleSupportLqrScale;
  } else if (output.airborne) {
    output.authority = 0.0;
  } else if (state_ == WbrContactSafetyState::kRecovery) {
    output.authority = 0.5 + 0.5 * Clamp(
        state_elapsed_ / kContactRecoveryRampTime, 0.0, 1.0);
  }
  output.state_elapsed = state_elapsed_;
  return output;
}

}  // namespace wbr::v2
