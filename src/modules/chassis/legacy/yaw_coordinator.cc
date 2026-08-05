// Legacy controller stack; excluded from the firmware build.
#include "yaw_coordinator.h"

#include <cmath>

#include "control_parameters.h"
#include "math_utils.h"

namespace wbr::v2 {

void YawCoordinator::Reset() {
  initialized_ = false;
  coordinated_rate_ = split_reference_ = 0.0;
  filtered_force_[0] = filtered_force_[1] = 0.0;
  filtered_force_rate_[0] = filtered_force_rate_[1] = 0.0;
}

YawCoordinatorOutput YawCoordinator::Update(
    const YawCoordinatorInput& input) {
  YawCoordinatorOutput output;
  const double reference_yaw_rate = input.measured_yaw_rate +
      0.10 * (input.commanded_yaw_rate - input.measured_yaw_rate);
  const double desired_reference = input.reference_enabled
      ? Clamp(kYawSplitReferencePerRate * reference_yaw_rate,
              -kMaxYawSplitReference, kMaxYawSplitReference)
      : 0.0;
  const double previous_reference = split_reference_;
  split_reference_ = MoveTowards(
      split_reference_, desired_reference,
      kYawSplitReferenceSlewRate * input.control_dt);
  const double reference_rate = input.control_dt > 0.0
      ? (split_reference_ - previous_reference) / input.control_dt
      : 0.0;
  output.split_residual = std::remainder(
      input.split_angle - split_reference_, kTwoPi);
  output.split_rate_residual = input.split_rate - reference_rate;

  const double force_alpha = 1.0 - std::exp(
      -input.control_dt / kYawForceFilterTimeConstant);
  const double force_rate_alpha = 1.0 - std::exp(
      -input.control_dt / kYawForceRateFilterTimeConstant);
  if (!initialized_ || input.time_reset) {
    for (int wheel = 0; wheel < 2; ++wheel) {
      filtered_force_[wheel] = input.normal_force[wheel];
      filtered_force_rate_[wheel] = 0.0;
    }
    coordinated_rate_ = 0.0;
    initialized_ = true;
  } else {
    for (int wheel = 0; wheel < 2; ++wheel) {
      const double previous_force = filtered_force_[wheel];
      filtered_force_[wheel] += force_alpha *
          (input.normal_force[wheel] - filtered_force_[wheel]);
      const double raw_rate =
          (filtered_force_[wheel] - previous_force) / input.control_dt;
      filtered_force_rate_[wheel] += force_rate_alpha *
          (raw_rate - filtered_force_rate_[wheel]);
    }
  }

  output.predicted_split_error = output.split_residual +
      kYawPredictionHorizon * output.split_rate_residual;
  // This prediction already includes residual split rate. Absolute angle/rate
  // limits below remain the independent hard-instability guard.
  output.split_residual_authority = FadeAuthority(
      std::fabs(output.predicted_split_error),
      kYawPredictedSplitSoft, kYawPredictedSplitHard);
  output.split_absolute_authority = std::fmin(
      FadeAuthority(std::fabs(input.split_angle),
                    kAbsoluteLegSplitSoftAngle, kAbsoluteLegSplitHardAngle),
      FadeAuthority(std::fabs(input.split_rate),
                    kAbsoluteLegSplitSoftRate, kAbsoluteLegSplitHardRate));
  output.split_authority = std::fmin(
      output.split_residual_authority, output.split_absolute_authority);

  output.predicted_roll =
      input.roll + kYawPredictionHorizon * input.roll_rate;
  output.attitude_authority = std::fmin(
      FadeAuthority(std::fabs(output.predicted_roll),
                    kYawPredictedRollSoft, kYawPredictedRollHard),
      FadeAuthority(std::fabs(input.pitch),
                    kYawPitchSoftLimit, kYawPitchHardLimit));
  for (int wheel = 0; wheel < 2; ++wheel) {
    output.predicted_normal_force[wheel] = std::fmax(
        0.0, filtered_force_[wheel] + kYawPredictionHorizon *
            std::fmin(filtered_force_rate_[wheel], 0.0));
  }
  const double min_force = std::fmin(
      output.predicted_normal_force[0], output.predicted_normal_force[1]);
  const double max_force = std::fmax(
      output.predicted_normal_force[0], output.predicted_normal_force[1]);
  const double load_ratio = max_force > 1e-6 ? min_force / max_force : 0.0;
  output.contact_authority = std::fmin(
      RiseAuthority(min_force, kYawContactForceHard, kYawContactForceSoft),
      RiseAuthority(load_ratio, kYawLoadRatioHard, kYawLoadRatioSoft));
  output.authority = std::fmin(
      output.split_authority,
      std::fmin(output.attitude_authority, output.contact_authority));

  const double target = output.authority * input.commanded_yaw_rate;
  const bool braking = std::fabs(target) < std::fabs(coordinated_rate_) ||
      target * coordinated_rate_ < 0.0;
  coordinated_rate_ = MoveTowards(
      coordinated_rate_, target,
      (braking ? kYawCoordinatorDeceleration : kYawCommandAcceleration) *
          input.control_dt);
  output.coordinated_yaw_rate = coordinated_rate_;
  return output;
}

}  // namespace wbr::v2
