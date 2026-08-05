// Legacy controller stack; excluded from the firmware build.
#ifndef WBR_CONTROL_CORE_CONTROLLER_TYPES_H_
#define WBR_CONTROL_CORE_CONTROLLER_TYPES_H_

enum class WbrContactSafetyState {
  kDualSupport = 0,
  kSingleSupportFirst = 1,
  kSingleSupportSecond = 2,
  kAirborne = 3,
  kRecovery = 4,
};

struct WbrControllerV2Telemetry {
  double leg_length[2] = {};
  double leg_length_rate[2] = {};
  double leg_angle[2] = {};
  double leg_angle_rate[2] = {};
  double commanded_leg_length = 0.18;
  double commanded_leg_length_rate = 0.0;
  double commanded_leg_angle = 0.0;
  double axial_force[2] = {};
  double integral_force[2] = {};
  double leg_angle_error[2] = {};
  double leg_angle_torque[2] = {};
  double requested_wheel_torque = 0.0;
  double wheel_attitude_torque_contribution = 0.0;
  double wheel_translation_torque_contribution = 0.0;
  double filtered_pitch_rate = 0.0;
  double world_leg_angle = 0.0;
  double world_leg_angle_reference = 0.0;
  double world_leg_angle_error = 0.0;
  double world_leg_angle_torque = 0.0;
  double leg_split_hold_error = 0.0;
  double leg_split_hold_torque = 0.0;
  double requested_leg_angle_torque = 0.0;
  double applied_wheel_torque = 0.0;
  double applied_leg_angle_torque = 0.0;
  double lqr_scale = 0.0;
  double yaw_rate = 0.0;
  double yaw_rate_error = 0.0;
  double yaw_authority_scale = 1.0;
  double yaw_attitude_authority = 1.0;
  double yaw_contact_authority = 1.0;
  double yaw_split_authority = 1.0;
  double yaw_split_residual_authority = 1.0;
  double yaw_split_absolute_authority = 1.0;
  double spin_mode_blend = 0.0;
  double reserved_yaw_torque_per_wheel = 0.0;
  double balance_torque_authority = 1.0;
  double coordinated_yaw_rate = 0.0;
  double wheel_normal_force[2] = {};
  double applied_yaw_torque = 0.0;
  double commanded_yaw_rate = 0.0;
  double differential_leg_angle_error = 0.0;
  double differential_leg_angle_rate = 0.0;
  double differential_leg_angle_torque = 0.0;
  double state_error[6] = {};
  bool wheel_grounded[2] = {};
  bool balance_active = false;
  WbrContactSafetyState contact_safety_state =
      WbrContactSafetyState::kAirborne;
  double contact_authority_scale = 0.0;
  double estimated_x_speed = 0.0;
  double wheel_odometry_x_speed = 0.0;
  double wheel_odometry_confidence = 0.0;
};

#endif  // WBR_CONTROL_CORE_CONTROLLER_TYPES_H_
