#include "ground_balance_controller.h"
#include "control_parameters.h"
#include "contact_safety.h"
#include "lqr_schedule.h"
#include "leg_kinematics.h"
#include "math_utils.h"
#include "wheel_allocator.h"
#include "yaw_coordinator.h"

#include <cmath>

using namespace wbr::v2;

void GroundBalanceController::Reset() {
  leg_speed_[0] = leg_speed_[1] = 0.0;
  leg_angle_speed_[0] = leg_angle_speed_[1] = 0.0;
  filtered_yaw_speed_ = 0.0;
  filtered_yaw_acceleration_ = previous_yaw_speed_ = 0.0;
  x_reference_ = 0.0;
  target_linear_velocity_ = target_yaw_rate_ = 0.0;
  commanded_linear_velocity_ = 0.0;
  yaw_coordinator_.Reset();
  commanded_yaw_torque_ = 0.0;
  commanded_differential_leg_angle_torque_ = 0.0;
  commanded_leg_length_ = 0.18;
  commanded_leg_angle_ = 0.0;
  support_factor_[0] = support_factor_[1] = 0.0;
  leg_length_integral_[0] = leg_length_integral_[1] = 0.0;
  contact_safety_.Reset();
  contact_leg_length_offset_[0] = contact_leg_length_offset_[1] = 0.0;
  contact_safety_state_ = WbrContactSafetyState::kAirborne;
  command_initialized_ = lqr_initialized_ = false;
  telemetry_ = {};
}

void GroundBalanceController::InitializeLegTarget(double length,
                                                  double angle) {
  commanded_leg_length_ = length;
  commanded_leg_angle_ = angle;
  command_initialized_ = true;
}

GroundBalanceOutput GroundBalanceController::Update(
    const GroundBalanceInput& input) {
  GroundBalanceOutput output;
  telemetry_ = {};

  output.sanitized_leg_length = Clamp(
      input.target_leg_length, kMinTargetLegLength, kTargetHRadius);
  output.sanitized_leg_angle = Clamp(
      input.target_leg_angle, -kMaxTargetLegAngle, kMaxTargetLegAngle);
  const bool time_reset = input.time_reset;
  const double control_dt = Clamp(input.control_dt, 1e-4, 0.02);
  target_linear_velocity_ = Clamp(target_linear_velocity_,
                                  -kMaxLinearVelocity, kMaxLinearVelocity);
  const double safe_target_yaw_rate = Clamp(
      target_yaw_rate_, -kSustainedYawRateLimit, kSustainedYawRateLimit);
  commanded_linear_velocity_ = MoveTowards(
      commanded_linear_velocity_, target_linear_velocity_,
      kLinearAccelerationLimit * control_dt);
  telemetry_.commanded_yaw_rate = safe_target_yaw_rate;

  const LegKinematics& leg_1 = input.leg[0];
  const LegKinematics& leg_2 = input.leg[1];
  const bool first_leg_valid = input.leg_valid[0];
  const bool second_leg_valid = input.leg_valid[1];

  if (first_leg_valid) {
    telemetry_.leg_length[0] = leg_1.length;
    telemetry_.leg_length_rate[0] = leg_1.length_rate;
    telemetry_.leg_angle[0] = leg_1.angle;
    telemetry_.leg_angle_rate[0] = leg_1.angle_rate;
    leg_speed_[0] += kLegSpeedFilter * (leg_1.length_rate - leg_speed_[0]);
    leg_angle_speed_[0] +=
        kStateSpeedFilter * (leg_1.angle_rate - leg_angle_speed_[0]);
  }
  if (second_leg_valid) {
    telemetry_.leg_length[1] = leg_2.length;
    telemetry_.leg_length_rate[1] = leg_2.length_rate;
    telemetry_.leg_angle[1] = leg_2.angle;
    telemetry_.leg_angle_rate[1] = leg_2.angle_rate;
    leg_speed_[1] += kLegSpeedFilter * (leg_2.length_rate - leg_speed_[1]);
    leg_angle_speed_[1] +=
        kStateSpeedFilter * (leg_2.angle_rate - leg_angle_speed_[1]);
  }

  if (!command_initialized_ && first_leg_valid && second_leg_valid) {
    commanded_leg_length_ = 0.5 * (leg_1.length + leg_2.length);
    commanded_leg_angle_ =
        std::atan2(std::sin(leg_1.angle) + std::sin(leg_2.angle),
                   std::cos(leg_1.angle) + std::cos(leg_2.angle));
    command_initialized_ = true;
  }
  const double previous_commanded_leg_length = commanded_leg_length_;
  commanded_leg_length_ =
      MoveTowards(commanded_leg_length_, output.sanitized_leg_length,
                  kTargetLengthSlewRate * control_dt);
  telemetry_.commanded_leg_length = commanded_leg_length_;
  telemetry_.commanded_leg_length_rate =
      (commanded_leg_length_ - previous_commanded_leg_length) / control_dt;
  commanded_leg_angle_ =
      MoveTowards(commanded_leg_angle_, output.sanitized_leg_angle,
                  kTargetAngleSlewRate * control_dt);
  telemetry_.commanded_leg_angle = commanded_leg_angle_;

  double total_wheel_torque = 0.0;
  double total_leg_angle_torque = 0.0;
  double total_yaw_torque = 0.0;
  double differential_leg_angle_torque = 0.0;
  double support_feedforward_1 = 0.0;
  double support_feedforward_2 = 0.0;
  bool effective_wheel_grounded[2] = {false, false};
  const bool lqr_state_valid = first_leg_valid && second_leg_valid &&
      input.observation_valid;
  if (lqr_state_valid) {
    const bool wheel_1_grounded = input.wheel_grounded[0];
    const bool wheel_2_grounded = input.wheel_grounded[1];
    const bool raw_wheel_grounded[2] = {
        wheel_1_grounded, wheel_2_grounded};
    const double imu_roll_rate = input.roll_rate;
    const double imu_pitch_rate = input.pitch_rate;
    const double imu_yaw_rate = input.yaw_rate;
    const double x = input.x;
    const double raw_x_speed = input.x_speed;
    const double raw_pitch_speed = imu_pitch_rate;
    const double phi = input.pitch;
    telemetry_.estimated_x_speed = input.x_speed;
    telemetry_.wheel_odometry_x_speed = input.wheel_odometry_x_speed;
    telemetry_.wheel_odometry_confidence =
        input.wheel_odometry_confidence;

    const ContactSafetyOutput contact = contact_safety_.Update(
        raw_wheel_grounded, input.roll, input.pitch, control_dt);
    contact_safety_state_ = contact.state;
    const bool wheel_1_effectively_grounded = contact.effective_grounded[0];
    const bool wheel_2_effectively_grounded = contact.effective_grounded[1];
    effective_wheel_grounded[0] = wheel_1_effectively_grounded;
    effective_wheel_grounded[1] = wheel_2_effectively_grounded;
    const bool single_support = contact.single_support;
    const bool contact_airborne = contact.airborne;
    if (contact.state_changed) {
      // Re-anchor when support changes so stale position debt is not carried
      // across a wheel-slip or airborne interval.
      x_reference_ = x;
    }
    const double contact_authority_scale = contact.authority;
    telemetry_.contact_safety_state = contact_safety_state_;
    telemetry_.contact_authority_scale = contact_authority_scale;

    if (single_support || contact_airborne) {
      commanded_linear_velocity_ = MoveTowards(
          commanded_linear_velocity_, 0.0,
          kEmergencyLinearDeceleration * control_dt);
    }
    const bool balance_active =
        lqr_enabled_ &&
        !contact_airborne &&
        (wheel_1_effectively_grounded || wheel_2_effectively_grounded);
    telemetry_.wheel_grounded[0] = wheel_1_grounded;
    telemetry_.wheel_grounded[1] = wheel_2_grounded;
    telemetry_.balance_active = balance_active;
    if (balance_active) {
      const bool reset_lqr = !lqr_initialized_ || time_reset;
      if (reset_lqr) {
        x_reference_ = x;
        filtered_yaw_speed_ = imu_yaw_rate;
        previous_yaw_speed_ = imu_yaw_rate;
        filtered_yaw_acceleration_ = 0.0;
        lqr_initialized_ = true;
      } else {
        const double yaw_rate_alpha =
            1.0 - std::exp(-control_dt / kYawRateFilterTimeConstant);
        filtered_yaw_speed_ +=
            yaw_rate_alpha * (imu_yaw_rate - filtered_yaw_speed_);
      }
      if (!reset_lqr && !time_reset && !single_support) {
        // Keep the body-forward position reference active during pure spin.
        // Releasing it with x_reference=x permitted a constant sagittal drift,
        // which appears as a curved left-forward path while yawing.
        x_reference_ += commanded_linear_velocity_ * control_dt;
        x_reference_ = Clamp(x_reference_, x - kMaxPositionTrackingError,
                             x + kMaxPositionTrackingError);
      }
    } else {
      lqr_initialized_ = false;
      filtered_yaw_speed_ = imu_yaw_rate;
    }
    const double world_leg_angle_1 =
        std::remainder(leg_1.angle + phi, kTwoPi);
    const double world_leg_angle_2 =
        std::remainder(leg_2.angle + phi, kTwoPi);
    const double theta =
        std::atan2(std::sin(world_leg_angle_1) + std::sin(world_leg_angle_2),
                   std::cos(world_leg_angle_1) + std::cos(world_leg_angle_2));
    const double theta_rate =
        0.5 * (leg_1.angle_rate + leg_2.angle_rate) + raw_pitch_speed;

    telemetry_.differential_leg_angle_error = std::remainder(
        leg_1.angle - leg_2.angle, kTwoPi);
    telemetry_.differential_leg_angle_rate =
        leg_angle_speed_[0] - leg_angle_speed_[1];

    // Stage 4: update contact safety and yaw-authority coordination.
    telemetry_.wheel_normal_force[0] = input.wheel_normal_force[0];
    telemetry_.wheel_normal_force[1] = input.wheel_normal_force[1];
    YawCoordinatorInput yaw_input;
    yaw_input.reference_enabled = yaw_enabled_ &&
        contact_safety_state_ == WbrContactSafetyState::kDualSupport &&
        wheel_1_effectively_grounded && wheel_2_effectively_grounded;
    yaw_input.time_reset = time_reset;
    yaw_input.control_dt = control_dt;
    yaw_input.commanded_yaw_rate = safe_target_yaw_rate;
    yaw_input.measured_yaw_rate = filtered_yaw_speed_;
    yaw_input.split_angle = telemetry_.differential_leg_angle_error;
    yaw_input.split_rate = telemetry_.differential_leg_angle_rate;
    yaw_input.roll = input.roll;
    yaw_input.roll_rate = imu_roll_rate;
    yaw_input.pitch = phi;
    yaw_input.normal_force[0] = telemetry_.wheel_normal_force[0];
    yaw_input.normal_force[1] = telemetry_.wheel_normal_force[1];
    const YawCoordinatorOutput yaw_coord = yaw_coordinator_.Update(yaw_input);

    telemetry_.coordinated_yaw_rate = yaw_coord.coordinated_yaw_rate;
    telemetry_.yaw_split_authority = yaw_coord.split_authority;
    telemetry_.yaw_split_residual_authority =
        yaw_coord.split_residual_authority;
    telemetry_.yaw_split_absolute_authority =
        yaw_coord.split_absolute_authority;
    telemetry_.yaw_attitude_authority = yaw_coord.attitude_authority;
    telemetry_.yaw_contact_authority = yaw_coord.contact_authority;
    telemetry_.yaw_authority_scale = yaw_coord.authority;
    if (wheel_1_grounded || wheel_2_grounded) {
      differential_leg_angle_torque = Clamp(
          -kDifferentialLegAngleKp *
              telemetry_.differential_leg_angle_error -
          kDifferentialLegAngleKd *
              telemetry_.differential_leg_angle_rate,
          -kDifferentialLegAngleTorqueLimit,
          kDifferentialLegAngleTorqueLimit);
    }
    const double support_alpha =
        1.0 - std::exp(-control_dt / kSupportFilterTimeConstant);
    support_factor_[0] +=
        support_alpha *
        ((wheel_1_grounded ? 1.0 : 0.0) -
         support_factor_[0]);
    support_factor_[1] +=
        support_alpha *
        ((wheel_2_grounded ? 1.0 : 0.0) -
         support_factor_[1]);
    const double half_weight =
        0.5 * input.total_mass * input.gravity_magnitude;
    if (contact_safety_state_ == WbrContactSafetyState::kSingleSupportFirst) {
      support_feedforward_1 = half_weight /
          std::fmax(std::cos(world_leg_angle_1), 0.5);
      support_feedforward_2 = 0.0;
    } else if (contact_safety_state_ ==
               WbrContactSafetyState::kSingleSupportSecond) {
      support_feedforward_1 = 0.0;
      support_feedforward_2 = half_weight /
          std::fmax(std::cos(world_leg_angle_2), 0.5);
    } else {
      support_feedforward_1 =
          support_factor_[0] * half_weight /
          std::fmax(std::cos(world_leg_angle_1), 0.5);
      support_feedforward_2 =
          support_factor_[1] * half_weight /
          std::fmax(std::cos(world_leg_angle_2), 0.5);
    }

    // The sagittal LQR controls the common leg force. Stabilize the orthogonal
    // roll mode with differential axial force, whose sign and gain are
    // identified by lqr_open_loop_test on the free-root model.
    if (wheel_1_grounded && wheel_2_grounded && !single_support &&
        !contact_airborne) {
      const double roll = input.roll;
      const double roll_rate = imu_roll_rate;
      const double differential_force = Clamp(
          -kRollForceKp * roll - kRollForceKd * roll_rate,
          -kRollDifferentialForceLimit, kRollDifferentialForceLimit);
      support_feedforward_1 += 0.5 * differential_force;
      support_feedforward_2 -= 0.5 * differential_force;
    }

    const double state_error[6] = {
        std::remainder(theta - commanded_leg_angle_, kTwoPi),
        theta_rate,
        x - x_reference_,
        raw_x_speed - commanded_linear_velocity_,
        phi,
        raw_pitch_speed,
    };
    for (int state = 0; state < 6; ++state) {
      telemetry_.state_error[state] = state_error[state];
    }
    if (balance_active) {
      // Stage 5: compose sagittal LQR, roll control and yaw requests.
      double scheduled_lqr_gain[2][6];
      EvaluateLqrGain(0.5 * (leg_1.length + leg_2.length),
                      scheduled_lqr_gain);
      for (int state = 0; state < 6; ++state) {
        total_wheel_torque -=
            scheduled_lqr_gain[0][state] * state_error[state];
        total_leg_angle_torque -=
            scheduled_lqr_gain[1][state] * state_error[state];
      }
      total_wheel_torque *= contact_authority_scale;
      total_leg_angle_torque *= contact_authority_scale;
      telemetry_.requested_wheel_torque = total_wheel_torque;

      // First form the yaw request from attitude/contact/split safety.  Wheel
      // torque availability is handled by the allocator below; feeding it
      // back here creates a deadlock where yaw cannot request the headroom it
      // needs in order to start.
      telemetry_.spin_mode_blend = Clamp(
          (std::fabs(safe_target_yaw_rate) - kSpinModeEntryYawRate) /
              (kSpinModeFullYawRate - kSpinModeEntryYawRate),
          0.0, 1.0);
      bool yaw_braking_request = false;
      if (yaw_enabled_ &&
          contact_safety_state_ == WbrContactSafetyState::kDualSupport &&
          wheel_1_grounded && wheel_2_grounded &&
          input.yaw_plant_enabled) {
        telemetry_.yaw_rate = filtered_yaw_speed_;
        telemetry_.yaw_rate_error =
            yaw_coord.coordinated_yaw_rate - telemetry_.yaw_rate;
        const double raw_yaw_acceleration =
            (filtered_yaw_speed_ - previous_yaw_speed_) / control_dt;
        const double yaw_acceleration_alpha = 1.0 - std::exp(
            -control_dt / kYawAccelerationFilterTimeConstant);
        filtered_yaw_acceleration_ += yaw_acceleration_alpha *
            (raw_yaw_acceleration - filtered_yaw_acceleration_);
        previous_yaw_speed_ = filtered_yaw_speed_;
        const double yaw_drive_torque =
            kYawRateKp * telemetry_.yaw_rate_error -
            kYawAccelerationKd * filtered_yaw_acceleration_;
        yaw_braking_request =
            std::fabs(telemetry_.yaw_rate) > 0.02 &&
            yaw_drive_torque * telemetry_.yaw_rate < 0.0;
        // Keep wheel-speed difference as slip/kinematics telemetry only.  The
        // yaw-rate loop follows the SPR structure and closes exclusively on
        // IMU wz, avoiding wheel-contact and mirrored-axis errors in feedback.
        // The coordinator shapes the target before the PD. A square-root
        // safety gate remains on the resulting torque: it is mild at partial
        // authority, but removes braking torque completely at a hard limit.
        // Safety authority suppresses torque that would add yaw energy.  Keep
        // counter-yaw braking available when roll/split authority collapses;
        // otherwise the robot coasts through the safety boundary and tips.
        const double yaw_drive_authority = yaw_braking_request
            ? 1.0
            : std::sqrt(telemetry_.yaw_authority_scale);
        const double requested_yaw_torque =
            yaw_drive_authority * yaw_drive_torque;
        total_yaw_torque = Clamp(requested_yaw_torque,
                                 -kTotalYawTorqueLimit,
                                 kTotalYawTorqueLimit);
      }

      // Jointly allocate the common balance torque and differential yaw
      // torque.  Spin mode reserves only what the current yaw request needs,
      // capped at 2 N.m/side; unsafe yaw posture immediately returns that
      // capacity to sagittal balance.
      const double demanded_yaw_reserve_per_wheel = Clamp(
          0.5 * kDecoupledYawWheelInputScale *
                  std::fmax(std::fabs(total_yaw_torque),
                            std::fabs(commanded_yaw_torque_)) +
              kSpinYawReserveBufferPerWheel,
          0.0, kSpinYawReservePerWheel);
      telemetry_.reserved_yaw_torque_per_wheel =
          demanded_yaw_reserve_per_wheel * telemetry_.spin_mode_blend *
          (yaw_braking_request ? 1.0 : telemetry_.yaw_authority_scale);
      const double balance_torque_limit = 2.0 * std::fmax(
          0.0, kPerWheelTorqueLimit -
                   telemetry_.reserved_yaw_torque_per_wheel);
      const double requested_balance_torque = total_wheel_torque;
      total_wheel_torque = Clamp(total_wheel_torque,
                                 -balance_torque_limit,
                                 balance_torque_limit);
      telemetry_.balance_torque_authority =
          std::fabs(requested_balance_torque) > 1e-9
              ? std::fabs(total_wheel_torque / requested_balance_torque)
              : 1.0;
    }
    telemetry_.requested_leg_angle_torque = total_leg_angle_torque;
    total_wheel_torque = Clamp(total_wheel_torque, -kTotalWheelTorqueLimit,
                               kTotalWheelTorqueLimit);
    total_leg_angle_torque =
        Clamp(total_leg_angle_torque, -kTotalLegAngleTorqueLimit,
              kTotalLegAngleTorqueLimit);
    telemetry_.applied_wheel_torque = total_wheel_torque;
    telemetry_.applied_leg_angle_torque = total_leg_angle_torque;
  }

  if (time_reset) {
    leg_length_integral_[0] = 0.0;
    leg_length_integral_[1] = 0.0;
  }
  const LegKinematics* legs[2] = {&leg_1, &leg_2};
  const double support_feedforward[2] = {
      support_feedforward_1, support_feedforward_2};
  const bool leg_valid[2] = {first_leg_valid, second_leg_valid};
  for (int leg = 0; leg < 2; ++leg) {
    if (leg_valid[leg]) {
      const double error = commanded_leg_length_ - legs[leg]->length;
      const double integral_limit = kLegIntegralForceLimit / kLegLengthKi;
      const double candidate = Clamp(
          leg_length_integral_[leg] + error * control_dt,
          -integral_limit, integral_limit);
      const double candidate_force =
          kLegLengthKp * error - kLegLengthKd * leg_speed_[leg] +
          support_feedforward[leg] + kLegLengthKi * candidate;
      const bool pushes_further_into_saturation =
          (candidate_force > kLegForceLimit && error > 0.0) ||
          (candidate_force < -kLegForceLimit && error < 0.0);
      if (!pushes_further_into_saturation) {
        leg_length_integral_[leg] = candidate;
      }
    } else {
      leg_length_integral_[leg] *= std::exp(-control_dt / 0.25);
    }
    telemetry_.integral_force[leg] =
        kLegLengthKi * leg_length_integral_[leg];
  }

  const bool yaw_command_released = std::fabs(target_yaw_rate_) < 1e-5;
  const bool yaw_command_reversed =
      target_yaw_rate_ * commanded_yaw_torque_ < 0.0;
  const bool yaw_torque_braking =
      yaw_command_reversed ||
      (yaw_command_released &&
       std::fabs(total_yaw_torque) < std::fabs(commanded_yaw_torque_));
  commanded_yaw_torque_ = MoveTowards(
      commanded_yaw_torque_, total_yaw_torque,
      (yaw_torque_braking ? kYawTorqueBrakeRate : kYawTorqueRiseRate) *
          control_dt);
  const bool yaw_actuation_disabled =
      contact_safety_state_ == WbrContactSafetyState::kSingleSupportFirst ||
      contact_safety_state_ == WbrContactSafetyState::kSingleSupportSecond ||
      contact_safety_state_ == WbrContactSafetyState::kAirborne;
  if (yaw_actuation_disabled) {
    commanded_yaw_torque_ = 0.0;
  }
  total_yaw_torque = commanded_yaw_torque_;

  // Stage 6: synchronize the identified yaw/leg-split actuator pair.
  differential_leg_angle_torque +=
      kDecoupledYawLegTorquePerCommand * total_yaw_torque;
  differential_leg_angle_torque = Clamp(
      differential_leg_angle_torque, -kDifferentialLegAngleTorqueLimit,
      kDifferentialLegAngleTorqueLimit);
  commanded_differential_leg_angle_torque_ = MoveTowards(
      commanded_differential_leg_angle_torque_, differential_leg_angle_torque,
      kDifferentialLegAngleTorqueSlewRate * control_dt);
  differential_leg_angle_torque = commanded_differential_leg_angle_torque_;
  if (yaw_actuation_disabled) {
    commanded_differential_leg_angle_torque_ = 0.0;
    differential_leg_angle_torque = 0.0;
  }
  telemetry_.differential_leg_angle_torque = differential_leg_angle_torque;

  // Stage 7: map generalized leg commands through VMC to joint actuators.
  // The LQR's Tp uses the paper's body-on-leg torque convention.  The VMC
  // below applies leg-on-body generalized force, which is the opposite side
  // of that action/reaction pair (the identification path uses the same
  // conversion in ApplyLegController).  Convert conventions before J^T F.
  const double common_leg_angle_torque = -0.5 * total_leg_angle_torque;
  double first_leg_angle_torque =
      common_leg_angle_torque + 0.5 * differential_leg_angle_torque;
  double second_leg_angle_torque =
      common_leg_angle_torque - 0.5 * differential_leg_angle_torque;
  if (first_leg_valid) {
    telemetry_.leg_angle_error[0] =
        std::remainder(commanded_leg_angle_ - leg_1.angle, kTwoPi);
    telemetry_.leg_angle_torque[0] = Clamp(
        kLegAngleKp * telemetry_.leg_angle_error[0] -
            kLegAngleKd * leg_angle_speed_[0],
        -kLegAngleTorqueLimit, kLegAngleTorqueLimit);
    first_leg_angle_torque += telemetry_.leg_angle_torque[0];
  }
  if (second_leg_valid) {
    telemetry_.leg_angle_error[1] =
        std::remainder(commanded_leg_angle_ - leg_2.angle, kTwoPi);
    telemetry_.leg_angle_torque[1] = Clamp(
        kLegAngleKp * telemetry_.leg_angle_error[1] -
            kLegAngleKd * leg_angle_speed_[1],
        -kLegAngleTorqueLimit, kLegAngleTorqueLimit);
    second_leg_angle_torque += telemetry_.leg_angle_torque[1];
  }
  double desired_contact_leg_offset[2] = {0.0, 0.0};
  if (contact_safety_state_ == WbrContactSafetyState::kSingleSupportFirst) {
    desired_contact_leg_offset[0] = -kGroundedLegYield;
    desired_contact_leg_offset[1] = kAirborneLegSearchExtension;
  } else if (contact_safety_state_ ==
             WbrContactSafetyState::kSingleSupportSecond) {
    desired_contact_leg_offset[0] = kAirborneLegSearchExtension;
    desired_contact_leg_offset[1] = -kGroundedLegYield;
  }
  for (int leg = 0; leg < 2; ++leg) {
    contact_leg_length_offset_[leg] = MoveTowards(
        contact_leg_length_offset_[leg], desired_contact_leg_offset[leg],
        kContactLegOffsetSlewRate * control_dt);
  }
  const double first_leg_target_length = Clamp(
      commanded_leg_length_ + contact_leg_length_offset_[0],
      kMinTargetLegLength, kTargetHRadius);
  const double second_leg_target_length = Clamp(
      commanded_leg_length_ + contact_leg_length_offset_[1],
      kMinTargetLegLength, kTargetHRadius);
  const auto motor_index = [](control::MotorId id) {
    return static_cast<int>(id);
  };
  if (first_leg_valid) {
    const LegVmcOutput vmc = ComputeLegVmc(
        leg_1, first_leg_target_length, support_feedforward_1,
        telemetry_.integral_force[0], first_leg_angle_torque, leg_speed_[0],
        telemetry_.commanded_leg_length_rate);
    telemetry_.axial_force[0] = vmc.axial_force;
    auto& joint_b = output.actuator.motor[
        motor_index(control::MotorId::kLeftJointB)];
    auto& joint_d = output.actuator.motor[
        motor_index(control::MotorId::kLeftJointD)];
    joint_b.torque = Clamp(
        vmc.joint_torque[1], -kJointTorqueLimit, kJointTorqueLimit);
    joint_d.torque = Clamp(
        vmc.joint_torque[0], -kJointTorqueLimit, kJointTorqueLimit);
    joint_b.enabled = joint_d.enabled = true;
  }
  if (second_leg_valid) {
    const LegVmcOutput vmc = ComputeLegVmc(
        leg_2, second_leg_target_length, support_feedforward_2,
        telemetry_.integral_force[1], second_leg_angle_torque, leg_speed_[1],
        telemetry_.commanded_leg_length_rate);
    telemetry_.axial_force[1] = vmc.axial_force;
    auto& joint_b = output.actuator.motor[
        motor_index(control::MotorId::kRightJointB)];
    auto& joint_d = output.actuator.motor[
        motor_index(control::MotorId::kRightJointD)];
    joint_b.torque = Clamp(
        vmc.joint_torque[1], -kJointTorqueLimit, kJointTorqueLimit);
    joint_d.torque = Clamp(
        vmc.joint_torque[0], -kJointTorqueLimit, kJointTorqueLimit);
    joint_b.enabled = joint_d.enabled = true;
  }

  WheelAllocationInput allocation_input;
  allocation_input.balance_torque = total_wheel_torque;
  allocation_input.yaw_torque = total_yaw_torque;
  allocation_input.grounded[0] = effective_wheel_grounded[0];
  allocation_input.grounded[1] = effective_wheel_grounded[1];
  allocation_input.contact_state = contact_safety_state_;
  const WheelAllocationOutput allocation =
      AllocateWheelTorque(allocation_input);
  auto& left_wheel = output.actuator.motor[
      motor_index(control::MotorId::kLeftWheel)];
  auto& right_wheel = output.actuator.motor[
      motor_index(control::MotorId::kRightWheel)];
  left_wheel.torque = allocation.actuator_torque[0];
  right_wheel.torque = -allocation.actuator_torque[1];
  left_wheel.enabled = right_wheel.enabled = input.observation_valid;
  telemetry_.applied_yaw_torque = allocation.applied_yaw_torque;
  return output;
}
