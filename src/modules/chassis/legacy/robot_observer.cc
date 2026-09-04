/**
* @file src/modules/chassis/legacy/robot_observer.cc
 * @ingroup wbr_modules
 * @brief 实现轮腿机器人状态观测与融合。
 * @details 该组件属于可独立计算的传统底盘控制链路。调用方按固定周期提供一致输入快照；组件保存的积分或滤波状态必须在失能、故障或时间跳变后复位。
 */

// Legacy controller stack; excluded from the firmware build.
#include "robot_observer.h"

#include <cmath>

#include "leg_kinematics.h"
#include "math_utils.h"

namespace wbr::control {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kLeftLegKinematicBranch = 1;
constexpr int kRightLegKinematicBranch = -1;

double WrapAngle(double angle) {
  return std::remainder(angle, 2.0 * kPi);
}

bool Fresh(bool valid, std::uint64_t timestamp_us, std::uint64_t now_us,
           std::uint64_t maximum_age_us) {
  return valid && timestamp_us <= now_us &&
      now_us - timestamp_us <= maximum_age_us;
}

int Index(MotorId id) {
  return static_cast<int>(id);
}

double FirstOrderAlpha(double dt, double time_constant) {
  if (time_constant <= 0.0) {
    return 1.0;
  }
  return v2::Clamp(dt / time_constant, 0.0, 1.0);
}

}  // namespace

void RobotObserver::Reset() {
  initialized_ = false;
  roll_ = pitch_ = x_ = x_speed_ = 0.0;
}

ObservationResult RobotObserver::Update(
    const ControlInput& sample, const RobotParameters& parameters,
    std::uint64_t now_us) {
  ObservationResult result;
  if (!std::isfinite(sample.dt) ||
      sample.dt < parameters.minimum_control_period ||
      sample.dt > parameters.maximum_control_period) {
    return result;
  }
  const double dt = v2::Clamp(sample.dt, 1e-4, 0.02);
  result.input.control_dt = dt;
  result.input.total_mass = parameters.total_mass;
  result.input.gravity_magnitude = parameters.gravity;
  result.input.target_leg_length = sample.command.leg_length;
  result.input.target_leg_angle = sample.command.leg_angle;
  result.input.yaw_plant_enabled = true;

  const bool imu_fresh = Fresh(
      sample.imu.valid, sample.imu.timestamp_us, now_us,
      parameters.maximum_sample_age_us);
  bool leg_motors_fresh = true;
  for (int motor = Index(MotorId::kLeftJointB);
       motor <= Index(MotorId::kRightJointD); ++motor) {
    leg_motors_fresh = leg_motors_fresh && Fresh(
        sample.motor[motor].valid, sample.motor[motor].timestamp_us, now_us,
        parameters.maximum_sample_age_us);
  }
  if (!imu_fresh || !leg_motors_fresh) return result;

  const double ax = sample.imu.acceleration[0];
  const double ay = sample.imu.acceleration[1];
  const double az = sample.imu.acceleration[2];
  bool accelerometer_trusted = false;
  double accel_roll = 0.0;
  double accel_pitch = 0.0;
  if (!sample.imu.attitude_valid) {
    const double acceleration_norm = std::sqrt(ax * ax + ay * ay + az * az);
    accelerometer_trusted =
        std::fabs(acceleration_norm - parameters.gravity) <=
        parameters.acceleration_norm_tolerance;
    if (accelerometer_trusted) {
      accel_roll = std::atan2(ay, az);
      accel_pitch = std::atan2(-ax, std::hypot(ay, az));
    }
  }

  if (!initialized_) {
    if (sample.imu.attitude_valid) {
      roll_ = sample.imu.roll;
      pitch_ = sample.imu.pitch;
    } else if (accelerometer_trusted) {
      roll_ = accel_roll;
      pitch_ = accel_pitch;
    }
    x_ = x_speed_ = 0.0;
    initialized_ = true;
    result.input.time_reset = true;
  } else {
    roll_ = WrapAngle(roll_ + sample.imu.angular_velocity[0] * dt);
    pitch_ = WrapAngle(pitch_ + sample.imu.angular_velocity[1] * dt);
    const double correction_alpha = FirstOrderAlpha(
        dt, parameters.attitude_correction_time_constant);
    if (sample.imu.attitude_valid) {
      roll_ += correction_alpha * WrapAngle(sample.imu.roll - roll_);
      pitch_ += correction_alpha * WrapAngle(sample.imu.pitch - pitch_);
    } else if (accelerometer_trusted) {
      roll_ += correction_alpha * WrapAngle(accel_roll - roll_);
      pitch_ += correction_alpha * WrapAngle(accel_pitch - pitch_);
    }
  }

  const MotorFeedback& left_b = sample.motor[Index(MotorId::kLeftJointB)];
  const MotorFeedback& left_d = sample.motor[Index(MotorId::kLeftJointD)];
  const MotorFeedback& right_b = sample.motor[Index(MotorId::kRightJointB)];
  const MotorFeedback& right_d = sample.motor[Index(MotorId::kRightJointD)];
  result.input.leg_valid[0] = v2::ComputeLegKinematics(
      left_d.position, left_b.position, left_d.velocity, left_b.velocity,
      kLeftLegKinematicBranch, result.input.leg[0]);
  result.input.leg_valid[1] = v2::ComputeLegKinematics(
      right_d.position, right_b.position, right_d.velocity, right_b.velocity,
      kRightLegKinematicBranch, result.input.leg[1]);

  const double left_wheel_speed =
      Fresh(sample.motor[Index(MotorId::kLeftWheel)].valid,
            sample.motor[Index(MotorId::kLeftWheel)].timestamp_us, now_us,
            parameters.maximum_sample_age_us)
          ? sample.motor[Index(MotorId::kLeftWheel)].velocity
          : 0.0;
  const double right_wheel_speed =
      Fresh(sample.motor[Index(MotorId::kRightWheel)].valid,
            sample.motor[Index(MotorId::kRightWheel)].timestamp_us, now_us,
            parameters.maximum_sample_age_us)
          ? sample.motor[Index(MotorId::kRightWheel)].velocity
          : 0.0;
  // The wheel motors are mounted as mirrored axes.  The direction probe
  // established that positive raw left-wheel speed is chassis-backward,
  // while positive raw right-wheel speed is chassis-forward.
  const double wheel_odometry_speed = 0.5 * parameters.wheel_radius *
      (-left_wheel_speed + right_wheel_speed);
  const double forward_acceleration =
      std::cos(pitch_) * ax + std::sin(pitch_) * az;
  if (!result.input.time_reset) {
    x_speed_ += forward_acceleration * dt;
    const double minimum_contact = std::fmin(
        sample.contact.confidence[0], sample.contact.confidence[1]);
    if (minimum_contact >= parameters.contact_threshold) {
      const double odometry_alpha = FirstOrderAlpha(
          dt, parameters.velocity_correction_time_constant);
      x_speed_ += odometry_alpha * (wheel_odometry_speed - x_speed_);
    }
    x_ += x_speed_ * dt;
  }

  result.input.roll = roll_;
  result.input.pitch = pitch_;
  result.input.roll_rate = sample.imu.angular_velocity[0];
  result.input.pitch_rate = sample.imu.angular_velocity[1];
  result.input.yaw_rate = sample.imu.angular_velocity[2];
  result.input.x = x_;
  result.input.x_speed = x_speed_;
  result.input.wheel_odometry_x_speed = wheel_odometry_speed;
  result.input.wheel_odometry_confidence = v2::Clamp(
      std::fmin(sample.contact.confidence[0],
                sample.contact.confidence[1]),
      0.0, 1.0);
  const double nominal_normal_force =
      0.5 * parameters.total_mass * parameters.gravity;
  for (int wheel = 0; wheel < 2; ++wheel) {
    const double confidence = v2::Clamp(
        sample.contact.confidence[wheel], 0.0, 1.0);
    result.input.wheel_grounded[wheel] =
        confidence >= parameters.contact_threshold;
    result.input.wheel_normal_force[wheel] =
        confidence * nominal_normal_force;
  }
  result.input.observation_valid =
      result.input.leg_valid[0] && result.input.leg_valid[1];
  result.valid = result.input.observation_valid;
  return result;
}

}  // namespace wbr::control
