// Legacy controller stack; excluded from the firmware build.
#ifndef WBR_CONTROL_CORE_CONTROLLER_IO_H_
#define WBR_CONTROL_CORE_CONTROLLER_IO_H_

#include <cstdint>

namespace wbr::control {

enum class MotorId : std::uint8_t {
  kLeftJointB,
  kLeftJointD,
  kRightJointB,
  kRightJointD,
  kLeftWheel,
  kRightWheel,
  kCount,
};

inline constexpr int kMotorCount = static_cast<int>(MotorId::kCount);

struct ImuState {
  double roll = 0.0;
  double pitch = 0.0;
  double angular_velocity[3] = {};
  double acceleration[3] = {};
  std::uint64_t timestamp_us = 0;
  bool valid = false;
  bool attitude_valid = false;
};

struct MotorFeedback {
  double position = 0.0;
  double velocity = 0.0;
  double torque = 0.0;
  std::uint64_t timestamp_us = 0;
  bool valid = false;
};

struct ContactObservation {
  double confidence[2] = {};
};

struct ControlCommand {
  double linear_velocity = 0.0;
  double yaw_rate = 0.0;
  double leg_length = 0.18;
  double leg_angle = 0.0;
  std::uint64_t timestamp_us = 0;
  bool valid = false;
  bool enabled = false;
};

struct ControlInput {
  double dt = 0.001;
  ImuState imu;
  MotorFeedback motor[kMotorCount];
  ContactObservation contact;
  ControlCommand command;
};

struct MotorCommand {
  double torque = 0.0;
  bool enabled = false;
};

struct ControlOutput {
  MotorCommand motor[kMotorCount];
  bool emergency_stop = false;
};

}  // namespace wbr::control

#endif  // WBR_CONTROL_CORE_CONTROLLER_IO_H_
