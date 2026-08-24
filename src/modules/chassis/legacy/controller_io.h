/**
* @file src/modules/chassis/legacy/controller_io.h
 * @ingroup wbr_modules
 * @brief 定义传统底盘控制器的输入输出数据结构。
 * @details 该组件属于可独立计算的传统底盘控制链路。调用方按固定周期提供一致输入快照；组件保存的积分或滤波状态必须在失能、故障或时间跳变后复位。
 */

// Legacy controller stack; excluded from the firmware build.
#ifndef WBR_CONTROL_CORE_CONTROLLER_IO_H_
#define WBR_CONTROL_CORE_CONTROLLER_IO_H_

#include <cstdint>

namespace wbr::control {

/** @brief 传统底盘控制器使用的电机编号。 */
enum class MotorId : std::uint8_t {
  kLeftJointB, ///< 左腿 B 关节电机发送槽位。
  kLeftJointD, ///< 左腿 D 关节电机发送槽位。
  kRightJointB, ///< 右腿 B 关节电机发送槽位。
  kRightJointD, ///< 右腿 D 关节电机发送槽位。
  kLeftWheel, ///< 左侧车轮电机发送槽位。
  kRightWheel, ///< 右侧车轮电机发送槽位。
  kCount, ///< 枚举项数量，仅用于数组边界。
};

inline constexpr int kMotorCount = static_cast<int>(MotorId::kCount); ///< 底盘控制器管理的电机数量。

/** @brief 控制器使用的 IMU 姿态与角速度。 */
struct ImuState {
  double roll = 0.0; ///< 机体横滚角，单位为弧度。
  double pitch = 0.0; ///< 机体俯仰角，单位为弧度。
  double angular_velocity[3] = {}; ///< 角速度，单位为弧度每秒。
  double acceleration[3] = {}; ///< 当前线加速度，单位为米每二次方秒。
  std::uint64_t timestamp_us = 0; ///< 采样或接收时间戳，单位为微秒，来自单调时钟。
  bool valid = false; ///< 数据有效标志；为 `false` 时其余字段不得用于控制。
  bool attitude_valid = false; ///< 姿态解算有效标志；无效时不得闭环控制。
};

/** @brief 控制器使用的电机位置、速度和力矩反馈。 */
struct MotorFeedback {
  double position = 0.0; ///< 位置或转角状态；单位由所属协议或控制接口规定。
  double velocity = 0.0; ///< 线速度或角速度状态；单位由所属数据结构规定。
  double torque = 0.0; ///< 电机或关节力矩，单位为牛·米。
  std::uint64_t timestamp_us = 0; ///< 采样或接收时间戳，单位为微秒，来自单调时钟。
  bool valid = false; ///< 数据有效标志；为 `false` 时其余字段不得用于控制。
};

/** @brief 腿部与地面接触的观测结果。 */
struct ContactObservation {
  double confidence[2] = {}; ///< 左右腿接触置信度，范围为 0 至 1。
};

/** @brief 上层下发给底盘控制器的运动目标。 */
struct ControlCommand {
  double linear_velocity = 0.0; ///< 线速度，单位为米每秒。
  double yaw_rate = 0.0; ///< 航向角速度，单位为弧度每秒。
  double leg_length = 0.18; ///< 目标腿长，单位为米。
  double leg_angle = 0.0; ///< 角度，单位为弧度。
  std::uint64_t timestamp_us = 0; ///< 采样或接收时间戳，单位为微秒，来自单调时钟。
  bool valid = false; ///< 数据有效标志；为 `false` 时其余字段不得用于控制。
  bool enabled = false; ///< 状态有效或功能启用标志。
};

/** @brief 传统底盘控制器的一次完整输入。 */
struct ControlInput {
  double dt = 0.001; ///< 控制周期，单位为秒且必须大于零。
  ImuState imu; ///< 本周期冻结的惯性测量和姿态状态。
  MotorFeedback motor[kMotorCount]; ///< 按 `MotorId` 索引的六个电机状态或命令。
  ContactObservation contact; ///< 本周期冻结的接触观测结果。
  ControlCommand command; ///< 本周期冻结的上层运动目标。
};

/** @brief 单个电机的目标控制量。 */
struct MotorCommand {
  double torque = 0.0; ///< 电机或关节力矩，单位为牛·米。
  bool enabled = false; ///< 状态有效或功能启用标志。
};

/** @brief 传统底盘控制器输出的全部电机指令。 */
struct ControlOutput {
  MotorCommand motor[kMotorCount]; ///< 按 `MotorId` 索引的六个电机状态或命令。
  bool emergency_stop = false; ///< 紧急停机标志；置位后应立即禁止执行器输出。
};

}  // namespace wbr::control

#endif  // WBR_CONTROL_CORE_CONTROLLER_IO_H_
