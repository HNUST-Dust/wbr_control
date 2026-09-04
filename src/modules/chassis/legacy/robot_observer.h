/**
* @file src/modules/chassis/legacy/robot_observer.h
 * @ingroup wbr_modules
 * @brief 实现轮腿机器人状态观测与融合。
 * @details 该组件属于可独立计算的传统底盘控制链路。调用方按固定周期提供一致输入快照；组件保存的积分或滤波状态必须在失能、故障或时间跳变后复位。
 */

// Legacy controller stack; excluded from the firmware build.
#ifndef WBR_CONTROL_CORE_ROBOT_OBSERVER_H_
#define WBR_CONTROL_CORE_ROBOT_OBSERVER_H_

#include <cstdint>

#include "controller_io.h"
#include "ground_balance_controller.h"

namespace wbr::control {

/** @brief 机器人观测模型使用的几何和动力学参数。 */
struct RobotParameters {
  double total_mass = 10.1; ///< 机器人参与模型计算的总质量，单位为千克。
  double gravity = 9.80665; ///< 重力方向或重力加速度向量。
  double wheel_radius = 0.058; ///< 驱动车轮有效半径，单位为米。
  double attitude_correction_time_constant = 0.30; ///< 加速度姿态修正时间常数，单位为秒。
  double velocity_correction_time_constant = 0.08; ///< 线速度，单位为米每秒。
  double acceleration_norm_tolerance = 3.0; ///< 允许加速度模偏离重力加速度的阈值。
  double contact_threshold = 0.5; ///< 控制器使用的可配置阈值。
  double minimum_control_period = 0.0002; ///< 允许的最小控制周期，单位为秒。
  double maximum_control_period = 0.003; ///< 允许的最大控制周期，单位为秒。
  std::uint64_t maximum_sample_age_us = 5000; ///< 时间长度，单位为微秒。
};

/** @brief 机器人状态观测器的一次计算结果。 */
struct ObservationResult {
  v2::GroundBalanceInput input; ///< 控制器保存的最近一次输入快照。
  bool valid = false; ///< 数据有效标志；为 `false` 时其余字段不得用于控制。
};

/** @brief 由 IMU 和电机反馈估计机器人运动状态。 */
class RobotObserver {
 public:
  /**
   * @brief 清空内部状态并恢复到初始条件。
   */
  void Reset();
  /**
   * @brief 使用当前采样更新内部状态并返回本周期结果。
   * @param[in] sample 当前控制或传感器采样。
   * @param[in] parameters 机器人质量、几何尺寸和传动参数。
   * @param now_us 当前单调时钟时间戳，单位为微秒。
   * @return 融合后的机体运动状态、轮速里程计与接地估计。
   */
  ObservationResult Update(const ControlInput& sample,
                           const RobotParameters& parameters,
                           std::uint64_t now_us);

 private:
  bool initialized_ = false;
  double roll_ = 0.0;
  double pitch_ = 0.0;
  double x_ = 0.0;
  double x_speed_ = 0.0;
};

}  // namespace wbr::control

#endif  // WBR_CONTROL_CORE_ROBOT_OBSERVER_H_
