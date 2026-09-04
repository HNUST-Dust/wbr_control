/**
* @file src/modules/chassis/legacy/ground_balance_controller.h
 * @ingroup wbr_modules
 * @brief 实现着地平衡控制器及其状态切换。
 * @details 该组件属于可独立计算的传统底盘控制链路。调用方按固定周期提供一致输入快照；组件保存的积分或滤波状态必须在失能、故障或时间跳变后复位。
 */

// Legacy controller stack; excluded from the firmware build.
#ifndef WBR_CONTROL_CORE_GROUND_BALANCE_CONTROLLER_H_
#define WBR_CONTROL_CORE_GROUND_BALANCE_CONTROLLER_H_

#include "contact_safety.h"
#include "controller_io.h"
#include "controller_types.h"
#include "leg_kinematics.h"
#include "yaw_coordinator.h"

namespace wbr::v2 {

/** @brief 着地平衡控制器所需的状态与目标。 */
struct GroundBalanceInput {
  double control_dt = 0.001; ///< 控制器更新周期，单位为秒。
  bool time_reset = false; ///< 时间基准或累计状态需要重新初始化的标志。
  bool observation_valid = false; ///< 本周期状态观测结果有效标志。
  bool yaw_plant_enabled = false; ///< 航向被控对象模型可用并允许闭环的标志。

  LegKinematics leg[2]; ///< 左右腿状态数组，索引 0 为左侧、1 为右侧。
  bool leg_valid[2] = {}; ///< 左右腿运动学解均有效的标志。
  bool wheel_grounded[2] = {}; ///< 左右车轮的触地判定。
  double wheel_normal_force[2] = {}; ///< 左右轮估计地面法向力，单位为牛。

  double roll = 0.0; ///< 机体横滚角，单位为弧度。
  double pitch = 0.0; ///< 机体俯仰角，单位为弧度。
  double roll_rate = 0.0; ///< 横滚角速度，单位为弧度每秒。
  double pitch_rate = 0.0; ///< 俯仰角速度，单位为弧度每秒。
  double yaw_rate = 0.0; ///< 航向角速度，单位为弧度每秒。
  double x = 0.0; ///< 状态向量或坐标的 X 分量。
  double x_speed = 0.0; ///< 线速度，单位为米每秒。
  double wheel_odometry_x_speed = 0.0; ///< 线速度，单位为米每秒。
  double wheel_odometry_confidence = 0.0; ///< 车轮里程计可信权重，范围为 0 至 1。

  double total_mass = 0.0; ///< 机器人参与模型计算的总质量，单位为千克。
  double gravity_magnitude = 9.81; ///< 当地重力加速度标量，单位为米每二次方秒。
  double target_leg_length = 0.18; ///< 长度或距离，单位为米。
  double target_leg_angle = 0.0; ///< 角度，单位为弧度。
};

/** @brief 着地平衡控制器计算出的轮腿控制量。 */
struct GroundBalanceOutput {
  control::ControlOutput actuator; ///< 按执行器编号排列的控制输出。
  double sanitized_leg_length = 0.18; ///< 长度或距离，单位为米。
  double sanitized_leg_angle = 0.0; ///< 角度，单位为弧度。
};

/** @brief 轮腿机器人着地状态下的平衡控制器。 */
class GroundBalanceController {
 public:
  /**
   * @brief 清空内部状态并恢复到初始条件。
   */
  void Reset();
  /**
   * @brief 用当前腿部位姿初始化平滑目标。
   * @param length 数据长度或腿长；具体单位由接口上下文确定。
   * @param angle 目标或测量角度，单位为弧度。
   */
  void InitializeLegTarget(double length, double angle);
  /**
   * @brief 使用当前采样更新内部状态并返回本周期结果。
   * @param[in] input 本周期使用的只读输入快照。
   * @return 本周期车轮与腿部力矩目标以及接地安全状态。
   */
  GroundBalanceOutput Update(const GroundBalanceInput& input);

  /**
   * @brief 启用或禁用 LQR 状态反馈。
   * @param enabled 为 `true` 时启用该控制功能。
   */
  void SetLqrEnabled(bool enabled) { lqr_enabled_ = enabled; }
  /**
   * @brief 设置 LQR 输出的全局缩放系数。
   * @param scale 控制输出的无量纲缩放系数。
   */
  void SetLqrScale(double scale) { lqr_scale_ = scale; }
  /**
   * @brief 配置仅使用车轮调节俯仰角的降级控制。
   * @param enabled 为 `true` 时启用该控制功能。
   * @param angle_scale_boost 俯仰角反馈的附加缩放系数。
   * @param rate_scale_boost 俯仰角速度反馈的附加缩放系数。
   */
  void SetPitchOnlyWheelControl(bool enabled, double angle_scale_boost,
                                double rate_scale_boost) {
    pitch_only_wheel_control_ = enabled;
    pitch_only_wheel_angle_scale_boost_ = angle_scale_boost;
    pitch_only_wheel_rate_scale_boost_ = rate_scale_boost;
  }
  /**
   * @brief 启用或禁用腿部角度闭环。
   * @param enabled 为 `true` 时启用该控制功能。
   */
  void SetLegAngleControlEnabled(bool enabled) {
    leg_angle_control_enabled_ = enabled;
  }
  /**
   * @brief 配置左右腿独立角度保持。
   * @param enabled 为 `true` 时启用该控制功能。
   */
  void SetIndependentLegAngleHoldEnabled(bool enabled) {
    independent_leg_angle_hold_enabled_ = enabled;
  }
  /**
   * @brief 配置双腿共模姿态的 LQR 控制。
   * @param enabled 为 `true` 时启用该控制功能。
   * @param torque_limit 输出力矩绝对值上限，单位为牛·米。
   */
  void SetCommonLegLqrEnabled(bool enabled, double torque_limit) {
    common_leg_lqr_enabled_ = enabled;
    common_leg_lqr_torque_limit_ = torque_limit;
  }
  /**
   * @brief 配置相对世界坐标系的腿部姿态控制。
   * @param enabled 为 `true` 时启用该控制功能。
   */
  void SetWorldLegPoseControlEnabled(bool enabled) {
    world_leg_pose_control_enabled_ = enabled;
  }
  /**
   * @brief 启用或禁用航向闭环控制。
   * @param enabled 为 `true` 时启用该控制功能。
   */
  void SetYawEnabled(bool enabled) { yaw_enabled_ = enabled; }
  /**
   * @brief 更新期望线速度和航向角速度。
   * @param linear_velocity 期望车体线速度，单位为米每秒。
   * @param yaw_rate 期望航向角速度，单位为弧度每秒。
   */
  void SetVelocityCommand(double linear_velocity, double yaw_rate) {
    target_linear_velocity_ = linear_velocity;
    target_yaw_rate_ = yaw_rate;
  }
  /**
   * @brief 使 LQR 参考状态在下一周期重新对齐。
   */
  void ResetLqrReference() { lqr_initialized_ = false; }
  /**
   * @brief 读取最近一次控制周期的遥测快照。
   * @return 内部遥测对象的常量引用，在下一次更新前有效。
   */
  const WbrControllerV2Telemetry& telemetry() const { return telemetry_; }

 private:
  double leg_speed_[2] = {};
  double leg_angle_speed_[2] = {};
  double filtered_yaw_speed_ = 0.0;
  double filtered_pitch_speed_ = 0.0;
  bool pitch_rate_filter_initialized_ = false;
  double filtered_yaw_acceleration_ = 0.0;
  double previous_yaw_speed_ = 0.0;
  double x_reference_ = 0.0;
  double target_linear_velocity_ = 0.0;
  double target_yaw_rate_ = 0.0;
  double commanded_linear_velocity_ = 0.0;
  YawCoordinator yaw_coordinator_;
  double commanded_yaw_torque_ = 0.0;
  double commanded_differential_leg_angle_torque_ = 0.0;
  double commanded_leg_length_ = 0.18;
  double commanded_leg_angle_ = 0.0;
  double support_factor_[2] = {};
  double leg_length_integral_[2] = {};
  ContactSafetyMachine contact_safety_;
  double contact_leg_length_offset_[2] = {};
  bool command_initialized_ = false;
  bool lqr_initialized_ = false;
  bool lqr_enabled_ = true;
  double lqr_scale_ = 1.0;
  bool pitch_only_wheel_control_ = false;
  double pitch_only_wheel_angle_scale_boost_ = 1.0;
  double pitch_only_wheel_rate_scale_boost_ = 1.0;
  bool leg_angle_control_enabled_ = true;
  bool independent_leg_angle_hold_enabled_ = false;
  bool independent_leg_angle_reference_initialized_ = false;
  double independent_leg_angle_reference_[2] = {};
  bool common_leg_lqr_enabled_ = false;
  double common_leg_lqr_torque_limit_ = 4.0;
  bool world_leg_pose_control_enabled_ = false;
  bool world_leg_angle_reference_initialized_ = false;
  double world_leg_angle_reference_ = 0.0;
  bool leg_split_reference_initialized_ = false;
  double leg_split_reference_ = 0.0;
  bool yaw_enabled_ = true;
  WbrContactSafetyState contact_safety_state_ =
      WbrContactSafetyState::kAirborne;
  WbrControllerV2Telemetry telemetry_{};
};

}  // namespace wbr::v2

#endif  // WBR_CONTROL_CORE_GROUND_BALANCE_CONTROLLER_H_
