/**
* @file src/modules/chassis/legacy/controller_types.h
 * @ingroup wbr_modules
 * @brief 定义传统底盘控制器的公共状态类型。
 * @details 该组件属于可独立计算的传统底盘控制链路。调用方按固定周期提供一致输入快照；组件保存的积分或滤波状态必须在失能、故障或时间跳变后复位。
 */

// Legacy controller stack; excluded from the firmware build.
#ifndef WBR_CONTROL_CORE_CONTROLLER_TYPES_H_
#define WBR_CONTROL_CORE_CONTROLLER_TYPES_H_

/** @brief 轮腿机器人接触安全状态。 */
enum class WbrContactSafetyState {
  kDualSupport = 0, ///< 左右轮均可靠触地，可使用完整平衡控制权重。
  kSingleSupportFirst = 1, ///< 仅第一条腿可靠触地，限制差动控制量。
  kSingleSupportSecond = 2, ///< 仅第二条腿可靠触地，限制差动控制量。
  kAirborne = 3, ///< 双轮离地，关闭依赖地面反力的控制分量。
  kRecovery = 4, ///< 重新触地后的平滑恢复阶段。
};

/** @brief 第二版轮腿控制器的调试遥测数据。 */
struct WbrControllerV2Telemetry {
  double leg_length[2] = {}; ///< 目标腿长，单位为米。
  double leg_length_rate[2] = {}; ///< 长度或距离，单位为米。
  double leg_angle[2] = {}; ///< 角度，单位为弧度。
  double leg_angle_rate[2] = {}; ///< 角度，单位为弧度。
  double commanded_leg_length = 0.18; ///< 长度或距离，单位为米。
  double commanded_leg_length_rate = 0.0; ///< 长度或距离，单位为米。
  double commanded_leg_angle = 0.0; ///< 角度，单位为弧度。
  double axial_force[2] = {}; ///< 沿腿轴方向的控制力，单位为牛。
  double integral_force[2] = {}; ///< 腿长误差积分产生的轴向力分量，单位为牛。
  double leg_angle_error[2] = {}; ///< 角度，单位为弧度。
  double leg_angle_torque[2] = {}; ///< 角度，单位为弧度。
  double requested_wheel_torque = 0.0; ///< 力矩，单位为牛·米。
  double wheel_attitude_torque_contribution = 0.0; ///< 力矩，单位为牛·米。
  double wheel_translation_torque_contribution = 0.0; ///< 力矩，单位为牛·米。
  double filtered_pitch_rate = 0.0; ///< 低通滤波后的俯仰角速度，单位为弧度每秒。
  double world_leg_angle = 0.0; ///< 角度，单位为弧度。
  double world_leg_angle_reference = 0.0; ///< 角度，单位为弧度。
  double world_leg_angle_error = 0.0; ///< 角度，单位为弧度。
  double world_leg_angle_torque = 0.0; ///< 角度，单位为弧度。
  double leg_split_hold_error = 0.0; ///< 左右腿差值保持目标与实测值之间的误差。
  double leg_split_hold_torque = 0.0; ///< 力矩，单位为牛·米。
  double requested_leg_angle_torque = 0.0; ///< 角度，单位为弧度。
  double applied_wheel_torque = 0.0; ///< 力矩，单位为牛·米。
  double applied_leg_angle_torque = 0.0; ///< 角度，单位为弧度。
  double lqr_scale = 0.0; ///< 控制器使用的比例或缩放系数。
  double yaw_rate = 0.0; ///< 航向角速度，单位为弧度每秒。
  double yaw_rate_error = 0.0; ///< 目标与实测航向角速度之差，单位为弧度每秒。
  double yaw_authority_scale = 1.0; ///< 控制器使用的比例或缩放系数。
  double yaw_attitude_authority = 1.0; ///< 航向姿态反馈权重，范围为 0 至 1。
  double yaw_contact_authority = 1.0; ///< 航向控制的触地权重，范围为 0 至 1。
  double yaw_split_authority = 1.0; ///< 航向差动控制权重，范围为 0 至 1。
  double yaw_split_residual_authority = 1.0; ///< 航向残差反馈权重，范围为 0 至 1。
  double yaw_split_absolute_authority = 1.0; ///< 航向绝对差值反馈权重，范围为 0 至 1。
  double spin_mode_blend = 0.0; ///< 普通航向与小陀螺模式之间的平滑混合系数。
  double reserved_yaw_torque_per_wheel = 0.0; ///< 力矩，单位为牛·米。
  double balance_torque_authority = 1.0; ///< 力矩，单位为牛·米。
  double coordinated_yaw_rate = 0.0; ///< 经过接触和安全约束后的航向角速度，单位为弧度每秒。
  double wheel_normal_force[2] = {}; ///< 左右轮估计地面法向力，单位为牛。
  double applied_yaw_torque = 0.0; ///< 力矩，单位为牛·米。
  double commanded_yaw_rate = 0.0; ///< 上层期望航向角速度，单位为弧度每秒。
  double differential_leg_angle_error = 0.0; ///< 角度，单位为弧度。
  double differential_leg_angle_rate = 0.0; ///< 角度，单位为弧度。
  double differential_leg_angle_torque = 0.0; ///< 角度，单位为弧度。
  double state_error[6] = {}; ///< 状态机用于切换判据的当前误差。
  bool wheel_grounded[2] = {}; ///< 左右车轮的触地判定。
  bool balance_active = false; ///< 平衡控制器当前正在输出控制量的标志。
  WbrContactSafetyState contact_safety_state =
      WbrContactSafetyState::kAirborne; ///< 当前接触安全状态，默认按双腿离地处理。
  double contact_authority_scale = 0.0; ///< 控制器使用的比例或缩放系数。
  double estimated_x_speed = 0.0; ///< 线速度，单位为米每秒。
  double wheel_odometry_x_speed = 0.0; ///< 线速度，单位为米每秒。
  double wheel_odometry_confidence = 0.0; ///< 车轮里程计可信权重，范围为 0 至 1。
};

#endif  // WBR_CONTROL_CORE_CONTROLLER_TYPES_H_
