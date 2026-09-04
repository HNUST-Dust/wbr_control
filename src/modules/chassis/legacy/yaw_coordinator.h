/**
* @file src/modules/chassis/legacy/yaw_coordinator.h
 * @ingroup wbr_modules
 * @brief 协调底盘航向角与云台航向控制。
 * @details 该组件属于可独立计算的传统底盘控制链路。调用方按固定周期提供一致输入快照；组件保存的积分或滤波状态必须在失能、故障或时间跳变后复位。
 */

// Legacy controller stack; excluded from the firmware build.
#ifndef WBR_CONTROL_CORE_YAW_COORDINATOR_H_
#define WBR_CONTROL_CORE_YAW_COORDINATOR_H_

namespace wbr::v2 {

/** @brief 航向协调器使用的姿态、速度和目标。 */
struct YawCoordinatorInput {
  bool reference_enabled = false; ///< 参考轨迹参与闭环控制的使能标志。
  bool time_reset = false; ///< 时间基准或累计状态需要重新初始化的标志。
  double control_dt = 0.001; ///< 控制器更新周期，单位为秒。
  double commanded_yaw_rate = 0.0; ///< 上层期望航向角速度，单位为弧度每秒。
  double measured_yaw_rate = 0.0; ///< 传感器测得的航向角速度，单位为弧度每秒。
  double split_angle = 0.0; ///< 角度，单位为弧度。
  double split_rate = 0.0; ///< 左右腿状态差值的变化率。
  double roll = 0.0; ///< 机体横滚角，单位为弧度。
  double roll_rate = 0.0; ///< 横滚角速度，单位为弧度每秒。
  double pitch = 0.0; ///< 机体俯仰角，单位为弧度。
  double normal_force[2] = {}; ///< 估计的地面法向支撑力，单位为牛。
};

/** @brief 航向协调器给出的旋转控制量。 */
struct YawCoordinatorOutput {
  double coordinated_yaw_rate = 0.0; ///< 经过接触和安全约束后的航向角速度，单位为弧度每秒。
  double split_residual = 0.0; ///< 左右控制量差值中未被主控制项解释的残差。
  double split_rate_residual = 0.0; ///< 左右状态变化率差值的残差。
  double predicted_split_error = 0.0; ///< 预测的左右腿状态差值误差。
  double predicted_roll = 0.0; ///< 状态模型预测的机体横滚角，单位为弧度。
  double predicted_normal_force[2] = {}; ///< 根据动力学预测的地面法向力，单位为牛。
  double split_residual_authority = 1.0; ///< 左右残差反馈权重，范围为 0 至 1。
  double split_absolute_authority = 1.0; ///< 左右绝对差值反馈权重，范围为 0 至 1。
  double split_authority = 1.0; ///< 左右差动控制量允许输出的权重，范围为 0 至 1。
  double attitude_authority = 1.0; ///< 姿态反馈允许输出的权重，范围为 0 至 1。
  double contact_authority = 1.0; ///< 依据触地状态计算的控制权重，范围为 0 至 1。
  double authority = 1.0; ///< 控制分量允许输出的权重，范围为 0 至 1。
};

/** @brief 协调底盘与云台航向运动。 */
class YawCoordinator {
 public:
  /**
   * @brief 清空内部状态并恢复到初始条件。
   */
  void Reset();
  /**
   * @brief 使用当前采样更新内部状态并返回本周期结果。
   * @param[in] input 本周期使用的只读输入快照。
   * @return 偏航目标、车轮力矩预算及腿部差动力矩协调结果。
   */
  YawCoordinatorOutput Update(const YawCoordinatorInput& input);

 private:
  bool initialized_ = false;
  double coordinated_rate_ = 0.0;
  double split_reference_ = 0.0;
  double filtered_force_[2] = {};
  double filtered_force_rate_[2] = {};
};

}  // namespace wbr::v2

#endif  // WBR_CONTROL_CORE_YAW_COORDINATOR_H_
