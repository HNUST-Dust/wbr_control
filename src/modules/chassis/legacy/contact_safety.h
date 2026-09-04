/**
* @file src/modules/chassis/legacy/contact_safety.h
 * @ingroup wbr_modules
 * @brief 实现轮腿接触状态相关的安全约束。
 * @details 该组件属于可独立计算的传统底盘控制链路。调用方按固定周期提供一致输入快照；组件保存的积分或滤波状态必须在失能、故障或时间跳变后复位。
 */

// Legacy controller stack; excluded from the firmware build.
#ifndef WBR_CONTROL_CORE_CONTACT_SAFETY_H_
#define WBR_CONTROL_CORE_CONTACT_SAFETY_H_

#include "controller_types.h"

namespace wbr::v2 {

/** @brief 接触安全状态机给出的控制约束。 */
struct ContactSafetyOutput {
  WbrContactSafetyState state = WbrContactSafetyState::kAirborne; ///< 协议、总线或控制状态枚举值。
  bool effective_grounded[2] = {}; ///< 经过去抖和安全状态机处理后的触地判定。
  bool single_support = false; ///< 机器人当前仅有一条腿可靠触地的标志。
  bool airborne = true; ///< 机器人当前处于双腿离地状态的标志。
  bool state_changed = false; ///< 本周期内安全状态发生切换的标志。
  double authority = 0.0; ///< 控制分量允许输出的权重，范围为 0 至 1。
  double state_elapsed = 0.0; ///< 保持当前安全状态的累计时间，单位为秒。
};

/** @brief 根据触地状态限制控制器输出的安全状态机。 */
class ContactSafetyMachine {
 public:
  /**
   * @brief 清空内部状态并恢复到初始条件。
   */
  void Reset();
  /**
   * @brief 使用当前采样更新内部状态并返回本周期结果。
   * @param[in] grounded 左右腿触地状态数组。
   * @param roll 机体横滚角，单位为弧度。
   * @param pitch 机体俯仰角，单位为弧度。
   * @param control_dt 本次状态机更新间隔，单位为秒且必须为正。
   * @return 当前接地判定、恢复进度和安全缩放系数。
   */
  ContactSafetyOutput Update(const bool grounded[2], double roll,
                             double pitch, double control_dt);

 private:
  double contact_grace_[2] = {};
  WbrContactSafetyState state_ = WbrContactSafetyState::kAirborne;
  WbrContactSafetyState candidate_ = WbrContactSafetyState::kAirborne;
  double state_elapsed_ = 0.0;
  double candidate_elapsed_ = 0.0;
};

}  // namespace wbr::v2

#endif  // WBR_CONTROL_CORE_CONTACT_SAFETY_H_
