/**
* @file src/modules/chassis/legacy/wheel_allocator.h
 * @ingroup wbr_modules
 * @brief 将底盘控制量分配为左右轮执行器指令。
 * @details 该组件属于可独立计算的传统底盘控制链路。调用方按固定周期提供一致输入快照；组件保存的积分或滤波状态必须在失能、故障或时间跳变后复位。
 */

// Legacy controller stack; excluded from the firmware build.
#ifndef WBR_CONTROL_CORE_WHEEL_ALLOCATOR_H_
#define WBR_CONTROL_CORE_WHEEL_ALLOCATOR_H_

#include "controller_types.h"

namespace wbr::v2 {

/** @brief 车体目标力矩到车轮力矩分配的输入。 */
struct WheelAllocationInput {
  double balance_torque = 0.0; ///< 力矩，单位为牛·米。
  double yaw_torque = 0.0; ///< 力矩，单位为牛·米。
  bool grounded[2] = {}; ///< 当前腿部与地面可靠接触的标志。
  WbrContactSafetyState contact_state = WbrContactSafetyState::kAirborne; ///< 接触安全状态机的当前状态。
};

/** @brief 左右车轮力矩分配结果。 */
struct WheelAllocationOutput {
  double actuator_torque[2] = {}; ///< 力矩，单位为牛·米。
  double applied_yaw_torque = 0.0; ///< 力矩，单位为牛·米。
};

/**
 * @brief 将总轮力矩与差动力矩分配到左右车轮。
 * @param[in] input 本周期使用的只读输入快照。
 * @return 满足单轮和总力矩限制的左右轮力矩分配结果。
 */
WheelAllocationOutput AllocateWheelTorque(const WheelAllocationInput& input);

}  // namespace wbr::v2

#endif  // WBR_CONTROL_CORE_WHEEL_ALLOCATOR_H_
