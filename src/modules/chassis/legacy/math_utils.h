/**
* @file src/modules/chassis/legacy/math_utils.h
 * @ingroup wbr_modules
 * @brief 提供底盘控制算法使用的基础数学工具。
 * @details 该组件属于可独立计算的传统底盘控制链路。调用方按固定周期提供一致输入快照；组件保存的积分或滤波状态必须在失能、故障或时间跳变后复位。
 */

// Legacy controller stack; excluded from the firmware build.
#ifndef WBR_CONTROL_CORE_MATH_UTILS_H_
#define WBR_CONTROL_CORE_MATH_UTILS_H_

namespace wbr::v2 {

/**
 * @brief 将数值限制在闭区间内。
 * @param value 待发布或参与计算的输入值。
 * @param lo 允许返回的最小值。
 * @param hi 允许返回的最大值，必须不小于 `lo`。
 * @return 限制在 `[lo, hi]` 内的数值。
 */
double Clamp(double value, double lo, double hi);
/**
 * @brief 按最大步长将当前值逼近目标值。
 * @param value 待发布或参与计算的输入值。
 * @param target 目标位置或角度。
 * @param max_step 单次允许变化的最大绝对值，必须非负。
 * @return 向目标移动且单次变化不超过给定步长的数值。
 */
double MoveTowards(double value, double target, double max_step);
/**
 * @brief 在接近硬限制时平滑降低控制权重。
 * @param magnitude 当前受限物理量的绝对值。
 * @param soft_limit 开始衰减控制权重的软阈值。
 * @param hard_limit 控制权重降为零的硬阈值，必须大于软阈值。
 * @return 范围为 0 至 1 的控制权重。
 */
double FadeAuthority(double magnitude, double soft_limit, double hard_limit);
/**
 * @brief 在离开硬限制时平滑恢复控制权重。
 * @param value 待发布或参与计算的输入值。
 * @param hard_limit 控制权重为零的硬阈值。
 * @param soft_limit 控制权重恢复为一的软阈值，必须大于硬阈值。
 * @return 范围为 0 至 1 的控制权重。
 */
double RiseAuthority(double value, double hard_limit, double soft_limit);

}  // namespace wbr::v2

#endif  // WBR_CONTROL_CORE_MATH_UTILS_H_
