/**
* @file src/modules/chassis/legacy/math_utils.cc
 * @ingroup wbr_modules
 * @brief 提供底盘控制算法使用的基础数学工具。
 * @details 该组件属于可独立计算的传统底盘控制链路。调用方按固定周期提供一致输入快照；组件保存的积分或滤波状态必须在失能、故障或时间跳变后复位。
 */

// Legacy controller stack; excluded from the firmware build.
#include "math_utils.h"

#include <cmath>

namespace wbr::v2 {

double Clamp(double value, double lo, double hi) {
  return std::fmin(std::fmax(value, lo), hi);
}

double MoveTowards(double value, double target, double max_step) {
  return value + Clamp(target - value, -max_step, max_step);
}

double FadeAuthority(double magnitude, double soft_limit, double hard_limit) {
  return 1.0 - Clamp((magnitude - soft_limit) /
                         (hard_limit - soft_limit),
                     0.0, 1.0);
}

double RiseAuthority(double value, double hard_limit, double soft_limit) {
  return Clamp((value - hard_limit) / (soft_limit - hard_limit), 0.0, 1.0);
}

}  // namespace wbr::v2
