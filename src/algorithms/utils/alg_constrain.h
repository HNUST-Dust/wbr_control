/**
 * @file alg_constrain.h
 * @brief 约束/限幅/死区与环形限幅等通用工具
 *
 * 设计思路：
 * =========
 * - 提供控制与估计中常用的基础“数值约束”操作：限幅、死区、符号函数、角度环形约束等。
 * - 实现为纯函数（无内部状态），便于在不同任务/模块复用。
 *
 * 注意事项：
 * =========
 * - `rad_format` 是历史宏：将弧度格式化到 $[-\pi,\pi]$，依赖 `common/alg_common.h` 提供的 PI。
 * - `loop_float_constrain()` 用 while 循环做环形收敛：若输入超出范围非常大，会多次循环；通常
 *   角度输入应当已经接近合理范围。
 */

#pragma once

#include <algorithms/common/alg_common.h>

#include <cstdint>

namespace alg {

float abs_limit(float num, float limit);
float sign(float value);
float float_deadband(float value, float minValue, float maxValue);
int16_t int16_deadline(int16_t value, int16_t minValue, int16_t maxValue);
float float_constrain(float value, float minValue, float maxValue);
int16_t int16_constrain(int16_t value, int16_t minValue, int16_t maxValue);
float loop_float_constrain(float input, float minValue, float maxValue);
float theta_format(float ang);
int float_rounding(float raw);

} // namespace alg

// Legacy exports
using alg::abs_limit;
using alg::sign;
using alg::float_deadband;
using alg::int16_deadline;
using alg::float_constrain;
using alg::int16_constrain;
using alg::loop_float_constrain;
using alg::theta_format;
using alg::float_rounding;

// 弧度格式化为 -PI ~ PI
#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)
