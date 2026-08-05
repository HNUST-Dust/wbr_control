/**
 * @file alg_ramp.h
 * @brief 斜坡函数（对输入变化率进行限制/缓启动）
 *
 * 设计思路：
 * =========
 * - 在每个周期按固定速率积分输入，形成平滑输出，常用于速度/电流指令的软启动与限加速度。
 * - 既提供 `alg::Ramp` 方法接口，也保留历史 wrapper（`ramp_init/ramp_calc`）。
 *
 * 注意事项：
 * =========
 * - `frame_period` 为周期秒数；`input` 的单位应为“每秒变化量”（per second）。
 * - 到达上下限会置 `is_completed=1`。
 */

#pragma once

// Public rate-limiter interface.

#include <cstdint>

namespace alg {

struct Ramp {
    float input = 0.0f;
    float out = 0.0f;
    float min_value = 0.0f;
    float max_value = 0.0f;
    float frame_period = 0.0f;
    uint8_t is_completed = 1;

    void init(float frame_period_s, float max, float min);
    float step(float input_per_s);
};

void ramp_init(Ramp* ramp, float frame_period, float max, float min);
float ramp_calc(Ramp* ramp, float input);

} // namespace alg

// Legacy aliases/exports
using ramp_function_source_t = alg::Ramp;
using alg::ramp_init;
using alg::ramp_calc;
