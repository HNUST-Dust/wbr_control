/**
 * @file alg_fast_math.h
 * @brief 快速数学近似（用于对性能敏感的场景）
 *
 * 设计思路：
 * =========
 * - 对部分常用数学操作提供更快的近似/迭代实现（例如 sqrt）。
 * - 适用于资源受限 MCU 的实时路径；精度要求极高的场景应优先使用标准库。
 *
 * 注意事项：
 * =========
 * - “fast” 通常意味着精度/收敛性/边界条件需要调用者自行评估。
 */

#pragma once

// Public fast-math interface.

namespace alg {

// Fast sqrt (Newton iteration)
float Sqrt(float x);

} // namespace alg

// Legacy export
using alg::Sqrt;
