/**
 * @file alg_common.h
 * @brief Algorithm 模块的公共轻量常量与单位换算
 *
 * 设计思路：
 * =========
 * - 为算法层提供最小公共依赖（例如 PI、角速度单位换算）。
 * - 该文件应保持“平台无关、无副作用、可被任意模块包含”。
 * - 仅放置通用常量与纯宏/纯 constexpr（如后续演进），避免引入 HAL/RTOS。
 *
 * 注意事项：
 * =========
 * - 这里的宏以 `#ifndef` 形式定义：允许外部在包含前覆盖，但也意味着应避免在不同翻译单元
 *   给出不一致的定义（会导致行为难以追踪）。
 */

#pragma once

// Public platform-independent algorithm constants.

#include <cstdint>
#include <cstddef>

// Unified math constants
#ifndef PI
#define PI 3.14159265358979323846f
#endif

#ifndef TWO_PI
#define TWO_PI (2.0f * PI)
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (PI / 180.0f)
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0f / PI)
#endif

// RPM conversions
#ifndef RPM_2_ANGLE_PER_SEC
#define RPM_2_ANGLE_PER_SEC 6.0f // ×360°/60sec
#endif

#ifndef RPM_2_RAD_PER_SEC
#define RPM_2_RAD_PER_SEC (TWO_PI / 60.0f) // ×2pi/60sec
#endif

#ifndef RPM_TO_RADPS
#define RPM_TO_RADPS RPM_2_RAD_PER_SEC
#endif
