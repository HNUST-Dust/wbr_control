#pragma once

// Public algorithm diagnostics interface.

#include <cstdint>

namespace alg {

enum class DebugMode : uint8_t {
    Chassis = 1,
    Gimbal,
    Ins,
    Rc,
    ImuHeat,
    Shoot,
    AimAssist,
};

extern uint8_t GlobalDebugMode;

} // namespace alg

// Optional legacy export (keeps existing call sites simple)
using alg::GlobalDebugMode;
