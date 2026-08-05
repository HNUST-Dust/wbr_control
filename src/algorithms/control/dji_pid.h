/**
 * @file dji_pid.h
 * @brief Lightweight discrete PID (DJI-style), implemented in modern C++.
 *
 * Design:
 * - Keeps the original lightweight discrete PID behavior (no explicit dt).
 * - Provides a `configure/reset/update` interface similar to `alg::Pid`.
 */

#pragma once

// Public reusable discrete PID interface.

#include <cstdint>

namespace alg {

using fp32 = float;

enum class PidMode : std::uint8_t {
    Position = 0,
    Delta = 1,
};

// Backward-compatible names (so existing call sites can keep using them)
static constexpr PidMode PID_POSITION = PidMode::Position;
static constexpr PidMode PID_DELTA = PidMode::Delta;

struct DjiPidConfig {
    fp32 kp = 0.0f;
    fp32 ki = 0.0f;
    fp32 kd = 0.0f;

    fp32 max_out = 0.0f;
    fp32 max_iout = 0.0f;

    PidMode mode = PID_POSITION;
};

class DjiPid {
public:
    void configure(const DjiPidConfig& cfg);
    void reset();

    // Align with `alg::Pid`: error = target - now
    fp32 update(fp32 target, fp32 now);

    fp32 out() const noexcept { return state_.out; }

private:
    struct State {
        PidMode mode = PID_POSITION;

        fp32 kp = 0.0f;
        fp32 ki = 0.0f;
        fp32 kd = 0.0f;

        fp32 max_out = 0.0f;
        fp32 max_iout = 0.0f;

        fp32 set = 0.0f;
        fp32 fdb = 0.0f;

        fp32 out = 0.0f;
        fp32 p_out = 0.0f;
        fp32 i_out = 0.0f;
        fp32 d_out = 0.0f;

        fp32 d_buf[3] = {0.0f, 0.0f, 0.0f};
        fp32 error[3] = {0.0f, 0.0f, 0.0f};
    };

    DjiPidConfig cfg_{};
    State state_{};
};

} // namespace alg
