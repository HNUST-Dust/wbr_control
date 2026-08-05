/**
 * @file controller_pid.h
 * @brief Wrapper for legacy `PID_t` (from controller.*) as a modern C++ class.
 *
 * Notes:
 * - The underlying implementation uses DWT to measure dt each update.
 * - This wrapper aims to keep behavior unchanged while providing a modern API.
 */

#pragma once

#include <cstdint>

#include <algorithms/utils/alg_ols.h>

namespace alg {

enum class Improve : std::uint8_t {
    None = 0x00,
    IntegralLimit = 0x01,
    DerivativeOnMeasurement = 0x02,
    TrapezoidIntegral = 0x04,
    ProportionalOnMeasurement = 0x08,
    OutputFilter = 0x10,
    ChangingIntegrationRate = 0x20,
    DerivativeFilter = 0x40,
    ErrorHandle = 0x80,
};

struct ControllerPidConfig {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    float max_out = 0.0f;
    float integral_limit = 0.0f;
    float deadband = 0.0f;

    float i_variable_speed_A = 0.0f;
    float i_variable_speed_B = 0.0f;

    float output_lpf_rc = 0.0f;
    float derivative_lpf_rc = 0.0f;

    std::uint16_t ols_order = 0;
    Improve improve = Improve::None;
};

class ControllerPid {
public:
    void configure(const ControllerPidConfig& cfg);
    void reset();

    // Align with `alg::Pid`: error = target - now
    float update(float target, float now);

    float out() const noexcept { return pid_.output; }

private:
    struct State {
        float ref = 0.0f;
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;

        float measure = 0.0f;
        float last_measure = 0.0f;
        float err = 0.0f;
        float last_err = 0.0f;
        float last_i_term = 0.0f;

        float p_out = 0.0f;
        float i_out = 0.0f;
        float d_out = 0.0f;
        float i_term = 0.0f;

        float output = 0.0f;
        float last_output = 0.0f;
        float last_d_out = 0.0f;

        float max_out = 0.0f;
        float integral_limit = 0.0f;
        float deadband = 0.0f;
        float coef_a = 0.0f;
        float coef_b = 0.0f;
        float output_lpf_rc = 0.0f;
        float derivative_lpf_rc = 0.0f;

        std::uint16_t ols_order = 0;
        OrdinaryLeastSquares ols{};

        std::uint32_t dwt_cnt = 0;
        float dt = 0.0f;

        std::uint8_t improve_mask = 0;

        std::uint64_t error_count = 0;
        bool motor_blocked = false;
    };

    static bool has_flag(std::uint8_t mask, Improve flag) noexcept;
    static void output_limit(State& pid) noexcept;
    static void proportion_limit(State& pid) noexcept;
    static void trapezoid_integral(State& pid) noexcept;
    static void changing_integration_rate(State& pid) noexcept;
    static void integral_limit(State& pid) noexcept;
    static void derivative_on_measurement(State& pid) noexcept;
    static void derivative_filter(State& pid) noexcept;
    static void output_filter(State& pid) noexcept;
    static void pid_error_handle(State& pid) noexcept;

    void reset_runtime_();

    ControllerPidConfig cfg_{};
    State pid_{};
};

} // namespace alg
