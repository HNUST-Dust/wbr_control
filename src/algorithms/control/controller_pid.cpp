/**
 * @file controller_pid.cpp
 * @brief Implementation for `alg::ControllerPid`.
 */

#include <algorithms/control/controller_pid.h>

#include <algorithm>
#include <cmath>

#include <zephyr/kernel.h>

namespace alg {

namespace {
inline float get_delta_t_s(std::uint32_t &last_cycle) noexcept
{
    const std::uint32_t now = k_cycle_get_32();
    float dt = 0.0f;

    if (last_cycle != 0U) {
        const std::uint32_t delta = now - last_cycle;
        const std::uint32_t hz = sys_clock_hw_cycles_per_sec();
        if (hz > 0U) {
            dt = static_cast<float>(delta) / static_cast<float>(hz);
        }
    }

    last_cycle = now;
    return dt;
}
} // namespace

bool ControllerPid::has_flag(std::uint8_t mask, Improve flag) noexcept
{
    return (mask & static_cast<std::uint8_t>(flag)) != 0U;
}

void ControllerPid::output_limit(State& pid) noexcept
{
    if (pid.output > pid.max_out) {
        pid.output = pid.max_out;
    }
    if (pid.output < -pid.max_out) {
        pid.output = -pid.max_out;
    }
}

void ControllerPid::proportion_limit(State& pid) noexcept
{
    if (pid.p_out > pid.max_out) {
        pid.p_out = pid.max_out;
    }
    if (pid.p_out < -pid.max_out) {
        pid.p_out = -pid.max_out;
    }
}

void ControllerPid::trapezoid_integral(State& pid) noexcept
{
    pid.i_term = pid.ki * ((pid.err + pid.last_err) * 0.5f) * pid.dt;
}

void ControllerPid::changing_integration_rate(State& pid) noexcept
{
    if (pid.err * pid.i_out > 0.0f) {
        const float abs_err = std::fabs(pid.err);
        if (abs_err <= pid.coef_b) {
            return;
        }
        if (abs_err <= (pid.coef_a + pid.coef_b)) {
            if (pid.coef_a > 0.0f) {
                pid.i_term *= (pid.coef_a - abs_err + pid.coef_b) / pid.coef_a;
            }
        } else {
            pid.i_term = 0.0f;
        }
    }
}

void ControllerPid::integral_limit(State& pid) noexcept
{
    const float temp_iout = pid.i_out + pid.i_term;
    const float temp_output = pid.p_out + pid.i_out + pid.d_out;

    if (std::fabs(temp_output) > pid.max_out) {
        if (pid.err * pid.i_out > 0.0f) {
            pid.i_term = 0.0f;
        }
    }

    if (pid.integral_limit > 0.0f) {
        if (temp_iout > pid.integral_limit) {
            pid.i_term = 0.0f;
            pid.i_out = pid.integral_limit;
        }
        if (temp_iout < -pid.integral_limit) {
            pid.i_term = 0.0f;
            pid.i_out = -pid.integral_limit;
        }
    }
}

void ControllerPid::derivative_on_measurement(State& pid) noexcept
{
    if (pid.dt <= 0.0f) {
        return;
    }

    if (pid.ols_order > 2) {
        pid.d_out = pid.kd * OLS_Derivative(&pid.ols, pid.dt, -pid.measure);
    } else {
        pid.d_out = pid.kd * (pid.last_measure - pid.measure) / pid.dt;
    }
}

void ControllerPid::derivative_filter(State& pid) noexcept
{
    const float denom = pid.derivative_lpf_rc + pid.dt;
    if (denom <= 0.0f) {
        return;
    }
    pid.d_out = pid.d_out * pid.dt / denom + pid.last_d_out * pid.derivative_lpf_rc / denom;
}

void ControllerPid::output_filter(State& pid) noexcept
{
    const float denom = pid.output_lpf_rc + pid.dt;
    if (denom <= 0.0f) {
        return;
    }
    pid.output = pid.output * pid.dt / denom + pid.last_output * pid.output_lpf_rc / denom;
}

void ControllerPid::pid_error_handle(State& pid) noexcept
{
    if (pid.output < pid.max_out * 0.001f || std::fabs(pid.ref) < 0.0001f) {
        return;
    }

    if ((std::fabs(pid.ref - pid.measure) / std::fabs(pid.ref)) > 0.95f) {
        pid.error_count++;
    } else {
        pid.error_count = 0;
    }

    if (pid.error_count > 500) {
        pid.motor_blocked = true;
    }
}

void ControllerPid::configure(const ControllerPidConfig& cfg)
{
    cfg_ = cfg;

    pid_.deadband = cfg.deadband;
    pid_.integral_limit = cfg.integral_limit;
    pid_.max_out = cfg.max_out;

    pid_.kp = cfg.kp;
    pid_.ki = cfg.ki;
    pid_.kd = cfg.kd;

    pid_.coef_a = cfg.i_variable_speed_A;
    pid_.coef_b = cfg.i_variable_speed_B;

    pid_.output_lpf_rc = cfg.output_lpf_rc;
    pid_.derivative_lpf_rc = cfg.derivative_lpf_rc;

    pid_.ols_order = cfg.ols_order;
    pid_.improve_mask = static_cast<std::uint8_t>(cfg.improve);

    reset_runtime_();
}

void ControllerPid::reset()
{
    // Keep configuration, clear runtime states.
    reset_runtime_();
}

float ControllerPid::update(float target, float now)
{
    if (has_flag(pid_.improve_mask, Improve::ErrorHandle)) {
        pid_error_handle(pid_);
    }

    pid_.dt = get_delta_t_s(pid_.dwt_cnt);
    if (pid_.dt <= 0.0f) {
        return pid_.output;
    }

    pid_.measure = now;
    pid_.ref = target;
    pid_.err = pid_.ref - pid_.measure;

    if (std::fabs(pid_.err) > pid_.deadband) {
        pid_.p_out = pid_.kp * pid_.err;
        pid_.i_term = pid_.ki * pid_.err * pid_.dt;
        if (pid_.ols_order > 2) {
            pid_.d_out = pid_.kd * OLS_Derivative(&pid_.ols, pid_.dt, pid_.err);
        } else {
            pid_.d_out = pid_.kd * (pid_.err - pid_.last_err) / pid_.dt;
        }

        if (has_flag(pid_.improve_mask, Improve::TrapezoidIntegral)) {
            trapezoid_integral(pid_);
        }
        if (has_flag(pid_.improve_mask, Improve::ChangingIntegrationRate)) {
            changing_integration_rate(pid_);
        }
        if (has_flag(pid_.improve_mask, Improve::DerivativeOnMeasurement)) {
            derivative_on_measurement(pid_);
        }
        if (has_flag(pid_.improve_mask, Improve::DerivativeFilter)) {
            derivative_filter(pid_);
        }
        if (has_flag(pid_.improve_mask, Improve::IntegralLimit)) {
            integral_limit(pid_);
        }

        pid_.i_out += pid_.i_term;
        pid_.output = pid_.p_out + pid_.i_out + pid_.d_out;

        if (has_flag(pid_.improve_mask, Improve::OutputFilter)) {
            output_filter(pid_);
        }

        output_limit(pid_);
        proportion_limit(pid_);
    }

    pid_.last_measure = pid_.measure;
    pid_.last_output = pid_.output;
    pid_.last_d_out = pid_.d_out;
    pid_.last_err = pid_.err;
    pid_.last_i_term = pid_.i_term;

    return pid_.output;
}

void ControllerPid::reset_runtime_()
{
    pid_.ref = 0.0f;
    pid_.measure = 0.0f;
    pid_.last_measure = 0.0f;
    pid_.err = 0.0f;
    pid_.last_err = 0.0f;
    pid_.last_i_term = 0.0f;

    pid_.p_out = 0.0f;
    pid_.i_out = 0.0f;
    pid_.d_out = 0.0f;
    pid_.i_term = 0.0f;

    pid_.output = 0.0f;
    pid_.last_output = 0.0f;
    pid_.last_d_out = 0.0f;

    pid_.dwt_cnt = 0;
    pid_.dt = 0.0f;
    OLS_Init(&pid_.ols, cfg_.ols_order);

    pid_.error_count = 0;
    pid_.motor_blocked = false;
}

} // namespace alg
