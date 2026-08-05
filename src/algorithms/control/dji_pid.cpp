/**
 * @file dji_pid.cpp
 * @brief Implementation for `alg::DjiPid`.
 */

#include <algorithms/control/dji_pid.h>

#include <algorithm>

namespace alg {

namespace {
inline void clamp_abs(fp32& value, fp32 abs_max) noexcept
{
    if (abs_max <= 0.0f) {
        return;
    }
    value = std::clamp(value, -abs_max, abs_max);
}
} // namespace

void DjiPid::configure(const DjiPidConfig& cfg)
{
    cfg_ = cfg;

    state_.mode = cfg.mode;
    state_.kp = cfg.kp;
    state_.ki = cfg.ki;
    state_.kd = cfg.kd;
    state_.max_out = cfg.max_out;
    state_.max_iout = cfg.max_iout;

    reset();
}

void DjiPid::reset()
{
    state_.set = 0.0f;
    state_.fdb = 0.0f;

    state_.out = 0.0f;
    state_.p_out = 0.0f;
    state_.i_out = 0.0f;
    state_.d_out = 0.0f;

    state_.d_buf[0] = state_.d_buf[1] = state_.d_buf[2] = 0.0f;
    state_.error[0] = state_.error[1] = state_.error[2] = 0.0f;
}

fp32 DjiPid::update(fp32 target, fp32 now)
{
    // Align with legacy API: set = target, ref = now
    state_.error[2] = state_.error[1];
    state_.error[1] = state_.error[0];

    state_.set = target;
    state_.fdb = now;
    state_.error[0] = target - now;

    if (state_.mode == PID_POSITION) {
        state_.p_out = state_.kp * state_.error[0];
        state_.i_out += state_.ki * state_.error[0];

        state_.d_buf[2] = state_.d_buf[1];
        state_.d_buf[1] = state_.d_buf[0];
        state_.d_buf[0] = (state_.error[0] - state_.error[1]);
        state_.d_out = state_.kd * state_.d_buf[0];

        clamp_abs(state_.i_out, state_.max_iout);
        state_.out = state_.p_out + state_.i_out + state_.d_out;
        clamp_abs(state_.out, state_.max_out);
    } else if (state_.mode == PID_DELTA) {
        state_.p_out = state_.kp * (state_.error[0] - state_.error[1]);
        state_.i_out = state_.ki * state_.error[0];

        state_.d_buf[2] = state_.d_buf[1];
        state_.d_buf[1] = state_.d_buf[0];
        state_.d_buf[0] = (state_.error[0] - 2.0f * state_.error[1] + state_.error[2]);
        state_.d_out = state_.kd * state_.d_buf[0];

        state_.out += state_.p_out + state_.i_out + state_.d_out;
        clamp_abs(state_.out, state_.max_out);
    }

    return state_.out;
}

} // namespace alg
