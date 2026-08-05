#include <algorithms/utils/alg_ramp.h>

namespace alg {

void Ramp::init(float frame_period_s, float max, float min)
{
    frame_period = frame_period_s;
    max_value = max;
    min_value = min;
    input = 0.0f;
    out = 0.0f;
    is_completed = 1;
}

float Ramp::step(float input_per_s)
{
    is_completed = 0;
    input = input_per_s;
    out += input * frame_period;

    if (out > max_value) {
        out = max_value;
    } else if (out < min_value) {
        out = min_value;
    }

    if (out == max_value || out == min_value) {
        is_completed = 1;
    }

    return out;
}

void ramp_init(Ramp* ramp_source_type, float frame_period, float max, float min)
{
    if (!ramp_source_type) {
        return;
    }
    ramp_source_type->init(frame_period, max, min);
}

float ramp_calc(Ramp* ramp_source_type, float input)
{
    if (!ramp_source_type) {
        return 0.0f;
    }
    return ramp_source_type->step(input);
}

} // namespace alg
