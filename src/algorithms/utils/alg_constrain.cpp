#include <algorithms/utils/alg_constrain.h>

#include <cmath>

namespace alg {

float abs_limit(float num, float limit)
{
    if (num > limit) {
        return limit;
    }
    if (num < -limit) {
        return -limit;
    }
    return num;
}

float sign(float value)
{
    return (value >= 0.0f) ? 1.0f : -1.0f;
}

float float_deadband(float value, float minValue, float maxValue)
{
    if (value < maxValue && value > minValue) {
        return 0.0f;
    }
    return value;
}

int16_t int16_deadline(int16_t value, int16_t minValue, int16_t maxValue)
{
    if (value < maxValue && value > minValue) {
        return 0;
    }
    return value;
}

float float_constrain(float value, float minValue, float maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

int16_t int16_constrain(int16_t value, int16_t minValue, int16_t maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

float loop_float_constrain(float input, float minValue, float maxValue)
{
    if (maxValue < minValue) {
        return input;
    }

    if (input > maxValue) {
        const float len = maxValue - minValue;
        while (input > maxValue) {
            input -= len;
        }
    } else if (input < minValue) {
        const float len = maxValue - minValue;
        while (input < minValue) {
            input += len;
        }
    }

    return input;
}

float theta_format(float ang)
{
    return loop_float_constrain(ang, -180.0f, 180.0f);
}

int float_rounding(float raw)
{
    const int integer = static_cast<int>(raw);
    const float decimal = raw - static_cast<float>(integer);
    return (decimal > 0.5f) ? (integer + 1) : integer;
}

} // namespace alg
