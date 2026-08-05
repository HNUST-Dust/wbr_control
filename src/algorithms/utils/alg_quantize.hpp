#pragma once

// Public fixed-width quantization helpers.

#include <cstdint>

namespace alg {

// Quantize float in [x_min, x_max] into unsigned integer with given bit-width.
// Commonly used in motor protocols (e.g., MIT). Out-of-range values are clamped.
inline uint32_t float_to_uint(float x, float x_min, float x_max, uint8_t bits) {
    if (bits == 0) {
        return 0u;
    }

    const uint32_t max_int = (bits >= 32) ? 0xFFFFFFFFu : ((1u << bits) - 1u);
    const float span = x_max - x_min;
    if (span == 0.0f) {
        return 0u;
    }

    const float offset = x - x_min;
    const float scaled = (offset * static_cast<float>(max_int)) / span;

    if (scaled <= 0.0f) {
        return 0u;
    }
    if (scaled >= static_cast<float>(max_int)) {
        return max_int;
    }
    return static_cast<uint32_t>(scaled);
}

// De-quantize unsigned integer into float in [x_min, x_max] with given bit-width.
inline float uint_to_float(uint32_t x_int, float x_min, float x_max, uint8_t bits) {
    if (bits == 0) {
        return x_min;
    }

    const uint32_t max_int = (bits >= 32) ? 0xFFFFFFFFu : ((1u << bits) - 1u);
    if (max_int == 0u) {
        return x_min;
    }

    const float span = x_max - x_min;
    return (static_cast<float>(x_int) * span) / static_cast<float>(max_int) + x_min;
}

} // namespace alg
