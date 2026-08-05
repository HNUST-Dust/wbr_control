/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include <channels/comm/seqlock_value.hpp>

namespace channels {

constexpr size_t kOscilloscopeMaxChannels = 27U;

struct OscilloscopeSample {
	uint32_t sequence;
	uint32_t uptime_ms;
	uint8_t channel_count;
	float value[kOscilloscopeMaxChannels];
};

extern SeqlockValue<OscilloscopeSample> latest_oscilloscope_sample;

}  // namespace channels
