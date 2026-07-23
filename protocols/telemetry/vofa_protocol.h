/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace protocols {

constexpr uint8_t kVofaJustFloatTail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};

constexpr size_t VofaJustFloatFrameSize(size_t channel_count)
{
	return (channel_count * sizeof(float)) + sizeof(kVofaJustFloatTail);
}

int EncodeVofaJustFloat(const float *channels,
		    size_t channel_count,
		    uint8_t *out,
		    size_t out_capacity,
		    size_t *out_size);

}  // namespace protocols
