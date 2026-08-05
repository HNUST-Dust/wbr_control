/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <protocols/telemetry/vofa_protocol.h>

#include <cerrno>
#include <cstring>

namespace protocols {

int EncodeVofaJustFloat(const float *channels,
		    size_t channel_count,
		    uint8_t *out,
		    size_t out_capacity,
		    size_t *out_size)
{
	if ((channels == nullptr) || (out == nullptr) || (out_size == nullptr)) {
		return -EINVAL;
	}

	const size_t required_size = VofaJustFloatFrameSize(channel_count);
	if (out_capacity < required_size) {
		return -ENOSPC;
	}

	size_t offset = 0U;
	for (size_t i = 0U; i < channel_count; ++i) {
		static_assert(sizeof(float) == 4U);
		std::memcpy(&out[offset], &channels[i], sizeof(float));
		offset += sizeof(float);
	}

	std::memcpy(&out[offset], kVofaJustFloatTail, sizeof(kVofaJustFloatTail));
	offset += sizeof(kVofaJustFloatTail);
	*out_size = offset;
	return 0;
}

}  // namespace protocols
