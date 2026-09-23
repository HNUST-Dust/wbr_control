/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <cstdint>

namespace msg {

template <typename T>
bool IsFresh(const T &value, uint64_t T::*timestamp, uint64_t now_us,
	     uint64_t timeout_us, bool inclusive = true)
{
	const uint64_t sample_us = value.*timestamp;
	if (!value.valid || sample_us > now_us) {
		return false;
	}
	const uint64_t age_us = now_us - sample_us;
	return inclusive ? age_us <= timeout_us : age_us < timeout_us;
}

} // namespace msg
