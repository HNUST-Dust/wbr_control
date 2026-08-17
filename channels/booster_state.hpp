/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>

#include <channels/comm/seqlock_value.hpp>

namespace channels {

struct BoosterState {
	uint32_t sequence;
	float bullet_speed;
	uint16_t bullet_count;
};

extern SeqlockValue<BoosterState> latest_booster_state;

}  // namespace channels
