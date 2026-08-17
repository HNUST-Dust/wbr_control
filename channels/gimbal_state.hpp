/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>

#include <channels/comm/seqlock_value.hpp>

namespace channels {

struct GimbalState {
	uint32_t sequence;
	float yaw_angle;
	float yaw_velocity;
	float pitch_angle;
	float pitch_velocity;
};

extern SeqlockValue<GimbalState> latest_gimbal_state;

}  // namespace channels
