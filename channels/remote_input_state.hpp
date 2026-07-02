/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>

#include <channels/comm/seqlock_value.hpp>

namespace channels {

enum RemoteInputSource : uint8_t {
	kRemoteInputUnknown = 0,
	kRemoteInputDr16 = 1,
	kRemoteInputVt03 = 2,
	kRemoteInputWfly = 3,
};

struct RemoteInputState {
	uint8_t source;
	float chassis_x;
	float chassis_rotate;
	float yaw_angle;
	float pitch_angle;
	float leg_length;
	float leg_length_delta;
	float friction_speed;
	float plucker;
	bool run;
	bool robot_enable;
	bool fast_spin;
	bool supercap;
	bool auto_aim;
	bool friction_wheel;
	bool auth_shoot;
	bool pc_shoot_control;

	uint32_t sequence;
};

}  // namespace channels

extern SeqlockValue<channels::RemoteInputState> latest_remote_state;
