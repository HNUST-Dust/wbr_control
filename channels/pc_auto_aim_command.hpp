/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>

#include <channels/comm/seqlock_value.hpp>

namespace channels {

/*
 * Auto-aim command decoded from a PC link packet and published for the
 * gimbal / booster modules to consume.
 */
struct PcAutoAimCommand {
	uint32_t sequence;
	uint8_t mode; /* 0 idle, 1 aim without firing, 2 aim and fire */
	float yaw_angle;
	float yaw_velocity;
	float yaw_acceleration;
	float pitch_angle;
	float pitch_velocity;
	float pitch_acceleration;
};

extern SeqlockValue<PcAutoAimCommand> latest_pc_auto_aim_command;

}  // namespace channels
