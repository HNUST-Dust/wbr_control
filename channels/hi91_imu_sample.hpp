/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>

#include <channels/comm/seqlock_value.hpp>

namespace channels {

struct Hi91ImuSample {
	uint32_t sequence;
	uint32_t uptime_ms;
	uint32_t system_time_ms;
	bool valid;
	float roll_deg;
	float pitch_deg;
	float yaw_deg;
	float quat[4];
	float gyro_dps[3];
	float accel_g[3];
};

extern SeqlockValue<Hi91ImuSample> latest_hi91_imu_sample;

}  // namespace channels
