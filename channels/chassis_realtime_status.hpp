/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

#include <channels/comm/seqlock_value.hpp>

namespace channels {

enum class ChassisImuSource : uint8_t {
	kHi91 = 0U,
	kOnboardEkf = 2U,
};

/** Low-rate diagnostics for proving the 1 kHz chassis scheduling margin. */
struct ChassisRealtimeStatus {
	uint32_t sequence;
	uint32_t uptime_ms;
	uint32_t deadline_miss_count;
	uint32_t loop_execution_us;
	uint32_t max_loop_execution_us;
	uint32_t stack_unused_bytes;
	uint64_t imu_age_us;
	ChassisImuSource imu_source;
	bool imu_fresh;
};

extern SeqlockValue<ChassisRealtimeStatus> latest_chassis_realtime_status;

}  // namespace channels
