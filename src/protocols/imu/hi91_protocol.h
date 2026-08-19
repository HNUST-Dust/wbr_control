/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

// Public HI91 wire-format contract.

#include <stddef.h>
#include <stdint.h>

namespace protocols {

constexpr uint8_t kHi91FrameSof0 = 0x5AU;
constexpr uint8_t kHi91FrameSof1 = 0xA5U;
constexpr uint8_t kHi91DataTag = 0x91U;
constexpr uint16_t kHi91DataLength = 76U;
constexpr uint16_t kHi91MaxPayloadLength = 256U;
constexpr size_t kHi91FrameHeaderSize = 6U; /* SOF0 SOF1 len(2) crc(2) */

struct Hi91Sample {
	uint16_t main_status;
	int8_t temperature_c;
	float air_pressure;
	uint32_t system_time_ms;
	float accel_g[3];
	float gyro_dps[3];
	float mag_ut[3];
	float roll_deg;
	float pitch_deg;
	float yaw_deg;
	float quat[4];
};

/*
 * Decode one complete HI91 frame (SOF + len + crc + payload).
 *
 * `strict_crc` rejects frames whose CRC16 does not match the device
 * (see Kconfig WBR_CONTROL_HI91_IMU_STRICT_CRC); keep it false until the exact
 * CRC variant is confirmed.
 *
 * Returns 0 on success, or a negative errno value (-EINVAL / -EMSGSIZE /
 * -EBADMSG) on failure.
 */
int DecodeHi91Frame(const uint8_t *data, size_t len, bool strict_crc, Hi91Sample *out);

/* HiPNUC CRC-16/CCITT update with the manual's initial value 0x0000. */
uint16_t Hi91Crc16CcittFalse(const uint8_t *data, size_t size);

}  // namespace protocols
