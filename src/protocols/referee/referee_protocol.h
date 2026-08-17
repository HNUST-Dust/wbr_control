/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

// Public RoboMaster referee system wire-format contract.

#include <stddef.h>
#include <stdint.h>

namespace protocols {

constexpr uint8_t kRefereeSof = 0xA5U;

/*
 * Frame layout:
 *   byte 0       SOF (0xa5)
 *   bytes 1-2    data length (little-endian)
 *   byte 3       sequence
 *   byte 4       crc8 (header)
 *   bytes 5-6    command id (little-endian)
 *   bytes 7..    data (data length bytes)
 *   last 2 bytes crc16 (frame)
 */
constexpr size_t kRefereeFrameHeaderSize = 7U;
constexpr size_t kRefereeFrameTrailerSize = 2U;
constexpr size_t kRefereeMaxFrameSize = 160U;
constexpr size_t kRefereeMaxDataLength =
	kRefereeMaxFrameSize - kRefereeFrameHeaderSize - kRefereeFrameTrailerSize;

constexpr uint16_t kRefereeCmdGameStatus = 0x0001U;
constexpr uint16_t kRefereeCmdRobotStatus = 0x0201U;
constexpr uint16_t kRefereeCmdShootData = 0x0207U;

struct RefereeGameStatus {
	uint8_t game_type;      /* 4 bits */
	uint8_t game_progress;  /* 4 bits */
	uint16_t stage_remain_time;
};

struct RefereeRobotStatus {
	uint16_t current_hp;
	uint16_t max_hp;
	uint16_t chassis_power_limit;
	uint8_t gimbal_power_on;
};

struct RefereeShootData {
	uint8_t bullet_type;
	uint8_t launching_frequency;
	float initial_speed;
};

struct RefereeFrame {
	uint16_t cmd_id;
	const uint8_t *data;
	uint16_t data_len;
};

/*
 * Validate one complete referee frame and extract its command id and data.
 *
 * CRC8/CRC16 are part of the referee protocol but are not verified yet;
 * see the frame layout comment above.  Returns 0 on success or a negative
 * errno (-EINVAL / -EMSGSIZE / -EBADMSG).
 */
int DecodeRefereeFrame(const uint8_t *frame, size_t frame_len, RefereeFrame *out);

/* Decode the payloads of the currently supported referee commands. */
int DecodeRefereeGameStatus(const uint8_t *data, size_t len, RefereeGameStatus *out);
int DecodeRefereeRobotStatus(const uint8_t *data, size_t len, RefereeRobotStatus *out);
int DecodeRefereeShootData(const uint8_t *data, size_t len, RefereeShootData *out);

}  // namespace protocols
