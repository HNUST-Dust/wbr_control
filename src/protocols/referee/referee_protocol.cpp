/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <protocols/referee/referee_protocol.h>

#include <errno.h>
#include <string.h>

namespace protocols {

namespace {

uint16_t ReadLe16(const uint8_t *data)
{
	return static_cast<uint16_t>(data[0]) |
	       (static_cast<uint16_t>(data[1]) << 8U);
}

float ReadLeFloat(const uint8_t *data)
{
	const uint32_t raw = static_cast<uint32_t>(data[0]) |
			     (static_cast<uint32_t>(data[1]) << 8U) |
			     (static_cast<uint32_t>(data[2]) << 16U) |
			     (static_cast<uint32_t>(data[3]) << 24U);
	float value = 0.0F;
	memcpy(&value, &raw, sizeof(value));
	return value;
}

}  // namespace

int DecodeRefereeFrame(const uint8_t *frame, size_t frame_len, RefereeFrame *out)
{
	if ((frame == nullptr) || (out == nullptr) ||
	    (frame_len < kRefereeFrameHeaderSize)) {
		return -EINVAL;
	}

	if (frame[0] != kRefereeSof) {
		return -EBADMSG;
	}

	const uint16_t data_len = ReadLe16(&frame[1]);
	if (data_len > kRefereeMaxDataLength) {
		return -EBADMSG;
	}

	const size_t full_len = kRefereeFrameHeaderSize + data_len + kRefereeFrameTrailerSize;
	if (frame_len < full_len) {
		return -EMSGSIZE;
	}

	out->cmd_id = ReadLe16(&frame[5]);
	out->data = &frame[kRefereeFrameHeaderSize];
	out->data_len = data_len;
	return 0;
}

int DecodeRefereeGameStatus(const uint8_t *data, size_t len, RefereeGameStatus *out)
{
	if ((data == nullptr) || (out == nullptr) || (len < 3U)) {
		return -EINVAL;
	}

	out->game_type = data[0] & 0x0fU;
	out->game_progress = static_cast<uint8_t>((data[0] >> 4) & 0x0fU);
	out->stage_remain_time = ReadLe16(&data[1]);
	return 0;
}

int DecodeRefereeRobotStatus(const uint8_t *data, size_t len, RefereeRobotStatus *out)
{
	if ((data == nullptr) || (out == nullptr) || (len < 13U)) {
		return -EINVAL;
	}

	out->current_hp = ReadLe16(&data[2]);
	out->max_hp = ReadLe16(&data[4]);
	out->chassis_power_limit = ReadLe16(&data[10]);
	out->gimbal_power_on = ((data[12] & 0x01U) != 0U) ? 1U : 0U;
	return 0;
}

int DecodeRefereeShootData(const uint8_t *data, size_t len, RefereeShootData *out)
{
	if ((data == nullptr) || (out == nullptr) || (len < 7U)) {
		return -EINVAL;
	}

	out->bullet_type = data[0];
	out->launching_frequency = data[2];
	out->initial_speed = ReadLeFloat(&data[3]);
	return 0;
}

}  // namespace protocols
