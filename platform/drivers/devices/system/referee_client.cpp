/* SPDX-License-Identifier: Apache-2.0 */

#include <platform/drivers/devices/system/referee_client.h>

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>

namespace platform {

namespace {

constexpr uint8_t kSof = 0xa5U;
constexpr size_t kMaxFrameSize = 160U;
constexpr uint16_t kCmdGameStatus = 0x0001U;
constexpr uint16_t kCmdRobotStatus = 0x0201U;
constexpr uint16_t kCmdShootData = 0x0207U;

uint8_t g_stream_buf[kMaxFrameSize] = {0};
size_t g_stream_len = 0U;

struct RefereeState {
	uint16_t current_hp;
	uint16_t max_hp;
	uint16_t chassis_power_limit;
	uint8_t gimbal_power_on;
	uint8_t bullet_type;
	uint8_t launching_frequency;
	float initial_speed;
	uint8_t game_type;
	uint8_t game_progress;
	uint16_t stage_remain_time;
};

RefereeState g_state = {};
struct k_mutex g_state_mutex;
bool g_initialized = false;

uint16_t ReadLe16(const uint8_t *p)
{
	return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

void ConsumeBytes(size_t n)
{
	if (n >= g_stream_len) {
		g_stream_len = 0U;
		return;
	}

	memmove(g_stream_buf, g_stream_buf + n, g_stream_len - n);
	g_stream_len -= n;
}

void ParseRobotStatus(const uint8_t *data, size_t len)
{
	if (len < 13U) {
		return;
	}

	(void)k_mutex_lock(&g_state_mutex, K_FOREVER);
	g_state.current_hp = ReadLe16(&data[2]);
	g_state.max_hp = ReadLe16(&data[4]);
	g_state.chassis_power_limit = ReadLe16(&data[10]);
	g_state.gimbal_power_on = ((data[12] & 0x01U) != 0U) ? 1U : 0U;
	k_mutex_unlock(&g_state_mutex);
}

void ParseShootData(const uint8_t *data, size_t len)
{
	if (len < 7U) {
		return;
	}

	float speed = 0.0f;
	memcpy(&speed, &data[3], sizeof(float));

	(void)k_mutex_lock(&g_state_mutex, K_FOREVER);
	g_state.bullet_type = data[0];
	g_state.launching_frequency = data[2];
	g_state.initial_speed = speed;
	k_mutex_unlock(&g_state_mutex);
}

void ParseGameStatus(const uint8_t *data, size_t len)
{
	if (len < 3U) {
		return;
	}

	(void)k_mutex_lock(&g_state_mutex, K_FOREVER);
	g_state.game_type = static_cast<uint8_t>(data[0] & 0x0fU);
	g_state.game_progress = static_cast<uint8_t>((data[0] >> 4) & 0x0fU);
	g_state.stage_remain_time = ReadLe16(&data[1]);
	k_mutex_unlock(&g_state_mutex);
}

void ParseOneFrame(const uint8_t *frame, size_t frame_len)
{
	if (frame_len < 9U) {
		return;
	}

	const uint16_t data_len = ReadLe16(&frame[1]);
	const uint16_t cmd = ReadLe16(&frame[5]);
	const uint8_t *payload = &frame[7];

	switch (cmd) {
	case kCmdRobotStatus:
		ParseRobotStatus(payload, data_len);
		break;
	case kCmdShootData:
		ParseShootData(payload, data_len);
		break;
	case kCmdGameStatus:
		ParseGameStatus(payload, data_len);
		break;
	default:
		break;
	}
}

void TryParseStream()
{
	while (g_stream_len >= 9U) {
		if (g_stream_buf[0] != kSof) {
			ConsumeBytes(1U);
			continue;
		}

		const uint16_t data_len = ReadLe16(&g_stream_buf[1]);
		const size_t full_len = 9U + data_len;
		if (full_len > kMaxFrameSize) {
			ConsumeBytes(1U);
			continue;
		}

		if (g_stream_len < full_len) {
			break;
		}

		ParseOneFrame(g_stream_buf, full_len);
		ConsumeBytes(full_len);
	}
}

}  // namespace

int InitializeRefereeClient()
{
	if (g_initialized) {
		return 0;
	}

	(void)k_mutex_init(&g_state_mutex);
	g_state = {};
	g_stream_len = 0U;
	g_initialized = true;
	return 0;
}

int FeedRefereeBytes(const uint8_t *data, size_t len)
{
	if ((data == nullptr) || (len == 0U)) {
		return -EINVAL;
	}

	if (!g_initialized) {
		const int rc = InitializeRefereeClient();
		if (rc != 0) {
			return rc;
		}
	}

	for (size_t i = 0U; i < len; ++i) {
		if (g_stream_len < kMaxFrameSize) {
			g_stream_buf[g_stream_len++] = data[i];
		} else {
			memmove(g_stream_buf, g_stream_buf + 1, kMaxFrameSize - 1U);
			g_stream_buf[kMaxFrameSize - 1U] = data[i];
		}
	}

	TryParseStream();
	return 0;
}



}  // namespace platform
