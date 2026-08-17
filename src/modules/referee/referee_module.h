/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#include "../module_base.h"
#include <protocols/referee/referee_protocol.h>

namespace modules
{

class RefereeModule : public ModuleBase
{
public:
	RefereeModule() = default;
	int Start() override;
	void RunLoop() override;

private:
	void DecodeUartFramesInQueue();
	void FeedBytes(const uint8_t *data, size_t len);
	void TryParseStream();
	void ConsumeStreamBytes(size_t n);
	void HandleFrame(const uint8_t *frame, size_t frame_len);

	uint32_t sequence_ = 0U;
	uint8_t stream_buf_[protocols::kRefereeMaxFrameSize] = {};
	size_t stream_len_ = 0U;
	protocols::RefereeGameStatus game_status_ = {};
	protocols::RefereeRobotStatus robot_status_ = {};
	protocols::RefereeShootData shoot_data_ = {};
};

} // namespace modules
