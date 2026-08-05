/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "../module_base.h"
#include <channels/oscilloscope_sample.hpp>
#include <protocols/telemetry/vofa_protocol.h>

namespace modules
{

class OscilloscopeModule : public ModuleBase
{
public:
	OscilloscopeModule() = default;
	int Start() override;
	void RunLoop() override;

private:
	static void UartCallback(const struct device *dev, struct uart_event *event,
				 void *user_data);
	int SendLatestSample();

	const struct device *uart_dev_ = nullptr;
	uint32_t last_sequence_ = 0U;
	uint32_t missed_release_count_ = 0U;
	atomic_t tx_busy_ = ATOMIC_INIT(0);
	uint8_t tx_frame_[protocols::VofaJustFloatFrameSize(channels::kOscilloscopeMaxChannels)] =
		{};
};

} // namespace modules
