/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <zephyr/device.h>
#include <zephyr/kernel.h>

namespace modules {

class OscilloscopeModule {
public:
	const char *Name() const { return "oscilloscope"; }
	int Initialize();
	int Start();

private:
	void RunLoop();
	int SendLatestSample();

	struct k_thread thread_;
	const struct device *uart_dev_ = nullptr;
	bool started_ = false;
	uint32_t last_sequence_ = 0U;
};

}  // namespace modules
