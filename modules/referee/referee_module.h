/* SPDX-License-Identifier: Apache-2.0 */

#pragma once
#include <zephyr/kernel.h>

namespace modules {

class RefereeModule {
public:
	const char *Name() const { return "referee"; }
	int Initialize();
	int Start();

private:
	void RunLoop();
	void DecodeUartFramesInQueue();

	struct k_thread thread_;
	bool started_ = false;
	uint32_t sequence_ = 0U;
};

}  // namespace modules
