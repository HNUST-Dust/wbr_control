/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <zephyr/kernel.h>

#include "../module_base.h"

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

	uint32_t sequence_ = 0U;
};

} // namespace modules
