/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include <zephyr/zbus/zbus.h>

namespace channels {

enum BootPhase : uint8_t {
	kBooting = 0,
	kRunning = 1,
};

struct SystemStatusMessage {
	BootPhase phase;
	uint32_t active_modules;
};

}  // namespace channels

ZBUS_CHAN_DECLARE(wbr_control_system_status_chan);
