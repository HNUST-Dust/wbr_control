/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <channels/system_status_channel.h>

ZBUS_CHAN_DEFINE(wbr_control_system_status_chan,
		 channels::SystemStatusMessage,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(.phase = channels::kBooting,
			       .active_modules = 0U));
