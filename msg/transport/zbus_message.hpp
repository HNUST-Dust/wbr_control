/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <zephyr/zbus/zbus.h>

namespace msg {

template <typename T>
class ZbusMessage {
public:
	explicit ZbusMessage(const struct zbus_channel *channel) : channel_(channel) {}
	int Publish(const T &value, k_timeout_t timeout = K_NO_WAIT) const
	{
		return zbus_chan_pub(channel_, &value, timeout);
	}

private:
	const struct zbus_channel *channel_;
};

} // namespace msg
