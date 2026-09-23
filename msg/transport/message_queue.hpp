/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <zephyr/kernel.h>

namespace msg {

template <typename T, size_t Depth>
class MessageQueue {
public:
	MessageQueue()
	{
		k_msgq_init(&queue_, buffer_, sizeof(T), Depth);
	}

	int TryPush(const T &value) { return k_msgq_put(&queue_, &value, K_NO_WAIT); }
	int Push(const T &value, k_timeout_t timeout) { return k_msgq_put(&queue_, &value, timeout); }
	int Pop(T &value, k_timeout_t timeout) { return k_msgq_get(&queue_, &value, timeout); }

private:
	struct k_msgq queue_ = {};
	alignas(4) char buffer_[sizeof(T) * Depth] = {};
};

} // namespace msg
