/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

namespace msg {

template <size_t Capacity>
class ByteStream {
public:
	ByteStream()
	{
		ring_buf_init(&ring_, Capacity, storage_);
		k_sem_init(&available_, 0, 1);
	}

	uint32_t Write(const uint8_t *data, size_t size)
	{
		const uint32_t written = ring_buf_put(&ring_, data, size);
		if (written != 0U) {
			k_sem_give(&available_);
		}
		return written;
	}

	uint32_t Read(uint8_t *data, size_t size) { return ring_buf_get(&ring_, data, size); }
	int Wait(k_timeout_t timeout) { return k_sem_take(&available_, timeout); }

private:
	uint8_t storage_[Capacity] = {};
	struct ring_buf ring_ = {};
	struct k_sem available_ = {};
};

} // namespace msg
