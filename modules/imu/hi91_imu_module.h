/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include <protocols/imu/hi91_protocol.h>

namespace modules::imu {

class Hi91ImuModule {
public:
	int Initialize();
	int Start();
	const char *Name() const { return "hi91_imu"; }

private:
	static constexpr size_t kRxBufferSize = 256U;
	static constexpr size_t kRxBufferCount = 2U;
	static constexpr size_t kRxRingSize = 8192U;
	static constexpr size_t kDmaCacheLineSize = 64U;

	static void UartCallback(const struct device *dev, struct uart_event *evt, void *user_data);

	void RunLoop();
	void HandleUartEvent(const struct device *dev, const struct uart_event *evt);
	void InvalidateDmaRxCache(const uint8_t *data, size_t len);
	void ProcessBytes(const uint8_t *data, size_t size);
	void PublishSample(const protocols::imu::hi91::Sample &sample);
	void ReportParseIssue(protocols::imu::hi91::ParseResult result);

	bool started_;
	const struct device *uart_dev_;
	struct k_thread thread_;
	struct k_sem rx_sem_;
	struct ring_buf rx_ring_;
	uint8_t rx_ring_storage_[kRxRingSize];
	alignas(kDmaCacheLineSize) uint8_t rx_buffers_[kRxBufferCount][kRxBufferSize];
	uint8_t next_rx_buffer_index_;
	protocols::imu::hi91::Parser parser_;
	uint32_t sample_sequence_;
	uint32_t parse_error_count_;
	uint32_t rx_drop_count_;
	uint32_t rx_stop_count_;
};

}  // namespace modules::imu
