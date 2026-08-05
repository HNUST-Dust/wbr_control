/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include "../module_base.h"
#include <protocols/imu/hi91_protocol.h>

namespace modules
{

class Hi91ImuModule : public ModuleBase
{
public:
	Hi91ImuModule();
	int Start() override;
	void RunLoop() override;

private:
	/*
	 * 一个 HI91 帧占 82 个 UART 字节。单次 DMA 传输 96 字节，可避免满缓冲
	 * 事件一次积压两个 1 kHz 样本。每个 DMA 缓冲保留 128 字节，使交替缓冲
	 * 分别按缓存行对齐，缓存失效操作不会越界影响另一个活动缓冲。
	 */
	static constexpr size_t kRxDmaTransferSize = 96U;
	static constexpr size_t kRxBufferStorageSize = 128U;
	static constexpr size_t kRxBufferCount = 2U;
	static constexpr size_t kRxRingSize = 8192U;
	static constexpr size_t kDmaCacheLineSize = 64U;

	static void UartCallback(const struct device *dev, struct uart_event *evt, void *user_data);
	void HandleUartEvent(const struct device *dev, const struct uart_event *evt);
	void InvalidateDmaRxCache(const uint8_t *data, size_t len);
	void ProcessBytes(const uint8_t *data, size_t size);
	void PublishSample(const protocols::Hi91Sample &sample);
	void ReportParseIssue(protocols::Hi91ParseResult result);

	const struct device *uart_dev_ = nullptr;
	struct k_sem rx_sem_;
	struct ring_buf rx_ring_;
	uint8_t rx_ring_storage_[kRxRingSize];
	alignas(kDmaCacheLineSize) uint8_t rx_buffers_[kRxBufferCount][kRxBufferStorageSize];
	uint8_t next_rx_buffer_index_ = 0U;
	protocols::Hi91Parser parser_;
	uint32_t sample_sequence_ = 0U;
	uint32_t parse_error_count_ = 0U;
	uint32_t rx_drop_count_ = 0U;
	uint32_t rx_stop_count_ = 0U;
	uint64_t last_publish_time_us_ = 0U;
	uint32_t max_publish_interval_us_ = 0U;
};

} // namespace modules
