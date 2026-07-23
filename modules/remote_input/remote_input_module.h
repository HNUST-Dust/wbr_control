/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <stddef.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include <channels/remote_input_state.hpp>
#include <channels/uart_raw_frame_queue.h>

struct device;
struct uart_event;

namespace modules {

class RemoteInputModule {
public:
	const char *Name() const { return "remote_input"; }
	int Initialize();
	int Start();

private:
	static constexpr size_t kLineBufSize = 96U;
	static constexpr size_t kBinaryBufSize = 256U;

	void RunLoop();
	int StartUartRx();
	static void UartCallback(const struct device *dev, struct uart_event *evt, void *user_data);
	void HandleUartEvent(const struct device *dev, const struct uart_event *evt);
	void DecodeUartBytesFromRing();
	int ParseLine(const char *line, channels::RemoteInputState *out);
	void TryDecodeBinaryFrames();
	void ConsumeBinary(size_t bytes);
	void PublishRemoteState(channels::RemoteInputState *input);

	struct k_thread thread_;
	const struct device *uart_dev_ = nullptr;
	bool started_ = false;
	bool uart_rx_started_ = false;
	uint32_t publish_sequence_ = 0U;
	uint32_t rx_drop_count_ = 0U;
	uint32_t rx_stop_count_ = 0U;
	uint8_t next_rx_buffer_index_ = 1U;
	char line_buf_[kLineBufSize] = {0};
	size_t line_pos_ = 0U;
	uint8_t binary_buf_[kBinaryBufSize] = {0};
	size_t binary_len_ = 0U;
};

}  // namespace modules
