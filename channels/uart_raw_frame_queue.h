/* SPDX-License-Identifier: Apache-2.0 */

#ifndef WBR_CONTROL_CHANNELS_UART_RAW_FRAME_QUEUE_H_
#define WBR_CONTROL_CHANNELS_UART_RAW_FRAME_QUEUE_H_

#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

namespace channels {

constexpr size_t kUartRawChunkSize = 32U;
constexpr size_t kQueueDepth = 32U;
constexpr size_t kRemoteInputRingBufSize = 512U;

struct UartRawFrameMessage {
  uint8_t len;
  uint8_t data[kUartRawChunkSize];
};

extern struct ring_buf remote_input_ring_buf;
extern struct k_sem remote_input_sem;
extern struct k_msgq referee_uart_raw_msgq;

} // namespace channels

#endif /* WBR_CONTROL_CHANNELS_UART_RAW_FRAME_QUEUE_H_ */
