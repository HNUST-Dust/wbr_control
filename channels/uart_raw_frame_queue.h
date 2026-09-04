/* SPDX-License-Identifier: Apache-2.0 */

/**
* @file channels/uart_raw_frame_queue.h
 * @ingroup wbr_channels
 * @brief 提供 UART 原始帧的线程安全消息队列。
 * @details 声明跨线程交换的数据快照及其唯一全局通道。写入方发布完整对象，读取方不得保存内部存储地址；使用顺序锁的通道允许读取失败，调用方应保留上一份有效快照。
 */

#ifndef WBR_CONTROL_CHANNELS_UART_RAW_FRAME_QUEUE_H_
#define WBR_CONTROL_CHANNELS_UART_RAW_FRAME_QUEUE_H_

#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

namespace channels {

constexpr size_t kUartRawChunkSize = 32U; ///< UART 原始消息单次可携带的最大字节数。
constexpr size_t kQueueDepth = 32U; ///< 消息队列可缓存的消息数量。
constexpr size_t kRemoteInputRingBufSize = 512U; ///< 遥控 UART 环形缓冲区容量，单位为字节。

/** @brief UART 接收到的原始数据帧。 */
struct UartRawFrameMessage {
  uint8_t len; ///< 缓冲区内的有效数据长度，单位为字节。
  uint8_t data[kUartRawChunkSize]; ///< 协议或总线载荷的原始字节。
};

/** @brief UART 回调与遥控解析线程之间的无锁字节环形缓冲区。 */
extern struct ring_buf remote_input_ring_buf;
/** @brief 遥控 UART 收到新字节时释放的线程同步信号量。 */
extern struct k_sem remote_input_sem;
/** @brief UART 回调向裁判模块线程投递原始帧的消息队列。 */
extern struct k_msgq referee_uart_raw_msgq;

} // namespace channels

#endif /* WBR_CONTROL_CHANNELS_UART_RAW_FRAME_QUEUE_H_ */
