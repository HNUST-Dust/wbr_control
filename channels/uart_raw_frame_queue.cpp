/* SPDX-License-Identifier: Apache-2.0 */

/**
* @file channels/uart_raw_frame_queue.cpp
 * @ingroup wbr_channels
 * @brief 提供 UART 原始帧的线程安全消息队列。
 * @details 本文件仅定义对应通道的全局存储实例，不启动线程、不访问硬件，也不改变消息字段。
 */

#include <channels/uart_raw_frame_queue.h>

namespace channels {

RING_BUF_DECLARE(remote_input_ring_buf, kRemoteInputRingBufSize);
K_SEM_DEFINE(remote_input_sem, 0, 1);
K_MSGQ_DEFINE(referee_uart_raw_msgq, sizeof(UartRawFrameMessage), kQueueDepth,
              4);

} // namespace channels
