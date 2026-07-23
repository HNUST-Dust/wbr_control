/* SPDX-License-Identifier: Apache-2.0 */

#include <channels/uart_raw_frame_queue.h>

namespace channels {

RING_BUF_DECLARE(remote_input_ring_buf, kRemoteInputRingBufSize);
K_SEM_DEFINE(remote_input_sem, 0, 1);
K_MSGQ_DEFINE(referee_uart_raw_msgq, sizeof(UartRawFrameMessage), kQueueDepth,
              4);

} // namespace channels
