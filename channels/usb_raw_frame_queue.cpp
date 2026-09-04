/* SPDX-License-Identifier: Apache-2.0 */

/**
* @file channels/usb_raw_frame_queue.cpp
 * @ingroup wbr_channels
 * @brief 提供 USB 原始帧的线程安全消息队列。
 * @details 本文件仅定义对应通道的全局存储实例，不启动线程、不访问硬件，也不改变消息字段。
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <channels/usb_raw_frame_queue.h>

namespace {

constexpr size_t kQueueDepth = 16U;

K_MSGQ_DEFINE(g_usb_raw_msgq,
	     sizeof(channels::UsbRawFrameMessage),
	     kQueueDepth,
	     4);

k_timeout_t TimeoutFromMs(int32_t timeout_ms)
{
	if (timeout_ms < 0) {
		return K_FOREVER;
	}

	if (timeout_ms > 0) {
		return K_MSEC(timeout_ms);
	}

	return K_NO_WAIT;
}

}  // namespace

namespace channels {

int EnqueueUsbRawFrame(const UsbRawFrameMessage *frame)
{
	if (frame == nullptr) {
		return -EINVAL;
	}

	return k_msgq_put(&g_usb_raw_msgq, frame, K_NO_WAIT);
}

int DequeueUsbRawFrame(UsbRawFrameMessage *frame, int32_t timeout_ms)
{
	if (frame == nullptr) {
		return -EINVAL;
	}

	return k_msgq_get(&g_usb_raw_msgq, frame, TimeoutFromMs(timeout_ms));
}

}  // namespace channels
