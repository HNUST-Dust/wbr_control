/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <channels/usb_raw_frame_queue.h>

namespace {

constexpr size_t kQueueDepth = 16U;

K_MSGQ_DEFINE(g_cdc_acm_usb_raw_msgq,
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

int EnqueueForCdcAcm(const UsbRawFrameMessage *frame)
{
	if (frame == nullptr) {
		return -EINVAL;
	}

	return k_msgq_put(&g_cdc_acm_usb_raw_msgq, frame, K_NO_WAIT);
}

int DequeueForCdcAcm(UsbRawFrameMessage *frame, int32_t timeout_ms)
{
	if (frame == nullptr) {
		return -EINVAL;
	}

	return k_msgq_get(&g_cdc_acm_usb_raw_msgq, frame, TimeoutFromMs(timeout_ms));
}

}  // namespace channels
