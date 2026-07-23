/* SPDX-License-Identifier: Apache-2.0 */

#ifndef RM_TEST_APP_CHANNELS_USB_RAW_FRAME_QUEUE_H_
#define RM_TEST_APP_CHANNELS_USB_RAW_FRAME_QUEUE_H_

#include <stddef.h>
#include <stdint.h>

namespace channels {

constexpr size_t kUsbRawChunkSize = 512U;

struct UsbRawFrameMessage {
	uint16_t len;
	uint8_t data[kUsbRawChunkSize];
};

int EnqueueForCdcAcm(const UsbRawFrameMessage *frame);
int DequeueForCdcAcm(UsbRawFrameMessage *frame, int32_t timeout_ms);

}  // namespace channels

#endif /* RM_TEST_APP_CHANNELS_USB_RAW_FRAME_QUEUE_H_ */
