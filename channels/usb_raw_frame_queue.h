/* SPDX-License-Identifier: Apache-2.0 */

#ifndef WBR_CONTROL_CHANNELS_USB_RAW_FRAME_QUEUE_H_
#define WBR_CONTROL_CHANNELS_USB_RAW_FRAME_QUEUE_H_

#include <stddef.h>
#include <stdint.h>

namespace channels {

constexpr size_t kUsbRawChunkSize = 64U;

struct UsbRawFrameMessage {
	uint16_t len;
	uint8_t data[kUsbRawChunkSize];
};

int EnqueueUsbRawFrame(const UsbRawFrameMessage *frame);
int DequeueUsbRawFrame(UsbRawFrameMessage *frame, int32_t timeout_ms);

}  // namespace channels

#endif /* WBR_CONTROL_CHANNELS_USB_RAW_FRAME_QUEUE_H_ */
