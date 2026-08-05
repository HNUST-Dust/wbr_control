/* SPDX-License-Identifier: Apache-2.0 */

#ifndef WBR_CONTROL_PROTOCOLS_REMOTE_INPUT_WFLY_SBUS_PROTOCOL_H_
#define WBR_CONTROL_PROTOCOLS_REMOTE_INPUT_WFLY_SBUS_PROTOCOL_H_

#include <array>
#include <stddef.h>
#include <stdint.h>

namespace protocols {

constexpr size_t kWflySbusFrameLength = 25;
constexpr uint8_t kWflySbusStartByte = 0x0f;
constexpr uint8_t kWflySbusEndByte = 0x00;
constexpr size_t kWflySbusChannelCount = 16;

struct WflySbusFrame {
  std::array<uint16_t, kWflySbusChannelCount> channels;
  bool channel17;
  bool channel18;
  bool frame_lost;
  bool failsafe;
};

bool DecodeWflySbusFrame(const uint8_t *data, size_t len, WflySbusFrame *out);

} // namespace protocols

#endif /* WBR_CONTROL_PROTOCOLS_REMOTE_INPUT_WFLY_SBUS_PROTOCOL_H_ */
