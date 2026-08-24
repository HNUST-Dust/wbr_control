/* SPDX-License-Identifier: Apache-2.0 */

/**
* @file src/protocols/remote_input/wfly_sbus_protocol.cpp
 * @ingroup wbr_protocols
 * @brief 实现天地飞 SBUS 遥控协议解码。
 * @details 实现显式处理字节序、帧长度和量化范围，不依赖动态内存。所有协议错误通过返回值报告，解析器不会直接驱动执行器。
 */

#include <protocols/remote_input/wfly_sbus_protocol.h>

namespace protocols {

namespace {

constexpr uint8_t kChannel17Mask = 0x01;
constexpr uint8_t kChannel18Mask = 0x02;
constexpr uint8_t kFrameLostMask = 0x04;
constexpr uint8_t kFailsafeMask = 0x08;
constexpr uint16_t kChannelMask = 0x07ff;

uint16_t U16(uint8_t value) { return static_cast<uint16_t>(value); }

} // namespace

bool DecodeWflySbusFrame(const uint8_t *data, size_t len, WflySbusFrame *out) {
  if ((data == nullptr) || (out == nullptr) || (len < kWflySbusFrameLength)) {
    return false;
  }

  if ((data[0] != kWflySbusStartByte) || (data[24] != kWflySbusEndByte)) {
    return false;
  }

  const uint8_t *payload = &data[1];

  out->channels[0] = (U16(payload[0]) | (U16(payload[1]) << 8)) & kChannelMask;
  out->channels[1] =
      ((U16(payload[1]) >> 3) | (U16(payload[2]) << 5)) & kChannelMask;
  out->channels[2] = ((U16(payload[2]) >> 6) | (U16(payload[3]) << 2) |
                      (U16(payload[4]) << 10)) &
                     kChannelMask;
  out->channels[3] =
      ((U16(payload[4]) >> 1) | (U16(payload[5]) << 7)) & kChannelMask;
  out->channels[4] =
      ((U16(payload[5]) >> 4) | (U16(payload[6]) << 4)) & kChannelMask;
  out->channels[5] = ((U16(payload[6]) >> 7) | (U16(payload[7]) << 1) |
                      (U16(payload[8]) << 9)) &
                     kChannelMask;
  out->channels[6] =
      ((U16(payload[8]) >> 2) | (U16(payload[9]) << 6)) & kChannelMask;
  out->channels[7] =
      ((U16(payload[9]) >> 5) | (U16(payload[10]) << 3)) & kChannelMask;

  out->channels[8] =
      (U16(payload[11]) | (U16(payload[12]) << 8)) & kChannelMask;
  out->channels[9] =
      ((U16(payload[12]) >> 3) | (U16(payload[13]) << 5)) & kChannelMask;
  out->channels[10] = ((U16(payload[13]) >> 6) | (U16(payload[14]) << 2) |
                       (U16(payload[15]) << 10)) &
                      kChannelMask;
  out->channels[11] =
      ((U16(payload[15]) >> 1) | (U16(payload[16]) << 7)) & kChannelMask;
  out->channels[12] =
      ((U16(payload[16]) >> 4) | (U16(payload[17]) << 4)) & kChannelMask;
  out->channels[13] = ((U16(payload[17]) >> 7) | (U16(payload[18]) << 1) |
                       (U16(payload[19]) << 9)) &
                      kChannelMask;
  out->channels[14] =
      ((U16(payload[19]) >> 2) | (U16(payload[20]) << 6)) & kChannelMask;
  out->channels[15] =
      ((U16(payload[20]) >> 5) | (U16(payload[21]) << 3)) & kChannelMask;

  const uint8_t flags = data[23];
  out->channel17 = (flags & kChannel17Mask) != 0U;
  out->channel18 = (flags & kChannel18Mask) != 0U;
  out->frame_lost = (flags & kFrameLostMask) != 0U;
  out->failsafe = (flags & kFailsafeMask) != 0U;
  return true;
}

} // namespace protocols
