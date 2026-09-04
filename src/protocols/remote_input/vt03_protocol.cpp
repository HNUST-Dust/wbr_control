/* SPDX-License-Identifier: Apache-2.0 */

/**
* @file src/protocols/remote_input/vt03_protocol.cpp
 * @ingroup wbr_protocols
 * @brief 实现 VT03 遥控器协议解码。
 * @details 实现显式处理字节序、帧长度和量化范围，不依赖动态内存。所有协议错误通过返回值报告，解析器不会直接驱动执行器。
 */

#include <protocols/remote_input/vt03_protocol.h>

#include <string.h>

#include <zephyr/sys/util.h>

namespace protocols {

namespace {

constexpr float kRockerOffset = 364.0f;
constexpr float kRockerScale = 1320.0f;
constexpr uint16_t kCustomControllerDataId = 0x0302;

float ClampNormalized(float x)
{
	if (x > 0.99f) {
		return 0.99f;
	}
	if (x < -0.99f) {
		return -0.99f;
	}
	return x;
}

float DecodeAxis(uint16_t raw)
{
	return ClampNormalized((static_cast<float>(raw) - kRockerOffset) / kRockerScale);
}

}  // namespace

bool DecodeVt03RemoteFrame(const uint8_t *data, size_t len, Vt03Frame *out)
{
	if ((data == nullptr) || (out == nullptr) || (len < kVt03RemoteFrameLength)) {
		return false;
	}

	if ((data[0] != 0xa9U) || (data[1] != 0x53U)) {
		return false;
	}

	const uint16_t ch0 = (static_cast<uint16_t>(data[2]) |
				      (static_cast<uint16_t>(data[3]) << 8)) &
				     0x07ffU;
	const uint16_t ch1 = (static_cast<uint16_t>(data[3]) >> 3 |
				      (static_cast<uint16_t>(data[4]) << 5)) &
				     0x07ffU;
	const uint16_t ch2 = (static_cast<uint16_t>(data[4]) >> 6 |
				      (static_cast<uint16_t>(data[5]) << 2) |
				      (static_cast<uint16_t>(data[6]) << 10)) &
				     0x07ffU;
	const uint16_t ch3 = (static_cast<uint16_t>(data[6]) >> 1 |
				      (static_cast<uint16_t>(data[7]) << 7)) &
				     0x07ffU;

	const uint8_t mode_switch = static_cast<uint8_t>((data[7] >> 4) & 0x03U);
	const uint16_t wheel_raw = (static_cast<uint16_t>(data[8]) >> 1 |
				      (static_cast<uint16_t>(data[9]) << 7)) &
				     0x07ffU;

	out->right_x = DecodeAxis(ch0);
	out->right_y = DecodeAxis(ch1);
	out->left_x = DecodeAxis(ch2);
	out->left_y = DecodeAxis(ch3);
	out->wheel = DecodeAxis(wheel_raw);
	out->mode_switch = mode_switch;
	out->chassis_enable = (mode_switch != 1U);
	return true;
}

bool DecodeVt03CustomFrame(const uint8_t *data, size_t len, Vt03CustomFrame *out)
{
	if ((data == nullptr) || (out == nullptr) || (len < kVt03CustomFrameLength)) {
		return false;
	}

	if (data[0] != 0xa5U) {
		return false;
	}

	const uint16_t cmd_id = static_cast<uint16_t>(data[5]) |
				(static_cast<uint16_t>(data[6]) << 8);
	if (cmd_id != kCustomControllerDataId) {
		return false;
	}

	memcpy(&out->joystick_x, &data[24], sizeof(float));
	memcpy(&out->joystick_y, &data[28], sizeof(float));
	memcpy(&out->joystick_z, &data[32], sizeof(float));
	out->chassis_enable = true;
	return true;
}

}  // namespace protocols
