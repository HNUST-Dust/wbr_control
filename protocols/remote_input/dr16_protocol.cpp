/* SPDX-License-Identifier: Apache-2.0 */

#include <protocols/remote_input/dr16_protocol.h>

namespace protocols {

namespace {

constexpr float kNormalizeScale = 660.0f;

int AbsI16(int16_t value)
{
	return (value < 0) ? -value : value;
}

}  // namespace

bool DecodeDr16Frame(const uint8_t *data, size_t len, Dr16Frame *out)
{
	if ((data == nullptr) || (out == nullptr) || (len < kDr16FrameLength)) {
		return false;
	}

	const int16_t ch0 = static_cast<int16_t>(((static_cast<uint16_t>(data[0]) |
							  (static_cast<uint16_t>(data[1]) << 8)) &
							  0x07ffU)) -
				  1024;
	const int16_t ch1 = static_cast<int16_t>(((static_cast<uint16_t>(data[1]) >> 3 |
							  (static_cast<uint16_t>(data[2]) << 5)) &
							  0x07ffU)) -
				  1024;
	const int16_t ch2 = static_cast<int16_t>(((static_cast<uint16_t>(data[2]) >> 6 |
							  (static_cast<uint16_t>(data[3]) << 2) |
							  (static_cast<uint16_t>(data[4]) << 10)) &
							  0x07ffU)) -
				  1024;
	const int16_t ch3 = static_cast<int16_t>(((static_cast<uint16_t>(data[4]) >> 1 |
							  (static_cast<uint16_t>(data[5]) << 7)) &
							  0x07ffU)) -
				  1024;

	const uint8_t sw1 = static_cast<uint8_t>(((data[5] >> 4) & 0x0cU) >> 2);
	const uint8_t sw2 = static_cast<uint8_t>((data[5] >> 4) & 0x03U);
	const int16_t wheel = -static_cast<int16_t>(((static_cast<uint16_t>(data[16]) |
							       (static_cast<uint16_t>(data[17]) << 8)) &
							      0x07ffU) -
							      1024);

	if ((AbsI16(ch0) > 660) || (AbsI16(ch1) > 660) || (AbsI16(ch2) > 660) ||
	    (AbsI16(ch3) > 660) || (AbsI16(wheel) > 660)) {
		return false;
	}

	if (!((sw1 == 1U) || (sw1 == 2U) || (sw1 == 3U))) {
		return false;
	}

	if (!((sw2 == 1U) || (sw2 == 2U) || (sw2 == 3U))) {
		return false;
	}

	out->right_stick_x = static_cast<float>(ch0) / kNormalizeScale;
	out->right_stick_y = static_cast<float>(ch1) / kNormalizeScale;
	out->left_stick_x = static_cast<float>(ch2) / kNormalizeScale;
	out->left_stick_y = static_cast<float>(ch3) / kNormalizeScale;
	out->wheel = static_cast<float>(wheel) / kNormalizeScale;
	out->left_switch = sw1;
	out->right_switch = sw2;
	out->chassis_enable = (sw2 == 1U);
	return true;
}

}  // namespace protocols
