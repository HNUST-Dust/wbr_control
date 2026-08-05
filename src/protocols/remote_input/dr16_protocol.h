/* SPDX-License-Identifier: Apache-2.0 */

#ifndef WBR_CONTROL_PROTOCOLS_REMOTE_INPUT_DR16_PROTOCOL_H_
#define WBR_CONTROL_PROTOCOLS_REMOTE_INPUT_DR16_PROTOCOL_H_

#include <stddef.h>
#include <stdint.h>

namespace protocols {

constexpr size_t kDr16FrameLength = 18;

struct Dr16Frame {
	float right_stick_x;
	float right_stick_y;
	float left_stick_x;
	float left_stick_y;
	float wheel;
	uint8_t left_switch;
	uint8_t right_switch;
	bool chassis_enable;
};

bool DecodeDr16Frame(const uint8_t *data, size_t len, Dr16Frame *out);

}  // namespace protocols

#endif /* WBR_CONTROL_PROTOCOLS_REMOTE_INPUT_DR16_PROTOCOL_H_ */
