/* SPDX-License-Identifier: Apache-2.0 */

#ifndef WBR_CONTROL_PROTOCOLS_REMOTE_INPUT_VT03_PROTOCOL_H_
#define WBR_CONTROL_PROTOCOLS_REMOTE_INPUT_VT03_PROTOCOL_H_

#include <stddef.h>
#include <stdint.h>

namespace protocols {

constexpr size_t kVt03RemoteFrameLength = 21;
constexpr size_t kVt03CustomFrameLength = 39;

struct Vt03Frame {
	float right_x;
	float right_y;
	float left_x;
	float left_y;
	float wheel;
	uint8_t mode_switch;
	bool chassis_enable;
};

struct Vt03CustomFrame {
	float joystick_x;
	float joystick_y;
	float joystick_z;
	bool chassis_enable;
};

bool DecodeVt03RemoteFrame(const uint8_t *data, size_t len, Vt03Frame *out);
bool DecodeVt03CustomFrame(const uint8_t *data, size_t len, Vt03CustomFrame *out);

}  // namespace protocols

#endif /* WBR_CONTROL_PROTOCOLS_REMOTE_INPUT_VT03_PROTOCOL_H_ */
