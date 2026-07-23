/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <stdint.h>

namespace protocols {

struct DjiMotorFeedback {
	uint16_t encoder;
	int16_t omega;
	int16_t current;
	uint8_t temperature;
};

bool IsDjiStandardFeedbackId(uint16_t can_id);
int DecodeDjiFeedback(const uint8_t *data, uint8_t dlc, DjiMotorFeedback *out);

// Pack 4 motor currents for CAN ID 0x200 (0x201~0x204).
int EncodeDjiCurrentFrame0x200(const int16_t current_cmd[4], uint8_t out[8]);
// Pack 4 motor currents for CAN ID 0x1FF.
int EncodeDjiCurrentFrame0x1ff(const int16_t current_cmd[4], uint8_t out[8]);

// Write one motor current command into a 0x200/0x1FF payload slot.
int WriteDjiCurrentCommandToSlot(uint16_t motor_can_id, int16_t current_cmd,
				 uint8_t frame_payload[8]);

}  // namespace protocols
