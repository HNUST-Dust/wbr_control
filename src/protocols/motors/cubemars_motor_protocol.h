/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef WBR_CONTROL_PROTOCOLS_MOTORS_CUBEMARS_MOTOR_PROTOCOL_H_
#define WBR_CONTROL_PROTOCOLS_MOTORS_CUBEMARS_MOTOR_PROTOCOL_H_

#include <stdint.h>

namespace protocols {

struct CubemarsFeedback {
	uint8_t id;
	uint16_t position_raw;
	uint16_t velocity_raw;
	uint16_t torque_raw;
};

struct CubemarsMitCommand {
	float position;
	float velocity;
	float kp;
	float kd;
	float torque;
};

struct CubemarsMitRange {
	float p_min;
	float p_max;
	float v_min;
	float v_max;
	float kp_min;
	float kp_max;
	float kd_min;
	float kd_max;
	float t_min;
	float t_max;
};

int DecodeCubemarsFeedback(const uint8_t *data, uint8_t dlc, CubemarsFeedback *out);
int GetCubemarsEnterFrame(uint8_t out[8]);
int GetCubemarsExitFrame(uint8_t out[8]);
int GetCubemarsSaveZeroFrame(uint8_t out[8]);
int PackCubemarsMitCommand(const CubemarsMitCommand *cmd, const CubemarsMitRange *range,
			   uint8_t out[8]);

}  // namespace protocols

#endif /* WBR_CONTROL_PROTOCOLS_MOTORS_CUBEMARS_MOTOR_PROTOCOL_H_ */
