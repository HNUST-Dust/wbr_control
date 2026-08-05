/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef WBR_CONTROL_PROTOCOLS_MOTORS_DM_MOTOR_PROTOCOL_H_
#define WBR_CONTROL_PROTOCOLS_MOTORS_DM_MOTOR_PROTOCOL_H_

#include <stdint.h>

namespace protocols {

struct DmMotorFeedback1To4 {
	uint16_t encoder;
	int16_t omega_x100;
	int16_t current_ma;
	uint8_t rotor_temperature;
	uint8_t mos_temperature;
};

struct DmMotorRawDataNormal
{
    uint8_t can_id : 4;
    uint8_t control_status_enum : 4;
    uint16_t angle_reverse;
    uint8_t omega_11_4;
    uint8_t omega_3_0_torque_11_8;
    uint8_t torque_7_0;
    uint8_t mos_temperature;
    uint8_t rotor_temperature;
} __attribute__((packed));

struct DmMotorFeedbackNormal
{
    uint8_t control_status;
    uint16_t angle;
    uint16_t omega;
    uint16_t torque;
    float mos_temperature;
    float rotor_temperature;
};

enum class DmControlCommand {
	kClearError,
	kEnter,
	kExit,
	kSaveZero,
};

struct DmMitCommand {
	float position;
	float velocity;
	float kp;
	float kd;
	float torque;
};

struct DmMitRange {
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

int DecodeDmFeedback1To4(const uint8_t *data, uint8_t dlc, DmMotorFeedback1To4 *out);
int DecodeDmFeedbackNormal(const uint8_t *data, uint8_t dlc, DmMotorFeedbackNormal *out);
float DecodeDmMitValue(uint16_t value, float minimum, float maximum, uint8_t bits);
float DmFeedbackPosition(const DmMotorFeedbackNormal &feedback, const DmMitRange &range);
float DmFeedbackVelocity(const DmMotorFeedbackNormal &feedback, const DmMitRange &range);
float DmFeedbackTorque(const DmMotorFeedbackNormal &feedback, const DmMitRange &range);
int GetDmControlCommandFrame(DmControlCommand cmd, uint8_t out[8]);
int PackDmMitCommand(const DmMitCommand *cmd, const DmMitRange *range, uint8_t out[8]);
int PackDm1To4CurrentFrame(uint16_t motor_can_id, int16_t current_ma,
			  uint8_t frame_payload[8]);

}  // namespace protocols

#endif /* WBR_CONTROL_PROTOCOLS_MOTORS_DM_MOTOR_PROTOCOL_H_ */
