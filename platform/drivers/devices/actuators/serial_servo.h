/* SPDX-License-Identifier: Apache-2.0 */

#ifndef WBR_CONTROL_PLATFORM_SERIAL_SERVO_H_
#define WBR_CONTROL_PLATFORM_SERIAL_SERVO_H_

#include <stdint.h>

namespace platform {

int InitializeSerialServo();
int MoveSerialServoToAngle(uint8_t id, float degrees, uint16_t time_ms);
int SetSerialServoSpeed(uint8_t id, int16_t speed);
int StopSerialServo(uint8_t id);
int ReadSerialServoId(uint8_t query_id, uint8_t *out_id,
		      uint32_t timeout_ms);

}  // namespace platform

#endif /* WBR_CONTROL_PLATFORM_SERIAL_SERVO_H_ */
