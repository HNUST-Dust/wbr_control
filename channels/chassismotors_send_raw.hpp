/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <cstdint>
#include <channels/comm/seqlock_value.hpp>

struct ChassisMotorSendRawFrame {
	uint8_t bus;
	uint16_t can_id;
	uint8_t dlc;
	uint8_t data[8];
};

extern SeqlockValue<ChassisMotorSendRawFrame> left_wheel_send_raw;
extern SeqlockValue<ChassisMotorSendRawFrame> right_wheel_send_raw;
extern SeqlockValue<ChassisMotorSendRawFrame> left_b_motor_send_raw;
extern SeqlockValue<ChassisMotorSendRawFrame> left_d_motor_send_raw;
extern SeqlockValue<ChassisMotorSendRawFrame> right_b_motor_send_raw;
extern SeqlockValue<ChassisMotorSendRawFrame> right_d_motor_send_raw;
