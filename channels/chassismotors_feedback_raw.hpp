/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <cstdint>
#include <channels/comm/seqlock_value.hpp>

struct ChassisMotorFeedbackRawFrame {
	uint64_t timestamp_us;
	bool valid;
	uint8_t data[8];

	bool IsFresh(uint64_t now_us, uint64_t timeout_us) const
	{
		return valid && timestamp_us <= now_us && now_us - timestamp_us <= timeout_us;
	}
};

extern SeqlockValue<ChassisMotorFeedbackRawFrame> left_wheel_feedback_raw;
extern SeqlockValue<ChassisMotorFeedbackRawFrame> right_wheel_feedback_raw;
extern SeqlockValue<ChassisMotorFeedbackRawFrame> left_B_motor_feedback_raw;
extern SeqlockValue<ChassisMotorFeedbackRawFrame> left_D_motor_feedback_raw;
extern SeqlockValue<ChassisMotorFeedbackRawFrame> right_B_motor_feedback_raw;
extern SeqlockValue<ChassisMotorFeedbackRawFrame> right_D_motor_feedback_raw;
