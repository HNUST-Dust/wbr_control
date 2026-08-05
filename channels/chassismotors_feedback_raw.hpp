/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <cstdint>
#include <channels/comm/seqlock_value.hpp>

struct ChassisMotorFeedbackRawFrame {
	// Legacy 1 ms kernel-tick timestamp retained for shadow comparison.
	uint64_t timestamp_us;
	// High-resolution timestamp from the same cycle counter used by chassis.
	uint64_t precise_timestamp_us;
	bool valid;
	uint8_t data[8];

	bool IsFresh(uint64_t now_us, uint64_t timeout_us) const
	{
		return valid && timestamp_us <= now_us && now_us - timestamp_us <= timeout_us;
	}

	bool IsPreciselyFresh(uint64_t now_us, uint64_t timeout_us) const
	{
		return valid && precise_timestamp_us <= now_us &&
		       now_us - precise_timestamp_us <= timeout_us;
	}
};

extern SeqlockValue<ChassisMotorFeedbackRawFrame> left_wheel_feedback_raw;
extern SeqlockValue<ChassisMotorFeedbackRawFrame> right_wheel_feedback_raw;
extern SeqlockValue<ChassisMotorFeedbackRawFrame> left_b_motor_feedback_raw;
extern SeqlockValue<ChassisMotorFeedbackRawFrame> left_d_motor_feedback_raw;
extern SeqlockValue<ChassisMotorFeedbackRawFrame> right_b_motor_feedback_raw;
extern SeqlockValue<ChassisMotorFeedbackRawFrame> right_d_motor_feedback_raw;
