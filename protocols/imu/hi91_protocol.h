/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace protocols::imu::hi91 {

constexpr uint8_t kFrameSof0 = 0x5AU;
constexpr uint8_t kFrameSof1 = 0xA5U;
constexpr uint8_t kDataTag = 0x91U;
constexpr uint16_t kDataLength = 76U;
constexpr uint16_t kMaxPayloadLength = 256U;

struct Sample {
	uint16_t main_status;
	int8_t temperature_c;
	float air_pressure;
	uint32_t system_time_ms;
	float accel_g[3];
	float gyro_dps[3];
	float mag_ut[3];
	float roll_deg;
	float pitch_deg;
	float yaw_deg;
	float quat[4];
};

enum class ParseResult {
	kNone,
	kFrame,
	kCrcError,
	kUnsupportedFrame,
	kInvalidLength,
};

class Parser {
public:
	Parser();

	void Reset();
	void SetStrictCrc(bool strict_crc);
	ParseResult Feed(uint8_t byte, Sample *sample);

private:
	enum class State {
		kSof0,
		kSof1,
		kLen0,
		kLen1,
		kCrc0,
		kCrc1,
		kPayload,
	};

	ParseResult FinishFrame(Sample *sample);

	State state_;
	bool strict_crc_;
	uint16_t payload_length_;
	uint16_t expected_crc_;
	uint16_t payload_index_;
	uint8_t payload_[kMaxPayloadLength];
};

uint16_t Crc16CcittFalse(const uint8_t *data, size_t size);

}  // namespace protocols::imu::hi91
