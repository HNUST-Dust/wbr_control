/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

// Public HI91 wire-format contract.

#include <stddef.h>
#include <stdint.h>

namespace protocols {

constexpr uint8_t kHi91FrameSof0 = 0x5AU;
constexpr uint8_t kHi91FrameSof1 = 0xA5U;
constexpr uint8_t kHi91DataTag = 0x91U;
constexpr uint16_t kHi91DataLength = 76U;
constexpr uint16_t kHi91MaxPayloadLength = 256U;

struct Hi91Sample {
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

enum class Hi91ParseResult {
	kNone,
	kFrame,
	kCrcError,
	kUnsupportedFrame,
	kInvalidLength,
};

class Hi91Parser {
public:
	Hi91Parser();

	void Reset();
	void SetStrictCrc(bool strict_crc);
	Hi91ParseResult Feed(uint8_t byte, Hi91Sample *sample);

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

	Hi91ParseResult FinishFrame(Hi91Sample *sample);

	State state_;
	bool strict_crc_;
	uint16_t payload_length_;
	uint16_t expected_crc_;
	uint16_t payload_index_;
	uint8_t payload_[kHi91MaxPayloadLength];
};

/* HiPNUC CRC-16/CCITT update with the manual's initial value 0x0000. */
uint16_t Hi91Crc16CcittFalse(const uint8_t *data, size_t size);

}  // namespace protocols
