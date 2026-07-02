/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <protocols/imu/hi91_protocol.h>

#include <string.h>

namespace protocols::imu::hi91 {

namespace {

uint16_t ReadLe16(const uint8_t *data)
{
	return static_cast<uint16_t>(data[0]) |
	       (static_cast<uint16_t>(data[1]) << 8U);
}

uint32_t ReadLe32(const uint8_t *data)
{
	return static_cast<uint32_t>(data[0]) |
	       (static_cast<uint32_t>(data[1]) << 8U) |
	       (static_cast<uint32_t>(data[2]) << 16U) |
	       (static_cast<uint32_t>(data[3]) << 24U);
}

float ReadLeFloat(const uint8_t *data)
{
	const uint32_t raw = ReadLe32(data);
	float value = 0.0F;
	memcpy(&value, &raw, sizeof(value));
	return value;
}

void DecodeFloat3(const uint8_t *data, float value[3])
{
	value[0] = ReadLeFloat(&data[0]);
	value[1] = ReadLeFloat(&data[4]);
	value[2] = ReadLeFloat(&data[8]);
}

uint16_t CalculateFrameCrc(uint16_t payload_length, const uint8_t *payload)
{
	uint8_t header[4] = {
		kFrameSof0,
		kFrameSof1,
		static_cast<uint8_t>(payload_length & 0xFFU),
		static_cast<uint8_t>((payload_length >> 8U) & 0xFFU),
	};

	uint16_t crc = Crc16CcittFalse(header, sizeof(header));
	for (uint16_t i = 0U; i < payload_length; ++i) {
		crc ^= static_cast<uint16_t>(payload[i]) << 8U;
		for (uint8_t bit = 0U; bit < 8U; ++bit) {
			if ((crc & 0x8000U) != 0U) {
				crc = static_cast<uint16_t>((crc << 1U) ^ 0x1021U);
			} else {
				crc = static_cast<uint16_t>(crc << 1U);
			}
		}
	}
	return crc;
}

}  // namespace

Parser::Parser()
{
	Reset();
	strict_crc_ = false;
}

void Parser::Reset()
{
	state_ = State::kSof0;
	payload_length_ = 0U;
	expected_crc_ = 0U;
	payload_index_ = 0U;
}

void Parser::SetStrictCrc(bool strict_crc)
{
	strict_crc_ = strict_crc;
}

ParseResult Parser::Feed(uint8_t byte, Sample *sample)
{
	switch (state_) {
	case State::kSof0:
		if (byte == kFrameSof0) {
			state_ = State::kSof1;
		}
		break;
	case State::kSof1:
		if (byte == kFrameSof1) {
			state_ = State::kLen0;
		} else {
			state_ = (byte == kFrameSof0) ? State::kSof1 : State::kSof0;
		}
		break;
	case State::kLen0:
		payload_length_ = byte;
		state_ = State::kLen1;
		break;
	case State::kLen1:
		payload_length_ |= static_cast<uint16_t>(byte) << 8U;
		if ((payload_length_ == 0U) || (payload_length_ > kMaxPayloadLength)) {
			Reset();
			return ParseResult::kInvalidLength;
		}
		state_ = State::kCrc0;
		break;
	case State::kCrc0:
		expected_crc_ = byte;
		state_ = State::kCrc1;
		break;
	case State::kCrc1:
		expected_crc_ |= static_cast<uint16_t>(byte) << 8U;
		payload_index_ = 0U;
		state_ = State::kPayload;
		break;
	case State::kPayload:
		payload_[payload_index_++] = byte;
		if (payload_index_ >= payload_length_) {
			return FinishFrame(sample);
		}
		break;
	}

	return ParseResult::kNone;
}

ParseResult Parser::FinishFrame(Sample *sample)
{
	const uint16_t payload_length = payload_length_;
	const uint16_t expected_crc = expected_crc_;
	ParseResult result = ParseResult::kFrame;

	if (strict_crc_ && (CalculateFrameCrc(payload_length, payload_) != expected_crc)) {
		result = ParseResult::kCrcError;
	} else if ((payload_length < kDataLength) || (payload_[0] != kDataTag)) {
		result = ParseResult::kUnsupportedFrame;
	} else if (sample != nullptr) {
		sample->main_status = ReadLe16(&payload_[1]);
		sample->temperature_c = static_cast<int8_t>(payload_[3]);
		sample->air_pressure = ReadLeFloat(&payload_[4]);
		sample->system_time_ms = ReadLe32(&payload_[8]);
		DecodeFloat3(&payload_[12], sample->accel_g);
		DecodeFloat3(&payload_[24], sample->gyro_dps);
		DecodeFloat3(&payload_[36], sample->mag_ut);
		sample->roll_deg = ReadLeFloat(&payload_[48]);
		sample->pitch_deg = ReadLeFloat(&payload_[52]);
		sample->yaw_deg = ReadLeFloat(&payload_[56]);
		sample->quat[0] = ReadLeFloat(&payload_[60]);
		sample->quat[1] = ReadLeFloat(&payload_[64]);
		sample->quat[2] = ReadLeFloat(&payload_[68]);
		sample->quat[3] = ReadLeFloat(&payload_[72]);
	}

	Reset();
	return result;
}

uint16_t Crc16CcittFalse(const uint8_t *data, size_t size)
{
	uint16_t crc = 0xFFFFU;
	for (size_t i = 0U; i < size; ++i) {
		crc ^= static_cast<uint16_t>(data[i]) << 8U;
		for (uint8_t bit = 0U; bit < 8U; ++bit) {
			if ((crc & 0x8000U) != 0U) {
				crc = static_cast<uint16_t>((crc << 1U) ^ 0x1021U);
			} else {
				crc = static_cast<uint16_t>(crc << 1U);
			}
		}
	}
	return crc;
}

}  // namespace protocols::imu::hi91
