/* SPDX-License-Identifier: Apache-2.0 */

#include "mpc_link.h"

#include <errno.h>
#include <string.h>

namespace protocols {
namespace {

constexpr uint16_t kCrcInitial = 0xffffU;
constexpr uint16_t kCrcReflectedPolynomial = 0x8408U;

void WriteLe16(uint8_t *out, uint16_t value)
{
	out[0] = static_cast<uint8_t>(value);
	out[1] = static_cast<uint8_t>(value >> 8U);
}

void WriteLe32(uint8_t *out, uint32_t value)
{
	out[0] = static_cast<uint8_t>(value);
	out[1] = static_cast<uint8_t>(value >> 8U);
	out[2] = static_cast<uint8_t>(value >> 16U);
	out[3] = static_cast<uint8_t>(value >> 24U);
}

void WriteLeFloat(uint8_t *out, float value)
{
	uint32_t raw = 0U;
	memcpy(&raw, &value, sizeof(raw));
	WriteLe32(out, raw);
}

uint16_t ReadLe16(const uint8_t *in)
{
	return static_cast<uint16_t>(in[0]) |
	       static_cast<uint16_t>(static_cast<uint16_t>(in[1]) << 8U);
}

uint32_t ReadLe32(const uint8_t *in)
{
	return static_cast<uint32_t>(in[0]) |
	       (static_cast<uint32_t>(in[1]) << 8U) |
	       (static_cast<uint32_t>(in[2]) << 16U) |
	       (static_cast<uint32_t>(in[3]) << 24U);
}

float ReadLeFloat(const uint8_t *in)
{
	const uint32_t raw = ReadLe32(in);
	float value = 0.0F;
	memcpy(&value, &raw, sizeof(value));
	return value;
}

void EncodeHeader(uint8_t *out, uint8_t type, uint8_t payload_size,
		  uint32_t sequence, uint32_t uptime_ms)
{
	out[0] = kMpcLinkMagic0;
	out[1] = kMpcLinkMagic1;
	out[2] = kMpcLinkVersion;
	out[3] = type;
	out[4] = payload_size;
	out[5] = 0U;
	WriteLe32(&out[6], sequence);
	WriteLe32(&out[10], uptime_ms);
}

int ValidateFrame(const uint8_t *frame, size_t frame_len, uint8_t expected_type,
		  uint8_t expected_payload_size)
{
	if (frame == nullptr) {
		return -EINVAL;
	}
	const size_t expected_size =
		kMpcLinkHeaderSize + expected_payload_size + kMpcLinkCrcSize;
	if (frame_len != expected_size) {
		return -EMSGSIZE;
	}
	if (frame[0] != kMpcLinkMagic0 || frame[1] != kMpcLinkMagic1 ||
	    frame[2] != kMpcLinkVersion || frame[3] != expected_type ||
	    frame[4] != expected_payload_size || frame[5] != 0U) {
		return -EBADMSG;
	}
	const uint16_t expected_crc = CalculateMpcLinkCrc16(frame, frame_len - 2U);
	return ReadLe16(&frame[frame_len - 2U]) == expected_crc ? 0 : -EBADMSG;
}

} // namespace

uint16_t CalculateMpcLinkCrc16(const uint8_t *data, size_t size)
{
	if (data == nullptr && size != 0U) {
		return 0U;
	}
	uint16_t crc = kCrcInitial;
	for (size_t i = 0U; i < size; ++i) {
		crc ^= static_cast<uint16_t>(data[i]);
		for (uint8_t bit = 0U; bit < 8U; ++bit) {
			crc = (crc & 1U) != 0U
				      ? static_cast<uint16_t>((crc >> 1U) ^ kCrcReflectedPolynomial)
				      : static_cast<uint16_t>(crc >> 1U);
		}
	}
	return crc;
}

int EncodeMpcLinkState(const MpcLinkState *state, uint8_t *out,
		       size_t capacity, size_t *out_len)
{
	if (state == nullptr || out == nullptr || out_len == nullptr) {
		return -EINVAL;
	}
	if (capacity < kMpcLinkStateFrameSize) {
		return -ENOSPC;
	}
	EncodeHeader(out, kMpcLinkStateMessageType, kMpcLinkStatePayloadSize,
		     state->sequence, state->sender_uptime_ms);
	const float values[] = {
		state->roll_rad, state->roll_rate_rad_s,
		state->body_height_m, state->body_height_rate_m_s,
		state->left_leg_length_m, state->left_leg_rate_m_s,
		state->right_leg_length_m, state->right_leg_rate_m_s,
		state->left_support_force_n, state->right_support_force_n,
	};
	for (size_t i = 0U; i < 10U; ++i) {
		WriteLeFloat(&out[kMpcLinkHeaderSize + i * 4U], values[i]);
	}
	const size_t flags_offset = kMpcLinkHeaderSize + 40U;
	out[flags_offset] = state->control_state;
	out[flags_offset + 1U] = state->valid_flags;
	out[flags_offset + 2U] = state->contact_flags;
	out[flags_offset + 3U] = 0U;
	WriteLe16(&out[kMpcLinkStateFrameSize - 2U],
		  CalculateMpcLinkCrc16(out, kMpcLinkStateFrameSize - 2U));
	*out_len = kMpcLinkStateFrameSize;
	return 0;
}

int DecodeMpcLinkState(const uint8_t *frame, size_t frame_len, MpcLinkState *out)
{
	if (out == nullptr) {
		return -EINVAL;
	}
	const int rc = ValidateFrame(frame, frame_len, kMpcLinkStateMessageType,
				     kMpcLinkStatePayloadSize);
	if (rc != 0) {
		return rc;
	}
	out->sequence = ReadLe32(&frame[6]);
	out->sender_uptime_ms = ReadLe32(&frame[10]);
	float *values[] = {
		&out->roll_rad, &out->roll_rate_rad_s,
		&out->body_height_m, &out->body_height_rate_m_s,
		&out->left_leg_length_m, &out->left_leg_rate_m_s,
		&out->right_leg_length_m, &out->right_leg_rate_m_s,
		&out->left_support_force_n, &out->right_support_force_n,
	};
	for (size_t i = 0U; i < 10U; ++i) {
		*values[i] = ReadLeFloat(&frame[kMpcLinkHeaderSize + i * 4U]);
	}
	const size_t flags_offset = kMpcLinkHeaderSize + 40U;
	out->control_state = frame[flags_offset];
	out->valid_flags = frame[flags_offset + 1U];
	out->contact_flags = frame[flags_offset + 2U];
	return 0;
}

int EncodeMpcLinkCommand(const MpcLinkCommand *command, uint8_t *out,
			 size_t capacity, size_t *out_len)
{
	if (command == nullptr || out == nullptr || out_len == nullptr) {
		return -EINVAL;
	}
	if (capacity < kMpcLinkCommandFrameSize) {
		return -ENOSPC;
	}
	EncodeHeader(out, kMpcLinkCommandMessageType, kMpcLinkCommandPayloadSize,
		     command->sequence, command->sender_uptime_ms);
	const size_t payload = kMpcLinkHeaderSize;
	WriteLe32(&out[payload], command->source_state_sequence);
	WriteLeFloat(&out[payload + 4U], command->left_support_force_n);
	WriteLeFloat(&out[payload + 8U], command->right_support_force_n);
	out[payload + 12U] = static_cast<uint8_t>(command->mode);
	out[payload + 13U] = static_cast<uint8_t>(command->contact_mode);
	out[payload + 14U] = command->solver_flags;
	out[payload + 15U] = 0U;
	WriteLe16(&out[payload + 16U], command->solver_iterations);
	WriteLe16(&out[payload + 18U], command->valid_for_ms);
	WriteLe32(&out[payload + 20U], command->solve_time_us);
	WriteLe16(&out[kMpcLinkCommandFrameSize - 2U],
		  CalculateMpcLinkCrc16(out, kMpcLinkCommandFrameSize - 2U));
	*out_len = kMpcLinkCommandFrameSize;
	return 0;
}

int DecodeMpcLinkCommand(const uint8_t *frame, size_t frame_len, MpcLinkCommand *out)
{
	if (out == nullptr) {
		return -EINVAL;
	}
	const int rc = ValidateFrame(frame, frame_len, kMpcLinkCommandMessageType,
				     kMpcLinkCommandPayloadSize);
	if (rc != 0) {
		return rc;
	}
	out->sequence = ReadLe32(&frame[6]);
	out->sender_uptime_ms = ReadLe32(&frame[10]);
	const size_t payload = kMpcLinkHeaderSize;
	out->source_state_sequence = ReadLe32(&frame[payload]);
	out->left_support_force_n = ReadLeFloat(&frame[payload + 4U]);
	out->right_support_force_n = ReadLeFloat(&frame[payload + 8U]);
	out->mode = static_cast<MpcLinkCommandMode>(frame[payload + 12U]);
	out->contact_mode = static_cast<MpcLinkContactMode>(frame[payload + 13U]);
	out->solver_flags = frame[payload + 14U];
	out->solver_iterations = ReadLe16(&frame[payload + 16U]);
	out->valid_for_ms = ReadLe16(&frame[payload + 18U]);
	out->solve_time_us = ReadLe32(&frame[payload + 20U]);
	return 0;
}

} // namespace protocols
