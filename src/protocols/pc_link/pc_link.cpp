/* SPDX-License-Identifier: Apache-2.0 */

#include <protocols/pc_link/pc_link.h>

#include <errno.h>
#include <string.h>

namespace protocols {

namespace {

void WriteLe16(uint8_t *out, uint16_t value)
{
	out[0] = static_cast<uint8_t>(value & 0xffU);
	out[1] = static_cast<uint8_t>((value >> 8) & 0xffU);
}

void WriteLe32(uint8_t *out, uint32_t value)
{
	out[0] = static_cast<uint8_t>(value & 0xffU);
	out[1] = static_cast<uint8_t>((value >> 8) & 0xffU);
	out[2] = static_cast<uint8_t>((value >> 16) & 0xffU);
	out[3] = static_cast<uint8_t>((value >> 24) & 0xffU);
}

void WriteLeFloat(uint8_t *out, float value)
{
	uint32_t raw = 0U;

	memcpy(&raw, &value, sizeof(raw));
	WriteLe32(out, raw);
}

uint16_t ReadLe16(const uint8_t *data)
{
	return static_cast<uint16_t>(data[0]) |
	       (static_cast<uint16_t>(data[1]) << 8U);
}

float ReadLeFloat(const uint8_t *data)
{
	const uint32_t raw = static_cast<uint32_t>(data[0]) |
			     (static_cast<uint32_t>(data[1]) << 8U) |
			     (static_cast<uint32_t>(data[2]) << 16U) |
			     (static_cast<uint32_t>(data[3]) << 24U);
	float value = 0.0F;

	memcpy(&value, &raw, sizeof(value));
	return value;
}

uint16_t Crc16Update(uint16_t crc, const uint8_t *data, size_t size)
{
	for (size_t i = 0U; i < size; ++i) {
		crc ^= static_cast<uint16_t>(data[i]) << 8U;
		for (uint8_t bit = 0U; bit < 8U; ++bit) {
			if ((crc & 0x8000U) != 0U) {
				crc = static_cast<uint16_t>((crc << 1U) ^ kPcCommCrc16Poly);
			} else {
				crc = static_cast<uint16_t>(crc << 1U);
			}
		}
	}
	return crc;
}

/* CRC over all packet bytes except the trailing crc16 field. */
uint16_t CalculatePacketCrc(const uint8_t *packet, size_t crc16_offset)
{
	return Crc16Update(kPcCommCrc16Init, packet, crc16_offset);
}

}  // namespace

int EncodePcCommSend(const PCSendAutoAimData *data,
		uint8_t *out,
		size_t out_capacity,
		size_t *out_len)
{
	if ((data == nullptr) || (out == nullptr) || (out_len == nullptr)) {
		return -EINVAL;
	}

	if (out_capacity < kPcCommSendPacketSize) {
		return -ENOSPC;
	}

	out[0] = data->head[0];
	out[1] = data->head[1];
	out[2] = data->mode;
	WriteLeFloat(&out[3], data->q[0]);
	WriteLeFloat(&out[7], data->q[1]);
	WriteLeFloat(&out[11], data->q[2]);
	WriteLeFloat(&out[15], data->q[3]);
	WriteLeFloat(&out[19], data->yaw.yaw_ang);
	WriteLeFloat(&out[23], data->yaw.yaw_vel);
	WriteLeFloat(&out[27], data->pitch.pitch_ang);
	WriteLeFloat(&out[31], data->pitch.pitch_vel);
	WriteLeFloat(&out[35], data->bullet.bullet_speed);
	WriteLe16(&out[39], data->bullet.bullet_count);
	WriteLe16(&out[41], CalculatePacketCrc(out, kPcCommSendPacketSize - 2U));

	*out_len = kPcCommSendPacketSize;
	return 0;
}

int DecodePcCommSend(const uint8_t *packet,
		size_t packet_len,
		PCSendAutoAimData *out)
{
	if ((packet == nullptr) || (out == nullptr)) {
		return -EINVAL;
	}

	if (packet_len != kPcCommSendPacketSize) {
		return -EMSGSIZE;
	}

	if ((packet[0] != 'S') || (packet[1] != 'P')) {
		return -EBADMSG;
	}

	const uint16_t stored_crc = ReadLe16(&packet[kPcCommSendPacketSize - 2U]);
	if (CalculatePacketCrc(packet, kPcCommSendPacketSize - 2U) != stored_crc) {
		return -EBADMSG;
	}

	out->head[0] = packet[0];
	out->head[1] = packet[1];
	out->mode = packet[2];
	out->q[0] = ReadLeFloat(&packet[3]);
	out->q[1] = ReadLeFloat(&packet[7]);
	out->q[2] = ReadLeFloat(&packet[11]);
	out->q[3] = ReadLeFloat(&packet[15]);
	out->yaw.yaw_ang = ReadLeFloat(&packet[19]);
	out->yaw.yaw_vel = ReadLeFloat(&packet[23]);
	out->pitch.pitch_ang = ReadLeFloat(&packet[27]);
	out->pitch.pitch_vel = ReadLeFloat(&packet[31]);
	out->bullet.bullet_speed = ReadLeFloat(&packet[35]);
	out->bullet.bullet_count = ReadLe16(&packet[39]);
	return 0;
}

int EncodePcCommRecv(const PCRecvAutoAimData *data,
		uint8_t *out,
		size_t out_capacity,
		size_t *out_len)
{
	if ((data == nullptr) || (out == nullptr) || (out_len == nullptr)) {
		return -EINVAL;
	}

	if (out_capacity < kPcCommRecvPacketSize) {
		return -ENOSPC;
	}

	out[0] = data->head[0];
	out[1] = data->head[1];
	out[2] = data->mode;
	WriteLeFloat(&out[3], data->yaw.yaw_ang);
	WriteLeFloat(&out[7], data->yaw.yaw_vel);
	WriteLeFloat(&out[11], data->yaw.yaw_acc);
	WriteLeFloat(&out[15], data->pitch.pitch_ang);
	WriteLeFloat(&out[19], data->pitch.pitch_vel);
	WriteLeFloat(&out[23], data->pitch.pitch_acc);
	WriteLe16(&out[27], CalculatePacketCrc(out, kPcCommRecvPacketSize - 2U));

	*out_len = kPcCommRecvPacketSize;
	return 0;
}

int DecodePcCommRecv(const uint8_t *packet,
		size_t packet_len,
		PCRecvAutoAimData *out)
{
	if ((packet == nullptr) || (out == nullptr)) {
		return -EINVAL;
	}

	if (packet_len != kPcCommRecvPacketSize) {
		return -EMSGSIZE;
	}

	if ((packet[0] != 'S') || (packet[1] != 'P')) {
		return -EBADMSG;
	}

	const uint16_t stored_crc = ReadLe16(&packet[kPcCommRecvPacketSize - 2U]);
	if (CalculatePacketCrc(packet, kPcCommRecvPacketSize - 2U) != stored_crc) {
		return -EBADMSG;
	}

	out->head[0] = packet[0];
	out->head[1] = packet[1];
	out->mode = packet[2];
	out->yaw.yaw_ang = ReadLeFloat(&packet[3]);
	out->yaw.yaw_vel = ReadLeFloat(&packet[7]);
	out->yaw.yaw_acc = ReadLeFloat(&packet[11]);
	out->pitch.pitch_ang = ReadLeFloat(&packet[15]);
	out->pitch.pitch_vel = ReadLeFloat(&packet[19]);
	out->pitch.pitch_acc = ReadLeFloat(&packet[23]);
	return 0;
}

}  // namespace protocols
