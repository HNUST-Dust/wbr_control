/* SPDX-License-Identifier: Apache-2.0 */

#ifndef WBR_CONTROL_PROTOCOLS_PC_LINK_PC_LINK_H_
#define WBR_CONTROL_PROTOCOLS_PC_LINK_PC_LINK_H_

#include <stddef.h>
#include <stdint.h>

namespace protocols {

/*
 * CRC-16 parameters used by the vision host.
 *
 * The host processes each byte least-significant bit first with the reflected
 * form of polynomial 0x1021.  It uses init 0xFFFF and no final XOR.  The CRC
 * field itself is serialized little-endian.
 */
constexpr uint16_t kPcCommCrc16ReflectedPoly = 0x8408U;
constexpr uint16_t kPcCommCrc16Init = 0xFFFFU;

/* Packet sizes on the wire (head + fields + crc16). */
constexpr size_t kPcCommSendPacketSize = 43U;
constexpr size_t kPcCommRecvPacketSize = 29U;

#pragma pack(push, 1)

/*
 * Auto-aim state reported by the MCU to the PC.
 * Wire layout: 'S','P' + mode + q[4] + yaw + pitch + bullet + crc16.
 */
struct PCSendAutoAimData
{
	uint8_t head[2] = {'S', 'P'};

	uint8_t mode = 0; /* 0 idle, 1 auto-aim */

	float q[4]; /* attitude quaternion [w, x, y, z] */

	struct
	{
		float yaw_ang; /* yaw axis angle */
		float yaw_vel; /* yaw axis angular velocity */
	} yaw;

	struct
	{
		float pitch_ang; /* pitch axis angle */
		float pitch_vel; /* pitch axis angular velocity */
	} pitch;

	struct
	{
		float bullet_speed;    /* bullet speed */
		uint16_t bullet_count; /* accumulated bullet count */
	} bullet;

	uint16_t crc16; /* checksum */
};

/*
 * Auto-aim command received from the PC.
 * Wire layout: 'S','P' + mode + yaw + pitch + crc16.
 */
struct PCRecvAutoAimData
{
	uint8_t head[2] = {'S', 'P'};
	uint8_t mode = 0; /* 0 idle, 1 aim without firing, 2 aim and fire */

	struct
	{
		float yaw_ang; /* yaw axis angle */
		float yaw_vel; /* yaw axis angular velocity */
		float yaw_acc; /* yaw axis angular acceleration */
	} yaw;

	struct
	{
		float pitch_ang; /* pitch axis angle */
		float pitch_vel; /* pitch axis angular velocity */
		float pitch_acc; /* pitch axis angular acceleration */
	} pitch;

	uint16_t crc16; /* checksum */
};

#pragma pack(pop)

static_assert(sizeof(PCSendAutoAimData) == kPcCommSendPacketSize,
	      "PCSendAutoAimData wire layout mismatch");
static_assert(sizeof(PCRecvAutoAimData) == kPcCommRecvPacketSize,
	      "PCRecvAutoAimData wire layout mismatch");

/* Serialize the auto-aim packets (head + fields + crc16), little-endian. */
int EncodePcCommSend(const PCSendAutoAimData *data,
		uint8_t *out,
		size_t out_capacity,
		size_t *out_len);

int EncodePcCommRecv(const PCRecvAutoAimData *data,
		uint8_t *out,
		size_t out_capacity,
		size_t *out_len);

/* Parse the auto-aim packets, verifying the crc16. */
int DecodePcCommSend(const uint8_t *packet,
		size_t packet_len,
		PCSendAutoAimData *out);

int DecodePcCommRecv(const uint8_t *packet,
		size_t packet_len,
		PCRecvAutoAimData *out);

}  // namespace protocols

#endif /* WBR_CONTROL_PROTOCOLS_PC_LINK_PC_LINK_H_ */
