/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>

#include <cstdint>

#include <zephyr/ztest.h>

#include <protocols/imu/hi91_protocol.h>
#include <protocols/motors/dji_motor_protocol.h>
#include <protocols/pc_link/pc_link.h>

ZTEST(protocols, hi91_rejects_invalid_frames)
{
	protocols::Hi91Sample sample{};
	const uint8_t short_frame[] = {protocols::kHi91FrameSof0};
	const uint8_t wrong_sof[] = {0x00U, 0x00U, 0x01U, 0x00U, 0x00U, 0x00U, 0x91U};

	zassert_equal(protocols::DecodeHi91Frame(nullptr, 0U, false, &sample), -EINVAL);
	zassert_equal(protocols::DecodeHi91Frame(short_frame, sizeof(short_frame), false, &sample),
		      -EINVAL);
	zassert_equal(protocols::DecodeHi91Frame(wrong_sof, sizeof(wrong_sof), false, &sample),
		      -EBADMSG);
}

ZTEST(protocols, dji_current_is_written_to_selected_big_endian_slot)
{
	uint8_t payload[8] = {};

	zassert_ok(protocols::WriteDjiCurrentCommandToSlot(0x203U, -1234, payload));
	zassert_equal(payload[0], 0U);
	zassert_equal(payload[1], 0U);
	zassert_equal(payload[2], 0U);
	zassert_equal(payload[3], 0U);
	zassert_equal(payload[4], 0xFBU);
	zassert_equal(payload[5], 0x2EU);
	zassert_equal(payload[6], 0U);
	zassert_equal(payload[7], 0U);
	zassert_equal(protocols::WriteDjiCurrentCommandToSlot(0x205U, 1, payload), -EINVAL);
}

ZTEST(protocols, pc_link_round_trip_and_crc_rejection)
{
	protocols::PCRecvAutoAimData input{};
	input.mode = 2U;
	input.yaw.yaw_ang = 1.25F;
	input.yaw.yaw_vel = -0.5F;
	input.yaw.yaw_acc = 0.125F;
	input.pitch.pitch_ang = -0.75F;
	input.pitch.pitch_vel = 0.25F;
	input.pitch.pitch_acc = -0.0625F;

	uint8_t packet[protocols::kPcCommRecvPacketSize] = {};
	size_t packet_size = 0U;
	zassert_ok(protocols::EncodePcCommRecv(&input, packet, sizeof(packet), &packet_size));
	zassert_equal(packet_size, sizeof(packet));

	protocols::PCRecvAutoAimData decoded{};
	zassert_ok(protocols::DecodePcCommRecv(packet, sizeof(packet), &decoded));
	zassert_equal(decoded.mode, input.mode);
	zassert_within(decoded.yaw.yaw_ang, input.yaw.yaw_ang, 1.0e-6F);
	zassert_within(decoded.pitch.pitch_acc, input.pitch.pitch_acc, 1.0e-6F);

	packet[10] ^= 0x01U;
	zassert_equal(protocols::DecodePcCommRecv(packet, sizeof(packet), &decoded),
		      -EBADMSG);
}

ZTEST_SUITE(protocols, nullptr, nullptr, nullptr, nullptr, nullptr);
