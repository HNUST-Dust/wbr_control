/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>

#include <cstdint>

#include <zephyr/ztest.h>

#include <protocols/imu/hi91_protocol.h>
#include <protocols/motors/dji_motor_protocol.h>
#include <protocols/mpc_link/mpc_link.h>
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

ZTEST(protocols, mpc_link_state_round_trip_and_crc_rejection)
{
	protocols::MpcLinkState input{};
	input.sequence = 0x12345678U;
	input.sender_uptime_ms = 9876U;
	input.roll_rad = -0.125F;
	input.roll_rate_rad_s = 0.75F;
	input.body_height_m = 0.24F;
	input.body_height_rate_m_s = -0.31F;
	input.left_leg_length_m = 0.23F;
	input.left_leg_rate_m_s = 0.12F;
	input.right_leg_length_m = 0.25F;
	input.right_leg_rate_m_s = -0.14F;
	input.left_support_force_n = 122.5F;
	input.right_support_force_n = 131.25F;
	input.control_state = 3U;
	input.valid_flags = protocols::kMpcStateImuFresh |
			    protocols::kMpcStateSupportForceValid;
	input.contact_flags = protocols::kMpcContactLeft;

	uint8_t frame[protocols::kMpcLinkStateFrameSize]{};
	size_t frame_len = 0U;
	zassert_ok(protocols::EncodeMpcLinkState(&input, frame, sizeof(frame), &frame_len));
	zassert_equal(frame_len, sizeof(frame));

	protocols::MpcLinkState decoded{};
	zassert_ok(protocols::DecodeMpcLinkState(frame, frame_len, &decoded));
	zassert_equal(decoded.sequence, input.sequence);
	zassert_within(decoded.roll_rad, input.roll_rad, 1.0e-6F);
	zassert_within(decoded.right_support_force_n, input.right_support_force_n, 1.0e-6F);
	zassert_equal(decoded.valid_flags, input.valid_flags);
	frame[30] ^= 0x80U;
	zassert_equal(protocols::DecodeMpcLinkState(frame, frame_len, &decoded), -EBADMSG);
}

ZTEST(protocols, mpc_link_command_round_trip)
{
	protocols::MpcLinkCommand input{};
	input.sequence = 42U;
	input.sender_uptime_ms = 1100U;
	input.source_state_sequence = 41U;
	input.left_support_force_n = 120.0F;
	input.right_support_force_n = 135.0F;
	input.mode = protocols::MpcLinkCommandMode::kShadow;
	input.contact_mode = protocols::MpcLinkContactMode::kBoth;
	input.solver_flags = protocols::kMpcSolverConverged |
			     protocols::kMpcSolverInputFinite;
	input.solver_iterations = 12U;
	input.valid_for_ms = 30U;
	input.solve_time_us = 420U;

	uint8_t frame[protocols::kMpcLinkCommandFrameSize]{};
	size_t frame_len = 0U;
	zassert_ok(protocols::EncodeMpcLinkCommand(&input, frame, sizeof(frame), &frame_len));
	protocols::MpcLinkCommand decoded{};
	zassert_ok(protocols::DecodeMpcLinkCommand(frame, frame_len, &decoded));
	zassert_equal(decoded.source_state_sequence, input.source_state_sequence);
	zassert_within(decoded.left_support_force_n, input.left_support_force_n, 1.0e-6F);
	zassert_equal(static_cast<uint8_t>(decoded.mode), static_cast<uint8_t>(input.mode));
	zassert_equal(decoded.valid_for_ms, input.valid_for_ms);
}

ZTEST_SUITE(protocols, nullptr, nullptr, nullptr, nullptr, nullptr);
