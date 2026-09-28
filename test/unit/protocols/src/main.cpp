/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>

#include <cstdint>

#include <zephyr/ztest.h>

#include <protocols/imu/hi91_protocol.h>
#include <protocols/motors/dji_motor_protocol.h>
#include <protocols/ethercat/ethercat_process_data.h>
#include <chassis_controller/communication/ethercat_slave_core.h>
#include <protocols/mpc_link/mpc_link.h>
#include <protocols/pc_link/pc_link.h>
#include <chassis_controller/modules/chassis/ethercat_chassis_port.h>

ZTEST(protocols, test_hi91_rejects_invalid_frames)
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

ZTEST(protocols, test_dji_current_is_written_to_selected_big_endian_slot)
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

ZTEST(protocols, test_pc_link_round_trip_and_crc_rejection)
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

ZTEST(protocols, test_mpc_link_state_round_trip_and_crc_rejection)
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

ZTEST(protocols, test_mpc_link_command_round_trip)
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

namespace {

class FakeChassis final : public protocols::ethercat::ChassisPort {
public:
	protocols::ethercat::ChassisSample ReadChassisSample() override { return sample; }
	void HandleMpcCommand(const protocols::ethercat::ChassisMpcCommand &value) override
	{
		last_command = value;
		command_count++;
	}
	void EnterLocalFallback() override { fallback_count++; }

	protocols::ethercat::ChassisSample sample{};
	protocols::ethercat::ChassisMpcCommand last_command{};
	uint32_t command_count = 0U;
	uint32_t fallback_count = 0U;
};

} // namespace

ZTEST(protocols, test_ethercat_benchmark_echo_validates_payload)
{
	using namespace protocols::ethercat;
	zassert_equal(PayloadCrc32(1U, 2U, 3U), 0xb0e02293U);
	EthercatSlaveCore core;
	FakeChassis chassis;
	MasterProcessImage master{};
	SlaveProcessImage slave{};
	master.benchmark.sequence = 123U;
	master.benchmark.command = kBenchmarkEcho;
	master.benchmark.payload0 = 0xa5a55a5aU;
	master.benchmark.payload1 = 0x12345678U;
	master.benchmark.payload_crc32 = PayloadCrc32(master.benchmark.sequence,
						      master.benchmark.payload0,
						      master.benchmark.payload1);

	core.Process(master, 1000U, chassis, slave);
	zassert_equal(slave.benchmark.echoed_sequence, master.benchmark.sequence);
	zassert_true((slave.benchmark.status & kBenchmarkAbiValid) != 0U);
	zassert_true((slave.benchmark.status & kBenchmarkPayloadValid) != 0U);
	zassert_equal(slave.benchmark.echoed_payload_crc32, master.benchmark.payload_crc32);

	master.benchmark.payload_crc32 ^= 1U;
	core.Process(master, 2000U, chassis, slave);
	zassert_true((slave.benchmark.status & kBenchmarkPayloadValid) == 0U);
}

ZTEST(protocols, test_ethercat_mpc_accepts_new_command_and_times_out_to_fallback)
{
	using namespace protocols::ethercat;
	EthercatSlaveCore core;
	FakeChassis chassis;
	MasterProcessImage master{};
	SlaveProcessImage slave{};
	chassis.sample.roll_rad = 0.125F;
	chassis.sample.status_flags = kStateImuFresh | kStateMotorFeedbackValid |
				      kStateControlEnabled;

	// Publish state sequence 1 before the host can reference it.
	core.Process(master, 0U, chassis, slave);
	zassert_equal(slave.state.sequence, 1U);

	master.mpc.sequence = 1U;
	master.mpc.source_state_sequence = 1U;
	master.mpc.mode_and_flags = kMpcActive | kMpcConverged | kMpcInputFinite;
	master.mpc.left_support_force_n = 200.0F;
	master.mpc.right_support_force_n = 10.0F;
	master.mpc.valid_for_us = 50000U;
	core.Process(master, 1000U, chassis, slave);
	zassert_equal(chassis.command_count, 1U);
	zassert_equal(chassis.fallback_count, 0U);
	zassert_equal(static_cast<uint32_t>(chassis.last_command.mode),
		      static_cast<uint32_t>(kMpcActive));
	zassert_within(chassis.last_command.left_support_force_n, 150.0F, 1.0e-6F);
	zassert_within(chassis.last_command.right_support_force_n, 20.0F, 1.0e-6F);
	zassert_equal(chassis.last_command.valid_for_us, 30000U);
	zassert_true((slave.state.status_flags & kStateCommandAccepted) != 0U);

	// A duplicate must neither run the controller nor refresh the watchdog.
	core.Process(master, 2000U, chassis, slave);
	zassert_equal(chassis.command_count, 1U);
	core.Process(master, 31001U, chassis, slave);
	zassert_equal(chassis.fallback_count, 1U);
	zassert_true((slave.state.status_flags & kStateCommandTimedOut) != 0U);
}

ZTEST(protocols, test_ethercat_mpc_rejects_stale_state_and_invalid_solver)
{
	using namespace protocols::ethercat;
	EthercatSlaveCore core;
	FakeChassis chassis;
	MasterProcessImage master{};
	SlaveProcessImage slave{};
	core.Process(master, 0U, chassis, slave);
	core.Process(master, 1000U, chassis, slave);
	core.Process(master, 2000U, chassis, slave);

	master.mpc.sequence = 1U;
	master.mpc.source_state_sequence = 1U;
	master.mpc.mode_and_flags = kMpcActive | kMpcInputFinite;
	master.mpc.left_support_force_n = 100.0F;
	master.mpc.right_support_force_n = 100.0F;
	master.mpc.valid_for_us = 30000U;
	core.Process(master, 3000U, chassis, slave);
	zassert_equal(chassis.command_count, 0U);
	zassert_true((slave.state.status_flags & kStateCommandAccepted) == 0U);
}

ZTEST(protocols, test_ethercat_mpc_rejects_unknown_mode_and_falls_back)
{
	using namespace protocols::ethercat;
	EthercatSlaveCore core;
	FakeChassis chassis;
	MasterProcessImage master{};
	SlaveProcessImage slave{};
	core.Process(master, 0U, chassis, slave);

	master.mpc.sequence = 1U;
	master.mpc.source_state_sequence = 1U;
	master.mpc.mode_and_flags = kMpcActive | kMpcConverged | kMpcInputFinite;
	master.mpc.left_support_force_n = 100.0F;
	master.mpc.right_support_force_n = 100.0F;
	master.mpc.valid_for_us = 30000U;
	core.Process(master, 1000U, chassis, slave);
	zassert_equal(chassis.command_count, 1U);

	master.mpc.sequence = 2U;
	master.mpc.source_state_sequence = slave.state.sequence;
	master.mpc.mode_and_flags = 0x7fU | kMpcConverged | kMpcInputFinite;
	core.Process(master, 2000U, chassis, slave);
	zassert_equal(chassis.command_count, 1U);
	zassert_equal(chassis.fallback_count, 1U);
	zassert_true((slave.state.status_flags & kStateCommandAccepted) == 0U);
}

ZTEST(protocols, test_ethercat_chassis_port_requires_both_local_gates)
{
	using namespace protocols::ethercat;
	auto &port = modules::EthercatChassisPort::Instance();
	port.Reset();
	ChassisSample published{};
	published.roll_rad = 0.25F;
	published.status_flags = kStateImuFresh | kStateControlEnabled;
	port.PublishChassisSample(published);
	zassert_within(port.ReadChassisSample().roll_rad, 0.25F, 1.0e-6F);

	ChassisMpcCommand command{};
	command.sequence = 1U;
	command.mode = kMpcActive;
	command.left_support_force_n = 120.0F;
	command.right_support_force_n = 130.0F;
	command.received_at_us = 1000U;
	command.valid_for_us = 30000U;
	port.HandleMpcCommand(command);
	ChassisMpcCommand consumed{};
	zassert_false(port.ConsumeActiveCommand(consumed, 1001U));
	zassert_equal(port.ReadDiagnostics().rejected_active_count, 1U);

	port.SetOperatorEnabled(true);
	port.SetChassisEligible(true);
	zassert_true((port.ReadChassisSample().status_flags &
		      kStateMpcOperatorEnabled) != 0U);
	command.sequence = 2U;
	port.HandleMpcCommand(command);
	zassert_true(port.ConsumeActiveCommand(consumed, 1001U));
	zassert_equal(consumed.sequence, 2U);
	zassert_true((port.ReadChassisSample().status_flags &
		      kStateMpcActiveApplied) != 0U);

	port.SetChassisEligible(false);
	zassert_false(port.ConsumeActiveCommand(consumed, 1002U));
	zassert_equal(port.ReadDiagnostics().fallback_count, 1U);
	zassert_false(port.ReadDiagnostics().operator_enabled);
	zassert_true((port.ReadChassisSample().status_flags &
		      kStateMpcActiveApplied) == 0U);

	// Shadow is observable but can never select the external VMC path.
	port.SetOperatorEnabled(true);
	port.SetChassisEligible(true);
	command.sequence = 3U;
	command.mode = kMpcShadow;
	port.HandleMpcCommand(command);
	zassert_false(port.ConsumeActiveCommand(consumed, 1003U));
	zassert_equal(port.ReadDiagnostics().shadow_count, 1U);

	// The chassis loop owns a second age watchdog. It must fall back even if
	// the SSC thread stops running and cannot call EthercatSlaveCore again.
	port.SetOperatorEnabled(true);
	command.sequence = 4U;
	command.mode = kMpcActive;
	command.received_at_us = 2000U;
	command.valid_for_us = 100U;
	port.HandleMpcCommand(command);
	zassert_true(port.ConsumeActiveCommand(consumed, 2100U));
	zassert_false(port.ConsumeActiveCommand(consumed, 2101U));
	zassert_false(port.ReadDiagnostics().operator_enabled);
	port.Reset();
}

ZTEST_SUITE(protocols, nullptr, nullptr, nullptr, nullptr, nullptr);
