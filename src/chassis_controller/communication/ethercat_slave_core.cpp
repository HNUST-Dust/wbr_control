/* SPDX-License-Identifier: Apache-2.0 */

#include <chassis_controller/communication/ethercat_slave_core.h>

#include <math.h>

namespace protocols::ethercat {
namespace {

bool IsNewer(uint32_t value, uint32_t previous)
{
	return value != previous && static_cast<int32_t>(value - previous) > 0;
}

float LimitStep(float value, float previous, float maximum_step)
{
	const float minimum = previous - maximum_step;
	const float maximum = previous + maximum_step;
	return value < minimum ? minimum : (value > maximum ? maximum : value);
}

float Clamp(float value, float minimum, float maximum)
{
	return value < minimum ? minimum : (value > maximum ? maximum : value);
}

} // namespace

EthercatSlaveCore::EthercatSlaveCore(SlaveCoreConfig config) : config_(config) {}

void EthercatSlaveCore::Reset(ChassisPort &chassis)
{
	state_sequence_ = 0U;
	last_command_sequence_ = 0U;
	last_command_receive_us_ = 0U;
	command_valid_for_us_ = 0U;
	last_left_force_n_ = 0.0F;
	last_right_force_n_ = 0.0F;
	have_command_ = false;
	active_ = false;
	timed_out_ = false;
	command_accepted_ = false;
	chassis.EnterLocalFallback();
}

void EthercatSlaveCore::CheckWatchdog(uint64_t now_us, ChassisPort &chassis)
{
	if (active_ && have_command_ && now_us - last_command_receive_us_ > command_valid_for_us_) {
		active_ = false;
		timed_out_ = true;
		command_accepted_ = false;
		chassis.EnterLocalFallback();
	}
}

bool EthercatSlaveCore::ProcessMpcCommand(const MpcCommandPdo &input, uint64_t now_us,
					ChassisPort &chassis)
{
	if (!IsNewer(input.sequence, last_command_sequence_)) {
		return false;
	}
	last_command_sequence_ = input.sequence;
	const auto mode = static_cast<MpcMode>(input.mode_and_flags & 0xffU);
	const bool mode_valid = mode == kMpcDisabled || mode == kMpcShadow ||
				mode == kMpcActive;
	if (!mode_valid) {
		if (active_) chassis.EnterLocalFallback();
		active_ = false;
		command_accepted_ = false;
		return false;
	}
	if (mode == kMpcDisabled) {
		if (active_) {
			chassis.EnterLocalFallback();
		}
		active_ = false;
		command_accepted_ = false;
		return false;
	}
	const uint32_t required_flags = kMpcConverged | kMpcInputFinite;
	const bool flags_valid = (input.mode_and_flags & required_flags) == required_flags;
	const bool values_finite = isfinite(input.left_support_force_n) &&
				   isfinite(input.right_support_force_n);
	const uint32_t state_lag = state_sequence_ - input.source_state_sequence;
	const bool state_valid = input.source_state_sequence != 0U &&
				 state_lag <= config_.maximum_state_lag;
	const uint32_t validity = input.valid_for_us < config_.maximum_valid_for_us
					  ? input.valid_for_us
					  : config_.maximum_valid_for_us;
	if (!flags_valid || !values_finite || !state_valid || validity == 0U) {
		if (active_) chassis.EnterLocalFallback();
		active_ = false;
		command_accepted_ = false;
		return false;
	}

	float left = Clamp(input.left_support_force_n, config_.minimum_force_n,
			   config_.maximum_force_n);
	float right = Clamp(input.right_support_force_n, config_.minimum_force_n,
			    config_.maximum_force_n);
	if (have_command_) {
		left = LimitStep(left, last_left_force_n_, config_.maximum_force_step_n);
		right = LimitStep(right, last_right_force_n_, config_.maximum_force_step_n);
	}
	ChassisMpcCommand command{};
	command.sequence = input.sequence;
	command.source_state_sequence = input.source_state_sequence;
	command.mode = mode;
	command.solver_flags = input.mode_and_flags & ~0xffU;
	command.left_support_force_n = left;
	command.right_support_force_n = right;
	command.valid_for_us = validity;
	command.solve_time_us = input.solve_time_us;
	command.received_at_us = now_us;
	chassis.HandleMpcCommand(command);
	last_command_receive_us_ = now_us;
	command_valid_for_us_ = validity;
	last_left_force_n_ = left;
	last_right_force_n_ = right;
	have_command_ = true;
	active_ = mode == kMpcActive;
	timed_out_ = false;
	command_accepted_ = true;
	return true;
}

void EthercatSlaveCore::Process(const MasterProcessImage &master, uint64_t now_us,
			       ChassisPort &chassis, SlaveProcessImage &slave)
{
	CheckWatchdog(now_us, chassis);
	if (master.mpc.abi_version == kAbiVersion) {
		(void)ProcessMpcCommand(master.mpc, now_us, chassis);
	} else if (active_) {
		active_ = false;
		command_accepted_ = false;
		chassis.EnterLocalFallback();
	}

	slave.benchmark = {};
	slave.benchmark.slave_receive_ticks = now_us;
	if (master.benchmark.abi_version == kAbiVersion) {
		slave.benchmark.status |= kBenchmarkAbiValid;
		slave.benchmark.echoed_sequence = master.benchmark.sequence;
		slave.benchmark.echoed_payload_crc32 = master.benchmark.payload_crc32;
		if (master.benchmark.payload_crc32 == PayloadCrc32(master.benchmark.sequence,
							      master.benchmark.payload0,
							      master.benchmark.payload1)) {
			slave.benchmark.status |= kBenchmarkPayloadValid;
		}
	}
	slave.benchmark.status |= kBenchmarkOperational;
	if (timed_out_) slave.benchmark.status |= kBenchmarkWatchdogExpired;

	const ChassisSample sample = chassis.ReadChassisSample();
	slave.state = {};
	slave.state.sequence = ++state_sequence_;
	slave.state.sample_ticks = now_us;
	slave.state.roll_rad = sample.roll_rad;
	slave.state.roll_rate_rad_s = sample.roll_rate_rad_s;
	slave.state.body_height_m = sample.body_height_m;
	slave.state.body_height_rate_m_s = sample.body_height_rate_m_s;
	slave.state.left_leg_length_m = sample.left_leg_length_m;
	slave.state.left_leg_rate_m_s = sample.left_leg_rate_m_s;
	slave.state.right_leg_length_m = sample.right_leg_length_m;
	slave.state.right_leg_rate_m_s = sample.right_leg_rate_m_s;
	slave.state.left_support_force_n = sample.left_support_force_n;
	slave.state.right_support_force_n = sample.right_support_force_n;
	slave.state.status_flags = sample.status_flags;
	if (command_accepted_) slave.state.status_flags |= kStateCommandAccepted;
	if (timed_out_) slave.state.status_flags |= kStateCommandTimedOut;
	slave.benchmark.slave_transmit_ticks = now_us;
}

} // namespace protocols::ethercat
