/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

#include <protocols/ethercat/ethercat_process_data.h>

namespace protocols::ethercat {

struct ChassisSample {
	float roll_rad = 0.0F;
	float roll_rate_rad_s = 0.0F;
	float body_height_m = 0.0F;
	float body_height_rate_m_s = 0.0F;
	float left_leg_length_m = 0.0F;
	float left_leg_rate_m_s = 0.0F;
	float right_leg_length_m = 0.0F;
	float right_leg_rate_m_s = 0.0F;
	float left_support_force_n = 0.0F;
	float right_support_force_n = 0.0F;
	uint32_t status_flags = 0U;
};

struct ChassisMpcCommand {
	uint32_t sequence = 0U;
	uint32_t source_state_sequence = 0U;
	MpcMode mode = kMpcDisabled;
	uint32_t solver_flags = 0U;
	float left_support_force_n = 0.0F;
	float right_support_force_n = 0.0F;
	uint32_t valid_for_us = 0U;
	uint32_t solve_time_us = 0U;
	uint64_t received_at_us = 0U;
};

class ChassisPort {
public:
	virtual ~ChassisPort() = default;
	virtual ChassisSample ReadChassisSample() = 0;
	// Shadow commands are delivered for logging/comparison but must not drive VMC.
	virtual void HandleMpcCommand(const ChassisMpcCommand &command) = 0;
	// The implementation must restore the local Roll PD + leg PID controller.
	virtual void EnterLocalFallback() = 0;
};

struct SlaveCoreConfig {
	float minimum_force_n = 20.0F;
	float maximum_force_n = 150.0F;
	float maximum_force_step_n = 30.0F;
	uint32_t maximum_valid_for_us = 30000U;
	uint32_t maximum_state_lag = 2U;
};

class EthercatSlaveCore {
public:
	explicit EthercatSlaveCore(SlaveCoreConfig config = {});
	void Reset(ChassisPort &chassis);
	void Process(const MasterProcessImage &master, uint64_t now_us,
		     ChassisPort &chassis, SlaveProcessImage &slave);

private:
	bool ProcessMpcCommand(const MpcCommandPdo &input, uint64_t now_us,
			       ChassisPort &chassis);
	void CheckWatchdog(uint64_t now_us, ChassisPort &chassis);

	SlaveCoreConfig config_;
	uint32_t state_sequence_ = 0U;
	uint32_t last_command_sequence_ = 0U;
	uint64_t last_command_receive_us_ = 0U;
	uint32_t command_valid_for_us_ = 0U;
	float last_left_force_n_ = 0.0F;
	float last_right_force_n_ = 0.0F;
	bool have_command_ = false;
	bool active_ = false;
	bool timed_out_ = false;
	bool command_accepted_ = false;
};

} // namespace protocols::ethercat
