/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace protocols::ethercat {

constexpr uint32_t kAbiVersion = 0x57425201U;
constexpr uint16_t kBenchmarkRxIndex = 0x7000U;
constexpr uint16_t kBenchmarkTxIndex = 0x6000U;
constexpr uint16_t kMpcCommandIndex = 0x7010U;
constexpr uint16_t kChassisStateIndex = 0x6010U;

enum BenchmarkCommand : uint32_t {
	kBenchmarkIdle = 0U,
	kBenchmarkEcho = 1U,
	kBenchmarkResetCounters = 2U,
};

enum BenchmarkStatus : uint32_t {
	kBenchmarkAbiValid = 1U << 0,
	kBenchmarkPayloadValid = 1U << 1,
	kBenchmarkOperational = 1U << 2,
	kBenchmarkWatchdogExpired = 1U << 3,
};

enum MpcMode : uint32_t {
	kMpcDisabled = 0U,
	kMpcShadow = 1U,
	kMpcActive = 2U,
};

enum MpcCommandFlag : uint32_t {
	kMpcConverged = 1U << 8,
	kMpcInputFinite = 1U << 9,
	kMpcUsedFallback = 1U << 10,
};

enum ChassisStateFlag : uint32_t {
	kStateImuFresh = 1U << 0,
	kStateSupportForceValid = 1U << 1,
	kStateMotorFeedbackValid = 1U << 2,
	kStateControlEnabled = 1U << 3,
	kStateContactLeft = 1U << 4,
	kStateContactRight = 1U << 5,
	kStateCommandAccepted = 1U << 6,
	kStateCommandTimedOut = 1U << 7,
	kStateMpcOperatorEnabled = 1U << 8,
	kStateMpcActiveApplied = 1U << 9,
};

struct BenchmarkRxPdo {
	uint32_t abi_version = kAbiVersion;
	uint32_t sequence = 0U;
	uint64_t host_send_time_ns = 0U;
	uint32_t command = kBenchmarkIdle;
	uint32_t payload_crc32 = 0U;
	uint32_t payload0 = 0U;
	uint32_t payload1 = 0U;
};

struct BenchmarkTxPdo {
	uint32_t abi_version = kAbiVersion;
	uint32_t echoed_sequence = 0U;
	uint64_t slave_receive_ticks = 0U;
	uint64_t slave_transmit_ticks = 0U;
	uint32_t status = 0U;
	uint32_t echoed_payload_crc32 = 0U;
};

struct MpcCommandPdo {
	uint32_t abi_version = kAbiVersion;
	uint32_t sequence = 0U;
	uint32_t source_state_sequence = 0U;
	uint32_t mode_and_flags = kMpcDisabled;
	uint64_t host_send_time_ns = 0U;
	float left_support_force_n = 0.0F;
	float right_support_force_n = 0.0F;
	uint32_t valid_for_us = 0U;
	uint32_t solve_time_us = 0U;
};

struct ChassisStatePdo {
	uint32_t abi_version = kAbiVersion;
	uint32_t sequence = 0U;
	uint64_t sample_ticks = 0U;
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
	uint32_t reserved = 0U;
};

struct MasterProcessImage {
	BenchmarkRxPdo benchmark;
	MpcCommandPdo mpc;
};

struct SlaveProcessImage {
	BenchmarkTxPdo benchmark;
	ChassisStatePdo state;
};

static_assert(sizeof(BenchmarkRxPdo) == 32U);
static_assert(sizeof(BenchmarkTxPdo) == 32U);
static_assert(sizeof(MpcCommandPdo) == 40U);
static_assert(sizeof(ChassisStatePdo) == 64U);
static_assert(sizeof(MasterProcessImage) == 72U);
static_assert(sizeof(SlaveProcessImage) == 96U);
static_assert(offsetof(MasterProcessImage, mpc) == 32U);
static_assert(offsetof(SlaveProcessImage, state) == 32U);

uint32_t PayloadCrc32(uint32_t sequence, uint32_t payload0, uint32_t payload1);

} // namespace protocols::ethercat
