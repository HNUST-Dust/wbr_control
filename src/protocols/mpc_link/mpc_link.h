/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace protocols {

constexpr uint8_t kMpcLinkMagic0 = 'W';
constexpr uint8_t kMpcLinkMagic1 = 'M';
constexpr uint8_t kMpcLinkVersion = 1U;
constexpr uint8_t kMpcLinkStateMessageType = 0x10U;
constexpr uint8_t kMpcLinkCommandMessageType = 0x20U;
constexpr size_t kMpcLinkHeaderSize = 14U;
constexpr size_t kMpcLinkCrcSize = 2U;
constexpr size_t kMpcLinkStatePayloadSize = 44U;
constexpr size_t kMpcLinkCommandPayloadSize = 24U;
constexpr size_t kMpcLinkStateFrameSize =
	kMpcLinkHeaderSize + kMpcLinkStatePayloadSize + kMpcLinkCrcSize;
constexpr size_t kMpcLinkCommandFrameSize =
	kMpcLinkHeaderSize + kMpcLinkCommandPayloadSize + kMpcLinkCrcSize;
constexpr size_t kMpcLinkMaximumFrameSize = 64U;

static_assert(kMpcLinkStateFrameSize == 60U);
static_assert(kMpcLinkCommandFrameSize == 40U);
static_assert(kMpcLinkStateFrameSize <= kMpcLinkMaximumFrameSize);
static_assert(kMpcLinkCommandFrameSize <= kMpcLinkMaximumFrameSize);

enum MpcLinkStateValidFlag : uint8_t {
	kMpcStateImuFresh = 1U << 0,
	kMpcStateSupportForceValid = 1U << 1,
	kMpcStateMotorFeedbackValid = 1U << 2,
	kMpcStateControlEnabled = 1U << 3,
};

enum MpcLinkContactFlag : uint8_t {
	kMpcContactLeft = 1U << 0,
	kMpcContactRight = 1U << 1,
};

enum class MpcLinkCommandMode : uint8_t {
	kDisabled = 0U,
	kShadow = 1U,
	kActive = 2U,
};

enum class MpcLinkContactMode : uint8_t {
	kUnknown = 0U,
	kBoth = 1U,
	kLeftOnly = 2U,
	kRightOnly = 3U,
	kAirborne = 4U,
};

enum MpcLinkSolverFlag : uint8_t {
	kMpcSolverConverged = 1U << 0,
	kMpcSolverInputFinite = 1U << 1,
	kMpcSolverUsedFallback = 1U << 2,
};

struct MpcLinkState {
	uint32_t sequence = 0U;
	uint32_t sender_uptime_ms = 0U;
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
	uint8_t control_state = 0U;
	uint8_t valid_flags = 0U;
	uint8_t contact_flags = 0U;
};

struct MpcLinkCommand {
	uint32_t sequence = 0U;
	uint32_t sender_uptime_ms = 0U;
	uint32_t source_state_sequence = 0U;
	float left_support_force_n = 0.0F;
	float right_support_force_n = 0.0F;
	MpcLinkCommandMode mode = MpcLinkCommandMode::kDisabled;
	MpcLinkContactMode contact_mode = MpcLinkContactMode::kUnknown;
	uint8_t solver_flags = 0U;
	uint16_t solver_iterations = 0U;
	uint16_t valid_for_ms = 0U;
	uint32_t solve_time_us = 0U;
};

uint16_t CalculateMpcLinkCrc16(const uint8_t *data, size_t size);

int EncodeMpcLinkState(const MpcLinkState *state, uint8_t *out,
		       size_t capacity, size_t *out_len);
int DecodeMpcLinkState(const uint8_t *frame, size_t frame_len,
		       MpcLinkState *out);
int EncodeMpcLinkCommand(const MpcLinkCommand *command, uint8_t *out,
			 size_t capacity, size_t *out_len);
int DecodeMpcLinkCommand(const uint8_t *frame, size_t frame_len,
			 MpcLinkCommand *out);

} // namespace protocols
