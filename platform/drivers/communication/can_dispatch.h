/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>

namespace platform {

enum class CanTxSlot : uint8_t {
	kLeftWheel = 0,
	kRightWheel,
	kLeftJointB,
	kLeftJointD,
	kRightJointB,
	kRightJointD,
	kCount,
};

struct CanBusHealth {
	bool valid = false;
	uint8_t state = 0U;
	uint8_t tx_error_count = 0U;
	uint8_t rx_error_count = 0U;
	uint32_t async_tx_error_count = 0U;
};

struct CanTxSlotStats {
	uint32_t submitted_count = 0U;
	uint32_t enqueued_count = 0U;
	uint32_t completed_count = 0U;
	uint32_t received_count = 0U;
	uint32_t coalesced_count = 0U;
	uint32_t max_rx_interval_us = 0U;
};

struct CanTxQueueStats {
	uint32_t current_depth = 0U;
	uint32_t max_depth = 0U;
	uint32_t capacity = 0U;
};

int InitializeCanDispatch();
void NotifyCanTxPending();
int SubmitCanStandardFrame(CanTxSlot slot, uint8_t bus, uint16_t can_id,
			   const uint8_t *data, uint8_t dlc);
int ReadCanBusHealth(uint8_t bus, CanBusHealth *health);
uint32_t ReadCanAsyncTxErrorCount(uint8_t bus);
int ReadCanTxSlotStats(CanTxSlot slot, CanTxSlotStats *stats);
int ReadCanTxQueueStats(uint8_t bus, CanTxQueueStats *stats);
void ResetCanRxIntervalStats();
void ResetCanTxDiagnosticStats();

}  // namespace platform
