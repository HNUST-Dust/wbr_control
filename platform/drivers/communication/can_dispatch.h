/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <cstdint>

namespace platform::drivers::communication::can_dispatch {

enum class TxSlot : uint8_t {
	kLeftWheel = 0,
	kRightWheel,
	kLeftJointB,
	kLeftJointD,
	kRightJointB,
	kRightJointD,
	kCount,
};

struct TxStats {
	uint32_t done_count[static_cast<uint8_t>(TxSlot::kCount)];
};

struct RxStats {
	uint32_t routed_count[static_cast<uint8_t>(TxSlot::kCount)];
};

int Initialize();
void NotifyTxPending();
TxStats GetTxStats();
RxStats GetRxStats();

}  // namespace platform::drivers::communication::can_dispatch
