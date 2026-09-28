/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <cstdint>

#include <zephyr/kernel.h>

#include <chassis_controller/communication/ethercat_slave_core.h>

namespace modules {

struct EthercatMpcDiagnostics {
	protocols::ethercat::ChassisMpcCommand last_shadow{};
	protocols::ethercat::ChassisMpcCommand last_active{};
	uint32_t shadow_count = 0U;
	uint32_t active_count = 0U;
	uint32_t rejected_active_count = 0U;
	uint32_t fallback_count = 0U;
	bool operator_enabled = false;
	bool chassis_eligible = false;
	bool active_applied = false;
};

/**
 * Thread-safe boundary between the SSC application thread and chassis loop.
 * Active output requires both an explicit local operator gate and an eligible
 * Balance state. Both default false after boot and reset.
 */
class EthercatChassisPort final : public protocols::ethercat::ChassisPort {
public:
	static EthercatChassisPort &Instance();

	protocols::ethercat::ChassisSample ReadChassisSample() override;
	void HandleMpcCommand(
		const protocols::ethercat::ChassisMpcCommand &command) override;
	void EnterLocalFallback() override;

	void PublishChassisSample(const protocols::ethercat::ChassisSample &sample);
	void SetOperatorEnabled(bool enabled);
	void SetChassisEligible(bool eligible);
	bool ConsumeActiveCommand(protocols::ethercat::ChassisMpcCommand &command,
				  uint64_t now_us);
	EthercatMpcDiagnostics ReadDiagnostics() const;
	void Reset();

private:
	mutable struct k_spinlock sample_lock_ = {};
	mutable struct k_spinlock control_lock_ = {};
	protocols::ethercat::ChassisSample sample_{};
	EthercatMpcDiagnostics diagnostics_{};
};

} // namespace modules
