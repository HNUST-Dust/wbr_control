/* SPDX-License-Identifier: Apache-2.0 */

#include "ethercat_chassis_port.h"

namespace modules {
namespace {
EthercatChassisPort g_ethercat_chassis_port;
}

EthercatChassisPort &EthercatChassisPort::Instance()
{
	return g_ethercat_chassis_port;
}

protocols::ethercat::ChassisSample EthercatChassisPort::ReadChassisSample()
{
	protocols::ethercat::ChassisSample result;
	{
		const k_spinlock_key_t key = k_spin_lock(&sample_lock_);
		result = sample_;
		k_spin_unlock(&sample_lock_, key);
	}
	{
		const k_spinlock_key_t key = k_spin_lock(&control_lock_);
		if (diagnostics_.operator_enabled && diagnostics_.chassis_eligible) {
			result.status_flags |= protocols::ethercat::kStateMpcOperatorEnabled;
		}
		if (diagnostics_.active_applied) {
			result.status_flags |= protocols::ethercat::kStateMpcActiveApplied;
		}
		k_spin_unlock(&control_lock_, key);
	}
	return result;
}

void EthercatChassisPort::HandleMpcCommand(
	const protocols::ethercat::ChassisMpcCommand &command)
{
	const k_spinlock_key_t key = k_spin_lock(&control_lock_);
	if (command.mode == protocols::ethercat::kMpcShadow) {
		diagnostics_.last_shadow = command;
		diagnostics_.shadow_count++;
		diagnostics_.active_applied = false;
	} else if (command.mode == protocols::ethercat::kMpcActive) {
		diagnostics_.last_active = command;
		if (diagnostics_.operator_enabled && diagnostics_.chassis_eligible) {
			diagnostics_.active_count++;
			diagnostics_.active_applied = true;
		} else {
			diagnostics_.rejected_active_count++;
			diagnostics_.active_applied = false;
		}
	}
	k_spin_unlock(&control_lock_, key);
}

void EthercatChassisPort::EnterLocalFallback()
{
	const k_spinlock_key_t key = k_spin_lock(&control_lock_);
	if (diagnostics_.active_applied) {
		diagnostics_.fallback_count++;
	}
	diagnostics_.active_applied = false;
	diagnostics_.operator_enabled = false;
	k_spin_unlock(&control_lock_, key);
}

void EthercatChassisPort::PublishChassisSample(
	const protocols::ethercat::ChassisSample &sample)
{
	const k_spinlock_key_t key = k_spin_lock(&sample_lock_);
	sample_ = sample;
	k_spin_unlock(&sample_lock_, key);
}

void EthercatChassisPort::SetOperatorEnabled(bool enabled)
{
	const k_spinlock_key_t key = k_spin_lock(&control_lock_);
	diagnostics_.operator_enabled = enabled;
	if (!enabled && diagnostics_.active_applied) {
		diagnostics_.fallback_count++;
		diagnostics_.active_applied = false;
	}
	k_spin_unlock(&control_lock_, key);
}

void EthercatChassisPort::SetChassisEligible(bool eligible)
{
	const k_spinlock_key_t key = k_spin_lock(&control_lock_);
	diagnostics_.chassis_eligible = eligible;
	if (!eligible && diagnostics_.active_applied) {
		diagnostics_.fallback_count++;
		diagnostics_.active_applied = false;
	}
	if (!eligible) diagnostics_.operator_enabled = false;
	k_spin_unlock(&control_lock_, key);
}

bool EthercatChassisPort::ConsumeActiveCommand(
	protocols::ethercat::ChassisMpcCommand &command, uint64_t now_us)
{
	const k_spinlock_key_t key = k_spin_lock(&control_lock_);
	bool active = diagnostics_.active_applied &&
		      diagnostics_.operator_enabled &&
		      diagnostics_.chassis_eligible;
	if (active &&
	    (now_us < diagnostics_.last_active.received_at_us ||
	     now_us - diagnostics_.last_active.received_at_us >
		     diagnostics_.last_active.valid_for_us)) {
		diagnostics_.active_applied = false;
		diagnostics_.operator_enabled = false;
		diagnostics_.fallback_count++;
		active = false;
	}
	if (active) command = diagnostics_.last_active;
	k_spin_unlock(&control_lock_, key);
	return active;
}

EthercatMpcDiagnostics EthercatChassisPort::ReadDiagnostics() const
{
	const k_spinlock_key_t key = k_spin_lock(&control_lock_);
	const EthercatMpcDiagnostics result = diagnostics_;
	k_spin_unlock(&control_lock_, key);
	return result;
}

void EthercatChassisPort::Reset()
{
	{
		const k_spinlock_key_t key = k_spin_lock(&sample_lock_);
		sample_ = {};
		k_spin_unlock(&sample_lock_, key);
	}
	{
		const k_spinlock_key_t key = k_spin_lock(&control_lock_);
		diagnostics_ = {};
		k_spin_unlock(&control_lock_, key);
	}
}

} // namespace modules
