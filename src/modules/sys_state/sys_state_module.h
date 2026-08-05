/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

#include "../module_base.h"

namespace modules
{

enum : uint32_t {
	kSysStateDiagBoot = 0U,
	kSysStateDiagInitEnter = 1U,
	kSysStateDiagInitGpioNotReady = 2U,
	kSysStateDiagInitConfigRFail = 3U,
	kSysStateDiagInitConfigGFail = 4U,
	kSysStateDiagInitConfigBFail = 5U,
	kSysStateDiagInitNoAliases = 6U,
	kSysStateDiagInitReady = 7U,
	kSysStateDiagStartSkippedNotReady = 8U,
	kSysStateDiagStartSkippedAlreadyStarted = 9U,
	kSysStateDiagThreadCreateFail = 10U,
	kSysStateDiagThreadCreated = 11U,
	kSysStateDiagRunLoopEnter = 12U,
	kSysStateDiagInitBuzzerNotReady = 13U,
	kSysStateDiagInitNoOutputs = 14U,
};

extern volatile uint32_t g_sys_state_diag_state;

class SysStateModule : public ModuleBase
{
public:
	SysStateModule() = default;
	int Start() override;
	void RunLoop() override;

private:
	void ApplyDuty(uint8_t r, uint8_t g, uint8_t b);
	void ApplyBuzzerPercent(uint8_t pct);

	struct gpio_dt_spec led_r_;
	struct gpio_dt_spec led_g_;
	struct gpio_dt_spec led_b_;
	struct pwm_dt_spec buzzer_;
	bool ready_ = false;
	bool led_ready_ = false;
	bool led_r_ready_ = false;
	bool led_g_ready_ = false;
	bool led_b_ready_ = false;
	bool buzzer_ready_ = false;
	uint8_t buzzer_duty_pct_ = 0U;
	uint32_t missed_release_count_ = 0U;
};

} // namespace modules
