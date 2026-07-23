/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

#include <modules/thread_utils.h>
#include <modules/sys_state/sys_state_module.h>

LOG_MODULE_REGISTER(sys_state_module, LOG_LEVEL_INF);

namespace {

#if DT_NODE_EXISTS(DT_ALIAS(led0))
const gpio_dt_spec kLedR = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led1))
const gpio_dt_spec kLedG = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led2))
const gpio_dt_spec kLedB = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
#endif

#if DT_NODE_EXISTS(DT_ALIAS(pwm_buzzer))
#define RM_TEST_BUZZER_NODE DT_ALIAS(pwm_buzzer)
#elif DT_NODE_EXISTS(DT_ALIAS(buzzer_pwm))
#define RM_TEST_BUZZER_NODE DT_ALIAS(buzzer_pwm)
#elif DT_NODE_EXISTS(DT_ALIAS(buzzer))
#define RM_TEST_BUZZER_NODE DT_ALIAS(buzzer)
#endif

#ifdef RM_TEST_BUZZER_NODE
const pwm_dt_spec kBuzzer = PWM_DT_SPEC_GET(RM_TEST_BUZZER_NODE);
#endif

K_THREAD_STACK_DEFINE(g_sys_state_module_stack, 1024);

constexpr uint8_t kPwmLevels = 64U;
constexpr uint32_t kLedFrameMs = 16U;
constexpr uint16_t kBreathSteps = 200U;
constexpr uint32_t kBuzzerPeriodUs = 2000U;
constexpr uint8_t kBuzzerTickPercent = 20U;
constexpr uint16_t kBuzzerTickSteps = 5U;

uint8_t TriangleBrightnessPercent(uint16_t step)
{
	const uint16_t half = kBreathSteps / 2U;
	if (step < half) {
		return static_cast<uint8_t>((step * 100U) / half);
	}

	return static_cast<uint8_t>(((kBreathSteps - step) * 100U) / half);
}

uint8_t PercentToDuty(uint8_t pct)
{
	if (pct >= 100U) {
		return static_cast<uint8_t>(kPwmLevels - 1U);
	}

	return static_cast<uint8_t>((pct * kPwmLevels) / 100U);
}

}  // namespace

namespace modules {

volatile uint32_t g_sys_state_diag_state = kSysStateDiagBoot;

int SysStateModule::Initialize()
{
	g_sys_state_diag_state = kSysStateDiagInitEnter;
	started_ = false;
	ready_ = false;
	led_ready_ = false;
	led_r_ready_ = false;
	led_g_ready_ = false;
	led_b_ready_ = false;
	buzzer_ready_ = false;
	buzzer_duty_pct_ = 0U;

#if DT_NODE_EXISTS(DT_ALIAS(led0)) && DT_NODE_EXISTS(DT_ALIAS(led1)) && DT_NODE_EXISTS(DT_ALIAS(led2))
	led_r_ = kLedR;
	led_g_ = kLedG;
	led_b_ = kLedB;

	if (!gpio_is_ready_dt(&led_r_) || !gpio_is_ready_dt(&led_g_) || !gpio_is_ready_dt(&led_b_)) {
		g_sys_state_diag_state = kSysStateDiagInitGpioNotReady;
		LOG_WRN("sys_state init skipped: gpio not ready");
		return 0;
	}

	int rc = gpio_pin_configure_dt(&led_r_, GPIO_OUTPUT_INACTIVE);
	if (rc != 0) {
		g_sys_state_diag_state = kSysStateDiagInitConfigRFail;
		LOG_WRN("sys_state init skipped: led_r config failed (%d)", rc);
		return 0;
	}

	rc = gpio_pin_configure_dt(&led_g_, GPIO_OUTPUT_INACTIVE);
	if (rc != 0) {
		g_sys_state_diag_state = kSysStateDiagInitConfigGFail;
		LOG_WRN("sys_state init skipped: led_g config failed (%d)", rc);
		return 0;
	}

	rc = gpio_pin_configure_dt(&led_b_, GPIO_OUTPUT_INACTIVE);
	if (rc != 0) {
		g_sys_state_diag_state = kSysStateDiagInitConfigBFail;
		LOG_WRN("sys_state init skipped: led_b config failed (%d)", rc);
		return 0;
	}

	led_ready_ = true;
	led_r_ready_ = true;
	led_g_ready_ = true;
	led_b_ready_ = true;
#else
	LOG_WRN("sys_state init skipped: led aliases not found");
#endif

#ifdef RM_TEST_BUZZER_NODE
	buzzer_ = kBuzzer;
	if (device_is_ready(buzzer_.dev)) {
		const uint32_t period = PWM_USEC(kBuzzerPeriodUs);
		const int rc = pwm_set_dt(&buzzer_, period, 0U);
		if (rc == 0) {
			buzzer_ready_ = true;
		}
	}
#else
	LOG_WRN("status_indicator init skipped: buzzer alias not found");
#endif

	ready_ = led_ready_ || buzzer_ready_;

	if (!ready_) {
		g_sys_state_diag_state = kSysStateDiagInitNoOutputs;
		LOG_WRN("status_indicator init skipped: no output device ready");
		return 0;
	}

	g_sys_state_diag_state = kSysStateDiagInitReady;
	return 0;
}

int SysStateModule::Start()
{
	if (!ready_) {
		g_sys_state_diag_state = kSysStateDiagStartSkippedNotReady;
		return 0;
	}

	if (started_) {
		g_sys_state_diag_state = kSysStateDiagStartSkippedAlreadyStarted;
		return 0;
	}

	k_tid_t tid = ::modules::StartMemberThread<SysStateModule, &SysStateModule::RunLoop>(
		&thread_,
		g_sys_state_module_stack,
		K_THREAD_STACK_SIZEOF(g_sys_state_module_stack),
		this,
		K_PRIO_PREEMPT(8),
		"sys_state_module");

	if (tid == nullptr) {
		g_sys_state_diag_state = kSysStateDiagStartThreadCreateFail;
		return -ENOMEM;
	}

	started_ = true;
	g_sys_state_diag_state = kSysStateDiagStartThreadCreated;
	return 0;
}

void SysStateModule::ApplyDuty(uint8_t r, uint8_t g, uint8_t b)
{
	if (!led_ready_) {
		return;
	}

	const uint8_t peak = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);
	if (peak == 0U) {
		(void)gpio_pin_set_dt(&led_r_, 0);
		(void)gpio_pin_set_dt(&led_g_, 0);
		(void)gpio_pin_set_dt(&led_b_, 0);
		k_sleep(K_MSEC(kLedFrameMs));
		return;
	}

	const uint32_t on_ms = (static_cast<uint32_t>(peak) * kLedFrameMs) / (kPwmLevels - 1U);
	const uint32_t off_ms = (on_ms >= kLedFrameMs) ? 0U : (kLedFrameMs - on_ms);

	(void)gpio_pin_set_dt(&led_r_, (r > 0U) ? 1 : 0);
	(void)gpio_pin_set_dt(&led_g_, (g > 0U) ? 1 : 0);
	(void)gpio_pin_set_dt(&led_b_, (b > 0U) ? 1 : 0);
	if (on_ms > 0U) {
		k_sleep(K_MSEC(on_ms));
	}

	(void)gpio_pin_set_dt(&led_r_, 0);
	(void)gpio_pin_set_dt(&led_g_, 0);
	(void)gpio_pin_set_dt(&led_b_, 0);
	if (off_ms > 0U) {
		k_sleep(K_MSEC(off_ms));
	}
}

void SysStateModule::ApplyBuzzerPercent(uint8_t pct)
{
	if (!buzzer_ready_) {
		return;
	}

	if (pct > 100U) {
		pct = 100U;
	}

	if (pct == buzzer_duty_pct_) {
		return;
	}

	buzzer_duty_pct_ = pct;
	const uint32_t period = PWM_USEC(kBuzzerPeriodUs);
	const uint32_t pulse = static_cast<uint32_t>((static_cast<uint64_t>(period) * pct) / 100U);
	(void)pwm_set_dt(&buzzer_, period, pulse);
}

void SysStateModule::RunLoop()
{
	g_sys_state_diag_state = kSysStateDiagRunLoopEnter;
	LOG_INF("status_indicator module started");

	uint16_t breathe_step = 0U;
	uint8_t color_idx = 0U;

	while (true) {
		const uint8_t brightness_pct = TriangleBrightnessPercent(breathe_step);
		const uint8_t duty = PercentToDuty(brightness_pct);

		if (led_ready_) {
			switch (color_idx) {
			case 0U:
				ApplyDuty(duty, 0U, 0U);
				break;
			case 1U:
				ApplyDuty(0U, duty, 0U);
				break;
			default:
				ApplyDuty(0U, 0U, duty);
				break;
			}
		}

		const uint8_t buzzer_pct = (breathe_step < kBuzzerTickSteps) ? kBuzzerTickPercent : 0U;
		ApplyBuzzerPercent(buzzer_pct);

		++breathe_step;
		if (breathe_step >= kBreathSteps) {
			breathe_step = 0U;
			color_idx = static_cast<uint8_t>((color_idx + 1U) % 3U);
		}

		/* LED pacing is handled inside ApplyDuty(). */

	}
}

}  // namespace modules
