/* SPDX-License-Identifier: Apache-2.0 */

/**
* @file src/modules/sys_state/sys_state_module.h
 * @ingroup wbr_modules
 * @brief 管理系统状态、使能条件与故障状态。
 * @details 模块遵循 `ModuleBase` 生命周期：`Start()` 只负责一次性资源初始化和线程创建，`RunLoop()` 持有周期状态。跨线程数据通过 channels 层交换。
 */

#pragma once

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

#include "../module_base.h"

namespace modules
{

/** @brief 系统状态模块写入调试变量的生命周期诊断码。 */
enum : uint32_t {
	kSysStateDiagBoot = 0U, ///< 静态初始化后的上电初始状态。
	kSysStateDiagInitEnter = 1U, ///< 已进入外设初始化流程。
	kSysStateDiagInitGpioNotReady = 2U, ///< 状态灯 GPIO 控制器尚未就绪。
	kSysStateDiagInitConfigRFail = 3U, ///< 红色状态灯 GPIO 配置失败。
	kSysStateDiagInitConfigGFail = 4U, ///< 绿色状态灯 GPIO 配置失败。
	kSysStateDiagInitConfigBFail = 5U, ///< 蓝色状态灯 GPIO 配置失败。
	kSysStateDiagInitNoAliases = 6U, ///< 设备树未提供任何状态输出别名。
	kSysStateDiagInitReady = 7U, ///< 可用状态输出已完成初始化。
	kSysStateDiagStartSkippedNotReady = 8U, ///< 外设未就绪，拒绝启动线程。
	kSysStateDiagStartSkippedAlreadyStarted = 9U, ///< 模块线程已经启动，忽略重复请求。
	kSysStateDiagThreadCreateFail = 10U, ///< 工作线程创建失败。
	kSysStateDiagThreadCreated = 11U, ///< 工作线程创建成功。
	kSysStateDiagRunLoopEnter = 12U, ///< 工作线程已进入周期主循环。
	kSysStateDiagInitBuzzerNotReady = 13U, ///< 蜂鸣器 PWM 控制器尚未就绪。
	kSysStateDiagInitNoOutputs = 14U, ///< 状态灯和蜂鸣器均不可用。
};

/** @brief 供调试器和诊断 Shell 读取的系统状态位图。 */
extern volatile uint32_t g_sys_state_diag_state;

/** @brief 维护整机启动、使能和故障状态。 */
class SysStateModule : public ModuleBase
{
public:
	SysStateModule() = default;
	/**
	 * @brief 初始化模块资源并创建工作线程。
	 * @return 成功返回 0；初始化或线程创建失败返回负 errno 错误码。
	 */
	int Start() override;
	/**
	 * @brief 执行模块线程的周期主循环。
	 */
	void RunLoop() override;

private:
	/**
	 * @brief 设置 RGB 状态灯三个通道的占空比。
	 * @param r 红色 LED 通道占空比。
	 * @param g 绿色 LED 通道占空比。
	 * @param b 蓝色 LED 通道占空比。
	 */
	void ApplyDuty(uint8_t r, uint8_t g, uint8_t b);
	/**
	 * @brief 设置蜂鸣器的百分比占空比。
	 * @param pct 蜂鸣器占空比百分数，范围为 0 至 100。
	 */
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
