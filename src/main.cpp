/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file src/main.cpp
 * @ingroup wbr_control
 * @brief 实现应用或测试程序的入口与初始化流程。
 * @details 主入口按依赖顺序初始化平台服务和应用模块。任一必需模块启动失败都会保留诊断信息，避免在输入或执行器未就绪时进入闭环控制。
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "modules/chassis/chassis_module.h"
#include "modules/imu/hi91_imu_module.h"
#include "modules/oscilloscope/oscilloscope_module.h"
#include "modules/referee/referee_module.h"
#include "modules/remote_input/remote_input_module.h"
#include "modules/sys_state/sys_state_module.h"
#include <channels/system_status_channel.h>
#include <platform/board/board_identity.h>

#include <hpm_iomux.h>
#include <hpm_pmic_iomux.h>
#include <hpm_soc.h>

LOG_MODULE_REGISTER(app_main, LOG_LEVEL_INF);

#if defined(CONFIG_WBR_CONTROL_RUNTIME_INIT_CAN) && CONFIG_WBR_CONTROL_RUNTIME_INIT_CAN
#include <platform/drivers/communication/can_dispatch.h>
#endif

#if defined(CONFIG_WBR_CONTROL_RUNTIME_INIT_LITTLEFS) && CONFIG_WBR_CONTROL_RUNTIME_INIT_LITTLEFS
#include <platform/storage/filesystem/littlefs_service.h>
#endif

#if defined(CONFIG_WBR_CONTROL_RUNTIME_INIT_USB) && CONFIG_WBR_CONTROL_RUNTIME_INIT_USB
#include <platform/drivers/communication/usb_session.h>
#endif

namespace
{

#if defined(CONFIG_WBR_CONTROL_UART0_OUTPUT_ALL_DIAGNOSTIC)
void ForceUart0PinRoute()
{
#if defined(CONFIG_WBR_CONTROL_UART0_OUTPUT_ALL_DIAGNOSTIC)
	/* DUST-HPM6750 UART0 uses the two-stage SOC IOC -> PMIC IOC PY06/PY07 route. */
	HPM_IOC->PAD[IOC_PAD_PY06].FUNC_CTL = IOC_PY06_FUNC_CTL_UART0_TXD;
	HPM_IOC->PAD[IOC_PAD_PY07].FUNC_CTL = IOC_PY07_FUNC_CTL_UART0_RXD;
	HPM_PIOC->PAD[IOC_PAD_PY06].FUNC_CTL = PIOC_PY06_FUNC_CTL_SOC_PY_06;
	HPM_PIOC->PAD[IOC_PAD_PY07].FUNC_CTL = PIOC_PY07_FUNC_CTL_SOC_PY_07;
#endif
}

void RawUart0Write(const char *text)
{
#if defined(CONFIG_WBR_CONTROL_UART0_OUTPUT_ALL_DIAGNOSTIC) && \
	DT_NODE_HAS_STATUS(DT_NODELABEL(uart0), okay)
	const struct device *uart0 = DEVICE_DT_GET(DT_NODELABEL(uart0));
	if (!device_is_ready(uart0)) {
		return;
	}
	while (*text != '\0') {
		uart_poll_out(uart0, static_cast<unsigned char>(*text++));
	}
#else
	ARG_UNUSED(text);
#endif
}
#endif

void EmitUart0BootProbe()
{
#if defined(CONFIG_WBR_CONTROL_UART0_OUTPUT_ALL_DIAGNOSTIC)
	ForceUart0PinRoute();
	RawUart0Write("\r\n[uart0-all] raw uart_poll_out @ 921600\r\n");
	printk("[uart0-all] direct printk @ 921600\n");
	LOG_INF("uart0-all: Zephyr logger @ 921600");
#elif defined(CONFIG_WBR_CONTROL_UART0_OUTPUT_PRINTK_AND_LOG)
	printk("[uart0-probe] printk -> logger -> uart0 @ 921600\n");
	LOG_INF("uart0 probe: LOG backend @ 921600");
#elif defined(CONFIG_WBR_CONTROL_UART0_OUTPUT_PRINTK)
	printk("[uart0-probe] direct printk -> uart0 @ 921600\n");
#elif defined(CONFIG_WBR_CONTROL_UART0_OUTPUT_LOG)
	LOG_INF("uart0 probe: LOG backend @ 921600");
#endif
}

void PublishSystemStatus(channels::BootPhase state, uint32_t module_count)
{
	const channels::SystemStatusMessage status = {
		state,
		module_count,
	};
	(void)zbus_chan_pub(&wbr_control_system_status_chan, &status, K_NO_WAIT);
}

} // namespace

/**
 * @brief 按依赖顺序启动平台服务和应用模块。
 * @return 初始化成功后线程永久休眠；启动失败时返回对应负 errno 错误码。
 */
int main(void)
{
	EmitUart0BootProbe();
	LOG_INF("wbr_control started on %s", board_identity_name());
	PublishSystemStatus(channels::kBooting, 0U);

	int rc = 0;

#if defined(CONFIG_WBR_CONTROL_RUNTIME_INIT_CAN) && CONFIG_WBR_CONTROL_RUNTIME_INIT_CAN
	if (IS_ENABLED(CONFIG_WBR_CONTROL_RUNTIME_INIT_CAN)) {
		rc = platform::InitializeCanDispatch();
		if (rc != 0) {
			if (rc == -ENODEV) {
				LOG_WRN("can_dispatch init skipped: no CAN device");
			} else {
				LOG_ERR("can_dispatch init failed: %d", rc);
				return rc;
			}
		}
	}
#endif

#if defined(CONFIG_WBR_CONTROL_RUNTIME_INIT_USB) && CONFIG_WBR_CONTROL_RUNTIME_INIT_USB
	if (IS_ENABLED(CONFIG_WBR_CONTROL_RUNTIME_INIT_USB)) {
		rc = platform::InitializeUsbSession();
		if (rc != 0) {
			if (rc == -ENODEV) {
				LOG_WRN("usb_session init skipped: no USB device");
			} else {
				LOG_ERR("usb_session init failed: %d", rc);
				return rc;
			}
		}
	}
#endif

#if defined(CONFIG_WBR_CONTROL_RUNTIME_INIT_LITTLEFS) && CONFIG_WBR_CONTROL_RUNTIME_INIT_LITTLEFS
	if (IS_ENABLED(CONFIG_WBR_CONTROL_RUNTIME_INIT_LITTLEFS)) {
		rc = platform::InitializeLittlefs();
		if (rc != 0) {
			LOG_WRN("littlefs init skipped: %d", rc);
		}
	}
#endif

	uint32_t module_count = 0U;

#if defined(CONFIG_WBR_CONTROL_MODULE_SYS_STATE) && CONFIG_WBR_CONTROL_MODULE_SYS_STATE
	{
		static modules::SysStateModule sys_state_module;
		rc = sys_state_module.Start();
		if (rc != 0) {
			LOG_ERR("module start failed: sys_state (%d)", rc);
			return rc;
		}
		++module_count;
	}
#endif
#if defined(CONFIG_WBR_CONTROL_MODULE_REMOTE_INPUT) && CONFIG_WBR_CONTROL_MODULE_REMOTE_INPUT
	{
		static modules::RemoteInputModule remote_input_module;
		rc = remote_input_module.Start();
		if (rc != 0) {
			LOG_ERR("module start failed: remote_input (%d)", rc);
			return rc;
		}
		++module_count;
	}
#endif
#if defined(CONFIG_WBR_CONTROL_MODULE_CHASSIS) && CONFIG_WBR_CONTROL_MODULE_CHASSIS
	{
		static modules::ChassisModule chassis_module;
		rc = chassis_module.Start();
		if (rc != 0) {
			LOG_ERR("module start failed: chassis (%d)", rc);
			return rc;
		}
		++module_count;
	}
#endif
#if defined(CONFIG_WBR_CONTROL_MODULE_REFEREE) && CONFIG_WBR_CONTROL_MODULE_REFEREE
	{
		static modules::RefereeModule referee_module;
		rc = referee_module.Start();
		if (rc != 0) {
			LOG_ERR("module start failed: referee (%d)", rc);
			return rc;
		}
		++module_count;
	}
#endif
#if defined(CONFIG_WBR_CONTROL_MODULE_OSCILLOSCOPE) && CONFIG_WBR_CONTROL_MODULE_OSCILLOSCOPE
	{
		static modules::OscilloscopeModule oscilloscope_module;
		rc = oscilloscope_module.Start();
		if (rc != 0) {
			LOG_ERR("module start failed: oscilloscope (%d)", rc);
			return rc;
		}
		++module_count;
	}
#endif
#if defined(CONFIG_WBR_CONTROL_MODULE_HI91_IMU) && CONFIG_WBR_CONTROL_MODULE_HI91_IMU
	{
		static modules::Hi91ImuModule hi91_imu_module;
		rc = hi91_imu_module.Start();
		if (rc != 0) {
			LOG_ERR("module start failed: hi91_imu (%d)", rc);
			return rc;
		}
		++module_count;
	}
#endif
	PublishSystemStatus(channels::kRunning, module_count);

	/* 诊断版持续重发，避免串口在启动瞬间未打开而错过唯一探针。 */
#if defined(CONFIG_WBR_CONTROL_UART0_OUTPUT_ALL_DIAGNOSTIC)
	uint32_t sequence = 0U;
	for (;;) {
		ForceUart0PinRoute();
		RawUart0Write("[uart0-all] raw uart_poll_out alive\r\n");
		printk("[uart0-all] printk alive seq=%u\n", sequence);
		LOG_INF("uart0-all: logger alive seq=%u", sequence);
		++sequence;
		k_sleep(K_SECONDS(1));
	}
#endif

	/* 初始化结束后主线程不再承担周期任务，各模块由自己的线程运行。 */
	k_sleep(K_FOREVER);
	return 0;
}
