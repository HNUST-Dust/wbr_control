/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "modules/chassis/chassis_module.h"
#include "modules/imu/hi91_imu_module.h"
#include "modules/oscilloscope/oscilloscope_module.h"
#include "modules/referee/referee_module.h"
#include "modules/remote_input/remote_input_module.h"
#include "modules/sys_state/sys_state_module.h"
#include <channels/system_status_channel.h>
#include <platform/board/board_identity.h>

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

void PublishSystemStatus(channels::BootPhase state, uint32_t module_count)
{
	const channels::SystemStatusMessage status = {
		state,
		module_count,
	};
	(void)zbus_chan_pub(&wbr_control_system_status_chan, &status, K_NO_WAIT);
}

} // namespace

int main(void)
{
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

	while (true) {
		k_sleep(K_SECONDS(1));
	}
}
