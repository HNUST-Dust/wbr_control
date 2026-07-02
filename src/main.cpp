/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <channels/system_status_channel.h>
#include <modules/chassis/chassis_module.h>
#include <modules/gimbal/gimbal_module.h>
#include <modules/imu/hi91_imu_module.h>
#include <modules/oscilloscope/oscilloscope_module.h>
#include <modules/referee/referee_module.h>
#include <modules/remote_input/remote_input_module.h>
#include <modules/sys_state/sys_state_module.h>
#include <platform/board/board_identity.h>

LOG_MODULE_REGISTER(app_main, LOG_LEVEL_INF);

#if defined(CONFIG_RM_TEST_RUNTIME_INIT_CAN) && CONFIG_RM_TEST_RUNTIME_INIT_CAN
#include <platform/drivers/communication/can_dispatch.h>
#endif

#if defined(CONFIG_RM_TEST_RUNTIME_INIT_LITTLEFS) && CONFIG_RM_TEST_RUNTIME_INIT_LITTLEFS
#include <platform/storage/filesystem/littlefs_service.h>
#endif

#if defined(CONFIG_RM_TEST_RUNTIME_INIT_UART) && CONFIG_RM_TEST_RUNTIME_INIT_UART
#include <platform/drivers/communication/uart_dispatch.h>
#endif

#if defined(CONFIG_RM_TEST_RUNTIME_INIT_USB) && CONFIG_RM_TEST_RUNTIME_INIT_USB
#include <platform/drivers/communication/usb_session.h>
#endif

namespace {

modules::sys_state::SysStateModule g_sys_state_module;
modules::remote_input::RemoteInputModule g_remote_input_module;
modules::chassis::ChassisModule g_chassis_module;
modules::gimbal::GimbalModule g_gimbal_module;
modules::referee::RefereeModule g_referee_module;
modules::imu::Hi91ImuModule g_hi91_imu_module;
modules::oscilloscope::OscilloscopeModule g_oscilloscope_module;

}  // namespace

int main(void)
{
	using channels::SystemStatusMessage;

	LOG_INF("rm_test started on %s", board_identity_name());

	const SystemStatusMessage booting_status = {
		channels::kBooting,
		0U,
	};

	(void)zbus_chan_pub(&rm_test_system_status_chan, &booting_status, K_NO_WAIT);

	int rc = 0;

#if defined(CONFIG_RM_TEST_RUNTIME_INIT_UART) && CONFIG_RM_TEST_RUNTIME_INIT_UART
	if (IS_ENABLED(CONFIG_RM_TEST_RUNTIME_INIT_UART)) {
		rc = platform::drivers::communication::uart_dispatch::Initialize();
		if (rc != 0) {
			LOG_ERR("uart_dispatch init failed: %d", rc);
			return rc;
		}
	}
#endif

#if defined(CONFIG_RM_TEST_RUNTIME_INIT_CAN) && CONFIG_RM_TEST_RUNTIME_INIT_CAN
	if (IS_ENABLED(CONFIG_RM_TEST_RUNTIME_INIT_CAN)) {
		rc = platform::drivers::communication::can_dispatch::Initialize();
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

#if defined(CONFIG_RM_TEST_RUNTIME_INIT_USB) && CONFIG_RM_TEST_RUNTIME_INIT_USB
	if (IS_ENABLED(CONFIG_RM_TEST_RUNTIME_INIT_USB)) {
		rc = platform::drivers::communication::usb_session::Initialize();
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

#if defined(CONFIG_RM_TEST_RUNTIME_INIT_LITTLEFS) && CONFIG_RM_TEST_RUNTIME_INIT_LITTLEFS
	if (IS_ENABLED(CONFIG_RM_TEST_RUNTIME_INIT_LITTLEFS)) {
		rc = platform::storage::filesystem::littlefs_service::Initialize();
		if (rc != 0) {
			LOG_WRN("littlefs init skipped: %d", rc);
		}
	}
#endif

	uint32_t module_count = 0U;

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_SYS_STATE)) {
		rc = g_sys_state_module.Initialize();
		if (rc != 0) {
			LOG_ERR("module init failed: %s (%d)", g_sys_state_module.Name(), rc);
			return rc;
		}
		++module_count;
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_REMOTE_INPUT)) {
		rc = g_remote_input_module.Initialize();
		if (rc != 0) {
			LOG_ERR("module init failed: %s (%d)", g_remote_input_module.Name(), rc);
			return rc;
		}
		++module_count;
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_CHASSIS)) {
		rc = g_chassis_module.Initialize();
		if (rc != 0) {
			LOG_ERR("module init failed: %s (%d)", g_chassis_module.Name(), rc);
			return rc;
		}
		++module_count;
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_GIMBAL)) {
		rc = g_gimbal_module.Initialize();
		if (rc != 0) {
			LOG_ERR("module init failed: %s (%d)", g_gimbal_module.Name(), rc);
			return rc;
		}
		++module_count;
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_REFEREE)) {
		rc = g_referee_module.Initialize();
		if (rc != 0) {
			LOG_ERR("module init failed: %s (%d)", g_referee_module.Name(), rc);
			return rc;
		}
		++module_count;
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_HI91_IMU)) {
		rc = g_hi91_imu_module.Initialize();
		if (rc != 0) {
			LOG_ERR("module init failed: %s (%d)", g_hi91_imu_module.Name(), rc);
			return rc;
		}
		++module_count;
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_OSCILLOSCOPE)) {
		rc = g_oscilloscope_module.Initialize();
		if (rc != 0) {
			LOG_ERR("module init failed: %s (%d)", g_oscilloscope_module.Name(), rc);
			return rc;
		}
		++module_count;
	}

	const SystemStatusMessage initialized_status = {
		channels::kModulesInitialized,
		module_count,
	};

	(void)zbus_chan_pub(&rm_test_system_status_chan, &initialized_status, K_NO_WAIT);

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_SYS_STATE)) {
		rc = g_sys_state_module.Start();
		if (rc != 0) {
			LOG_ERR("module start failed: %s (%d)", g_sys_state_module.Name(), rc);
			return rc;
		}
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_REMOTE_INPUT)) {
		rc = g_remote_input_module.Start();
		if (rc != 0) {
			LOG_ERR("module start failed: %s (%d)", g_remote_input_module.Name(), rc);
			return rc;
		}
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_CHASSIS)) {
		rc = g_chassis_module.Start();
		if (rc != 0) {
			LOG_ERR("module start failed: %s (%d)", g_chassis_module.Name(), rc);
			return rc;
		}
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_GIMBAL)) {
		rc = g_gimbal_module.Start();
		if (rc != 0) {
			LOG_ERR("module start failed: %s (%d)", g_gimbal_module.Name(), rc);
			return rc;
		}
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_REFEREE)) {
		rc = g_referee_module.Start();
		if (rc != 0) {
			LOG_ERR("module start failed: %s (%d)", g_referee_module.Name(), rc);
			return rc;
		}
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_OSCILLOSCOPE)) {
		rc = g_oscilloscope_module.Start();
		if (rc != 0) {
			LOG_ERR("module start failed: %s (%d)", g_oscilloscope_module.Name(), rc);
			return rc;
		}
	}

	if (IS_ENABLED(CONFIG_RM_TEST_MODULE_HI91_IMU)) {
		rc = g_hi91_imu_module.Start();
		if (rc != 0) {
			LOG_ERR("module start failed: %s (%d)", g_hi91_imu_module.Name(), rc);
			return rc;
		}
	}

	const SystemStatusMessage running_status = {
		channels::kRunning,
		module_count,
	};

	(void)zbus_chan_pub(&rm_test_system_status_chan, &running_status, K_NO_WAIT);

	while (true) {
		k_sleep(K_SECONDS(1));
	}
}
