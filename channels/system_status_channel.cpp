/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file channels/system_status_channel.cpp
 * @ingroup wbr_channels
 * @brief 定义系统运行状态与故障信息通道。
 * @details 本文件仅定义对应通道的全局存储实例，不启动线程、不访问硬件，也不改变消息字段。
 */

#include <channels/system_status_channel.h>

ZBUS_CHAN_DEFINE(wbr_control_system_status_chan,
		 channels::SystemStatusMessage,
		 NULL,
		 NULL,
		 ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(.phase = channels::kBooting,
			       .active_modules = 0U));
