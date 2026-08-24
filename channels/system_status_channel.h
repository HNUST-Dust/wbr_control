/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file channels/system_status_channel.h
 * @ingroup wbr_channels
 * @brief 定义系统运行状态与故障信息通道。
 * @details 声明跨线程交换的数据快照及其唯一全局通道。写入方发布完整对象，读取方不得保存内部存储地址；使用顺序锁的通道允许读取失败，调用方应保留上一份有效快照。
 */

#pragma once

#include <stdint.h>
#include <zephyr/zbus/zbus.h>

namespace channels {

/** @brief 系统启动流程所处的阶段。 */
enum BootPhase : uint8_t {
	kBooting = 0, ///< 启动和外设初始化仍在进行。
	kRunning = 1, ///< 必需模块已启动，系统进入正常运行阶段。
};

/** @brief 系统状态通道发布的诊断消息。 */
struct SystemStatusMessage {
	BootPhase phase; ///< 系统当前启动阶段。
	uint32_t active_modules; ///< 已经成功启动的应用模块数量。
};

}  // namespace channels

/** @brief 发布系统启动阶段和已启动模块数量的 Zbus 通道。 */
ZBUS_CHAN_DECLARE(wbr_control_system_status_chan);
