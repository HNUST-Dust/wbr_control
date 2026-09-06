/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file channels/pc_auto_aim_command.hpp
 * @ingroup wbr_channels
 * @brief 定义上位机自瞄控制指令通道。
 * @details 写入方和读取方通过有界 spinlock 临界区复制完整快照，读取始终成功；调用方不得保存内部存储地址。
 */

#pragma once

#include <cstdint>

#include <channels/comm/seqlock_value.hpp>

namespace channels {

/*
 * Auto-aim command decoded from a PC link packet and published for the
 * gimbal / booster modules to consume.
 */
/** @brief 上位机下发的自瞄目标与控制标志。 */
struct PcAutoAimCommand {
	uint32_t sequence; ///< 发布序号；每产生一个新快照递增一次。
	uint8_t mode; ///< 控制模式：0 空闲，1 仅瞄准，2 瞄准并发射。
	float yaw_angle; ///< 云台或自瞄目标航向角。
	float yaw_velocity; ///< 目标偏航角速度，单位为弧度每秒。
	float yaw_acceleration; ///< 航向角加速度估计，单位为弧度每二次方秒。
	float pitch_angle; ///< 云台或自瞄目标俯仰角。
	float pitch_velocity; ///< 目标俯仰角速度，单位为弧度每秒。
	float pitch_acceleration; ///< 俯仰角加速度估计，单位为弧度每二次方秒。
};

/** @brief 全局最新上位机自瞄命令快照。 */
extern SeqlockValue<PcAutoAimCommand> latest_pc_auto_aim_command;

}  // namespace channels
