/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file channels/gimbal_state.hpp
 * @ingroup wbr_channels
 * @brief 定义云台姿态与运动状态通道。
 * @details 写入方和读取方通过有界 spinlock 临界区复制完整快照，读取始终成功；调用方不得保存内部存储地址。
 */

#pragma once

#include <cstdint>

#include <channels/comm/seqlock_value.hpp>

namespace channels {

/** @brief 云台当前姿态、速度和在线状态。 */
struct GimbalState {
	uint32_t sequence; ///< 发布序号；每产生一个新快照递增一次。
	float yaw_angle; ///< 云台或自瞄目标航向角。
	float yaw_velocity; ///< 线速度，单位为米每秒。
	float pitch_angle; ///< 云台或自瞄目标俯仰角。
	float pitch_velocity; ///< 线速度，单位为米每秒。
};

/** @brief 全局最新云台姿态和在线状态快照。 */
extern SeqlockValue<GimbalState> latest_gimbal_state;

}  // namespace channels
