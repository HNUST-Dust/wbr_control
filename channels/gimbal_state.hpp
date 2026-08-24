/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file channels/gimbal_state.hpp
 * @ingroup wbr_channels
 * @brief 定义云台姿态与运动状态通道。
 * @details 声明跨线程交换的数据快照及其唯一全局通道。写入方发布完整对象，读取方不得保存内部存储地址；使用顺序锁的通道允许读取失败，调用方应保留上一份有效快照。
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
