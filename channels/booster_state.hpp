/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file channels/booster_state.hpp
 * @ingroup wbr_channels
 * @brief 定义弹仓与发射机构状态通道。
 * @details 声明跨线程交换的数据快照及其唯一全局通道。写入方发布完整对象，读取方不得保存内部存储地址；使用顺序锁的通道允许读取失败，调用方应保留上一份有效快照。
 */

#pragma once

#include <cstdint>

#include <channels/comm/seqlock_value.hpp>

namespace channels {

/** @brief 弹仓与发射机构的状态快照。 */
struct BoosterState {
	uint32_t sequence; ///< 发布序号；每产生一个新快照递增一次。
	float bullet_speed; ///< 线速度，单位为米每秒。
	uint16_t bullet_count; ///< 累计事件次数；计数达到类型上限后按实现方式饱和或回绕。
};

/** @brief 全局最新弹仓与发射机构状态快照。 */
extern SeqlockValue<BoosterState> latest_booster_state;

}  // namespace channels
