/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file channels/booster_state.hpp
 * @ingroup wbr_channels
 * @brief 定义弹仓与发射机构状态通道。
 * @details 写入方和读取方通过有界 spinlock 临界区复制完整快照，读取始终成功；调用方不得保存内部存储地址。
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
