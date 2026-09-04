/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file channels/oscilloscope_sample.hpp
 * @ingroup wbr_channels
 * @brief 定义调试示波器采样数据通道。
 * @details 声明跨线程交换的数据快照及其唯一全局通道。写入方发布完整对象，读取方不得保存内部存储地址；使用顺序锁的通道允许读取失败，调用方应保留上一份有效快照。
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include <channels/comm/seqlock_value.hpp>

namespace channels {

constexpr size_t kOscilloscopeMaxChannels = 20U; ///< 单帧示波器遥测允许的最大浮点通道数。

/** @brief 发送给调试示波器的一组采样通道。 */
struct OscilloscopeSample {
	uint32_t sequence; ///< 发布序号；每产生一个新快照递增一次。
	uint32_t uptime_ms; ///< 时间长度，单位为毫秒。
	uint8_t channel_count; ///< 累计事件次数；计数达到类型上限后按实现方式饱和或回绕。
	float value[kOscilloscopeMaxChannels]; ///< 通道或协议字段携带的数值。
};

/** @brief 全局最新调试示波器采样快照。 */
extern SeqlockValue<OscilloscopeSample> latest_oscilloscope_sample;

}  // namespace channels
