/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file src/protocols/telemetry/vofa_protocol.h
 * @ingroup wbr_protocols
 * @brief 实现 VOFA+ 调试遥测数据封装。
 * @details 接口直接处理线协议字节序列。调用方负责提供声明长度的有效缓冲区；解码失败时输出对象内容不应作为有效数据使用。
 */

#pragma once

// Public VOFA telemetry wire-format contract.

#include <cstddef>
#include <cstdint>

namespace protocols {

constexpr uint8_t kVofaJustFloatTail[4] = {0x00U, 0x00U, 0x80U, 0x7FU}; ///< VOFA+ JustFloat 协议的固定四字节帧尾。

/**
 * @brief 计算指定通道数对应的 VOFA+ JustFloat 帧长度。
 * @param channel_count 浮点采样通道数量。
 * @return 完整帧所需字节数，包含 4 字节帧尾。
 */
constexpr size_t VofaJustFloatFrameSize(size_t channel_count)
{
	return (channel_count * sizeof(float)) + sizeof(kVofaJustFloatTail);
}

/**
 * @brief 将浮点通道编码为 VOFA+ JustFloat 数据帧。
 * @param[in] channels 待编码的浮点采样通道数组。
 * @param channel_count 浮点采样通道数量。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @param out_capacity `out` 缓冲区容量，单位为字节。
 * @param[out] out_size 接收实际编码帧长度的指针；不得为空。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int EncodeVofaJustFloat(const float *channels,
		    size_t channel_count,
		    uint8_t *out,
		    size_t out_capacity,
		    size_t *out_size);

}  // namespace protocols
