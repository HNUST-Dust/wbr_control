/* SPDX-License-Identifier: Apache-2.0 */

/**
* @file src/modules/referee/referee_module.h
 * @ingroup wbr_modules
 * @brief 实现裁判系统数据接收与状态发布模块。
 * @details 模块遵循 `ModuleBase` 生命周期：`Start()` 只负责一次性资源初始化和线程创建，`RunLoop()` 持有周期状态。跨线程数据通过 channels 层交换。
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#include "../module_base.h"
#include <protocols/referee/referee_protocol.h>

namespace modules
{

/** @brief 接收并解析裁判系统数据的应用模块。 */
class RefereeModule : public ModuleBase
{
public:
	RefereeModule() = default;
	/**
	 * @brief 初始化模块资源并创建工作线程。
	 * @return 成功返回 0；初始化或线程创建失败返回负 errno 错误码。
	 */
	int Start() override;
	/**
	 * @brief 执行模块线程的周期主循环。
	 */
	void RunLoop() override;

private:
	/**
	 * @brief 取出 UART 队列中的原始帧并送入流式解析器。
	 */
	void DecodeUartFramesInQueue();
	/**
	 * @brief 向裁判系统流式缓冲区追加接收字节。
	 * @param[in] data 输入字节缓冲区；长度由相邻长度参数给出。
	 * @param len 输入缓冲区中的有效字节数。
	 */
	void FeedBytes(const uint8_t *data, size_t len);
	/**
	 * @brief 从流式缓冲区中提取所有完整裁判协议帧。
	 */
	void TryParseStream();
	/**
	 * @brief 从流式缓冲区头部移除指定数量的字节。
	 * @param n 从流式缓冲区消费的字节数。
	 */
	void ConsumeStreamBytes(size_t n);
	/**
	 * @brief 解码并处理一帧完整的裁判系统数据。
	 * @param[in] frame 待入队、解码或处理的数据帧；不得为空。
	 * @param frame_len 完整协议帧长度，单位为字节。
	 */
	void HandleFrame(const uint8_t *frame, size_t frame_len);

	uint32_t sequence_ = 0U;
	uint8_t stream_buf_[protocols::kRefereeMaxFrameSize] = {};
	size_t stream_len_ = 0U;
	protocols::RefereeGameStatus game_status_ = {};
	protocols::RefereeRobotStatus robot_status_ = {};
	protocols::RefereeShootData shoot_data_ = {};
};

} // namespace modules
