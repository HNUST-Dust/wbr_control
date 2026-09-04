/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file src/modules/remote_input/remote_input_module.h
 * @ingroup wbr_modules
 * @brief 实现多种遥控协议的解析与统一输入发布。
 * @details 模块遵循 `ModuleBase` 生命周期：`Start()` 只负责一次性资源初始化和线程创建，`RunLoop()` 持有周期状态。跨线程数据通过 channels 层交换。
 */

#pragma once

#include <stddef.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include <channels/remote_input_state.hpp>
#include <channels/uart_raw_frame_queue.h>
#include "../module_base.h"

struct device;
struct uart_event;

namespace modules
{

/** @brief 接收多种遥控协议并发布统一输入状态。 */
class RemoteInputModule : public ModuleBase
{
public:
	RemoteInputModule() = default;
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
	static constexpr size_t kLineBufSize = 96U; ///< 文本调试输入单行缓冲区容量，单位为字节。
	static constexpr size_t kBinaryBufSize = 256U; ///< 二进制遥控帧拼接缓冲区容量，单位为字节。

	/**
	 * @brief 处理 Zephyr UART 异步驱动回调事件。
	 * @param[in] dev 产生事件的 Zephyr 设备实例。
	 * @param[in,out] evt UART 异步事件描述。
	 * @param[in,out] user_data 注册回调时绑定的模块实例指针。
	 */
	static void UartCallback(const struct device *dev, struct uart_event *evt, void *user_data);
	/**
	 * @brief 配置并启动遥控串口异步接收。
	 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
	 */
	int StartUartRx();
	/**
	 * @brief 在模块实例上下文中处理 UART 事件。
	 * @param[in] dev 产生事件的 Zephyr 设备实例。
	 * @param[in] evt UART 异步事件描述。
	 */
	void HandleUartEvent(const struct device *dev, const struct uart_event *evt);
	/**
	 * @brief 从 UART 环形缓冲区取出并解析遥控数据。
	 */
	void DecodeUartBytesFromRing();
	/**
	 * @brief 解析文本形式的遥控调试输入。
	 * @param[in] line 以空字符结尾的文本遥控输入。
	 * @param[out] out 接收结果的输出对象；不得为空。
	 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
	 */
	int ParseLine(const char *line, channels::RemoteInputState *out);
	/**
	 * @brief 从累计缓冲区提取并解析二进制遥控帧。
	 */
	void TryDecodeBinaryFrames();
	/**
	 * @brief 从二进制累计缓冲区头部移除字节。
	 * @param bytes 从累计缓冲区消费的字节数。
	 */
	void ConsumeBinary(size_t bytes);
	/**
	 * @brief 检测天地飞遥控帧是否已经超时丢失。
	 */
	void CheckWflyFrameLostTimeout();
	/**
	 * @brief 规范化并发布最新遥控输入状态。
	 * @param[in,out] input 本周期使用的只读输入快照。
	 * @param clear_wfly_frame_lost 为 `true` 时清除天地飞协议的丢帧锁存状态。
	 */
	void PublishRemoteState(channels::RemoteInputState *input,
				bool clear_wfly_frame_lost = true);

	const struct device *uart_dev_ = nullptr;
	bool uart_rx_started_ = false;
	uint32_t publish_sequence_ = 0U;
	uint32_t rx_drop_count_ = 0U;
	uint32_t rx_stop_count_ = 0U;
	uint32_t frame_lost_count_ = 0U;
	uint32_t wfly_frame_lost_start_ms_ = 0U;
	bool wfly_frame_lost_active_ = false;
	bool wfly_frame_lost_disabled_published_ = false;
	uint8_t next_rx_buffer_index_ = 1U;
	char line_buf_[kLineBufSize] = {0};
	size_t line_pos_ = 0U;
	uint8_t binary_buf_[kBinaryBufSize] = {0};
	size_t binary_len_ = 0U;
};

} // namespace modules
