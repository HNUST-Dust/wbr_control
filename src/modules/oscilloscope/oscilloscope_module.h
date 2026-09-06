/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file src/modules/oscilloscope/oscilloscope_module.h
 * @ingroup wbr_modules
 * @brief 实现控制变量采样与示波器遥测模块。
 * @details 模块遵循 `ModuleBase` 生命周期：`Start()` 只负责一次性资源初始化和线程创建，`RunLoop()` 持有周期状态。跨线程数据通过 channels 层交换。
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "../module_base.h"
#include <channels/oscilloscope_sample.hpp>
#include <protocols/telemetry/vofa_protocol.h>

namespace modules
{

/** @brief 周期采样控制数据并发送 VOFA+ 遥测。 */
class OscilloscopeModule : public ModuleBase
{
public:
	OscilloscopeModule() = default;
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
	 * @brief 通过 UART0 async/DMA 发送一帧固定的 20 通道启动探针。
	 * @return 成功返回 0，编码或底层 UART 发送失败时返回负 errno 错误码。
	 */
	int SendBootProbe();
	/**
	 * @brief 编码、刷新缓存并异步发送一组 JustFloat 通道。
	 * @param[in] values 待发送的浮点通道。
	 * @param channel_count 通道数量。
	 * @return 成功返回 0，UART 忙、编码或发送失败时返回负 errno 错误码。
	 */
	int TransmitFrame(const float *values, size_t channel_count);
	/**
	 * @brief 处理 Zephyr UART 异步驱动回调事件。
	 * @param[in] dev 产生事件的 Zephyr 设备实例。
	 * @param[in,out] event UART 异步事件描述。
	 * @param[in,out] user_data 注册回调时绑定的模块实例指针。
	 */
	static void UartCallback(const struct device *dev, struct uart_event *event,
				 void *user_data);
	/**
	 * @brief 编码并发送最新的示波器采样快照。
	 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
	 */
	int SendLatestSample();

	const struct device *uart_dev_ = nullptr;
	uint32_t last_sequence_ = 0U;
	uint32_t last_probe_ms_ = 0U;
	uint32_t tx_start_ms_ = 0U;
	uint32_t tx_timeout_count_ = 0U;
	uint32_t missed_release_count_ = 0U;
	atomic_t tx_busy_ = ATOMIC_INIT(0);
	uint8_t tx_frame_[protocols::VofaJustFloatFrameSize(channels::kOscilloscopeMaxChannels)] =
		{};
};

} // namespace modules
