/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file src/modules/oscilloscope/oscilloscope_module.cpp
 * @ingroup wbr_modules
 * @brief 实现控制变量采样与示波器遥测模块。
 * @details 实现运行在模块自有 Zephyr 线程或其驱动回调中。回调路径只完成有界的数据搬运和通知，耗时解析与控制计算留在线程上下文执行。
 */

#include "oscilloscope_module.h"

#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <channels/oscilloscope_sample.hpp>
#include <protocols/telemetry/vofa_protocol.h>
#include <scheduling/periodic_schedule.h>
#include <scheduling/thread_priorities.h>

#include <hpm_l1c_drv.h>

LOG_MODULE_REGISTER(oscilloscope_module, LOG_LEVEL_INF);

namespace
{

K_THREAD_STACK_DEFINE(g_oscilloscope_module_stack, 1536);

#ifndef CONFIG_WBR_CONTROL_OSCILLOSCOPE_PERIOD_MS
/** @brief 未通过 Kconfig 配置时使用的示波器默认采样周期，单位为毫秒。 */
#define CONFIG_WBR_CONTROL_OSCILLOSCOPE_PERIOD_MS 10
#endif

#ifndef CONFIG_WBR_CONTROL_OSCILLOSCOPE_UART_BAUDRATE
/** @brief 未通过 Kconfig 配置时使用的示波器串口默认波特率。 */
#define CONFIG_WBR_CONTROL_OSCILLOSCOPE_UART_BAUDRATE 921600
#endif

constexpr uint32_t kOutputPeriodMs = CONFIG_WBR_CONTROL_OSCILLOSCOPE_PERIOD_MS;
constexpr uint32_t kUartBaudrate = CONFIG_WBR_CONTROL_OSCILLOSCOPE_UART_BAUDRATE;

const struct device *FindOutputUart()
{
#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart0), okay)
	return DEVICE_DT_GET(DT_NODELABEL(uart0));
#else
	return nullptr;
#endif
}

} // namespace

namespace modules
{

int OscilloscopeModule::Start()
{
	if (started_) {
		return 0;
	}

	uart_dev_ = FindOutputUart();
	if ((uart_dev_ == nullptr) || !device_is_ready(uart_dev_)) {
		return -ENODEV;
	}

	struct uart_config config = {};
	config.baudrate = kUartBaudrate;
	config.parity = UART_CFG_PARITY_NONE;
	config.stop_bits = UART_CFG_STOP_BITS_1;
	config.data_bits = UART_CFG_DATA_BITS_8;
	config.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
	const int rc = uart_configure(uart_dev_, &config);
	if (rc != 0) {
		LOG_ERR("uart configure failed: %d", rc);
		return rc;
	}
	const int callback_rc =
		uart_callback_set(uart_dev_, &OscilloscopeModule::UartCallback, this);
	if (callback_rc != 0) {
		LOG_ERR("uart async callback failed: %d", callback_rc);
		return callback_rc;
	}
	const int probe_rc = SendBootProbe();
	if (probe_rc != 0) {
		LOG_ERR("uart async boot probe failed: %d", probe_rc);
		return probe_rc;
	}

	return CreateThread(
		g_oscilloscope_module_stack, K_THREAD_STACK_SIZEOF(g_oscilloscope_module_stack),
		K_PRIO_PREEMPT(wbr_control::scheduling::thread_priority::kOscilloscope),
		"oscilloscope_module");
}

void OscilloscopeModule::RunLoop()
{
	LOG_INF("oscilloscope module started uart=%s baud=%u period=%u ms", uart_dev_->name,
		static_cast<unsigned int>(kUartBaudrate),
		static_cast<unsigned int>(kOutputPeriodMs));

	wbr_control::scheduling::AbsolutePeriodicSchedule release(
		kOutputPeriodMs, wbr_control::scheduling::thread_phase_ms::kOscilloscope);
	for (;;) {
		(void)release.WaitForNextRelease();
		missed_release_count_ = release.total_missed_releases();
		(void)SendLatestSample();
	}
}

void OscilloscopeModule::UartCallback(const struct device *dev, struct uart_event *event,
				      void *user_data)
{
	ARG_UNUSED(dev);
	auto *module = static_cast<OscilloscopeModule *>(user_data);
	if (module == nullptr || event == nullptr) {
		return;
	}
	if (event->type == UART_TX_DONE || event->type == UART_TX_ABORTED) {
		atomic_clear(&module->tx_busy_);
	}
}

int OscilloscopeModule::SendBootProbe()
{
	float probe[channels::kOscilloscopeMaxChannels] = {};
	probe[0] = 6750.0F;
	probe[1] = static_cast<float>(kUartBaudrate);
	probe[2] = static_cast<float>(kOutputPeriodMs);
	probe[3] = -1.0F;
	return TransmitFrame(probe, ARRAY_SIZE(probe));
}

int OscilloscopeModule::TransmitFrame(const float *values, size_t channel_count)
{
	if (!atomic_cas(&tx_busy_, 0, 1)) {
		return -EBUSY;
	}

	size_t frame_size = 0U;
	const int rc = protocols::EncodeVofaJustFloat(values, channel_count, tx_frame_,
						      sizeof(tx_frame_), &frame_size);
	if (rc != 0) {
		atomic_clear(&tx_busy_);
		return rc;
	}

	/* UART0 XDMA 从回写式缓存读取帧之前必须刷新完整缓存行。 */
	const uint32_t frame_address =
		static_cast<uint32_t>(reinterpret_cast<uintptr_t>(tx_frame_));
	const uint32_t cache_start = HPM_L1C_CACHELINE_ALIGN_DOWN(frame_address);
	const uint32_t cache_end =
		HPM_L1C_CACHELINE_ALIGN_UP(frame_address + static_cast<uint32_t>(frame_size));
	l1c_dc_flush(cache_start, cache_end - cache_start);

	const int tx_rc = uart_tx(uart_dev_, tx_frame_, frame_size, SYS_FOREVER_US);
	if (tx_rc != 0) {
		atomic_clear(&tx_busy_);
	}
	return tx_rc;
}

int OscilloscopeModule::SendLatestSample()
{
	channels::OscilloscopeSample sample = {};
	if (!channels::latest_oscilloscope_sample.read(sample)) {
#if defined(CONFIG_WBR_CONTROL_UART0_OUTPUT_ALL_DIAGNOSTIC)
		return SendBootProbe();
#else
		return -EAGAIN;
#endif
	}
	if ((sample.sequence == 0U) || (sample.sequence == last_sequence_)) {
		return 0;
	}

	const size_t channel_count =
		MIN(static_cast<size_t>(sample.channel_count), channels::kOscilloscopeMaxChannels);
	const int tx_rc = TransmitFrame(sample.value, channel_count);
	if (tx_rc != 0) {
		return tx_rc;
	}
	last_sequence_ = sample.sequence;
	return 0;
}

} // namespace modules
