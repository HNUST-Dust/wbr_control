/*
 * SPDX-License-Identifier: Apache-2.0
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
#define CONFIG_WBR_CONTROL_OSCILLOSCOPE_PERIOD_MS 10
#endif

#ifndef CONFIG_WBR_CONTROL_OSCILLOSCOPE_UART_BAUDRATE
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

int OscilloscopeModule::SendLatestSample()
{
	channels::OscilloscopeSample sample = {};
	if (!channels::latest_oscilloscope_sample.read(sample)) {
		return -EAGAIN;
	}
	if ((sample.sequence == 0U) || (sample.sequence == last_sequence_)) {
		return 0;
	}
	if (!atomic_cas(&tx_busy_, 0, 1)) {
		return -EBUSY;
	}

	const size_t channel_count =
		MIN(static_cast<size_t>(sample.channel_count), channels::kOscilloscopeMaxChannels);
	size_t frame_size = 0U;
	const int rc = protocols::EncodeVofaJustFloat(sample.value, channel_count, tx_frame_,
						      sizeof(tx_frame_), &frame_size);
	if (rc != 0) {
		atomic_clear(&tx_busy_);
		return rc;
	}

	/*
	 * HPM6750 使用回写式数据缓存，而 Zephyr DMA 驱动不会维护缓存。
	 * 在 XDMA 读取编码后的 VOFA 帧前，必须刷新覆盖该帧的完整缓存行。
	 */
	const uint32_t frame_address =
		static_cast<uint32_t>(reinterpret_cast<uintptr_t>(tx_frame_));
	const uint32_t cache_start = HPM_L1C_CACHELINE_ALIGN_DOWN(frame_address);
	const uint32_t cache_end =
		HPM_L1C_CACHELINE_ALIGN_UP(frame_address + static_cast<uint32_t>(frame_size));
	l1c_dc_flush(cache_start, cache_end - cache_start);

	const int tx_rc = uart_tx(uart_dev_, tx_frame_, frame_size, SYS_FOREVER_US);
	if (tx_rc != 0) {
		atomic_clear(&tx_busy_);
		return tx_rc;
	}
	last_sequence_ = sample.sequence;
	return 0;
}

} // namespace modules
