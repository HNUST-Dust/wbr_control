/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <modules/oscilloscope/oscilloscope_module.h>

#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <channels/oscilloscope_sample.hpp>
#include <modules/thread_utils.h>
#include <protocols/telemetry/vofa_protocol.h>

LOG_MODULE_REGISTER(oscilloscope_module, LOG_LEVEL_INF);

namespace {

K_THREAD_STACK_DEFINE(g_oscilloscope_module_stack, 1536);

constexpr uint32_t kOutputPeriodMs = CONFIG_RM_TEST_OSCILLOSCOPE_PERIOD_MS;
constexpr uint32_t kUartBaudrate = CONFIG_RM_TEST_OSCILLOSCOPE_UART_BAUDRATE;

const struct device *FindOutputUart()
{
#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart0), okay)
	return DEVICE_DT_GET(DT_NODELABEL(uart0));
#else
	return nullptr;
#endif
}

void UartWrite(const struct device *dev, const uint8_t *data, size_t size)
{
	for (size_t i = 0U; i < size; ++i) {
		uart_poll_out(dev, data[i]);
	}
}

}  // namespace

namespace modules::oscilloscope {

int OscilloscopeModule::Initialize()
{
	started_ = false;
	last_sequence_ = 0U;
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

	return 0;
}

int OscilloscopeModule::Start()
{
	if (started_) {
		return 0;
	}

	::modules::StartMemberThread<OscilloscopeModule, &OscilloscopeModule::RunLoop>(
		&thread_,
		g_oscilloscope_module_stack,
		K_THREAD_STACK_SIZEOF(g_oscilloscope_module_stack),
		this,
		K_PRIO_PREEMPT(9),
		"oscilloscope_module");
	started_ = true;
	return 0;
}

void OscilloscopeModule::RunLoop()
{
	LOG_INF("oscilloscope module started uart=%s baud=%u period=%u ms",
		uart_dev_->name,
		static_cast<unsigned int>(kUartBaudrate),
		static_cast<unsigned int>(kOutputPeriodMs));

	for (;;) {
		(void)SendLatestSample();
		k_sleep(K_MSEC(kOutputPeriodMs));
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

	const size_t channel_count = MIN(static_cast<size_t>(sample.channel_count),
					 channels::kOscilloscopeMaxChannels);
	uint8_t frame[protocols::telemetry::vofa::JustFloatFrameSize(
		channels::kOscilloscopeMaxChannels)] = {};
	size_t frame_size = 0U;
	const int rc = protocols::telemetry::vofa::EncodeJustFloat(
		sample.value, channel_count, frame, sizeof(frame), &frame_size);
	if (rc != 0) {
		return rc;
	}

	UartWrite(uart_dev_, frame, frame_size);
	last_sequence_ = sample.sequence;
	return 0;
}

}  // namespace modules::oscilloscope
