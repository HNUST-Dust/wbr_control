/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <modules/imu/hi91_imu_module.h>

#include <errno.h>
#include <cmath>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <hpm_l1c_drv.h>

#include <channels/hi91_imu_sample.hpp>
#include <modules/thread_utils.h>

LOG_MODULE_REGISTER(hi91_imu_module, LOG_LEVEL_INF);

namespace {

K_THREAD_STACK_DEFINE(g_hi91_imu_module_stack, 2048);

#if defined(CONFIG_RM_TEST_HI91_IMU_UART_BAUDRATE)
constexpr uint32_t kUartBaudrate = CONFIG_RM_TEST_HI91_IMU_UART_BAUDRATE;
#else
constexpr uint32_t kUartBaudrate = 921600U;
#endif
constexpr bool kStrictCrc = IS_ENABLED(CONFIG_RM_TEST_HI91_IMU_STRICT_CRC);
constexpr int32_t kRxIdleTimeoutUs = 1000;
constexpr uint32_t kStartupDelayMs = 2000U;
/* HI91 pitch is a physical Euler pitch angle.  Values outside this envelope
 * are corrupted UART payloads, not a pose the balancing chassis can use. */
constexpr float kMaximumValidPitchDeg = 100.0F;

const struct device *FindInputUart()
{
#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart2), okay)
	return DEVICE_DT_GET(DT_NODELABEL(uart2));
#else
	return nullptr;
#endif
}

}  // namespace

namespace modules {

int Hi91ImuModule::Initialize()
{
	started_ = false;
	uart_dev_ = FindInputUart();
	sample_sequence_ = 0U;
	parse_error_count_ = 0U;
	rx_drop_count_ = 0U;
	rx_stop_count_ = 0U;
	next_rx_buffer_index_ = 1U;
	k_sem_init(&rx_sem_, 0, 1);
	ring_buf_init(&rx_ring_, sizeof(rx_ring_storage_), rx_ring_storage_);
	parser_.Reset();
	parser_.SetStrictCrc(kStrictCrc);

	if ((uart_dev_ == nullptr) || !device_is_ready(uart_dev_)) {
		return -ENODEV;
	}

	struct uart_config config = {};
	config.baudrate = kUartBaudrate;
	config.parity = UART_CFG_PARITY_NONE;
	config.stop_bits = UART_CFG_STOP_BITS_1;
	config.data_bits = UART_CFG_DATA_BITS_8;
	config.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
	int rc = uart_configure(uart_dev_, &config);
	if (rc != 0) {
		LOG_ERR("uart configure failed: %d", rc);
		return rc;
	}

	return 0;
}

int Hi91ImuModule::Start()
{
	if (started_) {
		return 0;
	}

	::modules::StartMemberThread<Hi91ImuModule, &Hi91ImuModule::RunLoop>(
		&thread_,
		g_hi91_imu_module_stack,
		K_THREAD_STACK_SIZEOF(g_hi91_imu_module_stack),
		this,
		/* IMU parsing must preempt the 1 kHz chassis loop.  Otherwise an
		 * unusually expensive control cycle can delay sample publication long
		 * enough to trip the chassis freshness gate. */
		K_PRIO_PREEMPT(7),
		"hi91_imu_module");
	started_ = true;
	return 0;
}

void Hi91ImuModule::RunLoop()
{
	LOG_INF("hi91 imu async module started uart=%s baud=%u strict_crc=%u",
		uart_dev_->name,
		static_cast<unsigned int>(kUartBaudrate),
		static_cast<unsigned int>(kStrictCrc));

	k_sleep(K_MSEC(kStartupDelayMs));

	int rc = uart_callback_set(uart_dev_, UartCallback, this);
	if (rc != 0) {
		LOG_ERR("uart callback set failed: %d", rc);
		return;
	}

	rc = uart_rx_enable(uart_dev_, rx_buffers_[0], sizeof(rx_buffers_[0]), kRxIdleTimeoutUs);
	if (rc != 0) {
		LOG_ERR("uart rx enable failed: %d", rc);
		return;
	}

	uint8_t buffer[kRxBufferSize];
	for (;;) {
		k_sem_take(&rx_sem_, K_FOREVER);

		uint32_t count = 0U;
		do {
			count = ring_buf_get(&rx_ring_, buffer, sizeof(buffer));
			ProcessBytes(buffer, count);
		} while (count != 0U);
	}
}

void Hi91ImuModule::UartCallback(const struct device *dev, struct uart_event *evt, void *user_data)
{
	auto *self = static_cast<Hi91ImuModule *>(user_data);
	if (self != nullptr) {
		self->HandleUartEvent(dev, evt);
	}
}

void Hi91ImuModule::HandleUartEvent(const struct device *dev, const struct uart_event *evt)
{
	switch (evt->type) {
	case UART_RX_RDY: {
		const uint8_t *data = evt->data.rx.buf + evt->data.rx.offset;
		InvalidateDmaRxCache(data, evt->data.rx.len);
		const uint32_t written = ring_buf_put(&rx_ring_, data, evt->data.rx.len);
		if (written != evt->data.rx.len) {
			++rx_drop_count_;
		}
		k_sem_give(&rx_sem_);
		break;
	}
	case UART_RX_BUF_REQUEST: {
		const uint8_t index = next_rx_buffer_index_;
		next_rx_buffer_index_ = (next_rx_buffer_index_ + 1U) % kRxBufferCount;
		(void)uart_rx_buf_rsp(dev, rx_buffers_[index], sizeof(rx_buffers_[index]));
		break;
	}
	case UART_RX_DISABLED:
		next_rx_buffer_index_ = 1U;
		(void)uart_rx_enable(dev, rx_buffers_[0], sizeof(rx_buffers_[0]), kRxIdleTimeoutUs);
		break;
	case UART_RX_STOPPED:
		++rx_stop_count_;
		break;
	default:
		break;
	}
}

void Hi91ImuModule::InvalidateDmaRxCache(const uint8_t *data, size_t len)
{
	if ((data == nullptr) || (len == 0U)) {
		return;
	}

	const uint32_t start = HPM_L1C_CACHELINE_ALIGN_DOWN(reinterpret_cast<uint32_t>(data));
	const uint32_t end = HPM_L1C_CACHELINE_ALIGN_UP(
		reinterpret_cast<uint32_t>(data) + static_cast<uint32_t>(len));
	l1c_dc_invalidate(start, end - start);
}

void Hi91ImuModule::ProcessBytes(const uint8_t *data, size_t size)
{
	for (size_t i = 0U; i < size; ++i) {
		protocols::Hi91Sample sample = {};
		const auto result = parser_.Feed(data[i], &sample);
		if (result == protocols::Hi91ParseResult::kFrame) {
			PublishSample(sample);
		} else if (result != protocols::Hi91ParseResult::kNone) {
			ReportParseIssue(result);
		}
	}
}

void Hi91ImuModule::PublishSample(const protocols::Hi91Sample &sample)
{
	if (!std::isfinite(sample.pitch_deg) ||
		std::abs(sample.pitch_deg) > kMaximumValidPitchDeg) {
		++parse_error_count_;
		LOG_WRN("reject invalid HI91 pitch_mdeg=%d count=%u",
			static_cast<int>(sample.pitch_deg * 1000.0F),
			static_cast<unsigned int>(parse_error_count_));
		return;
	}

	channels::Hi91ImuSample channel_sample = {};
	channel_sample.sequence = ++sample_sequence_;
	channel_sample.uptime_ms = k_uptime_get_32();
	channel_sample.system_time_ms = sample.system_time_ms;
	channel_sample.valid = true;
	channel_sample.roll_deg = sample.roll_deg;
	channel_sample.pitch_deg = sample.pitch_deg;
	channel_sample.yaw_deg = sample.yaw_deg;
	memcpy(channel_sample.quat, sample.quat, sizeof(channel_sample.quat));
	memcpy(channel_sample.gyro_dps, sample.gyro_dps, sizeof(channel_sample.gyro_dps));
	memcpy(channel_sample.accel_g, sample.accel_g, sizeof(channel_sample.accel_g));
	channels::latest_hi91_imu_sample.write(channel_sample);
}

void Hi91ImuModule::ReportParseIssue(protocols::Hi91ParseResult result)
{
	++parse_error_count_;
	if ((parse_error_count_ % 1000U) != 1U) {
		return;
	}

	LOG_WRN("hi91 parse issue result=%u count=%u",
		static_cast<unsigned int>(result),
		static_cast<unsigned int>(parse_error_count_));
}

}  // namespace modules
