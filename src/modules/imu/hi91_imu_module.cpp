/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file src/modules/imu/hi91_imu_module.cpp
 * @ingroup wbr_modules
 * @brief 实现 HI91 IMU 的采集、校验与状态发布模块。
 * @details 实现运行在模块自有 Zephyr 线程或其驱动回调中。回调路径只完成有界的数据搬运和通知，耗时解析与控制计算留在线程上下文执行。
 */

#include "hi91_imu_module.h"

#include <algorithm>
#include <errno.h>
#include <cmath>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <drivers/uart_hpmicro.h>

#include <channels/hi91_imu_sample.hpp>
#include <scheduling/thread_priorities.h>

LOG_MODULE_REGISTER(hi91_imu_module, LOG_LEVEL_INF);

namespace
{

K_THREAD_STACK_DEFINE(g_hi91_imu_module_stack, 2048);

#if defined(CONFIG_WBR_CONTROL_HI91_IMU_UART_BAUDRATE)
constexpr uint32_t kUartBaudrate = CONFIG_WBR_CONTROL_HI91_IMU_UART_BAUDRATE;
#else
constexpr uint32_t kUartBaudrate = 921600U;
#endif
constexpr bool kStrictCrc = IS_ENABLED(CONFIG_WBR_CONTROL_HI91_IMU_STRICT_CRC);
constexpr uint32_t kStartupDelayMs = 2000U;
constexpr uint32_t kRxRingPollPeriodMs = 1U;
constexpr uint32_t kFreshnessDeadlineUs = 3000U;
// HI91 输出物理欧拉俯仰角，超过该范围视为 UART 数据损坏。
constexpr float kMaximumValidPitchDeg = 100.0F;

/*
 * UART RX ring 由 HDMA 持续写、HI91 线程持续读。放入 non-cacheable RAM 后
 * CPU 每次都直接观察物理内存，不再需要按 cache line 执行 flush/invalidate。
 * HPM HDMA 要求目标地址可由系统总线访问；64 字节对齐也方便后续调整大小。
 */
__attribute__((section(".nocache"), aligned(64)))
uint8_t g_hi91_rx_dma_ring[4096U];

const struct device *FindInputUart()
{
#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart2), okay)
	return DEVICE_DT_GET(DT_NODELABEL(uart2));
#else
	return nullptr;
#endif
}

} // namespace

namespace modules
{

Hi91ImuModule::Hi91ImuModule() = default;

int Hi91ImuModule::Start()
{
	if (started_) {
		return 0;
	}

	uart_dev_ = FindInputUart();
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

	return CreateThread(
		g_hi91_imu_module_stack, K_THREAD_STACK_SIZEOF(g_hi91_imu_module_stack),
		K_PRIO_PREEMPT(wbr_control::scheduling::thread_priority::kImu),
		"hi91_imu_module");
}

void Hi91ImuModule::RunLoop()
{
	LOG_INF("hi91 imu circular dma module started uart=%s baud=%u strict_crc=%u", uart_dev_->name,
		static_cast<unsigned int>(kUartBaudrate), static_cast<unsigned int>(kStrictCrc));

	k_sleep(K_MSEC(kStartupDelayMs));

	static_assert(sizeof(g_hi91_rx_dma_ring) == kRxDmaRingSize,
		      "HI91 DMA ring size must match the consumer modulo");
	rx_read_pos_ = 0U;
	rx_observed_write_pos_ = 0U;
	rx_available_bytes_ = 0U;
	int rc = uart_hpm_rx_circular_enable(
		uart_dev_, g_hi91_rx_dma_ring, kRxDmaRingSize);
	if (rc != 0) {
		LOG_ERR("uart circular rx enable failed: %d", rc);
		return;
	}
	for (;;) {
		/*
		 * write_pos 是 DMA 下一字节将写入的位置。先抓取快照，再只处理该
		 * 快照之前的数据；DMA 可以同时在后方继续写，不需要暂停通道。
		 * 驱动在一轮恰好结束的瞬间可能报告 size，等价于“尾部已写完、
		 * 下一次从0开始”，因此这里允许 write_pos == kRxDmaRingSize。
		 */
		size_t write_pos = 0U;
		rc = uart_hpm_rx_circular_get_position(uart_dev_, &write_pos);
		if ((rc != 0) || (write_pos > kRxDmaRingSize)) {
			++rx_drop_count_;
			k_sleep(K_MSEC(kRxRingPollPeriodMs));
			continue;
		}

		/*
		 * 只用连续两次位置的差值计算生产量。raw position 的 size 与 0
		 * 都表示 ring 起点，先归一化再处理回绕。只要轮询间隔远小于约
		 * 50ms 的整圈时间，生产者不会在两次观察之间套圈。
		 */
		const size_t normalized_write_pos =
			(write_pos == kRxDmaRingSize) ? 0U : write_pos;
		const size_t produced = (normalized_write_pos >= rx_observed_write_pos_)
			? (normalized_write_pos - rx_observed_write_pos_)
			: (kRxDmaRingSize - rx_observed_write_pos_ + normalized_write_pos);
		rx_observed_write_pos_ = normalized_write_pos;
		rx_available_bytes_ += produced;

		/*
		 * available 中最后 kRxCommitGuardSize 字节仍可能是 DMA outstanding
		 * write，只消费更早的数据。它们会在下一轮已经远离写入前沿后处理。
		 */
		size_t consume_length = (rx_available_bytes_ > kRxCommitGuardSize)
			? (rx_available_bytes_ - kRxCommitGuardSize)
			: 0U;
		while (consume_length != 0U) {
			const size_t contiguous = std::min(
				consume_length, kRxDmaRingSize - rx_read_pos_);
			const uint8_t *const consume_start = &g_hi91_rx_dma_ring[rx_read_pos_];
			rx_read_pos_ += contiguous;
			if (rx_read_pos_ == kRxDmaRingSize) {
				rx_read_pos_ = 0U;
			}
			consume_length -= contiguous;
			rx_available_bytes_ -= contiguous;
			ProcessBytes(consume_start, contiguous);
		}

		k_sleep(K_MSEC(kRxRingPollPeriodMs));
	}
}

void Hi91ImuModule::ProcessBytes(const uint8_t *data, size_t size)
{
	/* DMA ring 没有帧边界；状态机连续消费字节并允许帧跨越 ring 末尾。 */
	for (size_t i = 0U; i < size; ++i) {
		ProcessByte(data[i]);
	}
}

void Hi91ImuModule::ProcessByte(uint8_t byte)
{
	/*
	 * 组帧顺序：搜索 5A A5 -> 收取 2 字节 payload length ->
	 * 收取 2 字节 CRC 和 payload。收齐后才调用 DecodeHi91Frame 校验 CRC。
	 */
	switch (frame_state_) {
	case 0U: /* wait SOF0 */
		if (byte == protocols::kHi91FrameSof0) {
			frame_buf_[0] = byte;
			frame_pos_ = 1U;
			frame_state_ = 1U;
		}
		break;
	case 1U: /* wait SOF1 */
		if (byte == protocols::kHi91FrameSof1) {
			frame_buf_[1] = byte;
			frame_pos_ = 2U;
			frame_state_ = 2U;
		} else if (byte == protocols::kHi91FrameSof0) {
			frame_buf_[0] = byte;
			frame_pos_ = 1U;
		} else {
			frame_state_ = 0U;
			frame_pos_ = 0U;
		}
		break;
	case 2U: /* collect length */
		frame_buf_[frame_pos_++] = byte;
		if (frame_pos_ == 4U) {
			const uint16_t payload_length =
				static_cast<uint16_t>(frame_buf_[2]) |
				(static_cast<uint16_t>(frame_buf_[3]) << 8U);
			if ((payload_length == 0U) ||
			    (payload_length > protocols::kHi91MaxPayloadLength)) {
				ReportParseIssue(-EBADMSG);
				frame_state_ = 0U;
				frame_pos_ = 0U;
			} else {
				frame_remaining_ = 2U + payload_length; /* crc + payload */
				frame_state_ = 3U;
			}
		}
		break;
	case 3U: /* collect crc + payload */
		frame_buf_[frame_pos_++] = byte;
		if (--frame_remaining_ == 0U) {
			protocols::Hi91Sample sample = {};
			const int rc = protocols::DecodeHi91Frame(
				frame_buf_, frame_pos_, kStrictCrc, &sample);
			if (rc == 0) {
				PublishSample(sample);
			} else {
				ReportParseIssue(rc);
			}
			frame_state_ = 0U;
			frame_pos_ = 0U;
		}
		break;
	default:
		frame_state_ = 0U;
		frame_pos_ = 0U;
		break;
	}
}

void Hi91ImuModule::PublishSample(const protocols::Hi91Sample &sample)
{
	if (!std::isfinite(sample.pitch_deg) ||
	    std::abs(sample.pitch_deg) > kMaximumValidPitchDeg) {
		++parse_error_count_;
		++invalid_sample_count_;
		LOG_WRN("reject invalid HI91 pitch_mdeg=%d count=%u",
			static_cast<int>(sample.pitch_deg * 1000.0F),
			static_cast<unsigned int>(parse_error_count_));
		return;
	}

	channels::Hi91ImuSample channel_sample = {};
	const uint64_t publish_time_us = k_cyc_to_us_floor64(k_cycle_get_64());
	if (last_sensor_time_ms_ != 0U) {
		max_sensor_interval_ms_ =
			std::max(max_sensor_interval_ms_, sample.system_time_ms - last_sensor_time_ms_);
	}
	last_sensor_time_ms_ = sample.system_time_ms;
	if (last_publish_time_us_ != 0U && publish_time_us >= last_publish_time_us_) {
		const uint64_t interval_us = publish_time_us - last_publish_time_us_;
		max_publish_interval_us_ = std::max(
			max_publish_interval_us_,
			static_cast<uint32_t>(std::min<uint64_t>(interval_us, UINT32_MAX)));
		if (interval_us > kFreshnessDeadlineUs) {
			if (crc_errors_since_last_publish_ != 0U) {
				++publish_gap_with_crc_count_;
			} else {
				++publish_gap_without_crc_count_;
			}
		}
	}
	crc_errors_since_last_publish_ = 0U;
	last_publish_time_us_ = publish_time_us;
	channel_sample.sequence = ++sample_sequence_;
	channel_sample.uptime_ms = k_uptime_get_32();
	channel_sample.precise_timestamp_us = publish_time_us;
	channel_sample.max_publish_interval_us = max_publish_interval_us_;
	channel_sample.max_sensor_interval_ms = max_sensor_interval_ms_;
	channel_sample.parse_error_count = parse_error_count_;
	channel_sample.crc_error_count = crc_error_count_;
	channel_sample.publish_gap_with_crc_count = publish_gap_with_crc_count_;
	channel_sample.publish_gap_without_crc_count = publish_gap_without_crc_count_;
	channel_sample.frame_format_error_count = frame_format_error_count_;
	channel_sample.invalid_sample_count = invalid_sample_count_;
	channel_sample.rx_drop_count = rx_drop_count_;
	channel_sample.rx_stop_count = rx_stop_count_;
	channel_sample.rx_buf_rsp_error_count = rx_buf_rsp_error_count_;
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

void Hi91ImuModule::ReportParseIssue(int error)
{
	++parse_error_count_;
	if (error == -EILSEQ) {
		++crc_error_count_;
		++crc_errors_since_last_publish_;
	} else {
		++frame_format_error_count_;
	}
	if ((parse_error_count_ % 1000U) != 1U) {
		return;
	}

	LOG_WRN("hi91 parse issue error=%d count=%u", error,
		static_cast<unsigned int>(parse_error_count_));
}

} // namespace modules
