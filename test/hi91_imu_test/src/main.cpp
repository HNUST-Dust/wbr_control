/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/ring_buffer.h>

#include <hpm_l1c_drv.h>

#include <protocols/imu/hi91_protocol.h>

namespace {

constexpr uint32_t kHi91Baudrate = 921600U;
constexpr int32_t kRxIdleTimeoutUs = 1000;
constexpr size_t kRxBufferSize = 256U;
constexpr size_t kRxBufferCount = 2U;
constexpr size_t kRxRingSize = 4096U;
constexpr uint32_t kStatsPeriodMs = 1000U;
constexpr uint32_t kDumpBytes = 96U;
constexpr uint32_t kMaxDumpFrames = 12U;

const struct device *const kImuUart = DEVICE_DT_GET(DT_NODELABEL(uart2));
K_SEM_DEFINE(g_rx_sem, 0, 1);
RING_BUF_DECLARE(g_rx_ring, kRxRingSize);

uint8_t g_rx_buffers[kRxBufferCount][kRxBufferSize];
uint8_t g_next_rx_buffer_index = 1U;

uint32_t g_rx_byte_count = 0U;
uint32_t g_frame_count = 0U;
uint32_t g_parse_error_count = 0U;
uint32_t g_rx_stop_count = 0U;
uint32_t g_rx_drop_count = 0U;
uint32_t g_sof_a55a_count = 0U;
uint32_t g_sof_5aa5_count = 0U;
uint32_t g_byte_a5_count = 0U;
uint32_t g_byte_5a_count = 0U;
uint8_t g_previous_byte = 0U;
uint8_t g_dump_bytes[kDumpBytes] = {};
uint32_t g_dump_index = 0U;
uint32_t g_dump_frame_count = 0U;

float g_roll_deg = 0.0F;
float g_pitch_deg = 0.0F;
float g_yaw_deg = 0.0F;
float g_rx_hz = 0.0F;
float g_frame_hz = 0.0F;
float g_sof_a55a_hz = 0.0F;
float g_sof_5aa5_hz = 0.0F;
float g_byte_a5_hz = 0.0F;
float g_byte_5a_hz = 0.0F;

protocols::imu::hi91::Parser g_parser;

void InvalidateDmaRxCache(const uint8_t *data, size_t len)
{
	if ((data == nullptr) || (len == 0U)) {
		return;
	}

	const uint32_t start = HPM_L1C_CACHELINE_ALIGN_DOWN(
		reinterpret_cast<uint32_t>(data));
	const uint32_t end = HPM_L1C_CACHELINE_ALIGN_UP(
		reinterpret_cast<uint32_t>(data) + static_cast<uint32_t>(len));
	l1c_dc_invalidate(start, end - start);
}

void ProcessBytes(const uint8_t *data, size_t size)
{
	for (size_t i = 0U; i < size; ++i) {
		if (data[i] == 0xA5U) {
			++g_byte_a5_count;
		} else if (data[i] == 0x5AU) {
			++g_byte_5a_count;
		}

		if (g_dump_frame_count < kMaxDumpFrames) {
			g_dump_bytes[g_dump_index++] = data[i];
			if (g_dump_index >= sizeof(g_dump_bytes)) {
				printk("[hi91_raw %u]", static_cast<unsigned int>(g_dump_frame_count));
				for (size_t j = 0U; j < sizeof(g_dump_bytes); ++j) {
					printk(" %02x", g_dump_bytes[j]);
				}
				printk("\n");
				++g_dump_frame_count;
				g_dump_index = 0U;
			}
		}

		if ((g_previous_byte == 0xA5U) && (data[i] == 0x5AU)) {
			++g_sof_a55a_count;
		} else if ((g_previous_byte == 0x5AU) && (data[i] == 0xA5U)) {
			++g_sof_5aa5_count;
		}
		g_previous_byte = data[i];

		protocols::imu::hi91::Sample sample = {};
		const auto result = g_parser.Feed(data[i], &sample);
		if (result == protocols::imu::hi91::ParseResult::kFrame) {
			g_roll_deg = sample.roll_deg;
			g_pitch_deg = sample.pitch_deg;
			g_yaw_deg = sample.yaw_deg;
			++g_frame_count;
		} else if (result != protocols::imu::hi91::ParseResult::kNone) {
			++g_parse_error_count;
		}
	}
}

void UartCallback(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);

	switch (evt->type) {
	case UART_RX_RDY: {
		const uint8_t *data = evt->data.rx.buf + evt->data.rx.offset;
		InvalidateDmaRxCache(data, evt->data.rx.len);
		const uint32_t written = ring_buf_put(&g_rx_ring, data, evt->data.rx.len);
		g_rx_byte_count += written;
		if (written != evt->data.rx.len) {
			++g_rx_drop_count;
		}
		k_sem_give(&g_rx_sem);
		break;
	}
	case UART_RX_BUF_REQUEST: {
		const uint8_t index = g_next_rx_buffer_index;
		g_next_rx_buffer_index = (g_next_rx_buffer_index + 1U) % kRxBufferCount;
		(void)uart_rx_buf_rsp(dev, g_rx_buffers[index], sizeof(g_rx_buffers[index]));
		break;
	}
	case UART_RX_DISABLED:
		g_next_rx_buffer_index = 1U;
		(void)uart_rx_enable(dev, g_rx_buffers[0], sizeof(g_rx_buffers[0]), kRxIdleTimeoutUs);
		break;
	case UART_RX_STOPPED:
		++g_rx_stop_count;
		break;
	default:
		break;
	}
}

int ConfigureImuUart()
{
	if (!device_is_ready(kImuUart)) {
		return -ENODEV;
	}

	struct uart_config config = {};
	config.baudrate = kHi91Baudrate;
	config.parity = UART_CFG_PARITY_NONE;
	config.stop_bits = UART_CFG_STOP_BITS_1;
	config.data_bits = UART_CFG_DATA_BITS_8;
	config.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
	int rc = uart_configure(kImuUart, &config);
	if (rc != 0) {
		return rc;
	}

	rc = uart_callback_set(kImuUart, UartCallback, nullptr);
	if (rc != 0) {
		return rc;
	}

	return uart_rx_enable(kImuUart, g_rx_buffers[0], sizeof(g_rx_buffers[0]), kRxIdleTimeoutUs);
}

}  // namespace

int main()
{
	printk("hi91_imu_test started\n");
	printk("uart2 async rx, printing first %u dumps of %u bytes\n",
		static_cast<unsigned int>(kMaxDumpFrames),
		static_cast<unsigned int>(kDumpBytes));

	g_parser.SetStrictCrc(false);

	const int rc = ConfigureImuUart();
	if (rc != 0) {
		printk("uart2 async rx start failed: %d\n", rc);
		return 0;
	}

	uint8_t buffer[kRxBufferSize];
	uint32_t last_stats_ms = k_uptime_get_32();
	uint32_t last_rx_byte_count = 0U;
	uint32_t last_frame_count = 0U;
	uint32_t last_sof_a55a_count = 0U;
	uint32_t last_sof_5aa5_count = 0U;
	uint32_t last_byte_a5_count = 0U;
	uint32_t last_byte_5a_count = 0U;

	for (;;) {
		(void)k_sem_take(&g_rx_sem, K_MSEC(1));
		for (;;) {
			const uint32_t count = ring_buf_get(&g_rx_ring, buffer, sizeof(buffer));
			if (count == 0U) {
				break;
			}
			ProcessBytes(buffer, count);
		}

		const uint32_t now_ms = k_uptime_get_32();
		if ((now_ms - last_stats_ms) >= kStatsPeriodMs) {
			const uint32_t elapsed_ms = now_ms - last_stats_ms;
			const uint32_t rx_delta = g_rx_byte_count - last_rx_byte_count;
			const uint32_t frame_delta = g_frame_count - last_frame_count;
			const uint32_t sof_a55a_delta = g_sof_a55a_count - last_sof_a55a_count;
			const uint32_t sof_5aa5_delta = g_sof_5aa5_count - last_sof_5aa5_count;
			const uint32_t byte_a5_delta = g_byte_a5_count - last_byte_a5_count;
			const uint32_t byte_5a_delta = g_byte_5a_count - last_byte_5a_count;
			g_rx_hz = static_cast<float>(rx_delta) * 1000.0F / static_cast<float>(elapsed_ms);
			g_frame_hz = static_cast<float>(frame_delta) * 1000.0F / static_cast<float>(elapsed_ms);
			g_sof_a55a_hz = static_cast<float>(sof_a55a_delta) * 1000.0F / static_cast<float>(elapsed_ms);
			g_sof_5aa5_hz = static_cast<float>(sof_5aa5_delta) * 1000.0F / static_cast<float>(elapsed_ms);
			g_byte_a5_hz = static_cast<float>(byte_a5_delta) * 1000.0F / static_cast<float>(elapsed_ms);
			g_byte_5a_hz = static_cast<float>(byte_5a_delta) * 1000.0F / static_cast<float>(elapsed_ms);
			last_rx_byte_count = g_rx_byte_count;
			last_frame_count = g_frame_count;
			last_sof_a55a_count = g_sof_a55a_count;
			last_sof_5aa5_count = g_sof_5aa5_count;
			last_byte_a5_count = g_byte_a5_count;
			last_byte_5a_count = g_byte_5a_count;
			last_stats_ms = now_ms;
			printk("[hi91_stat] rx_Bps=%u frame_hz=%u sof_a55a=%u sof_5aa5=%u byte_a5=%u byte_5a=%u parse_err=%u stop=%u drop=%u roll_mdeg=%d pitch_mdeg=%d yaw_mdeg=%d\n",
				static_cast<unsigned int>(g_rx_hz),
				static_cast<unsigned int>(g_frame_hz),
				static_cast<unsigned int>(g_sof_a55a_hz),
				static_cast<unsigned int>(g_sof_5aa5_hz),
				static_cast<unsigned int>(g_byte_a5_hz),
				static_cast<unsigned int>(g_byte_5a_hz),
				static_cast<unsigned int>(g_parse_error_count),
				static_cast<unsigned int>(g_rx_stop_count),
				static_cast<unsigned int>(g_rx_drop_count),
				static_cast<int>(g_roll_deg * 1000.0F),
				static_cast<int>(g_pitch_deg * 1000.0F),
				static_cast<int>(g_yaw_deg * 1000.0F));
		}
	}
}
