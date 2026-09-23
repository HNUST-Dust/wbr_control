#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

#include <hpm_l1c_drv.h>

#define TEST_UART_NODE DT_NODELABEL(uart10)
#define RX_BUFFER_SIZE 128U
#define RX_BUFFER_COUNT 2U
#define RX_IDLE_TIMEOUT_US 2000
#define REPORT_PERIOD_MS 1000U
#define UART_DMA_ALIGN __aligned(HPM_L1C_CACHELINE_SIZE)

struct rx_chunk {
	uint16_t len;
	uint8_t data[RX_BUFFER_SIZE];
};

static const struct device *const uart_dev = DEVICE_DT_GET(TEST_UART_NODE);
static uint8_t rx_buffers[RX_BUFFER_COUNT][RX_BUFFER_SIZE] UART_DMA_ALIGN;
K_MSGQ_DEFINE(rx_msgq, sizeof(struct rx_chunk), 8, 4);

static uint8_t next_buffer_index = 1U;
static atomic_t total_bytes;
static atomic_t event_count;
static atomic_t dropped_chunks;
static atomic_t stopped_count;

static void invalidate_rx_cache(const uint8_t *data, size_t len)
{
	if ((data == NULL) || (len == 0U)) {
		return;
	}

	const uint32_t address = (uint32_t)(uintptr_t)data;
	const uint32_t start = HPM_L1C_CACHELINE_ALIGN_DOWN(address);
	const uint32_t end = HPM_L1C_CACHELINE_ALIGN_UP(address + len);
	l1c_dc_invalidate(start, end - start);
}

static void uart_callback(const struct device *dev, struct uart_event *event, void *user_data)
{
	ARG_UNUSED(user_data);

	if (event == NULL) {
		return;
	}

	switch (event->type) {
	case UART_RX_RDY: {
		const uint8_t *data = event->data.rx.buf + event->data.rx.offset;
		const size_t len = event->data.rx.len;
		invalidate_rx_cache(data, len);

		struct rx_chunk chunk = {.len = (uint16_t)len};
		memcpy(chunk.data, data, len);
		if (k_msgq_put(&rx_msgq, &chunk, K_NO_WAIT) != 0) {
			atomic_inc(&dropped_chunks);
		}
		atomic_add(&total_bytes, (atomic_val_t)len);
		atomic_inc(&event_count);
		break;
	}
	case UART_RX_BUF_REQUEST:
		(void)uart_rx_buf_rsp(dev, rx_buffers[next_buffer_index], RX_BUFFER_SIZE);
		next_buffer_index = (uint8_t)((next_buffer_index + 1U) % RX_BUFFER_COUNT);
		break;
	case UART_RX_STOPPED:
		atomic_inc(&stopped_count);
		break;
	case UART_RX_DISABLED:
		next_buffer_index = 1U;
		(void)uart_rx_enable(dev, rx_buffers[0], RX_BUFFER_SIZE, RX_IDLE_TIMEOUT_US);
		break;
	default:
		break;
	}
}

static void print_chunk(const struct rx_chunk *chunk)
{
	printk("UART10 RX len=%u:", chunk->len);
	for (uint16_t i = 0U; i < chunk->len; ++i) {
		printk(" %02x", chunk->data[i]);
	}
	printk("\r\n");
}

int main(void)
{
	if (!device_is_ready(uart_dev)) {
		printk("UART10 device is not ready\r\n");
		return -ENODEV;
	}

	struct uart_config config;
	int rc = uart_config_get(uart_dev, &config);
	if (rc != 0) {
		printk("UART10 uart_config_get failed: %d\r\n", rc);
		return rc;
	}

	printk("UART10 RX test: dev=%s baud=%u, 8E2\r\n", uart_dev->name,
	       config.baudrate);
	printk("Waiting for UART10 data; received bytes are printed in hex.\r\n");

	rc = uart_callback_set(uart_dev, uart_callback, NULL);
	if (rc != 0) {
		printk("UART10 callback setup failed: %d\r\n", rc);
		return rc;
	}

	rc = uart_rx_enable(uart_dev, rx_buffers[0], RX_BUFFER_SIZE, RX_IDLE_TIMEOUT_US);
	if (rc != 0) {
		printk("UART10 RX start failed: %d\r\n", rc);
		return rc;
	}

	int64_t next_report = k_uptime_get() + REPORT_PERIOD_MS;
	for (;;) {
		struct rx_chunk chunk;
		const int64_t now = k_uptime_get();
		const int64_t wait_ms = MAX(next_report - now, 0);

		if (k_msgq_get(&rx_msgq, &chunk, K_MSEC(wait_ms)) == 0) {
			print_chunk(&chunk);
		}

		if (k_uptime_get() >= next_report) {
			printk("UART10 status: bytes=%ld events=%ld dropped=%ld stopped=%ld\r\n",
			       (long)atomic_get(&total_bytes), (long)atomic_get(&event_count),
			       (long)atomic_get(&dropped_chunks), (long)atomic_get(&stopped_count));
			next_report = k_uptime_get() + REPORT_PERIOD_MS;
		}
	}

	return 0;
}
