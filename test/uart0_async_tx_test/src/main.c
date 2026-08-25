#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

#include <hpm_l1c_drv.h>

#define TEST_UART_NODE DT_NODELABEL(uart0)
#define TEST_BAUDRATE 921600U
#define TEST_PERIOD_MS 10U
#define TEST_TX_TIMEOUT_MS 5U
#define TEST_FRAME_SIZE 112U

static const struct device *const g_uart = DEVICE_DT_GET(TEST_UART_NODE);
static uint8_t g_tx_buffer[TEST_FRAME_SIZE] __aligned(32);
static struct k_sem g_tx_done_sem;
static atomic_t g_done_count;
static atomic_t g_abort_count;

static void poll_write(const char *text)
{
	while (*text != '\0') {
		uart_poll_out(g_uart, *text++);
	}
}

static void uart_callback(const struct device *dev, struct uart_event *event, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	if (event == NULL) {
		return;
	}

	if (event->type == UART_TX_DONE) {
		atomic_inc(&g_done_count);
		k_sem_give(&g_tx_done_sem);
	} else if (event->type == UART_TX_ABORTED) {
		atomic_inc(&g_abort_count);
		k_sem_give(&g_tx_done_sem);
	}
}

static void prepare_frame(uint32_t sequence)
{
	char header[96];
	const int header_len = snprintk(
		header, sizeof(header),
		"UART0 async DMA seq=%u done=%u abort=%u ", sequence,
		(uint32_t)atomic_get(&g_done_count), (uint32_t)atomic_get(&g_abort_count));

	memset(g_tx_buffer, '.', sizeof(g_tx_buffer));
	if (header_len > 0) {
		const size_t copy_len = MIN((size_t)header_len, sizeof(g_tx_buffer) - 2U);
		memcpy(g_tx_buffer, header, copy_len);
	}
	g_tx_buffer[sizeof(g_tx_buffer) - 2U] = '\r';
	g_tx_buffer[sizeof(g_tx_buffer) - 1U] = '\n';
}

static void flush_tx_buffer(void)
{
	const uint32_t address = (uint32_t)(uintptr_t)g_tx_buffer;
	const uint32_t start = HPM_L1C_CACHELINE_ALIGN_DOWN(address);
	const uint32_t end = HPM_L1C_CACHELINE_ALIGN_UP(address + sizeof(g_tx_buffer));
	l1c_dc_flush(start, end - start);
}

int main(void)
{
	if (!device_is_ready(g_uart)) {
		return -ENODEV;
	}

	struct uart_config config = {
		.baudrate = TEST_BAUDRATE,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
	};

	int rc = uart_configure(g_uart, &config);
	if (rc != 0) {
		return rc;
	}

	poll_write("UART0 poll path OK; starting async XDMA TX\r\n");

	k_sem_init(&g_tx_done_sem, 0, 1);
	rc = uart_callback_set(g_uart, uart_callback, NULL);
	if (rc != 0) {
		poll_write("UART0 callback setup FAILED\r\n");
		return rc;
	}

	uint32_t sequence = 0U;
	for (;;) {
		prepare_frame(++sequence);
		flush_tx_buffer();
		k_sem_reset(&g_tx_done_sem);

		rc = uart_tx(g_uart, g_tx_buffer, sizeof(g_tx_buffer), SYS_FOREVER_US);
		if (rc != 0) {
			char error[64];
			snprintk(error, sizeof(error), "UART0 uart_tx start FAILED rc=%d\r\n", rc);
			poll_write(error);
			k_sleep(K_MSEC(1000));
			continue;
		}

		if (k_sem_take(&g_tx_done_sem, K_MSEC(TEST_TX_TIMEOUT_MS)) != 0) {
			const int abort_rc = uart_tx_abort(g_uart);
			char error[80];
			snprintk(error, sizeof(error),
				 "UART0 async TX TIMEOUT seq=%u abort_rc=%d\r\n", sequence,
				 abort_rc);
			poll_write(error);
			k_sleep(K_MSEC(1000));
			continue;
		}

		k_sleep(K_MSEC(TEST_PERIOD_MS));
	}

	return 0;
}
