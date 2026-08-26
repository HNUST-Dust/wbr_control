/**
 * @file
 * @brief ICM42688P-HXY hardware SPI and scheduler smoke test.
 */

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define IMU_SPI_NODE DT_NODELABEL(spi2)
#define IMU_CS_NODE DT_NODELABEL(imu_cs)

#define IMU_WHO_AM_I_REG 0x01U
#define IMU_WHO_AM_I_EXPECTED 0x6AU
#define IMU_SPI_FREQUENCY_HZ 100000U
#define IMU_POWER_ON_DELAY_MS 100U
#define IMU_READ_INTERVAL_MS 20U

#if !DT_NODE_HAS_STATUS(IMU_SPI_NODE, okay)
#error "SPI2 must be enabled"
#endif

#if !DT_NODE_EXISTS(IMU_CS_NODE)
#error "imu_cs must be defined by the board overlay"
#endif

#if DT_NODE_HAS_PROP(IMU_SPI_NODE, cs_gpios)
#error "SPI2 cs-gpios must be removed because PA26 is controlled manually"
#endif

static const struct device *const spi_dev = DEVICE_DT_GET(IMU_SPI_NODE);
static const struct gpio_dt_spec imu_cs = GPIO_DT_SPEC_GET(IMU_CS_NODE, gpios);

static const struct spi_config imu_spi_config = {
	.frequency = IMU_SPI_FREQUENCY_HZ,
	.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
	.slave = 0,
};

/* These values remain visible in GDB even when UART output is unavailable. */
volatile uint32_t imu_scope_magic;
volatile uint32_t imu_scope_sequence;
volatile int32_t imu_scope_last_rc;
volatile uint32_t imu_scope_last_value;
volatile uint32_t imu_worker_sequence;

static int ImuReadRegister(uint8_t reg, uint8_t *value)
{
	uint8_t tx[2] = { (uint8_t)(0x80U | reg), 0U };
	uint8_t rx[2] = { 0U, 0U };
	const struct spi_buf tx_buffer = { .buf = tx, .len = sizeof(tx) };
	const struct spi_buf rx_buffer = { .buf = rx, .len = sizeof(rx) };
	const struct spi_buf_set tx_buffers = { .buffers = &tx_buffer, .count = 1 };
	const struct spi_buf_set rx_buffers = { .buffers = &rx_buffer, .count = 1 };
	int rc;

	/* imu_cs is active-low: logical 1 asserts CS and logical 0 releases it. */
	gpio_pin_set_dt(&imu_cs, 1);
	rc = spi_transceive(spi_dev, &imu_spi_config, &tx_buffers, &rx_buffers);
	gpio_pin_set_dt(&imu_cs, 0);

	if (rc == 0) {
		*value = rx[1];
	}

	return rc;
}

static void SchedulerProbeThread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		imu_worker_sequence++;
		k_sleep(K_MSEC(10));
	}
}

K_THREAD_DEFINE(scheduler_probe_tid, 1024, SchedulerProbeThread,
		NULL, NULL, NULL, K_PRIO_PREEMPT(1), 0, 0);

int main(void)
{
	uint8_t value = 0U;
	int rc;

	printk("ICM42688P-HXY hardware SPI test\n");
	printk("SPI2: mode 0, MSB first, %u Hz; CS=PA26\n",
	       IMU_SPI_FREQUENCY_HZ);

	if (!device_is_ready(spi_dev)) {
		printk("SPI2 is not ready\n");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&imu_cs)) {
		printk("IMU CS GPIO is not ready\n");
		return -ENODEV;
	}

	rc = gpio_pin_configure_dt(&imu_cs, GPIO_OUTPUT_INACTIVE);
	if (rc != 0) {
		printk("Failed to configure IMU CS: %d\n", rc);
		return rc;
	}

	k_sleep(K_MSEC(IMU_POWER_ON_DELAY_MS));
	imu_scope_magic = 0x53504921U; /* "SPI!" */

	while (1) {
		rc = ImuReadRegister(IMU_WHO_AM_I_REG, &value);
		imu_scope_last_rc = rc;
		imu_scope_last_value = value;
		imu_scope_sequence++;

		if ((imu_scope_sequence % 50U) == 1U) {
			printk("seq=%u WHO_AM_I=0x%02X rc=%d expected=0x%02X\n",
			       imu_scope_sequence, value, rc, IMU_WHO_AM_I_EXPECTED);
		}

		k_sleep(K_MSEC(IMU_READ_INTERVAL_MS));
	}

	return 0;
}
