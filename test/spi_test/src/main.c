/**
 * @file
 * @brief ICM42688P-HXY INT1-triggered asynchronous SPI DMA test.
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/sys/printk.h>

#define IMU_SPI_NODE DT_NODELABEL(spi2)
#define IMU_CS_NODE DT_ALIAS(onboard_imu_cs)
#define IMU_DRDY_NODE DT_ALIAS(onboard_imu_drdy)

#define IMU_READ_MASK 0x80U
#define IMU_WHO_AM_I_REG 0x01U
#define IMU_WHO_AM_I_EXPECTED 0x6AU
#define IMU_COM_CFG_REG 0x05U
#define IMU_INT_CFG1_REG 0x06U
#define IMU_ACCEL_DATA_REG 0x0CU
#define IMU_ACCEL_CFG_REG 0x40U
#define IMU_ACCEL_RANGE_REG 0x41U
#define IMU_GYRO_CFG_REG 0x42U
#define IMU_GYRO_RANGE_REG 0x43U
#define IMU_PWR_CTRL_REG 0x7DU

#define IMU_SPI_FREQUENCY_HZ 8000000U
#define IMU_POWER_ON_DELAY_MS 100U
#define IMU_TRANSFER_TIMEOUT_MS 20U
#define IMU_BURST_SIZE 13U

#if !DT_NODE_HAS_STATUS(IMU_SPI_NODE, okay)
#error "SPI2 must be enabled"
#endif
#if !DT_NODE_EXISTS(IMU_CS_NODE)
#error "onboard-imu-cs must be defined by the board overlay"
#endif
#if !DT_NODE_EXISTS(IMU_DRDY_NODE)
#error "onboard-imu-drdy must be defined by the board overlay"
#endif
#if DT_NODE_HAS_PROP(IMU_SPI_NODE, cs_gpios)
#error "SPI2 cs-gpios must be removed because spi_config.cs owns PA26"
#endif

static const struct device *const spi_dev = DEVICE_DT_GET(IMU_SPI_NODE);
static const struct gpio_dt_spec imu_cs = GPIO_DT_SPEC_GET(IMU_CS_NODE, gpios);
static const struct gpio_dt_spec imu_drdy = GPIO_DT_SPEC_GET(IMU_DRDY_NODE, gpios);

static const struct spi_config imu_spi_config = {
	.frequency = IMU_SPI_FREQUENCY_HZ,
	.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
	.slave = 0,
	.cs = {
		.gpio = GPIO_DT_SPEC_GET(IMU_CS_NODE, gpios),
		.delay = 0U,
	},
};

/* HPM HDMA sees physical RAM directly. Keep all DMA buffers out of D-cache. */
static uint8_t __nocache __aligned(64) imu_tx_buffer[IMU_BURST_SIZE];
static uint8_t __nocache __aligned(64) imu_rx_buffer[IMU_BURST_SIZE];
static uint8_t __nocache __aligned(64) imu_control_tx[2];
static uint8_t __nocache __aligned(64) imu_control_rx[2];

static struct spi_buf imu_tx_spi_buffer = {.buf = imu_tx_buffer, .len = sizeof(imu_tx_buffer)};
static struct spi_buf imu_rx_spi_buffer = {.buf = imu_rx_buffer, .len = sizeof(imu_rx_buffer)};
static const struct spi_buf_set imu_tx_buffers = {.buffers = &imu_tx_spi_buffer, .count = 1};
static const struct spi_buf_set imu_rx_buffers = {.buffers = &imu_rx_spi_buffer, .count = 1};

K_SEM_DEFINE(imu_data_ready, 0, 1);
K_SEM_DEFINE(imu_transfer_done, 0, 1);
static struct gpio_callback imu_drdy_callback;

/* Deliberately volatile: inspect these symbols directly in GDB. */
volatile uint32_t imu_scope_magic;
volatile uint32_t imu_scope_sequence;
volatile int32_t imu_scope_last_rc;
volatile uint32_t imu_data_ready_count;
volatile uint32_t imu_dropped_data_ready_count;
volatile uint32_t imu_async_submit_sequence;
volatile uint32_t imu_async_callback_sequence;
volatile uint32_t imu_transfer_timeout_count;
volatile uint32_t imu_transfer_abort_count;
volatile uint32_t imu_transfer_abort_error_count;
volatile int32_t imu_abort_recovery_self_test_result;
volatile uint32_t imu_async_outstanding_on_return_sequence;
volatile uint32_t imu_callback_before_submit_return_sequence;
volatile uint32_t imu_submit_return_cycle;
volatile uint32_t imu_callback_cycle;
volatile uint32_t imu_latest_data_ready_cycle;
volatile uint32_t imu_acquisition_latency_cycles;
volatile uint32_t imu_worker_sequence;
volatile uint32_t imu_worker_at_submit;
volatile uint32_t imu_worker_at_callback;
volatile uint32_t imu_worker_during_dma_last;
volatile uint8_t imu_submit_call_active;
volatile int16_t imu_accel_raw[3];
volatile int16_t imu_gyro_raw[3];

static int16_t DecodeBigEndian(const uint8_t *bytes)
{
	return (int16_t)(((uint16_t)bytes[0] << 8U) | (uint16_t)bytes[1]);
}

static int ImuReadRegister(uint8_t reg, uint8_t *value)
{
	imu_control_tx[0] = IMU_READ_MASK | reg;
	imu_control_tx[1] = 0U;
	imu_control_rx[0] = 0U;
	imu_control_rx[1] = 0U;
	const struct spi_buf tx_buf = {.buf = imu_control_tx, .len = 2U};
	const struct spi_buf rx_buf = {.buf = imu_control_rx, .len = 2U};
	const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1U};
	const struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1U};
	const int rc = spi_transceive(spi_dev, &imu_spi_config, &tx_set, &rx_set);

	if (rc == 0) {
		*value = imu_control_rx[1];
	}
	return rc;
}

static int ImuWriteRegister(uint8_t reg, uint8_t value)
{
	imu_control_tx[0] = reg & (uint8_t)~IMU_READ_MASK;
	imu_control_tx[1] = value;
	const struct spi_buf tx_buf = {.buf = imu_control_tx, .len = 2U};
	const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1U};

	return spi_write(spi_dev, &imu_spi_config, &tx_set);
}

static int ImuWriteAndVerify(uint8_t reg, uint8_t value)
{
	int rc = ImuWriteRegister(reg, value);
	uint8_t readback = 0U;

	if (rc != 0) {
		return rc;
	}
	k_sleep(K_MSEC(1));
	rc = ImuReadRegister(reg, &readback);
	if (rc != 0) {
		return rc;
	}
	if (readback != value) {
		printk("verify failed: reg=0x%02X wrote=0x%02X read=0x%02X\n",
		       reg, value, readback);
		return -EIO;
	}
	return 0;
}

static void ImuTransferCallback(const struct device *dev, int status, void *userdata)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(userdata);

	imu_callback_cycle = k_cycle_get_32();
	imu_worker_at_callback = imu_worker_sequence;
	imu_worker_during_dma_last = imu_worker_at_callback - imu_worker_at_submit;
	if (imu_submit_call_active != 0U) {
		imu_callback_before_submit_return_sequence++;
	}
	imu_scope_last_rc = status;
	imu_async_callback_sequence++;
	k_sem_give(&imu_transfer_done);
}

static int ImuReadSixAxesAsync(void)
{
	const uint32_t callbacks_before_submit = imu_async_callback_sequence;

	imu_tx_buffer[0] = IMU_READ_MASK | IMU_ACCEL_DATA_REG;
	memset(&imu_tx_buffer[1], 0, sizeof(imu_tx_buffer) - 1U);
	memset(imu_rx_buffer, 0, sizeof(imu_rx_buffer));
	k_sem_reset(&imu_transfer_done);
	imu_worker_at_submit = imu_worker_sequence;
	imu_submit_call_active = 1U;
	int rc = spi_transceive_cb(spi_dev, &imu_spi_config,
				   &imu_tx_buffers, &imu_rx_buffers,
				   ImuTransferCallback, NULL);
	if (rc != 0) {
		imu_submit_call_active = 0U;
		return rc;
	}

	imu_submit_return_cycle = k_cycle_get_32();
	imu_submit_call_active = 0U;
	if (imu_async_callback_sequence == callbacks_before_submit) {
		imu_async_outstanding_on_return_sequence++;
	}
	imu_async_submit_sequence++;
	return 0;
}

static int ImuAbortRecoverySelfTest(void)
{
	static const struct spi_config slow_spi_config = {
		.frequency = 100000U,
		.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
		.slave = 0,
		.cs = {
			.gpio = GPIO_DT_SPEC_GET(IMU_CS_NODE, gpios),
			.delay = 0U,
		},
	};

	imu_tx_buffer[0] = IMU_READ_MASK | IMU_ACCEL_DATA_REG;
	memset(&imu_tx_buffer[1], 0, sizeof(imu_tx_buffer) - 1U);
	memset(imu_rx_buffer, 0, sizeof(imu_rx_buffer));
	k_sem_reset(&imu_transfer_done);
	imu_scope_last_rc = -EINPROGRESS;
	int rc = spi_transceive_cb(spi_dev, &slow_spi_config,
				   &imu_tx_buffers, &imu_rx_buffers,
				   ImuTransferCallback, NULL);
	if (rc != 0) {
		return rc;
	}
	rc = spi_release(spi_dev, &slow_spi_config);
	if (rc != 0) {
		return rc;
	}
	rc = k_sem_take(&imu_transfer_done, K_MSEC(IMU_TRANSFER_TIMEOUT_MS));
	if (rc != 0) {
		return rc;
	}
	if (imu_scope_last_rc != -ECANCELED) {
		return -EIO;
	}

	uint8_t who_am_i = 0U;
	rc = ImuReadRegister(IMU_WHO_AM_I_REG, &who_am_i);
	if (rc != 0) {
		return rc;
	}
	return (who_am_i == IMU_WHO_AM_I_EXPECTED) ? 0 : -ENODEV;
}

static void ImuDataReadyCallback(const struct device *port,
				 struct gpio_callback *callback,
				 gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(callback);
	ARG_UNUSED(pins);

	imu_data_ready_count++;
	imu_latest_data_ready_cycle = k_cycle_get_32();
	if (k_sem_count_get(&imu_data_ready) != 0U) {
		imu_dropped_data_ready_count++;
		return;
	}
	k_sem_give(&imu_data_ready);
}

static int ImuInitialize(void)
{
	uint8_t who_am_i = 0U;
	int rc = ImuReadRegister(IMU_WHO_AM_I_REG, &who_am_i);

	if ((rc != 0) || (who_am_i != IMU_WHO_AM_I_EXPECTED)) {
		printk("WHO_AM_I failed: rc=%d value=0x%02X expected=0x%02X\n",
		       rc, who_am_i, IMU_WHO_AM_I_EXPECTED);
		return (rc != 0) ? rc : -ENODEV;
	}
	printk("WHO_AM_I=0x%02X OK\n", who_am_i);

	rc = ImuWriteRegister(IMU_PWR_CTRL_REG, 0x0EU);
	if (rc != 0) {
		return rc;
	}
	k_sleep(K_MSEC(10));

	const struct {
		uint8_t reg;
		uint8_t value;
	} configuration[] = {
		{IMU_COM_CFG_REG, 0x50U},
		{IMU_ACCEL_RANGE_REG, 0x02U},
		{IMU_GYRO_RANGE_REG, 0x00U},
		{IMU_ACCEL_CFG_REG, 0xACU},
		{IMU_GYRO_CFG_REG, 0xACU},
	};
	for (size_t i = 0U; i < ARRAY_SIZE(configuration); ++i) {
		rc = ImuWriteAndVerify(configuration[i].reg, configuration[i].value);
		if (rc != 0) {
			return rc;
		}
	}

	/* Arm PB09 before routing sensor DRDY to INT1. */
	gpio_init_callback(&imu_drdy_callback, ImuDataReadyCallback, BIT(imu_drdy.pin));
	rc = gpio_add_callback(imu_drdy.port, &imu_drdy_callback);
	if (rc != 0) {
		return rc;
	}
	rc = gpio_pin_interrupt_configure_dt(&imu_drdy, GPIO_INT_EDGE_TO_ACTIVE);
	if (rc != 0) {
		return rc;
	}
	return ImuWriteAndVerify(IMU_INT_CFG1_REG, 0x03U);
}

static void SchedulerProbeThread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		imu_worker_sequence++;
		k_yield();
	}
}

K_THREAD_DEFINE(scheduler_probe_tid, 1024, SchedulerProbeThread,
		/* Keep the busy scheduler probe below the RTT/log threads. */
		NULL, NULL, NULL, K_PRIO_PREEMPT(K_LOWEST_APPLICATION_THREAD_PRIO), 0, 0);

int main(void)
{
	printk("ICM42688P-HXY INT1 + asynchronous SPI DMA test\n");
	printk("SPI2=%u Hz CS=PA26 INT1=PB09 INT2=PB12(unused)\n",
	       IMU_SPI_FREQUENCY_HZ);

	if (!device_is_ready(spi_dev) || !gpio_is_ready_dt(&imu_cs) ||
	    !gpio_is_ready_dt(&imu_drdy)) {
		printk("SPI2/CS/DRDY device is not ready\n");
		return -ENODEV;
	}
	int rc = gpio_pin_configure_dt(&imu_cs, GPIO_OUTPUT_INACTIVE);
	if (rc == 0) {
		rc = gpio_pin_configure_dt(&imu_drdy, GPIO_INPUT);
	}
	if (rc != 0) {
		printk("GPIO configuration failed: %d\n", rc);
		return rc;
	}

	k_sleep(K_MSEC(IMU_POWER_ON_DELAY_MS));
	rc = ImuInitialize();
	if (rc != 0) {
		imu_scope_last_rc = rc;
		printk("IMU initialization failed: %d\n", rc);
		return rc;
	}
	imu_scope_magic = 0x44524459U; /* "DRDY" */
	imu_abort_recovery_self_test_result = ImuAbortRecoverySelfTest();
	if (imu_abort_recovery_self_test_result != 0) {
		printk("DMA abort/recovery self-test failed: %d\n",
		       imu_abort_recovery_self_test_result);
		return imu_abort_recovery_self_test_result;
	}
	printk("DMA abort/recovery self-test OK\n");

	for (;;) {
		k_sem_take(&imu_data_ready, K_FOREVER);
		const uint32_t drdy_cycle = imu_latest_data_ready_cycle;
		rc = ImuReadSixAxesAsync();
		if (rc == 0) {
			rc = k_sem_take(&imu_transfer_done, K_MSEC(IMU_TRANSFER_TIMEOUT_MS));
		}
		if (rc != 0) {
			if (rc == -EAGAIN) {
				imu_transfer_timeout_count++;
				const int abort_rc = spi_release(spi_dev, &imu_spi_config);
				if (abort_rc == 0) {
					imu_transfer_abort_count++;
				} else {
					imu_transfer_abort_error_count++;
				}
				imu_scope_last_rc = (abort_rc == 0) ? -ETIMEDOUT : abort_rc;
			} else {
				imu_scope_last_rc = rc;
			}
			continue;
		}

		for (size_t axis = 0U; axis < 3U; ++axis) {
			imu_accel_raw[axis] = DecodeBigEndian(&imu_rx_buffer[1U + axis * 2U]);
			imu_gyro_raw[axis] = DecodeBigEndian(&imu_rx_buffer[7U + axis * 2U]);
		}
		imu_acquisition_latency_cycles = k_cycle_get_32() - drdy_cycle;
		imu_scope_sequence++;

		if ((imu_scope_sequence % 200U) == 1U) {
			printk("seq=%u drdy=%u drop=%u cb=%u out=%u worker=%u "
			       "acc=[%d %d %d] gyro=[%d %d %d] rc=%d\n",
			       imu_scope_sequence, imu_data_ready_count,
			       imu_dropped_data_ready_count,
			       imu_async_callback_sequence,
			       imu_async_outstanding_on_return_sequence,
			       imu_worker_during_dma_last,
			       imu_accel_raw[0], imu_accel_raw[1], imu_accel_raw[2],
			       imu_gyro_raw[0], imu_gyro_raw[1], imu_gyro_raw[2],
			       imu_scope_last_rc);
		}
	}
	return 0;
}
