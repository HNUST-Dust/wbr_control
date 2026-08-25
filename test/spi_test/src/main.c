/**
 * @file test/spi_test/src/main.c
 * @brief 实现应用或测试程序的入口与初始化流程。
 * @details 该文件属于独立 Zephyr 测试镜像，只验证指定外设或算法路径，不会链接进主固件。测试会直接访问目标硬件并通过串口输出判定结果。
 */

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "hpm_soc.h"

/* ========== 根据实际硬件修改这里 ========== */
#define IMU_SPI_NODE DT_NODELABEL(spi2) /* HPM6750EVK2: spi2 */
#define IMU_SPI_SLAVE 0                 /* cs-gpios 中的第 0 个 */
#define IMU_CS_NODE DT_NODELABEL(imu_cs)
#define IMU_WHO_AM_I_REG 0x0FU
#define IMU_WHO_AM_I_EXPECTED 0x6AU
#define IMU_SPI_FREQ_HZ 100000U
#define IMU_CS_DEBUG_HOLD_MS 2
#define IMU_POWER_ON_DELAY_MS 100
#define IMU_BITBANG_HALF_PERIOD_US 10
#define IMU_I2C_HALF_PERIOD_US 20
#define IMU_KEEP_CS_LOW_FOR_SPI 0
#define RUN_GPIO_PROBES 0
#define IMU_MISO_TRACE_SAMPLES 8
#define IMU_VERBOSE_TRANSFER_LOG 1
#define IMU_RUN_I2C_PROBES 0
#define IMU_PROBE_EVERY_HEARTBEAT 1
#define IMU_SCOPE_SPI_ONLY 1
#define IMU_SCOPE_SOFTWARE_SPI 1
#define IMU_SCOPE_SPI_INTERVAL_MS 20
/* ======================================== */

#if !DT_NODE_HAS_STATUS(IMU_SPI_NODE, okay)
#error "IMU SPI is not enabled in devicetree"
#endif

#if !DT_NODE_EXISTS(IMU_CS_NODE)
#error "imu_cs is missing; build spi_test with hpm6750evk2.overlay"
#endif

#if DT_NODE_HAS_PROP(IMU_SPI_NODE, cs_gpios)
#error "spi2 still has cs-gpios; build spi_test with the no-CS overlay"
#endif

static const struct device *const spi_dev = DEVICE_DT_GET(IMU_SPI_NODE);
static const struct device *const console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static const struct device *const gpioa_dev = DEVICE_DT_GET(DT_NODELABEL(gpioa));
static const struct device *const gpiob_dev = DEVICE_DT_GET(DT_NODELABEL(gpiob));
static const struct gpio_dt_spec imu_cs = GPIO_DT_SPEC_GET(IMU_CS_NODE, gpios);

static const struct spi_config spi_mode3_lsb_cfg = {
	.frequency = IMU_SPI_FREQ_HZ,
	.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |
		     SPI_TRANSFER_LSB | SPI_MODE_CPOL | SPI_MODE_CPHA,
	.slave = IMU_SPI_SLAVE,
};

static const struct spi_config spi_mode3_msb_cfg = {
	.frequency = IMU_SPI_FREQ_HZ,
	.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |
		     SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA,
	.slave = IMU_SPI_SLAVE,
};

static const struct spi_config spi_mode0_msb_cfg = {
	.frequency = IMU_SPI_FREQ_HZ,
	.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
	.slave = IMU_SPI_SLAVE,
};

static const struct spi_config spi_mode1_msb_cfg = {
	.frequency = IMU_SPI_FREQ_HZ,
	.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |
		     SPI_TRANSFER_MSB | SPI_MODE_CPHA,
	.slave = IMU_SPI_SLAVE,
};

static const struct spi_config spi_mode2_msb_cfg = {
	.frequency = IMU_SPI_FREQ_HZ,
	.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |
		     SPI_TRANSFER_MSB | SPI_MODE_CPOL,
	.slave = IMU_SPI_SLAVE,
};

static void ConsoleWriteRaw(const char *text)
{
	if (!device_is_ready(console_dev)) {
		return;
	}

	while (*text != '\0') {
		uart_poll_out(console_dev, *text++);
	}
}

static void cs_low(void)
{
	gpio_pin_set_raw(imu_cs.port, imu_cs.pin, 0);
}

static void cs_high(void)
{
	gpio_pin_set_raw(imu_cs.port, imu_cs.pin, 1);
}

static void cs_idle(void)
{
#if IMU_KEEP_CS_LOW_FOR_SPI
	cs_low();
#else
	cs_high();
#endif
}

#if RUN_GPIO_PROBES
static void ProbeCsGpio(void)
{
	printk("Probing CS GPIO raw toggle: PA%u should go low/high\n", imu_cs.pin);
	for (int i = 0; i < 4; i++) {
		cs_low();
		printk("  CS raw low\n");
		k_sleep(K_MSEC(100));
		cs_high();
		printk("  CS raw high\n");
		k_sleep(K_MSEC(100));
	}
}
#endif

static void SetSpi2Pinmux(void)
{
	HPM_IOC->PAD[IOC_PAD_PA27].FUNC_CTL =
		IOC_PA27_FUNC_CTL_SPI2_MOSI | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
	HPM_IOC->PAD[IOC_PAD_PA31].FUNC_CTL =
		IOC_PA31_FUNC_CTL_SPI2_MISO | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
	HPM_IOC->PAD[IOC_PAD_PB00].FUNC_CTL =
		IOC_PB00_FUNC_CTL_SPI2_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
}

static void SetSpiPinsGpioMux(void)
{
	HPM_IOC->PAD[IOC_PAD_PA27].FUNC_CTL =
		IOC_PA27_FUNC_CTL_GPIO_A_27 | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
	HPM_IOC->PAD[IOC_PAD_PA31].FUNC_CTL =
		IOC_PA31_FUNC_CTL_GPIO_A_31 | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
	HPM_IOC->PAD[IOC_PAD_PB00].FUNC_CTL =
		IOC_PB00_FUNC_CTL_GPIO_B_00 | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
}

static int MisoLevelRaw(void)
{
	return gpio_pin_get_raw(gpioa_dev, 31);
}

static char MisoTraceChar(void)
{
	int level = MisoLevelRaw();

	if (level < 0) {
		return '?';
	}

	return level > 0 ? '1' : '0';
}

static void TraceAppendChar(char *trace, size_t trace_size, size_t *trace_len, char value)
{
	if (*trace_len + 1U >= trace_size) {
		return;
	}

	trace[*trace_len] = value;
	(*trace_len)++;
	trace[*trace_len] = '\0';
}

static void MisoTraceAppend(char *trace, size_t trace_size, size_t *trace_len)
{
	TraceAppendChar(trace, trace_size, trace_len, MisoTraceChar());
}

static void SampleMisoBurst(const char *tag)
{
	char trace[IMU_MISO_TRACE_SAMPLES + 1U] = { 0 };

	for (size_t i = 0U; i < IMU_MISO_TRACE_SAMPLES; i++) {
		trace[i] = MisoTraceChar();
		k_busy_wait(IMU_BITBANG_HALF_PERIOD_US);
	}

	if (IMU_VERBOSE_TRANSFER_LOG) {
		printk("MISO burst (%s): %s\n", tag, trace);
	}
}

#if RUN_GPIO_PROBES
static void ProbeSpiPinsAsGpio(void)
{
	int rc;

	printk("Probing SPI pins as GPIO: PA27=MOSI, PB00=SCLK should toggle\n");
	SetSpiPinsGpioMux();

	rc = gpio_pin_configure(gpioa_dev, 27, GPIO_OUTPUT_LOW);
	if (rc != 0) {
		printk("  PA27 GPIO configure failed: %d\n", rc);
	}
	rc = gpio_pin_configure(gpiob_dev, 0, GPIO_OUTPUT_LOW);
	if (rc != 0) {
		printk("  PB00 GPIO configure failed: %d\n", rc);
	}
	rc = gpio_pin_configure(gpioa_dev, 31, GPIO_INPUT);
	if (rc != 0) {
		printk("  PA31 GPIO input configure failed: %d\n", rc);
	}

	for (int i = 0; i < 4; i++) {
		gpio_pin_set_raw(gpioa_dev, 27, 1);
		gpio_pin_set_raw(gpiob_dev, 0, 1);
		printk("  MOSI/SCLK raw high, MISO raw=%d\n",
		       gpio_pin_get_raw(gpioa_dev, 31));
		k_sleep(K_MSEC(100));

		gpio_pin_set_raw(gpioa_dev, 27, 0);
		gpio_pin_set_raw(gpiob_dev, 0, 0);
		printk("  MOSI/SCLK raw low, MISO raw=%d\n",
		       gpio_pin_get_raw(gpioa_dev, 31));
		k_sleep(K_MSEC(100));
	}

	SetSpi2Pinmux();
	printk("Restored PA27/PA31/PB00 to SPI2 pinmux\n");
}
#endif

static void SampleMisoIdle(const char *tag)
{
	int rc;
	int cs_high_miso;
	int cs_low_miso;

	SetSpiPinsGpioMux();
	rc = gpio_pin_configure(gpioa_dev, 31, GPIO_INPUT);
	if (rc != 0) {
		printk("MISO GPIO input configure failed: %d\n", rc);
		SetSpi2Pinmux();
		return;
	}

	cs_high();
	k_sleep(K_MSEC(1));
	cs_high_miso = gpio_pin_get_raw(gpioa_dev, 31);
	SampleMisoBurst("idle, CS high");
	cs_low();
	k_sleep(K_MSEC(1));
	cs_low_miso = gpio_pin_get_raw(gpioa_dev, 31);
	SampleMisoBurst("selected, CS low");
	cs_high();

	printk("MISO idle sample (%s): CS high=%d, CS low=%d\n",
	       tag, cs_high_miso, cs_low_miso);
	SetSpi2Pinmux();
}

static void ProbeInputBias(const char *name, const struct device *port, gpio_pin_t pin)
{
	int down;
	int up;

	SetSpiPinsGpioMux();
	cs_high();

	gpio_pin_configure(port, pin, GPIO_INPUT | GPIO_PULL_DOWN);
	k_sleep(K_MSEC(2));
	down = gpio_pin_get_raw(port, pin);

	gpio_pin_configure(port, pin, GPIO_INPUT | GPIO_PULL_UP);
	k_sleep(K_MSEC(2));
	up = gpio_pin_get_raw(port, pin);

	gpio_pin_configure(port, pin, GPIO_INPUT);
	printk("Input bias probe %s: pulldown=%d pullup=%d\n", name, down, up);

	SetSpi2Pinmux();
}

static void ProbeOutputDrive(const char *name, const struct device *port, uint32_t hpm_port,
			     gpio_pin_t pin)
{
	int low;
	int high;
	uint32_t oe_low;
	uint32_t do_low;
	uint32_t di_low;
	uint32_t oe_high;
	uint32_t do_high;
	uint32_t di_high;
	int rc;

	SetSpiPinsGpioMux();

	rc = gpio_pin_configure(port, pin, GPIO_OUTPUT_LOW);
	if (rc != 0) {
		printk("Output drive probe %s: configure low failed rc=%d\n", name, rc);
		SetSpi2Pinmux();
		return;
	}
	k_sleep(K_MSEC(1));
	low = gpio_pin_get_raw(port, pin);
	oe_low = HPM_GPIO0->OE[hpm_port].VALUE;
	do_low = HPM_GPIO0->DO[hpm_port].VALUE;
	di_low = HPM_GPIO0->DI[hpm_port].VALUE;

	rc = gpio_pin_configure(port, pin, GPIO_OUTPUT_HIGH);
	if (rc != 0) {
		printk("Output drive probe %s: configure high failed rc=%d\n", name, rc);
		SetSpi2Pinmux();
		return;
	}
	k_sleep(K_MSEC(1));
	high = gpio_pin_get_raw(port, pin);
	oe_high = HPM_GPIO0->OE[hpm_port].VALUE;
	do_high = HPM_GPIO0->DO[hpm_port].VALUE;
	di_high = HPM_GPIO0->DI[hpm_port].VALUE;

	gpio_pin_configure(port, pin, GPIO_INPUT);
	printk("Output drive probe %s: drive-low-read=%d drive-high-read=%d",
	       name, low, high);
	if (IMU_VERBOSE_TRANSFER_LOG) {
		printk(" | low(OE=%08x DO=%08x DI=%08x) high(OE=%08x DO=%08x DI=%08x)",
		       oe_low, do_low, di_low, oe_high, do_high, di_high);
	}
	printk("\n");

	SetSpi2Pinmux();
}

static uint8_t bit_reverse8(uint8_t v)
{
	v = (uint8_t)(((v & 0xF0U) >> 4) | ((v & 0x0FU) << 4));
	v = (uint8_t)(((v & 0xCCU) >> 2) | ((v & 0x33U) << 2));
	v = (uint8_t)(((v & 0xAAU) >> 1) | ((v & 0x55U) << 1));
	return v;
}

static void i2c_scl_set(bool high)
{
	gpio_pin_set_raw(gpiob_dev, 0, high ? 1 : 0);
	k_busy_wait(IMU_I2C_HALF_PERIOD_US);
}

static void i2c_sda_drive(bool high)
{
	if (high) {
		gpio_pin_configure(gpioa_dev, 27, GPIO_INPUT);
	} else {
		gpio_pin_configure(gpioa_dev, 27, GPIO_OUTPUT_LOW);
		gpio_pin_set_raw(gpioa_dev, 27, 0);
	}
	k_busy_wait(IMU_I2C_HALF_PERIOD_US);
}

static bool i2c_sda_sample(void)
{
	gpio_pin_configure(gpioa_dev, 27, GPIO_INPUT);
	k_busy_wait(IMU_I2C_HALF_PERIOD_US);
	return gpio_pin_get_raw(gpioa_dev, 27) > 0;
}

static void i2c_start(void)
{
	i2c_sda_drive(true);
	i2c_scl_set(true);
	i2c_sda_drive(false);
	i2c_scl_set(false);
}

static void i2c_stop(void)
{
	i2c_sda_drive(false);
	i2c_scl_set(true);
	i2c_sda_drive(true);
}

static bool i2c_write_byte(uint8_t byte)
{
	bool ack;

	for (int i = 7; i >= 0; i--) {
		i2c_sda_drive((byte & BIT(i)) != 0U);
		i2c_scl_set(true);
		i2c_scl_set(false);
	}

	gpio_pin_configure(gpioa_dev, 27, GPIO_INPUT);
	k_busy_wait(IMU_I2C_HALF_PERIOD_US);
	i2c_scl_set(true);
	ack = !i2c_sda_sample();
	i2c_scl_set(false);
	return ack;
}

static uint8_t i2c_read_byte_nack(void)
{
	uint8_t byte = 0U;

	gpio_pin_configure(gpioa_dev, 27, GPIO_INPUT);
	for (int i = 7; i >= 0; i--) {
		i2c_scl_set(true);
		if (gpio_pin_get_raw(gpioa_dev, 27) > 0) {
			byte |= BIT(i);
		}
		i2c_scl_set(false);
	}

	i2c_sda_drive(true);
	i2c_scl_set(true);
	i2c_scl_set(false);
	return byte;
}

static int i2c_read_reg_once(uint8_t addr7, uint8_t reg, uint8_t *val)
{
	bool ack_addr_w;
	bool ack_reg;
	bool ack_addr_r;

	i2c_start();
	ack_addr_w = i2c_write_byte((uint8_t)(addr7 << 1));
	ack_reg = i2c_write_byte(reg);
	i2c_start();
	ack_addr_r = i2c_write_byte((uint8_t)((addr7 << 1) | 0x01U));
	*val = i2c_read_byte_nack();
	i2c_stop();

	return (ack_addr_w && ack_reg && ack_addr_r) ? 0 : -EIO;
}

static int imu_read_whoami_bitbang_i2c(uint8_t addr7, uint8_t *val)
{
	uint8_t scan_val;
	bool ack_addr_w;
	bool ack_reg;
	bool ack_addr_r;

	cs_high();
	SetSpiPinsGpioMux();
	gpio_pin_configure(gpiob_dev, 0, GPIO_OUTPUT_HIGH);
	gpio_pin_configure(gpioa_dev, 27, GPIO_OUTPUT_HIGH);
	i2c_sda_drive(true);
	i2c_scl_set(true);

	i2c_start();
	ack_addr_w = i2c_write_byte((uint8_t)(addr7 << 1));
	ack_reg = i2c_write_byte(IMU_WHO_AM_I_REG);
	i2c_start();
	ack_addr_r = i2c_write_byte((uint8_t)((addr7 << 1) | 0x01U));
	*val = i2c_read_byte_nack();
	i2c_stop();

	SetSpi2Pinmux();
	if (IMU_VERBOSE_TRANSFER_LOG) {
		printk("  [bitbang-i2c addr=0x%02X] ackW=%d ackReg=%d ackR=%d | WHO_AM_I=0x%02X\n",
		       addr7, ack_addr_w, ack_reg, ack_addr_r, *val);
	}

	if (IMU_VERBOSE_TRANSFER_LOG) {
		printk("  [bitbang-i2c addr=0x%02X scan 00-0F]", addr7);
		for (uint8_t reg = 0U; reg <= 0x0FU; reg++) {
			if (i2c_read_reg_once(addr7, reg, &scan_val) == 0) {
				printk(" %02X:%02X", reg, scan_val);
			} else {
				printk(" %02X:--", reg);
			}
		}
		printk("\n");
	}

	return (ack_addr_w && ack_reg && ack_addr_r) ? 0 : -EIO;
}

static void bitbang_sclk_low(void)
{
	gpio_pin_set_raw(gpiob_dev, 0, 0);
	k_busy_wait(IMU_BITBANG_HALF_PERIOD_US);
}

static void bitbang_sclk_high(void)
{
	gpio_pin_set_raw(gpiob_dev, 0, 1);
	k_busy_wait(IMU_BITBANG_HALF_PERIOD_US);
}

static void bitbang_write_bit_mode3(bool bit)
{
	/* IMU manual: SDI/SDO drive on SPC falling edge, read on rising edge. */
	bitbang_sclk_low();
	gpio_pin_set_raw(gpioa_dev, 27, bit ? 1 : 0);
	bitbang_sclk_high();
}

static bool bitbang_read_sdi_bit_mode3(void)
{
	bool bit;

	bitbang_sclk_low();
	bitbang_sclk_high();
	bit = gpio_pin_get_raw(gpioa_dev, 27) > 0;
	return bit;
}

static void bitbang_write_bit_mode0(bool bit)
{
	gpio_pin_set_raw(gpioa_dev, 27, bit ? 1 : 0);
	bitbang_sclk_high();
	bitbang_sclk_low();
}

static void bitbang_write_bit(bool mode3, bool bit)
{
	if (mode3) {
		bitbang_write_bit_mode3(bit);
	} else {
		bitbang_write_bit_mode0(bit);
	}
}

static bool bitbang_read_bit_with_phase_trace(bool mode3,
					      char *low_trace, size_t low_trace_size,
					      size_t *low_trace_len,
					      char *high_trace, size_t high_trace_size,
					      size_t *high_trace_len)
{
	char sampled;

	if (mode3) {
		bitbang_sclk_low();
		MisoTraceAppend(low_trace, low_trace_size, low_trace_len);
		bitbang_sclk_high();
		sampled = MisoTraceChar();
		TraceAppendChar(high_trace, high_trace_size, high_trace_len, sampled);
	} else {
		bitbang_sclk_high();
		sampled = MisoTraceChar();
		TraceAppendChar(high_trace, high_trace_size, high_trace_len, sampled);
		bitbang_sclk_low();
		MisoTraceAppend(low_trace, low_trace_size, low_trace_len);
	}

	return sampled == '1';
}

static int imu_read_reg_bitbang_variant(const char *name, uint8_t cmd, bool mode3,
					bool cmd_lsb_first, bool data_msb_first,
					uint8_t *val)
{
	uint8_t rx = 0U;
	char cmd_miso_trace[9] = { 0 };
	char data_miso_trace[9] = { 0 };
	char data_low_trace[9] = { 0 };
	char data_high_trace[9] = { 0 };
	size_t cmd_trace_len = 0U;
	size_t data_trace_len = 0U;
	size_t data_low_trace_len = 0U;
	size_t data_high_trace_len = 0U;
	char before_select;
	char after_select;
	char after_read;
	int rc;

	SetSpiPinsGpioMux();

	rc = gpio_pin_configure(gpioa_dev, 27, GPIO_OUTPUT_LOW);
	if (rc != 0) {
		printk("  [bitbang] MOSI GPIO configure failed: %d\n", rc);
		SetSpi2Pinmux();
		return rc;
	}
	rc = gpio_pin_configure(gpiob_dev, 0, GPIO_OUTPUT_HIGH);
	if (rc != 0) {
		printk("  [bitbang] SCLK GPIO configure failed: %d\n", rc);
		SetSpi2Pinmux();
		return rc;
	}
	rc = gpio_pin_configure(gpioa_dev, 31, GPIO_INPUT);
	if (rc != 0) {
		printk("  [bitbang] MISO GPIO configure failed: %d\n", rc);
		SetSpi2Pinmux();
		return rc;
	}

	if (mode3) {
		gpio_pin_set_raw(gpiob_dev, 0, 1);
	} else {
		gpio_pin_set_raw(gpiob_dev, 0, 0);
	}

	before_select = MisoTraceChar();
	cs_low();
	k_busy_wait(IMU_BITBANG_HALF_PERIOD_US);
	after_select = MisoTraceChar();

	for (int n = 0; n < 8; n++) {
		int bit = cmd_lsb_first ? n : (7 - n);

		bitbang_write_bit(mode3, (cmd & BIT(bit)) != 0U);
		MisoTraceAppend(cmd_miso_trace, sizeof(cmd_miso_trace), &cmd_trace_len);
	}

	for (int n = 0; n < 8; n++) {
		int bit = data_msb_first ? (7 - n) : n;
		bool sampled = bitbang_read_bit_with_phase_trace(
			mode3, data_low_trace, sizeof(data_low_trace), &data_low_trace_len,
			data_high_trace, sizeof(data_high_trace), &data_high_trace_len);

		if (sampled) {
			rx |= BIT(bit);
		}
		TraceAppendChar(data_miso_trace, sizeof(data_miso_trace), &data_trace_len,
				sampled ? '1' : '0');
	}
	after_read = MisoTraceChar();

	cs_idle();
	gpio_pin_set_raw(gpiob_dev, 0, mode3 ? 1 : 0);
	SetSpi2Pinmux();

	*val = rx;
	if (IMU_VERBOSE_TRANSFER_LOG) {
		printk("  [%s] cmd=0x%02X | data=0x%02X | bitrev=0x%02X\n",
		       name, cmd, rx, bit_reverse8(rx));
		printk("    MISO trace PA31: before-CS=%c after-CS=%c cmd-clocks=%s data-clocks=%s after-read=%c\n",
		       before_select, after_select, cmd_miso_trace, data_miso_trace, after_read);
		printk("    MISO phase PA31: data-low=%s data-high=%s sampled-on=%s\n",
		       data_low_trace, data_high_trace,
		       mode3 ? "high(mode3)" : "high-before-low(mode0)");
	}
	return 0;
}

static int imu_read_reg_bitbang_3wire(uint8_t reg, uint8_t *val)
{
	uint8_t cmd = (uint8_t)((reg << 1) | 0x01U);
	uint8_t rx = 0U;
	char before_select;
	char after_select;
	char data_sdi_trace[9] = { 0 };
	size_t data_trace_len = 0U;
	int rc;

	SetSpiPinsGpioMux();

	rc = gpio_pin_configure(gpioa_dev, 27, GPIO_OUTPUT_LOW);
	if (rc != 0) {
		printk("  [bitbang-3wire] SDI GPIO output configure failed: %d\n", rc);
		SetSpi2Pinmux();
		return rc;
	}
	rc = gpio_pin_configure(gpiob_dev, 0, GPIO_OUTPUT_HIGH);
	if (rc != 0) {
		printk("  [bitbang-3wire] SCLK GPIO configure failed: %d\n", rc);
		SetSpi2Pinmux();
		return rc;
	}

	before_select = MisoTraceChar();
	cs_low();
	k_busy_wait(IMU_BITBANG_HALF_PERIOD_US);
	after_select = MisoTraceChar();

	for (int n = 0; n < 8; n++) {
		bitbang_write_bit(true, (cmd & BIT(n)) != 0U);
	}

	rc = gpio_pin_configure(gpioa_dev, 27, GPIO_INPUT);
	if (rc != 0) {
		printk("  [bitbang-3wire] SDI GPIO input configure failed: %d\n", rc);
		cs_high();
		SetSpi2Pinmux();
		return rc;
	}

	for (int n = 0; n < 8; n++) {
		int bit = 7 - n;
		bool sampled = bitbang_read_sdi_bit_mode3();

		if (sampled) {
			rx |= BIT(bit);
		}
		TraceAppendChar(data_sdi_trace, sizeof(data_sdi_trace), &data_trace_len,
				sampled ? '1' : '0');
	}

	cs_high();
	gpio_pin_set_raw(gpiob_dev, 0, 1);
	SetSpi2Pinmux();

	*val = rx;
	if (IMU_VERBOSE_TRANSFER_LOG) {
		printk("  [bitbang-3wire mode3 SDI-as-SDO] cmd=0x%02X | data=0x%02X | bitrev=0x%02X\n",
		       cmd, rx, bit_reverse8(rx));
		printk("    SDI trace PA27: before-CS MISO(PA31)=%c after-CS MISO(PA31)=%c data-clocks=%s\n",
		       before_select, after_select, data_sdi_trace);
	}
	return 0;
}

static int imu_read_reg_bitbang(uint8_t reg, uint8_t *val)
{
	uint8_t candidate;
	uint8_t cmd_hxy = (uint8_t)((reg << 1) | 0x01U);
	uint8_t cmd_common = (uint8_t)(0x80U | reg);
	int rc;

	rc = imu_read_reg_bitbang_variant("bitbang-mode3 cmd=0x03 lsb/msb",
					  cmd_hxy, true, true, true, val);
	if (rc == 0 && *val == IMU_WHO_AM_I_EXPECTED) {
		return 0;
	}

	rc = imu_read_reg_bitbang_variant("bitbang-mode3 cmd=0x81 msb/msb",
					  cmd_common, true, false, true, &candidate);
	if (rc == 0 && candidate == IMU_WHO_AM_I_EXPECTED) {
		*val = candidate;
		return 0;
	}

	rc = imu_read_reg_bitbang_variant("bitbang-mode3 cmd=0xC0 msb/msb",
					  bit_reverse8(cmd_hxy), true, false, true, &candidate);
	if (rc == 0 && candidate == IMU_WHO_AM_I_EXPECTED) {
		*val = candidate;
		return 0;
	}

	rc = imu_read_reg_bitbang_variant("bitbang-mode0 cmd=0x03 lsb/msb",
					  cmd_hxy, false, true, true, &candidate);
	if (rc == 0 && candidate == IMU_WHO_AM_I_EXPECTED) {
		*val = candidate;
		return 0;
	}

	rc = imu_read_reg_bitbang_variant("bitbang-mode0 cmd=0x81 msb/msb",
					  cmd_common, false, false, true, &candidate);
	if (rc == 0 && candidate == IMU_WHO_AM_I_EXPECTED) {
		*val = candidate;
		return 0;
	}

	rc = imu_read_reg_bitbang_3wire(reg, &candidate);
	if (rc == 0 && candidate == IMU_WHO_AM_I_EXPECTED) {
		*val = candidate;
		return 0;
	}

	return rc;
}

/* 手册协议：第一个发送 bit 是 R/W，随后 7 bit 地址；读数据 byte 为 MSB first。 */
static int imu_read_reg_split_lsb_cmd(uint8_t reg, uint8_t *val)
{
	uint8_t cmd = (uint8_t)((reg << 1) | 0x01U);
	uint8_t rx = 0U;
	const struct spi_buf tx_buf = { .buf = &cmd, .len = sizeof(cmd) };
	const struct spi_buf rx_buf = { .buf = &rx, .len = sizeof(rx) };
	const struct spi_buf_set tx_bufs = { .buffers = &tx_buf, .count = 1 };
	const struct spi_buf_set rx_bufs = { .buffers = &rx_buf, .count = 1 };
	char before_transfer;
	char after_write;
	char after_read;
	int rc;

	cs_low();
	if (IMU_VERBOSE_TRANSFER_LOG) {
		printk("  CS low before transfer\n");
	}
	k_sleep(K_MSEC(IMU_CS_DEBUG_HOLD_MS));
	before_transfer = MisoTraceChar();

	rc = spi_write(spi_dev, &spi_mode3_lsb_cfg, &tx_bufs);
	after_write = MisoTraceChar();
	if (rc == 0) {
		k_sleep(K_MSEC(IMU_CS_DEBUG_HOLD_MS));
		rc = spi_read(spi_dev, &spi_mode3_msb_cfg, &rx_bufs);
	}
	after_read = MisoTraceChar();

	k_sleep(K_MSEC(IMU_CS_DEBUG_HOLD_MS));
	cs_high();
	if (IMU_VERBOSE_TRANSFER_LOG) {
		printk("  CS high after transfer\n");
	}

	if (rc != 0) {
		return rc;
	}

	*val = rx;
	if (IMU_VERBOSE_TRANSFER_LOG) {
		printk("  [reg=0x%02X] cmd=0x%02X (LSB first) | rx=0x%02X | data=0x%02X\n",
		       reg, cmd, rx, rx);
		printk("    MISO level PA31: before-transfer=%c after-write=%c after-read=%c\n",
		       before_transfer, after_write, after_read);
	}
	return 0;
}

static int imu_read_reg_2byte(const char *name, const struct spi_config *cfg,
			     uint8_t cmd, uint8_t *val)
{
	uint8_t tx[2] = { cmd, 0x00U };
	uint8_t rx[2] = { 0U, 0U };
	const struct spi_buf tx_buf = { .buf = tx, .len = sizeof(tx) };
	const struct spi_buf rx_buf = { .buf = rx, .len = sizeof(rx) };
	const struct spi_buf_set tx_bufs = { .buffers = &tx_buf, .count = 1 };
	const struct spi_buf_set rx_bufs = { .buffers = &rx_buf, .count = 1 };
	char before_transfer;
	char after_transfer;
	int rc;

	cs_low();
	k_sleep(K_MSEC(IMU_CS_DEBUG_HOLD_MS));
	before_transfer = MisoTraceChar();
	rc = spi_transceive(spi_dev, cfg, &tx_bufs, &rx_bufs);
	after_transfer = MisoTraceChar();
	k_sleep(K_MSEC(IMU_CS_DEBUG_HOLD_MS));
	cs_high();

	if (rc != 0) {
		if (IMU_VERBOSE_TRANSFER_LOG) {
			printk("  [%s] failed rc=%d\n", name, rc);
		}
		return rc;
	}

	*val = rx[1];
	if (IMU_VERBOSE_TRANSFER_LOG) {
		printk("  [%s] tx=%02X %02X | rx=%02X %02X | data=0x%02X | bitrev=0x%02X\n",
		       name, tx[0], tx[1], rx[0], rx[1], rx[1], bit_reverse8(rx[1]));
		printk("    MISO level PA31: before-transfer=%c after-transfer=%c\n",
		       before_transfer, after_transfer);
	}
	return 0;
}

static int imu_scope_spi_read_once(uint8_t *val)
{
	uint8_t tx[2] = { (uint8_t)(0x80U | IMU_WHO_AM_I_REG), 0x00U };
	uint8_t rx[2] = { 0U, 0U };
	const struct spi_buf tx_buf = { .buf = tx, .len = sizeof(tx) };
	const struct spi_buf rx_buf = { .buf = rx, .len = sizeof(rx) };
	const struct spi_buf_set tx_bufs = { .buffers = &tx_buf, .count = 1 };
	const struct spi_buf_set rx_bufs = { .buffers = &rx_buf, .count = 1 };
	int rc;

	cs_low();
	rc = spi_transceive(spi_dev, &spi_mode3_msb_cfg, &tx_bufs, &rx_bufs);
	cs_high();

	*val = rx[1];
	return rc;
}

static bool software_spi_mode3_transfer_bit(bool mosi)
{
	bool miso;

	gpio_pin_set_raw(gpiob_dev, 0, 0);
	k_busy_wait(IMU_BITBANG_HALF_PERIOD_US);
	gpio_pin_set_raw(gpioa_dev, 27, mosi ? 1 : 0);
	gpio_pin_set_raw(gpiob_dev, 0, 1);
	k_busy_wait(IMU_BITBANG_HALF_PERIOD_US);
	miso = gpio_pin_get_raw(gpioa_dev, 31) > 0;

	return miso;
}

static uint8_t software_spi_mode3_transfer_byte(uint8_t tx)
{
	uint8_t rx = 0U;

	for (int bit = 7; bit >= 0; bit--) {
		if (software_spi_mode3_transfer_bit((tx & BIT(bit)) != 0U)) {
			rx |= BIT(bit);
		}
	}

	return rx;
}

static int imu_scope_software_spi_read_once(uint8_t *val)
{
	int rc;
	uint8_t rx0;
	uint8_t rx1;

	SetSpiPinsGpioMux();

	rc = gpio_pin_configure(gpioa_dev, 27, GPIO_OUTPUT_LOW);
	if (rc != 0) {
		return rc;
	}
	rc = gpio_pin_configure(gpiob_dev, 0, GPIO_OUTPUT_HIGH);
	if (rc != 0) {
		return rc;
	}
	rc = gpio_pin_configure(gpioa_dev, 31, GPIO_INPUT);
	if (rc != 0) {
		return rc;
	}

	gpio_pin_set_raw(gpioa_dev, 27, 0);
	gpio_pin_set_raw(gpiob_dev, 0, 1);
	k_busy_wait(IMU_BITBANG_HALF_PERIOD_US);

	cs_low();
	k_busy_wait(IMU_BITBANG_HALF_PERIOD_US);
	rx0 = software_spi_mode3_transfer_byte((uint8_t)(0x80U | IMU_WHO_AM_I_REG));
	rx1 = software_spi_mode3_transfer_byte(0x00U);
	k_busy_wait(IMU_BITBANG_HALF_PERIOD_US);
	cs_high();

	gpio_pin_set_raw(gpiob_dev, 0, 1);
	ARG_UNUSED(rx0);
	*val = rx1;
	return 0;
}

static void RunScopeSpiLoop(void)
{
	uint8_t val;
	uint32_t seq = 0U;
	int rc;

	printk("scope %s SPI loop: mode3 MSB, cmd=0x%02X, interval=%u ms",
	       IMU_SCOPE_SOFTWARE_SPI ? "software" : "hardware",
	       (uint8_t)(0x80U | IMU_WHO_AM_I_REG),
	       IMU_SCOPE_SPI_INTERVAL_MS);
	if (IMU_SCOPE_SOFTWARE_SPI) {
		printk(", half-period=%u us\n", IMU_BITBANG_HALF_PERIOD_US);
	} else {
		printk(", freq=%u Hz\n", IMU_SPI_FREQ_HZ);
	}
	printk("probe CS=PA%u, MOSI=PA27, MISO=PA31, SCLK=PB00\n", imu_cs.pin);

	while (1) {
		if (IMU_SCOPE_SOFTWARE_SPI) {
			rc = imu_scope_software_spi_read_once(&val);
		} else {
			rc = imu_scope_spi_read_once(&val);
		}
		if ((seq % 50U) == 0U) {
			if (rc == 0) {
				printk("scope SPI seq=%u WHO_AM_I/raw=0x%02X\n", seq, val);
			} else {
				printk("scope SPI seq=%u rc=%d\n", seq, rc);
			}
		}
		seq++;
		k_sleep(K_MSEC(IMU_SCOPE_SPI_INTERVAL_MS));
	}
}

static int imu_read_reg(uint8_t reg, uint8_t *val)
{
	uint8_t candidate;
	uint8_t found = 0U;
	int rc = imu_read_reg_bitbang(reg, val);

	if (rc == 0 && *val == IMU_WHO_AM_I_EXPECTED) {
		return 0;
	}
	if (rc != 0) {
		*val = 0xFFU;
	}

	if (IMU_RUN_I2C_PROBES) {
		rc = imu_read_whoami_bitbang_i2c(0x18U, &candidate);
		if (rc == 0 && candidate == IMU_WHO_AM_I_EXPECTED) {
			*val = candidate;
			return 0;
		}

		rc = imu_read_whoami_bitbang_i2c(0x19U, &candidate);
		if (rc == 0 && candidate == IMU_WHO_AM_I_EXPECTED) {
			*val = candidate;
			return 0;
		}
	}

	rc = imu_read_reg_split_lsb_cmd(reg, val);

	if (rc == 0 && *val == IMU_WHO_AM_I_EXPECTED) {
		return 0;
	}
	if (rc != 0) {
		*val = 0xFFU;
	}

	rc = imu_read_reg_2byte("mode3-msb cmd=0x81", &spi_mode3_msb_cfg,
				(uint8_t)(0x80U | reg), &candidate);
	if (rc == 0 && candidate == IMU_WHO_AM_I_EXPECTED) {
		*val = candidate;
		found = 1U;
	}

	rc = imu_read_reg_2byte("mode3-msb cmd=0x03", &spi_mode3_msb_cfg,
				(uint8_t)((reg << 1) | 0x01U), &candidate);
	if (!found && rc == 0 && candidate == IMU_WHO_AM_I_EXPECTED) {
		*val = candidate;
		found = 1U;
	}

	rc = imu_read_reg_2byte("mode3-msb cmd=bitrev(0x03)", &spi_mode3_msb_cfg,
				bit_reverse8((uint8_t)((reg << 1) | 0x01U)), &candidate);
	if (!found && rc == 0 && candidate == IMU_WHO_AM_I_EXPECTED) {
		*val = candidate;
		found = 1U;
	}

	rc = imu_read_reg_2byte("mode3-lsb cmd=0x03", &spi_mode3_lsb_cfg,
				(uint8_t)((reg << 1) | 0x01U), &candidate);
	if (!found && rc == 0 &&
	    (candidate == IMU_WHO_AM_I_EXPECTED ||
	     bit_reverse8(candidate) == IMU_WHO_AM_I_EXPECTED)) {
		*val = (candidate == IMU_WHO_AM_I_EXPECTED) ? candidate : bit_reverse8(candidate);
		found = 1U;
	}

	rc = imu_read_reg_2byte("mode0-msb cmd=0x81", &spi_mode0_msb_cfg,
				(uint8_t)(0x80U | reg), &candidate);
	if (!found && rc == 0 && candidate == IMU_WHO_AM_I_EXPECTED) {
		*val = candidate;
		found = 1U;
	}

	rc = imu_read_reg_2byte("mode0-msb cmd=bitrev(0x03)", &spi_mode0_msb_cfg,
				bit_reverse8((uint8_t)((reg << 1) | 0x01U)), &candidate);
	if (!found && rc == 0 && candidate == IMU_WHO_AM_I_EXPECTED) {
		*val = candidate;
		found = 1U;
	}

	rc = imu_read_reg_2byte("mode1-msb cmd=0x81", &spi_mode1_msb_cfg,
				(uint8_t)(0x80U | reg), &candidate);
	if (!found && rc == 0 && candidate == IMU_WHO_AM_I_EXPECTED) {
		*val = candidate;
		found = 1U;
	}

	rc = imu_read_reg_2byte("mode1-msb cmd=bitrev(0x03)", &spi_mode1_msb_cfg,
				bit_reverse8((uint8_t)((reg << 1) | 0x01U)), &candidate);
	if (!found && rc == 0 && candidate == IMU_WHO_AM_I_EXPECTED) {
		*val = candidate;
		found = 1U;
	}

	rc = imu_read_reg_2byte("mode2-msb cmd=0x81", &spi_mode2_msb_cfg,
				(uint8_t)(0x80U | reg), &candidate);
	if (!found && rc == 0 && candidate == IMU_WHO_AM_I_EXPECTED) {
		*val = candidate;
		found = 1U;
	}

	rc = imu_read_reg_2byte("mode2-msb cmd=bitrev(0x03)", &spi_mode2_msb_cfg,
				bit_reverse8((uint8_t)((reg << 1) | 0x01U)), &candidate);
	if (!found && rc == 0 && candidate == IMU_WHO_AM_I_EXPECTED) {
		*val = candidate;
		found = 1U;
	}

	ARG_UNUSED(found);
	return 0;
}

int main(void)
{
	uint8_t val;
	int rc;

	ConsoleWriteRaw("\r\n[raw-uart] spi_test main entered at 921600 baud\r\n");
	printk("lsm6 spi test booted\n");
	printk("SPI config: freq=%u Hz, mode=3 (CPOL|CPHA), slave=%d\n",
	       spi_mode3_lsb_cfg.frequency, spi_mode3_lsb_cfg.slave);

	if (!device_is_ready(spi_dev)) {
		printk("SPI device not ready\n");
		return -ENODEV;
	}
	printk("SPI device ready\n");

	if (!gpio_is_ready_dt(&imu_cs)) {
		printk("GPIO device not ready\n");
		return -ENODEV;
	}
	printk("GPIO device ready (CS pin=%u flags=0x%x)\n", imu_cs.pin, imu_cs.dt_flags);
	if (!device_is_ready(gpioa_dev) || !device_is_ready(gpiob_dev)) {
		printk("GPIOA/GPIOB device not ready\n");
		return -ENODEV;
	}

	rc = gpio_pin_configure(imu_cs.port, imu_cs.pin,
				IMU_KEEP_CS_LOW_FOR_SPI ? GPIO_OUTPUT_LOW : GPIO_OUTPUT_HIGH);
	if (rc != 0) {
		printk("CS configure failed: %d\n", rc);
		return rc;
	}

#if RUN_GPIO_PROBES
	ProbeCsGpio();
	ProbeSpiPinsAsGpio();
#endif
	cs_idle();
	printk("CS held %s, waiting %u ms for IMU interface startup\n",
	       IMU_KEEP_CS_LOW_FOR_SPI ? "low" : "high", IMU_POWER_ON_DELAY_MS);
	k_sleep(K_MSEC(IMU_POWER_ON_DELAY_MS));

#if IMU_SCOPE_SPI_ONLY
#if IMU_SCOPE_SOFTWARE_SPI
	SetSpiPinsGpioMux();
#else
	SetSpi2Pinmux();
#endif
	RunScopeSpiLoop();
#endif

	SampleMisoIdle("before first read");
	ProbeInputBias("PA27 SDX/MOSI", gpioa_dev, 27);
	ProbeInputBias("PA31 SDO/MISO", gpioa_dev, 31);
	ProbeOutputDrive("PA27 SDX/MOSI", gpioa_dev, GPIO_DO_GPIOA, 27);
	ProbeOutputDrive("PB00 SCLK", gpiob_dev, GPIO_DO_GPIOB, 0);

	printk("Reading WHO_AM_I (0x%02X)...\n", IMU_WHO_AM_I_REG);
	rc = imu_read_reg(IMU_WHO_AM_I_REG, &val);
	if (rc != 0) {
		printk("WHO_AM_I read failed: %d\n", rc);
		return rc;
	}
	printk("WHO_AM_I = 0x%02X (expected 0x%02X)\n", val, IMU_WHO_AM_I_EXPECTED);

	if (val == IMU_WHO_AM_I_EXPECTED) {
		printk("IMU detected!\n");
	} else if (val == 0xFF || val == 0x00) {
		printk("ERROR: read back 0x%02X\n", val);
	} else {
		printk("WARNING: unexpected WHO_AM_I\n");
	}

	while (1) {
		rc = imu_read_reg(IMU_WHO_AM_I_REG, &val);
		if (rc == 0) {
			printk("heartbeat WHO_AM_I = 0x%02X\n", val);
#if IMU_PROBE_EVERY_HEARTBEAT
			ProbeInputBias("PA27 SDX/MOSI", gpioa_dev, 27);
			ProbeInputBias("PA31 SDO/MISO", gpioa_dev, 31);
			ProbeOutputDrive("PA27 SDX/MOSI", gpioa_dev, GPIO_DO_GPIOA, 27);
			ProbeOutputDrive("PB00 SCLK", gpiob_dev, GPIO_DO_GPIOB, 0);
#endif
		} else {
			printk("heartbeat read failed: %d\n", rc);
		}
		k_sleep(K_SECONDS(2));
	}

	return 0;
}
