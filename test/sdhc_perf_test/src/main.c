/**
 * @file test/sdhc_perf_test/src/main.c
 * @brief 实现应用或测试程序的入口与初始化流程。
 * @details 该文件属于独立 Zephyr 测试镜像，只验证指定外设或算法路径，不会链接进主固件。测试会直接访问目标硬件并通过串口输出判定结果。
 */

#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/kernel.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/sys/printk.h>

#if defined(CONFIG_DISK_DRIVER_SDMMC)
#define TEST_DISK_NAME CONFIG_SDMMC_VOLUME_NAME
#elif defined(CONFIG_DISK_DRIVER_MMC)
#define TEST_DISK_NAME CONFIG_MMC_VOLUME_NAME
#else
#define TEST_DISK_NAME "SDMMC"
#endif

#define TEST_SECTOR_SIZE 512U
#define TEST_BLOCK_COUNT 128U
#define TEST_BYTES (TEST_SECTOR_SIZE * TEST_BLOCK_COUNT)
#define TEST_ITERATIONS 8U
#define RANDOM_ITERATIONS 128U
#define WBR_SDHC_RAW_PROBE 1

static uint8_t test_buf[TEST_BYTES] __aligned(32);
static uint8_t read_buf[TEST_BYTES] __aligned(32);
static uint8_t backup_buf[TEST_BYTES] __aligned(32);
static uint32_t random_sectors[RANDOM_ITERATIONS];

static uint32_t sector_count;
static uint32_t sector_size;
static uint32_t test_start_sector;

static void print_sdhc_info(void)
{
#if DT_NODE_HAS_STATUS(DT_ALIAS(sdhc0), okay)
	const struct device *sdhc = DEVICE_DT_GET(DT_ALIAS(sdhc0));
	struct sdhc_host_props props;
	int rc;

	printk("[INFO] sdhc0 ready=%d\n", device_is_ready(sdhc));
	if (!device_is_ready(sdhc)) {
		return;
	}

	rc = sdhc_card_present(sdhc);
	printk("[INFO] sdhc0 card_present=%d\n", rc);

	rc = sdhc_get_host_props(sdhc, &props);
	if (rc == 0) {
		printk("[INFO] sdhc0 f_min=%u f_max=%u 4bit=%u 1v8=%u 3v3=%u\n",
		       props.f_min, props.f_max,
		       props.host_caps.bus_4_bit_support,
		       props.host_caps.vol_180_support,
		       props.host_caps.vol_330_support);
	} else {
		printk("[WARN] sdhc_get_host_props rc=%d\n", rc);
	}
#else
	printk("[WARN] no okay sdhc0 alias in devicetree\n");
#endif
}

static uint32_t lcg_next(uint32_t *state)
{
	*state = (*state * 1664525U) + 1013904223U;
	return *state;
}

static void fill_pattern(uint32_t seed)
{
	uint32_t value = seed;

	for (uint32_t i = 0; i < sizeof(test_buf); i++) {
		test_buf[i] = (uint8_t)lcg_next(&value);
	}
}

static uint64_t elapsed_us(int64_t start_us)
{
	int64_t elapsed = k_uptime_get() * 1000LL - start_us;

	return (elapsed > 0) ? (uint64_t)elapsed : 1U;
}

static void print_rate(const char *name, uint32_t bytes, uint64_t elapsed_us_value)
{
	uint64_t bytes_per_s = ((uint64_t)bytes * 1000000ULL) / elapsed_us_value;
	uint64_t kib_per_s = bytes_per_s / 1024ULL;
	uint64_t mib_x100 = (bytes_per_s * 100ULL) / (1024ULL * 1024ULL);

	printk("%s: %u bytes in %llu us, %llu KiB/s, %llu.%02llu MiB/s\n",
	       name, bytes, elapsed_us_value, kib_per_s,
	       mib_x100 / 100ULL, mib_x100 % 100ULL);
}

static int raw_send_cmd(const struct device *sdhc, uint32_t opcode,
			uint32_t arg, uint32_t response_type)
{
	struct sdhc_command cmd = {
		.opcode = opcode,
		.arg = arg,
		.response_type = response_type,
		.timeout_ms = 100,
		.retries = 0,
	};
	int rc;

	printk("[RAW] send CMD%u arg=0x%08x resp=0x%x\n", opcode, arg, response_type);
	rc = sdhc_request(sdhc, &cmd, NULL);
	printk("[RAW] CMD%u rc=%d r0=0x%08x\n", opcode, rc, cmd.response[0]);
	return rc;
}

static int raw_sdhc_probe(void)
{
#if DT_NODE_HAS_STATUS(DT_ALIAS(sdhc0), okay)
	const struct device *sdhc = DEVICE_DT_GET(DT_ALIAS(sdhc0));
	struct sdhc_io io = {
		.clock = 0,
		.bus_width = SDHC_BUS_WIDTH1BIT,
		.timing = SDHC_TIMING_LEGACY,
		.signal_voltage = SD_VOL_3_3_V,
		.power_mode = SDHC_POWER_OFF,
		.bus_mode = SDHC_BUSMODE_PUSHPULL,
	};
	int rc;

	printk("sdhc_raw_probe booted\n");
	print_sdhc_info();

	printk("[RAW] power off\n");
	rc = sdhc_set_io(sdhc, &io);
	printk("[RAW] set_io power off rc=%d\n", rc);
	k_msleep(100);

	printk("[RAW] power on\n");
	io.power_mode = SDHC_POWER_ON;
	rc = sdhc_set_io(sdhc, &io);
	printk("[RAW] set_io power on rc=%d\n", rc);
	k_msleep(100);

	printk("[RAW] clock 400k\n");
	io.clock = 400000;
	rc = sdhc_set_io(sdhc, &io);
	printk("[RAW] set_io clock rc=%d\n", rc);
	k_msleep(500);

	for (uint32_t i = 0; i < 10U; i++) {
		printk("[RAW] scope window %u\n", i);
		(void)raw_send_cmd(sdhc, SD_GO_IDLE_STATE, 0, SD_RSP_TYPE_NONE);
		k_msleep(500);
		(void)raw_send_cmd(sdhc, SD_SEND_IF_COND, 0x000001aa, SD_RSP_TYPE_R7);
		k_msleep(500);
		(void)raw_send_cmd(sdhc, SD_APP_CMD, 0, SD_RSP_TYPE_R1);
		k_msleep(500);
	}

	printk("[RAW] done\n");
	return 0;
#else
	printk("[RAW] no okay sdhc0 alias\n");
	return -ENODEV;
#endif
}

static int init_disk(void)
{
	int rc;

	printk("sdhc_perf_test booted, disk=%s\n", TEST_DISK_NAME);
	print_sdhc_info();

	printk("[INFO] calling disk_access_init(%s)\n", TEST_DISK_NAME);
	rc = disk_access_init(TEST_DISK_NAME);
	printk("[INFO] disk_access_init rc=%d\n", rc);
	if (rc != 0) {
		printk("[FAIL] disk_access_init rc=%d\n", rc);
		return rc;
	}

	rc = disk_access_status(TEST_DISK_NAME);
	if (rc != DISK_STATUS_OK) {
		printk("[FAIL] disk status rc=%d\n", rc);
		return -EIO;
	}

	rc = disk_access_ioctl(TEST_DISK_NAME, DISK_IOCTL_GET_SECTOR_COUNT, &sector_count);
	if (rc != 0) {
		printk("[FAIL] get sector count rc=%d\n", rc);
		return rc;
	}

	rc = disk_access_ioctl(TEST_DISK_NAME, DISK_IOCTL_GET_SECTOR_SIZE, &sector_size);
	if (rc != 0) {
		printk("[FAIL] get sector size rc=%d\n", rc);
		return rc;
	}

	printk("[INFO] sectors=%u sector_size=%u capacity=%llu MiB\n",
	       sector_count, sector_size,
	       ((uint64_t)sector_count * sector_size) / (1024ULL * 1024ULL));

	if (sector_size != TEST_SECTOR_SIZE) {
		printk("[FAIL] unsupported sector size %u\n", sector_size);
		return -ENOTSUP;
	}

	if (sector_count <= (TEST_BLOCK_COUNT + 4096U)) {
		printk("[FAIL] card is too small for test window\n");
		return -ENOSPC;
	}

	test_start_sector = sector_count - TEST_BLOCK_COUNT - 2048U;
	printk("[INFO] test window: start_sector=%u blocks=%u bytes=%u\n",
	       test_start_sector, TEST_BLOCK_COUNT, TEST_BYTES);

	return 0;
}

static int sequential_read_test(void)
{
	uint64_t total_us = 0U;
	int rc = 0;

	for (uint32_t i = 0; i < TEST_ITERATIONS; i++) {
		int64_t start_us = k_uptime_get() * 1000LL;

		rc = disk_access_read(TEST_DISK_NAME, read_buf, test_start_sector, TEST_BLOCK_COUNT);
		total_us += elapsed_us(start_us);
		if (rc != 0) {
			printk("[FAIL] sequential read rc=%d\n", rc);
			return rc;
		}
	}

	print_rate("[READ seq avg]", TEST_BYTES, total_us / TEST_ITERATIONS);
	return 0;
}

static int sequential_write_test(void)
{
	uint64_t total_us = 0U;
	int rc;

	rc = disk_access_read(TEST_DISK_NAME, backup_buf, test_start_sector, TEST_BLOCK_COUNT);
	if (rc != 0) {
		printk("[FAIL] backup read rc=%d\n", rc);
		return rc;
	}

	fill_pattern(0x6750U);

	for (uint32_t i = 0; i < TEST_ITERATIONS; i++) {
		int64_t start_us = k_uptime_get() * 1000LL;

		rc = disk_access_write(TEST_DISK_NAME, test_buf, test_start_sector, TEST_BLOCK_COUNT);
		total_us += elapsed_us(start_us);
		if (rc != 0) {
			printk("[FAIL] sequential write rc=%d\n", rc);
			goto restore;
		}
	}

	rc = disk_access_read(TEST_DISK_NAME, read_buf, test_start_sector, TEST_BLOCK_COUNT);
	if (rc != 0) {
		printk("[FAIL] verify read rc=%d\n", rc);
		goto restore;
	}

	if (memcmp(test_buf, read_buf, sizeof(test_buf)) != 0) {
		printk("[FAIL] write verify mismatch\n");
		rc = -EIO;
		goto restore;
	}

	print_rate("[WRITE seq avg]", TEST_BYTES, total_us / TEST_ITERATIONS);

restore:
	(void)disk_access_write(TEST_DISK_NAME, backup_buf, test_start_sector, TEST_BLOCK_COUNT);
	return rc;
}

static int single_block_latency_test(void)
{
	uint64_t read_total_us = 0U;
	uint64_t write_total_us = 0U;
	int rc;

	rc = disk_access_read(TEST_DISK_NAME, backup_buf, test_start_sector, 1);
	if (rc != 0) {
		printk("[FAIL] single backup read rc=%d\n", rc);
		return rc;
	}

	fill_pattern(0x512U);

	for (uint32_t i = 0; i < TEST_ITERATIONS; i++) {
		int64_t start_us = k_uptime_get() * 1000LL;

		rc = disk_access_read(TEST_DISK_NAME, read_buf, test_start_sector, 1);
		read_total_us += elapsed_us(start_us);
		if (rc != 0) {
			printk("[FAIL] single read rc=%d\n", rc);
			goto restore;
		}

		start_us = k_uptime_get() * 1000LL;
		rc = disk_access_write(TEST_DISK_NAME, test_buf, test_start_sector, 1);
		write_total_us += elapsed_us(start_us);
		if (rc != 0) {
			printk("[FAIL] single write rc=%d\n", rc);
			goto restore;
		}
	}

	printk("[LAT read 512B avg]: %llu us\n", read_total_us / TEST_ITERATIONS);
	printk("[LAT write 512B avg]: %llu us\n", write_total_us / TEST_ITERATIONS);

restore:
	(void)disk_access_write(TEST_DISK_NAME, backup_buf, test_start_sector, 1);
	return rc;
}

static int random_read_test(void)
{
	uint32_t rng = 0x12345678U;
	int64_t start_us;
	uint64_t total_us;
	int rc = 0;

	for (uint32_t i = 0; i < RANDOM_ITERATIONS; i++) {
		random_sectors[i] = test_start_sector +
				    (lcg_next(&rng) % TEST_BLOCK_COUNT);
	}

	start_us = k_uptime_get() * 1000LL;
	for (uint32_t i = 0; i < RANDOM_ITERATIONS; i++) {
		rc = disk_access_read(TEST_DISK_NAME, read_buf, random_sectors[i], 1);
		if (rc != 0) {
			printk("[FAIL] random read rc=%d\n", rc);
			return rc;
		}
	}
	total_us = elapsed_us(start_us);

	printk("[READ random 512B]: %u ops in %llu us, %llu IOPS, avg %llu us/op\n",
	       RANDOM_ITERATIONS, total_us,
	       ((uint64_t)RANDOM_ITERATIONS * 1000000ULL) / total_us,
	       total_us / RANDOM_ITERATIONS);

	return 0;
}

int main(void)
{
#if WBR_SDHC_RAW_PROBE
	return raw_sdhc_probe();
#endif

	int rc = init_disk();

	if (rc != 0) {
		return rc;
	}

	printk("[WARN] write tests temporarily overwrite and restore the test window.\n");

	rc = sequential_read_test();
	if (rc == 0) {
		rc = sequential_write_test();
	}
	if (rc == 0) {
		rc = single_block_latency_test();
	}
	if (rc == 0) {
		rc = random_read_test();
	}

	printk("sdhc_perf_test %s\n", (rc == 0) ? "done" : "failed");
	return rc;
}
