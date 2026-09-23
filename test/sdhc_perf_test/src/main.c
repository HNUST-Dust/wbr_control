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
#include <zephyr/fs/fs.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <ff.h>

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
#define TEST_MOUNT_POINT "/SD:"
#define TEST_LOG_PATH TEST_MOUNT_POINT "/SDHC_PROBE.BIN"

static uint8_t test_buf[TEST_BYTES] __aligned(32);
static uint8_t read_buf[TEST_BYTES] __aligned(32);
static uint8_t backup_buf[TEST_BYTES] __aligned(32);
static uint32_t random_sectors[RANDOM_ITERATIONS];

static uint32_t sector_count;
static uint32_t sector_size;
static uint32_t test_start_sector;
static FATFS fat_fs;
static struct fs_mount_t mount = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
	.mnt_point = TEST_MOUNT_POINT,
};

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

static int filesystem_test(void)
{
	static const char log_text[] =
		"PX4-style TF log test\n"
		"hpm6750 sdc0 filesystem write/read verified\n";
	char readback[sizeof(log_text)];
	struct fs_file_t file;
	ssize_t n;
	int rc;

	rc = fs_mount(&mount);
	if (rc != 0) {
		printk("[FAIL] FAT mount %s rc=%d\n", TEST_MOUNT_POINT, rc);
		return rc;
	}
	printk("[PASS] FAT mounted at %s\n", TEST_MOUNT_POINT);

	fs_file_t_init(&file);
	rc = fs_open(&file, TEST_LOG_PATH, FS_O_CREATE | FS_O_RDWR);
	if (rc != 0) {
		printk("[FAIL] open %s rc=%d\n", TEST_LOG_PATH, rc);
		goto out_unmount;
	}

	rc = fs_truncate(&file, 0);
	if (rc == 0) {
		n = fs_write(&file, log_text, sizeof(log_text));
		if (n != (ssize_t)sizeof(log_text)) {
			printk("[FAIL] log write bytes=%d expected=%u\n", (int)n,
			       (unsigned int)sizeof(log_text));
			rc = (n < 0) ? (int)n : -EIO;
		}
	}
	if (rc == 0) {
		rc = fs_sync(&file);
		if (rc != 0) {
			printk("[FAIL] log sync rc=%d\n", rc);
		}
	}
	if (rc == 0) {
		rc = fs_seek(&file, 0, FS_SEEK_SET);
	}
	if (rc == 0) {
		n = fs_read(&file, readback, sizeof(readback));
		if (n != (ssize_t)sizeof(log_text) ||
		    memcmp(readback, log_text, sizeof(log_text)) != 0) {
			printk("[FAIL] log readback mismatch bytes=%d\n", (int)n);
			rc = -EIO;
		}
	}
	fs_close(&file);
	if (rc == 0) {
		printk("[PASS] file write/read verified: %s\n", TEST_LOG_PATH);
		(void)fs_unlink(TEST_LOG_PATH);
		printk("[PASS] FAT remains mounted for RTT fs shell\n");
		return 0;
	}

out_unmount:
	if (fs_unmount(&mount) != 0 && rc == 0) {
		rc = -EIO;
	}
	return rc;
}

static void dump_boot_sector(void)
{
	uint8_t *sector = read_buf;
	uint32_t partition_lba;
	int rc = disk_access_read(TEST_DISK_NAME, sector, 0, 1);

	if (rc != 0) {
		printk("[INFO] boot-sector read rc=%d\n", rc);
		return;
	}

	printk("[INFO] sector0: jump=%02x %02x %02x oem=%c%c%c%c%c%c%c%c\n",
	       sector[0], sector[1], sector[2], sector[3], sector[4], sector[5],
	       sector[6], sector[7], sector[8], sector[9], sector[10]);
	printk("[INFO] sector0: sig=%02x%02x part0=%02x type=%02x lba=%02x%02x%02x%02x count=%02x%02x%02x%02x\n",
	       sector[511], sector[510], sector[446], sector[446 + 4],
	       sector[446 + 8], sector[446 + 9], sector[446 + 10], sector[446 + 11],
	       sector[446 + 12], sector[446 + 13], sector[446 + 14], sector[446 + 15]);

	partition_lba = (uint32_t)sector[446 + 8] |
		((uint32_t)sector[446 + 9] << 8) |
		((uint32_t)sector[446 + 10] << 16) |
		((uint32_t)sector[446 + 11] << 24);
	if (partition_lba == 0U || partition_lba >= sector_count) {
		return;
	}

	rc = disk_access_read(TEST_DISK_NAME, sector, partition_lba, 1);
	if (rc != 0) {
		printk("[INFO] partition boot-sector read lba=%u rc=%d\n",
		       partition_lba, rc);
		return;
	}
	printk("[INFO] partition boot lba=%u: jump=%02x %02x %02x oem=%c%c%c%c%c%c%c%c fs=%c%c%c%c%c%c%c%c\n",
	       partition_lba, sector[0], sector[1], sector[2],
	       sector[3], sector[4], sector[5], sector[6], sector[7],
	       sector[8], sector[9], sector[10], sector[82], sector[83],
	       sector[84], sector[85], sector[86], sector[87], sector[88], sector[89]);
	printk("[INFO] FAT BPB: bytes=%u sec/clus=%u reserved=%u fats=%u fat_sz=%u root=%u sig=%02x%02x\n",
	       (uint16_t)sector[11] | ((uint16_t)sector[12] << 8), sector[13],
	       (uint16_t)sector[14] | ((uint16_t)sector[15] << 8), sector[16],
	       (uint32_t)sector[36] | ((uint32_t)sector[37] << 8) |
	       ((uint32_t)sector[38] << 16) | ((uint32_t)sector[39] << 24),
	       (uint32_t)sector[44] | ((uint32_t)sector[45] << 8) |
	       ((uint32_t)sector[46] << 16) | ((uint32_t)sector[47] << 24),
	       sector[511], sector[510]);
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

static int write_size_sweep(void)
{
	static const uint32_t block_counts[] = {1U, 8U, 16U, 64U};
	int rc;

	rc = disk_access_read(TEST_DISK_NAME, backup_buf, test_start_sector, TEST_BLOCK_COUNT);
	if (rc != 0) {
		printk("[FAIL] backup read rc=%d\n", rc);
		return rc;
	}

	for (uint32_t size_index = 0; size_index < ARRAY_SIZE(block_counts); size_index++) {
		const uint32_t blocks = block_counts[size_index];
		const uint32_t bytes = blocks * TEST_SECTOR_SIZE;
		int64_t start_us;
		uint64_t write_us;

		fill_pattern(0x6750U + blocks);
		start_us = k_uptime_get() * 1000LL;
		rc = disk_access_write(TEST_DISK_NAME, test_buf, test_start_sector, blocks);
		write_us = elapsed_us(start_us);
		if (rc != 0) {
			printk("[FAIL] write-size bytes=%u blocks=%u rc=%d\n",
			       bytes, blocks, rc);
			goto restore;
		}
		rc = disk_access_read(TEST_DISK_NAME, read_buf, test_start_sector, blocks);
		if (rc != 0 || memcmp(test_buf, read_buf, bytes) != 0) {
			printk("[FAIL] write-size verify bytes=%u read_rc=%d\n", bytes, rc);
			rc = -EIO;
			goto restore;
		}
		print_rate("[PASS write-size]", bytes, write_us);
		rc = disk_access_write(TEST_DISK_NAME, backup_buf, test_start_sector, blocks);
		if (rc != 0) {
			printk("[FAIL] restore bytes=%u rc=%d\n", bytes, rc);
			return rc;
		}
	}
	return 0;

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
	int rc = init_disk();

	if (rc != 0) {
		return rc;
	}

	rc = filesystem_test();
	if (rc != 0) {
		dump_boot_sector();
		printk("sdhc filesystem test failed\n");
		return rc;
	}

	printk("[WARN] write tests temporarily overwrite and restore the test window.\n");

	rc = sequential_read_test();
	if (rc == 0) {
		rc = write_size_sweep();
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
