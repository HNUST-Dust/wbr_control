/* SPDX-License-Identifier: Apache-2.0 */

#include "sdlog_module.h"

#include <cerrno>
#include <cstdio>
#include <cstdint>
#include <cstring>

#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/disk_access.h>

#ifndef FS_FATFS_WINDOW_ALIGNMENT
#define FS_FATFS_WINDOW_ALIGNMENT CONFIG_FS_FATFS_WINDOW_ALIGNMENT
#endif
#include <ff.h>
#include <msg/chassis_realtime_status.hpp>
#include <msg/hi91_imu_sample.hpp>
#include <msg/onboard_imu_sample.hpp>
#include <msg/oscilloscope_sample.hpp>
#include <scheduling/thread_priorities.h>
#include <sdlog_params_generated.h>

LOG_MODULE_REGISTER(sdlog_module, LOG_LEVEL_INF);

namespace {

K_THREAD_STACK_DEFINE(g_sdlog_stack, 4096);
constexpr char kDiskName[] = CONFIG_SDMMC_VOLUME_NAME;
constexpr char kMountPoint[] = "/SD:";
constexpr char kLogPathFormat[] = "/SD:/PX4LOG%02u.BIN";
constexpr uint32_t kMaximumLogFiles = modules::sdlog_params::kSdlogMaximumLogFiles;
constexpr uint32_t kPeriodMs = modules::sdlog_params::kSdlogPeriodMs;
constexpr size_t kBatchCapacity = 8U * 1024U;

struct SdLogRecord {
	uint32_t magic;
	uint16_t version;
	uint16_t size;
	uint64_t timestamp_us;
	uint32_t sequence;
	msg::OnboardImuSample onboard_imu;
	msg::Hi91ImuSample hi91_imu;
	msg::ChassisRealtimeStatus chassis;
};

static_assert(sizeof(SdLogRecord) < kBatchCapacity, "log record must fit batch");
static uint8_t g_write_batch[kBatchCapacity] __aligned(32);

FATFS g_fat_fs;
struct fs_mount_t g_mount = {
	.node = {},
	.type = FS_FATFS,
	.mnt_point = kMountPoint,
	.fs_data = &g_fat_fs,
};

} // namespace

namespace modules {

int SdLogModule::Start()
{
	if (started_) {
		return 0;
	}
	return CreateThread(g_sdlog_stack, K_THREAD_STACK_SIZEOF(g_sdlog_stack),
		K_PRIO_PREEMPT(wbr_control::scheduling::thread_priority::kSdLog), "sdlog");
}

void SdLogModule::RunLoop()
{
	/* A missing/unformatted card is a logging fault, never a boot fault. */
	int rc = disk_access_init(kDiskName);
	if (rc != 0) {
		LOG_WRN("SD disk init failed: %d; logging disabled", rc);
		return;
	}

	rc = fs_mount(&g_mount);
	if (rc != 0) {
		LOG_WRN("FAT mount %s failed: %d; logging disabled", kMountPoint, rc);
		return;
	}

	struct fs_file_t file;
	char log_path[sizeof("/SD:/PX4LOG99.BIN")] = {};
	struct fs_dirent entry;
	fs_file_t_init(&file);
	for (uint32_t index = 0U; index < kMaximumLogFiles; ++index) {
		(void)snprintf(log_path, sizeof(log_path), kLogPathFormat, (unsigned)index);
		rc = fs_stat(log_path, &entry);
		if (rc == -ENOENT) {
			break;
		}
		if (rc != 0) {
			LOG_WRN("stat %s failed: %d; logging disabled", log_path, rc);
			(void)fs_unmount(&g_mount);
			return;
		}
	}
	if (rc != -ENOENT) {
		LOG_WRN("no free numbered SD log filename; logging disabled");
		(void)fs_unmount(&g_mount);
		return;
	}

	rc = fs_open(&file, log_path, FS_O_CREATE | FS_O_WRITE);
	if (rc != 0) {
		LOG_WRN("open %s failed: %d; logging disabled", log_path, rc);
		(void)fs_unmount(&g_mount);
		return;
	}

	LOG_INF("SD logging started: %s period=%u ms batch=%u bytes", log_path,
		(unsigned)kPeriodMs, (unsigned)kBatchCapacity);

	size_t batch_size = 0U;
	uint32_t sequence = 0U;
	uint32_t sync_counter = 0U;
	uint32_t batch_count = 0U;
	uint64_t total_written = 0U;
	for (;;) {
		k_sleep(K_MSEC(kPeriodMs));

		SdLogRecord record = {};
		msg::OnboardImuSample onboard_imu = {};
		msg::Hi91ImuSample hi91_imu = {};
		msg::ChassisRealtimeStatus chassis = {};
		record.magic = 0x5742524CUL; /* WBRL */
		record.version = 1U;
		record.size = sizeof(record);
		record.timestamp_us = k_cyc_to_us_floor64(k_cycle_get_64());
		record.sequence = ++sequence;
		(void)msg::latest_onboard_imu_sample.read(onboard_imu);
		(void)msg::latest_hi91_imu_sample.read(hi91_imu);
		(void)msg::latest_chassis_realtime_status.read(chassis);
		record.onboard_imu = onboard_imu;
		record.hi91_imu = hi91_imu;
		record.chassis = chassis;

		if (batch_size + sizeof(record) > sizeof(g_write_batch)) {
			const ssize_t written = fs_write(&file, g_write_batch, batch_size);
			if (written != (ssize_t)batch_size) {
				LOG_ERR("SD batch write failed: rc=%d batch=%u total=%llu bytes=%u",
					(int)written, (unsigned)batch_count,
					(unsigned long long)total_written, (unsigned)batch_size);
				break;
			}
			total_written += batch_size;
			++batch_count;
			batch_size = 0U;
		}
		memcpy(&g_write_batch[batch_size], &record, sizeof(record));
		batch_size += sizeof(record);

		if (batch_size + sizeof(SdLogRecord) > sizeof(g_write_batch)) {
			const ssize_t written = fs_write(&file, g_write_batch, batch_size);
			if (written != (ssize_t)batch_size) {
				LOG_ERR("SD batch write failed: rc=%d batch=%u total=%llu bytes=%u",
					(int)written, (unsigned)batch_count,
					(unsigned long long)total_written, (unsigned)batch_size);
				break;
			}
			total_written += batch_size;
			++batch_count;
			batch_size = 0U;
			if (++sync_counter >= 50U) {
				rc = fs_sync(&file);
				if (rc != 0) {
					LOG_ERR("SD sync failed: rc=%d batch=%u total=%llu",
						rc, (unsigned)batch_count,
						(unsigned long long)total_written);
					break;
				}
				LOG_INF("SD sync ok: batch=%u total=%llu", (unsigned)batch_count,
					(unsigned long long)total_written);
				sync_counter = 0U;
			}
		}
	}

	if (batch_size != 0U) {
		(void)fs_write(&file, g_write_batch, batch_size);
	}
	(void)fs_sync(&file);
	(void)fs_close(&file);
	(void)fs_unmount(&g_mount);
}

} // namespace modules
