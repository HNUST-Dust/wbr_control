/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/devicetree/fixed-partitions.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/storage/flash_map.h>

#include "littlefs_service.h"

LOG_MODULE_REGISTER(littlefs_service, LOG_LEVEL_INF);

namespace {

FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(rm_test_storage_lfs);

fs_mount_t g_lfs_mount = {};

bool g_mounted = false;

int EraseStoragePartition()
{
	const struct flash_area *area = nullptr;
	int rc = flash_area_open(FIXED_PARTITION_ID(storage_partition), &area);
	if (rc != 0) {
		return rc;
	}

	/* Prefer flatten to support devices that do not expose pure erase semantics. */
	rc = flash_area_flatten(area, 0, area->fa_size);
	if (rc != 0) {
		rc = flash_area_erase(area, 0, area->fa_size);
	}
	flash_area_close(area);
	return rc;
}

int ProbeStoragePartitionIo()
{
	const struct flash_area *area = nullptr;
	int rc = flash_area_open(FIXED_PARTITION_ID(storage_partition), &area);
	if (rc != 0) {
		return rc;
	}

	/* Keep probe small and aligned to typical flash program granularity. */
	uint8_t wr[16] = {
		0x5A, 0xA5, 0x3C, 0xC3,
		0x11, 0x22, 0x33, 0x44,
		0x55, 0x66, 0x77, 0x88,
		0x99, 0xAA, 0xBB, 0xCC,
	};
	uint8_t rd[sizeof(wr)] = {0};

	rc = flash_area_flatten(area, 0, sizeof(wr));
	if (rc != 0) {
		const int erase_rc = flash_area_erase(area, 0, sizeof(wr));
		rc = erase_rc;
	}
	if (rc != 0) {
		flash_area_close(area);
		return rc;
	}

	rc = flash_area_write(area, 0, wr, sizeof(wr));
	if (rc != 0) {
		flash_area_close(area);
		return rc;
	}

	rc = flash_area_read(area, 0, rd, sizeof(rd));
	if (rc != 0) {
		flash_area_close(area);
		return rc;
	}

	if (memcmp(wr, rd, sizeof(wr)) != 0) {
		flash_area_close(area);
		return -EIO;
	}

	/* Return partition head to erased state for mkfs. */
	rc = flash_area_flatten(area, 0, sizeof(wr));
	if (rc != 0) {
		const int erase_rc = flash_area_erase(area, 0, sizeof(wr));
		rc = erase_rc;
	}
	flash_area_close(area);
	return rc;
}

}  // namespace

namespace platform {

int InitializeLittlefs()
{
	if (g_mounted) {
		return 0;
	}

	g_lfs_mount.type = FS_LITTLEFS;
	g_lfs_mount.mnt_point = "/lfs";
	g_lfs_mount.fs_data = &rm_test_storage_lfs;
	g_lfs_mount.storage_dev = (void *)FIXED_PARTITION_ID(storage_partition);

	int rc = fs_mount(&g_lfs_mount);
	if ((rc != 0) && (rc != -EALREADY)) {
		LOG_INF("littlefs mount failed: %d, try mkfs", rc);

		int mkfs_rc = fs_mkfs(
			FS_LITTLEFS,
			static_cast<uintptr_t>(FIXED_PARTITION_ID(storage_partition)),
			&rm_test_storage_lfs,
			0);
		if (mkfs_rc != 0) {
			LOG_INF("littlefs mkfs failed: %d, erase partition and retry", mkfs_rc);

			const int erase_rc = EraseStoragePartition();
			if (erase_rc != 0) {
				LOG_INF("littlefs erase storage failed: %d", erase_rc);
				return erase_rc;
			}
			LOG_INF("littlefs erase storage ok");

			mkfs_rc = fs_mkfs(
				FS_LITTLEFS,
				static_cast<uintptr_t>(FIXED_PARTITION_ID(storage_partition)),
				&rm_test_storage_lfs,
				0);
			if (mkfs_rc != 0) {
				const int probe_rc = ProbeStoragePartitionIo();
				LOG_INF("littlefs mkfs retry failed: %d", mkfs_rc);
				return (probe_rc != 0) ? probe_rc : mkfs_rc;
			}
		}

		rc = fs_mount(&g_lfs_mount);
		if ((rc != 0) && (rc != -EALREADY)) {
			LOG_INF("littlefs remount after mkfs failed: %d", rc);
			return rc;
		}
	}

	g_mounted = true;
	return 0;
}

bool IsLittlefsReady()
{
	return g_mounted;
}

const char *LittlefsMountPoint()
{
	return g_lfs_mount.mnt_point;
}

}  // namespace platform
