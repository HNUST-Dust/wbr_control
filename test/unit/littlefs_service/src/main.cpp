/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>

#include <zephyr/fff.h>
#include <zephyr/fs/fs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/ztest.h>

#include <platform/storage/filesystem/littlefs_service.h>

DEFINE_FFF_GLOBALS;

FAKE_VALUE_FUNC(int, fs_mount, struct fs_mount_t *);
FAKE_VALUE_FUNC(int, fs_mkfs, int, uintptr_t, void *, int);
FAKE_VALUE_FUNC(int, flash_area_open, uint8_t, const struct flash_area **);
FAKE_VOID_FUNC(flash_area_close, const struct flash_area *);
FAKE_VALUE_FUNC(int, flash_area_flatten, const struct flash_area *, off_t, size_t);
FAKE_VALUE_FUNC(int, flash_area_erase, const struct flash_area *, off_t, size_t);
FAKE_VALUE_FUNC(int, flash_area_write, const struct flash_area *, off_t, const void *, size_t);
FAKE_VALUE_FUNC(int, flash_area_read, const struct flash_area *, off_t, void *, size_t);

ZTEST(littlefs_service, formats_and_remounts_after_initial_mount_failure)
{
	int mount_results[] = {-ENODEV, 0};
	SET_RETURN_SEQ(fs_mount, mount_results, ARRAY_SIZE(mount_results));
	fs_mkfs_fake.return_val = 0;

	zassert_ok(platform::InitializeLittlefs());
	zassert_true(platform::IsLittlefsReady());
	zassert_equal(fs_mount_fake.call_count, 2U);
	zassert_equal(fs_mkfs_fake.call_count, 1U);
	zassert_equal(fs_mkfs_fake.arg0_val, FS_LITTLEFS);
	zassert_equal(fs_mkfs_fake.arg1_val, 7U);
	zassert_equal(fff.call_history_idx, 3U);
	zassert_equal_ptr(fff.call_history[0], fs_mount);
	zassert_equal_ptr(fff.call_history[1], fs_mkfs);
	zassert_equal_ptr(fff.call_history[2], fs_mount);

	/* Once mounted, subsequent callers must not touch the filesystem again. */
	zassert_ok(platform::InitializeLittlefs());
	zassert_equal(fs_mount_fake.call_count, 2U);
	zassert_equal(fs_mkfs_fake.call_count, 1U);
	zassert_str_equal(platform::LittlefsMountPoint(), "/lfs");
}

ZTEST_SUITE(littlefs_service, nullptr, nullptr, nullptr, nullptr, nullptr);
