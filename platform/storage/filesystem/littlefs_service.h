/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RM_TEST_PLATFORM_STORAGE_FILESYSTEM_LITTLEFS_SERVICE_H_
#define RM_TEST_PLATFORM_STORAGE_FILESYSTEM_LITTLEFS_SERVICE_H_

namespace platform {

int InitializeLittlefs();
bool IsLittlefsReady();
const char *LittlefsMountPoint();

}  // namespace platform

#endif /* RM_TEST_PLATFORM_STORAGE_FILESYSTEM_LITTLEFS_SERVICE_H_ */
