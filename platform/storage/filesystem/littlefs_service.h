/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef WBR_CONTROL_PLATFORM_LITTLEFS_SERVICE_H_
#define WBR_CONTROL_PLATFORM_LITTLEFS_SERVICE_H_

namespace platform {

int InitializeLittlefs();
bool IsLittlefsReady();
const char *LittlefsMountPoint();

}  // namespace platform

#endif /* WBR_CONTROL_PLATFORM_LITTLEFS_SERVICE_H_ */
