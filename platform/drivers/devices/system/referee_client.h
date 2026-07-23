/* SPDX-License-Identifier: Apache-2.0 */

#ifndef RM_TEST_PLATFORM_DRIVERS_DEVICES_SYSTEM_REFEREE_CLIENT_H_
#define RM_TEST_PLATFORM_DRIVERS_DEVICES_SYSTEM_REFEREE_CLIENT_H_

#include <stddef.h>
#include <stdint.h>


namespace platform {

int InitializeRefereeClient();
int FeedRefereeBytes(const uint8_t *data, size_t len);

}  // namespace platform

#endif /* RM_TEST_PLATFORM_DRIVERS_DEVICES_SYSTEM_REFEREE_CLIENT_H_ */
