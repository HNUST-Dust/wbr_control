/* SPDX-License-Identifier: Apache-2.0 */

#ifndef WBR_CONTROL_PLATFORM_REFEREE_CLIENT_H_
#define WBR_CONTROL_PLATFORM_REFEREE_CLIENT_H_

#include <stddef.h>
#include <stdint.h>

namespace platform {

int InitializeRefereeClient();
int FeedRefereeBytes(const uint8_t *data, size_t len);

}  // namespace platform

#endif /* WBR_CONTROL_PLATFORM_REFEREE_CLIENT_H_ */
