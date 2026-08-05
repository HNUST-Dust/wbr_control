/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace platform {

int InitializeUsbSession();
bool IsUsbConfigured();
int SendUsb(const uint8_t *data, size_t len);
int ReceiveUsb(uint8_t *out, size_t capacity, size_t *out_len,
	       int32_t timeout_ms);

}  // namespace platform
