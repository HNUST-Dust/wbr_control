/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <platform/board/board_identity.h>

const char *board_identity_name(void)
{
	return CONFIG_BOARD_TARGET;
}
