/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file platform/board/board_identity.c
 * @ingroup wbr_platform
 * @brief 读取并输出目标板的硬件身份信息。
 * @details 实现封装 Zephyr 设备 API、硬件初始化和异步回调。共享状态在中断、回调与线程之间访问时使用原子量、内核队列或短临界区保护。
 */

#include <platform/board/board_identity.h>

const char *board_identity_name(void)
{
	return CONFIG_BOARD_TARGET;
}
