/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file platform/board/board_identity.h
 * @ingroup wbr_platform
 * @brief 读取并输出目标板的硬件身份信息。
 * @details 这是业务层可见的平台边界。返回负 errno 表示参数、设备或传输失败；调用方不得绕过该接口直接依赖具体驱动实例。
 */

#ifndef WBR_CONTROL_PLATFORM_BOARD_IDENTITY_H_
#define WBR_CONTROL_PLATFORM_BOARD_IDENTITY_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 获取当前目标板的可读名称。
 * @return 指向静态板名字符串的只读指针。
 */
const char *board_identity_name(void);

#ifdef __cplusplus
}
#endif

#endif /* WBR_CONTROL_PLATFORM_BOARD_IDENTITY_H_ */
