/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file platform/drivers/communication/usb_session.h
 * @ingroup wbr_platform
 * @brief 管理 USB 通信会话及其收发状态。
 * @details 这是业务层可见的平台边界。返回负 errno 表示参数、设备或传输失败；调用方不得绕过该接口直接依赖具体驱动实例。
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace platform {

/**
 * @brief 初始化 USB 设备会话。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int InitializeUsbSession();
/**
 * @brief 判断 USB 主机是否已完成设备配置。
 * @return USB 已枚举并配置时返回 `true`。
 */
bool IsUsbConfigured();
/**
 * @brief 通过当前 USB 会话发送字节序列。
 * @param[in] data 输入字节缓冲区；长度由相邻长度参数给出。
 * @param len 输入缓冲区中的有效字节数。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int SendUsb(const uint8_t *data, size_t len);
/**
 * @brief 在指定超时时间内接收一帧 USB 数据。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @param capacity 输出缓冲区可容纳的最大字节数。
 * @param[out] out_len 接收实际输出字节数的指针；不得为空。
 * @param timeout_ms 等待超时时间，单位为毫秒；负值表示永久等待。
 * @return 成功返回 0；超时、参数无效或设备异常时返回负 errno 错误码。
 */
int ReceiveUsb(uint8_t *out, size_t capacity, size_t *out_len,
	       int32_t timeout_ms);

}  // namespace platform
