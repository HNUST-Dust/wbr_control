/* SPDX-License-Identifier: Apache-2.0 */

/**
* @file channels/usb_raw_frame_queue.h
 * @ingroup wbr_channels
 * @brief 提供 USB 原始帧的线程安全消息队列。
 * @details 声明跨线程交换的数据快照及其唯一全局通道。写入方发布完整对象，读取方不得保存内部存储地址；使用顺序锁的通道允许读取失败，调用方应保留上一份有效快照。
 */

#ifndef WBR_CONTROL_CHANNELS_USB_RAW_FRAME_QUEUE_H_
#define WBR_CONTROL_CHANNELS_USB_RAW_FRAME_QUEUE_H_

#include <stddef.h>
#include <stdint.h>

namespace channels {

constexpr size_t kUsbRawChunkSize = 64U; ///< USB 原始消息单次可携带的最大字节数。

/** @brief USB 会话收发的原始数据帧。 */
struct UsbRawFrameMessage {
	uint16_t len; ///< 缓冲区内的有效数据长度，单位为字节。
	uint8_t data[kUsbRawChunkSize]; ///< 协议或总线载荷的原始字节。
};

/**
 * @brief 将一帧 USB 原始数据压入接收队列。
 * @param[in] frame 待入队、解码或处理的数据帧；不得为空。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int EnqueueUsbRawFrame(const UsbRawFrameMessage *frame);
/**
 * @brief 在指定超时时间内从 USB 接收队列取出一帧数据。
 * @param[in,out] frame 待入队、解码或处理的数据帧；不得为空。
 * @param timeout_ms 等待超时时间，单位为毫秒；负值表示永久等待。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int DequeueUsbRawFrame(UsbRawFrameMessage *frame, int32_t timeout_ms);

}  // namespace channels

#endif /* WBR_CONTROL_CHANNELS_USB_RAW_FRAME_QUEUE_H_ */
