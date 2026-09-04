/* SPDX-License-Identifier: Apache-2.0 */

/**
* @file src/protocols/remote_input/dr16_protocol.h
 * @ingroup wbr_protocols
 * @brief 实现 DR16 遥控器协议解码。
 * @details 接口直接处理线协议字节序列。调用方负责提供声明长度的有效缓冲区；解码失败时输出对象内容不应作为有效数据使用。
 */

#ifndef WBR_CONTROL_PROTOCOLS_REMOTE_INPUT_DR16_PROTOCOL_H_
#define WBR_CONTROL_PROTOCOLS_REMOTE_INPUT_DR16_PROTOCOL_H_

#include <stddef.h>
#include <stdint.h>

namespace protocols {

constexpr size_t kDr16FrameLength = 18; ///< DR16 协议固定帧长度，单位为字节。

/** @brief DR16 遥控器帧的归一化解码结果。 */
struct Dr16Frame {
	float right_stick_x; ///< 右摇杆横向归一化值。
	float right_stick_y; ///< 右摇杆纵向归一化值。
	float left_stick_x; ///< 左摇杆横向归一化值。
	float left_stick_y; ///< 左摇杆纵向归一化值。
	float wheel; ///< 左右车轮状态数组，索引 0 为左侧、1 为右侧。
	uint8_t left_switch; ///< 遥控器左侧开关状态。
	uint8_t right_switch; ///< 遥控器右侧开关状态。
	bool chassis_enable; ///< 底盘闭环和执行器输出使能标志。
};

/**
 * @brief 解码一帧 DR16 遥控数据。
 * @param[in] data 输入字节缓冲区；长度由相邻长度参数给出。
 * @param len 输入缓冲区中的有效字节数。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 操作成功或条件成立时返回 `true`，否则返回 `false`。
 */
bool DecodeDr16Frame(const uint8_t *data, size_t len, Dr16Frame *out);

}  // namespace protocols

#endif /* WBR_CONTROL_PROTOCOLS_REMOTE_INPUT_DR16_PROTOCOL_H_ */
