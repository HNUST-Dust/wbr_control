/* SPDX-License-Identifier: Apache-2.0 */

/**
* @file src/protocols/remote_input/vt03_protocol.h
 * @ingroup wbr_protocols
 * @brief 实现 VT03 遥控器协议解码。
 * @details 接口直接处理线协议字节序列。调用方负责提供声明长度的有效缓冲区；解码失败时输出对象内容不应作为有效数据使用。
 */

#ifndef WBR_CONTROL_PROTOCOLS_REMOTE_INPUT_VT03_PROTOCOL_H_
#define WBR_CONTROL_PROTOCOLS_REMOTE_INPUT_VT03_PROTOCOL_H_

#include <stddef.h>
#include <stdint.h>

namespace protocols {

constexpr size_t kVt03RemoteFrameLength = 21; ///< VT03 基础遥控帧固定长度，单位为字节。
constexpr size_t kVt03CustomFrameLength = 39; ///< VT03 自定义帧固定长度，单位为字节。

/** @brief VT03 遥控器基础帧的解码结果。 */
struct Vt03Frame {
	float right_x; ///< 右侧二维输入的 X 分量。
	float right_y; ///< 右侧二维输入的 Y 分量。
	float left_x; ///< 左侧二维输入的 X 分量。
	float left_y; ///< 左侧二维输入的 Y 分量。
	float wheel; ///< 左右车轮状态数组，索引 0 为左侧、1 为右侧。
	uint8_t mode_switch; ///< 遥控器模式选择开关状态。
	bool chassis_enable; ///< 底盘闭环和执行器输出使能标志。
};

/** @brief VT03 自定义控制帧的解码结果。 */
struct Vt03CustomFrame {
	float joystick_x; ///< 自定义控制器 X 轴输入值。
	float joystick_y; ///< 自定义控制器 Y 轴输入值。
	float joystick_z; ///< 自定义控制器 Z 轴输入值。
	bool chassis_enable; ///< 底盘闭环和执行器输出使能标志。
};

/**
 * @brief 解码 VT03 基础遥控帧。
 * @param[in] data 输入字节缓冲区；长度由相邻长度参数给出。
 * @param len 输入缓冲区中的有效字节数。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 操作成功或条件成立时返回 `true`，否则返回 `false`。
 */
bool DecodeVt03RemoteFrame(const uint8_t *data, size_t len, Vt03Frame *out);
/**
 * @brief 解码 VT03 自定义控制帧。
 * @param[in] data 输入字节缓冲区；长度由相邻长度参数给出。
 * @param len 输入缓冲区中的有效字节数。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 操作成功或条件成立时返回 `true`，否则返回 `false`。
 */
bool DecodeVt03CustomFrame(const uint8_t *data, size_t len, Vt03CustomFrame *out);

}  // namespace protocols

#endif /* WBR_CONTROL_PROTOCOLS_REMOTE_INPUT_VT03_PROTOCOL_H_ */
