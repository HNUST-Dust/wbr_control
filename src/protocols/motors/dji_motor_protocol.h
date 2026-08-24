/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file src/protocols/motors/dji_motor_protocol.h
 * @ingroup wbr_protocols
 * @brief 实现大疆电机 CAN 协议编解码。
 * @details 接口直接处理线协议字节序列。调用方负责提供声明长度的有效缓冲区；解码失败时输出对象内容不应作为有效数据使用。
 */

#pragma once

// Public DJI motor wire-format contract.

#include <stdint.h>

namespace protocols {

/** @brief 大疆电机标准反馈帧的解码结果。 */
struct DjiMotorFeedback {
	uint16_t encoder; ///< 电机编码器原始计数值。
	int16_t omega; ///< 电机协议反馈的原始角速度。
	int16_t current; ///< 电机协议反馈或控制的原始电流值。
	uint8_t temperature; ///< 电机内部温度，单位为摄氏度。
};

/**
 * @brief 判断 CAN 标识符是否为大疆电机标准反馈 ID。
 * @param can_id 11 位标准 CAN 标识符。
 * @return 操作成功或条件成立时返回 `true`，否则返回 `false`。
 */
bool IsDjiStandardFeedbackId(uint16_t can_id);
/**
 * @brief 解码大疆电机标准反馈帧。
 * @param[in] data 输入字节缓冲区；长度由相邻长度参数给出。
 * @param dlc CAN 数据长度码，本接口要求不超过 8。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int DecodeDjiFeedback(const uint8_t *data, uint8_t dlc, DjiMotorFeedback *out);

// Pack 4 motor currents for CAN ID 0x200 (0x201~0x204).
/**
 * @brief 编码 CAN ID 0x200 的四路电流指令。
 * @param[in] current_cmd 目标电流指令，单位由对应电机协议规定。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int EncodeDjiCurrentFrame0x200(const int16_t current_cmd[4], uint8_t out[8]);
// Pack 4 motor currents for CAN ID 0x1FF.
/**
 * @brief 编码 CAN ID 0x1FF 的四路电流指令。
 * @param[in] current_cmd 目标电流指令，单位由对应电机协议规定。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int EncodeDjiCurrentFrame0x1ff(const int16_t current_cmd[4], uint8_t out[8]);

// Write one motor current command into a 0x200/0x1FF payload slot.
/**
 * @brief 将单个大疆电机电流指令写入对应载荷槽位。
 * @param motor_can_id 目标电机的标准 CAN 标识符。
 * @param current_cmd 目标电流指令，单位由对应电机协议规定。
 * @param frame_payload 待写入的 8 字节 CAN 载荷。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int WriteDjiCurrentCommandToSlot(uint16_t motor_can_id, int16_t current_cmd,
				 uint8_t frame_payload[8]);

}  // namespace protocols
