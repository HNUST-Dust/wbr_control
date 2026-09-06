/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file channels/chassismotors_send_raw.hpp
 * @ingroup wbr_channels
 * @brief 定义底盘电机原始控制指令通道。
 * @details 写入方和读取方通过有界 spinlock 临界区复制完整快照，读取始终成功；调用方不得保存内部存储地址。
 */

#pragma once
#include <cstdint>
#include <channels/comm/seqlock_value.hpp>

/** @brief 单个底盘电机的待发送原始 CAN 帧。 */
struct ChassisMotorSendRawFrame {
	uint8_t bus; ///< CAN 总线编号，从 0 开始。
	uint16_t can_id; ///< 11 位标准 CAN 标识符。
	uint8_t dlc; ///< CAN 数据长度码。
	uint8_t data[8]; ///< 协议或总线载荷的原始字节。
};

/** @brief 左轮电机下一待发送原始指令。 */
extern SeqlockValue<ChassisMotorSendRawFrame> left_wheel_send_raw;
/** @brief 右轮电机下一待发送原始指令。 */
extern SeqlockValue<ChassisMotorSendRawFrame> right_wheel_send_raw;
/** @brief 左腿 B 关节电机下一待发送原始指令。 */
extern SeqlockValue<ChassisMotorSendRawFrame> left_b_motor_send_raw;
/** @brief 左腿 D 关节电机下一待发送原始指令。 */
extern SeqlockValue<ChassisMotorSendRawFrame> left_d_motor_send_raw;
/** @brief 右腿 B 关节电机下一待发送原始指令。 */
extern SeqlockValue<ChassisMotorSendRawFrame> right_b_motor_send_raw;
/** @brief 右腿 D 关节电机下一待发送原始指令。 */
extern SeqlockValue<ChassisMotorSendRawFrame> right_d_motor_send_raw;
