/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file channels/serialservo_send_raw.hpp
 * @ingroup wbr_channels
 * @brief 定义串行舵机原始发送数据通道。
 * @details 写入方和读取方通过有界 spinlock 临界区复制完整快照，读取始终成功；调用方不得保存内部存储地址。
 */

#pragma once
#include <cstdint>

#include <channels/comm/seqlock_value.hpp>

/** @brief 串行舵机的一帧原始发送数据。 */
struct SerialServoSendRawFrame {
	uint8_t valid; ///< 数据有效标志；为 `false` 时其余字段不得用于控制。
	uint8_t len; ///< 缓冲区内的有效数据长度，单位为字节。
	uint8_t data[16]; ///< 协议或总线载荷的原始字节。
};

/** @brief 偏航串行舵机下一待发送原始指令。 */
extern SeqlockValue<SerialServoSendRawFrame> yaw_servo_send_raw;
/** @brief 俯仰串行舵机下一待发送原始指令。 */
extern SeqlockValue<SerialServoSendRawFrame> pitch_servo_send_raw;
