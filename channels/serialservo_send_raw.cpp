/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file channels/serialservo_send_raw.cpp
 * @ingroup wbr_channels
 * @brief 定义串行舵机原始发送数据通道。
 * @details 本文件仅定义对应通道的全局存储实例，不启动线程、不访问硬件，也不改变消息字段。
 */

#include <channels/serialservo_send_raw.hpp>

SeqlockValue<SerialServoSendRawFrame> yaw_servo_send_raw;
SeqlockValue<SerialServoSendRawFrame> pitch_servo_send_raw;
