/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file channels/chassismotors_send_raw.cpp
 * @ingroup wbr_channels
 * @brief 定义底盘电机原始控制指令通道。
 * @details 本文件仅定义对应通道的全局存储实例，不启动线程、不访问硬件，也不改变消息字段。
 */

#include <errno.h>

#include <channels/chassismotors_send_raw.hpp>

SeqlockValue<ChassisMotorSendRawFrame> left_wheel_send_raw;
SeqlockValue<ChassisMotorSendRawFrame> right_wheel_send_raw;
SeqlockValue<ChassisMotorSendRawFrame> left_b_motor_send_raw;
SeqlockValue<ChassisMotorSendRawFrame> left_d_motor_send_raw;
SeqlockValue<ChassisMotorSendRawFrame> right_b_motor_send_raw;
SeqlockValue<ChassisMotorSendRawFrame> right_d_motor_send_raw;
