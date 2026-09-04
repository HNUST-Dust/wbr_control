/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file channels/chassismotors_feedback_raw.cpp
 * @ingroup wbr_channels
 * @brief 定义底盘电机原始反馈数据通道。
 * @details 本文件仅定义对应通道的全局存储实例，不启动线程、不访问硬件，也不改变消息字段。
 */

#include <channels/chassismotors_feedback_raw.hpp>

SeqlockValue<ChassisMotorFeedbackRawFrame> left_wheel_feedback_raw;
SeqlockValue<ChassisMotorFeedbackRawFrame> right_wheel_feedback_raw;
SeqlockValue<ChassisMotorFeedbackRawFrame> left_b_motor_feedback_raw;
SeqlockValue<ChassisMotorFeedbackRawFrame> left_d_motor_feedback_raw;
SeqlockValue<ChassisMotorFeedbackRawFrame> right_b_motor_feedback_raw;
SeqlockValue<ChassisMotorFeedbackRawFrame> right_d_motor_feedback_raw;
