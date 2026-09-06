/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file channels/chassismotors_feedback_raw.hpp
 * @ingroup wbr_channels
 * @brief 定义底盘电机原始反馈数据通道。
 * @details 写入方和读取方通过有界 spinlock 临界区复制完整快照，读取始终成功；调用方不得保存内部存储地址。
 */

#pragma once
#include <cstdint>
#include <channels/comm/seqlock_value.hpp>

/** @brief 单个底盘电机的原始 CAN 反馈帧。 */
struct ChassisMotorFeedbackRawFrame {
	// Legacy 1 ms kernel-tick timestamp retained for shadow comparison.
	uint64_t timestamp_us; ///< 采样或接收时间戳，单位为微秒，来自单调时钟。
	// High-resolution timestamp from the same cycle counter used by chassis.
	uint64_t precise_timestamp_us; ///< 时间长度，单位为微秒。
	bool valid; ///< 数据有效标志；为 `false` 时其余字段不得用于控制。
	uint8_t data[8]; ///< 协议或总线载荷的原始字节。

	/**
	 * @brief 判断反馈在宽松时限内是否仍然有效。
	 * @param now_us 当前单调时钟时间戳，单位为微秒。
	 * @param timeout_us 允许的数据最大年龄，单位为微秒。
	 * @return 时间戳有效且数据年龄不超过时限时返回 `true`。
	 */
	bool IsFresh(uint64_t now_us, uint64_t timeout_us) const
	{
		return valid && timestamp_us <= now_us && now_us - timestamp_us <= timeout_us;
	}

	/**
	 * @brief 判断反馈在严格时限内是否仍然有效。
	 * @param now_us 当前单调时钟时间戳，单位为微秒。
	 * @param timeout_us 允许的数据最大年龄，单位为微秒。
	 * @return 时间戳有效且数据年龄严格小于时限时返回 `true`。
	 */
	bool IsPreciselyFresh(uint64_t now_us, uint64_t timeout_us) const
	{
		return valid && precise_timestamp_us <= now_us &&
		       now_us - precise_timestamp_us <= timeout_us;
	}
};

/** @brief 左轮电机最近一次原始反馈帧。 */
extern SeqlockValue<ChassisMotorFeedbackRawFrame> left_wheel_feedback_raw;
/** @brief 右轮电机最近一次原始反馈帧。 */
extern SeqlockValue<ChassisMotorFeedbackRawFrame> right_wheel_feedback_raw;
/** @brief 左腿 B 关节电机最近一次原始反馈帧。 */
extern SeqlockValue<ChassisMotorFeedbackRawFrame> left_b_motor_feedback_raw;
/** @brief 左腿 D 关节电机最近一次原始反馈帧。 */
extern SeqlockValue<ChassisMotorFeedbackRawFrame> left_d_motor_feedback_raw;
/** @brief 右腿 B 关节电机最近一次原始反馈帧。 */
extern SeqlockValue<ChassisMotorFeedbackRawFrame> right_b_motor_feedback_raw;
/** @brief 右腿 D 关节电机最近一次原始反馈帧。 */
extern SeqlockValue<ChassisMotorFeedbackRawFrame> right_d_motor_feedback_raw;
