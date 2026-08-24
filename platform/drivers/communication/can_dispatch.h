/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file platform/drivers/communication/can_dispatch.h
 * @ingroup wbr_platform
 * @brief 实现 CAN 收发调度、合帧和诊断统计。
 * @details 这是业务层可见的平台边界。返回负 errno 表示参数、设备或传输失败；调用方不得绕过该接口直接依赖具体驱动实例。
 */

#pragma once

#include <cstdint>

namespace platform {

/** @brief 周期 CAN 指令对应的固定发送槽位。 */
enum class CanTxSlot : uint8_t {
	kLeftWheel = 0, ///< 左侧车轮电机发送槽位。
	kRightWheel, ///< 右侧车轮电机发送槽位。
	kLeftJointB, ///< 左腿 B 关节电机发送槽位。
	kLeftJointD, ///< 左腿 D 关节电机发送槽位。
	kRightJointB, ///< 右腿 B 关节电机发送槽位。
	kRightJointD, ///< 右腿 D 关节电机发送槽位。
	kCount, ///< 枚举项数量，仅用于数组边界。
};

/** @brief CAN 控制器的当前健康状态。 */
struct CanBusHealth {
	bool valid = false; ///< 数据有效标志；为 `false` 时其余字段不得用于控制。
	uint8_t state = 0U; ///< 协议、总线或控制状态枚举值。
	uint8_t tx_error_count = 0U; ///< CAN 控制器发送错误计数器快照。
	uint8_t rx_error_count = 0U; ///< CAN 控制器接收错误计数器快照。
	uint32_t async_tx_error_count = 0U; ///< 异步发送完成回调报告的累计错误次数。
};

/** @brief 单个 CAN 发送槽位的运行统计。 */
struct CanTxSlotStats {
	uint32_t submitted_count = 0U; ///< 上层向该槽位提交帧的累计次数。
	uint32_t enqueued_count = 0U; ///< 该槽位实际进入发送队列的累计次数。
	uint32_t completed_count = 0U; ///< 该槽位发送完成的累计次数。
	uint32_t received_count = 0U; ///< 对应电机反馈帧的累计接收次数。
	uint32_t coalesced_count = 0U; ///< 新指令覆盖尚未发送旧指令的累计次数。
	uint32_t max_rx_interval_us = 0U; ///< 相邻反馈帧最大接收间隔，单位为微秒。
};

/** @brief CAN 异步发送队列的深度统计。 */
struct CanTxQueueStats {
	uint32_t current_depth = 0U; ///< 采样时发送队列中的帧数量。
	uint32_t max_depth = 0U; ///< 自复位以来观测到的发送队列最大深度。
	uint32_t capacity = 0U; ///< 发送队列可容纳的最大帧数量。
};

/**
 * @brief 初始化 CAN 设备、接收过滤器和发送调度线程。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int InitializeCanDispatch();
/**
 * @brief 通知 CAN 调度线程存在待发送数据。
 */
void NotifyCanTxPending();
/**
 * @brief 向指定发送槽位提交标准 CAN 数据帧。
 * @param slot 目标固定发送槽位。
 * @param bus CAN 总线编号，从 0 开始。
 * @param can_id 11 位标准 CAN 标识符。
 * @param[in] data 输入字节缓冲区；长度由相邻长度参数给出。
 * @param dlc CAN 数据长度码，本接口要求不超过 8。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int SubmitCanStandardFrame(CanTxSlot slot, uint8_t bus, uint16_t can_id,
			   const uint8_t *data, uint8_t dlc);
/**
 * @brief 读取指定 CAN 总线的健康状态。
 * @param bus CAN 总线编号，从 0 开始。
 * @param[in,out] health 接收 CAN 健康状态的输出对象；不得为空。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int ReadCanBusHealth(uint8_t bus, CanBusHealth *health);
/**
 * @brief 读取指定 CAN 总线的异步发送错误累计值。
 * @param bus CAN 总线编号，从 0 开始。
 * @return 自初始化以来累计的 CAN 发送截止期错过次数。
 */
uint32_t ReadCanAsyncTxErrorCount(uint8_t bus);
/**
 * @brief 读取指定 CAN 发送槽位的统计数据。
 * @param slot 目标固定发送槽位。
 * @param[in,out] stats 接收统计快照的输出对象；不得为空。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int ReadCanTxSlotStats(CanTxSlot slot, CanTxSlotStats *stats);
/**
 * @brief 读取指定 CAN 总线的发送队列统计。
 * @param bus CAN 总线编号，从 0 开始。
 * @param[in,out] stats 接收统计快照的输出对象；不得为空。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int ReadCanTxQueueStats(uint8_t bus, CanTxQueueStats *stats);
/**
 * @brief 清零 CAN 接收间隔统计。
 */
void ResetCanRxIntervalStats();
/**
 * @brief 清零 CAN 发送诊断统计。
 */
void ResetCanTxDiagnosticStats();

}  // namespace platform
