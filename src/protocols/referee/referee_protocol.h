/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file src/protocols/referee/referee_protocol.h
 * @ingroup wbr_protocols
 * @brief 实现机器人裁判系统协议的帧解析。
 * @details 接口直接处理线协议字节序列。调用方负责提供声明长度的有效缓冲区；解码失败时输出对象内容不应作为有效数据使用。
 */

#pragma once

// Public RoboMaster referee system wire-format contract.

#include <stddef.h>
#include <stdint.h>

namespace protocols {

constexpr uint8_t kRefereeSof = 0xA5U; ///< 裁判系统协议帧起始字节。

/*
 * Frame layout:
 *   byte 0       SOF (0xa5)
 *   bytes 1-2    data length (little-endian)
 *   byte 3       sequence
 *   byte 4       crc8 (header)
 *   bytes 5-6    command id (little-endian)
 *   bytes 7..    data (data length bytes)
 *   last 2 bytes crc16 (frame)
 */
constexpr size_t kRefereeFrameHeaderSize = 7U; ///< 裁判系统帧头长度，单位为字节。
constexpr size_t kRefereeFrameTrailerSize = 2U; ///< 裁判系统帧尾 CRC-16 长度，单位为字节。
constexpr size_t kRefereeMaxFrameSize = 160U; ///< 解析器接受的裁判系统最大完整帧长度。
constexpr size_t kRefereeMaxDataLength =
	kRefereeMaxFrameSize - kRefereeFrameHeaderSize - kRefereeFrameTrailerSize; ///< 单帧允许的最大数据区长度，单位为字节。

constexpr uint16_t kRefereeCmdGameStatus = 0x0001U; ///< 比赛状态命令标识符。
constexpr uint16_t kRefereeCmdRobotStatus = 0x0201U; ///< 机器人状态命令标识符。
constexpr uint16_t kRefereeCmdShootData = 0x0207U; ///< 射击事件命令标识符。

/** @brief 裁判系统比赛状态数据。 */
struct RefereeGameStatus {
	uint8_t game_type; ///< 比赛类型，来自状态字节的低 4 位。
	uint8_t game_progress; ///< 当前比赛阶段，来自状态字节的高 4 位。
	uint16_t stage_remain_time; ///< 当前比赛阶段剩余时间，单位为秒。
};

/** @brief 裁判系统机器人状态数据。 */
struct RefereeRobotStatus {
	uint16_t current_hp; ///< 电流值，单位由对应执行器协议规定。
	uint16_t max_hp; ///< 机器人最大生命值。
	uint16_t chassis_power_limit; ///< 控制器使用的可配置阈值。
	uint8_t gimbal_power_on; ///< 裁判系统报告的云台供电状态。
};

/** @brief 裁判系统射击事件数据。 */
struct RefereeShootData {
	uint8_t bullet_type; ///< 裁判系统报告的弹丸类型。
	uint8_t launching_frequency; ///< 发射机构目标或实测频率，单位为赫兹。
	float initial_speed; ///< 线速度，单位为米每秒。
};

/** @brief 裁判系统协议帧的通用解码结果。 */
struct RefereeFrame {
	uint16_t cmd_id; ///< 协议或设备标识符。
	const uint8_t *data; ///< 协议或总线载荷的原始字节。
	uint16_t data_len; ///< 协议数据有效长度，单位为字节。
};

/*
 * Validate one complete referee frame and extract its command id and data.
 *
 * CRC8/CRC16 are part of the referee protocol but are not verified yet;
 * see the frame layout comment above.  Returns 0 on success or a negative
 * errno (-EINVAL / -EMSGSIZE / -EBADMSG).
 */
/**
 * @brief 校验并拆解一帧裁判系统协议数据。
 * @param[in] frame 待入队、解码或处理的数据帧；不得为空。
 * @param frame_len 完整协议帧长度，单位为字节。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int DecodeRefereeFrame(const uint8_t *frame, size_t frame_len, RefereeFrame *out);

/* Decode the payloads of the currently supported referee commands. */
/**
 * @brief 解码裁判系统比赛状态载荷。
 * @param[in] data 输入字节缓冲区；长度由相邻长度参数给出。
 * @param len 输入缓冲区中的有效字节数。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int DecodeRefereeGameStatus(const uint8_t *data, size_t len, RefereeGameStatus *out);
/**
 * @brief 解码裁判系统机器人状态载荷。
 * @param[in] data 输入字节缓冲区；长度由相邻长度参数给出。
 * @param len 输入缓冲区中的有效字节数。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int DecodeRefereeRobotStatus(const uint8_t *data, size_t len, RefereeRobotStatus *out);
/**
 * @brief 解码裁判系统射击事件载荷。
 * @param[in] data 输入字节缓冲区；长度由相邻长度参数给出。
 * @param len 输入缓冲区中的有效字节数。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int DecodeRefereeShootData(const uint8_t *data, size_t len, RefereeShootData *out);

}  // namespace protocols
