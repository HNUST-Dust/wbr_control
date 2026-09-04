/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file src/protocols/imu/hi91_protocol.h
 * @ingroup wbr_protocols
 * @brief 实现 HI91 IMU 串口协议的帧解析。
 * @details 接口直接处理线协议字节序列。调用方负责提供声明长度的有效缓冲区；解码失败时输出对象内容不应作为有效数据使用。
 */

#pragma once

// Public HI91 wire-format contract.

#include <stddef.h>
#include <stdint.h>

namespace protocols {

constexpr uint8_t kHi91FrameSof0 = 0x5AU; ///< HI91 帧起始标志的第一个固定字节。
constexpr uint8_t kHi91FrameSof1 = 0xA5U; ///< HI91 帧起始标志的第二个固定字节。
constexpr uint8_t kHi91DataTag = 0x91U; ///< 标识标准姿态数据载荷的协议标签。
constexpr uint16_t kHi91DataLength = 76U; ///< HI91 数据帧标准载荷长度，单位为字节。
constexpr uint16_t kHi91MaxPayloadLength = 256U; ///< 解析器接受的最大载荷长度，单位为字节。
constexpr size_t kHi91FrameHeaderSize = 6U; ///< 帧头长度，包含双字节 SOF、长度和 CRC 字段，单位为字节。

/** @brief HI91 协议帧解码后的物理量。 */
struct Hi91Sample {
	uint16_t main_status; ///< 上位机协议报告的主要工作状态。
	int8_t temperature_c; ///< 温度，单位为摄氏度。
	float air_pressure; ///< 气压测量值，单位为帕。
	uint32_t system_time_ms; ///< 时间长度，单位为毫秒。
	float accel_g[3]; ///< 以标准重力加速度为单位的三轴加速度。
	float gyro_dps[3]; ///< 三轴陀螺仪角速度，单位为度每秒。
	float mag_ut[3]; ///< 三轴磁场测量值，单位为微特斯拉。
	float roll_deg; ///< 机体横滚角，单位为度。
	float pitch_deg; ///< 机体俯仰角，单位为度。
	float yaw_deg; ///< 归一化航向角，单位为度。
	float quat[4]; ///< 按 `[w, x, y, z]` 排列的姿态单位四元数。
};

/*
 * Decode one complete HI91 frame (SOF + len + crc + payload).
 *
 * `strict_crc` rejects frames whose CRC16 does not match the device
 * (see Kconfig WBR_CONTROL_HI91_IMU_STRICT_CRC); keep it false until the exact
 * CRC variant is confirmed.
 *
 * Returns 0 on success, -EILSEQ for a CRC mismatch, or another negative errno
 * value (-EINVAL / -EMSGSIZE / -EBADMSG) for framing/format failures.
 */
/**
 * @brief 校验并解码一帧 HI91 协议数据。
 * @param[in] data 输入字节缓冲区；长度由相邻长度参数给出。
 * @param len 输入缓冲区中的有效字节数。
 * @param strict_crc 为 `true` 时要求协议 CRC 校验通过。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int DecodeHi91Frame(const uint8_t *data, size_t len, bool strict_crc, Hi91Sample *out);

/* Calculate the CRC stored in a frame from its payload length and payload. */
/**
 * @brief 计算 HI91 帧头与载荷的协议校验值。
 * @param payload_length 协议载荷长度，单位为字节。
 * @param[in] payload 协议载荷字节缓冲区。
 * @return 按 HI91 帧格式计算的 16 位 CRC。
 */
uint16_t CalculateHi91FrameCrc(uint16_t payload_length, const uint8_t *payload);

/* HiPNUC CRC-16/CCITT update with the manual's initial value 0x0000. */
/**
 * @brief 计算 CRC-16/CCITT-FALSE 校验值。
 * @param[in] data 输入字节缓冲区；长度由相邻长度参数给出。
 * @param size 输入数据的字节数。
 * @return 从初值 0x0000 开始迭代得到的 CRC-16 值。
 */
uint16_t Hi91Crc16CcittFalse(const uint8_t *data, size_t size);

}  // namespace protocols
