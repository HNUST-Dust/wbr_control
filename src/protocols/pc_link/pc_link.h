/* SPDX-License-Identifier: Apache-2.0 */

/**
* @file src/protocols/pc_link/pc_link.h
 * @ingroup wbr_protocols
 * @brief 实现控制器与上位机之间的通信协议。
 * @details 接口直接处理线协议字节序列。调用方负责提供声明长度的有效缓冲区；解码失败时输出对象内容不应作为有效数据使用。
 */

#ifndef WBR_CONTROL_PROTOCOLS_PC_LINK_PC_LINK_H_
#define WBR_CONTROL_PROTOCOLS_PC_LINK_PC_LINK_H_

#include <stddef.h>
#include <stdint.h>

namespace protocols {

/*
 * CRC-16 parameters used by the vision host.
 *
 * The host processes each byte least-significant bit first with the reflected
 * form of polynomial 0x1021.  It uses init 0xFFFF and no final XOR.  The CRC
 * field itself is serialized little-endian.
 */
constexpr uint16_t kPcCommCrc16ReflectedPoly = 0x8408U; ///< 视觉链路 CRC-16 多项式的反射形式。
constexpr uint16_t kPcCommCrc16Init = 0xFFFFU; ///< 视觉链路 CRC-16 初始值。

/* Packet sizes on the wire (head + fields + crc16). */
constexpr size_t kPcCommSendPacketSize = 43U; ///< MCU 状态上行数据包固定长度，单位为字节。
constexpr size_t kPcCommRecvPacketSize = 29U; ///< 上位机命令下行数据包固定长度，单位为字节。

#pragma pack(push, 1)

/*
 * Auto-aim state reported by the MCU to the PC.
 * Wire layout: 'S','P' + mode + q[4] + yaw + pitch + bullet + crc16.
 */
/** @brief 控制器发送给上位机的自瞄状态数据。 */
struct PCSendAutoAimData
{
	uint8_t head[2] = {'S', 'P'}; ///< 固定帧头字节，线上顺序为 `'S'`、`'P'`。

	uint8_t mode = 0; ///< MCU 工作模式：0 表示空闲，1 表示自瞄。

	float q[4]; ///< 机体姿态单位四元数，按 `[w, x, y, z]` 排列。

	struct
	{
		float yaw_ang; ///< 偏航轴角度，单位为弧度。
		float yaw_vel; ///< 偏航轴角速度，单位为弧度每秒。
		} yaw; ///< 偏航轴当前状态。

	struct
	{
		float pitch_ang; ///< 俯仰轴角度，单位为弧度。
		float pitch_vel; ///< 俯仰轴角速度，单位为弧度每秒。
		} pitch; ///< 俯仰轴当前状态。

	struct
	{
		float bullet_speed; ///< 最近弹丸初速度，单位为米每秒。
		uint16_t bullet_count; ///< 上电后累计发射弹丸数量。
		} bullet; ///< 发射机构状态。

	uint16_t crc16; ///< 整帧 CRC-16 校验值，在线上按小端序发送。
};

/*
 * Auto-aim command received from the PC.
 * Wire layout: 'S','P' + mode + yaw + pitch + crc16.
 */
/** @brief 上位机发送给控制器的自瞄目标轨迹。 */
struct PCRecvAutoAimData
{
	uint8_t head[2] = {'S', 'P'}; ///< 固定帧头字节，线上顺序为 `'S'`、`'P'`。
	uint8_t mode = 0; ///< 0 表示空闲，1 表示仅瞄准，2 表示瞄准并请求发射。

	struct
	{
		float yaw_ang; ///< 目标偏航角，单位为弧度。
		float yaw_vel; ///< 目标偏航角速度，单位为弧度每秒。
		float yaw_acc; ///< 目标偏航角加速度，单位为弧度每二次方秒。
		} yaw; ///< 偏航轴目标轨迹。

	struct
	{
		float pitch_ang; ///< 目标俯仰角，单位为弧度。
		float pitch_vel; ///< 目标俯仰角速度，单位为弧度每秒。
		float pitch_acc; ///< 目标俯仰角加速度，单位为弧度每二次方秒。
		} pitch; ///< 俯仰轴目标轨迹。

	uint16_t crc16; ///< 整帧 CRC-16 校验值，在线上按小端序发送。
};

#pragma pack(pop)

static_assert(sizeof(PCSendAutoAimData) == kPcCommSendPacketSize,
	      "PCSendAutoAimData wire layout mismatch");
static_assert(sizeof(PCRecvAutoAimData) == kPcCommRecvPacketSize,
	      "PCRecvAutoAimData wire layout mismatch");

/**
 * @brief 将 MCU 自瞄状态序列化为小端线上数据包。
 * @param[in] data 待编码状态；不得为空。
 * @param[out] out 输出字节缓冲区；不得为空。
 * @param out_capacity 输出缓冲区容量，至少为 `kPcCommSendPacketSize`。
 * @param[out] out_len 实际编码长度，成功时为 `kPcCommSendPacketSize`。
 * @return 成功返回 0，参数无效或容量不足返回负 errno 错误码。
 */
int EncodePcCommSend(const PCSendAutoAimData *data,
		uint8_t *out,
		size_t out_capacity,
		size_t *out_len);

/**
 * @brief 将上位机自瞄目标结构序列化为小端线上数据包。
 * @param[in] data 待编码目标；不得为空。
 * @param[out] out 输出字节缓冲区；不得为空。
 * @param out_capacity 输出缓冲区容量，至少为 `kPcCommRecvPacketSize`。
 * @param[out] out_len 实际编码长度，成功时为 `kPcCommRecvPacketSize`。
 * @return 成功返回 0，参数无效或容量不足返回负 errno 错误码。
 */
int EncodePcCommRecv(const PCRecvAutoAimData *data,
		uint8_t *out,
		size_t out_capacity,
		size_t *out_len);

/**
 * @brief 校验并解码 MCU 自瞄状态数据包。
 * @param[in] packet 完整线上数据包；不得为空。
 * @param packet_len 数据包长度，必须等于 `kPcCommSendPacketSize`。
 * @param[out] out 接收解码结果；失败时内容保持未定义。
 * @return CRC、帧头和长度均有效时返回 0，否则返回负 errno 错误码。
 */
int DecodePcCommSend(const uint8_t *packet,
		size_t packet_len,
		PCSendAutoAimData *out);

/**
 * @brief 校验并解码上位机自瞄目标数据包。
 * @param[in] packet 完整线上数据包；不得为空。
 * @param packet_len 数据包长度，必须等于 `kPcCommRecvPacketSize`。
 * @param[out] out 接收解码结果；失败时内容保持未定义。
 * @return CRC、帧头和长度均有效时返回 0，否则返回负 errno 错误码。
 */
int DecodePcCommRecv(const uint8_t *packet,
		size_t packet_len,
		PCRecvAutoAimData *out);

}  // namespace protocols

#endif /* WBR_CONTROL_PROTOCOLS_PC_LINK_PC_LINK_H_ */
