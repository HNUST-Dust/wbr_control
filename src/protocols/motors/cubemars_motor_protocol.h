/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file src/protocols/motors/cubemars_motor_protocol.h
 * @ingroup wbr_protocols
 * @brief 实现 CubeMars 电机 CAN 协议编解码。
 * @details 接口直接处理线协议字节序列。调用方负责提供声明长度的有效缓冲区；解码失败时输出对象内容不应作为有效数据使用。
 */

#ifndef WBR_CONTROL_PROTOCOLS_MOTORS_CUBEMARS_MOTOR_PROTOCOL_H_
#define WBR_CONTROL_PROTOCOLS_MOTORS_CUBEMARS_MOTOR_PROTOCOL_H_

#include <stdint.h>

namespace protocols {

/** @brief CubeMars 电机反馈数据。 */
struct CubemarsFeedback {
	uint8_t id; ///< 设备或协议对象标识符。
	uint16_t position_raw; ///< 协议反馈的原始位置量化值。
	uint16_t velocity_raw; ///< 线速度，单位为米每秒。
	uint16_t torque_raw; ///< 力矩，单位为牛·米。
};

/** @brief CubeMars MIT 模式控制目标。 */
struct CubemarsMitCommand {
	float position; ///< 位置或转角状态；单位由所属协议或控制接口规定。
	float velocity; ///< 线速度或角速度状态；单位由所属数据结构规定。
	float kp; ///< 位置环比例增益。
	float kd; ///< 速度反馈微分增益。
	float torque; ///< 电机或关节力矩，单位为牛·米。
};

/** @brief CubeMars MIT 模式各物理量的编码范围。 */
struct CubemarsMitRange {
	float p_min; ///< MIT 位置编码下限，单位为弧度。
	float p_max; ///< MIT 位置编码上限，单位为弧度。
	float v_min; ///< MIT 速度编码下限，单位为弧度每秒。
	float v_max; ///< MIT 速度编码上限，单位为弧度每秒。
	float kp_min; ///< MIT 位置增益编码下限。
	float kp_max; ///< MIT 位置增益编码上限。
	float kd_min; ///< MIT 速度增益编码下限。
	float kd_max; ///< MIT 速度增益编码上限。
	float t_min; ///< MIT 力矩编码下限，单位为牛·米。
	float t_max; ///< MIT 力矩编码上限，单位为牛·米。
};

/**
 * @brief 解码 CubeMars 电机反馈帧。
 * @param[in] data 输入字节缓冲区；长度由相邻长度参数给出。
 * @param dlc CAN 数据长度码，本接口要求不超过 8。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int DecodeCubemarsFeedback(const uint8_t *data, uint8_t dlc, CubemarsFeedback *out);
/**
 * @brief 生成 CubeMars 电机进入控制模式的命令帧。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int GetCubemarsEnterFrame(uint8_t out[8]);
/**
 * @brief 生成 CubeMars 电机退出控制模式的命令帧。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int GetCubemarsExitFrame(uint8_t out[8]);
/**
 * @brief 生成 CubeMars 电机保存零位的命令帧。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int GetCubemarsSaveZeroFrame(uint8_t out[8]);
/**
 * @brief 将 CubeMars MIT 控制目标编码为 CAN 载荷。
 * @param[in] cmd 待编码的电机控制目标；不得为空。
 * @param[in] range 各物理量的编码范围；不得为空。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int PackCubemarsMitCommand(const CubemarsMitCommand *cmd, const CubemarsMitRange *range,
			   uint8_t out[8]);

}  // namespace protocols

#endif /* WBR_CONTROL_PROTOCOLS_MOTORS_CUBEMARS_MOTOR_PROTOCOL_H_ */
