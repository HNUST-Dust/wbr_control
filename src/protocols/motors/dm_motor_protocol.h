/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file src/protocols/motors/dm_motor_protocol.h
 * @ingroup wbr_protocols
 * @brief 实现达妙电机 CAN 协议编解码。
 * @details 接口直接处理线协议字节序列。调用方负责提供声明长度的有效缓冲区；解码失败时输出对象内容不应作为有效数据使用。
 */

#ifndef WBR_CONTROL_PROTOCOLS_MOTORS_DM_MOTOR_PROTOCOL_H_
#define WBR_CONTROL_PROTOCOLS_MOTORS_DM_MOTOR_PROTOCOL_H_

#include <stdint.h>

namespace protocols {

/** @brief 达妙电机 1 至 4 号反馈格式。 */
struct DmMotorFeedback1To4 {
	uint16_t encoder; ///< 电机编码器原始计数值。
	int16_t omega_x100; ///< 角速度的百倍定点数，除以 100 后单位为弧度每秒。
	int16_t current_ma; ///< 电机相电流，单位为毫安。
	uint8_t rotor_temperature; ///< 电机转子温度，单位为摄氏度。
	uint8_t mos_temperature; ///< 电机功率 MOS 温度，单位为摄氏度。
};

/** @brief 达妙电机普通模式的原始字段。 */
struct DmMotorRawDataNormal
{
    uint8_t can_id : 4; ///< 反馈源电机 ID 的低 4 位。
    uint8_t control_status_enum : 4; ///< 电机控制状态的 4 位协议枚举。
    uint16_t angle_reverse; ///< 按协议字节序重组后的 16 位位置量化值。
    uint8_t omega_11_4; ///< 12 位速度量化值的高 8 位。
    uint8_t omega_3_0_torque_11_8; ///< 速度低 4 位与力矩高 4 位的组合字节。
    uint8_t torque_7_0; ///< 12 位力矩量化值的低 8 位。
    uint8_t mos_temperature; ///< 电机功率 MOS 温度，单位为摄氏度。
    uint8_t rotor_temperature; ///< 电机转子温度，单位为摄氏度。
} __attribute__((packed));

/** @brief 达妙电机普通模式的解码结果。 */
struct DmMotorFeedbackNormal
{
    uint8_t control_status; ///< 电机控制状态的协议枚举值。
    uint16_t angle; ///< 16 位位置量化值，需结合 `DmMitRange` 解码。
    uint16_t omega; ///< 12 位速度量化值，需结合 `DmMitRange` 解码。
    uint16_t torque; ///< 12 位力矩量化值，需结合 `DmMitRange` 解码。
    float mos_temperature; ///< 电机功率 MOS 温度，单位为摄氏度。
    float rotor_temperature; ///< 电机转子温度，单位为摄氏度。
};

/** @brief 达妙电机特殊控制命令。 */
enum class DmControlCommand {
	kClearError, ///< 清除电机当前锁存故障。
	kEnter, ///< 进入 MIT 闭环控制模式。
	kExit, ///< 退出闭环控制并停止力矩输出。
	kSaveZero, ///< 将当前机械位置保存为电机零位。
};

/** @brief 达妙电机 MIT 模式控制目标。 */
struct DmMitCommand {
	float position; ///< 位置或转角状态；单位由所属协议或控制接口规定。
	float velocity; ///< 线速度或角速度状态；单位由所属数据结构规定。
	float kp; ///< 位置环比例增益。
	float kd; ///< 速度反馈微分增益。
	float torque; ///< 电机或关节力矩，单位为牛·米。
};

/** @brief 达妙电机 MIT 模式各物理量的编码范围。 */
struct DmMitRange {
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
 * @brief 解码达妙电机 1 至 4 号反馈帧。
 * @param[in] data 输入字节缓冲区；长度由相邻长度参数给出。
 * @param dlc CAN 数据长度码，本接口要求不超过 8。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int DecodeDmFeedback1To4(const uint8_t *data, uint8_t dlc, DmMotorFeedback1To4 *out);
/**
 * @brief 解码达妙电机普通模式反馈帧。
 * @param[in] data 输入字节缓冲区；长度由相邻长度参数给出。
 * @param dlc CAN 数据长度码，本接口要求不超过 8。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int DecodeDmFeedbackNormal(const uint8_t *data, uint8_t dlc, DmMotorFeedbackNormal *out);
/**
 * @brief 将 MIT 无符号编码还原为物理量。
 * @param value 待发布或参与计算的输入值。
 * @param minimum 编码物理量允许的最小值。
 * @param maximum 编码物理量允许的最大值。
 * @param bits 无符号编码使用的有效位数。
 * @return 映射到指定物理范围后的浮点值。
 */
float DecodeDmMitValue(uint16_t value, float minimum, float maximum, uint8_t bits);
/**
 * @brief 读取达妙反馈中的物理位置。
 * @param[in] feedback 达妙普通模式反馈的原始量化字段。
 * @param[in] range 各物理量的编码范围；不得为空。
 * @return 解码后的位置，单位为弧度。
 */
float DmFeedbackPosition(const DmMotorFeedbackNormal &feedback, const DmMitRange &range);
/**
 * @brief 读取达妙反馈中的物理速度。
 * @param[in] feedback 达妙普通模式反馈的原始量化字段。
 * @param[in] range 各物理量的编码范围；不得为空。
 * @return 解码后的角速度，单位为弧度每秒。
 */
float DmFeedbackVelocity(const DmMotorFeedbackNormal &feedback, const DmMitRange &range);
/**
 * @brief 读取达妙反馈中的物理力矩。
 * @param[in] feedback 达妙普通模式反馈的原始量化字段。
 * @param[in] range 各物理量的编码范围；不得为空。
 * @return 解码后的电机力矩，单位为牛·米。
 */
float DmFeedbackTorque(const DmMotorFeedbackNormal &feedback, const DmMitRange &range);
/**
 * @brief 生成达妙电机特殊控制命令帧。
 * @param cmd 待编码的电机控制目标；不得为空。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int GetDmControlCommandFrame(DmControlCommand cmd, uint8_t out[8]);
/**
 * @brief 将达妙 MIT 控制目标编码为 CAN 载荷。
 * @param[in] cmd 待编码的电机控制目标；不得为空。
 * @param[in] range 各物理量的编码范围；不得为空。
 * @param[out] out 接收结果的输出对象；不得为空。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int PackDmMitCommand(const DmMitCommand *cmd, const DmMitRange *range, uint8_t out[8]);
/**
 * @brief 将单台达妙电机电流写入 1 至 4 号电机组合帧的对应槽位。
 * @param motor_can_id 电机 CAN ID；支持 0x301 至 0x308。
 * @param current_ma 目标电流，单位为毫安。
 * @param[in,out] frame_payload 长度为 8 字节的组合帧载荷；函数仅改写目标电机槽位。
 * @return 成功返回 0；ID 越界或输出缓冲区为空时返回负 errno 错误码。
 */
int PackDm1To4CurrentFrame(uint16_t motor_can_id, int16_t current_ma,
			  uint8_t frame_payload[8]);

}  // namespace protocols

#endif /* WBR_CONTROL_PROTOCOLS_MOTORS_DM_MOTOR_PROTOCOL_H_ */
