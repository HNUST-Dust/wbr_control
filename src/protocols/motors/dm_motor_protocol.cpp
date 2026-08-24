/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file src/protocols/motors/dm_motor_protocol.cpp
 * @ingroup wbr_protocols
 * @brief 实现达妙电机 CAN 协议编解码。
 * @details 实现显式处理字节序、帧长度和量化范围，不依赖动态内存。所有协议错误通过返回值报告，解析器不会直接驱动执行器。
 */

#include <errno.h>

#include <algorithm>

#include <zephyr/sys/byteorder.h>

#include <protocols/motors/dm_motor_protocol.h>

/** @brief 摄氏温标转换为开尔文温标时使用的零点偏移。 */
constexpr float kCelsiusToKelvin = 273.15f;

namespace protocols {

namespace {

uint16_t FloatToUInt(float value, float min_value, float max_value, uint8_t bits)
{
	const float v = std::clamp(value, min_value, max_value);
	const float span = max_value - min_value;
	const float scaled = (v - min_value) * (static_cast<float>((1U << bits) - 1U)) / span;
	return static_cast<uint16_t>(scaled + 0.5f);
}

}  // namespace

int DecodeDmFeedback1To4(const uint8_t *data, uint8_t dlc, DmMotorFeedback1To4 *out)
{
	if ((data == nullptr) || (out == nullptr) || (dlc < 8U)) {
		return -EINVAL;
	}

	out->encoder = sys_get_be16(&data[0]);
	out->omega_x100 = static_cast<int16_t>(sys_get_be16(&data[2]));
	out->current_ma = static_cast<int16_t>(sys_get_be16(&data[4]));
	out->rotor_temperature = data[6];
	out->mos_temperature = data[7];
	return 0;
}

int DecodeDmFeedbackNormal(const uint8_t *data, uint8_t dlc, DmMotorFeedbackNormal *out)
{
	if ((data == nullptr) || (out == nullptr) || (dlc < 8U)) {
		return -EINVAL;
	}
	out->control_status = data[0] >> 4;
	out->angle = (static_cast<uint16_t>(data[1]) << 8) | data[2];
	out->omega = (static_cast<uint16_t>(data[3]) << 4) | (data[4] >> 4);
	out->torque = (static_cast<uint16_t>(data[4] & 0x0f) << 8) | data[5];
	out->mos_temperature = data[6] + kCelsiusToKelvin;
	out->rotor_temperature = data[7] + kCelsiusToKelvin;

	return 0;
}

float DecodeDmMitValue(uint16_t value, float minimum, float maximum, uint8_t bits)
{
	const float maximum_integer = static_cast<float>((1U << bits) - 1U);
	return static_cast<float>(value) * (maximum - minimum) / maximum_integer + minimum;
}

float DmFeedbackPosition(const DmMotorFeedbackNormal &feedback, const DmMitRange &range)
{
	return DecodeDmMitValue(feedback.angle, range.p_min, range.p_max, 16U);
}

float DmFeedbackVelocity(const DmMotorFeedbackNormal &feedback, const DmMitRange &range)
{
	return DecodeDmMitValue(feedback.omega, range.v_min, range.v_max, 12U);
}

float DmFeedbackTorque(const DmMotorFeedbackNormal &feedback, const DmMitRange &range)
{
	return DecodeDmMitValue(feedback.torque, range.t_min, range.t_max, 12U);
}

int GetDmControlCommandFrame(DmControlCommand cmd, uint8_t out[8])
{
	if (out == nullptr) {
		return -EINVAL;
	}

	for (int i = 0; i < 8; ++i) {
		out[i] = 0xFFU;
	}

	switch (cmd) {
	case DmControlCommand::kClearError:
		out[7] = 0xFBU;
		break;
	case DmControlCommand::kEnter:
		out[7] = 0xFCU;
		break;
	case DmControlCommand::kExit:
		out[7] = 0xFDU;
		break;
	case DmControlCommand::kSaveZero:
		out[7] = 0xFEU;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int PackDmMitCommand(const DmMitCommand *cmd, const DmMitRange *range, uint8_t out[8])
{
	if ((cmd == nullptr) || (range == nullptr) || (out == nullptr)) {
		return -EINVAL;
	}

	const uint16_t p_int = FloatToUInt(cmd->position, range->p_min, range->p_max, 16);
	const uint16_t v_int = FloatToUInt(cmd->velocity, range->v_min, range->v_max, 12);
	const uint16_t kp_int = FloatToUInt(cmd->kp, range->kp_min, range->kp_max, 12);
	const uint16_t kd_int = FloatToUInt(cmd->kd, range->kd_min, range->kd_max, 12);
	const uint16_t t_int = FloatToUInt(cmd->torque, range->t_min, range->t_max, 12);

	out[0] = static_cast<uint8_t>(p_int >> 8);
	out[1] = static_cast<uint8_t>(p_int & 0xFFU);
	out[2] = static_cast<uint8_t>(v_int >> 4);
	out[3] = static_cast<uint8_t>(((v_int & 0x0FU) << 4) | (kp_int >> 8));
	out[4] = static_cast<uint8_t>(kp_int & 0xFFU);
	out[5] = static_cast<uint8_t>(kd_int >> 4);
	out[6] = static_cast<uint8_t>(((kd_int & 0x0FU) << 4) | (t_int >> 8));
	out[7] = static_cast<uint8_t>(t_int & 0xFFU);
	return 0;
}

int PackDm1To4CurrentFrame(uint16_t motor_can_id, int16_t current_ma,
			  uint8_t frame_payload[8])
{
	if (frame_payload == nullptr) {
		return -EINVAL;
	}

	int slot = -1;
	if ((motor_can_id >= 0x301U) && (motor_can_id <= 0x304U)) {
		slot = static_cast<int>(motor_can_id - 0x301U);
	} else if ((motor_can_id >= 0x305U) && (motor_can_id <= 0x308U)) {
		slot = static_cast<int>(motor_can_id - 0x305U);
	} else {
		return -EINVAL;
	}

	const uint16_t encoded = static_cast<uint16_t>(current_ma);
	frame_payload[slot * 2] = static_cast<uint8_t>((encoded >> 8) & 0xFFU);
	frame_payload[slot * 2 + 1] = static_cast<uint8_t>(encoded & 0xFFU);
	return 0;
}

}  // namespace protocols
