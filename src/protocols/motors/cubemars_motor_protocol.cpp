/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <algorithm>

#include <protocols/motors/cubemars_motor_protocol.h>

namespace protocols {

namespace {

uint16_t FloatToUInt(float value, float min_value, float max_value, uint8_t bits)
{
	const float v = std::clamp(value, min_value, max_value);
	const float span = max_value - min_value;
	const float scaled = (v - min_value) * (static_cast<float>((1U << bits) - 1U)) / span;
	return static_cast<uint16_t>(scaled + 0.5f);
}

int GetFixedCommand(uint8_t tail, uint8_t out[8])
{
	if (out == nullptr) {
		return -EINVAL;
	}

	for (int i = 0; i < 8; ++i) {
		out[i] = 0xFFU;
	}
	out[7] = tail;
	return 0;
}

}  // namespace

int DecodeCubemarsFeedback(const uint8_t *data, uint8_t dlc, CubemarsFeedback *out)
{
	if ((data == nullptr) || (out == nullptr) || (dlc < 6U)) {
		return -EINVAL;
	}

	out->id = data[0];
	out->position_raw = (static_cast<uint16_t>(data[1]) << 8) | data[2];
	out->velocity_raw = (static_cast<uint16_t>(data[3]) << 4) | (data[4] >> 4);
	out->torque_raw = (static_cast<uint16_t>(data[4] & 0x0F) << 8) | data[5];
	return 0;
}

int GetCubemarsEnterFrame(uint8_t out[8])
{
	return GetFixedCommand(0xFCU, out);
}

int GetCubemarsExitFrame(uint8_t out[8])
{
	return GetFixedCommand(0xFDU, out);
}

int GetCubemarsSaveZeroFrame(uint8_t out[8])
{
	return GetFixedCommand(0xFEU, out);
}

int PackCubemarsMitCommand(const CubemarsMitCommand *cmd, const CubemarsMitRange *range,
			   uint8_t out[8])
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

}  // namespace protocols
