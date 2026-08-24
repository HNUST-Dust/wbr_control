/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file src/protocols/imu/hi91_protocol.cpp
 * @ingroup wbr_protocols
 * @brief 实现 HI91 IMU 串口协议的帧解析。
 * @details 实现显式处理字节序、帧长度和量化范围，不依赖动态内存。所有协议错误通过返回值报告，解析器不会直接驱动执行器。
 */

#include <protocols/imu/hi91_protocol.h>

#include <errno.h>
#include <string.h>

namespace protocols {

namespace {

uint16_t ReadLe16(const uint8_t *data)
{
	return static_cast<uint16_t>(data[0]) |
	       (static_cast<uint16_t>(data[1]) << 8U);
}

uint32_t ReadLe32(const uint8_t *data)
{
	return static_cast<uint32_t>(data[0]) |
	       (static_cast<uint32_t>(data[1]) << 8U) |
	       (static_cast<uint32_t>(data[2]) << 16U) |
	       (static_cast<uint32_t>(data[3]) << 24U);
}

float ReadLeFloat(const uint8_t *data)
{
	const uint32_t raw = ReadLe32(data);
	float value = 0.0F;
	memcpy(&value, &raw, sizeof(value));
	return value;
}

void DecodeFloat3(const uint8_t *data, float value[3])
{
	value[0] = ReadLeFloat(&data[0]);
	value[1] = ReadLeFloat(&data[4]);
	value[2] = ReadLeFloat(&data[8]);
}

uint16_t Crc16Update(uint16_t crc, const uint8_t *data, size_t size)
{
	for (size_t i = 0U; i < size; ++i) {
		crc ^= static_cast<uint16_t>(data[i]) << 8U;
		for (uint8_t bit = 0U; bit < 8U; ++bit) {
			if ((crc & 0x8000U) != 0U) {
				crc = static_cast<uint16_t>((crc << 1U) ^ 0x1021U);
			} else {
				crc = static_cast<uint16_t>(crc << 1U);
			}
		}
	}
	return crc;
}

}  // namespace

uint16_t CalculateHi91FrameCrc(uint16_t payload_length, const uint8_t *payload)
{
	const uint8_t header[4] = {
		kHi91FrameSof0,
		kHi91FrameSof1,
		static_cast<uint8_t>(payload_length & 0xFFU),
		static_cast<uint8_t>((payload_length >> 8U) & 0xFFU),
	};
	/* HiPNUC manual: start from 0, then update over SOF+LEN and payload
	 * separately. CRC bytes themselves are excluded. */
	uint16_t crc = 0U;
	crc = Crc16Update(crc, header, sizeof(header));
	return Crc16Update(crc, payload, payload_length);
}

int DecodeHi91Frame(const uint8_t *data, size_t len, bool strict_crc, Hi91Sample *out)
{
	if ((data == nullptr) || (out == nullptr)) {
		return -EINVAL;
	}

	if (len < kHi91FrameHeaderSize) {
		return -EINVAL;
	}

	if ((data[0] != kHi91FrameSof0) || (data[1] != kHi91FrameSof1)) {
		return -EBADMSG;
	}

	const uint16_t payload_length = ReadLe16(&data[2]);
	if ((payload_length == 0U) || (payload_length > kHi91MaxPayloadLength)) {
		return -EBADMSG;
	}

	if (len < kHi91FrameHeaderSize + payload_length) {
		return -EMSGSIZE;
	}

	const uint8_t *payload = &data[kHi91FrameHeaderSize];
	if (strict_crc &&
	    (CalculateHi91FrameCrc(payload_length, payload) != ReadLe16(&data[4]))) {
		return -EILSEQ;
	}

	if ((payload_length < kHi91DataLength) || (payload[0] != kHi91DataTag)) {
		return -EBADMSG;
	}

	out->main_status = ReadLe16(&payload[1]);
	out->temperature_c = static_cast<int8_t>(payload[3]);
	out->air_pressure = ReadLeFloat(&payload[4]);
	out->system_time_ms = ReadLe32(&payload[8]);
	DecodeFloat3(&payload[12], out->accel_g);
	DecodeFloat3(&payload[24], out->gyro_dps);
	DecodeFloat3(&payload[36], out->mag_ut);
	out->roll_deg = ReadLeFloat(&payload[48]);
	out->pitch_deg = ReadLeFloat(&payload[52]);
	out->yaw_deg = ReadLeFloat(&payload[56]);
	out->quat[0] = ReadLeFloat(&payload[60]);
	out->quat[1] = ReadLeFloat(&payload[64]);
	out->quat[2] = ReadLeFloat(&payload[68]);
	out->quat[3] = ReadLeFloat(&payload[72]);
	return 0;
}

uint16_t Hi91Crc16CcittFalse(const uint8_t *data, size_t size)
{
	return Crc16Update(0U, data, size);
}

}  // namespace protocols
