/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file src/protocols/telemetry/vofa_protocol.cpp
 * @ingroup wbr_protocols
 * @brief 实现 VOFA+ 调试遥测数据封装。
 * @details 实现显式处理字节序、帧长度和量化范围，不依赖动态内存。所有协议错误通过返回值报告，解析器不会直接驱动执行器。
 */

#include <protocols/telemetry/vofa_protocol.h>

#include <cerrno>
#include <cstring>

namespace protocols {

int EncodeVofaJustFloat(const float *channels,
		    size_t channel_count,
		    uint8_t *out,
		    size_t out_capacity,
		    size_t *out_size)
{
	if ((channels == nullptr) || (out == nullptr) || (out_size == nullptr)) {
		return -EINVAL;
	}

	const size_t required_size = VofaJustFloatFrameSize(channel_count);
	if (out_capacity < required_size) {
		return -ENOSPC;
	}

	size_t offset = 0U;
	for (size_t i = 0U; i < channel_count; ++i) {
		static_assert(sizeof(float) == 4U);
		std::memcpy(&out[offset], &channels[i], sizeof(float));
		offset += sizeof(float);
	}

	std::memcpy(&out[offset], kVofaJustFloatTail, sizeof(kVofaJustFloatTail));
	offset += sizeof(kVofaJustFloatTail);
	*out_size = offset;
	return 0;
}

}  // namespace protocols
