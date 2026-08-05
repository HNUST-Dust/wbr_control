/* SPDX-License-Identifier: Apache-2.0 */

#include <protocols/pc_link/pc_link.h>

#include <errno.h>
#include <string.h>

namespace protocols {

int EncodePcFrame(uint16_t cmd_id,
		const uint8_t *payload,
		size_t payload_len,
		uint8_t *out,
		size_t out_capacity,
		size_t *out_len)
{
	if ((out == nullptr) || (out_len == nullptr)) {
		return -EINVAL;
	}

	if ((payload_len > 0U) && (payload == nullptr)) {
		return -EINVAL;
	}

	if (payload_len > 0xffffU) {
		return -E2BIG;
	}

	const size_t total_len = 7U + payload_len;
	if (out_capacity < total_len) {
		return -ENOSPC;
	}

	out[0] = kFrameSof;
	out[1] = static_cast<uint8_t>(payload_len & 0xffU);
	out[2] = static_cast<uint8_t>((payload_len >> 8) & 0xffU);
	out[3] = 0U;
	out[4] = 0U;
	out[5] = static_cast<uint8_t>(cmd_id & 0xffU);
	out[6] = static_cast<uint8_t>((cmd_id >> 8) & 0xffU);
	if (payload_len > 0U) {
		memcpy(&out[7], payload, payload_len);
	}

	*out_len = total_len;
	return 0;
}

int DecodePcFrame(const uint8_t *frame, size_t frame_len, PcFrame *out)
{
	if ((frame == nullptr) || (out == nullptr) || (frame_len < 7U)) {
		return -EINVAL;
	}

	if (frame[0] != kFrameSof) {
		return -EBADMSG;
	}

	const uint16_t payload_len = static_cast<uint16_t>(frame[1]) |
				    (static_cast<uint16_t>(frame[2]) << 8);
	const size_t expect_len = 7U + payload_len;
	if (frame_len < expect_len) {
		return -EMSGSIZE;
	}

	out->cmd_id = static_cast<uint16_t>(frame[5]) | (static_cast<uint16_t>(frame[6]) << 8);
	out->payload = &frame[7];
	out->payload_len = payload_len;
	return 0;
}

}  // namespace protocols
