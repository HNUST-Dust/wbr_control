/* SPDX-License-Identifier: Apache-2.0 */

#ifndef WBR_CONTROL_PROTOCOLS_PC_LINK_PC_LINK_H_
#define WBR_CONTROL_PROTOCOLS_PC_LINK_PC_LINK_H_

#include <stddef.h>
#include <stdint.h>

namespace protocols {

constexpr uint8_t kFrameSof = 0xa5U;

struct PcFrameHeader {
	uint8_t sof;
	uint16_t data_length;
	uint8_t seq;
	uint8_t reserved;
};

struct PcFrame {
	uint16_t cmd_id;
	const uint8_t *payload;
	uint16_t payload_len;
};

int EncodePcFrame(uint16_t cmd_id,
		const uint8_t *payload,
		size_t payload_len,
		uint8_t *out,
		size_t out_capacity,
		size_t *out_len);

int DecodePcFrame(const uint8_t *frame, size_t frame_len, PcFrame *out);

}  // namespace protocols

#endif /* WBR_CONTROL_PROTOCOLS_PC_LINK_PC_LINK_H_ */
