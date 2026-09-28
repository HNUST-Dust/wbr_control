/* SPDX-License-Identifier: Apache-2.0 */

#include "ethercat_process_data.h"

namespace protocols::ethercat {

uint32_t PayloadCrc32(uint32_t sequence, uint32_t payload0, uint32_t payload1)
{
	const uint32_t words[] = {sequence, payload0, payload1};
	uint32_t crc = 0xffffffffU;
	for (uint32_t word : words) {
		for (unsigned int byte = 0U; byte < 4U; ++byte) {
			crc ^= (word >> (byte * 8U)) & 0xffU;
			for (unsigned int bit = 0U; bit < 8U; ++bit) {
				crc = (crc >> 1U) ^ (0xedb88320U & (0U - (crc & 1U)));
			}
		}
	}
	return ~crc;
}

} // namespace protocols::ethercat
