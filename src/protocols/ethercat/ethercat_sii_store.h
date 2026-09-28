/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace protocols::ethercat {

constexpr size_t kMaximumSiiImageSize = 16384U;

class SiiStorageBackend {
public:
	virtual ~SiiStorageBackend() = default;
	virtual size_t Size() const = 0;
	virtual int Read(size_t offset, void *data, size_t size) = 0;
	virtual int Write(size_t offset, const void *data, size_t size) = 0;
	virtual int Erase(size_t offset, size_t size) = 0;
};

// Power-fail-safe two-slot SII store. ESC word writes only update the RAM image;
// Commit() must be called explicitly from a non-OP maintenance state.
class EthercatSiiStore {
public:
	explicit EthercatSiiStore(SiiStorageBackend &backend);
	int Load(const uint8_t *default_image, size_t default_image_size);
	int ReadWords(uint32_t word_address, uint16_t *data, size_t word_count) const;
	int WriteWord(uint32_t word_address, uint16_t data);
	int ReloadData(uint32_t *data) const;
	int Commit(bool writes_allowed);

	bool dirty() const { return dirty_; }
	size_t image_size() const { return image_size_; }
	uint32_t generation() const { return generation_; }
	const uint8_t *image() const { return image_; }

	static int ReadCallback(uint32_t word_address, uint16_t *data,
				size_t word_count, void *user_data);
	static int WriteCallback(uint32_t word_address, uint16_t data, void *user_data);
	static int ReloadCallback(uint32_t *data, void *user_data);

private:
	struct SlotHeader {
		uint32_t magic;
		uint16_t format_version;
		uint16_t header_size;
		uint32_t generation;
		uint32_t image_size;
		uint32_t image_crc32;
		uint32_t header_crc32;
		uint32_t committed;
	};
	static_assert(sizeof(SlotHeader) == 28U, "SII slot header layout changed");

	bool ValidateSlot(unsigned int slot, SlotHeader &header);
	int ReadSlotImage(unsigned int slot, const SlotHeader &header);
	size_t SlotSize() const;
	size_t SlotOffset(unsigned int slot) const;
	static uint32_t Crc32(const uint8_t *data, size_t size);
	static bool GenerationNewer(uint32_t candidate, uint32_t current);

	SiiStorageBackend &backend_;
	alignas(8) uint8_t image_[kMaximumSiiImageSize]{};
	size_t image_size_ = 0U;
	uint32_t generation_ = 0U;
	int active_slot_ = -1;
	bool dirty_ = false;
};

} // namespace protocols::ethercat
