/* SPDX-License-Identifier: Apache-2.0 */

#include "ethercat_sii_store.h"

#include <errno.h>
#include <string.h>

namespace protocols::ethercat {
namespace {

constexpr uint32_t kSlotMagic = 0x53494957U; // "WIIS" little-endian.
constexpr uint16_t kFormatVersion = 1U;
constexpr uint32_t kCommittedMarker = 0x434f4d54U; // "TMOC" little-endian.
constexpr uint32_t kErasedMarker = 0xffffffffU;

uint32_t Crc32Update(uint32_t crc, const uint8_t *data, size_t size)
{
	for (size_t i = 0U; i < size; ++i) {
		crc ^= data[i];
		for (unsigned int bit = 0U; bit < 8U; ++bit) {
			crc = (crc >> 1U) ^ (0xedb88320U & (0U - (crc & 1U)));
		}
	}
	return crc;
}

} // namespace

EthercatSiiStore::EthercatSiiStore(SiiStorageBackend &backend) : backend_(backend) {}

size_t EthercatSiiStore::SlotSize() const
{
	return backend_.Size() / 2U;
}

size_t EthercatSiiStore::SlotOffset(unsigned int slot) const
{
	return static_cast<size_t>(slot) * SlotSize();
}

uint32_t EthercatSiiStore::Crc32(const uint8_t *data, size_t size)
{
	return ~Crc32Update(0xffffffffU, data, size);
}

bool EthercatSiiStore::GenerationNewer(uint32_t candidate, uint32_t current)
{
	return candidate != current && static_cast<int32_t>(candidate - current) > 0;
}

bool EthercatSiiStore::ValidateSlot(unsigned int slot, SlotHeader &header)
{
	if (backend_.Read(SlotOffset(slot), &header, sizeof(header)) != 0) return false;
	if (header.magic != kSlotMagic || header.format_version != kFormatVersion ||
	    header.header_size != sizeof(SlotHeader) || header.committed != kCommittedMarker ||
	    header.image_size == 0U || header.image_size > kMaximumSiiImageSize ||
	    sizeof(SlotHeader) + header.image_size > SlotSize()) {
		return false;
	}
	const uint32_t expected_header_crc =
		Crc32(reinterpret_cast<const uint8_t *>(&header), offsetof(SlotHeader, header_crc32));
	if (header.header_crc32 != expected_header_crc) return false;

	uint8_t scratch[128];
	uint32_t crc = 0xffffffffU;
	size_t remaining = header.image_size;
	size_t offset = SlotOffset(slot) + sizeof(SlotHeader);
	while (remaining != 0U) {
		const size_t chunk = remaining < sizeof(scratch) ? remaining : sizeof(scratch);
		if (backend_.Read(offset, scratch, chunk) != 0) return false;
		crc = Crc32Update(crc, scratch, chunk);
		offset += chunk;
		remaining -= chunk;
	}
	return ~crc == header.image_crc32;
}

int EthercatSiiStore::ReadSlotImage(unsigned int slot, const SlotHeader &header)
{
	const int ret = backend_.Read(SlotOffset(slot) + sizeof(SlotHeader), image_,
				      header.image_size);
	if (ret != 0) return ret;
	image_size_ = header.image_size;
	generation_ = header.generation;
	active_slot_ = static_cast<int>(slot);
	dirty_ = false;
	return 0;
}

int EthercatSiiStore::Load(const uint8_t *default_image, size_t default_image_size)
{
	if (backend_.Size() < 2U * (sizeof(SlotHeader) + 1U)) return -ENOSPC;
	SlotHeader headers[2]{};
	const bool valid0 = ValidateSlot(0U, headers[0]);
	const bool valid1 = ValidateSlot(1U, headers[1]);
	if (valid0 || valid1) {
		const unsigned int selected = valid1 &&
			(!valid0 || GenerationNewer(headers[1].generation, headers[0].generation))
					      ? 1U
					      : 0U;
		return ReadSlotImage(selected, headers[selected]);
	}
	if (default_image == nullptr || default_image_size == 0U ||
	    default_image_size > kMaximumSiiImageSize ||
	    sizeof(SlotHeader) + default_image_size > SlotSize()) {
		return -EINVAL;
	}
	memcpy(image_, default_image, default_image_size);
	image_size_ = default_image_size;
	generation_ = 0U;
	active_slot_ = -1;
	dirty_ = true;
	return 0;
}

int EthercatSiiStore::ReadWords(uint32_t word_address, uint16_t *data,
				size_t word_count) const
{
	if (data == nullptr) return -EINVAL;
	const size_t offset = static_cast<size_t>(word_address) * 2U;
	const size_t size = word_count * 2U;
	if (word_count > kMaximumSiiImageSize / 2U || offset > image_size_ ||
	    size > image_size_ - offset) {
		return -ERANGE;
	}
	for (size_t i = 0U; i < word_count; ++i) {
		data[i] = static_cast<uint16_t>(image_[offset + i * 2U]) |
			  static_cast<uint16_t>(static_cast<uint16_t>(image_[offset + i * 2U + 1U])
					<< 8U);
	}
	return 0;
}

int EthercatSiiStore::WriteWord(uint32_t word_address, uint16_t data)
{
	const size_t offset = static_cast<size_t>(word_address) * 2U;
	if (offset > image_size_ || 2U > image_size_ - offset) return -ERANGE;
	image_[offset] = static_cast<uint8_t>(data);
	image_[offset + 1U] = static_cast<uint8_t>(data >> 8U);
	dirty_ = true;
	return 0;
}

int EthercatSiiStore::ReloadData(uint32_t *data) const
{
	if (data == nullptr) return -EINVAL;
	if (image_size_ < sizeof(*data)) return -ENODATA;
	*data = static_cast<uint32_t>(image_[0]) |
		(static_cast<uint32_t>(image_[1]) << 8U) |
		(static_cast<uint32_t>(image_[2]) << 16U) |
		(static_cast<uint32_t>(image_[3]) << 24U);
	return 0;
}

int EthercatSiiStore::Commit(bool writes_allowed)
{
	if (!writes_allowed) return -EPERM;
	if (!dirty_) return 0;
	if (image_size_ == 0U || sizeof(SlotHeader) + image_size_ > SlotSize()) return -ENOSPC;
	const unsigned int target = active_slot_ == 0 ? 1U : 0U;
	SlotHeader header{};
	header.magic = kSlotMagic;
	header.format_version = kFormatVersion;
	header.header_size = sizeof(SlotHeader);
	header.generation = generation_ + 1U;
	header.image_size = image_size_;
	header.image_crc32 = Crc32(image_, image_size_);
	header.header_crc32 =
		Crc32(reinterpret_cast<const uint8_t *>(&header), offsetof(SlotHeader, header_crc32));
	header.committed = kErasedMarker;
	const size_t base = SlotOffset(target);
	int ret = backend_.Erase(base, SlotSize());
	if (ret == 0) ret = backend_.Write(base, &header, sizeof(header));
	if (ret == 0) ret = backend_.Write(base + sizeof(header), image_, image_size_);
	if (ret == 0) {
		const uint32_t marker = kCommittedMarker;
		ret = backend_.Write(base + offsetof(SlotHeader, committed), &marker, sizeof(marker));
	}
	if (ret != 0) return ret;
	active_slot_ = static_cast<int>(target);
	generation_ = header.generation;
	dirty_ = false;
	return 0;
}

int EthercatSiiStore::ReadCallback(uint32_t word_address, uint16_t *data,
				   size_t word_count, void *user_data)
{
	if (user_data == nullptr) return -EINVAL;
	return static_cast<EthercatSiiStore *>(user_data)->ReadWords(word_address, data,
								    word_count);
}

int EthercatSiiStore::WriteCallback(uint32_t word_address, uint16_t data, void *user_data)
{
	if (user_data == nullptr) return -EINVAL;
	return static_cast<EthercatSiiStore *>(user_data)->WriteWord(word_address, data);
}

int EthercatSiiStore::ReloadCallback(uint32_t *data, void *user_data)
{
	if (user_data == nullptr) return -EINVAL;
	return static_cast<EthercatSiiStore *>(user_data)->ReloadData(data);
}

} // namespace protocols::ethercat
