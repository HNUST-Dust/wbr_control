/* SPDX-License-Identifier: Apache-2.0 */

#include "ethercat_sii_flash_backend.h"

#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/storage/flash_map.h>

namespace platform::storage {

EthercatSiiFlashBackend::~EthercatSiiFlashBackend()
{
	if (area_ != nullptr) flash_area_close(area_);
}

int EthercatSiiFlashBackend::Open()
{
	if (area_ != nullptr) return 0;
#if DT_NODE_EXISTS(DT_NODELABEL(ethercat_eeprom_partition))
	return flash_area_open(FIXED_PARTITION_ID(ethercat_eeprom_partition), &area_);
#else
	return -ENODEV;
#endif
}

size_t EthercatSiiFlashBackend::Size() const
{
	return area_ != nullptr ? area_->fa_size : 0U;
}

int EthercatSiiFlashBackend::Read(size_t offset, void *data, size_t size)
{
	if (area_ == nullptr) return -ENODEV;
	return flash_area_read(area_, static_cast<off_t>(offset), data, size);
}

int EthercatSiiFlashBackend::Write(size_t offset, const void *data, size_t size)
{
	if (area_ == nullptr) return -ENODEV;
	return flash_area_write(area_, static_cast<off_t>(offset), data, size);
}

int EthercatSiiFlashBackend::Erase(size_t offset, size_t size)
{
	if (area_ == nullptr) return -ENODEV;
	return flash_area_erase(area_, static_cast<off_t>(offset), size);
}

} // namespace platform::storage
