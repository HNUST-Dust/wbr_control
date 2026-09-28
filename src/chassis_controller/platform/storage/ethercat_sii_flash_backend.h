/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stddef.h>

#include <protocols/ethercat/ethercat_sii_store.h>

struct flash_area;

namespace platform::storage {

class EthercatSiiFlashBackend final : public protocols::ethercat::SiiStorageBackend {
public:
	~EthercatSiiFlashBackend() override;
	int Open();
	size_t Size() const override;
	int Read(size_t offset, void *data, size_t size) override;
	int Write(size_t offset, const void *data, size_t size) override;
	int Erase(size_t offset, size_t size) override;

private:
	const struct flash_area *area_ = nullptr;
};

} // namespace platform::storage
