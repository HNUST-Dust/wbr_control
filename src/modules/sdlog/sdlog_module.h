/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <cstdint>

#include <modules/module_base.h>

namespace modules {

/** Low-priority FAT32 logger. It never runs in a control or sensor thread. */
class SdLogModule final : public ModuleBase {
public:
	int Start() override;
	void RunLoop() override;
};

} // namespace modules
