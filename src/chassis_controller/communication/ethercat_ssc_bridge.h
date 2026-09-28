/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <chassis_controller/communication/ethercat_slave_core.h>

namespace protocols::ethercat {

// Thin, license-independent boundary for Beckhoff SSC generated code.
// APPL_OutputMapping copies SM2 bytes into Process(), and APPL_InputMapping
// copies the resulting bytes back to SM3. memcpy is used so SSC buffers do not
// need native C++ alignment.
class SscProcessDataBridge {
public:
	explicit SscProcessDataBridge(SlaveCoreConfig config = {});
	int Process(const uint8_t *sm2, size_t sm2_size, uint8_t *sm3,
		    size_t sm3_size, uint64_t now_us, ChassisPort &chassis);
	void Reset(ChassisPort &chassis);

private:
	EthercatSlaveCore core_;
};

} // namespace protocols::ethercat
