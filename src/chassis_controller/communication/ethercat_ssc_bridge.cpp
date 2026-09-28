/* SPDX-License-Identifier: Apache-2.0 */

#include <chassis_controller/communication/ethercat_ssc_bridge.h>

#include <errno.h>
#include <string.h>

namespace protocols::ethercat {

SscProcessDataBridge::SscProcessDataBridge(SlaveCoreConfig config) : core_(config) {}

int SscProcessDataBridge::Process(const uint8_t *sm2, size_t sm2_size, uint8_t *sm3,
				  size_t sm3_size, uint64_t now_us, ChassisPort &chassis)
{
	if (sm2 == nullptr || sm3 == nullptr) return -EINVAL;
	if (sm2_size != sizeof(MasterProcessImage) || sm3_size != sizeof(SlaveProcessImage)) {
		return -EMSGSIZE;
	}
	MasterProcessImage master{};
	SlaveProcessImage slave{};
	memcpy(&master, sm2, sizeof(master));
	core_.Process(master, now_us, chassis, slave);
	memcpy(sm3, &slave, sizeof(slave));
	return 0;
}

void SscProcessDataBridge::Reset(ChassisPort &chassis)
{
	core_.Reset(chassis);
}

} // namespace protocols::ethercat
