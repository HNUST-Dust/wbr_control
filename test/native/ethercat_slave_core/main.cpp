#include <cassert>
#include <cstdint>
#include <iostream>
#include <cstring>
#include <cerrno>

#include <chassis_controller/communication/ethercat_slave_core.h>
#include <chassis_controller/communication/ethercat_ssc_bridge.h>
#include <protocols/ethercat/ethercat_sii_store.h>

namespace ec = protocols::ethercat;

class FakeChassis final : public ec::ChassisPort {
 public:
  ec::ChassisSample ReadChassisSample() override { return sample; }
  void HandleMpcCommand(const ec::ChassisMpcCommand& value) override {
    command = value;
    ++commands;
  }
  void EnterLocalFallback() override { ++fallbacks; }

  ec::ChassisSample sample{};
  ec::ChassisMpcCommand command{};
  std::uint32_t commands = 0U;
  std::uint32_t fallbacks = 0U;
};

class MemorySiiBackend final : public ec::SiiStorageBackend {
 public:
  MemorySiiBackend() { std::memset(bytes, 0xff, sizeof(bytes)); }
  std::size_t Size() const override { return sizeof(bytes); }
  int Read(std::size_t offset, void* data, std::size_t size) override {
    if (offset > sizeof(bytes) || size > sizeof(bytes) - offset) return -ERANGE;
    std::memcpy(data, bytes + offset, size);
    return 0;
  }
  int Write(std::size_t offset, const void* data, std::size_t size) override {
    if (offset > sizeof(bytes) || size > sizeof(bytes) - offset) return -ERANGE;
    const auto* source = static_cast<const std::uint8_t*>(data);
    for (std::size_t i = 0; i < size; ++i) {
      if ((bytes[offset + i] & source[i]) != source[i]) return -EIO;
      bytes[offset + i] = source[i];
    }
    return 0;
  }
  int Erase(std::size_t offset, std::size_t size) override {
    if (offset > sizeof(bytes) || size > sizeof(bytes) - offset) return -ERANGE;
    std::memset(bytes + offset, 0xff, size);
    return 0;
  }
  std::uint8_t bytes[65536];
};

int main() {
  assert(ec::PayloadCrc32(1U, 2U, 3U) == 0xb0e02293U);
  ec::EthercatSlaveCore core;
  FakeChassis chassis;
  ec::MasterProcessImage master{};
  ec::SlaveProcessImage slave{};
  master.benchmark.sequence = 7U;
  master.benchmark.payload0 = 11U;
  master.benchmark.payload1 = 13U;
  master.benchmark.payload_crc32 = ec::PayloadCrc32(7U, 11U, 13U);
  core.Process(master, 0U, chassis, slave);
  assert(slave.benchmark.echoed_sequence == 7U);
  assert((slave.benchmark.status & ec::kBenchmarkPayloadValid) != 0U);

  master.mpc.sequence = 1U;
  master.mpc.source_state_sequence = 1U;
  master.mpc.mode_and_flags = ec::kMpcActive | ec::kMpcConverged |
                              ec::kMpcInputFinite;
  master.mpc.left_support_force_n = 200.0F;
  master.mpc.right_support_force_n = 10.0F;
  master.mpc.valid_for_us = 50000U;
  core.Process(master, 1000U, chassis, slave);
  assert(chassis.commands == 1U);
  assert(chassis.command.left_support_force_n == 150.0F);
  assert(chassis.command.right_support_force_n == 20.0F);
  assert(chassis.command.valid_for_us == 30000U);

  core.Process(master, 2000U, chassis, slave);
  assert(chassis.commands == 1U);
  core.Process(master, 31001U, chassis, slave);
  assert(chassis.fallbacks == 1U);
  assert((slave.state.status_flags & ec::kStateCommandTimedOut) != 0U);

  // Unknown mode values must never reach the chassis interface. Start a new
  // valid active command, then prove that an invalid mode forces fallback.
  master.mpc.sequence = 2U;
  master.mpc.source_state_sequence = slave.state.sequence;
  core.Process(master, 32000U, chassis, slave);
  assert(chassis.commands == 2U);
  master.mpc.sequence = 3U;
  master.mpc.source_state_sequence = slave.state.sequence;
  master.mpc.mode_and_flags = 0x7fU | ec::kMpcConverged | ec::kMpcInputFinite;
  core.Process(master, 33000U, chassis, slave);
  assert(chassis.commands == 2U);
  assert(chassis.fallbacks == 2U);

  ec::SscProcessDataBridge bridge;
  std::uint8_t sm2[sizeof(ec::MasterProcessImage)]{};
  std::uint8_t sm3[sizeof(ec::SlaveProcessImage)]{};
  std::memcpy(sm2, &master, sizeof(master));
  assert(bridge.Process(sm2, sizeof(sm2), sm3, sizeof(sm3), 40000U, chassis) == 0);
  ec::SlaveProcessImage bridged{};
  std::memcpy(&bridged, sm3, sizeof(bridged));
  assert(bridged.benchmark.echoed_sequence == master.benchmark.sequence);
  assert(bridge.Process(sm2, sizeof(sm2) - 1U, sm3, sizeof(sm3), 40000U,
                        chassis) < 0);

  MemorySiiBackend backend;
  std::uint8_t default_sii[128]{};
  default_sii[0] = 0x34U;
  default_sii[1] = 0x12U;
  default_sii[2] = 0x78U;
  default_sii[3] = 0x56U;
  ec::EthercatSiiStore sii(backend);
  assert(sii.Load(default_sii, sizeof(default_sii)) == 0);
  assert(sii.dirty());
  assert(sii.Commit(false) == -EPERM);
  assert(sii.Commit(true) == 0);
  assert(sii.generation() == 1U);

  ec::EthercatSiiStore reloaded(backend);
  assert(reloaded.Load(nullptr, 0U) == 0);
  std::uint32_t reload_data = 0U;
  assert(reloaded.ReloadData(&reload_data) == 0);
  assert(reload_data == 0x56781234U);
  assert(reloaded.WriteWord(0U, 0xabcdU) == 0);
  assert(reloaded.Commit(true) == 0);
  assert(reloaded.generation() == 2U);

  // Simulate power loss before the second slot's final commit marker. The
  // next boot must select generation 1 from the other slot.
  std::memset(&backend.bytes[32768U + 24U], 0xff, 4U);
  ec::EthercatSiiStore recovered(backend);
  assert(recovered.Load(nullptr, 0U) == 0);
  assert(recovered.generation() == 1U);
  assert(recovered.ReloadData(&reload_data) == 0);
  assert(reload_data == 0x56781234U);
  std::cout << "EtherCAT slave core passed\n";
}
