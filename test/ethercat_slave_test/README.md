# HPM5E EtherCAT slave test application integration

The repository contains all license-independent parts of the slave test:

- HPM5E ESC device driver and interrupt callbacks;
- 72-byte SM2 / 96-byte SM3 process-data ABI;
- benchmark echo, CRC and timing fields;
- MPC command validation, force limiting and local fallback;
- an alignment-safe SSC process-data bridge;
- host-native and Zephyr protocol tests.

Beckhoff SSC generated sources are deliberately not committed. HPM SDK also
omits them for the same licensing reason. Before the hardware integration build:

1. obtain SSC Tool under the applicable Beckhoff/ETG license;
2. create the objects and PDOs listed in
   `docs/ethercat/WBR_ECAT_OBJECT_DICTIONARY_V1.csv`;
3. select two RxPDOs on SM2 and two TxPDOs on SM3 in the documented order;
4. generate SSC sources, ESI XML and `eeprom.h` using the final Vendor ID,
   Product Code and Revision;
5. copy `ssc_application_hooks.cpp.template` into the generated application and
   replace its generated application mapping hooks;
6. use the provided `modules::EthercatChassisPort`; wire an explicit local UI
   action to `SetOperatorEnabled(true/false)` (it defaults false after boot);
7. register `PDI_Isr`, `Sync0_Isr`, `Sync1_Isr` and reset callbacks with
   `hpm_ethercat_set_callbacks()` before starting the ESC;
8. register EEPROM callbacks before `hpm_ethercat_start()`.

## Flash-backed SII setup

`EthercatSiiStore` keeps the live SII in RAM so ESC reads/writes never erase
flash in the realtime path. `EthercatSiiFlashBackend` uses the board's
`ethercat_eeprom_partition` as two power-fail-safe slots. Each slot has a CRC,
generation and final commit marker.

Application setup outline:

```cpp
platform::storage::EthercatSiiFlashBackend flash;
flash.Open();
protocols::ethercat::EthercatSiiStore sii(flash);
sii.Load(generated_eeprom_bytes, generated_eeprom_size);

hpm_ethercat_eeprom_ops ops{};
ops.read = protocols::ethercat::EthercatSiiStore::ReadCallback;
ops.write = protocols::ethercat::EthercatSiiStore::WriteCallback;
ops.reload = protocols::ethercat::EthercatSiiStore::ReloadCallback;
ops.user_data = &sii;
hpm_ethercat_set_eeprom_ops(esc, &ops);
```

Call `sii.Commit(true)` only from INIT/PREOP maintenance logic after outputs are
disabled. Never call it from OP, SYNC0, PDI ISR or the chassis control thread.
`Commit(false)` deliberately returns `-EPERM`, making the state check explicit.

The first firmware should use FreeRun and Shadow mode. Enable DC/SYNC0 only
after FreeRun reaches OP with WKC 3 and stable sequence echo. Active mode must
remain separately gated in the chassis state machine.

The concrete chassis bridge is already connected to the 1 kHz chassis loop.
It publishes a coherent IMU/leg/support-force sample, records Shadow commands,
and substitutes Active left/right support feed-forward at the existing VMC
boundary. Local leg-length PD/I and all joint torque limits remain active. An
Active command is applied only while the explicit operator gate is true and
the chassis is in an IMU-fresh, feedback-valid Balance state. Either gate
dropping atomically restores the local roll compensation path.

The template assumes SSC 5.13-style hook names used by the HPM SDK examples.
If a later SSC version changes a signature, keep the process-data sizes and
call order unchanged and adjust only the hook declarations.
