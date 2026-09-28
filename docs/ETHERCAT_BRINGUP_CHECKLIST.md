# HPM5E EtherCAT bring-up and acceptance checklist

This checklist is the boundary between the hardware-independent implementation
in this repository and the first HPM5E board. Do not enable host MPC output on
the chassis until all gates through Shadow mode pass.

## 1. Freeze slave identity and generate licensed SSC artifacts

- Assign the final EtherCAT Vendor ID, Product Code and Revision Number.
- In Beckhoff SSC Tool, reproduce
  `docs/ethercat/WBR_ECAT_OBJECT_DICTIONARY_V1.csv` exactly.
- Map `0x1600`, then `0x1601`, to SM2: 72 bytes total.
- Map `0x1a00`, then `0x1a01`, to SM3: 96 bytes total.
- Generate SSC sources, ESI XML and the default SII image (`eeprom.h`).
- Confirm the generated SyncManager watchdog is enabled for SM2.
- Put the final identity values in the host commands; do not leave the example
  `vendor=0`, `product=1` values for an active-control test.

Acceptance evidence:

```text
ethercat slaves -v
ethercat pdos -p 0
ethercat cstruct -p 0
```

The identity and every PDO entry must match the protocol document before
continuing.

## 2. Wire the HPM5E application

- Copy the licensed SSC output into a board-owned application directory.
- Adapt `test/ethercat_slave_test/ssc_application_hooks.cpp.template` only at
  the generated hook signatures and chassis TODOs.
- Use the existing `modules::EthercatChassisPort`; the chassis loop already
  publishes coherent IMU/leg/support-force snapshots and consumes its bounded
  Active command at the VMC support-feed-forward boundary.
- Wire a deliberate local UI action (never a host command) to
  `EthercatChassisPort::Instance().SetOperatorEnabled(true)`. Wire its release
  or disable action to `SetOperatorEnabled(false)`.
- Do not remove the second `SetChassisEligible()` gate maintained by the
  chassis loop. Leaving Balance, stale IMU/feedback, protocol failure or
  watchdog fallback clears Active and revokes the operator gate.
- Shadow commands are already isolated to diagnostics and never reach VMC.
- Register PDI, SYNC0, SYNC1 and reset callbacks before starting the ESC.
- Use `k_cycle_get_64()` converted with `k_cyc_to_us_floor64()` (or a proven
  HPM hardware timer) for `now_us`; do not use a millisecond tick.

## 3. Load and persist SII safely

- Add a fixed 64 KiB `ethercat_eeprom_partition` to the final board DTS.
- Load the generated SII bytes through `EthercatSiiStore::Load()`.
- Register the store's read/write/reload callbacks with the HPM ESC driver.
- Allow `Commit(true)` only while the slave is INIT/PREOP and motor output is
  disabled. Never erase flash in OP, an ESC ISR or the chassis control thread.
- Power-cycle twice and verify identity/PDOs remain readable.

## 4. Electrical and state-machine bring-up

Keep the mini PC Wi-Fi connection for SSH. Connect only the dedicated I225-LM
port to the EtherCAT IN port.

```bash
systemctl is-active ethercat.service
lspci -nnk -s 58:00.0
ethercat master
watch -n 0.5 ethercat slaves
```

Required result for the native path: I225-LM uses `ec_igc`, link is UP, one
slave reaches OP, and the AL status code is zero. First use FreeRun. Enable
SYNC0/DC only after FreeRun is stable.

If OP fails, capture before changing firmware:

```bash
ethercat slaves -v
ethercat pdos -p 0
ethercat debug 1
journalctl -k -b --no-pager | tail -n 300
ethercat debug 0
```

## 5. Protocol and fail-safe acceptance

Run these checks before performance comparison:

- ABI version is `0x57425201` in both directions.
- WKC is continuously 3 after warm-up.
- Echoed sequence and payload CRC remain error-free for at least ten minutes.
- Unplugging the cable while Active immediately restores local control.
- Freezing the host command sequence does not refresh the slave watchdog.
- Suspending the SSC task while the chassis loop still runs also restores local
  control through the independent chassis-side command-age watchdog.
- A stale state sequence, NaN force, non-converged flag, unknown mode, invalid
  ABI or command age over 30 ms restores local control.
- Requested forces are clamped to 20..150 N and limited to 30 N per accepted
  command with the current default configuration.
- Reconnecting does not automatically re-enter Active; the operator gate must
  be asserted again.

## 6. Compare SOEM, IgH generic and IgH native

Run from the deployed companion source directory on the mini PC. Use the same
cable, slave firmware, CPU, priority, duration and period for all paths.

```bash
sudo DURATION_S=600 PERIODS_US="1000 500 250" \
  ./scripts/run_ethercat_matrix.sh VENDOR_ID PRODUCT_CODE \
  ~/wbr_realtime_lab/results/first-board enp88s0
./scripts/summarize_ethercat_results.py \
  ~/wbr_realtime_lab/results/first-board \
  --output ~/wbr_realtime_lab/results/first-board/summary.md
```

Retain all CSV, JSON, stress logs and `summary.md`. Reject a path/period
combination if it has any WKC, exchange, echo or CRC failure. Compare
p99.9/p99.99/max values, not only the mean. The matrix script restores native
on normal exit, error, SIGINT and SIGTERM. If the machine itself loses power,
restore it explicitly after reboot:

```bash
sudo ./scripts/select_ethercat_path.sh native enp88s0
```

The matrix deliberately keeps all three paths in FreeRun. After that comparison
passes, run a separate IgH native DC/SYNC0 measurement (do not merge it into the
FreeRun rows):

```bash
sudo ./scripts/select_ethercat_path.sh native enp88s0
sudo ./build/ethercat_bench --backend igh --dc --path-label native-dc \
  --load-label idle --vendor VENDOR_ID --product PRODUCT_CODE \
  --period-us 1000 --duration-s 600 --warmup 1000 --cpu 2 --priority 80 \
  --csv native-dc-idle-1000us.csv --json native-dc-idle-1000us.json
```

SOEM DC is intentionally rejected by this benchmark because its cyclic phase
controller has not been implemented. Silently calling `ec_configdc()` without
phase-locking the host loop would not be a fair DC comparison.

## 7. MPC staged release

1. Run `ethercat_mpc` in Shadow mode for at least ten minutes.
2. Compare host force output against the local controller without feeding VMC.
3. Test cable pull, host process kill, solver failure and stale sensor flags.
4. Check that each case selects local control within the agreed safety bound.
5. Only then run Active with both command-line and local operator gates:

```bash
sudo ./build/ethercat_mpc --backend igh \
  --vendor VENDOR_ID --product PRODUCT_CODE \
  --mode active --allow-active --period-us 1000 --mpc-divisor 10 \
  --duration-s 60 --cpu 2 --priority 80 --csv active-validation.csv
```

Start with wheels/legs unloaded or mechanically restrained. CPU isolation and
BIOS power tuning belong after this baseline so their effect is measurable.
