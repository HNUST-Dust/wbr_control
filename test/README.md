# Tests

The directory is split into two kinds of tests:

- `unit/` and `onboard_imu_filter_test/` are automated Ztest suites. They run
  on `qemu_riscv32` and do not require the HPM6750 board.
- The remaining applications are explicit hardware integration or calibration
  tests. They are built and flashed individually when that hardware path needs
  verification.

## Automated tests

Run all automated suites with Twister:

```sh
CCACHE_DISABLE=1 ./zephyr/scripts/twister \
  -T wbr_control/test/unit \
  -T wbr_control/test/onboard_imu_filter_test \
  -p qemu_riscv32
```

Zephyr 3.7 still imports `pkg_resources`. If Twister reports that module as
missing, install `setuptools<82` in the active virtual environment; direct
`west build` and `-t run` do not depend on it.

To build and run one suite without Twister:

```sh
CCACHE_DISABLE=1 west build -p always -b qemu_riscv32 \
  -d /tmp/wbr-protocols-test wbr_control/test/unit/protocols
CCACHE_DISABLE=1 west build -d /tmp/wbr-protocols-test -t run
```

`unit/littlefs_service` demonstrates FFF against production code. It replaces
the filesystem and flash APIs, then verifies return sequences, arguments, call
counts, call order, and idempotent initialization.

## Hardware tests kept intentionally

- `can_test`: CAN controller/transceiver bring-up.
- `cherryusb_interrupt_test`: current USB interrupt endpoint transport.
- `leg_feedforward_test`: guarded wheel-leg force calibration.
- `sdhc_perf_test`: destructive/restore SDHC performance characterization.
- `spi_test`: ICM42688P SPI/DMA integration.
- `uart0_async_tx_test`: UART0 asynchronous DMA recovery.
- `wheel_current_mapping_test`: guarded C620 current mapping calibration.

## Diagnostic output policy

All HPM hardware tests include `common/diagnostics.cmake`, which appends
`common/rtt.conf`: LOG and printk use the RTT shell backend, UART console and
UART logging are disabled, and the RTT up buffer is 16384 bytes as in the main
application. UART0 is reserved for binary VOFA JustFloat frames at 921600 baud;
tests without telemetry leave it unused. QEMU suites retain their harness console.
New test applications must include this CMake file before `find_package(Zephyr)`.

Run the SPI diagnostic from the workspace root:

```sh
west build -p always -b dust-hpm6750 -s wbr_control/test/spi_test -d wbr_control/test/spi_test/build
west flash -d wbr_control/test/spi_test/build
west rtt -d wbr_control/test/spi_test/build
```

Expect WHO_AM_I=0x6A, a successful DMA abort/recovery self-test, and increasing
sample/DRDY/callback counts with raw accelerometer and gyroscope values on RTT.
