# Coredump debugging

The production configuration stores a minimal Zephyr coredump in a dedicated
128 KiB flash partition. The dump contains the exception registers, faulting
thread object, and faulting thread stack. It survives a normal reset or power
cycle and does not overlap the 128 KiB `storage` partition used by Settings and
NVS.

## Flash layout

| Partition | Offset | Size |
| --- | ---: | ---: |
| `storage` | `0x363000` | 128 KiB |
| `coredump-partition` | `0x383000` | 128 KiB |

Do not erase or reprogram the complete flash before exporting a dump. Keep the
exact `build/zephyr/zephyr.elf` that produced the firmware; a different ELF can
give incorrect symbols and stack frames.

## Check and export a stored dump

Connect with `west rtt`, then use:

```text
coredump find
coredump verify
coredump print
```

To capture the printable dump on macOS, start the RTT session through
`script` before running those commands:

```sh
script -q coredump.log west rtt
```

After `coredump print` finishes, exit `west rtt` with Ctrl-C, then convert the
captured `#CD:` records to a binary coredump:

```sh
python ../zephyr/scripts/coredump/coredump_serial_log_parser.py \
  coredump.log coredump.bin
```

Start the RISC-V debugger with the matching ELF:

```sh
../../zephyr-sdk-0.16.5/riscv64-zephyr-elf/bin/\
riscv64-zephyr-elf-gdb build/zephyr/zephyr.elf
```

At the GDB prompt:

```gdb
target remote | ../zephyr/scripts/coredump/coredump_gdbserver.py --pipe build/zephyr/zephyr.elf coredump.bin
info registers
bt
```

Only after saving and inspecting the dump, erase it from the RTT shell with:

```text
coredump erase
```

`coredump error get` reports a failure encountered by the flash backend;
`coredump error clear` clears that status.

## What triggers a dump

Zephyr invokes coredump handling for fatal errors such as CPU exceptions,
assertion failures, explicit kernel panic, and detected stack overflows. A pure
deadlock, interrupt storm, externally asserted reset, power loss, or hardware
watchdog reset does not execute the fatal-error path and therefore cannot
create a coredump by itself. For those cases, inspect the running target with
OpenOCD or arrange for a software/task watchdog timeout to enter a controlled
fatal path before the hardware watchdog resets the MCU.
