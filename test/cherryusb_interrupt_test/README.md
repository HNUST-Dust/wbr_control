# CherryUSB Interrupt Endpoint Test

This sample exposes a vendor-specific USB interface with one interrupt OUT
endpoint and one interrupt IN endpoint.

It is intended for control-frame experiments:

- host sends one fixed-size frame on interrupt OUT
- device echoes a response on interrupt IN
- host measures round-trip latency and exchange rate

## Build

From the workspace root:

```sh
CCACHE_DISABLE=1 /Users/panpoming/Documents/zephyr_projects/.venv/bin/west build -p always -b dust-hpm6750 wbr_control/test/cherryusb_interrupt_test
```

Flash the generated `build/zephyr/zephyr.elf` using your usual HPM workflow.

## Host Tool

Install PyUSB:

```sh
python3 -m pip install pyusb
```

Run from this directory:

```sh
python3 tools/usb_interrupt_perf.py --size 64 ping
python3 tools/usb_interrupt_perf.py --size 64 latency --count 200
python3 tools/usb_interrupt_perf.py --size 64 exchange --count 1000
python3 tools/usb_interrupt_perf.py --size 64 stream-in --count 2000
python3 tools/usb_interrupt_perf.py --size 64 stream-out --count 2000
```

For high-speed interrupt endpoint experiments you can also try larger packets:

```sh
python3 tools/usb_interrupt_perf.py --size 1024 latency --count 200
python3 tools/usb_interrupt_perf.py --size 1024 stream-in --count 2000
```

Modes:

- `ping`: one OUT frame followed by one IN response.
- `latency`: repeated synchronous OUT/IN RTT test.
- `exchange`: repeated synchronous OUT/IN exchange-rate test.
- `stream-in`: device continuously submits interrupt IN frames; host reads and
  reports frame interval jitter.
- `stream-out`: host continuously writes interrupt OUT frames while the device
  sinks them and reports the final device-side count.

## Notes

The descriptor uses `wMaxPacketSize=1024` at high speed and `64` at full speed,
with `bInterval=1`.

This is not a CDC ACM serial device. The host tool uses the device VID/PID
directly through libusb/PyUSB:

```text
VID=0x34b7 PID=0xffff interface=0 OUT=0x01 IN=0x81
```

On macOS, PyUSB usually needs libusb installed through Homebrew:

```sh
brew install libusb
```
