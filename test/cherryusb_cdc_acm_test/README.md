# CherryUSB CDC ACM Perf Test

This test ports the HPMicro CherryUSB CDC ACM virtual COM sample into the
`wbr_control/test` tree and adds simple speed/latency measurement modes.

## Build

From the workspace root:

```sh
west build -p always -b dust-hpm6750 wbr_control/test/cherryusb_cdc_acm_test
```

## Hardware

- Connect the board debug/power USB cable.
- Connect the board USB0 port to the host PC.

## Expected Result

After flashing and booting, the host should enumerate a USB CDC ACM serial
device.

## Performance Tests

Use the host helper from the workspace root. Replace the serial port with the
enumerated CDC ACM device, for example `/dev/tty.usbmodemXXXX` on macOS or
`/dev/ttyACM0` on Linux.

The helper uses pyserial:

```sh
python3 -m pip install pyserial
```

```sh
python3 wbr_control/test/cherryusb_cdc_acm_test/tools/usb_cdc_perf.py /dev/tty.usbmodemXXXX ping
python3 wbr_control/test/cherryusb_cdc_acm_test/tools/usb_cdc_perf.py /dev/tty.usbmodemXXXX latency --count 200
python3 wbr_control/test/cherryusb_cdc_acm_test/tools/usb_cdc_perf.py /dev/tty.usbmodemXXXX rx --bytes 4194304
python3 wbr_control/test/cherryusb_cdc_acm_test/tools/usb_cdc_perf.py /dev/tty.usbmodemXXXX tx --bytes 4194304
```

- `ping`: host sends a command and expects `PONG`.
- `latency`: host repeatedly sends `$PING` commands and waits for `PONG`. The
  reported value is host-observed USB CDC command round-trip time.
- `rx`: host sends a byte stream to the device. The device discards data and
  returns `RX_DONE <bytes> <device_elapsed_ms>`.
- `tx`: device sends a byte stream to the host. The host reports receive
  throughput.

The CDC line baud rate is configured by the host for driver compatibility, but
USB CDC bulk transfer speed is not limited by that baud value.

## HPM OUT Queue Note

The HPM CherryUSB device port used by this test tracks one OUT transfer buffer
per endpoint. The test therefore arms a single OUT read at a time and re-arms it
from the OUT completion callback.

Do not submit two `usbd_ep_start_read()` calls for the same OUT endpoint before
the first completion. The current HPM path overwrites the endpoint transfer
state, and the CherryUSB endpoint callback reports only `nbytes`, not the
completed buffer pointer or request context. RX throughput is lower than TX on
this baseline because the OUT path is correctness-first single queue behavior.

## Experimental Double OUT

`src/cdc_acm.c` contains a disabled experiment switch:

```c
#define PERF_EXPERIMENTAL_DOUBLE_OUT 0
```

Changing it to `1` makes only `rx` mode attempt two in-flight OUT reads. Command,
echo, latency, and tx paths remain on the single-read baseline. When enabled, the
device reports:

```text
RX_READY <bytes> DOUBLE_OUT
RX_DONE <bytes> <device_elapsed_ms> submit=<n> complete=<n> err=<n>
```

This mode is intentionally not the default. It is useful for reproducing whether
the HPM CherryUSB port can tolerate double OUT submission, but it may hang, lose
data, or report misleading throughput because the current port does not return a
completed buffer pointer/context.
