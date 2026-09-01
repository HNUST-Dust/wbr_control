# UART0 asynchronous TX test

This isolated test validates both UART0 output paths used on the HPM6750:

1. a polling boot message;
2. continuous Zephyr asynchronous `uart_tx()` through UART0 XDMA TX channel 1.

USB is disabled in the devicetree overlay. The asynchronous test transmits a
readable 112-byte frame every 10 ms at 921600 baud, matching the current
oscilloscope frame size and period.

## Build

From the workspace root:

```sh
west build -p always -b dust-hpm6750 \
  -s wbr_control/test/uart0_async_tx_test \
  -d wbr_control/test/uart0_async_tx_test/build
```

## Expected UART0 output

```text
UART0 poll path OK; starting async XDMA TX
UART0 async abort/recovery OK
UART0 async DMA seq=1 done=0 abort=1 ........................................
UART0 async DMA seq=2 done=1 abort=1 ........................................
```

If only the first line appears, the UART0 polling path works but asynchronous
XDMA completion is failing. A timeout path attempts to print
`UART0 async TX TIMEOUT` using polling output.
