# UART10 RX test

This hardware test verifies the UART10 asynchronous DMA receive path used by
the remote-control receiver. It uses the production UART settings: 100000 baud,
8 data bits, even parity, and 2 stop bits (8E2).

From the workspace root:

```sh
west build -p always -b dust-hpm6750 \
  -s wbr_control/test/uart10_rx_test \
  -d wbr_control/test/uart10_rx_test/build
west flash -d wbr_control/test/uart10_rx_test/build
west rtt -d wbr_control/test/uart10_rx_test/build
```

Connect the signal source to UART10 RX and share ground with the board. RTT
prints every received chunk as hexadecimal bytes and reports cumulative
statistics once per second. Increasing `bytes` and `events` confirms reception;
`dropped` and `stopped` should remain zero.
