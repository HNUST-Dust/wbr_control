# HI91 IMU UART2 Test

This isolated test validates HI91 IMU reception on UART2 before the driver is
merged into the main control application.

It configures UART2 for Zephyr async RX through HPM UART DMA plus the UART RX
timeout interrupt, parses HI91 frames, and outputs VOFA+ JustFloat data on
UART0. No TRGM/GPTMR idle-detect pin is required for this path.

VOFA channel order:

1. roll_deg
2. pitch_deg
3. yaw_deg
4. uart_rx_hz
5. frame_hz
6. sof_a55a_hz
7. sof_5aa5_hz
8. parse_error_count
9. rx_stop_count
10. rx_drop_count
11. h0, first byte after A5 5A
12. h1
13. h2
14. h3
15. h4
16. h5
17. h6
18. h7

Build:

```sh
west build -b dust-hpm6750/hpm6750 -d wbr_control/test/hi91_imu_test/build wbr_control/test/hi91_imu_test
```

Flash with the same method used by the other tests in this workspace.
