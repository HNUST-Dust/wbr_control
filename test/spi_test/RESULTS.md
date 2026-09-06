# SPI test on the replacement board — 2026-09-06

Built and flashed `dust-hpm6750` using the local CMSIS-DAP probe. RTT captured:

```text
ICM42688P-HXY INT1 + asynchronous SPI DMA test
SPI2=8000000 Hz CS=PA26 INT1=PB09 INT2=PB12(unused)
WHO_AM_I failed: rc=0 value=0xFF expected=0x6A
IMU initialization failed: -19
```

The SPI API completed, but the identity register did not match. Initialization
returned before DMA abort/recovery and streaming acquisition could run. This is
not a successful six-axis/DMA test. Power, CS/MISO wiring, assembly and sensor
identity remain to be checked; this capture alone does not identify which failed.
DMA buffers are non-cacheable. The observed VOFA values 6750/921600/10 are the
application's diagnostic probe, not measured angles.

The scheduler probe now runs at the lowest application priority so it cannot
starve the deferred RTT shell/log threads.
