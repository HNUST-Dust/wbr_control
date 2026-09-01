# SDHC Performance Test

Build for `dust-hpm6750`. This test carries its own board overlay and enables
the SDMMC disk backend in `prj.conf`, so `-S sdhc` is not required.

```sh
west build -p always -b dust-hpm6750 wbr_control/test/sdhc_perf_test
```

The disk name is `SDMMC` (`CONFIG_SDMMC_VOLUME_NAME`). It is the Zephyr disk
driver name, not the filesystem volume label such as `WBR`.

The test initializes the SDMMC disk, prints card geometry, then measures:

- sequential read throughput
- sequential write throughput with backup, verify, and restore
- single-block read/write latency
- random single-block read IOPS

The write test uses a window near the end of the card and restores the original
contents after measurement. Use a noncritical card while bringing up hardware.
