# SDHC Performance Test

Build for `dust-hpm6750`. This test carries its own board overlay and enables
the SDMMC disk backend in `prj.conf`, so `-S sdhc` is not required.

```sh
west build -p always -b dust-hpm6750 wbr_control/test/sdhc_perf_test
```

The disk name is `SD` (`CONFIG_SDMMC_VOLUME_NAME`). It is the Zephyr disk
driver name and matches Zephyr's FatFs volume mapping.

The test initializes the SDMMC disk, prints card geometry, then measures:

- FAT filesystem mount
- file write/readback verification using `/SD:/PX4LOG.TXT`

- sequential read throughput
- sequential write throughput with backup, verify, and restore
- single-block read/write latency
- random single-block read IOPS

The write test uses a window near the end of the card and restores the original
contents after measurement. Use a noncritical card while bringing up hardware.

The card must already contain a readable FAT filesystem. This test never formats
the card; if formatting or reinserting the card is required, stop and perform
that physical operation manually before rerunning the test.

The RTT shell also exposes filesystem commands. After boot, use `help fs` to
see the commands, then access the mounted volume at `/SD:`.
