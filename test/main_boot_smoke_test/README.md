# Main boot-path smoke test

这是用于故障二分定位的主工程变体。它使用真正的 `src/main.cpp`、RTT 和
`SysStateModule`，但不启动 CAN、USB、串口、SPI 及其他业务模块。

在 `wbr_control` 目录执行：

```sh
west build -p always -b dust-hpm6750 -d build-main-boot-smoke . \
  -- -DCONF_FILE=test/main_boot_smoke_test/boot_smoke.conf \
     -DDTC_OVERLAY_FILE=test/main_boot_smoke_test/dust-hpm6750.overlay
west flash -d build-main-boot-smoke
```

预期现象是 RGB 灯持续缓慢呼吸并在红、绿、蓝之间切换，同时 `west rtt`
能够进入。若两者正常，故障位于本测试关闭的生产外设或业务模块中。
