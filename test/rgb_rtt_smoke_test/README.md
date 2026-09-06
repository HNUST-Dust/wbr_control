# RGB + basic RTT smoke test

该镜像复用 RGB GPIO 程序，并使用与主工程一致的 RTT Shell、日志和 printk 配置，上行缓冲为 16 KB。

在 `wbr_control` 目录执行：

```sh
west build -p always -b dust-hpm6750 \
  -d build-rgb-rtt-smoke test/rgb_rtt_smoke_test
west flash -d build-rgb-rtt-smoke
west rtt -d build-rgb-rtt-smoke
```

LED 节奏应与 `rgb_led_smoke_test` 完全相同，RTT 中应持续出现
`RGB phase: ...`。RTT 同时提供 Shell 命令提示符。

`west rtt` 默认通过 HPM6750 的 System Bus Access 在 CPU 运行时读取 RTT，不会停核。
板级链接脚本会将 RTT 控制块和默认缓冲区放入不可缓存区，使 CPU 与调试器看到一致的
读写指针。若只为故障诊断需要短暂停核轮询，可显式使用 `west rtt --halt-polling`。
