# dust-hpm6750 RGB LED smoke test

这个测试初始化 Zephyr 内核、RGB GPIO 和与主工程一致的 RTT 诊断。UART 节点由 overlay 禁用，UART0 不输出文本。

在 `wbr_control` 目录执行：

```sh
west build -p always -b dust-hpm6750 \
  -d build-rgb-led-smoke test/rgb_led_smoke_test
west flash -d build-rgb-led-smoke
```

正常时会一直重复以下节奏：

1. 熄灭 0.25 秒
2. 红灯 0.75 秒
3. 绿灯 0.75 秒
4. 蓝灯 0.75 秒
5. 白灯（RGB 全亮）0.75 秒
6. 熄灭 0.75 秒

冷启动和每次按复位键后，序列都应从短暂熄灭、红灯开始，并持续循环。
