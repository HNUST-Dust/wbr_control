# can_test

用于在 Zephyr 上验证 CAN 收发链路是否正常。

测试内容：
- 优先使用 devicetree alias `canbus` 指向的 CAN 设备。
- 设为 `CAN_MODE_NORMAL`，bitrate 固定为 500 kbit/s。
- 可选拉起 alias `can-transceiver-enable` 指向的 GPIO，用来把外部 CAN 收发器拉出 standby/shutdown。
- 添加标准帧过滤器（ID=`0x123`）。
- 每 500 ms 发送 1 帧标准 CAN 数据，并打印 TX/RX/state/error counter。

## 构建示例

```bash
cmake -S wbr_control/test/can_test -B wbr_control/test/can_test/build -GNinja -DBOARD=dust-hpm6750
cmake --build wbr_control/test/can_test/build -j8
```

## 收发器 Enable GPIO

如果收发器有 EN/STB/SHDN 管脚且没有硬件默认使能，需要在 board overlay 中添加一个 GPIO 节点，并把 alias 命名为 `can-transceiver-enable`：

```dts
/ {
	aliases {
		canbus = &can1;
		can-transceiver-enable = &can_xcvr_en;
	};

	can_xcvr_en: can-xcvr-en {
		gpios = <&gpioe 26 GPIO_ACTIVE_HIGH>;
	};
};
```

把 `gpioe 26` 和 `GPIO_ACTIVE_HIGH` 改成实际原理图上的管脚和有效电平。若接的是 STB 管脚且高电平表示待机，通常应写成 `GPIO_ACTIVE_LOW`，这样测试程序“assert enable”时会输出物理低电平。

## 预期日志

- 正常发送：持续打印 `[TX] seq=... id=0x123 dlc=8`。
- 无 ACK/总线未接好：可能打印 `tx completion timeout`，随后 state/error counter 会继续输出。
- 收发器未配置 GPIO：会打印 `no can-transceiver-enable alias; assuming transceiver is hardware-enabled`。
