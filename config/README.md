# RTT 诊断与 UART0 输出模式

默认构建将 Zephyr shell、`LOG_*` 和 `printk()` 统一输出到 SEGGER RTT，
同时保留 UART0 给 VOFA oscilloscope。`printk()` 先重定向 logger，再由
shell log backend 序列化到 RTT channel 0，避免多个输出者互相穿插。

RTT 上位机需要能够读写 SEGGER RTT control block 的调试器和工具。
连接后可在 RTT terminal 0 中查看日志并输入 shell 命令。固件使用
non-blocking skip 策略和 deferred logging；业务线程不等待日志缓冲区，
缓冲区满时优先丢弃旧日志，以免调试输出阻塞 1 kHz 控制链路。

板载 Vllink CMSIS-DAP 使用工作区中的 HPM OpenOCD RTT 补丁版。日常使用直接
运行以下命令；它会从 ELF 自动解析 `_SEGGER_RTT` 地址、启动 OpenOCD，并连接
RTT channel 0：

```sh
west rtt
```

按 `Ctrl+C` 会同时关闭 RTT terminal 和 OpenOCD。其他构建目录或端口可通过
参数指定：

```sh
west rtt -d build-rtt -p 9091
```

如需只启动 server、再从另一个终端连接：

```sh
west rtt --server-only
nc 127.0.0.1 9090
```

HPM6750 RTT uses the RISC-V Debug Module System Bus Access port. OpenOCD is
forced to use `sysbus` with identity address mapping, and the RTT control block
plus both channel-0 ring buffers are linked into non-cache SRAM. RTT polling
therefore does not halt or resume the CPU. It can still add a small amount of
SRAM/JTAG bus traffic, which should be considered for the most stringent
cycle-level measurements.

Channel 0 uses a 16 KiB target-to-host buffer and OpenOCD drains it every
10 ms. This accommodates verbose shell commands such as `kernel threads`
without switching to a blocking RTT mode; if the host is absent, diagnostic
output is still allowed to drop instead of stalling a control thread.

OpenOCD 与烧录/调试命令不能同时占用同一个 CMSIS-DAP；执行 `west flash` 或
`west debug` 前先用 `Ctrl+C` 退出 RTT server。

UART0 同时是 Zephyr console 和 VOFA oscilloscope 使用的物理串口，因此一次
构建只能选择一种 UART0 输出模式。基础 `prj.conf` 默认选择 VOFA
oscilloscope；RTT console 不占用 UART0。需要把文本诊断临时改回 UART0 时，
通过 `EXTRA_CONF_FILE` 合并本目录中的一个配置文件；这些 UART 诊断
片段会显式关闭 RTT shell，保持“仅该 UART0 模式”的原有语义。

以下命令均在 `wbr_control` 目录执行：

```sh
# 默认：UART0 仅 VOFA oscilloscope，诊断文本走 RTT
west build -p always -b dust-hpm6750 -d build

# 仅 printk()
west build -p always -b dust-hpm6750 -d build -- \
  -DEXTRA_CONF_FILE=config/printk.conf

# 仅 LOG_ERR/LOG_WRN/LOG_INF/LOG_DBG
west build -p always -b dust-hpm6750 -d build -- \
  -DEXTRA_CONF_FILE=config/log.conf

# 同时允许 printk() 和 LOG_*；printk() 会经 logger 统一输出
west build -p always -b dust-hpm6750 -d build -- \
  -DEXTRA_CONF_FILE=config/printk_log.conf

# 显式选择 VOFA-only（效果与当前基础 prj.conf 相同）
west build -p always -b dust-hpm6750 -d build-vofa -- \
  -DEXTRA_CONF_FILE=config/oscilloscope.conf
```

UART0 的设备树波特率为 921600。oscilloscope 模式输出二进制 JustFloat 数据，
不能与文本 console 混用。`CONFIG_LOG` 可能因 coredump 仍为 `y`；判断日志是否
占用 UART0，应查看 `CONFIG_LOG_BACKEND_UART`，而不是只看 `CONFIG_LOG`。

顶层 CMake 会检查 Kconfig 求解后的最终配置。任何模式与 Zephyr 原生
`CONSOLE`、`PRINTK`、`LOG_BACKEND_UART` 所有权不一致时都会直接停止构建，避免将
看似为 VOFA-only、实际仍混有文本输出的固件烧录到机器人。

VOFA-only 固件在没有任何业务采样时每秒发送一次三通道探针
`[6750, 921600, 10]`。AHRS 不再发布 VOFA/oscilloscope 数据；UART0 上后续出现的
业务波形只来自显式写入 `latest_oscilloscope_sample` 的其他控制模块。
