# UART0 输出模式

UART0 同时是 Zephyr console 和 VOFA oscilloscope 使用的物理串口，因此一次
构建只能选择一种输出模式。基础 `prj.conf` 默认选择仅 VOFA oscilloscope，
即关闭 UART console、printk、boot banner 和 UART log backend。需要切换到
其他诊断模式时，通过 `EXTRA_CONF_FILE` 合并本目录中的一个配置文件。

以下命令均在 `wbr_control` 目录执行：

```sh
# 默认：仅 VOFA oscilloscope
west build -p always -b hpm6750evk2 -d build

# 仅 printk()
west build -p always -b hpm6750evk2 -d build -- \
  -DEXTRA_CONF_FILE=config/printk.conf

# 仅 LOG_ERR/LOG_WRN/LOG_INF/LOG_DBG
west build -p always -b hpm6750evk2 -d build -- \
  -DEXTRA_CONF_FILE=config/log.conf

# 同时允许 printk() 和 LOG_*；printk() 会经 logger 统一输出
west build -p always -b hpm6750evk2 -d build -- \
  -DEXTRA_CONF_FILE=config/printk_log.conf

# 显式选择 VOFA-only（效果与当前基础 prj.conf 相同）
west build -p always -b hpm6750evk2 -d build-vofa -- \
  -DEXTRA_CONF_FILE=config/oscilloscope.conf
```

UART0 的设备树波特率为 921600。oscilloscope 模式输出二进制 JustFloat 数据，
不能与文本 console 混用。`CONFIG_LOG` 可能因 coredump 仍为 `y`；判断日志是否
占用 UART0，应查看 `CONFIG_LOG_BACKEND_UART`，而不是只看 `CONFIG_LOG`。
