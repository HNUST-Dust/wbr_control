# Device Adapters

这里放应用自己维护的设备封装，而不是 Zephyr 通用驱动。

命名约定：

- 用设备角色命名，而不是沿用旧 `dvc_*` 前缀
- 尽量表达设备语义，例如 `dji_motor`、`serial_servo`
- 设备封装负责管理具体器件协议、状态缓存和命令收发
