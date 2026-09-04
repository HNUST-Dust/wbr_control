@mainpage wbr_control 固件开发文档

# 项目简介

`wbr_control` 是运行在 Zephyr RTOS 上的轮腿机器人控制固件。代码按数据通道、应用模块、
线协议、平台适配和调度工具分层，控制线程通过快照通道交换数据，并通过平台层访问 CAN、
UART、USB、文件系统及执行器。

# 文档导航

- @ref wbr_channels "数据通道"：跨线程状态快照和原始帧队列。
- @ref wbr_modules "应用模块"：底盘、IMU、遥控、裁判系统及系统状态模块。
- @ref wbr_protocols "通信协议"：电机、遥控器、裁判系统、上位机及遥测协议。
- @ref wbr_platform "平台适配"：板级身份、通信驱动、执行器和文件系统服务。
- @ref wbr_scheduling "调度策略"：周期释放和线程优先级约定。
- @ref wbr_debug "调试功能"：Shell 和调试辅助功能。

# 核心约定

1. 时间戳来自单调时钟，字段名后缀 `_us` 和 `_ms` 分别表示微秒和毫秒。
2. 控制算法内部角度默认使用弧度，角速度默认使用弧度每秒；带 `_deg` 后缀的字段使用度。
3. 平台与协议接口成功返回 0，失败返回负 errno；布尔解码接口使用 `true` 表示完整帧有效。
4. UART、CAN 和 USB 回调只执行有界数据搬运，解析与控制计算在线程上下文完成。
5. 使用 SeqlockValue 的读取方必须处理读取失败，并保留上一份有效快照。

# 构建文档

在仓库根目录执行：

```sh
bash tools/build_doxygen.sh
```

生成首页位于 `build/doxygen/html/index.html`，警告日志位于
`build/doxygen-warnings.log`。
