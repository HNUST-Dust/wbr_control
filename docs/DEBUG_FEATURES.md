# Debug And Storage Features

## 当前结论

### 1. Shell

默认接入 Zephyr UART shell。

理由：

- 板级 DTS 已经指定 `zephyr,shell-uart = &uart0`
- 最容易直接落地
- 便于后续把模块状态、文件系统和诊断命令统一收进 shell

### 2. 文件系统

当前选择：

- 存储介质：外置 XPI flash 的 `storage_partition`
- 文件系统：LittleFS

理由：

- [hpm6e00evk_v2.dts](/Users/panpoming/Documents/hpm-zephyr/applications/wbr_control/boards/hpmicro/hpm6e00evk_v2/hpm6e00evk_v2.dts) 已经存在 `storage_partition`
- 当前板定义里没有现成启用的 SDHC 节点
- LittleFS 比 FatFS 更适合 NOR flash 上的小文件、参数文件、日志文件

这意味着当前方案更偏向：

- 参数文件
- 小到中等规模日志文件
- 设备本地持久化

如果以后主要目标变成“长时间大量日志”或“拔卡导出”，再考虑加 SDHC + FatFS。

当前 workspace 约束：

- 这套工作区现在已经补齐 `littlefs` 模块
- 本项目已经把 LittleFS 放进默认配置

## 调试功能选型

### 默认建议保留

- `shell stats`
- `thread runtime stats`
- `thread analyzer`
- `logging + shell log backend`
- `coredump(logging backend)`

### 可选增强

- `SEGGER SystemView`

## 哪些功能重复或相近

### `thread runtime stats` 和 `thread analyzer`

不是重复关系，建议都保留。

- `thread runtime stats` 更偏底层统计能力
- `thread analyzer` 更偏可读性更好的线程诊断入口

### `logging + shell log backend` 和 `shell stats`

不是重复关系，建议都保留。

- `logging + shell log backend` 负责日志输出和日志控制
- `shell stats` 负责 shell 自身和系统统计类命令

### `SEGGER SystemView` 和 `thread analyzer/runtime stats`

不是严格重复，但属于不同层级的观测工具。

- `SystemView` 是时间线级别的事件追踪
- `thread analyzer/runtime stats` 是轻量级运行时统计

建议：

- 平时常开 `thread analyzer/runtime stats`
- 需要深入时序分析时再启用 `SystemView`

### `SEGGER SystemView`

SystemView 是合理的增强方向，但当前这块 HPM 板还没有满足 RTT 依赖。

另外它依赖 RTT 通道：

- 当前 `segger` 模块已经补齐
- 但 `USE_SEGGER_RTT` 仍然因为 `HAS_SEGGER_RTT` 不满足而无法打开
- 所以 `conf/systemview.conf` 当前仍属于“预留方案”，不是立即可构建方案

## 当前最终建议

### 默认配置中保留

- UART shell
- LittleFS on external flash
- shell stats
- thread runtime stats
- thread analyzer
- logging + shell log backend
- coredump(logging backend)

### 单独配置文件中保留

- `conf/systemview.conf`

## 目前不建议删掉的项

- `shell stats`
- `thread runtime stats`
- `thread analyzer`
- `logging + shell log backend`
- `coredump`

## 目前最建议移出默认配置、保留为条件项的

- `SEGGER SystemView`

原因不是它们没价值，而是：

- `SystemView` 目前缺少当前板级的 RTT 支撑
