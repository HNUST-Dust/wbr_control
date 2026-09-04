# wbr_control 最小回归检查（Smoke）

本文件提供一套轻量回归检查流程，用于在没有硬件联调条件时，快速验证关键主链路没有被重构破坏。

脚本位置：
- applications/wbr_control/tools/smoke_regression.sh

## 覆盖范围

1. 启动链路（静态契约）
- `main.cpp` 仍直接初始化基础设施
- `main.cpp` 仍直接初始化并启动各模块

2. 模块注册链路（静态契约）
- remote_input/chassis 仍受 Kconfig 开关控制

3. 调参链路可用性（静态契约）
- shell 暴露 `chassis pid status`
- tuning service 暴露 provider 可用性接口（`HasProvider()` / `GetProviderStatus()`）

4. 构建可用性（动态）
- 默认配置可编译并产出 ELF
- CAN-off 配置可编译并产出 ELF

## 运行方式

在仓库根目录执行：

```bash
bash applications/wbr_control/tools/smoke_regression.sh
```

## 通过标准

脚本末尾打印：

- `Smoke regression passed with N checks.`

并且过程中无 `[FAIL]`。

## 说明与边界

- 该脚本是“最小回归资产”，重点在于重构安全网，不替代硬件行为验证。
- 对于控制精度、实时抖动、总线时序等，需要后续补充实机测试与回放资产。
