# wbr_control 目录导览（阶段 A）

本页用于帮助新开发者在 3 分钟内看懂工程入口、主链路和当前 active/staged 状态。

## 1. 从哪里开始看

1. 程序入口
- src/main.cpp

2. 当前运行主链路（active）
- 输入模块：modules/remote_input
- 控制模块：modules/chassis
- 执行服务：services/actuator
- 调参服务：services/chassis
- 协议层：protocols/motors
- 平台通信：platform/drivers/communication

3. 调试与回归入口
- Shell 命令：debug/shell/chassis_tuning_shell.cpp
- 最小回归脚本：tools/smoke_regression.sh
- 回归说明：docs/SMOKE_REGRESSION.md

## 2. 目录分层速览

- src/: Zephyr 主入口
- modules/: 模块生命周期管理
- 应用根目录：领域语义聚合入口（含 channels/modules/services/protocols）
- modules/: legacy 说明目录（实体实现已迁移至 modules）
- services/: legacy 说明目录（实体实现已迁移至 services）
- channels/: legacy 说明目录（实体实现已迁移至 channels）
- protocols/: legacy 说明目录（实体实现已迁移至 protocols）
- 模块内控制器/估计器：算法实现随其实际使用模块维护
- platform/: 板级、驱动、存储适配
- platform/legacy/: 平台历史资产归档分区（不参与当前主链路）
- 应用根目录 include：对外稳定头路径（如 modules/*、channels/*、platform/*）
- docs/: 架构、迁移、交接与规范文档
- tools/: 自动化脚本（如 smoke 回归）

## 3. active 与 staged 约定

为避免“目录存在但未接线”误判，约定如下：

- active: 当前已接入构建并参与主运行链路
- staged: 目录与文件用于迁移落点/后续接线，默认不视为当前主链路

当前模块状态：

- active
  - modules/chassis
  - modules/remote_input

- staged
  - modules/staging/arm
  - modules/staging/gimbal
  - modules/staging/gantry
  - modules/staging/referee

## 4. 建议阅读顺序

1. docs/AGENT_HANDOFF.md
2. docs/LAYERING_DECOUPLING_STATUS.md
3. docs/DIRECTORY_RESTRUCTURE_PLAN.md
4. modules/src/module_manager.cpp
5. modules/remote_input/remote_input_module.cpp
6. modules/chassis/chassis_module.cpp

## 5. 结构重构计划

目录重构终态与迁移阶段见：
- docs/DIRECTORY_RESTRUCTURE_PLAN.md

模块 active/staged 维护约定见：
- docs/MODULE_LIFECYCLE_POLICY.md

平台 legacy 归档策略见：
- docs/PLATFORM_LEGACY_ARCHIVE_POLICY.md
