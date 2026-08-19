# 模块生命周期维护约定（active / staged）

日期：2026-04-05
范围：applications/wbr_control/modules

## 1. 目的

本约定用于避免以下问题反复出现：

- 目录存在但未接线，被误认为“已可用”
- 模块状态变化后文档未同步
- 重构阶段中 active/staged 边界反复漂移

## 2. 状态定义

1. active
- 已接入 CMake 构建
- 已注册到模块链路（直接或经 registry/config）
- 通过最小回归检查（默认构建 + 关键链路检查）

2. staged
- 用于迁移落点或预研
- 默认不承诺接入当前主运行链路
- 可存在占位实现，但不能在文档中描述为“已上线链路”
- 默认目录：`modules/staging/*`

## 3. 状态变更流程

1. staged -> active
- 完成模块最小接线（输入/输出/生命周期）
- 更新模块状态文档：
  - modules/README.md
  - modules/README.md（legacy 说明同步）
  - 对应模块目录 README（若存在）
- 更新导览文档：
  - docs/DIRECTORY_GUIDE.md
  - docs/AGENT_HANDOFF.md（如涉及主链路）
- 运行最小回归：
  - bash applications/wbr_control/tools/smoke_regression.sh

2. active -> staged（降级/暂时下线）
- 明确降级原因（依赖缺失/待重构/故障隔离）
- 同步更新上述文档与构建状态说明
- 确保主链路不再引用该模块

## 4. 文档同步清单（必做）

每次状态变化至少同步以下文件：

- modules/README.md
- modules/README.md（legacy 说明同步）
- docs/DIRECTORY_GUIDE.md
- docs/AGENT_HANDOFF.md（主链路变化时）

建议同步：

- docs/LAYERING_DECOUPLING_STATUS.md（涉及架构边界时）

## 5. 准入检查（active 最低标准）

模块要标记为 active，至少满足：

1. 编译可通过
- 默认配置构建通过并产出 ELF

2. 生命周期可预期
- Initialize/Start 行为可重复执行（幂等或可解释）

3. 输入输出边界清晰
- 输入来自 channel/service/protocol 的明确接口
- 输出不直接越层调用（遵循当前分层约束）

4. 观测入口可用
- 至少有一种可观测方式（log/shell/channel 状态）

## 6. 变更模板（建议提交说明）

```text
[module-status] <module_name>: staged -> active
- reason:
- wiring:
- docs updated:
- smoke result:
```

## 7. 与目录重构的关系

本约定是阶段 A 的治理手段，不依赖阶段 B/C 完成后才生效。
即使目录尚未全部重命名，也必须先按本约定维护状态一致性。
