# 平台 legacy 归档策略

日期：2026-04-05
范围：applications/wbr_control/platform

## 1. 目标

- 将不参与当前 Zephyr 主链路的平台历史资产集中归档。
- 维持 active 平台目录（board/drivers/storage）的可读性和接入确定性。
- 为后续接管 legacy 资产提供可审计流程。

## 2. 目录约定

- active：
  - `platform/board`
  - `platform/drivers`
  - `platform/storage`
- legacy：
  - `platform/legacy/*`

## 3. 准入与退出规则

1. 进入 `platform/legacy/*` 的条件
- 当前不参与默认构建或主运行链路。
- 仅保留参考映射、迁移占位、或待评估实现。

2. 从 `platform/legacy/*` 恢复到 active 的条件
- 明确接口归属（board/drivers/storage 之一）。
- 默认构建与 smoke 回归通过。
- 文档同步：
  - `docs/DIRECTORY_RESTRUCTURE_PLAN.md`
  - `docs/DIRECTORY_GUIDE.md`
  - `docs/AGENT_HANDOFF.md`

## 4. 当前归档清单

- `platform/legacy/board/legacy_dm_h723/`
  - 来源：历史 FreeRTOS 工程 `Board/dm-h723/` 对应映射说明。
  - 状态：归档参考，不参与当前构建。

## 5. 维护要求

- 每个 legacy 子目录必须含 README（来源、状态、接管条件）。
- active 代码禁止 include 或依赖 `platform/legacy/*`。
- 每次新增/迁出归档资产，必须在本文件更新清单。
