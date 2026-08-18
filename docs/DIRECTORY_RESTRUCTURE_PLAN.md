# rm_test 目录结构重构方案

日期：2026-04-05
范围：applications/rm_test

## 0. 当前进度（实时）

- 阶段 A：已完成（导览、active/staged 标记、维护约定）。
- 阶段 B：已完成前三步中的前两步，第三步已完成“语义入口层”子任务。
  - 已完成：`core` 目录迁移到 `modules`（源码与主头路径）。
  - 已完成：核心命名空间迁移到顶层领域命名空间（如 `modules`、`channels`、`protocols`、`services`）。
  - 已完成：`core` 目录已直接删除（无兼容层）。
  - 已完成：建立 `app/{channels,modules,services,protocols}` 语义目录与 `app/*` 稳定头入口。
  - 已完成：active 主链路 include 统一切换到 `app/*`。
  - 已完成：`channels` 实体文件已迁入 `channels`，并完成 CMake 源列表切换。
  - 已完成：`services` 实体文件已迁入 `services`，并完成 CMake 源列表切换。
  - 已完成：`modules` 实体文件已迁入 `modules`，并完成 CMake 源列表切换。
  - 已完成：`protocols` 实体文件已迁入 `protocols`，并完成 CMake 源列表切换。
  - 验证：默认构建与 smoke 回归通过。
- 阶段 C：已完成。
  - 已完成：staged 模块已集中迁入 `modules/staging/*`，与 active 模块物理分区。
  - 已完成（第 2 步）：移除独立 `src/algorithms` 层；仍需使用的算法归入实际控制器或估计器模块。
  - 已完成（第 3 步）：建立 `platform/legacy/*` 归档分区并迁入 `platform/legacy/board/legacy_dm_h723`，同时落地平台归档策略文档。

## 1. 问题定义

当前目录虽然功能上可用，但存在以下开发体验问题：

1. 入口分散
- 启动编排已收敛到 `src/main.cpp`，模块生命周期基础设施保留在 `modules/`。

2. 语义层次混杂
- `app/` 下既有领域逻辑（modules/services/protocols/channels），又有历史算法资产和调试资产，边界不够直观。

3. 公共头导出路径不统一
- 目前同时存在 `include/rm_test/platform/*` 与 `app/include/app/*`，理解成本偏高。

4. 历史迁移目录与当前主干并存
- `arm/gimbal/gantry/referee` 等目录仍以迁移占位为主，和当前已接线主干（remote_input/chassis）混在一起。

## 2. 重构目标

1. 一眼能看懂系统分层和入口。
2. 目录命名体现职责，不依赖历史背景知识。
3. 公共 API 导出路径统一。
4. 支持渐进迁移，不破坏当前可编译和可运行状态。

## 3. 目标目录结构（建议）

> 说明：先给“终态”，后续按阶段迁移到位。

```text
applications/rm_test/
  CMakeLists.txt
  Kconfig
  prj.conf

  src/
    main.cpp                    # Zephyr C/C++ 程序入口（唯一入口）

  app/
    bootstrap/
      module_manager.cpp
      module_registry.cpp

    domain/
      channels/                 # 仅消息定义和 zbus channel 定义
      modules/                  # 业务模块（chassis/remote_input/...）
      services/                 # 应用服务（actuator/tuning/runtime facade）
      protocols/                # 协议编解码（motors/pc_link）

    platform/
      board/
      drivers/
      storage/

    debug/
      shell/
      tracing/

  include/
    rm_test/
      modules/...               # 领域层公共 API（统一导出）
      platform/...              # platform 层公共 API（统一导出）

  docs/
  tools/
  boards/
  conf/
```

## 4. 命名与边界规则

1. 目录命名规则
- 用职责命名，不用实现手段命名。
- 例：`domain/services/actuator`（职责）优于 `can_sender`（实现）。

2. 头文件导出规则
- 上层代码只能 include：
  - `#include <modules/...>`、`#include <channels/...>` 等领域目录头路径
  - `#include <rm_test/modules/...>`（启动编排层）
  - `#include <rm_test/platform/...>`
- 禁止上层直接 include 源码目录相对路径（例如 `modules/...`）。

3. 依赖方向规则
- `domain/modules -> domain/services -> platform`
- `domain/modules -> domain/protocols`
- `platform` 不依赖 `domain/modules`

4. 迁移占位规则
- 占位模块统一放 `domain/modules/staging/`（或在 README 标注 staged），避免与 active 模块混淆。

## 5. 入手点说明（给新开发者）

建议在 README 顶部固定放以下“3 分钟上手路径”：

1. 启动入口
- `src/main.cpp`

2. 当前主链路
- 输入：`modules/remote_input`
- 控制：`modules/chassis`
- 下发：`services/actuator`

3. 调参与运维
- Shell：`debug/shell/chassis_tuning_shell.cpp`
- 最小回归：`tools/smoke_regression.sh`

## 6. 分阶段迁移计划（低风险）

### 阶段 A：语义收敛（不挪文件）

1. 补充目录 README 与“active/staging”标记。
2. 在 docs 中固定入口索引（main/bootstrap/domain chain）。
3. 统一 include 风格到 `app/*` 与 `rm_test/platform/*`。

退出条件：
- 新同学 5 分钟内能定位入口与主链路。

### 阶段 B：轻量重命名（少量挪动）

1. 已完成：`core` -> `modules`，且兼容层已删除。
2. 已完成（语义入口层）：建立 `app/*` 与 `app/*` 稳定头入口，并将 active 主链路 include 切换到该入口。
3. 已完成（物理目录迁移）：`channels`、`services`、`modules`、`protocols` 已迁入 `app/*`。
4. 已完成：CMake source 列表与 include 导出已随物理迁移同步。

退出条件：
- 编译通过，smoke 回归通过。

### 阶段 C：历史资产分区

1. 已完成：迁移占位模块移入 staging 区。
2. 已完成：拆除通用算法目录，将保留的实现归入实际业务模块。
3. 已完成：平台 legacy 目录明确归档策略。

退出条件：
- active 与 staging 明确，主干目录噪音显著降低。

## 7. 本周可立即执行的最小动作

1. 在 `docs/` 增加目录导览页（入口/主链路/调试入口）。
2. 已完成：staged 模块已收敛到 `modules/staging/*`。
3. 进入后续能力演进：补齐回放/实机测试资产与服务层能力扩展。

## 8. 风险与回滚

1. 风险：路径迁移导致 include/CMake 断裂。
- 对策：每一步都跑 `tools/smoke_regression.sh` + 默认全量构建。

2. 风险：大规模移动影响正在开发分支。
- 对策：按阶段提交，小步落地；每阶段不超过 1 个核心目录迁移。

3. 风险：文档滞后再次出现。
- 对策：每次结构变更必须同步更新 `AGENT_HANDOFF.md` 与本方案文档。
