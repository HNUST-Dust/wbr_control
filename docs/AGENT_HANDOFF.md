# wbr_control Agent Handoff（当前可接手版本）

日期：2026-04-05  
范围：applications/wbr_control

目录重构规划：
- 详见 `docs/DIRECTORY_RESTRUCTURE_PLAN.md`

目录导览（入口与 active/staged 状态）：
- 详见 `docs/DIRECTORY_GUIDE.md`

模块状态维护约定：
- 详见 `docs/MODULE_LIFECYCLE_POLICY.md`

平台 legacy 归档策略：
- 详见 `docs/PLATFORM_LEGACY_ARCHIVE_POLICY.md`

## 1. 一句话结论

wbr_control 已经不是“仅骨架”阶段，而是“主干可运行 + 分层已解耦到服务层 + 正在做边界收口”的阶段。

## 2. 当前真实架构

启动链路：
- main -> infrastructure init -> modules

分层结构：
- src/main.cpp：启动编排
- modules：remote_input、chassis 等业务模块
- services：chassis_tuning
- channels：zbus 消息主题
- protocols：motors 协议编解码
- platform：uart/can/littlefs 与板级封装

核心解耦状态：
- main 按 Kconfig 条件直接初始化并启动模块，不再使用注册表。
- main 按 Kconfig 条件直接初始化 UART/CAN/USB/LittleFS 基础设施。
- 模块发送路径已去掉 actuator/can_dispatch 包装，直接编码协议帧并调用原生 CAN/UART API。
- shell 调参通过 chassis_tuning_service 与 provider 对接，不再通过模块全局桥接函数。

结构迁移状态（阶段 B）：
- 启动编排已收敛到 `src/main.cpp`，模块实例也在 main 中直接拉起。
- 核心命名空间已迁移为顶层领域命名空间（如 `modules`、`channels`、`protocols`、`services`）。
- `core` 已彻底删除（无兼容层残留），统一使用 `modules/*`、`channels/*` 等根目录头路径。
- 已去掉外层 `app/` 物理目录，语义目录直接位于应用根目录。
- staged 模块已统一收敛到 `modules/staging/*`，active 与 staged 已物理分区。
- 独立 `src/algorithms` 已拆除；算法随实际控制器或估计器维护。板载 IMU 的 Quaternion EKF 位于 `src/modules/ahrs/`，不处理 HI91 数据。
- 平台历史资产已建立 `platform/legacy/*` 归档分区；`legacy_dm_h723` 已归档到 `platform/legacy/board/legacy_dm_h723`。

## 3. 构建现状

默认构建已接入：
- modules：remote_input、chassis
- services：chassis_tuning
- channels：system_status、chassis_command、chassis_state、remote_input、motor_feedback
- protocols/motors：dji、dm、cubemars（按具体协议直接接入）
- 模块专用算法：随对应 module 一起构建
- platform：board_identity

条件构建：
- UART：由使用对应串口的模块独占管理，接收优先使用 UART async/DMA
- CAN dispatch：由 WBR_CONTROL_RUNTIME_INIT_CAN 控制
- LittleFS service：由 WBR_CONTROL_RUNTIME_INIT_LITTLEFS 或 WBR_CONTROL_SHELL_CHASSIS_TUNING 控制
- chassis tuning shell：由 WBR_CONTROL_SHELL_CHASSIS_TUNING 控制

说明：
- 已完成 runtime 的编译期裁剪，不再只是“初始化行为开关”。

## 4. Kconfig 关键开关

模块与调试：
- WBR_CONTROL_MODULE_REMOTE_INPUT
- WBR_CONTROL_MODULE_CHASSIS
- WBR_CONTROL_SHELL_CHASSIS_TUNING

runtime 初始化：
- WBR_CONTROL_RUNTIME_INIT_CAN（depends on CAN）
- WBR_CONTROL_RUNTIME_INIT_LITTLEFS（depends on FILE_SYSTEM_LITTLEFS）

## 5. 当前验证状态

已验证：
- 默认配置可成功编译并链接 zephyr.elf。
- 关闭 CAN runtime 初始化（overlay: WBR_CONTROL_RUNTIME_INIT_CAN=n）时可成功构建。
- CAN 关闭场景下镜像 ROM 有下降，说明编译期裁剪生效。

已知非阻塞告警：
- dts 中 bus-speed deprecated（上游绑定告警）
- HPM SDK 与 Zephyr 的 ARRAY_SIZE 宏重定义

## 6. 尚未完成的重点

P0（高优先）：
1. 文档全量同步
- 现已更新 LAYERING_DECOUPLING_STATUS 与本 handoff。
- 仍建议后续检查 MIGRATION_PLAN 中“进行中”措辞，避免误导为骨架阶段。

P1（下一阶段）：
2. chassis_tuning provider 生命周期完善
- 已完成：服务已升级为多 provider registry，支持按名称/优先级注册并选择 active provider；shell 可观测 active provider 名称/优先级/数量。

P2（演进）：
4. include 可见范围收敛
- 已完成第二步：上层 include 已迁移到应用根目录下的领域前缀（如 `modules/...`、`channels/...`、`platform/...`）。

5. 回归测试与回放资产补齐
- 已完成第一步：新增最小 smoke 回归脚本 `applications/wbr_control/tools/smoke_regression.sh`（文档见 `applications/wbr_control/docs/SMOKE_REGRESSION.md`）；后续仍需补齐行为一致性回放与实机时序测试。

## 7. 后续 agent 操作建议

LQR 物理模型、增益重新计算、三次腿长拟合和固件同步的完整流程见：

- `docs/LQR_GAIN_RECOMPUTATION.md`

重新计算增益时必须先阅读该文档；不要把 `controller.m` 第 3 节的示例零数组当成正式 K 数据。

1. 先看 CMake 与 Kconfig，再改业务代码
- 当前工程功能由开关与条件编译驱动，先确认构建开关再动代码可减少返工。

2. 改动通信或存储路径时优先落在 services 层
- 继续保持 modules 不直连 platform 的边界。

3. 改动后至少做两种配置构建验证
- 默认配置
- 至少一个裁剪配置（例如 CAN off）

4. 调参链路联调可先用状态命令检查 provider
- shell: `chassis pid status`
- 输出 `provider=ready/not_ready` 可快速判断调参接口是否已绑定到模块。

5. 提交前可执行最小回归脚本
- `bash applications/wbr_control/tools/smoke_regression.sh`
- 覆盖：启动链路、模块 Kconfig 裁剪、调参链路可用性、默认/CAN-off 构建。

## 8. 不要再按旧结论行动

以下旧结论已不成立：
- “modules 与 protocols 尚未接入默认构建”
- “模块还需要通过注册表拉起”
- “chassis 仍直接调用 can_dispatch”

请以后以本文件与 LAYERING_DECOUPLING_STATUS 的最新版本为准。
