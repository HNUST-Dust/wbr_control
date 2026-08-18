# rm_test 分层解耦现状报告（更新）

日期：2026-04-05  
范围：applications/rm_test

## 1. 结论摘要

当前工程的分层解耦已进入“主干完成、细节收口”阶段。

现状主干：
- 启动层：main
- 编排层：main 直接按 Kconfig 条件初始化并启动模块
- 业务层：remote_input / chassis
- 服务层：actuator / chassis_tuning
- 协议层：protocols/motors
- 消息层：zbus channels
- 平台层：platform/drivers + platform/storage

综合评估：
- 架构成熟度：较高
- 分层解耦评分：8.3 / 10

与上一版相比，以下问题已关闭：
- 已关闭：chassis 业务层直连 can_dispatch。
- 已关闭：shell 通过模块全局桥接调参。
- 已关闭：core 直接 include modules 注册头。
- 已关闭：main 直连 UART/CAN/LittleFS 初始化实现。

---

## 2. 已完成项（按解耦目标）

### 2.1 main 与 modules 的启动边界

已完成：
- main 按 Kconfig 条件直接初始化并启动模块。
- modules 层保留各自 `Initialize()` / `Start()` 生命周期函数。

关键落点：
- src/main.cpp
- modules/*/*_module.{h,cpp}

收益：
- 启动顺序更直观，不再需要注册表中转。
- 新增/裁剪模块时，核心编排层改动面更小。

### 2.2 启动编排与平台初始化解耦

已完成：
- main 按 Kconfig 条件直接初始化 UART/CAN/USB/LittleFS。

关键落点：
- src/main.cpp

收益：
- 启动顺序仍集中可控，入口能直接看到完整初始化流程。

### 2.3 业务执行路径简化（module -> native bus API）

已完成：
- actuator service 与 can_dispatch 发送包装已移除。
- chassis/arm/gantry 模块直接编码协议帧，并调用 Zephyr `can_send()`。
- UART 发送路径保持直接调用 `uart_poll_out()`，未额外保留发送包装。

关键落点：
- modules/chassis/chassis_module.cpp
- modules/arm/arm_module.cpp
- modules/gantry/gantry_module.cpp

收益：
- 发送路径更直观，module 内能直接看到协议编码与总线发送行为。

### 2.4 调参与业务模块解耦（shell -> service -> provider）

已完成：
- shell 命令通过 `chassis_tuning_service` 访问 PID 参数。
- chassis 通过 provider 接口注册调参能力。

关键落点：
- debug/shell/chassis_tuning_shell.cpp
- services/chassis/chassis_tuning_service.h
- services/chassis/chassis_tuning_service.cpp
- modules/chassis/chassis_module.cpp

收益：
- shell 不再依赖模块命名空间全局桥接函数。

### 2.5 配置裁剪能力（第一阶段）

已完成：
- 模块注册裁剪：`RM_TEST_MODULE_REMOTE_INPUT` / `RM_TEST_MODULE_CHASSIS`。
- shell 调参裁剪：`RM_TEST_SHELL_CHASSIS_TUNING`。
- runtime 初始化行为裁剪：
  - `RM_TEST_RUNTIME_INIT_CAN`
  - `RM_TEST_RUNTIME_INIT_LITTLEFS`

关键落点：
- Kconfig
- CMakeLists.txt
- src/main.cpp
- src/main.cpp

说明：
- runtime 已支持编译期裁剪：相关驱动/存储源按 Kconfig 条件加入构建。

---

## 3. 未完成项（当前剩余工作）

### P0（优先）

1. runtime 初始化从“行为裁剪”升级到“编译期裁剪”
- 状态：已完成（2026-04-05）。
- 实现：CMake 按开关条件加入 UART/CAN/LittleFS 相关源文件；runtime 与 actuator 增加编译期防护，避免关闭特性后的链接断裂。
- 结果：默认配置行为不变；关闭特性时可减少对应编译对象与依赖面。

2. 更新文档基线，消除“文档落后代码”
- 状态：已完成（2026-04-05）。
- 实现：重写 AGENT_HANDOFF，移除“仅骨架/未接线”结论，替换为当前已接线架构、条件构建与剩余工作清单。
- 结果：后续协作以当前实现为基线，不再按历史阶段信息推进。

### P1（其次）

3. 发送包装收敛
- 状态：已完成。
- 实现：移除 actuator service 与 `can_dispatch::SendStdData*`，CAN 发送由 module 直接构造 `can_frame` 并调用 `can_send()`。
- 后续：若需要复用，可优先抽取协议编码工具，而不是重新引入底层发送包装。

4. tuning service 生命周期完善
- 状态：已完成（2026-04-05）。
- 实现：在原有 `UnregisterProvider(provider)` 与 `HasProvider()` 基础上，升级为多 provider registry；新增 `RegisterProvider(provider, name, priority)`、`ProviderCount()`、`GetProviderStatus()`；服务按 priority 选择 active provider；shell `chassis pid status` 可观测 active provider 名称/优先级/数量。

### P2（演进）

5. include 边界收敛
- 状态：已完成。
- 实现：上层 include 使用领域前缀（如 `modules/...`、`channels/...`）；独立 algorithms include 根已移除，控制器和估计器头文件归对应模块所有。

6. 测试与回放资产补齐
- 状态：已完成第一步（2026-04-05）。
- 实现：新增最小 smoke 回归脚本 `applications/rm_test/tools/smoke_regression.sh` 与文档 `applications/rm_test/docs/SMOKE_REGRESSION.md`，覆盖启动链路、模块注册链路、调参链路可用性，以及默认/CAN-off 构建矩阵。
- 后续：补充行为一致性回放与实机时序测试资产。

---

## 4. 风险评估（当前）

1. 配置开关与构建开关不一致
- 风险：用户以为关闭功能即可减体积，但实际仅跳过初始化逻辑。

2. 服务层单点状态（provider/global）
- 风险：未来多模块扩展时，可能出现注册顺序依赖或重入边界问题。

3. 文档滞后导致的协作偏差
- 风险：后续开发依据过时文档重复做已完成工作或误改架构边界。

---

## 5. 建议的下一步顺序

1. 继续完善 tuning service provider 生命周期（P1-4），如需多 provider 则升级 registry。
2. 最后做 include 边界收敛与测试资产补齐（P2）。

---

## 6. 当前结语

rm_test 已从“可运行的迁移主干”推进到“可维护的分层架构主干”。
接下来重点不再是继续铺目录，而是把剩余边界收口（编译期裁剪、服务抽象、生命周期与测试）做扎实。完成这些后，整体解耦度可稳定到 9/10 区间。
