# rm_test 重构迁移计划（Dust_EngineerRobot -> rm_test）

日期：2026-04-05
范围：applications/rm_test

## 1. 文档定位

本文件用于描述“从旧工程迁移到当前 rm_test 架构”的进度与剩余事项。

说明：
- 目录结构重构进度以 `docs/DIRECTORY_RESTRUCTURE_PLAN.md` 为准。
- 本文件聚焦“能力迁移与验证资产”，避免与目录规划重复。

## 2. 迁移目标（保持不变）

1. 行为一致性优先，重构不改变核心控制意图。
2. 在 rm_test 当前分层架构下完成能力落地：
- modules
- services
- channels
- protocols
3. 形成可持续验证闭环（构建 + smoke + 回放/实机）。

## 3. 当前已完成（截至今天）

### 3.1 主运行链路已完成迁移

- 启动链路：main -> infrastructure init -> modules。
- active 模块：remote_input、chassis。
- 服务层：actuator、chassis_tuning。
- 协议层：motors（dji/dm/cubemars）。
- 通信链路：UART/CAN dispatch 已接线。

### 3.2 目录与分层已完成收敛

- core -> modules 已完成并删除兼容层。
- channels/services/modules/protocols 实体已收敛到 app 顶层同名目录。
- staged 模块已集中到 modules/staging。
- 平台 legacy 分区已建立：platform/legacy。

### 3.3 最小回归能力已建立

- 已有脚本：applications/rm_test/tools/smoke_regression.sh。
- 已覆盖：启动链路、模块注册链路、调参链路、默认构建、CAN-off 构建。

## 4. 当前未完成项（真实剩余）

### P1（高优先）

1. 发送路径后续收敛
- 现状：actuator service 与 CAN 发送包装已移除，module 直接调用原生 CAN/UART 发送 API。
- 剩余：按需补齐协议编码测试，避免发送路径再次漂移。

2. tuning service 多 provider 演进（按需）
- 现状：单 provider 生命周期已完善（register/unregister/status）。
- 剩余：如出现多 provider 需求，升级 registry 与优先级策略。

### P2（演进）

3. 板载 IMU 姿态估计接线
- 现状：独立算法目录已拆除，Mahony 与 Quaternion EKF 已归入 `modules/imu/onboard/`。
- 剩余：完成 ICM42688P 驱动后接入采样、零偏校准与姿态发布链路。

4. 回放与实机测试资产补齐
- 现状：仅有 smoke 最小安全网。
- 剩余：补充行为一致性回放、实时时序与总线时序实机验证资产。

## 5. 验收分级（沿用）

### A 级（发布候选）
- 关键场景误差满足严格阈值。
- 异常输入下控制稳定，无明显发散。
- 长时回放无漂移累积异常。

### B 级（可联调）
- 常规场景满足目标阈值。
- 极限场景存在偏差但可解释。
- 不阻塞其他模块并行推进。

### C 级（可继续开发）
- 主链路跑通，输出方向与趋势正确。
- 数值偏差较大，仍需调参与修正。

## 6. 建议执行顺序（下一轮）

1. 先完成算法目录去重第二刀（低风险、结构收益高）。
2. 再补一组离线回放资产（覆盖 chassis 关键输入场景）。
3. 最后按需求推进 actuator 执行组扩展与 tuning 多 provider。

## 7. 风险与控制

1. 风险：文档口径再次落后代码。
- 对策：每次结构/能力变更同步更新本文件与 handoff。

2. 风险：仅 smoke 通过但实机行为偏差。
- 对策：补齐回放与时序测试，不把 smoke 当作行为验收替代品。

3. 风险：服务抽象扩展时引入兼容回归。
- 对策：小步扩展执行组，每步都跑默认构建 + CAN-off + smoke。
