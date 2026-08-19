# 底盘速度控制迁移映射（Dust_EngineerRobot -> wbr_control）

## 1. 迁移目标
在不修改 Algorithm 子模块代码的前提下，将底盘速度控制链路迁移到 wbr_control，并保持行为一致性。

## 2. 旧工程链路拆解

### 2.1 输入侧（目标速度设定）
- 入口位置：Robot::Task
- 来源：DR16 和 VT03
- 目标：设置 chassis 的 vx、vy、wz

关键调用：
- SetTargetVxInChassis
- SetTargetVyInChassis
- SetTargetVelocityRotation

### 2.2 控制侧（速度到轮速）
- 入口位置：Chassis::Task，周期 1ms
- 核心函数：Chassis::KinematicsInverseResolution
- 控制形式：
  - wheel1 = +0.707107 * vx - 0.707107 * vy + wz
  - wheel2 = -0.707107 * vx - 0.707107 * vy + wz
  - wheel3 = -0.707107 * vx + 0.707107 * vy + wz
  - wheel4 = +0.707107 * vx + 0.707107 * vy + wz

### 2.3 执行侧（电机输出）
- 四个 C620 电机各自 CalculatePeriodElapsedCallback。
- 每周期发送 CAN 0x200 控制帧。

### 2.4 反馈侧（CAN 回调注入）
- system_startup 中 can1_callback：
  - 0x201 -> motor1
  - 0x202 -> motor2
  - 0x203 -> motor3
  - 0x204 -> motor4

## 3. 新工程目标映射

### 3.1 模块映射
- 旧 Chassis 类 -> modules/chassis/chassis_module
- 旧 Robot::Task 中底盘输入子集 -> modules/remote_input + channel 发布
- 旧 system_startup 的 CAN 注入 -> platform/drivers/communication/can_dispatch

### 3.2 话题映射（第一版）
- 输入话题：chassis_command_topic
  - 字段建议：target_vx, target_vy, target_wz, source, timestamp
- 状态话题：chassis_state_topic
  - 字段建议：wheel_target_omega[4], wheel_now_omega[4], online_mask
- 反馈话题：motor_feedback_topic
  - 字段建议：id, now_omega, now_current, encoder, timestamp

### 3.3 控制循环映射
- Zephyr 线程周期：1ms（保持语义一致）
- 线程流程：
  1) 读取最新 chassis_command_topic
  2) 逆解计算 wheel_target_omega
  3) 通过电机接口下发输出
  4) 发布 chassis_state_topic

## 4. 必须保持一致的行为点

1. 逆解系数与轮序必须完全一致。
2. 零输入时四轮输出必须回零。
3. 输入突变时不出现轮序错位和符号翻转。
4. 控制周期目标 1kHz（允许 Zephyr 调度抖动但不改变控制语义）。

## 5. 第一版可后置的点

1. 云台坐标系到底盘坐标系旋转变换。
2. 功率限制和复杂保护逻辑。
3. 多输入源融合策略（先单源打通）。

## 6. 实施步骤（按顺序）

1. 定义 chassis_command_topic 和 chassis_state_topic 实体结构。
2. 扩展 zbus channel，打通发布/订阅接口。
3. 将 chassis_module 从占位改为真实线程实现。
4. main 按 Kconfig 条件直接拉起 chassis_module。
5. CMake 接入 chassis_module 源文件。
6. 添加离线逆解一致性测试（输入向量 -> 四轮目标速度）。

## 7. 第一版验收样例

输入样例（vx, vy, wz）：
1. (0, 0, 0)
2. (+1, 0, 0)
3. (0, +1, 0)
4. (0, 0, +1)
5. (+1, -1, +0.5)

验收标准：
- 轮速方向、相对比例、轮序与旧工程一致。
- 同输入下重复运行结果一致。
