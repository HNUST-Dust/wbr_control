# wbr_control 架构审阅稿

## 目标

本文基于旧 `Dust_EngineerRobot` FreeRTOS 工程的实际组织方式，为
`applications/wbr_control` 定义一版更适合 Zephyr 的新架构。

当前已经明确的决策是：

- 不再保留 `Robot` 聚合类
- 不再保留 `system_startup.cpp` 这类“中断入口 + 全局对象分发中心”
- 主功能按模块拆分，每个模块运行在自己的线程或等价执行上下文中
- 模块之间通过类型化话题和官方 `zbus` channel 传递信息
- `gimbal_pwm` 并入 `gimbal`
- 新工程不再沿用旧 `dvc_*` 风格命名
- 应用层默认采用 C++ 组织模块和生命周期对象

## 旧工程真正的职责分布

从旧工程代码来看，最核心的结构不是“功能类很多”，而是下面这四类职责被绑在了一起：

1. STM32 HAL 外设初始化
2. HAL 中断回调分发
3. FreeRTOS 任务创建
4. 通过全局 `Robot` 对象完成业务组合

尤其是 [Dust_EngineerRobot/App/system_startup.cpp](/Users/panpoming/Documents/hpm-zephyr/Dust_EngineerRobot/App/system_startup.cpp)，它实际承担的是一个总分发器角色：

- 注册 UART/CAN 回调
- 在回调里按总线和 ID 把数据直接转发给 `robot` 里的成员对象
- 延时等待外设稳定
- 最后调用 `robot.Init()`

而 [Dust_EngineerRobot/App/Robot.h](/Users/panpoming/Documents/hpm-zephyr/Dust_EngineerRobot/App/Robot.h) 则把输入设备、调试工具、底盘、龙门架、机械臂、舵机等全部挂在一个大对象下面。

这种写法在 STM32 HAL + FreeRTOS 下可以工作，但迁到 Zephyr 后会有三个明显问题：

- 回调路径过于中心化，后续扩展会越来越难维护
- 模块之间依赖关系藏在聚合对象内部，不利于并行开发
- 启动逻辑、设备初始化、业务组织混在一起，职责不清楚

## Zephyr 下的替代方案

### 1. `system_startup` 的替代方式

旧工程里 `system_startup` 的职责，在 Zephyr 下应拆成四块：

- 板级和外设初始化：交给 `boards/`、DTS、Kconfig、Zephyr 驱动初始化
- 应用启动编排：交给 `src/main.cpp`
- 模块注册与拉起：交给 `modules/module_manager.*`
- 数据分发：交给 `platform/drivers/communication/*` + `channels/*`

也就是说，Zephyr 下不应再保留一个统一的“全局启动任务”去做所有事情。

更推荐的流程是：

1. Zephyr 完成板级、驱动和设备初始化
2. `main.cpp` 初始化 `zbus` 接入层和传输适配层
3. `main.cpp` 调用 `module_manager`
4. `module_manager` 创建并拉起各个模块线程
5. 底层通信适配层把原始事件发布到 `zbus` channel
6. 各模块在线程上下文中订阅并处理自己关心的 channel

### 2. `Robot` 的替代方式

旧 `Robot` 的作用，本质上是“把所有模块和设备对象放在一个树里，再通过对象引用互相调用”。

新的替代方式是：

- 每个模块自己持有自己的状态
- 模块依赖通过 channel/话题显式表达
- 模块启动由 `main.cpp` 按 Kconfig 条件直接编排
- 输入和输出都通过话题流动，而不是经由聚合对象转发

这会让依赖关系从“对象持有关系”变成“channel 订阅关系”，更接近你想要的分布式模型。

在代码组织上，建议进一步落实为：

- 每个功能模块实现自己的 C++ 类
- `main.cpp` 负责按顺序初始化、启动模块，不承担模块内部业务逻辑

### 3. `gimbal_pwm` 的替代方式

旧思路里把 `gimbal_pwm` 单独拆出去，会把同一个控制链路人为拆成两个功能域。

新架构中：

- `gimbal` 负责完整的云台控制闭环
- `PWM` 或串口舵机输出属于 `gimbal` 的执行侧实现细节
- 执行器封装仍可落在 `platform/drivers/devices/actuators/`
- 但应用层不再保留一个独立的 `gimbal_pwm` 模块

## 新架构建议

### 运行模型

推荐采用“模块线程 + `zbus` + 传输适配”的三段式结构：

- 模块层：负责业务决策、状态机、控制律、任务调度
- `zbus` 层：负责模块间数据交换
- 传输层：负责和 UART/CAN/USB/外设驱动交互

除此之外，项目还需要两个与业务正交的支撑层：

- `debug/`：Zephyr shell、tracing、运行时诊断入口
- `platform/storage/`：文件系统、参数文件、日志文件的持久化接入

考虑到项目主体是 C++，建议模块层采用“C++ 对象 + Zephyr 内核对象”的方式实现：

- 线程、消息队列、延时工作等仍然使用 Zephyr 原生能力
- 模块状态、控制流程、设备协作关系由 C++ 类表达
- `zbus` 作为内核侧消息总线，外面包一层很薄的项目级 C++ 包装

典型链路如下：

1. `remote_input_module` 通过独占 UART DMA 接收 DR16 数据
2. `remote_input_module` 解析并发布 `remote_input_topic`
3. `chassis_module` 订阅需要的输入
4. 模块计算后发布 `chassis_command_topic`
5. 对应执行器封装把命令转换为 CAN/UART/PWM 输出

### 目录建议

```text
applications/wbr_control/
  src/
    main.cpp
  app/
    bootstrap/
      include/wbr_control/modules/
        bootstrap.h
        module.h
        module_manager.h
      src/
        bootstrap.cpp
        module_manager.cpp
    debug/
      shell/
      tracing/
    modules/
      chassis/
      gimbal/
      gantry/
      arm/
      remote_input/
      referee/
    zbus/
      channels.h
      channels.cpp
      observers.h
    topics/
      remote_input_channel.h
      imu_channel.h
      system_status_channel.h
      chassis_command_channel.h
      chassis_state_channel.h
      gimbal_command_channel.h
      gimbal_state_channel.h
      arm_command_channel.h
      arm_state_channel.h
      gantry_command_channel.h
      referee_state_channel.h
      motor_feedback_channel.h
    protocols/
      pc_link/
    modules/
      imu/onboard/             # 板载 IMU 专用姿态估计器
  platform/
    board/
      board_identity.*
    drivers/
      communication/
        can_dispatch.*
        usb_session.*
    storage/
      filesystem/
      devices/
        input/
        motors/
        actuators/
        system/
        common/
  boards/
    hpmicro/hpm6e00evk_v2/
```

## 旧工程到新工程的映射

### `Algorithm/`

不再保留独立的通用算法层。算法随实际使用者迁移，例如板载 IMU 的 Quaternion EKF
位于 `modules/ahrs/`，控制器实现位于对应控制模块内部。

### `App/`

旧工程里的应用层要拆成两部分：

- `modules/`
- `modules/`

对应关系建议如下：

- `app_chassis.*` -> `modules/chassis/chassis_module.*`
- `app_gimbal.*` -> 当前旧实现已移除，后续按新需求重写
- `app_gantry.*` -> `modules/gantry/gantry_module.*`
- `app_arm.*` -> `modules/arm/arm_module.*`
- `system_startup.*` -> 被 `bootstrap.*`、`module_manager.*`、`communication/*` 共同替代

### `Communication/`

建议拆成两层：

- `platform/drivers/communication/`：总线事件接入、底层分发
- `protocols/`：MCU/PC 等链路协议

也就是：

- 总线收发适配不再直接操控业务对象
- 协议逻辑也不直接承担总线初始化职责

### `Device/`

保留在：

- `platform/drivers/devices/`

但命名改成“设备角色名”，不再使用旧 `dvc_*` 前缀。

例如：

- `dvc_motor_dji.*` -> `dji_motor.*`
- `dvc_motor_dm.*` -> `dm_motor.*`
- `dvc_motor_cubemars.*` -> `cubemars_motor.*`
- `dvc_servo.*` -> `serial_servo.*`
- `dvc_pwm_servo.*` -> `pwm_servo.*`
- `dvc_referee.*` -> `referee_client.*`

### `Drivers/`

不再保留为顶层架构层。

这部分大多是围绕 STM32 HAL 的薄封装，在 Zephyr 下要么被驱动模型替代，要么以后按真实需要再回补。

## 本轮结构收敛结果

这次已经做的目录收敛包括：

- 删除 `app/features/` 下和 `modules/` 语义重复的旧占位文件
- 保留并强化 `modules/` 作为主功能目录
- 放弃自建 `app/pubsub/`，改为使用官方 `zbus`
- 保留 `channels/` 作为消息类型定义层
- 将设备占位命名从旧 `dvc_*` 风格切到角色命名
- 维持 `protocols/pc_link` 作为链路层命名

补充说明：

- 当前默认构建只接入了 `main/module_manager/zbus/board_identity`
  这条最小主干
- `modules/*`、`protocols/*`、`platform/drivers/*` 目前仍以迁移骨架为主
- 将应用入口和核心调度骨架切换到 C++ 形态
- 将调试和存储能力单独纳入 `debug/` 与 `platform/storage/`

## 建议的下一步修改顺序

为了风险最小，建议后续按这个顺序继续收敛：

1. 先把 `modules/` 的 C++ 生命周期接口定型
2. 再把 `channels/` 的基础接口定型
3. 再把 `debug/` 与 `platform/storage/` 的接入边界定型
4. 再定 `remote_input_module`、`referee_module` 这类输入模块的话题输出
5. 然后定 `chassis`、`gimbal`、`arm`、`gantry` 的命令与状态话题
5. 最后再决定每个设备封装和模块之间的边界细节

这一步先只收敛架构和文件语义，不急着把旧实现搬进来。
