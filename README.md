## wbr_control 项目简介

设计并实现一套面向复杂地形的串联型轮腿机器人控制系统（整车约 25 kg），完成从状态估计、运动控制到视觉感知与决策的全链路闭环开发。

- 控制侧基于 LQR + TinyMPC 构建轮腿系统控制器，实现底盘平衡与双腿伸缩调节，引入 VMC 提升地形适应能力，支持越障与动态跳跃（实测约 30 cm）；同时建立世界 / 底盘 / 云台多坐标系变换模型，实现底盘高速旋转下云台姿态稳定。
- 状态估计方面规划基于板载 ICM42688P 构建 QEKF + Mahony 姿态解算链路，支撑高频闭环控制；外置 HI91 保持设备侧解算结果直通。功率侧通过 RLS 建模与分配策略结合超级电容优化瞬态功率输出。
- 视觉侧基于 YOLO + PnP + MPC + 弹道模型 构建“感知 → 预测 → 控制”链路，实现目标识别与自动瞄准。

主要性能指标：
- 控制频率 1k Hz
- 系统延迟 < 2 ms
- 视觉处理延迟 10 ms（100 FPS）

系统实现方面，基于 HPM6754 双核 MCU（816 MHz） 进行异构任务划分：CPU1 负责控制算法与状态估计，CPU0 负责外设驱动与通信调度，并设计核间通信（IPC）实现低延迟数据同步。

在软件架构上基于 Zephyr RTOS + zbus 构建模块化发布-订阅系统，实现控制、感知与通信多线程解耦与并行执行；同时基于 Zephyr Shell 构建调试接口，结合 MAVLink + QGroundControl 实现实时数据可视化与在线参数调优，并支持参数 Flash 持久化。

- 核间通信延迟 < 10 μs
- 调参响应时间 < 20 ms

## 分层架构

| 层级 | 职责 |
|------|------|
| `src/main.cpp` | Zephyr 入口，`main()` 完成启动编排 |
| `src/modules/` | 业务模块及其专用控制器/估计器（remote_input、chassis、imu 等） |
| `src/protocols/` | 电机、遥控和遥测协议实现 |
| 各组件所属目录中的头文件 | 仓库内部接口，头文件跟随模块或库 |
| `channels/` | zbus 消息主题定义 |
| `platform/` | 板级、驱动、存储适配 |
| `debug/shell/` | Shell 调试命令 |

---

## 主要链路

```
remote_input_module 通过独占 UART DMA 接收遥控器数据
    ↓
remote_input_module 解析并发布 zbus channel
    ↓
chassis_module 订阅并计算，发布底盘指令
    ↓
module 直接编码并调用 CAN 原生发送接口下发到电机
```

---

## 快速上手

### 环境搭建
```bash
# 1. 安装 Python 虚拟环境和 West
python -m venv .venv
source .venv/bin/activate  # 或 .venv\Scripts\activate
pip install west

# 2. 新建 workspace 并 clone 工程
mkdir rm_test_ws
cd wbr_control_ws
west init -m https://github.com/NoneOfEver/wbr_control.git

# 3. 拉取依赖
west update
```

### 构建与烧录
```bash
# 构建
west build -p always -b hpm6e00evk_v2 -s .

# 烧录
west flash
```

### 常见问题

1) 找不到 Zephyr SDK

症状：配置阶段提示缺少 Zephyr SDK。

解决：指定 SDK 路径后再构建（根据实际安装路径调整）。

```bash
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
export ZEPHYR_SDK_INSTALL_DIR=/path/to/zephyr-sdk-0.16.5
west build -p always -b hpm6750evk2 -s wbr_control -d wbr_control/build
```

2) 缺少 pyelftools

症状：生成 kobject/driver 校验文件时报 `ModuleNotFoundError: No module named 'elftools'`。

解决：在当前虚拟环境中安装依赖。

```bash
python -m pip install pyelftools
```

## CI/CD

仓库中的 GitHub Actions 工作流会在以下场景运行：

- Pull Request、`main` 分支提交：构建 `hpm6750evk2` 固件，并保存 14 天构建产物。
- 推送 `v*` 标签（例如 `v0.1.0`）：在 CI 通过后自动创建或更新 GitHub Release，上传固件压缩包和 SHA-256 校验文件。
- Actions 页面手动运行：只执行 CI 和保存构建产物，不创建 Release。

发布新版本：

```bash
git tag -a v0.1.0 -m "v0.1.0"
git push origin v0.1.0
```

默认 CD 只发布经过同一次 CI 编译的固件，不会从云端 runner 烧录实体开发板。若需要自动烧录或硬件在环测试，应另外配置连接了 HPM6750EVK2 的 self-hosted runner。

## 许可证

Copyright 2026 .noe

除非文件中另有声明，本项目采用 [Apache License 2.0](LICENSE) 开源。第三方组件及明确标注其他 SPDX 标识符的文件继续遵循各自的许可证。
