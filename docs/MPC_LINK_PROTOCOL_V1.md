# MPC Link v1 上下位机通信协议

## 1. 适用范围

MPC Link 用于 `chassis_controller` 与 Linux `wbr_control_companion` 之间的实时状态和
支撑力指令传输。协议独立于帧头为 `SP` 的自瞄 PC Link，不能把 MPC 字段追加到旧自瞄帧。

传输基线为 USB 2.0 High-Speed Vendor Interrupt：

- OUT `0x01`：Linux 到下位机；
- IN `0x81`：下位机到 Linux；
- endpoint MPS 为 64 字节，轮询周期 1 ms；
- 一次 USB transfer 恰好承载一帧，不拆包、不粘包；
- 所有整数和 IEEE-754 `float32` 均为小端；
- 状态发布频率为 100 Hz；Linux 每消费一帧最新状态，至多回复一帧命令。

## 2. 公共帧头

| 偏移 | 长度 | 字段 | 说明 |
|---:|---:|---|---|
| 0 | 2 | magic | 固定 `W`,`M` (`0x57,0x4d`) |
| 2 | 1 | version | 固定为 `1` |
| 3 | 1 | message_type | 状态 `0x10`，命令 `0x20` |
| 4 | 1 | payload_length | 状态 44，命令 24 |
| 5 | 1 | header_flags | v1 固定为 0，非零帧必须丢弃 |
| 6 | 4 | sequence | 发送方递增序号，允许 `uint32` 自然回绕 |
| 10 | 4 | sender_uptime_ms | 发送方单调时钟，不用于跨设备直接相减 |
| 14 | N | payload | 消息负载 |
| 14+N | 2 | crc16 | 覆盖此前所有字节，小端 |

CRC 参数与既有 PC Link 一致：初值 `0xffff`、反射多项式 `0x8408`、无最终异或。

接收方必须严格检查 transfer 长度、magic、版本、类型、payload 长度、保留位和 CRC。v1
不做流式搜帧；任何检查失败都丢弃整次 transfer。

## 3. 状态帧 `0x10`（下位机到 Linux，60 字节）

payload 从整帧偏移 14 开始：

| payload 偏移 | 长度 | 类型 | 字段 | 单位 |
|---:|---:|---|---|---|
| 0 | 4 | float32 | roll | rad |
| 4 | 4 | float32 | roll_rate | rad/s |
| 8 | 4 | float32 | body_height | m |
| 12 | 4 | float32 | body_height_rate | m/s |
| 16 | 4 | float32 | left_leg_length | m |
| 20 | 4 | float32 | left_leg_rate | m/s |
| 24 | 4 | float32 | right_leg_length | m |
| 28 | 4 | float32 | right_leg_rate | m/s |
| 32 | 4 | float32 | left_support_force | N |
| 36 | 4 | float32 | right_support_force | N |
| 40 | 1 | uint8 | control_state | 下位机状态机枚举值 |
| 41 | 1 | uint8 | valid_flags | 见下表 |
| 42 | 1 | uint8 | contact_flags | 见下表 |
| 43 | 1 | uint8 | reserved | 固定为 0 |

`valid_flags`：

- bit 0：IMU 数据新鲜；
- bit 1：支持力估计有效；
- bit 2：电机反馈有效；
- bit 3：下位机平衡控制已使能；
- bit 4..7：保留为 0。

`contact_flags` 是下位机的接触提示，不是强制状态：bit 0 左侧接触，bit 1 右侧接触。Linux
仍应对支持力施加滞回和最短驻留时间。支持力无效时必须忽略这两个位。

## 4. 命令帧 `0x20`（Linux 到下位机，40 字节）

| payload 偏移 | 长度 | 类型 | 字段 | 说明 |
|---:|---:|---|---|---|
| 0 | 4 | uint32 | source_state_sequence | 本次求解所用状态帧序号 |
| 4 | 4 | float32 | left_support_force | 左腿绝对目标支撑力，N |
| 8 | 4 | float32 | right_support_force | 右腿绝对目标支撑力，N |
| 12 | 1 | uint8 | mode | 0 Disabled，1 Shadow，2 Active |
| 13 | 1 | uint8 | contact_mode | 0 Unknown，1 Both，2 LeftOnly，3 RightOnly，4 Airborne |
| 14 | 1 | uint8 | solver_flags | 见下表 |
| 15 | 1 | uint8 | reserved | 固定为 0 |
| 16 | 2 | uint16 | solver_iterations | 求解迭代次数 |
| 18 | 2 | uint16 | valid_for_ms | Linux 请求的有效期 |
| 20 | 4 | uint32 | solve_time_us | 本次求解耗时 |

`solver_flags`：bit 0 已收敛，bit 1 输出有限，bit 2 使用了 Linux 侧 fallback。

`Shadow` 命令只允许记录和比较，绝不能进入 VMC。`Active` 也不是直接授权；下位机仍必须
完成所有安全检查。`valid_for_ms` 建议为 30 ms，下位机必须再以本地最大值钳制，不能允许 Linux
扩大安全窗口。

## 5. 下位机命令接纳条件

下位机只有在以下条件全部满足时才能使用 MPC 力目标：

1. 帧结构和 CRC 有效，模式为 Active；
2. solver 的 Converged 与 InputFinite 位均置位；
3. 两个力均为有限数，并通过本地物理限幅和变化率限制；
4. `source_state_sequence` 是最近发送的状态之一，建议最多落后 2 帧；
5. 从收到该命令开始尚未超过 `min(valid_for_ms, 30 ms)`；
6. IMU、电机反馈和控制状态仍允许平衡；
7. USB reset、disconnect、序号长时间不前进或任一检查失败时立即 fallback。

fallback 是下位机现有 Roll PD + 腿长 PID + 重力前馈，不能依赖 Linux 发送“停止”帧才能触发。
由 Shadow 切换 Active 应经过人工使能，并建议做 100-300 ms 的无扰力交接。

## 6. 时序与丢包策略

- 状态帧只保留最新值，不重传历史状态；
- 命令帧不 ACK、不重传，下一状态周期会产生新命令；
- Linux 发现状态序号重复或倒退时不求解；
- 下位机发现命令序号重复时不刷新 watchdog；
- 两端不得用各自 uptime 直接计算链路延迟；延迟以本地收包时间和回显的
  `source_state_sequence` 判断；
- `uint32` 序号比较采用模运算，差值小于 `2^31` 视为向前。

## 7. 部署顺序

1. 只实现状态上行并核对单位、符号和 100 Hz 抖动；
2. Ubuntu 回传 Disabled 帧，验证 CRC、序号和 watchdog；
3. 使用 Shadow 模式记录 MPC 与现有控制器输出；
4. 静止架台测试 Active，验证拔 USB 后 30 ms 内回退；
5. 平地低腿长测试，再逐步进入斜坡和台阶测试。

协议 codec 位于下位机 `src/protocols/mpc_link/`，Linux 对应实现为
`wbr_control_companion/include/wbr/mpc_link.hpp` 与 `src/mpc_link.cpp`。
