# WBR EtherCAT Process Data ABI v1

## 1. 目标与边界

该 ABI 同时服务于两种用途：

- 对比 SOEM、IgH generic、IgH native 的周期抖动、往返周期、WKC 和丢帧；
- 使用 IgH native 将 Linux 上的 roll/腿高 MPC 输出安全地送入下位机 VMC。

ABI 只定义实时 PDO。设备身份、邮箱、EEPROM/SII 和 SSC 代码由最终产品配置提供。
Vendor ID 必须使用实际获配值；开发阶段不得把文档中的占位值当成正式身份发布。

所有多字节量均为小端，浮点数为 IEEE-754 `float32`。ABI 标识固定为
`0x57425201`。主站每周期发送 72 字节，读取 96 字节。

## 2. Sync Manager 与 PDO 分配

| SM | 方向（相对从站） | PDO | 对象 | 字节数 |
|---|---|---|---|---:|
| SM2 | Output，主站到从站 | `0x1600` | Benchmark Rx `0x7000` | 32 |
| SM2 | Output，主站到从站 | `0x1601` | MPC Command `0x7010` | 40 |
| SM3 | Input，从站到主站 | `0x1a00` | Benchmark Tx `0x6000` | 32 |
| SM3 | Input，从站到主站 | `0x1a01` | Chassis State `0x6010` | 64 |

SSC Tool 中必须按下表顺序映射，不能插入自动 padding。最后一个 `reserved` 字段用于把
Chassis State 固定到 64 字节。
可直接照录的完整对象清单位于
`docs/ethercat/WBR_ECAT_OBJECT_DICTIONARY_V1.csv`。

## 3. Benchmark Rx `0x7000`

| SubIndex | 类型 | 字段 |
|---:|---|---|
| 1 | `UINT32` | `abi_version` |
| 2 | `UINT32` | `sequence` |
| 3 | `UINT64` | `host_send_time_ns`，仅用于主站本地 RTT 关联 |
| 4 | `UINT32` | `command`：0 Idle，1 Echo，2 ResetCounters |
| 5 | `UINT32` | `payload_crc32` |
| 6 | `UINT32` | `payload0` |
| 7 | `UINT32` | `payload1` |

CRC32 覆盖 `sequence,payload0,payload1` 的小端字节，参数为 IEEE CRC32：初值
`0xffffffff`、反射多项式 `0xedb88320`、最终取反。CRC 用于发现过程映像错位，不代替
EtherCAT FCS/WKC。

## 4. Benchmark Tx `0x6000`

| SubIndex | 类型 | 字段 |
|---:|---|---|
| 1 | `UINT32` | `abi_version` |
| 2 | `UINT32` | `echoed_sequence` |
| 3 | `UINT64` | `slave_receive_ticks` |
| 4 | `UINT64` | `slave_transmit_ticks` |
| 5 | `UINT32` | `status` |
| 6 | `UINT32` | `echoed_payload_crc32` |

`status`：bit0 ABI 正确，bit1 payload CRC 正确，bit2 从站应用运行，bit3 MPC watchdog
超时。MCU ticks 只用于计算从站内部处理时间；没有完成 DC/时钟标定前，不得与 Linux
`CLOCK_MONOTONIC` 直接相减。

## 5. MPC Command `0x7010`

| SubIndex | 类型 | 字段 |
|---:|---|---|
| 1 | `UINT32` | `abi_version` |
| 2 | `UINT32` | `sequence` |
| 3 | `UINT32` | `source_state_sequence` |
| 4 | `UINT32` | `mode_and_flags` |
| 5 | `UINT64` | `host_send_time_ns`，诊断用途 |
| 6 | `REAL32` | `left_support_force_n` |
| 7 | `REAL32` | `right_support_force_n` |
| 8 | `UINT32` | `valid_for_us` |
| 9 | `UINT32` | `solve_time_us` |

`mode_and_flags` 的低 8 位为模式：0 Disabled、1 Shadow、2 Active。bit8 Converged、
bit9 InputFinite、bit10 UsedFallback。

下位机仅在下列条件全部满足时接纳命令：

1. ABI、序号和浮点数有效；
2. 模式为 Shadow 或 Active，且 Converged/InputFinite 同时置位；
3. `source_state_sequence` 最多落后当前状态 2 帧；
4. `valid_for_us` 非零，并在下位机钳制到不超过 30 ms；
5. 支撑力经过下位机本地幅值及变化率限制；
6. 重复命令不能刷新 watchdog；
7. Active 超时、ABI 错误或非法新命令立即回到本地 Roll PD + 腿长 PID。

主站因 WKC、链路、OP 状态、传感器状态或求解器状态失效而降级时，必须发送一个**新序号**
的 Disabled 命令。不得只修改旧序号中的 mode；旧序号会被下位机按重放帧忽略。即使停止帧
丢失，下位机 30 ms watchdog 仍作为独立的最终保护。

Shadow 命令允许记录和对比，但不得进入执行器链路。Active 必须在 Linux 与下位机两侧
分别人工使能。下位机的本地 operator gate 默认关闭；通信 watchdog、非法命令或离开
安全的 Balance 状态会撤销该 gate，因此链路恢复后必须再次由本地操作员授权。协议核心
与 1 kHz 底盘消费端各自检查命令年龄；即使 SSC 任务停摆，底盘侧独立 watchdog 也会在
`valid_for_us` 到期后停止使用最后一次外部力。

## 6. Chassis State `0x6010`

| SubIndex | 类型 | 字段 |
|---:|---|---|
| 1 | `UINT32` | `abi_version` |
| 2 | `UINT32` | `sequence` |
| 3 | `UINT64` | `sample_ticks` |
| 4..13 | `REAL32` | roll、roll rate、body height、height rate、左右腿长/速度、左右支撑力 |
| 14 | `UINT32` | `status_flags` |
| 15 | `UINT32` | `reserved`，固定为 0 |

`status_flags`：bit0 IMU fresh、bit1 support force valid、bit2 motor feedback valid、
bit3 control enabled、bit4/5 左右接触、bit6 最近新命令已接纳、bit7 Active watchdog 超时、
bit8 本地 MPC operator gate 已使能且底盘状态允许、bit9 Active 支撑力正在 VMC 边界应用。

## 7. SSC 接入点

由于 Beckhoff SSC 受许可约束，仓库不包含生成代码。使用 SSC Tool 建立上述四个对象和四个
PDO 后：

1. 在 `APPL_OutputMapping` 将 SM2 的 72 字节复制到连续缓冲区；
2. 调用 `SscProcessDataBridge::Process()`；
3. 在 `APPL_InputMapping` 将产生的 96 字节复制到 SM3；
4. `ChassisPort::ReadChassisSample()` 从底盘最新值快照取数；
5. `HandleMpcCommand()` 在 Shadow 时只记录，在 Active 时送入 VMC；
6. `EnterLocalFallback()` 必须同步切回本地控制，不依赖 Linux 再发停止帧。

具体实现位于 `src/protocols/ethercat/`。HPM ESC 的 PDI/SYNC0/SYNC1/reset 中断由
`hpm_ethercat_set_callbacks()` 接入 SSC；Flash EEPROM 回调必须在
`hpm_ethercat_start()` 前注册。

内部 Flash 仿真由 `EthercatSiiStore` 与 `EthercatSiiFlashBackend` 提供：ESC 访问只读写
RAM 镜像，Flash 使用双槽、CRC、generation 和最终 commit marker。应用只能在输出禁用的
INIT/PREOP 维护阶段显式提交，禁止在 OP、SYNC0 或 PDI ISR 中擦写 Flash。

## 8. 测试顺序

1. 无执行器，FreeRun，1 ms，确认 WKC=3、CRC 和序号连续；
2. 分别运行 SOEM、IgH generic、IgH native，每组执行 idle/stress；
3. 测试周期 1000/500/250 us；每组至少 10 分钟；
4. 开启 DC/SYNC0 后重复 IgH native；
5. MPC Disabled，再 Shadow，最后架台 Active；
6. Active 时执行断线、主站退出、重复帧、NaN、陈旧状态和超时回退测试。

比较时固定从站固件、PDO、网线、周期、CPU 和压力负载。SOEM 使用普通 `igc`；IgH
generic 使用普通 `igc + ec_generic`；IgH native 使用 `ec_igc`，三者不可同时占用网卡。
