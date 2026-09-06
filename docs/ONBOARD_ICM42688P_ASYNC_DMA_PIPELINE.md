# 板载 ICM42688P-HXY：DRDY + SPI HDMA + Quaternion EKF 完整链路

## 1. 这套实现解决什么问题

板载 IMU 链路的目标不是“用 SPI 读到寄存器”这么简单，而是同时满足：

- ICM42688P-HXY 每产生一组新数据，由 `INT1/DRDY` 主动唤醒 MCU；
- MCU 用 SPI2 硬件控制器发起一次连续 burst，读取完整六轴数据；
- TX/RX 由 HDMA 搬运，调用 `spi_transceive_cb()` 后提交线程立即返回；
- 只有 TX DMA、RX DMA 和 SPI `END` 三个条件都完成，并确认控制器不再
  `SPIACTIVE`，才允许完成事务和释放 CS；
- DMA 数据区不与 D-cache 争用；
- 解析、标定和四元数 EKF 全部在线程上下文运行，不在 GPIO/SPI ISR
  中执行浮点算法；
- 采样结果通过有界 spinlock 快照 channel 发布，避免非原子 C++
  对象并发拷贝的数据竞争。

> 重要：随项目提供的芯片是 **ICM42688P-HXY**，其手册寄存器布局和常见的
> TDK/InvenSense ICM-42688-P 驱动并不相同。本实现以用户提供的
> `ICM42688P-HXY手册.PDF` 为准，不能直接套用 Zephyr/PX4 中同名芯片的寄存器表。

## 2. 从中断到姿态发布的数据流

```text
ICM INT1 上升沿（1.6 kHz）
  -> HPM GPIO ISR
     -> 只记录 DRDY 计数/周期戳并写入 latest-only 通知队列
  -> ahrs 线程被唤醒
     -> Zephyr SPI context 通过 spi_config.cs 拉低 GPIO CS
     -> spi_transceive_cb() 提交 13 字节全双工事务
        TX: [0x80 | 0x0C, 0, 0, ...]
        RX: [dummy, AX_H, AX_L, AY_H, AY_L, AZ_H, AZ_L,
                    GX_H, GX_L, GY_H, GY_L, GZ_H, GZ_L]
     -> 函数立即返回，线程继续处理上一帧 estimator
  -> RX HDMA 完成 + TX HDMA 完成 + SPI END
     -> 驱动确认 SPIACTIVE=0
     -> 驱动释放 GPIO CS、更新 SPI context
     -> Zephyr SPI callback 只记录状态、完成周期戳和 atomic complete 标志
  -> ahrs 线程在下一个 1 ms 释放点轮询到完成状态
     -> 复制本次 DMA 结果
     -> 大端 int16 解码
     -> 换算 m/s² 和 rad/s
     -> 静止零偏标定
     -> Quaternion EKF 更新
     -> latest_onboard_imu_sample.write()
```

ISR 中没有寄存器轮询、日志格式化、矩阵运算或等待 DMA。`Ahrs`
对象同时持有 `OnboardImu` 和 Quaternion EKF 对象；唯一的 `ahrs`
线程负责提交 DMA，并在传输完成后解码、标定、更新 estimator 和发布快照。

## 3. 芯片初始化与寄存器依据

当前传感器按 1.6 kHz、加速度 ±8 g、陀螺仪 ±2000 dps 工作，AHRS 线程与
Quaternion EKF 按固定 1 kHz 运行：

| 寄存器 | 地址 | 写入值 | 用途 |
|---|---:|---:|---|
| `WHO_AM_I` | `0x01` | 只读，期望 `0x6A` | 确认器件和 SPI 链路 |
| `COM_CFG` | `0x05` | `0x50` | BDU 与地址自动递增 |
| `INT_CFG1` | `0x06` | `0x03` | 将 gyro DRDY 路由到 INT1；默认推挽、高有效 |
| `ACC_CONF` | `0x40` | `0xAC` | 高性能、normal BWP、1.6 kHz |
| `ACC_RANGE` | `0x41` | `0x02` | ±8 g，4096 LSB/g |
| `GYR_CONF` | `0x42` | `0xAC` | 高性能、normal BWP、1.6 kHz |
| `GYR_RANGE` | `0x43` | `0x00` | ±2000 dps，16.384 LSB/(°/s) |
| `PWR_CTRL` | `0x7D` | `0x0E` | 开启加速度计与陀螺仪，之后等待 10 ms |

初始化顺序为：GPIO/CS → `WHO_AM_I` → 上电传感器 → 等待 10 ms → 配置通信、
量程、ODR 和 INT1 → 每项读回校验 → 注册并开启 GPIO 上升沿中断。任何一步失败，
模块都不会创建采样线程，也不会向控制系统发布“有效姿态”。

### 为什么只触发 gyro DRDY 却读取六轴

加速度计和陀螺仪都配置成 1.6 kHz。`INT_CFG1=0x03` 使用 gyro DRDY 作为每帧触发源，
随后从 `0x0C` 连续读取到 `0x17`，一次得到同一更新周期的 accel + gyro。相比每个轴
单独读寄存器，它只有一次 CS、一次命令字节和一次 DMA 完成流程，六轴之间的时间
偏差也最小。

## 4. SPI DMA 驱动状态机

HPM SPI 驱动为每个 DMA chunk 维护以下完成位：

- `RX_DONE`：RX HDMA channel 已完成；
- `TX_DONE`：TX HDMA channel 已完成；
- `END_DONE`：SPI 外设产生 END，且 `spi_wait_for_idle_status()` 确认
  `SPIACTIVE=0`；
- `ERROR`：DMA、SPI setup 或 idle 确认失败。

全双工事务必须满足：

```text
(RX_DONE && TX_DONE && END_DONE) || ERROR
```

不能只看 DMA 完成。TX DMA 完成只代表最后一个字节已经写入 SPI FIFO，并不必然代表
最后一位已经从 MOSI 发出；如果这时释放 CS，传感器可能丢失帧尾。也不能只看 SPI
END，因为 RX DMA callback 可能仍未完成内存可见性处理。

完成顺序由硬件决定，状态机不假设 END、RX 或 TX 谁先到。最后一个条件到达时才：

1. 关闭 END interrupt 与 SPI DMA request；
2. 对 cacheable RX buffer 做完成后的 invalidate；
3. 若为最后一个 chunk，通过 `spi_config.cs` 释放 GPIO CS；
4. 更新 Zephyr `spi_context`；
5. 处理下一 buffer/chunk，或调用用户 callback，随后释放 SPI context lock。

采集层对每次异步事务设置 2 ms 超时。超时后驱动停止 TX/RX DMA、禁用
SPI DMA request、复位无法进入 idle 的控制器、释放 CS，并且只以
`-ECANCELED` 完成一次 callback。超时恢复与正常 ISR 完成使用同一状态所有权，
避免二次 callback 或二次释放 context。

CS 不在用户 callback 中手动操作，因此 callback 观察到的必然是已经结束且 CS 已释放的
事务。多 buffer 事务只在最后一个 chunk 释放 CS，中间 chunk 保持连续选中。

## 5. D-cache 与 DMA 内存一致性

DMA 直接访问物理内存，不会自动读取或更新 CPU D-cache。因此驱动对普通 cacheable
buffer 做：

- TX 前：cache line writeback；
- RX 前：flush（写回并失效），保存边界 cache line 中可能存在的脏数据；
- RX 后：invalidate，让 CPU 下一次读取 DMA 写入的新数据。

板载 IMU 进一步采用更严格的方式：控制寄存器 TX/RX buffer 和 13 字节 burst TX/RX
buffer 全部声明为 `__nocache __aligned(64)`，不使用线程栈作为 DMA 数据区。这样可避免
小 buffer 与栈变量共用 cache line，也避免早期曾出现的 DMA/cache/栈破坏类故障。

`spi_buf` 和 `spi_buf_set` 描述符在异步事务整个生命周期内同样保持静态存活；不能把
异步传输的数据 buffer 或描述符定义为提交函数的局部临时对象。

## 6. 实时性与 CPU 占用

当前 SPI 默认为 8 MHz，一帧传输 13 字节：

```text
13 * 8 / 8 MHz = 13 us
```

AHRS 每 1 ms 最多提交一帧，纯线速占用约 `13 us / 1 ms = 1.3%`。
DMA 传输期间 CPU 不逐字节轮询，AHRS 线程也不等待 DMA；它会在下一个 1 ms
周期轮询完成标志。

DRDY 时间戳队列只保留一项。队列满时丢弃旧项并保留最新项，这是从 1.6 kHz
取最新样本到 1 kHz 时的预期降采样。生产 AHRS 不保留该过程的调试计数。

调度优先级为：AHRS `4` > 底盘闭环 `5` > CAN TX `6` > USB 异步提交 `7`。
由于采集与融合使用同一线程，Quaternion EKF 也会按 AHRS 优先级运行；实时性能
验收应使用独立测试构建或外部测量，不在生产 AHRS 周期中加入遥测。

## 7. 启动零偏标定

默认要求 2000 个**连续静止**样本，即 1 kHz 下约 2 秒。每帧同时满足：

- `|norm(accel) - 9.80665| <= 0.5 m/s²`；
- `norm(gyro) <= 0.1 rad/s`。

运动会清空累计和并从 0 重新计数。标定完成前仍发布原始值和 SI 单位值，但
`calibrated=false`、`valid=false`。完成时用平均重力向量初始化 EKF 的
roll/pitch，而不是先发布单位四元数；之后减去三轴启动零偏。

样本数由 `CONFIG_WBR_CONTROL_ONBOARD_IMU_CALIBRATION_SAMPLES` 设置。不要为了缩短启动
时间而在机器人运动时强行完成标定。

## 8. Quaternion EKF 输出含义

Quaternion EKF 接收已校准输入：gyro 为 rad/s，accel 为 m/s²。其六维状态为
四元数加 X/Y gyro bias，用加速度重力方向进行观测更新，并在强运动下通过创新检验
降低不可信加速度的影响。预测和后验都将协方差投影到单位四元数的切空间，避免归一化
状态却保留径向不可观方差。

没有磁力计，因此 roll/pitch 可以由重力长期约束，yaw 只能靠陀螺积分，长期必然漂移。
四元数排列为 `[w, x, y, z]`，欧拉角单位为度。

标定、初始化和滤波始终在底盘 IMU 原始坐标系内完成。AHRS 不执行 sensor frame
到 chassis frame 的变换；`BODY_MAP_CONFIRMED` 由底盘消费者检查。

滤波器的 `dt` 由相邻两个实际处理的 DRDY 周期戳计算，不再固定写死为 5 ms。
丢帧时会按实际间隔积分，并将大于 10 ms 的区间分为多个子步；超出
2.5–100 ms 合理范围时本帧 `valid=false` 并从重力方向重新初始化。

## 9. 发布 channel

`channels::latest_onboard_imu_sample` 包含：

- DRDY 物理采样时间戳 `timestamp_us`；
- 底盘 IMU 原始坐标系下的 EKF roll/pitch/yaw，单位为度；
- 底盘 IMU 原始坐标系下、经启动零偏与 EKF 残余零偏修正的三轴角速度，单位为 rad/s；
- 综合 `valid`。该标志只证明采集、标定、时序和 EKF 有效，不证明底盘 TF 已确认。

channel 是“最新值快照”，不是历史队列。AHRS 不再写入
`latest_oscilloscope_sample`，也不保留 DRDY/DMA/滤波矩阵遥测计数。

## 10. Kconfig 开关

正式工程已经启用硬件 SPI、异步 API 与 HPM SPI DMA：

```ini
CONFIG_SPI=y
CONFIG_SPI_HPMICRO=y
CONFIG_SPI_ASYNC=y
CONFIG_SPI_HPM_SPI_DMA=y
CONFIG_SPI_HPM_SPI_INTERRUPT=n
```

板载模块本身由下列开关控制：

```ini
CONFIG_WBR_CONTROL_MODULE_AHRS=y
CONFIG_WBR_CONTROL_ONBOARD_IMU_SPI_FREQUENCY_HZ=8000000
CONFIG_WBR_CONTROL_ONBOARD_IMU_CALIBRATION_SAMPLES=400
CONFIG_WBR_CONTROL_ONBOARD_IMU_BODY_MAP_CONFIRMED=n
CONFIG_WBR_CONTROL_CHASSIS_IMU_HI91=y
```

底盘 IMU 到底盘控制坐标系的静态旋转由 `src/tf_tree.h` 中的通用 `TfTree`
统一管理，当前定义为
绕 Z 轴旋转 180°：`X_chassis=-X_imu`、`Y_chassis=-Y_imu`、
`Z_chassis=Z_imu`。

该旋转由底盘线程在读取 AHRS 快照后执行：EKF 的标定、初始化、预测、重力观测和
发布始终使用底盘 IMU 原始坐标系。AHRS 不产生 VOFA 数据。预留的另一棵树为
`gimbal_imu -> gimbal -> pc_link`，用于未来生成视觉上位机协议中的姿态；
在云台 IMU 接入前不配置也不参与运行。当前 `pc_link` 上行四元数仍来自 HI91。

当前 `app.overlay` 已将 INT1 配置为 PB09，`prj.conf` 已启用模块。

## 11. PCB Devicetree 接线要求

已从现有 `spi_test` 验证的 SPI2 连接是：

| 信号 | HPM6754 引脚 |
|---|---|
| CS（软件 GPIO） | PA26 |
| SCLK/MOSI/MISO | 现有 SPI2 pinctrl：PA27、PA31、PB00 |
| INT1/DRDY | PB09 |

正式 overlay 必须完成三件事：

1. 定义 `onboard-imu-cs` 和 `onboard-imu-drdy` 两个 alias；
2. 从 SPI2 pinctrl 中移除硬件 CS 的 PA26，并删除控制器默认 `cs-gpios`；模块把
   `onboard-imu-cs` 填入 `spi_config.cs`，由 SPI context 按事务控制 GPIO CS；
3. 把 PA26 与真实 INT1 引脚配置为 GPIO，INT1 alias 使用 `GPIO_ACTIVE_HIGH`。

如果后续 PCB 版本更换，必须重新核对 PB09；错误的 pinmux 可能覆盖 CAN、UART、
PWM 或其他算法依赖的外设。

## 12. 上板验证顺序

1. 上电后保持 PCB 静止至少 2 秒；
2. 确认 `latest_onboard_imu_sample.valid` 在标定及首个有效周期后变为 true；
3. 平放时 roll/pitch 稳定，静止角速度接近 0 rad/s；
4. 分别绕 PCB 三轴转动，确认 IMU 坐标系中的 roll/pitch/yaw 方向；
5. 在底盘线程确认 `chassis_imu -> chassis` 后的 pitch 和 pitch-rate 符合控制约定；

初始化完成时如果 INT1 已经为高，驱动会向同一个单元素队列补投递一次采集请求。
该操作使用 `K_NO_WAIT`，用于读取并释放启动阶段已经锁存的数据就绪状态，避免在
等待下一次上升沿时形成死锁。

启动静止校准允许原始加速度模长相对标准重力存在最多 `2.0 m/s²` 的板级误差。
完整静止窗口结束后，根据平均加速度模长计算单一比例系数，使送入 EKF 的静止
加速度模长回到 `9.80665 m/s²`；该比例校正不改变测得的重力方向。

底盘 IMU 原始坐标到底盘控制坐标使用 Sophus `SO3f` 管理。当前固定变换为绕
`+Z` 旋转 180°，即 `X_chassis=-X_imu`、`Y_chassis=-Y_imu`、
`Z_chassis=Z_imu`。EKF 始终在底盘 IMU 原始坐标系运行，滤波后的姿态和角速度
才转换到底盘坐标系；底盘分支没有通往 `pc_link` 的边。
6. 检查 EKF：静止时 roll/pitch 应收敛，绕 Z 轴旋转时 yaw 方向应正确；
7. 记录端到端延迟、两线程峰值运行时间和最小剩余栈，并在所有模块
   开启时做压力测试；
8. 最后再把 `valid=true` 的 onboard IMU 姿态接入底盘闭环。

主机数值测试已覆盖：倾斜重力初始化、丢帧形式的可变 `dt`、强动态加速度拒绝、
20 秒不规则采样积分/连续 yaw、非法 `dt` 以及 NaN 健康检测。

独立 `spi_test` 还提供以下 GDB 证据变量：

- `imu_async_submit_sequence` / `imu_async_callback_sequence`：提交和回调必须一一对应；
- `imu_async_outstanding_on_return_sequence`：每次 API 返回时 callback 尚未发生则增加，
  用来证明事务并非在 `spi_transceive_cb()` 内同步等待完成；
- `imu_worker_during_dma_last`：DMA 在途期间调度探针线程实际运行的次数；
- `imu_callback_before_submit_return_sequence`：正常应为 0；短事务即使偶尔在 API 返回前
  完成也不构成 API 错误，但 13 字节、100 kHz 测试应稳定表现为异步在途；
- `imu_submit_return_cycle` / `imu_callback_cycle`：可直接计算 API 返回到硬件完成的周期差。

测试启动时还会在 100 kHz 下故意中止一次在途 DMA，要求 callback 收到
`-ECANCELED`，然后立即以正常速率重读 `WHO_AM_I`。
`imu_abort_recovery_self_test_result == 0` 才表示超时恢复路径已经在真机上通过。

## 13. 底盘 IMU 来源与硬件确认门槛

底盘来源是一个编译期 choice：HI91 或 onboard EKF。默认仍为 HI91。
onboard 选项依赖 `BODY_MAP_CONFIRMED=y`；底盘线程还会在运行时检查该开关和
TF 转换结果，AHRS 自身的 `valid` 不包含坐标映射确认状态。

HI91 和 onboard AHRS 都使用 3 ms 新鲜度门限。onboard 正常每 1 ms 发布一次，
因此 3 ms 可容纳一次调度抖动，同时仍能快速拒绝过期姿态。

真机上需依次绕 chassis X/Y/Z 正方向转动，确认底盘线程转换后的 gyro 只在对应轴为正，
再检查正俯仰时 pitch 和 pitch-rate 与底盘现有约定同号。只有完成这些测试后
才应将 `BODY_MAP_CONFIRMED` 设为 `y` 并选择 onboard 来源。

`latest_chassis_realtime_status` 以 10 Hz 发布底盘循环本次/历史最大执行时间、累计错过
释放次数、栈剩余高水位、IMU 年龄/新鲜度和实际编译进固件的 IMU 来源。
满载验收时应要求 `deadline_miss_count` 不增长、`max_loop_execution_us < 1000`，
并为栈保留充足余量。
