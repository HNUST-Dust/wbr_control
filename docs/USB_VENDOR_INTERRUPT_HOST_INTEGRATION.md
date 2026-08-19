# PC Link Vendor Interrupt 上位机对接说明

## 1. 目标与迁移状态

下位机使用 CherryUSB，枚举为单接口 USB 2.0 High-Speed Vendor Specific 设备。设备不再提供
CDC-ACM 虚拟串口，PC Link 仅通过 Interrupt IN/OUT 传输。

## 2. USB 枚举参数

| 项目 | 值 |
|---|---|
| USB 版本 | USB 2.0 High-Speed，固定 HS 配置 |
| VID | `0x34B7`（当前 HPMicro 示例 VID） |
| PID | `0xFFFF`（当前开发 PID） |
| 配置值 | `1` |
| Vendor Interface | `0` |
| Interface Class | `0xFF`（Vendor Specific） |
| Interface Subclass | `0x00` |
| Interface Protocol | `0x00` |
| Interrupt OUT | `0x01`，PC -> 下位机 |
| Interrupt IN | `0x81`，下位机 -> PC |
| 最大包长 | HS 64 字节 |
| 轮询周期 | HS 1 ms |

High-Speed 描述符使用 `bInterval=4`，即 `2^(4-1) * 125 us = 1 ms`。固件配置为
`CONFIG_CHERRYUSB_DEVICE_SPEED_HS=y`，联调时必须确认 Host 实际枚举速度为 480 Mbit/s High-Speed。

`0x34B7:0xFFFF` 目前是开发标识，正式发布前应申请或分配产品自己的 VID/PID。上位机不要把产品
字符串作为唯一识别依据，优先使用 VID、PID、接口号和可选序列号。

当前联调基线版本：

- CherryUSB：`50c7e908`（基于 `5ffe9cbe`）；
- hpm_support：`52f5268`（基于 `c21855f`）；
- HPM SDK：`7b8cb8bb2f04f52353a6429bffb024f0a2867876`。

## 3. 传输约定

每次 Interrupt Transfer 恰好承载一个完整 PC Link 帧：

- PC 读取 `0x81` 时，成功传输长度必须是 43 字节；
- PC 写入 `0x01` 时，单次写入必须是 29 字节；
- 不要把一帧拆成多次 USB 写入；
- 不要在一个 USB 写入中拼接多帧；
- 收到长度异常、帧头错误或 CRC 错误的帧时直接丢弃。

下位机为 OUT `0x01` 提交的 DMA 接收长度也是精确的 29 字节，而不是端点 MPS 64。首次提交发生在
`USBD_EVENT_CONFIGURED` 之后；每次 OUT completion 都会先把数据复制到独立消息队列并把端点标记为
待重挂。1 ms PC Link 工作线程在 USB completion ISR 返回后提交下一次 29 字节接收，避免在 HPM
驱动仍在回收 qTD 时从 callback 内立即复用同一个 qTD。若底层提交失败，线程会持续重试。

所有多字节整数和 IEEE-754 `float32` 都使用小端序。当前目标主机和 MCU 都是小端，但建议上位机
仍显式序列化字段，不要依赖结构体内存布局、默认对齐或编译器 ABI。

## 4. 下位机到上位机：43 字节遥测帧

| 偏移 | 长度 | 类型 | 字段 |
|---:|---:|---|---|
| 0 | 2 | `uint8[2]` | 帧头，固定为 `'S','P'` (`0x53,0x50`) |
| 2 | 1 | `uint8` | `mode`：0 空闲，1 自瞄 |
| 3 | 4 | `float32` | 四元数 `q.w` |
| 7 | 4 | `float32` | 四元数 `q.x` |
| 11 | 4 | `float32` | 四元数 `q.y` |
| 15 | 4 | `float32` | 四元数 `q.z` |
| 19 | 4 | `float32` | `yaw_angle` |
| 23 | 4 | `float32` | `yaw_velocity` |
| 27 | 4 | `float32` | `pitch_angle` |
| 31 | 4 | `float32` | `pitch_velocity` |
| 35 | 4 | `float32` | `bullet_speed` |
| 39 | 2 | `uint16` | `bullet_count` |
| 41 | 2 | `uint16` | CRC16，小端 |

下位机使用 completion-driven IN pipeline：配置完成后提交第一帧，之后每次 EP `0x81` completion
仅在 ISR 中释放 DMA buffer 并唤醒专用发送线程；该线程在 ISR 返回后立即采集最新状态、编码并提交
下一帧。它不再与一个独立的 1 ms producer 周期竞争，因此不会因为“producer 唤醒时上一帧仍 busy”
而整帧跳过。Host 必须持续提交 IN 读取；如果应用没有挂起读取，USB Device 无法主动把数据推送到
用户态。实时控制只应消费最新帧，不建议积压后补处理旧遥测。

## 5. 上位机到下位机：29 字节控制帧

| 偏移 | 长度 | 类型 | 字段 |
|---:|---:|---|---|
| 0 | 2 | `uint8[2]` | 帧头，固定为 `'S','P'` (`0x53,0x50`) |
| 2 | 1 | `uint8` | `mode`：0 不控制，1 控制不开火，2 控制并开火 |
| 3 | 4 | `float32` | `yaw_angle` |
| 7 | 4 | `float32` | `yaw_velocity` |
| 11 | 4 | `float32` | `yaw_acceleration` |
| 15 | 4 | `float32` | `pitch_angle` |
| 19 | 4 | `float32` | `pitch_velocity` |
| 23 | 4 | `float32` | `pitch_acceleration` |
| 27 | 2 | `uint16` | CRC16，小端 |

## 6. CRC16

CRC 覆盖除末尾 CRC 字段外的全部字节：遥测计算前 41 字节，控制命令计算前 27 字节。

- 初值：`0xFFFF`
- 反射多项式：`0x8408`（`0x1021` 的反射形式）
- 按每个字节的最低位优先处理
- 不执行最终异或
- CRC 字段按小端序追加

```cpp
uint16_t pc_link_crc16(const uint8_t* data, std::size_t size)
{
  uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (unsigned bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) ? static_cast<uint16_t>((crc >> 1U) ^ 0x8408U)
                       : static_cast<uint16_t>(crc >> 1U);
    }
  }
  return crc;
}
```

现有上位机 `tools::get_crc16()` 与该算法一致，可以原样复用。

## 7. 推荐的 C++ libusb 实现

### 7.1 构建依赖

Ubuntu：

```bash
sudo apt install libusb-1.0-0-dev
```

CMake：

```cmake
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)
target_link_libraries(your_target PRIVATE PkgConfig::LIBUSB)
```

### 7.2 打开与关闭

建议把 libusb 封装成独立 `UsbInterruptTransport`，不要把 VID/PID、端点号和 libusb 错误处理散落到
`Gimbal` 业务类中。

打开顺序：

1. `libusb_init()`；
2. 按 VID/PID 查找设备；
3. `libusb_set_auto_detach_kernel_driver(handle, 1)`；
4. `libusb_set_configuration(handle, 1)`，已处于配置 1 时允许忽略 `BUSY`；
5. `libusb_claim_interface(handle, 0)`；
6. 启动持续 IN 读取线程。

关闭顺序：停止读线程、release interface、close handle、exit context。

### 7.3 同步传输最小示例

```cpp
constexpr uint8_t kEpOut = 0x01;
constexpr uint8_t kEpIn = 0x81;
constexpr int kTimeoutMs = 20;

int write_command(libusb_device_handle* handle, const uint8_t frame[29])
{
  int transferred = 0;
  const int rc = libusb_interrupt_transfer(
      handle, kEpOut, const_cast<unsigned char*>(frame), 29,
      &transferred, kTimeoutMs);
  if (rc != LIBUSB_SUCCESS) return rc;
  return transferred == 29 ? LIBUSB_SUCCESS : LIBUSB_ERROR_IO;
}

int read_telemetry(libusb_device_handle* handle, uint8_t frame[43])
{
  int transferred = 0;
  const int rc = libusb_interrupt_transfer(
      handle, kEpIn, frame, 43, &transferred, kTimeoutMs);
  if (rc != LIBUSB_SUCCESS) return rc;
  return transferred == 43 ? LIBUSB_SUCCESS : LIBUSB_ERROR_IO;
}
```

当前数据率很低，同步 API 配合一个专用读线程已经足够。若后续追求更低抖动，可以改为
`libusb_submit_transfer()`，始终保持多个异步 IN 请求挂起，并在回调中只保留最新合法帧。

### 7.4 与现有 `Gimbal` 类的迁移边界

建议保留现有：

- `GimbalToVision` / `VisionToGimbal` 的业务字段；
- CRC 工具；
- `Gimbal::send()` 对外接口；
- 四元数队列与状态更新逻辑；
- 读线程和重连框架。

替换以下 transport 操作：

- `serial_.setPort/open/close` -> `usb_.open/close`；
- `serial_.write()` -> 单次 29 字节 Interrupt OUT；
- 两段式串口读取和搜帧头 -> 单次 43 字节 Interrupt IN；
- `com_port` 配置 -> VID/PID/interface/endpoint 配置。

建议配置形式：

```yaml
gimbal_transport: usb_interrupt
usb_vid: 0x34b7
usb_pid: 0xffff
usb_interface: 0
usb_ep_in: 0x81
usb_ep_out: 0x01
```

如果上位机希望在开发阶段保留源码级回退能力，可以暂时保留 `gimbal_transport: serial` 实现；但当前
下位机固件不再枚举 CDC 接口，运行时只能选择 `usb_interrupt`。

## 8. Linux 权限与部署

开发期 udev 规则示例：

```udev
SUBSYSTEM=="usb", ATTR{idVendor}=="34b7", ATTR{idProduct}=="ffff", MODE="0660", GROUP="plugdev"
```

修改规则后执行：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

如果上位机运行在容器或 OrbStack 虚拟 Linux 中，需要额外确认原始 USB Device 可以透传，并能访问
对应的 `/dev/bus/usb/*/*`。该固件不会创建 `/dev/ttyACM*`。

## 9. 错误处理要求

- `LIBUSB_ERROR_TIMEOUT`：读线程正常继续，不视为断线；
- `LIBUSB_ERROR_NO_DEVICE`：设备拔出，停止传输并进入重连；
- `LIBUSB_ERROR_ACCESS`：提示 udev/权限问题；
- `LIBUSB_ERROR_BUSY`：检查接口是否被其他进程占用；
- 重连后重新 claim interface，并清空旧状态队列；
- 读到非 43 字节包、错误帧头或 CRC 错误时丢弃；
- 写入返回长度不是 29 时视为失败；
- 多个线程可能调用 `send()` 时，对 OUT 传输和发送缓冲加锁。

下位机默认控制命令超时为 100 ms，由 `CONFIG_WBR_CONTROL_PC_LINK_COMMAND_TIMEOUT_MS` 配置。超过
该时间没有收到新的合法 29 字节命令，或者发生 USB reset/disconnect/reconfiguration，下位机会发布
新的 `mode=0` 空闲命令，避免继续保留旧控制状态。

## 10. 联调验收项

1. `lsusb -v` 能看到 Interface 0、Class `0xff`、端点 `0x81/0x01`，类型为 Interrupt；
2. `lsusb -t` 显示设备运行在 `480M`，端点最大包长为 64、轮询周期为 1 ms；
3. 连续读取只能得到 43 字节合法遥测帧；
4. 写入 29 字节合法命令后，下位机自瞄命令状态正确更新；
5. CRC 错误、长度错误和错误帧头不会更新控制状态；
6. 设备拔插后上位机可自动重连；
7. 连续运行压力测试，统计端到端延迟、P99 抖动、错误帧数和重连次数；
8. 先以 20 ms 间隔发送 2 帧，再发送 10 帧和 1000 帧；每次成功均要求 `transferred == 29`。
