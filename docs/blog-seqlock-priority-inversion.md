# 一次由 seqlock 引发的 2 秒后“全系统停摆”

> 这是一个很典型的实时系统 bug：表面看是 UART、DMA、VOFA、遥控器一起出问题，根因却藏在一个共享数据结构的读写时序里。

## 背景

项目里有几个关键线程同时运行：

- `remote_input` 从 UART 接收遥控器数据，发布最新遥控状态；
- `hi91_imu` 从 UART2 接收 HI91 IMU 数据，发布最新姿态；
- `chassis` 以 1 kHz 周期运行控制逻辑，读取遥控器和 IMU 数据；
- `oscilloscope` 把底盘调试数据通过 UART0 以 VOFA JustFloat 格式发出。

IMU 接收最开始通过一个独立测试程序验证：

```text
[uart_diag] serial@f0048000 irq4=0 irqc=0 irq_other=0 dma_rx=323 dma_err=0 flush_nz=323 flush_z=0
[hi91_stat] rx_Bps=82093 frame_hz=1000 sof_a55a=0 sof_5aa5=1000 byte_a5=1069 byte_5a=1074 parse_err=1 stop=0 drop=80 roll_mdeg=727 pitch_mdeg=-607 yaw_mdeg=-240
```

独立测试能稳定跑，HI91 每秒约 1000 帧，UART DMA 也持续有数据。因此第一反应很自然：IMU 串口本身应该没问题。

但接入主工程后，现象变得很诡异：

- 上电后 VOFA 和遥控器都能正常工作；
- 大约 2 秒后，VOFA 停止更新；
- 遥控器输入也停止响应；
- HI91 独立测试却不会停。

这个“2 秒后”后来变成了破案线索，因为主工程里的 HI91 模块启动后刚好延迟 2 秒才启用 UART RX。

## 最初的怀疑方向

看到 VOFA、遥控器、HI91 都和 UART 有关，第一轮排查自然会怀疑：

- UART2 的 DMA 或中断配置是否和测试程序不同；
- 多路 UART async RX 是否触发 HPM UART 驱动的多实例问题；
- VOFA 的 `uart_poll_out()` 是否被某个 async UART 影响；
- 遥控器 UART10 和 HI91 UART2 是否有 DMA 通道冲突；
- chassis 1 kHz 控制线程是否把低优先级线程饿死。

逐项对比后，关键信息是：

- HI91 独立测试和主工程里的 UART2 DTS 配置一致；
- UART2 使用的 DMA/IRQ 没有和遥控器 UART10 重叠；
- 问题不是一上来就出现，而是在 HI91 开始发布 sample 后出现；
- VOFA 和遥控器一起停，说明更像是调度层面被高优先级线程卡住，而不是单个外设失效。

于是排查方向从“UART 收不到数据”转向“某个高优先级线程是否进入了死循环或忙等”。

## 真正的根因

项目里多个模块之间通过 `SeqlockValue<T>` 传递最新状态。它的设计很简单：

```cpp
void write(const T& value)
{
    seq_.fetch_add(1, std::memory_order_release); // odd: writing
    data_ = value;
    seq_.fetch_add(1, std::memory_order_release); // even: stable
}

bool read(T& out) const
{
    uint32_t before;
    uint32_t after = 0;

    do {
        before = seq_.load(std::memory_order_acquire);
        if (before & 1U) {
            continue;
        }
        out = data_;
        after = seq_.load(std::memory_order_acquire);
    } while (before != after);

    return true;
}
```

这段代码在普通多线程程序里看起来没什么问题：写者把序号置成奇数表示“正在写”，写完再置回偶数；读者如果看到奇数，就继续等待。

但在实时系统里，这里藏着一个非常危险的条件：

> 如果低优先级写线程把 `seq_` 置成奇数后被高优先级读线程抢占，高优先级读线程会在 `read()` 里忙等，而低优先级写线程再也没有机会运行，也就永远不能把 `seq_` 置回偶数。

这就是一个由 seqlock 造成的优先级反转式死锁。

这次刚好满足了所有触发条件：

- `hi91_imu` 线程优先级较低；
- `chassis` 线程优先级较高，并且 1 kHz 读取 `latest_hi91_imu_sample`；
- HI91 模块 2 秒后开始接收并发布 IMU sample；
- 某次 HI91 写 sample 时，在 `seq_` 已经变成奇数但 `data_` 还没写完时被 `chassis` 抢占；
- `chassis` 看到奇数后在 `read()` 里原地 `continue`；
- 高优先级 `chassis` 一直运行，低优先级 HI91 无法恢复；
- 系统调度被卡住后，VOFA 和遥控器都表现为停止。

所以表面现象是“串口都停了”，实际是高优先级控制线程在 seqlock 读侧自旋，把系统锁死了。

## 修复

当前项目的修复方式是让写侧成为一个非常短的不可抢占临界区：

```cpp
void write(const T& value)
{
    const unsigned int key = irq_lock();
    seq_.fetch_add(1, std::memory_order_release); // odd: writing
    data_ = value;
    seq_.fetch_add(1, std::memory_order_release); // even: stable
    irq_unlock(key);
}
```

这样可以保证写者不会停在“序号为奇数”的中间状态。读者即使自旋，也只会遇到一个极短暂、可结束的写窗口。

这个修复适合当前场景，原因是：

- channel 中传递的是小结构体；
- 写入频率可控；
- 写侧临界区很短；
- 系统是单核 MCU，上锁成本明确；
- 比在读侧加入 `k_yield()` 更直接地消除了“写者被抢占后永远无法完成”的根因。

## 为什么独立测试没复现

独立 HI91 测试只做 UART RX、解析和统计打印，没有主工程里的高优先级 `chassis` 线程持续读取共享 IMU channel。

也就是说，独立测试验证的是：

- UART2 async RX 正常；
- DMA flush 正常；
- HI91 协议解析正常。

它没有覆盖：

- HI91 sample 发布到共享 channel；
- 高优先级控制线程读取该 channel；
- 低优先级写者和高优先级读者之间的调度交错。

这也是嵌入式系统里很常见的陷阱：外设单测通过，不代表接入主调度后仍然安全。

## 经验总结

这次问题有几个值得记住的点。

第一，seqlock 的读侧自旋不能无条件用于实时系统。  
如果读者优先级高于写者，而写者可能在写到一半时被抢占，就可能出现高优先级读者自旋、低优先级写者无法完成的死锁。

第二，“多个外设一起停”不一定是外设问题。  
当 VOFA、遥控器、IMU 看起来同时异常时，真正该怀疑的是调度、锁、优先级、临界区和共享数据结构。

第三，时间点是很重要的线索。  
这次“2 秒后停”一开始像是 UART 或 DMA 跑一段时间后出问题，但最后发现它刚好对应 HI91 模块的启动延迟。

第四，独立测试只能证明局部链路。  
HI91 单测证明了 UART 和解析没问题，但不能证明主工程里的数据发布、线程优先级和控制循环都没问题。

第五，实时系统里的“无锁”不是免费午餐。  
很多 lock-free 或 wait-free 风格的结构，在抢占式实时调度里需要重新审视。无锁不等于不会卡死，尤其当某一侧会忙等另一侧完成时。

## 最后

这个 bug 的有趣之处在于，它一开始几乎把所有注意力都吸到了 UART/DMA 上：独立测试稳定，主工程却在 2 秒后 VOFA 和遥控器一起停。真正的根因不是串口，而是一个共享 channel 的 seqlock 写者被抢占后，读者在高优先级线程里忙等。

这类问题很适合用一句话提醒自己：

> 在抢占式实时系统里，只要读者会自旋等待写者完成，就必须保证写者真的有机会完成。

