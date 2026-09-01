# Zephyr Kconfig、`printk`、Logging 与 Backend：从配置到 UART0 的完整机制

> 适用仓库：`wbr_control` 及其 west 工作区
>
> 分析基线：Zephyr 3.7.0，工作区 Zephyr 提交 `8c6db8fa778cf90eb93e823e513a1ab97703711d`（tag `zsg_v0.7.0`）
>
> 目标板：`dust-hpm6750` / HPM6750
>
> 最后核对：2026-08-25
>
> 说明：本文描述的是当前工作区中实际存在的 Zephyr 3.7.0 和 HPMicro 驱动，不假定最新版上游行为完全相同。

## 1. 先给结论

当前配置之所以让人感觉“明明写了 `n`，最后还是 `y`”“关闭了 LOG，UART 却还可能输出”，根本原因是把四个层次混在了一起：

1. **能力定义**：`Kconfig` 定义符号、依赖、默认值、choice 和反向依赖。
2. **构建请求**：板级 `*_defconfig`、`prj.conf`、`EXTRA_CONF_FILE` 只是向 Kconfig 求解器提交期望值。
3. **最终配置**：求解后的唯一事实是构建目录里的 `zephyr/.config`，C/C++ 实际读取的是由它生成的 `autoconf.h`。
4. **物理输出路径**：Kconfig 决定代码是否存在；Devicetree 的 `chosen` 决定 console/logger 指向哪个设备；具体 UART API 又决定轮询、IRQ 还是 DMA。

对当前 `wbr_control`，最重要的实测结论如下：

- 板级 DTS 把 `zephyr,console` 和 `zephyr,shell-uart` 都指定为 `uart0`。
- 当前 `app.overlay` 使 `uart0` 为 `okay`、波特率为 921600，并关闭 `cherryusb_usb0`。
- 未定义 `zephyr,log-uart`，所以 Zephyr UART log backend 回退使用 `zephyr,console`，即 UART0。
- `printk()` 在 `CONFIG_LOG_PRINTK=n` 时，经 UART console hook 逐字符调用 `uart_poll_out(uart0, c)`。
- `printk()` 在 `CONFIG_LOG_PRINTK=y` 时不再直接走 console，而是变成 logger 的无条件 raw message，再由 active backend 输出。
- `LOG_*` 与 `CONFIG_PRINTK` 没有必然关系；普通 deferred/immediate logger 不依赖 `printk`。只有 minimal logging 等特殊路径会复用 `printk`。
- `CONFIG_LOG=y` 只表示 logger core 存在，不表示 UART 上一定有日志。判断 UART0 是否被 logger 占用，应看 `CONFIG_LOG_BACKEND_UART=y` 以及 backend 是否 active。
- 当前 `CONFIG_DEBUG_COREDUMP_BACKEND_LOGGING=n` **没有真正关闭该 choice**。因为没有选择另一个 coredump backend，choice 回到默认的 logging backend，最终仍是 `CONFIG_DEBUG_COREDUMP_BACKEND_LOGGING=y`，并反向 `select LOG`。
- 当前基础配置已改为 `CONFIG_BOOT_BANNER=n`；因此 LOG-only 和 OSCILLOSCOPE-only 模式能够真正保持 `CONFIG_PRINTK=n`。如果某个诊断 fragment 重新打开 boot banner，它仍会通过 `select PRINTK` 强制启用 `printk`。
- 当前默认是 `WBR_CONTROL_UART0_OUTPUT_OSCILLOSCOPE=y`：UART console、direct `printk`、boot banner、UART log backend 和 `LOG_PRINTK` 均关闭，UART0 仅由 VOFA async/DMA owner 使用。
- “全路径诊断”同时让 raw polling UART、direct `printk`、UART logger 和 VOFA async DMA 使用 UART0。这能用于观察路径是否存活，但它们没有一个覆盖所有生产者的统一仲裁，输出交错或二进制帧损坏是预期风险，不应作为生产配置。

一句话记忆：

> `.conf` 是请求，`.config` 是裁决；`LOG` 是管线，backend 是出口；console 是设备选择和 hook，UART 是物理通道。

## 2. 配置系统全景：文件各自负责什么

### 2.1 `Kconfig`：定义“可配置世界”

`Kconfig` 不是赋值文件。它定义：

- 符号的类型：`bool`、`tristate`、`int`、`hex`、`string`；
- 是否向用户显示：是否有 prompt；
- `depends on`：本符号可见/可选的正向条件；
- `default`：没有有效用户值时使用的默认值；
- `select`：当前符号为 `y` 时，反向强制另一个符号为 `y`；
- `imply`：较弱的建议性反向依赖；
- `choice`：多个候选中恰好选择一个；
- `source`/`rsource`：把其他 Kconfig 文件纳入同一符号图。

本项目顶层 `Kconfig` 首先：

```kconfig
source "Kconfig.zephyr"
```

它把 Zephyr、board、SoC、driver、subsystem 和 module 的 Kconfig 全部纳入，然后定义 `WBR_CONTROL_*` 应用符号。也就是说，应用 choice 与 Zephyr 的 `PRINTK`、`LOG`、`UART_CONSOLE` 等符号在同一次求解中互相影响。

### 2.2 `*_defconfig`：板级初始请求

DUST-HPM6750 的：

```text
hpm_support/boards/hpmicro/dust-hpm6750/dust-hpm6750_defconfig
```

包含：

```conf
CONFIG_CONSOLE=y
CONFIG_UART_CONSOLE=y
CONFIG_SERIAL=y
...
```

它给板子一个可用的默认基线，但优先级不是最高。应用配置可以请求覆盖它，只要依赖和反向依赖允许。

不要混淆以下两类文件：

- `<board>_defconfig`：Kconfig fragment，语法与 `prj.conf` 类似；
- `Kconfig.defconfig`：Kconfig 语言，用于补充符号默认值或 choice 默认值。

### 2.3 `prj.conf`：应用的常驻请求

默认情况下，`wbr_control/prj.conf` 是应用主配置 fragment。它适合放所有构建共有的配置，而不是临时诊断模式。

当前文件同时包含：

- C++、FPU、Zbus、filesystem 等基础能力；
- console/logger/UART 输出策略；
- coredump；
- 应用模块开关；
- 临时的 UART0 全路径诊断选择。

这正是当前混乱的来源之一：稳定产品能力和临时诊断场景被写进同一基础文件。

### 2.4 `config/*.conf` 与 `EXTRA_CONF_FILE`

命令：

```sh
west build -p always -b dust-hpm6750 -d build -- \
  -DEXTRA_CONF_FILE=config/printk_log.conf
```

表示保留默认 `prj.conf`，再追加一个 fragment。后出现的普通用户赋值通常覆盖前面的赋值。对 choice，如果后面的 fragment 把另一个 choice 成员设为 `y`，其他成员会自动变成 `n`。

但“后写的 `n` 一定获胜”是错误理解。以下情况都能让最终值与文本请求不同：

- 符号依赖不满足；
- 符号不可见，用户赋值本来就无效；
- 另一个符号通过 `select` 强制它为 `y`；
- choice 必须选一个成员，全部写 `n` 后仍会使用默认成员；
- 缓存中的 `.config` 仍被复用；
- 使用了不同的 `CONF_FILE`、board、overlay 或 build directory。

### 2.5 `app.overlay`：不是 Kconfig

Devicetree 描述“有哪些硬件、地址是什么、设备是否启用、哪个设备承担特定角色”。Kconfig 描述“编译哪些软件能力”。两者缺一不可。

本项目当前 overlay：

```dts
&uart0 {
    status = "okay";
    current-speed = <921600>;
};

&cherryusb_usb0 {
    status = "disabled";
};
```

而板级 DTS 已经定义：

```dts
chosen {
    zephyr,console = &uart0;
    zephyr,shell-uart = &uart0;
};
```

所以这里形成两个独立条件：

- `CONFIG_UART_CONSOLE=y`：编译 UART console 软件；
- `zephyr,console = &uart0` 且 `uart0 status = "okay"`：告诉软件使用哪个已启用设备。

只有前者，没有可用 chosen 设备，初始化会失败；只有后者，没有编译 console，也不会自动产生 console 输出。

### 2.6 CMake、`.config` 与 `autoconf.h`

构建时的关键链路是：

```text
board *_defconfig
      + CMake cache 中的 CONFIG_*
      + prj.conf / CONF_FILE
      + board/soc fragments（若存在）
      + EXTRA_CONF_FILE
                    │
                    ▼
              Kconfig 求解器
                    │
          ┌─────────┴──────────┐
          ▼                    ▼
build/zephyr/.config   build/zephyr/include/generated/zephyr/autoconf.h
          │                    │
          │                    └── C/C++ 中的 CONFIG_* 宏
          └── 人类排查最终值的第一依据
```

CMake 再根据 `CONFIG_*` 条件决定源文件是否参与编译。例如：

- `CONFIG_LOG_BACKEND_UART=y` 才编译 `log_backend_uart.c`；
- `CONFIG_UART_CONSOLE=y` 才编译 UART console driver；
- `CONFIG_PRINTK=y` 才保留真正的 `printk()` 实现，否则头文件内联为空操作。

## 3. 必须理解的 Kconfig 语义

### 3.1 `depends on` 与 `select` 的方向相反

```kconfig
config LOG_BACKEND_UART
    bool "UART backend"
    depends on UART_CONSOLE
```

意思是：没有 `UART_CONSOLE`，用户不能有效启用 UART backend。

项目 choice 中则写了：

```kconfig
config WBR_CONTROL_UART0_OUTPUT_LOG
    bool "Zephyr logger only"
    select SERIAL
    select CONSOLE
    select UART_CONSOLE
    select LOG
    select LOG_BACKEND_UART
```

意思是：选择这个模式后，反向强制相关符号为 `y`。

`select` 很强，它通常不适合被当作“可被 fragment 再关闭”的默认值。当前 `config/*.conf` 已各自只选择一个应用 mode，派生值全部由应用 `Kconfig` 定义，避免 fragment 与 Kconfig 形成两个配置真相。

### 3.2 choice 不是多个互不相关的 bool

`WBR_CONTROL_UART0_OUTPUT_MODE` 是 choice：

```text
DISABLED / OSCILLOSCOPE / PRINTK / LOG / PRINTK_AND_LOG / ALL_DIAGNOSTIC
```

最终只能有一个成员为 `y`。如果 `prj.conf` 先选 `ALL_DIAGNOSTIC=y`，`EXTRA_CONF_FILE` 后选 `LOG=y`，最终是 `LOG=y`、`ALL_DIAGNOSTIC=n`。

如果一个 choice 的所有成员都被写成 `n`，不是“choice 被关闭”，而是求解器选择该 choice 的默认成员。要允许“整个 choice 不选”，必须让 choice 自身带依赖、可选语义，或提供明确的 `DISABLED` 成员；本项目采用的是最后一种。

### 3.3 为什么 `DEBUG_COREDUMP_BACKEND_LOGGING=n` 没生效

Zephyr 3.7 的 coredump backend 是 choice：

```text
LOGGING（默认） / FLASH_PARTITION / INTEL_ADSP_MEM_WINDOW / OTHER
```

当前只写：

```conf
CONFIG_DEBUG_COREDUMP=y
CONFIG_DEBUG_COREDUMP_BACKEND_LOGGING=n
```

却没有选择另一个成员。choice 仍必须选一个，于是回到默认 `LOGGING=y`。该成员又：

```kconfig
select LOG
```

所以所有实测模式最后都有 `CONFIG_LOG=y`。

若目标是不需要 coredump，关闭 `CONFIG_DEBUG_COREDUMP`；若目标是保留 coredump 但不用 logger，必须明确选择一个真正可用的其他 backend。`OTHER=y` 只表示项目自己提供实现，不能只选符号而不实现代码。

### 3.4 为什么 `PRINTK=n` 没生效

Zephyr 的 `BOOT_BANNER` 定义包含：

```kconfig
config BOOT_BANNER
    bool "Boot banner"
    default y
    select PRINTK
    select EARLY_CONSOLE
```

当前基础 `prj.conf` 设置 `CONFIG_BOOT_BANNER=n`，因此 LOG-only 和 OSCILLOSCOPE-only 模式可以保持 `PRINTK=n`。如果以后重新打开 boot banner，它仍会通过 `select PRINTK` 强制启用 `printk`。

要真正编译掉 `printk`，必须同时消除所有反向依赖；在当前项目最直接的是 `CONFIG_BOOT_BANNER=n`，再用 `menuconfig` 或 Kconfig warning 检查是否还有其他 selector。

### 3.5 缓存为何会制造“幽灵配置”

Zephyr 会在 `.config` 仍被认为是最新时复用它。不同场景反复使用同一个 `build/`，又变更 `EXTRA_CONF_FILE`，很容易让人误读。

可靠做法：

```sh
west build -p always -b dust-hpm6750 -d build-log -- \
  -DEXTRA_CONF_FILE=config/log.conf

grep -E 'CONFIG_(LOG|PRINTK|UART_CONSOLE|WBR_CONTROL_UART0)' \
  build-log/zephyr/.config
```

每个场景使用独立 build directory，比所有模式共享 `build/` 更容易复现和审计。

## 4. Console、UART driver、`stdout`、Shell、Logger 的边界

这些概念经常都“从串口看到文字”，但不是同一层。

| 概念 | 主要职责 | 当前项目的关键开关/选择 |
|---|---|---|
| UART driver | 提供 `uart_poll_out`、IRQ API、async/DMA API | `CONFIG_SERIAL`、`UART_ASYNC_API`、`UART_INTERRUPT_DRIVEN` |
| Console driver | 安装 `printk`/`stdout` 字符 hook，选择 console 设备 | `CONFIG_CONSOLE`、`CONFIG_UART_CONSOLE`、`zephyr,console=&uart0` |
| `printk` | 低层调试格式化 API，可直接逐字符输出或重定向到 logger | `CONFIG_PRINTK`、`CONFIG_LOG_PRINTK` |
| libc `printf`/stdout | 标准 C 库输出；可安装到 console hook | `CONFIG_STDOUT_CONSOLE`，当前为 `n` |
| Logging core | 日志源、过滤、消息封装、buffer、调度、backend fan-out | `CONFIG_LOG` |
| Log backend | logger 的实际出口 | 当前主要是 `CONFIG_LOG_BACKEND_UART` |
| Shell | 命令解析与交互，可拥有自己的 serial/RTT backend | 当前 `CONFIG_SHELL=n` |
| VOFA oscilloscope | 应用自定义二进制协议，直接调用 UART async API | `WBR_CONTROL_MODULE_OSCILLOSCOPE` |

几个容易出错的判断：

- `SERIAL=y` 不等于 console 已打开；它只让 UART driver 能存在。
- `CONSOLE=y` 不等于一定是 UART console；也可能是 RTT、RAM、semihost 等 console。
- `UART_CONSOLE=y` 不等于 logger UART backend 已打开。
- `LOG=y` 不等于任何 backend 已打开。
- `SHELL_BACKEND_SERIAL` 不是 `LOG_BACKEND_UART`。Shell 可以接收日志并格式化显示，但它有自己的 backend 和线程模型。
- `printf` 与 `printk` 不是同一个 API。当前 `STDOUT_CONSOLE=n`，不要因为 `printk` 可见就假设 `printf` 也会出现在 UART0。

## 5. `printk()` 从调用到引脚的完整路径

### 5.1 编译关闭时

`include/zephyr/sys/printk.h` 在 `CONFIG_PRINTK=n` 时把 `printk()` 定义为空 inline 函数。调用点通常没有运行时输出成本，格式字符串也可能被链接器消除。

### 5.2 直接输出：`CONFIG_LOG_PRINTK=n`

调用链：

```text
printk(fmt, ...)
  -> vprintk()
  -> cbvprintf(char_out, ...)
  -> _char_out(c)
  -> UART console 安装的 console_out(c)
  -> 换行时补 '\r'
  -> uart_poll_out(uart0, c)
  -> HPM uart_hpm_poll_out()
  -> HPM SDK uart_send_byte()
  -> UART0 TX 引脚
```

关键源码：

- `zephyr/lib/os/printk.c`
- `zephyr/drivers/console/uart_console.c`
- `hpm_support/drivers/serial/uart_hpmicro.c`

UART console 在初始化时调用 `__printk_hook_install(console_out)`。在此之前 `_char_out` 默认指向 weak `arch_printk_char_out()`；当前工作区没有找到 HPM 平台覆盖它，因此没有已安装 hook 时输出会被吞掉。

直接 `printk` 的特性：

- 格式化与发送发生在调用者上下文；
- 当前 HPM path 是 polling，一字节一字节等硬件发送；
- 在 ISR 或高优先级控制线程里调用会直接增加延迟和 jitter；
- `CONFIG_PRINTK_SYNC` 只在启用时用 spinlock 序列化不同 `printk` 调用；当前最终配置中它是 `n`；
- 即使打开 `PRINTK_SYNC`，它也不会统一保护 logger、raw UART 和 VOFA DMA；
- 支持的格式子集由 `cbprintf`/libc 配置决定，不应默认等同于完整桌面 `printf`。

### 5.3 经 logger：`CONFIG_LOG_PRINTK=y`

`vprintk()` 入口首先检查 `CONFIG_LOG_PRINTK`：

```text
printk(fmt, ...)
  -> vprintk()
  -> z_log_vprintk()
  -> 创建 LOG_LEVEL_INTERNAL_RAW_STRING 消息
  -> logger core/buffer
  -> active backend(s)
```

这时 `printk` 不再直接调用 console hook。其消息是 level 0 的无条件消息，不能用普通 ERR/WRN/INF/DBG level 过滤掉。

收益：

- 与 `LOG_*` 共享 logger 的串行处理和 backend，避免 direct printk 与 logger 在同一 transport 上逐字符交错；
- 可被多个 active backend 分发；
- panic 时能跟随 logger 进入同步 flush。

行为变化：

- deferred mode 下，`printk` 不再“调用即到线”，而是等待 log processing thread；
- logger buffer 满时可能按 overflow 策略丢旧消息；
- 启动早期、logger 尚未处理或 backend 未 active 时，观察到的时序不同；
- 没有 active backend 时，消息不会神奇地出现在 console 上。

### 5.4 `LOG_PRINTK()`、`LOG_RAW()` 与 `printk()`

- `printk()`：是否经过 logger 由全局 `CONFIG_LOG_PRINTK` 决定。
- `LOG_PRINTK()`：显式经 logging infrastructure，保留 printk 风格，并处理换行规则。
- `LOG_RAW()`：显式经 logger，但内容原样输出，不自动追加 prefix、颜色或换行。

## 6. Zephyr Logging 从宏到 backend 的完整路径

### 6.1 日志源注册

单文件模块常见写法：

```cpp
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_main, LOG_LEVEL_INF);
```

多文件组成同一模块时：

- 恰好一个 `.c/.cpp` 使用 `LOG_MODULE_REGISTER`；
- 其他文件使用 `LOG_MODULE_DECLARE`；
- 模块名必须唯一。

本项目多个模块都显式注册为 `LOG_LEVEL_INF`。这意味着：

- `LOG_ERR`、`LOG_WRN`、`LOG_INF` 会编译；
- `LOG_DBG` 在这些 translation unit 中通常在编译期消失；
- `CONFIG_LOG_DEFAULT_LEVEL=3` 只影响没有显式 level 的模块；
- 若要临时全局拉高到 DEBUG，可理解 `LOG_OVERRIDE_LEVEL` 与 `LOG_MAX_LEVEL` 后再配置，不能只改 default level 并期待显式 INF 模块变化。

### 6.2 四个等级与过滤

数值语义：

| 数值 | 等级 | 包含范围 |
|---:|---|---|
| 0 | OFF | 全关 |
| 1 | ERR | 仅错误 |
| 2 | WRN | ERR + WRN |
| 3 | INF | ERR + WRN + INF |
| 4 | DBG | ERR + WRN + INF + DBG |

过滤有两道：

1. **编译期过滤**：决定日志调用和相关数据是否进入镜像，影响 ROM 和运行时开销。
2. **运行时过滤**：`CONFIG_LOG_RUNTIME_FILTERING=y` 时，可按 source、instance、backend 调整；日志代码仍在镜像里。

`CONFIG_LOG_MAX_LEVEL` 是整个系统的编译上限；`LOG_OVERRIDE_LEVEL` 能把模块最低编译等级向更详细方向推高；模块声明的 level 决定该 source 的常规上限。当前实测：

```text
LOG_DEFAULT_LEVEL=3
LOG_OVERRIDE_LEVEL=0
LOG_MAX_LEVEL=4
LOG_RUNTIME_FILTERING=n
```

### 6.3 deferred mode

当前最终配置采用：

```text
CONFIG_LOG_MODE_DEFERRED=y
CONFIG_LOG_BUFFER_SIZE=1024
CONFIG_LOG_MODE_OVERFLOW=y
CONFIG_LOG_PROCESS_THREAD=y
CONFIG_LOG_PROCESS_TRIGGER_THRESHOLD=10
CONFIG_LOG_PROCESS_THREAD_SLEEP_MS=1000
CONFIG_LOG_PROCESS_THREAD_STACK_SIZE=768
```

路径：

```text
LOG_INF(...)
  -> 编译期/运行时过滤
  -> cbprintf 参数打包 + source/domain/level/timestamp
  -> MPSC packet buffer
  -> 唤醒或等待 log processing thread
  -> core 逐个遍历 active backend
  -> backend 过滤、格式化、发送
```

优点是调用现场不做昂贵的文本格式化和 transport 发送，适合实时系统。代价是：

- 需要 RAM buffer 和 processing thread stack；
- 输出有延迟；
- 参数生命周期必须符合 deferred logging 规则，瞬态字符串需要复制或使用相应宏；
- 生产速度超过消费速度时会丢日志；当前 overflow 策略丢最旧消息以容纳新消息；
- reset 前未 flush 的日志会丢失。

### 6.4 immediate mode

`CONFIG_LOG_MODE_IMMEDIATE=y` 时，日志在调用者上下文立即过滤、格式化并送到 backend。

它适合：

- 极早期 bring-up；
- 无线程环境；
- 必须立即看到日志且能接受延迟。

它不适合高频实时控制路径，因为 backend 的 polling、网络、文件系统等延迟会直接记到调用者头上；ISR 中的代价尤其危险。网络 backend 明确不允许 immediate mode。

### 6.5 minimal mode

`CONFIG_LOG_MODE_MINIMAL=y` 追求最小 footprint：

- `select PRINTK`；
- 普通 LOG 宏最终使用 `printk`；
- 支持编译期过滤；
- 没有正常 logger 的 timestamp、prefix、颜色、runtime filtering 和异步处理。

因此“启用 LOG 但想完全去掉 PRINTK”时，不能使用 minimal mode。

### 6.6 panic mode

系统 fatal/panic 后调度器和普通异步处理可能不再可靠。调用 `log_panic()` 后：

1. 通知所有 active backend 进入 panic；
2. flush 已缓存消息；
3. 后续日志改为 blocking/synchronous 处理。

UART backend 在 panic 中会强制使用 polling path，即使正常时配置了 async backend。

## 7. Backend 到底是什么

Backend 是 logging core 的消费者和物理/逻辑出口。一个 logger 可以同时激活多个 backend，core 会 fan-out；每个 backend 可有独立 runtime filter 和 output format。

Backend 生命周期通常包括：

- build-time 注册；
- init；
- autostart 或应用调用 `log_backend_enable()`；
- `process()` 消费消息；
- `dropped()` 报告丢失；
- `panic()` 切到灾难路径；
- 可选的 format 切换。

“编译了 backend”与“backend 已 active”也不是同一件事。`*_AUTOSTART=n` 时必须由应用在 transport 就绪后手动启用。

### 7.1 UART backend：本项目最相关

Zephyr 3.7 的依赖：

```text
LOG_BACKEND_UART depends on UART_CONSOLE
```

设备选择：

- 若 DTS 有 `zephyr,log-uart` chosen，则使用其中指定的 UART；
- 否则使用 `zephyr,console`；
- 本项目没有 `zephyr,log-uart`，所以使用 UART0。

当前最终配置没有启用 `CONFIG_LOG_BACKEND_UART_ASYNC`，因此 backend 的 `char_out()` 对每段数据逐字节 `uart_poll_out()`。`UART_ASYNC_API=y` 只表示 driver 提供 async 能力，不会自动让 log backend 使用它。

若启用 `LOG_BACKEND_UART_ASYNC=y`：

- backend 调用 `uart_tx()`；
- 等待 `UART_TX_DONE` callback 释放 semaphore；
- 初始化 callback 失败时会回退 polling；
- panic 时仍回退 polling。

这里对本项目尤其危险：VOFA 模块也会对 UART0 调用 `uart_callback_set()` 和 `uart_tx()`。Zephyr UART async API 通常只有一个用户 callback；两个独立 owner 会覆盖 callback 或争用 TX 状态。因此生产设计必须让 UART0 只有一个 async owner，不能简单同时打开 async log backend 和 VOFA。

### 7.2 RTT backend

`LOG_BACKEND_RTT` 通过 SEGGER J-Link RTT 的 RAM up-buffer 输出，不占 UART 引脚。常见模式：

- DROP：空间不足丢整条消息；
- BLOCK：等待 host 腾出空间；
- OVERWRITE：覆盖最旧内容。

优点是 bring-up 时不占串口，速度通常较高；限制是依赖调试器/host 持续读取，BLOCK 模式在 host 未连接时可能影响实时性。RTT console 与 RTT logger 共用 terminal buffer 时，Zephyr 会强制 `LOG_PRINTK` 以减少数据破坏风险。

对当前 UART0 还要承担 VOFA 的项目，RTT 是文本诊断的优先候选，但要先确认 HPM6750/J-Link 工具链和 RAM 放置工作正常。

### 7.3 SWO backend

`LOG_BACKEND_SWO` 使用 ARM CoreSight ITM/SWO 单线跟踪输出，依赖 `HAS_SWO`。它不适用于当前 RISC-V HPM6750 目标。

### 7.4 Filesystem backend

`LOG_BACKEND_FS` 把文本/格式化日志写入已挂载文件系统，支持：

- 目录和文件前缀；
- 单文件大小；
- 文件数量上限；
- 覆盖最旧文件或空间满后丢新消息；
- autostart。

文件系统未挂载前消息会被丢弃。当前项目有 LittleFS 能力，但 runtime init 默认关闭；即使打开 FS backend，也必须安排 mount 与 backend activation 的正确时序，并评估 flash 擦写、wear、控制周期干扰和 fatal 时的可用性。

### 7.5 Network backend

`LOG_BACKEND_NET` 以 RFC 5424 syslog 经 UDP/TCP 发送，可配置结构化字段、server、buffer 和 DHCP log-server option。

限制：

- 依赖 networking 与 UDP/TCP；
- 不支持 immediate mode；
- 网络未建立时不应盲目 autostart，否则 logging thread 可能阻塞；
- 不适合当前无网络 transport 的默认目标。

### 7.6 BLE backend

`LOG_BACKEND_BLE` 通过兼容 Nordic UART Service 的 BLE notification 发送，是 experimental backend；依赖 Bluetooth，并要求 log processing thread stack 至少 2048 字节。适合无线维护，不适合没有 BLE stack 的当前板级基线。

### 7.7 Spinel backend

`LOG_BACKEND_SPINEL` 通过 OpenThread 专用 Spinel 协议封装日志，依赖 OpenThread L2，并要求 UART backend 关闭或使用另一 UART，避免串行帧互相干扰。它不是通用串口文本 backend。

### 7.8 平台专用 backend

本地 Zephyr 3.7 还包含：

- `ADSP`：Intel ADSP host trace protocol buffer；
- `ADSP_MTRACE`：Intel ADSP/SOF Linux mtrace SRAM window；
- `EFI_CONSOLE`：x86 EFI console；
- `NATIVE_POSIX`：native POSIX 仿真目标；
- `XTENSA_SIM`：Xtensa simulator；
- `IPC_SERVICE`：多 domain/多核日志传输 backend。

它们都受架构或 subsystem 依赖限制，不是当前 HPM6750 可随意替换的出口。

### 7.9 Frontend、link 与 backend 不要混淆

默认 frontend 负责在日志调用现场过滤、封装和提交消息。自定义 `LOG_FRONTEND` 可以在函数层截获日志；`LOG_FRONTEND_ONLY` 表示完全不用 backend。

多 domain logging 中，link 负责在 domain 之间传消息，backend 负责最终消费/输出。它们解决的问题不同。

## 8. 输出格式：text、dictionary、Sys-T、custom

多个 backend 使用公共 format Kconfig template，可选择：

- **TEXT**：设备直接输出人可读字符串；最直观，但目标端格式化开销和带宽较大。
- **DICTIONARY**：目标输出紧凑的二进制/编码消息，host 结合 ELF dictionary 离线还原；节省带宽和目标格式化成本，但固件与 dictionary 必须精确匹配。
- **MIPI Sys-T**：标准化的软件跟踪格式，需相应解析工具。
- **CUSTOM**：应用提供自定义 formatter。

UART dictionary 还可选 binary 或 hexadecimal transport。若 UART 上混入 bootloader、raw UART 或 VOFA 字节，dictionary parser 同样会被破坏；格式更紧凑不代表自动解决多 owner 仲裁问题。

## 9. 当前项目 UART0 的真实数据路径

### 9.1 物理和 Devicetree

```text
dust-hpm6750.dts
  chosen zephyr,console  ─┐
  chosen zephyr,shell-uart├──> uart0 @ 921600
app.overlay               ┘        │
                                  ▼
                         HPM UART driver / PY06 TX
```

### 9.2 四类生产者

```text
RawUart0Write() ───────────── uart_poll_out ───────────┐
                                                       │
printk(), LOG_PRINTK=n ─ console hook ─ uart_poll_out ─┤
                                                       ├─> 同一个 UART0
LOG_*/printk redirected ─ log core ─ UART backend ─────┤
                                                       │
VOFA oscilloscope ─ Encode JustFloat ─ uart_tx/DMA ────┘
```

全诊断模式中的三条文本路径并不完全独立：raw 与 direct printk 最终都调用同一个 UART driver polling API，logger 当前也使用 polling。VOFA 则是 async DMA。它们能证明不同上层调用发生了，但不能证明 transport 有统一并发保护。

### 9.3 HPM driver 的具体行为

- polling：`uart_hpm_poll_out()` 直接调用 HPM SDK `uart_send_byte()`；
- async TX：配置 DMA、调用 `uart_tx()`，完成后通过唯一 UART callback 通知；
- interrupt-driven API 另有 FIFO/IRQ 实现；
- 打开 `UART_ASYNC_API` 和 `UART_INTERRUPT_DRIVEN` 只是同时编译能力，不会自动替所有调用选择一种模式。

同一外设上混用 polling 与正在进行的 DMA TX，是否不丢字节/不重排不能由 Zephyr logging core 保证；这需要 UART driver 和应用 owner 设计明确的仲裁。

## 10. 各场景的实际求解结果

以下结果来自 2026-08-25 使用独立临时 build directory、`-p always` 对当前工作树重新求解 `.config`。不是根据 fragment 名称猜测。

| 最终值 | 默认/base | `printk.conf` | `log.conf` | `printk_log.conf` | `oscilloscope.conf` |
|---|---:|---:|---:|---:|---:|
| 应用 choice | OSCILLOSCOPE | PRINTK | LOG | PRINTK_AND_LOG | OSCILLOSCOPE |
| `SERIAL` | y | y | y | y | y |
| `CONSOLE` | n | y | y | y | n |
| `UART_CONSOLE` | n | y | y | y | n |
| `PRINTK` | n | y | n | y | n |
| `BOOT_BANNER` | n | n | n | n | n |
| `LOG` | y | y | y | y | y |
| `LOG_PRINTK` | n | n | n | y | n |
| `LOG_BACKEND_UART` | n | n | y | y | n |
| oscilloscope module | y | n | n | n | y |
| UART0 上预期内容 | 仅 VOFA | direct printk | 仅 LOG | printk 经 logger + LOG | 仅 VOFA |

所有列的 `LOG=y` 仍是 coredump logging backend choice 的结果。默认和 `oscilloscope.conf` 中没有 active log output backend：项目中的 `LOG_*` 调用仍会按编译期 level 生成代码，deferred logger 的 buffer/thread 也仍存在，消息却不会送到 UART0；依赖 logging backend 的 coredump同样没有可靠物理出口。这不是等价于 `LOG=n` 的零成本状态，但它不会污染 VOFA 字节流。

默认 VOFA-only、全应用模块（文件系统和 MAVLink shell 除外）镜像已完成完整编译链接，使用 ROM 138556 B、RAM 103952 B。链接输入中没有 `uart_console.c` 或 `log_backend_uart.c`，ELF 中也没有 `printk`、UART console init 或 UART log backend symbol。

## 11. 当前配置中的具体问题清单

### P0（仅 ALL_DIAGNOSTIC）：生产 owner 不唯一

`ALL_DIAGNOSTIC` 同时使用 polling text 与 VOFA DMA。此模式只能用于短期硬件诊断，不能成为默认产品配置。

### 已修复：基础配置与 README 冲突

基础 `prj.conf` 与 `config/README.md` 现在都把默认构建定义为 VOFA-only；其他输出模式通过 `EXTRA_CONF_FILE` 显式选择。

### P1：coredump backend 被错误“关闭”

只写 choice 默认成员为 `n` 不足以关闭 choice，导致 `LOG` 在所有模式被强制打开；而没有 log output backend 的模式中，logging coredump 也没有可靠物理出口。

### 已修复：`BOOT_BANNER` 破坏“无 printk”语义

基础配置已关闭 `BOOT_BANNER`。重新求解后，`log.conf` 和 `oscilloscope.conf` 都是 `PRINTK=n`；只有明确需要 printk 或显式打开 boot banner 的诊断模式才会编译它。

### 已修复：场景 fragment 重复派生配置

四个场景 fragment 现在都只设置一个 `WBR_CONTROL_UART0_OUTPUT_*` choice 成员；`SERIAL/CONSOLE/UART_CONSOLE/PRINTK/LOG/LOG_BACKEND_UART/LOG_PRINTK` 等派生关系集中在应用 `Kconfig` 中。

### P1：UART async callback owner 冲突风险

VOFA 注册 UART0 callback。如果未来仅因为 `UART_ASYNC_API=y` 就启用 `LOG_BACKEND_UART_ASYNC=y`，log backend 也会注册 callback；后注册者可能覆盖前者。

### P2：日志在实时控制路径中的预算未显式化

Deferred logging 降低调用现场开销，但 buffer 分配、参数打包仍有成本；direct printk/polling 更会阻塞。1 kHz chassis/control 线程不应做高频文本输出，错误洪泛也要限频。

### P2：现有 `build/` 曾与当前配置不一致

检查中发现仓库现有 `build/zephyr/.config` 与当前文本配置存在不一致迹象。共享 build directory 和未 pristine rebuild 会继续放大误解。

## 12. 推荐的配置架构

当前配置已开始按以下结构收敛；仍未解决的风险会在对应小节明确标出。

### 12.1 一条规则：一个物理 UART 只有一个 owner

建议生产场景：

| 场景 | UART0 owner | 文本诊断出口 |
|---|---|---|
| 正常控制 + VOFA | VOFA async module | RTT，或关闭文本日志 |
| 串口文本调试 | Zephyr logger UART backend | 关闭 VOFA/raw/direct printk |
| 极早期 bring-up | direct printk UART console | 关闭 logger UART 和 VOFA |
| 全路径诊断 | 明确标注不保证流完整性 | 仅短期人工诊断 |

若必须把文本和 VOFA 都走 UART0，应实现一个真正的单 owner transport/multiplexer：统一 queue、帧协议、发送状态机和 callback，而不是让多个 subsystem 各自调用 UART API。

### 12.2 `prj.conf` 只放共同基线

共同基线应明确：

- 是否需要 coredump；
- 是否允许 boot banner；
- logger 默认 mode、buffer、level；
- UART driver 能力；
- 产品默认模块。

不要把临时全诊断设置成长期 base default。

### 12.3 场景 fragment 只选应用 choice

当前 `config/log.conf` 已缩减为：

```conf
CONFIG_WBR_CONTROL_UART0_OUTPUT_LOG=y
```

其余派生值在应用 `Kconfig` 中统一表达。若某个 Zephyr 符号需要按场景为 `n`，优先检查是否应由 `depends on`、choice 或独立应用 policy symbol 表达，而不是在每个 fragment 重复正反赋值。

### 12.4 不要用 `select` 强推复杂用户符号而不审计依赖

`select` 适合强制启用无依赖的内部能力。对于 `LOG_BACKEND_UART` 这类本身带依赖、还会牵引 output formatter 的用户符号，更稳妥的做法通常是：

- 应用 mode 暴露清晰 policy；
- 通过 `depends on` 保证组合合法；
- 或由 fragment 明确设置所有顶层用户可见能力，但只保留一个配置真相；
- 每次用 `.config` 和 Kconfig warning 验证。

### 12.5 coredump 做明确决策

二选一：

1. **不需要 coredump**：`DEBUG_COREDUMP=n`，消除它对 LOG 的隐式牵引。
2. **需要 coredump**：明确选择并验证一个可用 backend；若用 logging backend，保证 fatal/panic 时至少一个 backend 能可靠输出，并评估 UART 与 VOFA 冲突。

### 12.6 对日志等级建立项目规则

建议：

- 每个 subsystem 用 `Kconfig.template.log_config` 暴露 `CONFIG_WBR_*_LOG_LEVEL`，不要在源码全部硬编码 `LOG_LEVEL_INF`；
- 默认产品构建保留 ERR/WRN，INF 按需；
- 诊断构建按模块提升 DBG；
- 高频错误使用计数器、`LOG_WRN_ONCE` 或有界限频；
- ISR 和 1 kHz 控制线程禁止 direct `printk`，日志参数也保持有界；
- 需要波形的数据走结构化 telemetry，不要用文本日志模拟 oscilloscope。

## 13. 推荐的排查工作流

### 13.1 每次先确认构建输入

```sh
grep -E '^(BOARD|CONF_FILE|EXTRA_CONF_FILE|DTC_OVERLAY_FILE):' \
  build/CMakeCache.txt
```

注意 CMake cache 中变量类型可能不是 `STRING`，必要时放宽 grep。

### 13.2 看最终 Kconfig，不看注释猜

```sh
grep -E 'CONFIG_(WBR_CONTROL_UART0|LOG|PRINTK|CONSOLE|UART_CONSOLE|BOOT_BANNER)' \
  build/zephyr/.config
```

重点同时检查：

```text
CONFIG_LOG
CONFIG_LOG_MODE_*
CONFIG_LOG_PRINTK
CONFIG_LOG_BACKEND_*
CONFIG_PRINTK
CONFIG_BOOT_BANNER
CONFIG_CONSOLE
CONFIG_UART_CONSOLE
```

### 13.3 用 menuconfig 追反向依赖

```sh
west build -d build -t menuconfig
```

按 `/` 搜索符号，查看：

- direct dependencies；
- selected by；
- 当前值与 user value；
- 定义位置。

当“写 n 却是 y”时，先查 `Selected by`；当“写 y 却是 n”时，先查 dependencies。

### 13.4 看最终 Devicetree

```sh
grep -n -A8 -B3 'chosen' build/zephyr/zephyr.dts
grep -n -A16 'uart0: serial@' build/zephyr/zephyr.dts
```

确认 `zephyr,console`、`zephyr,shell-uart`、可选的 `zephyr,log-uart` 和 UART status/baudrate。

### 13.5 确认 backend 真的编译进入镜像

```sh
grep CONFIG_LOG_BACKEND_UART build/zephyr/.config
ninja -C build -t commands | grep log_backend_uart.c
```

还可用 map/nm 检查 backend symbol，但 `.config` 与编译命令通常已足够定位配置问题。

### 13.6 场景使用独立目录

```sh
west build -p always -b dust-hpm6750 -d build-printk -- \
  -DEXTRA_CONF_FILE=config/printk.conf

west build -p always -b dust-hpm6750 -d build-log -- \
  -DEXTRA_CONF_FILE=config/log.conf

west build -p always -b dust-hpm6750 -d build-vofa -- \
  -DEXTRA_CONF_FILE=config/oscilloscope.conf
```

这样串口现象、ELF、`.config` 和 map 文件都能一一对应。

## 14. 选择 `printk` 还是 `LOG_*`

| 需求 | 推荐 |
|---|---|
| 驱动最早期 bring-up、logger 尚不可用 | 少量 direct `printk` |
| 常规模块状态、错误、告警 | `LOG_ERR/WRN/INF/DBG` |
| 高频控制波形 | VOFA/结构化 telemetry |
| fatal 前需要尽量 flush | logger + 合适 backend + panic path |
| 不占 UART0 的调试 | RTT（先验证平台支持） |
| 长期离线记录 | FS backend（评估 mount、wear、实时性） |
| 多设备/多核日志 | multidomain/link/backend 体系 |

工程上默认选 logger；把 direct `printk` 限定为极早期和最小 bring-up。logger 也不是“免费”的，实时线程仍应控制频率和参数大小。

## 15. 源码索引

本文结论可从当前工作区以下文件复核：

### 项目配置

- `wbr_control/Kconfig`
- `wbr_control/prj.conf`
- `wbr_control/config/*.conf`
- `wbr_control/app.overlay`
- `wbr_control/src/main.cpp`
- `wbr_control/src/modules/oscilloscope/oscilloscope_module.cpp`

### 板级与 HPM driver

- `hpm_support/boards/hpmicro/dust-hpm6750/dust-hpm6750_defconfig`
- `hpm_support/boards/hpmicro/dust-hpm6750/dust-hpm6750.dts`
- `hpm_support/drivers/serial/uart_hpmicro.c`

### Zephyr Kconfig 与实现

- `zephyr/doc/build/kconfig/setting.rst`
- `zephyr/doc/services/logging/index.rst`
- `zephyr/kernel/Kconfig`（`BOOT_BANNER`）
- `zephyr/lib/os/printk.c`
- `zephyr/include/zephyr/sys/printk.h`
- `zephyr/drivers/console/Kconfig`
- `zephyr/drivers/console/uart_console.c`
- `zephyr/subsys/logging/Kconfig*`
- `zephyr/subsys/logging/log_core.c`
- `zephyr/subsys/logging/log_msg.c`
- `zephyr/subsys/logging/log_output.c`
- `zephyr/subsys/logging/backends/Kconfig.*`
- `zephyr/subsys/logging/backends/log_backend_uart.c`
- `zephyr/subsys/debug/coredump/Kconfig`

## 16. 最终心智模型

以后遇到任何“为什么没有输出/为什么关不掉”的问题，按下面顺序问：

1. **调用是什么？** `printk`、`printf`、`LOG_*`、shell，还是直接 UART API？
2. **代码是否编译？** 看最终 `.config`，不要只看 fragment。
3. **谁强制了它？** 查 `depends on`、`select`、choice、默认值和 Kconfig warning。
4. **经过 logger 吗？** 对 `printk` 看 `LOG_PRINTK`；对 `LOG_*` 看 mode 和过滤。
5. **有 active backend 吗？** `LOG=y` 本身不是出口。
6. **backend 用哪个设备？** 看 `zephyr,log-uart`，否则 UART backend 回退到 `zephyr,console`。
7. **设备是否可用？** 看最终 `zephyr.dts`、status、pinmux、baudrate 和 `device_is_ready()`。
8. **UART API 是哪种？** polling、interrupt、async/DMA 的 owner 是否唯一？
9. **是否被缓存误导？** 用独立目录和 `-p always` 重建。
10. **是否符合实时预算？** 能看到日志不代表该输出路径适合生产控制循环。

只要严格按这十问从上到下走，Kconfig、console、`printk`、logger、backend 和 UART 之间的关系就不会再混成一团。
