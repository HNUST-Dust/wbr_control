# msg

本目录保存跨模块消息定义、消息存储和队列操作。

所有跨模块消息都直接在本目录的 `*.msg` 中定义，构建时由
`tools/generate_messages.py` 自动生成消息结构头文件和统一存储源文件。
传输类型通过指令选择：

- `@latest name`：由 `LatestValue<T>` 保存最新快照。
- `@queue name depth`：由 `MessageQueue<T, Depth>` 保存有序消息。
- `@byte_stream name capacity`：由 `ByteStream<Capacity>` 保存原始字节流。
- `@zbus name raw_name`：由 `ZbusMessage<T>` 封装 Zephyr ZBus。
- `@enum` 和 `@value`：在所属消息头中生成枚举。
- `@constant`：在所属消息头中生成数组容量等编译期常量。

例如：

```text
# @struct BoosterState
# @latest latest_booster_state
uint32 sequence
float32 bullet_speed
```

枚举和常量也直接归属消息：

```text
# @enum RemoteInputSource uint8
# @value kRemoteInputUnknown 0
# @value kRemoteInputDr16 1
# @constant size_t kPayloadSize 64
```

生成文件位于 `build/generated/msg/`，使用方通过稳定路径包含：

```cpp
#include <msg/booster_state.hpp>
```

这些传输机制均封装在 `transport/*.hpp` 或生成代码中，业务模块不再直接定义
ZBus channel、内核消息队列、ring buffer 或最新值存储。

`msg/` 不允许依赖 `modules/` 或具体 `platform/` 实现。
