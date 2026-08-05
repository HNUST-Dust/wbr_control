# channels

本目录保存 channel 的声明、消息类型、存储定义和队列操作：

- `ZBUS_CHAN_DEFINE`
- `K_MSGQ_DEFINE`、`K_SEM_DEFINE` 和 ring buffer 存储
- 全局 `SeqlockValue` 实例
- enqueue/dequeue 等非内联操作

使用方统一通过以下形式包含：

```cpp
#include <channels/remote_input_state.hpp>
```

`channels/` 不允许依赖 `modules/` 或具体 `platform/` 实现。
