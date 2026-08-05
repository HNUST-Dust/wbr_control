# protocols

本目录同时保存协议声明和编解码实现。

所有协议共享单层 `protocols` 命名空间。公共符号使用协议前缀区分，
例如 `Hi91Parser`、`DecodeDmFeedbackNormal` 和 `DecodeDr16Frame`。

使用方统一通过以下形式包含：

```cpp
#include <protocols/motors/dji_motor_protocol.h>
```

protocols 不允许依赖业务 modules、channels 或具体 platform 适配。
