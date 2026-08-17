# protocols

本目录同时保存协议声明和编解码实现。

所有协议共享单层 `protocols` 命名空间。公共符号使用协议前缀区分，
例如 `DecodeHi91Frame`、`DecodeDmFeedbackNormal` 和 `DecodeDr16Frame`。

`pc_link` 是 MCU 与 PC 之间的通信协议，提供自瞄数据包的编解码
（`EncodePcCommSend`/`DecodePcCommRecv` 等），严格保持上位机约定的
`'S','P'` 头 + 数据 + crc16 的 43/29 字节裸包格式，包格式见 `pc_link.h`。

使用方统一通过以下形式包含：

```cpp
#include <protocols/motors/dji_motor_protocol.h>
```

protocols 不允许依赖业务 modules、channels 或具体 platform 适配。
