# Include 边界

工程采用 PX4 风格的内部组织：头文件跟随其所属模块或库，不再维护
`include/wbr_control/` 镜像目录。

## 目录与包含路径

| 所有者 | 头文件与实现位置 | 包含形式 |
|---|---|---|
| algorithms | `src/algorithms/` | `<algorithms/...>` |
| protocols | `src/protocols/` | `<protocols/...>` |
| modules | `src/modules/` | `<modules/...>` 或模块内引号包含 |
| scheduling | `src/scheduling/` | `<scheduling/...>` |
| channels | `channels/` | `<channels/...>` |
| platform | `platform/` 对应实现目录 | `<platform/...>` |

同目录实现优先使用引号，例如：

```cpp
#include "chassis_module.h"
```

跨所有者依赖保留目录前缀，例如：

```cpp
#include <channels/remote_input_state.hpp>
#include <protocols/motors/dji_motor_protocol.h>
#include <platform/drivers/communication/can_dispatch.h>
```

## 允许的依赖方向

```text
main
  └─ modules
       ├─ algorithms
       ├─ channels
       ├─ protocols
       ├─ scheduling
       └─ platform
            └─ channels

debug ──> channels
```

- algorithms、channels 和 protocols 不依赖业务模块。
- platform 可以使用 channel，但不能依赖 modules。
- 模块之间通过 channel 交换运行数据，不直接包含彼此的内部头。
- chassis 的运动学、VMC、LQR 调度和状态估计头只属于 chassis；聚焦白盒测试是
  唯一例外。
- `main.cpp` 只包含各模块入口头，不直接包含模块内部控制器。

## CMake include 根

PX4 风格需要两个内部 include 根：

- `${PROJECT_SOURCE_DIR}`：解析 `channels/...` 和 `platform/...`。
- `${PROJECT_SOURCE_DIR}/src`：解析 `algorithms/...`、`protocols/...`、
  `modules/...` 和 `scheduling/...`。

这些路径只表示仓库内部所有权，不表示对仓库外发布 SDK。每个 CMake 目标仍需
通过 `target_link_libraries()` 声明实际依赖，边界脚本负责阻止反向包含。

`src/scheduling` 是头文件接口目标 `wbr_scheduling`，由 modules 和 platform
显式链接。

运行检查：

```sh
bash tools/check_include_boundaries.sh
```
