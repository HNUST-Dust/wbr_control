# Include 边界

工程采用 PX4 风格的内部组织：头文件跟随其所属模块或库，不再维护
`include/wbr_control/` 镜像目录。

## 目录与包含路径

| 所有者 | 头文件与实现位置 | 包含形式 |
|---|---|---|
| protocols | `src/protocols/` | `<protocols/...>` |
| shared module lifecycle | `src/module_base.h` | `<module_base.h>` |
| shared periodic scheduler | `src/periodic_schedule.h` | `<periodic_schedule.h>` |
| chassis modules | `src/chassis_controller/modules/` | `<chassis_controller/modules/...>` 或模块内引号包含 |
| msg | `msg/` | `<msg/...>` |
| chassis platform | `src/chassis_controller/platform/` | `<chassis_controller/platform/...>` |
| gimbal platform | `src/gimbal_controller/platform/` | `<gimbal_controller/platform/...>` |

同目录实现优先使用引号，例如：

```cpp
#include "chassis_module.h"
```

跨所有者依赖保留目录前缀，例如：

```cpp
#include <msg/remote_input_state.hpp>
#include <protocols/motors/dji_motor_protocol.h>
#include <chassis_controller/communication/can_dispatch.h>
```

## 允许的依赖方向

```text
main
  └─ modules
       ├─ msg
       ├─ protocols
       ├─ periodic_schedule.h
       └─ application platform
            └─ msg

debug ──> msg
```

- msg 和 protocols 不依赖业务模块。
- 每个应用的 platform 可以使用共享契约，但不能依赖业务模块，也不能依赖另一个应用的 platform。
- 模块之间通过 channel 交换运行数据，不直接包含彼此的内部头。
- 控制器和估计器由其业务模块拥有，不建立跨项目复用的通用 algorithms 层。
- chassis 的运动学、VMC、LQR 调度和状态估计头只属于 chassis；聚焦白盒测试是
  唯一例外。
- `main.cpp` 只包含各模块入口头，不直接包含模块内部控制器。

## CMake include 根

PX4 风格需要两个内部 include 根：

- `${WBR_CONTROL_ROOT}`：解析 `msg/...`。
- `${WBR_CONTROL_ROOT}/src`：解析共享头、`protocols/...`、
  `chassis_controller/...` 和 `gimbal_controller/...`。

这些路径只表示仓库内部所有权，不表示对仓库外发布 SDK。每个 CMake 目标仍需
通过 `target_link_libraries()` 声明实际依赖，边界脚本负责阻止反向包含。

`module_base.h` 与 `periodic_schedule.h` 都是纯头文件机制，不单独创建 CMake
目标；具体线程优先级与释放相位仍由各 application 自己维护。

运行检查：

```sh
bash tools/check_include_boundaries.sh
```
