# Chassis modules

这里保存 `chassis_controller` 的业务模块和模块私有算法。模块入口可被应用的
`main.cpp` 包含；控制器、估计器和协议适配细节应继续留在所属模块目录中。

共享的生命周期与周期调度机制分别来自 `<module_base.h>` 和
`<periodic_schedule.h>`。线程优先级和初始相位不是共享机制，统一由相邻的
`chassis_controller/scheduling/` 定义。

新增模块时，需要同时在 `modules.cmake` 中声明其 Kconfig 开关与源码目录；参数化
模块应把参数 YAML 和 schema YAML 放在自己的目录中，并接入
`wbr_generate_params()`。
