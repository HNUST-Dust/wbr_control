# Chassis controller

本目录是独立的底盘 Zephyr application，同时保存底盘领域模块的内部实现。
应用自身的 `CMakeLists.txt`、`Kconfig`、`prj.conf`、设备树 overlay 和
`main.cpp` 均位于此处。每个常驻线程模块只包含一个头文件和一个实现文件：

```text
<name>_module.h
<name>_module.cpp
```

`ModuleBase` 是普通抽象基类，只提供线程对象、重复启动保护、线程创建和入口转发：

```cpp
CreateThread(...);
ThreadEntry(...);
```

基类声明纯虚接口 `Start()` 和 `RunLoop()`。每个具体模块在构造函数中初始化纯
内存和算法状态，在重写的 `Start()` 中完成设备检查与驱动配置，再调用
`CreateThread()`。基类不管理生命周期状态、停止流程或系统启动顺序。

头文件位于模块自己的目录，因为这些类是应用内部的具体实现，不是可复用的公共
SDK 接口。`main.cpp` 显式持有模块实例，并直接表达初始化顺序和线程启动顺序。

## 参数配置

模块中需要现场调节的策略参数统一采用“参数 YAML + schema YAML”。每个模块
在自己的目录中保存 `<module>_params.yaml` 和
`<module>_params.schema.yaml`，参数值、校验规则和使用代码保持在同一个
模块边界内。

仓库统一使用 `tools/generate_params.py`，并通过本 application 的 CMake 中的
`wbr_generate_params()` 接入任意编译目标。生成头文件位于
`build/chassis_controller/generated/params/`，不应编辑或提交。

协议标识、CRC 常量、芯片寄存器值、数组维度和缓冲区容量不属于调参项，
仍应保留在对应源码或 Kconfig/devicetree 中。
