# Algorithms

本目录同时保存算法头文件和实现。使用方通过所属库路径包含：

```cpp
#include <algorithms/control/alg_pid.h>
```

公共算法头不得依赖 Zephyr、业务 modules、channels、protocols 或 platform。
`controller_pid.cpp` 当前仍使用 Zephyr cycle counter 计算 `dt`，该依赖只存在
于实现文件中，后续需要改变时间输入接口时再单独解耦。

重复资产检查：

```sh
bash tools/algorithms_dedupe_audit.sh
```
