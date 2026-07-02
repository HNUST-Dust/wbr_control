# leg_feedforward_test

用于实机测轮腿五连杆的腿长轴向力前馈。

这个测试不走正式 `chassis_module`，也不走腿长 PID。程序只做：

1. 初始化 CAN0/CAN1。
2. 给四个 DM 关节发送 MIT enter。
3. 读取 DM 反馈并计算五连杆运动学。
4. 给一个开环 `axial_force`。
5. 用当前 Jacobian 做 `J^T * F`，发送四个关节力矩。
6. 自动扫描 `0 -> -40N -> short 0 -> +40N -> short 0`，先收腿再伸腿。

## 构建

如果当前 shell 的 Python 缺 Zephyr 依赖，指定 workspace 上一级的 venv：

```bash
cmake -S wbr_control/test/leg_feedforward_test \
  -B wbr_control/test/leg_feedforward_test/build \
  -GNinja \
  -DBOARD=hpm6750evk2 \
  -DPython3_EXECUTABLE=/Users/panpoming/Documents/zephyr_projects/.venv/bin/python

CCACHE_DISABLE=1 ninja -C wbr_control/test/leg_feedforward_test/build
```

## 调参入口

在 `src/main.cpp` 顶部：

```cpp
kEnableLeftLeg
kEnableRightLeg
kForceMaxN
kForceStepN
kDwellMs
kZeroDwellMs
kJointTorqueLimitNm
kExitBetweenSweepCycles
```

第一次实机建议只测一条腿，把另一条腿设为 `false`。

如果机构在无电机力时会因为重力自动伸到限位，保持：

```cpp
kExitBetweenSweepCycles = false
```

不要在每轮扫描后退出 MIT 模式释放电机；同时 `kZeroDwellMs` 应该保持较短，避免零力阶段腿自己滑到伸腿限位。

## 看日志

日志格式：

```text
[leg_ff] phase=extend force_N=10 L len_mm=220 rate_mms=8 tq_mNm B=...
```

判据建议：

- 伸腿前馈：看 `phase=extend`，找到 `len_mm` 开始连续增加、`rate_mms` 持续非零的最小 `force_N`。
- 缩腿前馈：看 `phase=retract`，找到 `len_mm` 开始连续减小、`rate_mms` 持续非零的最小负 `force_N`。

如果零力时重力已经会让腿伸出，那么伸腿方向通常不需要正向静摩擦前馈，反而要关注：

- 收腿需要多大的负向 `force_N`。
- 不同腿长下，保持不继续伸出大约需要多大的负向力。
- 正向 `phase=extend` 主要用来看是否会过快伸出，而不是找“刚开始伸”的阈值。

可以按不同腿长区间记录：

```text
180mm: extend +?N, retract -?N
220mm: extend +?N, retract -?N
260mm: extend +?N, retract -?N
300mm: extend +?N, retract -?N
```

这些值后续可以先做常数前馈，再扩展成腿长一维表。
