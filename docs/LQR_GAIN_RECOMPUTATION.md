# LQR 增益重新计算与固件同步

本文是当前轮腿机器人 LQR 增益的可复现说明。后续 agent 如需因质量、惯量、质心、腿部 CAD 数据或权重变化而重新计算增益，应以本文和 `tools/derive_physical_lqr_schedule.py` 为准，不要直接使用 `controller.m` 第 3 节的示例零数组。

## 1. 当前计算链

```text
腿部 CAD 数据 CSV + 机器人固定物理参数
                  |
                  v
derive_physical_lqr_schedule.py
  按陈阳论文式(3)~(9)求解经典力学方程
  -> 从轮轴位置 x 转换到机体/髋部位置 xb
  -> 每个腿长建立连续时间 A(L0), B(L0)
  -> 检查可控性
  -> 解连续时间 LQR
  -> 得到 11 组 K(L0)
                  |
                  v
对归一化腿长做三次最小二乘拟合
                  |
                  v
physical_lqr_samples.csv + physical_lqr_cubic.csv
                  |
                  v
将 2 x 6 x 4 系数同步到 lqr_schedule.cc
                  |
                  v
运行时 EvaluateLqrGain(当前平均腿长)
```

相关文件：

- 计算程序：`tools/derive_physical_lqr_schedule.py`
- 腿部 CAD 输入：`tools/data/leg_mass_properties.csv`
- 每个腿长的 A、B、K 和闭环检查结果：`tools/generated/physical_lqr_samples.csv`
- 三次拟合系数：`tools/generated/physical_lqr_cubic.csv`
- 固件系数与求值函数：`src/modules/chassis/lqr_schedule.cc`
- 当前实机调用：`src/modules/chassis/chassis_module.cpp`
- 符号方程参考：`controller.m`

## 2. 状态、输入与控制律约定

状态顺序固定为：

\[
X=\begin{bmatrix}
\theta & \dot\theta & x_b & \dot x_b & \phi & \dot\phi
\end{bmatrix}^{T}
\]

其中：

| 状态 | 单位 | 定义 |
|---|---:|---|
| \(\theta\) | rad | 等效腿相对竖直向上的倾角 |
| \(\dot\theta\) | rad/s | 等效腿角速度 |
| \(x_b\) | m | 机体/髋部转轴水平位置 |
| \(\dot x_b\) | m/s | 机体/髋部转轴水平速度 |
| \(\phi\) | rad | 机体 pitch |
| \(\dot\phi\) | rad/s | 机体 pitch 角速度 |

固件中的腿角 `alpha` 是腿相对机体的角度，因此当前坐标关系为：

\[
\boxed{\theta=\alpha-\phi}
\]

必须先通过实机方向实验确认 `alpha` 和 `pitch` 的正方向仍符合该式，才能重新算 K。不能为了让输出“看起来方向正确”而只翻转某一列 K；若坐标定义变化，应先统一状态映射。

输入顺序固定为：

\[
U=\begin{bmatrix}T&T_p\end{bmatrix}^{T}
\]

- \(T\)：左右两个轮子的总广义力矩，单位 N·m。
- \(T_p\)：左右两条腿共同作用于机体姿态的总广义力矩，单位 N·m。

Python 程序使用标准连续 LQR 约定：

\[
U=-KX
\]

固件构造的是 `state_error = current - reference`，因此同样执行 `-K * state_error`。如果改成 `reference - current`，控制输出前的负号也必须同时改变。

当前 `chassis` 的实际误差向量为：

```cpp
{
    common_theta,
    common_theta_rate,
    body_position_error,
    body_speed,
    pitch,
    pitch_rate,
}
```

也就是说，除水平位置外其余状态当前都以 0 为参考值。

因为模型输入是左右总力矩，左右对称控制时每侧执行：

\[
T_{\mathrm{wheel,side}}=-\frac{1}{2}K_{0,:}e,
\qquad
T_{p,\mathrm{side}}=-\frac{1}{2}K_{1,:}e
\]

当前固件中的 `kPerSideGainScale = 0.5` 正是这个总力矩到单侧力矩的换算。不要在生成 K 时先除以 2、进入固件后又乘 0.5。

## 3. 当前物理参数

`RobotParameters` 当前使用：

| 参数 | 当前值 | 含义与来源 |
|---|---:|---|
| `wheel_mass_total_kg` | 1.2 kg | 两个车轮总质量，单轮 0.6 kg |
| `wheel_inertia_total_kg_m2` | 0.0005046 kg·m² | 两侧轮毂与电机转子折算到轮端后的总等效惯量 |
| `wheel_radius_m` | 0.058 m | 轮半径 |
| `leg_mass_total_kg` | 2.0 kg | 两条腿总质量，单腿 1.0 kg |
| `body_mass_kg` | 6.9 kg | 机体质量 |
| `body_pitch_inertia_kg_m2` | 0.066012040 kg·m² | CAD 坐标系 3 中绕 pitch 的 z 轴、关于机体质心的惯量；由 66012.040 kg·mm² 换算 |
| `body_com_offset_m` | -0.0483 m | 腿转轴到机体质心的有符号距离；质心在转轴下方所以取负 |
| `gravity_m_s2` | 9.80665 m/s² | 重力加速度 |

质量核对：\(1.2+2.0+6.9=10.1\text{ kg}\)，与整车称重一致。

腿部随腿长变化的数据来自 `leg_mass_properties.csv`。每行字段为：

| 字段 | 单位 | 含义 |
|---|---:|---|
| `leg_length_m` | m | 轮心到机体腿转轴的等效腿长 \(h\) |
| `leg_com_from_wheel_m` | m | 轮心到单腿质心的距离 \(L\) |
| `single_leg_inertia_kg_m2` | kg·m² | 单条腿关于自身质心、绕 pitch 轴的转动惯量 |

**这里的惯量参考点必须是单腿杆组自身质心。** 质量矩阵使用
`Ip + mp * L^2`，脚本先令 `Ip = 2 * single_leg_inertia`，再使用两腿总质量
`mp` 加入平行轴项。如果 CAD 给出的是绕轮心或上端转轴的惯量，直接填入会重复
应用平行轴定理；必须先换算回质心惯量。CSV 列名没有记录 CAD 的参考点，因此每次
替换机械数据时都要向数据提供者再次确认。

当前共有 11 个 CAD 点，腿长范围为 0.15105–0.30273 m。程序内部使用两腿总惯量：

\[
I_p=2I_{p,\mathrm{single}}
\]

## 4. 从论文经典力学方程建立 A、B

程序以陈阳论文式（3）~（9）作为生产模型。每次状态求值解一个 7×7
线性方程组，未知量为：

\[
\begin{bmatrix}
\ddot x&\ddot\theta&\ddot\phi&N&P&N_M&P_M
\end{bmatrix}^{T}
\]

其中 \(x\) 是论文经典力学分析中使用的轮轴水平位置。求出加速度后，按论文给出的
运动学关系转换到最终状态中的机体/髋部位置：

\[
x=x_b-h\sin\theta
\]

\[
\dot x_b=\dot x+h\dot\theta\cos\theta
\]

\[
\ddot x_b=\ddot x+h\ddot\theta\cos\theta
-h\dot\theta^2\sin\theta
\]

由此得到非线性状态导数
\([\dot\theta,\ddot\theta,\dot x_b,\ddot x_b,\dot\phi,\ddot\phi]^T\)，
再在直立平衡点用中心差分计算雅可比 \(A,B\)。

为防止经典方程转录、力矩符号或 \(x\to x_b\) 变换出错，程序还保留一套独立的
解析线性质量矩阵作为回归参考。令轮轴状态为 \(X_w\)、论文最终状态为 \(X_b\)，则

\[
X_b=P X_w,\qquad
A_b=P A_wP^{-1},\qquad B_b=P B_w
\]

每次正式生成前，11 个腿长点的经典方程雅可比必须与解析坐标变换结果在
\(10^{-6}\) 以内一致；当前最大绝对误差为 \(9.6949293\times10^{-10}\)。

平衡点为：

\[
\theta=\dot\theta=\dot x_b=\phi=\dot\phi=T=T_p=0
\]

位置 \(x_b\) 可取任意常数，因为模型具有平移不变性。

## 5. LQR 求解

文章基准权重为：

\[
Q=\operatorname{diag}(1,1,500,100,5000,1)
\]

\[
R=\operatorname{diag}(1,0.25)
\]

当前 `chassis` 为抑制实机的公共 \(\theta\) 摆动并增加腿部姿态分担，采用：

\[
\boxed{Q=\operatorname{diag}(1500,100,500,300,24000,800)}
\]

\[
\boxed{R=\operatorname{diag}(90,1)}
\]

这是当前脚本中的实机整定值。状态坐标从轮轴位置修正为论文的机体位置后，Q
中的第 3、4 项也明确对应 \(x_b,\dot x_b\)。后续改动必须通过脚本重算，不能只
手动修改某一列固件增益。

每个腿长点执行：

1. 建立连续时间 \(A(L_0),B(L_0)\)。
2. 计算可控矩阵并要求 `rank(ctrb(A,B)) == 6`。
3. 解连续代数 Riccati 方程，得到 \(K=R^{-1}B^TP\)。
4. 检查闭环矩阵 \(A-BK\) 的全部特征值实部为负。

脚本没有依赖 SciPy，而是通过 Hamiltonian 矩阵的稳定不变子空间求解 CARE。为防止求解器或符号约定被无意改坏，`verify_article_regression()` 会先对 `controller.m` 中文章给出的数值 A、B 做回归，要求得到的 K 与文章参考值一致。

## 6. 不同腿长的三次增益拟合

对每个输入、每个状态分别拟合，共 12 条曲线。为改善数值条件，不直接用米制腿长做多项式变量，而是先归一化：

\[
\boxed{s=\frac{L_0-0.25}{0.15}}
\]

每个增益元素采用：

\[
K_{ij}(L_0)=p_3s^3+p_2s^2+p_1s+p_0
\]

系数存储顺序固定为 `[p3, p2, p1, p0]`，与 NumPy `polyfit/polyval` 和固件 Horner 求值顺序一致。

固件会先把腿长限制在 CAD 数据范围：

\[
L_0\in[0.15105,0.30273]\text{ m}
\]

超出范围时不会外推，而是使用最近端点的增益。

当前结果的验证指标为：

- CAD 点数量：11；
- 经典方程与解析坐标变换最大绝对误差：\(9.6949293\times10^{-10}\)；
- 最大增益拟合绝对误差：0.026149243；
- CAD 点上拟合闭环极点最大实部：−1.8181402；
- 在整个范围内取 401 个点检查，闭环极点最大实部：−1.8181402。

## 7. 完整重算命令

从 `wbr_control` 目录运行。Python 环境必须安装 NumPy：

```bash
python3 tools/derive_physical_lqr_schedule.py \
  --input tools/data/leg_mass_properties.csv \
  --output tools/generated/physical_lqr_samples.csv \
  --coeff-output tools/generated/physical_lqr_cubic.csv
```

如需调整 LQR 权重，可以显式传入：

```bash
python3 tools/derive_physical_lqr_schedule.py \
  --input tools/data/leg_mass_properties.csv \
  --q 1500,100,500,300,24000,800 \
  --r 90,1 \
  --output tools/generated/physical_lqr_samples.csv \
  --coeff-output tools/generated/physical_lqr_cubic.csv
```

程序会在终端打印：

- 样本数和腿长范围；
- 论文七方程数值线性化与独立解析坐标变换模型的最大绝对误差；
- 最大拟合误差；
- 样本点和密集插值点的最差闭环极点实部；
- 可直接复制到 C++ 的 `kGainPolynomial[2][6][4]` 初始化器。

只有在命令成功退出、所有腿长点可控、所有验证指标通过后，才能把打印出的初始化器替换到 `src/modules/chassis/lqr_schedule.cc`。

## 8. 固件同步与核对

更新步骤：

1. 修改 `RobotParameters` 或 `leg_mass_properties.csv`。
2. 运行上一节命令重新生成两个 CSV。
3. 将终端打印的完整 `kGainPolynomial` 替换到 `lqr_schedule.cc`。
4. 确认 `kMinScheduledLegLength` 和 `kMaxScheduledLegLength` 等于 CSV 的最小、最大腿长。
5. 保持 `kLegLengthCenter = 0.25`、`kLegLengthHalfRange = 0.15` 与 Python 程序一致；若修改归一化参数，生成端和固件端必须同时修改。
6. 重新构建固件。
7. 检查链接结果确实包含 `EvaluateLqrGain` 和当前控制模块。

建议使用临时文件重算，并与已提交结果比较：

```bash
python3 tools/derive_physical_lqr_schedule.py \
  --output /tmp/physical_lqr_samples.check.csv \
  --coeff-output /tmp/physical_lqr_cubic.check.csv

diff -u tools/generated/physical_lqr_cubic.csv \
  /tmp/physical_lqr_cubic.check.csv
```

输出为空表示生成 CSV 完全一致。注意 CSV 比较只能证明生成结果一致；仍需核对 C++ 初始化器是否已经同步。

当前运行时以左右腿平均长度求 K：

```cpp
const double common_leg_length = 0.5 * (leg[0].length + leg[1].length);
EvaluateLqrGain(common_leg_length, gain);
```

然后按当前状态误差计算总力矩，并乘 0.5 分配到每侧。腿部 LQR 力矩随后还会经过每侧腿姿态力矩限制；轮端物理力矩还会经过 DJI 电流命令换算和协议范围限制。因此“模型 K 很大”不等于执行器实际一定输出相应力矩，调试时应同时观察未裁剪请求、裁剪后力矩和最终电流命令。

当前 `chassis` 的执行链还包括：

- 每侧公共腿姿态力矩先限制在 ±10 N·m；
- 左右腿协调力矩限制在 ±2 N·m，叠加后每侧再次限制在 ±10 N·m；
- VMC 接口与 LQR 的作用/反作用力矩定义以及右腿镜像会引入额外方向变换，不能只根据 K 的正负直接判断最终关节电机符号；
- 轮端使用 `kDjiCurrentPerNm = 3450` 将单轮 N·m 转为 DJI 电流命令，再限制到协议范围 ±16384；
- 左轮因安装方向在最终发送前取负，右轮保持正向。

上述比例和限幅不参与 A、B 或 K 的计算，但会改变实机可实现的闭环控制能力。早期讨论过的 `3326.26`、`1663.13` 等换算值不是当前 `chassis` 的实际常量；其中“总力矩除以 2”已经由 `kPerSideGainScale = 0.5` 完成，不能再在电流换算中重复除以 2。

## 9. 参数变化时该改哪里

| 变化项 | 修改位置 | 是否必须重算 K |
|---|---|---|
| 轮/腿/机体质量 | `RobotParameters` | 是 |
| 轮半径 | `RobotParameters` | 是 |
| 轮系等效惯量 | `RobotParameters` | 是 |
| 机体 pitch 惯量 | `RobotParameters` | 是 |
| 机体质心偏移及符号 | `RobotParameters` | 是 |
| 腿长、腿质心、腿惯量 CAD 点 | `leg_mass_properties.csv` | 是 |
| Q、R | 命令行参数或脚本默认值 | 是 |
| 电流命令/N·m 换算 | 固件执行器换算 | K 本身不变，但实际闭环增益会改变，必须重新验证 |
| 单侧力矩限制 | 固件限幅 | K 本身不变，但必须重新验证非线性饱和行为 |
| 状态正方向或状态顺序 | 观测器、控制器和模型一起修改 | 是，且必须重新做方向实验 |

## 10. 最容易犯的错误

1. 把单轮/单腿参数与左右总参数混用。当前模型统一使用左右总质量、总惯量和总力矩。
2. 把 `body_pitch_inertia_kg_m2` 填成 CAD 的错误轴，或忘记从 kg·mm² 乘 \(10^{-6}\) 换到 kg·m²。
3. 忘记机体质心在腿转轴下方，误把 `body_com_offset_m` 写成正数。
4. 把腿质心到轮心距离与腿质心到上端转轴距离混淆。CSV 输入的是前者。
5. 直接将米制腿长代入固件系数。当前系数必须使用归一化变量 \(s\)。
6. 混淆 `current-reference` 与 `reference-current`，造成控制律整体翻转。
7. 忘记 K 的第一行对应总轮力矩、第二行对应总腿姿态力矩。
8. 对总力矩重复除以 2，或完全没有分配到单侧。
9. 只检查 `A-BK` 数学稳定，不检查传感器符号、执行器符号、单位和饱和。
10. 运行 `controller.m` 第 3 节并把示例 `K_samples=zeros(...)` 当成正式计算结果。

## 11. 实机部署前最低检查清单

- [ ] 参数表与最新称重/CAD/辨识结果一致。
- [ ] `theta = alpha - pitch` 的正方向已在悬空实验中验证。
- [ ] 正轮力矩对应的实际车辆运动方向已验证。
- [ ] 正腿姿态力矩对应的实际机体/腿运动方向已验证。
- [ ] 11 个模型的可控矩阵秩均为 6。
- [ ] 所有样本点和密集验证点的 `max(real(eig(A-BK))) < 0`。
- [ ] 生成 CSV 与 C++ 的 48 个系数一致。
- [ ] C++ 和 Python 使用相同的腿长归一化与裁剪范围。
- [ ] 固件仍使用每侧 0.5 倍总力矩。
- [ ] 遥控使能/急停、倾倒保护、通信超时保护有效。
- [ ] 首次落地测试从受限力矩、小腿长和安全支撑开始，并记录状态误差、LQR 请求、裁剪后力矩及最终电流命令。
