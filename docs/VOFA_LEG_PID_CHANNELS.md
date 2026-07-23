# VOFA 小板凳至 LQR 过渡实验通道

当前 `chassis` 已启用“小板凳姿态到 LQR”自动接管，共 14 路。坐标约定与物理 LQR 模型一致：`leg_angle = alpha`（相对机体的腿角，轮轴/脚点在胯关节后方为正）、`pitch = phi`，因此 `theta = alpha - phi`。小板凳阶段以 2 Hz 低通、0.20 rad/s 限速让 `alpha_ref` 跟随 `pitch`，从而先将 `theta` 对齐到零附近；对齐完成后直接进入完整 LQR。

小板凳固定腿长为 `150 mm`，利用机构机械限位承担静态支撑；准入腿长窗口为 `145–155 mm`。

| 通道 | 名称 | 单位/含义 |
| --- | --- | --- |
| 0 | `controller_state` | `200` 小板凳 theta 对齐；`250` LQR 渐入；`300` LQR 平衡 |
| 1 | `stool_ready_hold` | 小板凳两腿同时进入 theta 准入窗时为 `1`，否则为 `0` |
| 2 | `common_theta` | 双腿平均 `theta = alpha - phi`，deg；应收敛至 0 |
| 3–4 | `left_wheel_torque_request`、`right_wheel_torque_request` | 单侧物理轮力矩请求，Nm；小板凳阶段为零，进入 LQR 后为 LQR 请求；电流裁剪前 |
| 5–6 | `left_wheel_torque_sent`、`right_wheel_torque_sent` | 实际写入 CAN 后换算的物理 Nm |
| 7 | `pitch` | `phi`，deg |
| 8–9 | `left_wheel_saturated`、`right_wheel_saturated` | CAN 电流是否已撞到 ±16384：`0` 否、`1` 是 |
| 10–11 | `left_leg_torque_request`、`left_leg_torque_applied` | 每侧 LQR 机体对腿姿态力矩，Nm；前者为协调前，后者为叠加左右协调项后；不再有中间 ±10 Nm 限幅 |
| 12–13 | `right_leg_torque_request`、`right_leg_torque_applied` | 同左侧定义 |

左右 `theta` 同时进入 ±5° 后，流程直接进入完整 LQR/VMC；不再存在独立的 pitch 对齐或轮子 pitch-PD 阶段。通道 2 输出 `common_theta`，用于直接检查它是否收敛至零。300 ms 渐入实现仍保留，可通过 `kEnableLqrRamp` 恢复：轮子力矩从零上升至完整输出；腿部力矩在板凳 PID 与完整 VMC/LQR 支撑之间连续混合。75° 倾倒保护和原有遥控/IMU 门控仍然有效。

首次实验应让机器保持小板凳支撑并随时可失能：观察状态 `200 → 300`，确认通道 2 在切换前进入 ±5°，以及双轮力矩没有立刻饱和。
