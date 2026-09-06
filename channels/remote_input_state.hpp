/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file channels/remote_input_state.hpp
 * @ingroup wbr_channels
 * @brief 定义遥控输入的统一状态通道。
 * @details 写入方和读取方通过有界 spinlock 临界区复制完整快照，读取始终成功；调用方不得保存内部存储地址。
 */

#pragma once

#include <cstdint>

#include <channels/comm/seqlock_value.hpp>

namespace channels {

/** @brief 统一遥控输入的来源类型。 */
enum RemoteInputSource : uint8_t {
	kRemoteInputUnknown = 0, ///< 尚未识别到有效遥控输入。
	kRemoteInputDr16 = 1, ///< 输入来自 DR16 遥控器。
	kRemoteInputVt03 = 2, ///< 输入来自 VT03 遥控器。
	kRemoteInputWfly = 3, ///< 输入来自天地飞 SBUS 接收机。
};

/** @brief 各类遥控设备归一化后的控制输入。 */
struct RemoteInputState {
	uint8_t source; ///< 产生当前输入快照的遥控设备类型。
	float chassis_x; ///< 归一化前后运动指令，正值表示前进。
	float chassis_rotate; ///< 归一化底盘旋转指令，正负号遵循底盘坐标系。
	float yaw_angle; ///< 云台或自瞄目标航向角。
	float pitch_angle; ///< 云台或自瞄目标俯仰角。
	float leg_length; ///< 目标腿长，单位为米。
	float leg_length_delta; ///< 遥控输入要求的腿长增量。
	float friction_speed; ///< 摩擦轮目标速度。
	float plucker; ///< 拨弹机构目标控制量。
	bool run; ///< 操作员允许运动的命令标志。
	bool robot_enable; ///< 整机执行器使能标志。
	bool fast_spin; ///< 底盘小陀螺模式使能标志。
	bool supercap; ///< 超级电容辅助供能使能标志。
	bool auto_aim; ///< 上位机自瞄模式使能标志。
	bool friction_wheel; ///< 摩擦轮使能标志。
	bool auth_shoot; ///< 当前是否具备发射授权。
	bool pc_shoot_control; ///< 是否由上位机控制发射时机。

	uint32_t sequence; ///< 发布序号；每产生一个新快照递增一次。
};

}  // namespace channels

/** @brief 全局最新归一化遥控输入快照。 */
extern SeqlockValue<channels::RemoteInputState> latest_remote_state;
