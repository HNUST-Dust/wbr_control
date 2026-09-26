/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file src/chassis_controller/chassis/chassis_module.h
 * @ingroup wbr_modules
 * @brief 实现轮腿底盘控制模块及周期控制流程。
 * @details 模块遵循 `ModuleBase` 生命周期：`Start()` 只负责一次性资源初始化和线程创建，`RunLoop()` 持有周期状态。跨线程数据通过 msg 层交换。
 */

#pragma once

#include <cstdint>

#include <zephyr/kernel.h>

#include "balance_controller.h"
#include "climb_stairs_controller.h"
#include "chassis_actuator.h"
#include "chassis_input_reader.h"
#include "chassis_state_machine.h"
#include "chassis_types.h"
#include "recovery_controller.h"
#include "support_force_estimator.h"
#include "flight_controller.h"
#include "jump_controller.h"
#include "../module_base.h"

namespace modules
{

/** @brief 执行轮腿底盘状态估计和闭环控制的应用模块。 */
class ChassisModule : public ModuleBase
{
public:
	/** @brief 构造底盘模块并保持执行器处于未使能状态。 */
	ChassisModule();
	/**
	 * @brief 初始化模块资源并创建工作线程。
	 * @return 成功返回 0；初始化或线程创建失败返回负 errno 错误码。
	 */
	int Start() override;
	/**
	 * @brief 执行模块线程的周期主循环。
	 */
	void RunLoop() override;

private:
	// 以下函数严格按照 RunLoop() 中的调用顺序排列。
	/**
	 * @brief 根据输入和安全条件更新底盘控制状态机。
	 * @param[in] input 本周期使用的只读输入快照。
	 */
	void UpdateControlState(const ChassisCycleInput &input);
	/**
	 * @brief 根据冻结输入计算本周期的全部执行器目标。
	 * @param[in] input 本周期使用的只读输入快照。
	 * @param[out] output 接收本周期计算结果的输出对象。
	 */
	void ComputeControlOutput(const ChassisCycleInput &input, ChassisControlOutput &output);
	/**
	 * @brief 对控制输出执行限幅并写入执行器发送通道。
	 * @param[out] output 接收本周期计算结果的输出对象。
	 */
	void ApplyControlOutput(ChassisControlOutput &output);
	void PublishTelemetry(const ChassisCycleInput &input,
			      const ChassisControlOutput &output,
			      const ChassisSupportForceEstimate &support_force);
	/**
	 * @brief 发布本控制周期的状态和诊断遥测。
	 * @param[in] input 本周期使用的只读输入快照。
	 * @param[out] output 接收本周期计算结果的输出对象。
	 */
	void PublishRealtimeStatus(const ChassisCycleInput &input, uint32_t loop_start_cycle);

	/**
	 * @brief 复位控制器、积分器和状态估计历史。
	 */
	void ResetControlState();
	uint32_t loop_ticks_ = 0U;
	uint64_t last_loop_time_us_ = 0U;
	uint32_t deadline_miss_count_ = 0U;
	uint32_t realtime_status_sequence_ = 0U;
	uint32_t telemetry_sequence_ = 0U;
	uint32_t max_loop_execution_us_ = 0U;
	uint32_t loop_period_us_ = 0U;
	uint32_t min_loop_period_us_ = UINT32_MAX;
	uint32_t max_loop_period_us_ = 0U;
	RecoveryController recovery_controller_;
	ClimbStairsController climb_stairs_controller_;
	BalanceController balance_controller_;
	ChassisInputReader input_reader_;
	SupportForceEstimator support_force_estimator_;
	FlightController flight_controller_;
	JumpController jump_controller_;
	ChassisStateMachine state_machine_;
	ChassisActuator actuator_;
};

} // namespace modules
