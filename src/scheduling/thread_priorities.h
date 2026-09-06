/* SPDX-License-Identifier: Apache-2.0 */

/**
* @file src/scheduling/thread_priorities.h
 * @ingroup wbr_scheduling
 * @brief 集中定义应用线程优先级。
 * @details 调度工具以 Zephyr 单调时钟为基准，使用绝对释放时刻避免执行时间抖动累积为长期周期漂移。
 */

#pragma once

namespace wbr_control::scheduling::thread_priority {

/*
 * Zephyr uses smaller numeric values for higher preemptive priorities.
 * Safety-critical sensing/control stays above transport and telemetry work.
 * The USB IN submitter performs bounded snapshot/encode/DMA-start work and
 * sleeps until the previous transfer completes, but must not preempt the
 * 1 kHz chassis loop merely to keep a host-facing endpoint primed.
 */
constexpr int kImu = 4; ///< IMU 采集与姿态解算线程优先级。
constexpr int kAhrs = 4; ///< 板载 IMU 异步采集与姿态解算线程。
constexpr int kChassis = 5; ///< 底盘闭环控制线程优先级。
constexpr int kCanTx = 6; ///< CAN 发送调度线程优先级。
constexpr int kPcLink = 7; ///< 上位机链路接收与解析线程优先级。
constexpr int kPcLinkTx = 7; ///< 上位机链路异步发送提交线程优先级。
constexpr int kRemoteInput = 8; ///< 遥控器输入解析线程优先级。
constexpr int kReferee = 9; ///< 裁判系统协议解析线程优先级。
constexpr int kOscilloscope = 10; ///< 调试示波器数据输出线程优先级。
constexpr int kSystemState = 11; ///< 系统状态灯和蜂鸣器线程优先级。

static_assert(kImu < kChassis);
static_assert(kAhrs < kChassis);
static_assert(kChassis < kCanTx);
static_assert(kCanTx < kPcLinkTx);
static_assert(kCanTx < kPcLink);
static_assert(kPcLink < kRemoteInput);
static_assert(kRemoteInput < kReferee);
static_assert(kReferee < kOscilloscope);
static_assert(kOscilloscope < kSystemState);

}  // namespace wbr_control::scheduling::thread_priority
