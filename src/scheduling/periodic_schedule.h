/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file src/scheduling/periodic_schedule.h
 * @ingroup wbr_scheduling
 * @brief 提供基于 Zephyr 内核时钟的无漂移周期调度器。
 * @details 调度工具以 Zephyr 单调时钟为基准，使用绝对释放时刻避免执行时间抖动累积为长期周期漂移。
 */

#ifndef WBR_CONTROL_SCHEDULING_PERIODIC_SCHEDULE_H_
#define WBR_CONTROL_SCHEDULING_PERIODIC_SCHEDULE_H_

#include <cstdint>
#include <limits>

#include <zephyr/kernel.h>

namespace wbr_control::scheduling {

namespace thread_phase_ms {

constexpr uint32_t kChassis = 0U; ///< 底盘控制线程相对周期边界的释放相位，单位为毫秒。
constexpr uint32_t kOscilloscope = 2U; ///< 示波器线程释放相位，单位为毫秒。
constexpr uint32_t kPcLink = 4U; ///< 上位机链路线程释放相位，单位为毫秒。
constexpr uint32_t kSystemState = 5U; ///< 系统状态线程释放相位，单位为毫秒。

}  // namespace thread_phase_ms

/*
 * Absolute periodic release with phase preservation.
 *
 * If the caller or scheduler falls behind, old releases are counted and
 * skipped. The caller executes at most once per invocation, preventing an
 * overloaded thread from running back-to-back to replay obsolete work.
 */
/** @brief 以绝对截止时间避免累计漂移的周期调度器。 */
class AbsolutePeriodicSchedule {
public:
	/**
	 * @brief 创建保持指定周期和初始相位的绝对时间调度器。
	 * @param period_ms 调度周期，单位为毫秒；零值按一个内核节拍处理。
	 * @param initial_phase_ms 相对系统周期边界的首次释放相位，单位为毫秒。
	 */
	AbsolutePeriodicSchedule(uint32_t period_ms, uint32_t initial_phase_ms)
		: period_ticks_(TicksFromMs(period_ms)),
		  initial_phase_ticks_(TicksFromMsAllowZero(initial_phase_ms))
	{
		/**
		 * @brief 清空内部状态并恢复到初始条件。
		 */
		Reset();
	}

	/**
	 * @brief 清空内部状态并恢复到初始条件。
	 */
	void Reset()
	{
		const int64_t now = k_uptime_ticks();
		const int64_t phase_ticks =
			initial_phase_ticks_ % period_ticks_;
		next_release_tick_ =
			now - (now % period_ticks_) + phase_ticks;
		if (next_release_tick_ < now) {
			next_release_tick_ += period_ticks_;
		}
		total_missed_releases_ = 0U;
	}

	/**
	 * @brief 等待下一绝对释放时刻并更新丢周期统计。
	 * @return 本次等待期间错过的释放次数。
	 */
	uint32_t WaitForNextRelease()
	{
		uint64_t missed = 0U;
		int64_t now = k_uptime_ticks();

		if (now > next_release_tick_) {
			const uint64_t late_ticks =
				static_cast<uint64_t>(now - next_release_tick_);
			const uint64_t skipped =
				(late_ticks + static_cast<uint64_t>(period_ticks_) - 1U) /
				static_cast<uint64_t>(period_ticks_);
			next_release_tick_ +=
				static_cast<int64_t>(skipped) * period_ticks_;
			missed += skipped;
		}

		k_sleep(K_TIMEOUT_ABS_TICKS(next_release_tick_));

		/*
		 * A higher-priority task or ISR may delay the wake-up past one or
		 * more later releases. Treat the wake-up as the newest release and
		 * skip the obsolete ones before calculating the following deadline.
		 */
		now = k_uptime_ticks();
		if (now >= next_release_tick_ + period_ticks_) {
			const uint64_t skipped =
				static_cast<uint64_t>(
					(now - next_release_tick_) / period_ticks_);
			next_release_tick_ +=
				static_cast<int64_t>(skipped) * period_ticks_;
			missed += skipped;
		}

		next_release_tick_ += period_ticks_;
		AddMissedReleases(missed);
		return missed > std::numeric_limits<uint32_t>::max() ?
			std::numeric_limits<uint32_t>::max() :
			static_cast<uint32_t>(missed);
	}

	/**
	 * @brief 读取累计错过的周期释放次数。
	 * @return 自创建或复位以来累计错过的释放次数。
	 */
	uint32_t total_missed_releases() const
	{
		return total_missed_releases_;
	}

private:
	/**
	 * @brief 将非零毫秒周期转换为内核时钟节拍。
	 * @param milliseconds 待转换的时间长度，单位为毫秒。
	 * @return 至少为 1 的内核节拍数。
	 */
	static int64_t TicksFromMs(uint32_t milliseconds)
	{
		const int64_t ticks = static_cast<int64_t>(
			k_ms_to_ticks_ceil64(milliseconds));
		return ticks > 0 ? ticks : 1;
	}

	/**
	 * @brief 将毫秒时长转换为内核节拍并允许零值。
	 * @param milliseconds 待转换的时间长度，单位为毫秒。
	 * @return 对应的内核节拍数；输入为零时返回零。
	 */
	static int64_t TicksFromMsAllowZero(uint32_t milliseconds)
	{
		return static_cast<int64_t>(
			k_ms_to_ticks_ceil64(milliseconds));
	}

	/**
	 * @brief 以饱和方式累加错过的释放次数。
	 * @param missed 本次检测到的错过释放次数。
	 */
	void AddMissedReleases(uint64_t missed)
	{
		const uint32_t maximum =
			std::numeric_limits<uint32_t>::max();
		if (missed >= maximum - total_missed_releases_) {
			total_missed_releases_ = maximum;
		} else {
			total_missed_releases_ += static_cast<uint32_t>(missed);
		}
	}

	int64_t period_ticks_;
	int64_t initial_phase_ticks_;
	int64_t next_release_tick_;
	uint32_t total_missed_releases_ = 0U;
};

}  // namespace wbr_control::scheduling

#endif  // WBR_CONTROL_SCHEDULING_PERIODIC_SCHEDULE_H_
