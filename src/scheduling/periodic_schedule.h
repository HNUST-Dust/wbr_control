/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef WBR_CONTROL_SCHEDULING_PERIODIC_SCHEDULE_H_
#define WBR_CONTROL_SCHEDULING_PERIODIC_SCHEDULE_H_

#include <cstdint>
#include <limits>

#include <zephyr/kernel.h>

namespace wbr_control::scheduling {

namespace thread_phase_ms {

constexpr uint32_t kChassis = 0U;
constexpr uint32_t kOscilloscope = 2U;
constexpr uint32_t kPcLink = 4U;
constexpr uint32_t kSystemState = 5U;

}  // namespace thread_phase_ms

/*
 * Absolute periodic release with phase preservation.
 *
 * If the caller or scheduler falls behind, old releases are counted and
 * skipped. The caller executes at most once per invocation, preventing an
 * overloaded thread from running back-to-back to replay obsolete work.
 */
class AbsolutePeriodicSchedule {
public:
	AbsolutePeriodicSchedule(uint32_t period_ms, uint32_t initial_phase_ms)
		: period_ticks_(TicksFromMs(period_ms)),
		  initial_phase_ticks_(TicksFromMsAllowZero(initial_phase_ms))
	{
		Reset();
	}

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

	uint32_t total_missed_releases() const
	{
		return total_missed_releases_;
	}

private:
	static int64_t TicksFromMs(uint32_t milliseconds)
	{
		const int64_t ticks = static_cast<int64_t>(
			k_ms_to_ticks_ceil64(milliseconds));
		return ticks > 0 ? ticks : 1;
	}

	static int64_t TicksFromMsAllowZero(uint32_t milliseconds)
	{
		return static_cast<int64_t>(
			k_ms_to_ticks_ceil64(milliseconds));
	}

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
