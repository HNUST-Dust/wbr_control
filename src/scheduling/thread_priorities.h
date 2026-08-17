/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

namespace wbr_control::scheduling::thread_priority {

/*
 * Zephyr uses smaller numeric values for higher preemptive priorities.
 * Keep the event-to-control-to-output chain ordered, then place services
 * below it so overload degrades telemetry before control.
 */
constexpr int kImu = 4;
constexpr int kChassis = 5;
constexpr int kCanTx = 6;
constexpr int kPcLink = 7;
constexpr int kRemoteInput = 8;
constexpr int kReferee = 9;
constexpr int kOscilloscope = 10;
constexpr int kSystemState = 11;

static_assert(kImu < kChassis);
static_assert(kChassis < kCanTx);
static_assert(kCanTx < kPcLink);
static_assert(kPcLink < kRemoteInput);
static_assert(kRemoteInput < kReferee);
static_assert(kReferee < kOscilloscope);
static_assert(kOscilloscope < kSystemState);

}  // namespace wbr_control::scheduling::thread_priority
