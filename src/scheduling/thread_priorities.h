/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

namespace wbr_control::scheduling::thread_priority {

/*
 * Zephyr uses smaller numeric values for higher preemptive priorities.
 * Keep the USB IN submitter above the 1 kHz control chain so the interrupt
 * endpoint remains primed. The submitter performs bounded snapshot/encode/
 * DMA-start work and sleeps until the previous IN transfer completes.
 */
constexpr int kImu = 4;
constexpr int kChassis = 5;
constexpr int kCanTx = 6;
constexpr int kPcLink = 7;
constexpr int kPcLinkTx = 3;
constexpr int kRemoteInput = 8;
constexpr int kReferee = 9;
constexpr int kOscilloscope = 10;
constexpr int kSystemState = 11;

static_assert(kPcLinkTx < kImu);
static_assert(kImu < kChassis);
static_assert(kChassis < kCanTx);
static_assert(kCanTx < kPcLink);
static_assert(kPcLink < kRemoteInput);
static_assert(kRemoteInput < kReferee);
static_assert(kReferee < kOscilloscope);
static_assert(kOscilloscope < kSystemState);

}  // namespace wbr_control::scheduling::thread_priority
