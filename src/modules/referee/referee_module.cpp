/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/logging/log.h>

#include "referee_module.h"

#include <channels/uart_raw_frame_queue.h>
#include <platform/drivers/devices/system/referee_client.h>
#include <scheduling/thread_priorities.h>

LOG_MODULE_REGISTER(referee_module, LOG_LEVEL_INF);

namespace
{

K_THREAD_STACK_DEFINE(g_referee_module_stack, 1024);

} // namespace

namespace modules
{

int RefereeModule::Start()
{
	if (started_) {
		return 0;
	}

	const int initialize_result = platform::InitializeRefereeClient();
	if (initialize_result != 0) {
		return initialize_result;
	}

	return CreateThread(
		g_referee_module_stack, K_THREAD_STACK_SIZEOF(g_referee_module_stack),
		K_PRIO_PREEMPT(wbr_control::scheduling::thread_priority::kReferee),
		"referee_module");
}

void RefereeModule::RunLoop()
{
	LOG_INF("referee module started");

	while (true) {
		channels::UartRawFrameMessage frame = {};
		if (k_msgq_get(&channels::referee_uart_raw_msgq, &frame, K_FOREVER) != 0) {
			continue;
		}
		(void)platform::FeedRefereeBytes(frame.data, frame.len);
		DecodeUartFramesInQueue();
	}
}

void RefereeModule::DecodeUartFramesInQueue()
{
	while (true) {
		channels::UartRawFrameMessage frame = {};
		if (k_msgq_get(&channels::referee_uart_raw_msgq, &frame, K_NO_WAIT) != 0) {
			break;
		}

		(void)platform::FeedRefereeBytes(frame.data, frame.len);
	}
}

} // namespace modules
