/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/logging/log.h>

#include <channels/uart_raw_frame_queue.h>
#include <modules/referee/referee_module.h>
#include <modules/thread_utils.h>
#include <platform/drivers/devices/system/referee_client.h>

LOG_MODULE_REGISTER(referee_module, LOG_LEVEL_INF);

namespace {

K_THREAD_STACK_DEFINE(g_referee_module_stack, 1024);

} // namespace

namespace modules::referee {

int RefereeModule::Initialize() {
  started_ = false;
  sequence_ = 0U;
  return platform::drivers::devices::system::referee_client::Initialize();
}

int RefereeModule::Start() {
  if (started_) {
    return 0;
  }

  ::modules::StartMemberThread<RefereeModule, &RefereeModule::RunLoop>(
      &thread_, g_referee_module_stack,
      K_THREAD_STACK_SIZEOF(g_referee_module_stack), this, K_PRIO_PREEMPT(8),
      "referee_module");

  started_ = true;
  return 0;
}

void RefereeModule::RunLoop() {
  LOG_INF("referee module started");

  while (true) {
    DecodeUartFramesInQueue();

    k_sleep(K_MSEC(20));
  }
}

void RefereeModule::DecodeUartFramesInQueue() {
  while (true) {
    channels::uart_raw_frame_queue::UartRawFrameMessage frame = {};
    if (k_msgq_get(&channels::uart_raw_frame_queue::referee_uart_raw_msgq,
                   &frame, K_NO_WAIT) != 0) {
      break;
    }

    (void)platform::drivers::devices::system::referee_client::FeedBytes(
        frame.data, frame.len);
  }
}

} // namespace modules::referee
