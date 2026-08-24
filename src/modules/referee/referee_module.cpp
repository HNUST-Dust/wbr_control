/* SPDX-License-Identifier: Apache-2.0 */

/**
* @file src/modules/referee/referee_module.cpp
 * @ingroup wbr_modules
 * @brief 实现裁判系统数据接收与状态发布模块。
 * @details 实现运行在模块自有 Zephyr 线程或其驱动回调中。回调路径只完成有界的数据搬运和通知，耗时解析与控制计算留在线程上下文执行。
 */

#include <zephyr/logging/log.h>

#include <string.h>

#include "referee_module.h"

#include <channels/uart_raw_frame_queue.h>
#include <protocols/referee/referee_protocol.h>
#include <scheduling/thread_priorities.h>

LOG_MODULE_REGISTER(referee_module, LOG_LEVEL_INF);

namespace
{

K_THREAD_STACK_DEFINE(g_referee_module_stack, 1024);

uint16_t ReadLe16(const uint8_t *data)
{
	return static_cast<uint16_t>(data[0]) |
	       (static_cast<uint16_t>(data[1]) << 8U);
}

} // namespace

namespace modules
{

int RefereeModule::Start()
{
	if (started_) {
		return 0;
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
		FeedBytes(frame.data, frame.len);
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

		FeedBytes(frame.data, frame.len);
	}
}

void RefereeModule::FeedBytes(const uint8_t *data, size_t len)
{
	if ((data == nullptr) || (len == 0U)) {
		return;
	}

	for (size_t i = 0U; i < len; ++i) {
		if (stream_len_ < sizeof(stream_buf_)) {
			stream_buf_[stream_len_++] = data[i];
		} else {
			memmove(stream_buf_, stream_buf_ + 1, sizeof(stream_buf_) - 1U);
			stream_buf_[sizeof(stream_buf_) - 1U] = data[i];
		}
	}

	TryParseStream();
}

void RefereeModule::TryParseStream()
{
	while (stream_len_ >= protocols::kRefereeFrameHeaderSize) {
		if (stream_buf_[0] != protocols::kRefereeSof) {
			ConsumeStreamBytes(1U);
			continue;
		}

		const uint16_t data_len = ReadLe16(&stream_buf_[1]);
		const size_t full_len = protocols::kRefereeFrameHeaderSize + data_len +
					protocols::kRefereeFrameTrailerSize;
		if (full_len > sizeof(stream_buf_)) {
			ConsumeStreamBytes(1U);
			continue;
		}

		if (stream_len_ < full_len) {
			break;
		}

		HandleFrame(stream_buf_, full_len);
		ConsumeStreamBytes(full_len);
	}
}

void RefereeModule::ConsumeStreamBytes(size_t n)
{
	if (n >= stream_len_) {
		stream_len_ = 0U;
		return;
	}

	memmove(stream_buf_, stream_buf_ + n, stream_len_ - n);
	stream_len_ -= n;
}

void RefereeModule::HandleFrame(const uint8_t *frame, size_t frame_len)
{
	protocols::RefereeFrame decoded = {};
	if (protocols::DecodeRefereeFrame(frame, frame_len, &decoded) != 0) {
		return;
	}

	switch (decoded.cmd_id) {
	case protocols::kRefereeCmdGameStatus:
		(void)protocols::DecodeRefereeGameStatus(decoded.data, decoded.data_len,
							 &game_status_);
		break;
	case protocols::kRefereeCmdRobotStatus:
		(void)protocols::DecodeRefereeRobotStatus(decoded.data, decoded.data_len,
							  &robot_status_);
		break;
	case protocols::kRefereeCmdShootData:
		(void)protocols::DecodeRefereeShootData(decoded.data, decoded.data_len,
							&shoot_data_);
		break;
	default:
		break;
	}
}

} // namespace modules
