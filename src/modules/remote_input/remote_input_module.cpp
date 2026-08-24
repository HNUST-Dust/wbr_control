/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file src/modules/remote_input/remote_input_module.cpp
 * @ingroup wbr_modules
 * @brief 实现多种遥控协议的解析与统一输入发布。
 * @details 实现运行在模块自有 Zephyr 线程或其驱动回调中。回调路径只完成有界的数据搬运和通知，耗时解析与控制计算留在线程上下文执行。
 */

#include <errno.h>

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

#include <hpm_l1c_drv.h>

#include "remote_input_module.h"

#include <protocols/remote_input/dr16_protocol.h>
#include <protocols/remote_input/vt03_protocol.h>
#include <protocols/remote_input/wfly_sbus_protocol.h>
#include <scheduling/thread_priorities.h>

LOG_MODULE_REGISTER(remote_input_module, LOG_LEVEL_INF);

namespace
{

K_THREAD_STACK_DEFINE(g_remote_input_module_stack, 1024);

#if !DT_HAS_CHOSEN(wbr_control_remote_input_uart)
#error "wbr-control,remote-input-uart must be selected for remote_input_module"
#endif

/** @brief 遥控器接收模块绑定的设备树 UART 节点。 */
#define WBR_REMOTE_INPUT_UART_NODE DT_CHOSEN(wbr_control_remote_input_uart)

constexpr size_t kUartRxBufferSize = 128U;
constexpr uint8_t kUartRxBufferCount = 2U;
constexpr int32_t kRxIdleTimeoutUs = 1000;
/** @brief 将 UART DMA 缓冲区对齐到一级缓存行边界。 */
#define UART_DMA_ALIGN __attribute__((aligned(HPM_L1C_CACHELINE_SIZE)))

uint8_t g_remote_input_rx_buffers[kUartRxBufferCount][kUartRxBufferSize] UART_DMA_ALIGN;

constexpr float kWflySbusMid = 1024.0f;
constexpr float kWflySbusScale = 670.0f;
constexpr uint16_t kWflySwitchLowMidThreshold = 689U;
constexpr uint16_t kWflySwitchMidHighThreshold = 1359U;
constexpr uint32_t kWflyFrameLostTimeoutMs = 1000U;

enum class WflySwitchPosition : uint8_t {
	kLow = 1,
	kMid = 2,
	kHigh = 3,
};

const struct device *FindRemoteInputUart()
{
	return DEVICE_DT_GET(WBR_REMOTE_INPUT_UART_NODE);
}

void InvalidateDmaRxCache(const uint8_t *data, size_t len)
{
	if ((data == nullptr) || (len == 0U)) {
		return;
	}

	const uint32_t start = HPM_L1C_CACHELINE_ALIGN_DOWN(reinterpret_cast<uint32_t>(data));
	const uint32_t end = HPM_L1C_CACHELINE_ALIGN_UP(reinterpret_cast<uint32_t>(data) +
							static_cast<uint32_t>(len));
	l1c_dc_invalidate(start, end - start);
}

float ClampNormalized(float value)
{
	if (value > 1.0f) {
		return 1.0f;
	}

	if (value < -1.0f) {
		return -1.0f;
	}

	return value;
}

float NormalizeWflyChannel(uint16_t raw)
{
	return ClampNormalized((static_cast<float>(raw) - kWflySbusMid) / kWflySbusScale);
}

float PositiveOnly(float value)
{
	return (value > 0.0f) ? value : 0.0f;
}

WflySwitchPosition DecodeWflySwitch(uint16_t raw)
{
	if (raw < kWflySwitchLowMidThreshold) {
		return WflySwitchPosition::kLow;
	}

	if (raw < kWflySwitchMidHighThreshold) {
		return WflySwitchPosition::kMid;
	}

	return WflySwitchPosition::kHigh;
}

void SetCommonActiveDefaults(channels::RemoteInputState *input)
{
	input->run = true;
	input->robot_enable = true;
	input->leg_length = 0.0f;
	input->leg_length_delta = 0.0f;
	input->friction_speed = 0.0f;
	input->plucker = 0.0f;
	input->fast_spin = false;
}

void SetDisabled(channels::RemoteInputState *input)
{
	input->chassis_x = 0.0f;
	input->chassis_rotate = 0.0f;
	input->yaw_angle = 0.0f;
	input->pitch_angle = 0.0f;
	input->leg_length = 0.0f;
	input->leg_length_delta = 0.0f;
	input->friction_speed = 0.0f;
	input->plucker = 0.0f;
	input->run = false;
	input->robot_enable = false;
	input->fast_spin = false;
}

} // namespace

namespace modules
{

int RemoteInputModule::Start()
{
	if (started_) {
		return 0;
	}

	uart_dev_ = FindRemoteInputUart();
	if ((uart_dev_ == nullptr) || !device_is_ready(uart_dev_)) {
		return -ENODEV;
	}

	return CreateThread(
		g_remote_input_module_stack, K_THREAD_STACK_SIZEOF(g_remote_input_module_stack),
		K_PRIO_PREEMPT(wbr_control::scheduling::thread_priority::kRemoteInput),
		"remote_input_module");
}

void RemoteInputModule::RunLoop()
{
	LOG_INF("remote_input module started");

	const int rc = StartUartRx();
	if (rc != 0) {
		LOG_ERR("remote_input uart rx start failed: %d", rc);
		return;
	}

	while (true) {
		DecodeUartBytesFromRing();
		CheckWflyFrameLostTimeout();
	}
}

int RemoteInputModule::StartUartRx()
{
	if (uart_rx_started_) {
		return 0;
	}

	int rc = uart_callback_set(uart_dev_, UartCallback, this);
	if (rc != 0) {
		return rc;
	}

	next_rx_buffer_index_ = 1U;
	rc = uart_rx_enable(uart_dev_, g_remote_input_rx_buffers[0],
			    sizeof(g_remote_input_rx_buffers[0]), kRxIdleTimeoutUs);
	if (rc != 0) {
		return rc;
	}

	uart_rx_started_ = true;
	LOG_INF("remote_input uart async rx started dev=%s", uart_dev_->name);
	return 0;
}

void RemoteInputModule::UartCallback(const struct device *dev, struct uart_event *evt,
				     void *user_data)
{
	auto *self = static_cast<RemoteInputModule *>(user_data);
	if (self != nullptr) {
		self->HandleUartEvent(dev, evt);
	}
}

void RemoteInputModule::HandleUartEvent(const struct device *dev, const struct uart_event *evt)
{
	if (evt == nullptr) {
		return;
	}

	switch (evt->type) {
	case UART_RX_RDY: {
		const uint8_t *data = evt->data.rx.buf + evt->data.rx.offset;
		InvalidateDmaRxCache(data, evt->data.rx.len);
		const uint32_t written =
			ring_buf_put(&channels::remote_input_ring_buf, data, evt->data.rx.len);
		if (written != evt->data.rx.len) {
			++rx_drop_count_;
		}
		k_sem_give(&channels::remote_input_sem);
		break;
	}
	case UART_RX_BUF_REQUEST: {
		const uint8_t index = next_rx_buffer_index_;
		next_rx_buffer_index_ =
			static_cast<uint8_t>((next_rx_buffer_index_ + 1U) % kUartRxBufferCount);
		(void)uart_rx_buf_rsp(dev, g_remote_input_rx_buffers[index],
				      sizeof(g_remote_input_rx_buffers[index]));
		break;
	}
	case UART_RX_DISABLED:
		next_rx_buffer_index_ = 1U;
		(void)uart_rx_enable(dev, g_remote_input_rx_buffers[0],
				     sizeof(g_remote_input_rx_buffers[0]), kRxIdleTimeoutUs);
		break;
	case UART_RX_STOPPED:
		++rx_stop_count_;
		break;
	default:
		break;
	}
}

int RemoteInputModule::ParseLine(const char *line, channels::RemoteInputState *out)
{
	if ((line == nullptr) || (out == nullptr)) {
		return -EINVAL;
	}

	char type[8] = {0};
	float chassis_x = 0.0f;
	float chassis_rotate = 0.0f;
	float yaw_angle = 0.0f;
	float pitch_angle = 0.0f;

	if (sscanf(line, "%7s %f %f %f %f", type, &chassis_x, &chassis_rotate, &yaw_angle,
		   &pitch_angle) != 5) {
		return -EINVAL;
	}

	if (strcmp(type, "dr16") == 0) {
		out->source = channels::kRemoteInputDr16;
		SetCommonActiveDefaults(out);
		out->chassis_x = chassis_x;
		out->chassis_rotate = chassis_rotate;
		out->yaw_angle = yaw_angle;
		out->pitch_angle = pitch_angle;
	} else if (strcmp(type, "vt03") == 0) {
		out->source = channels::kRemoteInputVt03;
		SetCommonActiveDefaults(out);
		out->chassis_x = chassis_x;
		out->chassis_rotate = chassis_rotate;
		out->yaw_angle = yaw_angle;
		out->pitch_angle = pitch_angle;
	} else if (strcmp(type, "wfly") == 0) {
		out->source = channels::kRemoteInputWfly;
		SetCommonActiveDefaults(out);
		out->chassis_x = chassis_x;
		out->chassis_rotate = chassis_rotate;
		out->yaw_angle = yaw_angle;
		out->pitch_angle = pitch_angle;
	} else {
		return -EINVAL;
	}

	return 0;
}

void RemoteInputModule::ConsumeBinary(size_t bytes)
{
	if ((bytes == 0U) || (binary_len_ == 0U)) {
		return;
	}

	if (bytes >= binary_len_) {
		binary_len_ = 0U;
		return;
	}

	memmove(binary_buf_, binary_buf_ + bytes, binary_len_ - bytes);
	binary_len_ -= bytes;
}

void RemoteInputModule::TryDecodeBinaryFrames()
{
	while (binary_len_ > 0U) {
		channels::RemoteInputState input = {};

		if (binary_buf_[0] == protocols::kWflySbusStartByte) {
			if (binary_len_ < protocols::kWflySbusFrameLength) {
				break;
			}

			protocols::WflySbusFrame wfly_frame = {};
			if (protocols::DecodeWflySbusFrame(binary_buf_, binary_len_, &wfly_frame)) {
				if (wfly_frame.frame_lost) {
					++frame_lost_count_;
					if (!wfly_frame_lost_active_) {
						wfly_frame_lost_start_ms_ = k_uptime_get_32();
						wfly_frame_lost_active_ = true;
						wfly_frame_lost_disabled_published_ = false;
					}
					CheckWflyFrameLostTimeout();
					ConsumeBinary(protocols::kWflySbusFrameLength);
					continue;
				}

				input.source = channels::kRemoteInputWfly;
				SetDisabled(&input);

				if (!wfly_frame.failsafe) {
					const float right_x =
						NormalizeWflyChannel(wfly_frame.channels[0]);
					const float right_y =
						NormalizeWflyChannel(wfly_frame.channels[1]);
					const float left_y =
						NormalizeWflyChannel(wfly_frame.channels[2]);
					const float left_x =
						NormalizeWflyChannel(wfly_frame.channels[3]);
					const WflySwitchPosition ch4 =
						DecodeWflySwitch(wfly_frame.channels[4]);
					const WflySwitchPosition ch5 =
						DecodeWflySwitch(wfly_frame.channels[5]);

					if (ch4 == WflySwitchPosition::kLow) {
						if (ch5 == WflySwitchPosition::kMid) {
							input.run = true;
							input.robot_enable = true;
							input.chassis_x = left_y;
							input.chassis_rotate = left_x;
							input.yaw_angle = right_x;
							input.pitch_angle = right_y;
						} else if (ch5 == WflySwitchPosition::kHigh) {
							input.run = true;
							input.robot_enable = true;
							input.friction_speed = PositiveOnly(left_y);
							input.plucker = right_y;
						}
					} else if (ch4 == WflySwitchPosition::kMid) {
						input.run = true;
						input.robot_enable = true;
						input.chassis_x = left_y;
						input.chassis_rotate = left_x;
						input.leg_length_delta = right_y;
						input.leg_length = input.leg_length_delta;
					}
				}
				PublishRemoteState(&input);
				ConsumeBinary(protocols::kWflySbusFrameLength);
				continue;
			}

			ConsumeBinary(1U);
			continue;
		}

		if ((binary_len_ >= protocols::kVt03RemoteFrameLength) &&
		    (binary_buf_[0] == 0xa9U) && (binary_buf_[1] == 0x53U)) {
			protocols::Vt03Frame vt03_frame = {};
			if (protocols::DecodeVt03RemoteFrame(binary_buf_, binary_len_,
							     &vt03_frame)) {
				input.source = channels::kRemoteInputVt03;
				SetCommonActiveDefaults(&input);
				input.chassis_x = vt03_frame.left_y;
				input.chassis_rotate = vt03_frame.left_x;
				input.yaw_angle = vt03_frame.right_x;
				input.pitch_angle = vt03_frame.right_y;
				PublishRemoteState(&input);
				ConsumeBinary(protocols::kVt03RemoteFrameLength);
				continue;
			}
			ConsumeBinary(1U);
			continue;
		}

		if ((binary_len_ >= protocols::kVt03CustomFrameLength) &&
		    (binary_buf_[0] == 0xa5U)) {
			protocols::Vt03CustomFrame custom = {};
			if (protocols::DecodeVt03CustomFrame(binary_buf_, binary_len_, &custom)) {
				input.source = channels::kRemoteInputVt03;
				SetCommonActiveDefaults(&input);
				input.yaw_angle = custom.joystick_x;
				input.pitch_angle = custom.joystick_y;
				PublishRemoteState(&input);
				ConsumeBinary(protocols::kVt03CustomFrameLength);
				continue;
			}
		}

		if (binary_len_ >= protocols::kDr16FrameLength) {
			protocols::Dr16Frame dr16_frame = {};
			if (protocols::DecodeDr16Frame(binary_buf_, binary_len_, &dr16_frame)) {
				input.source = channels::kRemoteInputDr16;
				SetCommonActiveDefaults(&input);
				input.chassis_x = dr16_frame.left_stick_y;
				input.chassis_rotate = dr16_frame.left_stick_x;
				input.yaw_angle = dr16_frame.right_stick_x;
				input.pitch_angle = dr16_frame.right_stick_y;
				PublishRemoteState(&input);
				ConsumeBinary(protocols::kDr16FrameLength);
				continue;
			}
		}

		if (binary_len_ < protocols::kDr16FrameLength) {
			break;
		}

		ConsumeBinary(1U);
	}
}

void RemoteInputModule::DecodeUartBytesFromRing()
{
	while (true) {
		uint8_t data[64] = {};
		uint32_t read_len = ring_buf_get(&channels::remote_input_ring_buf, data,
						 static_cast<uint32_t>(sizeof(data)));
		if (read_len == 0U) {
			if (k_sem_take(&channels::remote_input_sem, K_MSEC(20)) != 0) {
				break;
			}

			read_len = ring_buf_get(&channels::remote_input_ring_buf, data,
						static_cast<uint32_t>(sizeof(data)));
		}

		const size_t len = static_cast<size_t>(read_len);
		if (len == 0U) {
			break;
		}

		for (size_t i = 0U; i < len; ++i) {
			const uint8_t byte = data[i];

			if (binary_len_ < kBinaryBufSize) {
				binary_buf_[binary_len_++] = byte;
			} else {
				memmove(binary_buf_, binary_buf_ + 1, kBinaryBufSize - 1U);
				binary_buf_[kBinaryBufSize - 1U] = byte;
			}

			TryDecodeBinaryFrames();

			if (byte == '\r') {
				continue;
			}

			if (byte == '\n') {
				line_buf_[line_pos_] = '\0';
				if (line_pos_ > 0U) {
					channels::RemoteInputState input = {};
					if (ParseLine(line_buf_, &input) == 0) {
						PublishRemoteState(&input);
					}
				}
				line_pos_ = 0U;
				line_buf_[0] = '\0';
				continue;
			}

			if (isprint(byte) == 0) {
				continue;
			}

			if (line_pos_ < (kLineBufSize - 1U)) {
				line_buf_[line_pos_++] = static_cast<char>(byte);
			} else {
				line_pos_ = 0U;
				line_buf_[0] = '\0';
			}
		}
	}
}

void RemoteInputModule::CheckWflyFrameLostTimeout()
{
	if (!wfly_frame_lost_active_ || wfly_frame_lost_disabled_published_) {
		return;
	}

	const uint32_t elapsed_ms = k_uptime_get_32() - wfly_frame_lost_start_ms_;
	if (elapsed_ms < kWflyFrameLostTimeoutMs) {
		return;
	}

	channels::RemoteInputState input = {};
	input.source = channels::kRemoteInputWfly;
	SetDisabled(&input);
	wfly_frame_lost_disabled_published_ = true;
	PublishRemoteState(&input, false);
}

void RemoteInputModule::PublishRemoteState(channels::RemoteInputState *input,
					   bool clear_wfly_frame_lost)
{
	if (input == nullptr) {
		return;
	}

	if (clear_wfly_frame_lost) {
		wfly_frame_lost_active_ = false;
		wfly_frame_lost_disabled_published_ = false;
	}

	input->sequence = ++publish_sequence_;
	latest_remote_state.write(*input);
}

} // namespace modules
