/* SPDX-License-Identifier: Apache-2.0 */

#include <platform/drivers/devices/actuators/serial_servo.h>

#include <errno.h>

#include <channels/serialservo_send_raw.hpp>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

namespace platform {

namespace {

constexpr uint8_t kFrameHead = 0x55;
const struct device *g_uart_dev = nullptr;
bool g_initialized = false;
bool g_tx_thread_started = false;
k_thread g_tx_thread;
K_THREAD_STACK_DEFINE(g_serial_servo_tx_stack, 1024);

uint16_t AngleToRaw(float degrees)
{
	float clamped = degrees;
	if (clamped < 0.0f) {
		clamped = 0.0f;
	}
	if (clamped > 240.0f) {
		clamped = 240.0f;
	}

	return static_cast<uint16_t>(clamped * (1000.0f / 240.0f) + 0.5f);
}

int SendPacket(uint8_t id, uint8_t cmd, const uint8_t *params, uint8_t params_len)
{
	if (!g_initialized || (g_uart_dev == nullptr)) {
		return -ENODEV;
	}

	uint8_t frame[16] = {0};
	const uint8_t length = static_cast<uint8_t>(params_len + 3U);
	frame[0] = kFrameHead;
	frame[1] = kFrameHead;
	frame[2] = id;
	frame[3] = length;
	frame[4] = cmd;

	uint16_t sum = static_cast<uint16_t>(id + length + cmd);
	for (uint8_t i = 0U; i < params_len; ++i) {
		frame[5U + i] = params[i];
		sum = static_cast<uint16_t>(sum + params[i]);
	}

	frame[5U + params_len] = static_cast<uint8_t>(~(sum & 0xffU));
	const uint8_t total = static_cast<uint8_t>(6U + params_len);
	for (uint8_t i = 0U; i < total; ++i) {
		uart_poll_out(g_uart_dev, frame[i]);
	}

	return 0;
}

int SendMoveToAnglePacket(uint8_t id, float degrees, uint16_t time_ms)
{
	const uint16_t raw = AngleToRaw(degrees);
	const uint8_t params[4] = {
		static_cast<uint8_t>(raw & 0xffU),
		static_cast<uint8_t>((raw >> 8) & 0xffU),
		static_cast<uint8_t>(time_ms & 0xffU),
		static_cast<uint8_t>((time_ms >> 8) & 0xffU),
	};
	return SendPacket(id, 1U, params, sizeof(params));
}

int SendStopPacket(uint8_t id)
{
	return SendPacket(id, 12U, nullptr, 0U);
}

void TxLoop(void *, void *, void *)
{
	SeqlockValue<SerialServoSendRawFrame> *const slots[] = {
		&yaw_servo_send_raw,
		&pitch_servo_send_raw,
	};
	uint32_t last_sequence[sizeof(slots) / sizeof(slots[0])] = {};

	while (true) {
		for (size_t i = 0U; i < (sizeof(slots) / sizeof(slots[0])); ++i) {
			const uint32_t sequence = slots[i]->sequence();
			if ((sequence == 0U) || (sequence == last_sequence[i])) {
				continue;
			}

			SerialServoSendRawFrame value = {};
			if (!slots[i]->read(value)) {
				continue;
			}
			last_sequence[i] = sequence;

			if ((value.valid == 0U) || (value.len > sizeof(value.data))) {
				continue;
			}

			for (uint8_t j = 0U; j < value.len; ++j) {
				uart_poll_out(g_uart_dev, value.data[j]);
			}
		}

		k_sleep(K_MSEC(1));
	}
}

int PollOneByte(uint8_t *out, int64_t deadline_ms)
{
	if ((out == nullptr) || (g_uart_dev == nullptr)) {
		return -EINVAL;
	}

	while (k_uptime_get() < deadline_ms) {
		uint8_t b = 0U;
		if (uart_poll_in(g_uart_dev, &b) == 0) {
			*out = b;
			return 0;
		}
		k_msleep(1);
	}

	return -ETIMEDOUT;
}

int ReceivePacket(uint8_t *out_id,
		     uint8_t *out_cmd,
		     uint8_t *payload,
		     uint8_t *payload_len,
		     uint32_t timeout_ms)
{
	if ((out_id == nullptr) || (out_cmd == nullptr) || (payload == nullptr) || (payload_len == nullptr)) {
		return -EINVAL;
	}

	const int64_t deadline = k_uptime_get() + static_cast<int64_t>(timeout_ms);
	uint8_t b = 0U;

	while (true) {
		if (PollOneByte(&b, deadline) != 0) {
			return -ETIMEDOUT;
		}
		if (b != kFrameHead) {
			continue;
		}

		if (PollOneByte(&b, deadline) != 0) {
			return -ETIMEDOUT;
		}
		if (b != kFrameHead) {
			continue;
		}
		break;
	}

	uint8_t id = 0U;
	uint8_t length = 0U;
	uint8_t cmd = 0U;
	if ((PollOneByte(&id, deadline) != 0) || (PollOneByte(&length, deadline) != 0) ||
	    (PollOneByte(&cmd, deadline) != 0)) {
		return -ETIMEDOUT;
	}

	if (length < 3U) {
		return -EIO;
	}

	const uint8_t params_len = static_cast<uint8_t>(length - 3U);
	if (params_len > 16U) {
		return -EOVERFLOW;
	}

	uint16_t sum = static_cast<uint16_t>(id + length + cmd);
	for (uint8_t i = 0U; i < params_len; ++i) {
		if (PollOneByte(&payload[i], deadline) != 0) {
			return -ETIMEDOUT;
		}
		sum = static_cast<uint16_t>(sum + payload[i]);
	}

	uint8_t checksum = 0U;
	if (PollOneByte(&checksum, deadline) != 0) {
		return -ETIMEDOUT;
	}

	const uint8_t expected = static_cast<uint8_t>(~(sum & 0xFFU));
	if (checksum != expected) {
		return -EIO;
	}

	*out_id = id;
	*out_cmd = cmd;
	*payload_len = params_len;
	return 0;
}

}  // namespace

int InitializeSerialServo()
{
	if (g_initialized) {
		return 0;
	}

#if DT_HAS_CHOSEN(zephyr_serial_servo_uart)
	g_uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_serial_servo_uart));
#else
	return -ENODEV;
#endif
	if ((g_uart_dev == nullptr) || !device_is_ready(g_uart_dev)) {
		return -ENODEV;
	}

	g_initialized = true;
	if (!g_tx_thread_started) {
		k_tid_t tid = k_thread_create(&g_tx_thread,
					      g_serial_servo_tx_stack,
					      K_THREAD_STACK_SIZEOF(g_serial_servo_tx_stack),
					      TxLoop,
					      nullptr,
					      nullptr,
					      nullptr,
					      K_PRIO_PREEMPT(7),
					      0,
					      K_NO_WAIT);
		k_thread_name_set(tid, "uart_servo_tx");
		g_tx_thread_started = true;
	}
	return 0;
}

int MoveSerialServoToAngle(uint8_t id, float degrees, uint16_t time_ms)
{
	return SendMoveToAnglePacket(id, degrees, time_ms);
}

int StopSerialServo(uint8_t id)
{
	return SendStopPacket(id);
}

int SetSerialServoSpeed(uint8_t id, int16_t speed)
{
	const uint16_t speed_u = static_cast<uint16_t>(speed);
	const uint8_t params[4] = {
		1U,
		0U,
		static_cast<uint8_t>(speed_u & 0xFFU),
		static_cast<uint8_t>((speed_u >> 8) & 0xFFU),
	};
	return SendPacket(id, 29U, params, sizeof(params));
}

int ReadSerialServoId(uint8_t query_id, uint8_t *out_id, uint32_t timeout_ms)
{
	if (out_id == nullptr) {
		return -EINVAL;
	}

	const int send_rc = SendPacket(query_id, 14U, nullptr, 0U);
	if (send_rc != 0) {
		return send_rc;
	}

	uint8_t resp_id = 0U;
	uint8_t resp_cmd = 0U;
	uint8_t payload[16] = {0U};
	uint8_t payload_len = 0U;
	const int recv_rc = ReceivePacket(&resp_id, &resp_cmd, payload, &payload_len, timeout_ms);
	if (recv_rc != 0) {
		return recv_rc;
	}

	if ((resp_cmd != 14U) || (payload_len < 1U)) {
		return -EIO;
	}

	*out_id = payload[0];
	return 0;
}

}  // namespace platform
