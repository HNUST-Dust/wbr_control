/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file debug/shell/mavlink_mode_shell.cpp
 * @ingroup wbr_debug
 * @brief 实现 MAVLink 工作模式的 Shell 调试命令。
 * @details 本文件是 wbr_control 固件的一部分；接口量纲、所有权和线程约束以相邻声明的 Doxygen 文档为准。
 */

#include <stdint.h>
#include <string.h>

#include <atomic>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include <mavlink/v2.0/dust_custom/mavlink.h>

#include <channels/uart_raw_frame_queue.h>

namespace {

#if DT_HAS_CHOSEN(zephyr_shell_uart)
/** @brief MAVLink 调试模式使用的串口设备树节点。 */
#define WBR_CONTROL_MAVLINK_UART_NODE DT_CHOSEN(zephyr_shell_uart)
#else
/** @brief 未配置 Shell UART 时回退使用的控制台串口节点。 */
#define WBR_CONTROL_MAVLINK_UART_NODE DT_CHOSEN(zephyr_console)
#endif

constexpr uint8_t kMavlinkSysId = 1U;
constexpr uint8_t kMavlinkCompId = MAV_COMP_ID_AUTOPILOT1;
constexpr int kTxThreadPrio = 8;
constexpr int kRxThreadPrio = 9;
constexpr int kHeartbeatPeriodMs = 1000;
constexpr size_t kMavlinkMsgqTimeoutMs = 10;

K_THREAD_STACK_DEFINE(g_mavlink_tx_stack, 1024);
K_THREAD_STACK_DEFINE(g_mavlink_rx_stack, 1024);
struct k_thread g_mavlink_tx_thread;
struct k_thread g_mavlink_rx_thread;
std::atomic<bool> g_mavlink_enabled{false};
std::atomic<bool> g_mavlink_tx_thread_started{false};
std::atomic<bool> g_mavlink_rx_thread_started{false};
const struct device *g_shell_uart_dev = nullptr;

mavlink_status_t g_mavlink_parse_status = {};
uint32_t g_rx_msg_count = 0U;
uint32_t g_tx_msg_count = 0U;

void UartWrite(const uint8_t *data, size_t len)
{
	if ((g_shell_uart_dev == nullptr) || (data == nullptr) || (len == 0U)) {
		return;
	}

	for (size_t i = 0U; i < len; ++i) {
		uart_poll_out(g_shell_uart_dev, data[i]);
	}
}

void SendMavlinkMessage(const mavlink_message_t *msg)
{
	if ((msg == nullptr) || !g_mavlink_enabled.load()) {
		return;
	}

	uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN] = {};
	const uint16_t tx_len = mavlink_msg_to_send_buffer(tx_buf, msg);
	UartWrite(tx_buf, tx_len);
	++g_tx_msg_count;
}

void SendHeartbeat()
{
	mavlink_message_t msg = {};
	mavlink_msg_heartbeat_pack(
		kMavlinkSysId,
		kMavlinkCompId,
		&msg,
		MAV_TYPE_GROUND_ROVER,
		MAV_AUTOPILOT_GENERIC,
		0,
		0,
		MAV_STATE_ACTIVE);
	SendMavlinkMessage(&msg);
}

void SendSystemTime()
{
	const uint64_t now_ms = k_uptime_get();
	mavlink_message_t msg = {};
	mavlink_msg_system_time_pack(
		kMavlinkSysId,
		kMavlinkCompId,
		&msg,
		0U,
		now_ms);
	SendMavlinkMessage(&msg);
}

void MavlinkTxThreadMain()
{
	while (true) {
		if (g_mavlink_enabled.load()) {
			SendHeartbeat();
			SendSystemTime();
		}
		k_sleep(K_MSEC(kHeartbeatPeriodMs));
	}
}

void MavlinkRxThreadMain()
{
	while (true) {
		if (!g_mavlink_enabled.load()) {
			k_sleep(K_MSEC(kMavlinkMsgqTimeoutMs));
			continue;
		}

		channels::UartRawFrameMessage frame = {};
		if (channels::DequeueForMavlink(&frame) == 0) {
			mavlink_message_t msg = {};
			for (size_t i = 0U; i < frame.len; ++i) {
				if (mavlink_parse_char(MAVLINK_COMM_0, frame.data[i], &msg, &g_mavlink_parse_status) != 0U) {
					++g_rx_msg_count;
				}
			}
		} else {
			k_sleep(K_MSEC(kMavlinkMsgqTimeoutMs));
		}
	}
}

void StartMavlinkTxThreadOnce()
{
	if (g_mavlink_tx_thread_started.load()) {
		return;
	}

	k_thread_create(&g_mavlink_tx_thread,
			g_mavlink_tx_stack,
			K_THREAD_STACK_SIZEOF(g_mavlink_tx_stack),
			[](void *p1, void *p2, void *p3) {
				ARG_UNUSED(p1);
				ARG_UNUSED(p2);
				ARG_UNUSED(p3);
				MavlinkTxThreadMain();
			},
			nullptr,
			nullptr,
			nullptr,
			K_PRIO_PREEMPT(kTxThreadPrio),
			0,
			K_NO_WAIT);

	k_thread_name_set(&g_mavlink_tx_thread, "mavlink_tx");
	g_mavlink_tx_thread_started.store(true);
}

void StartMavlinkRxThreadOnce()
{
	if (g_mavlink_rx_thread_started.load()) {
		return;
	}

	k_thread_create(&g_mavlink_rx_thread,
			g_mavlink_rx_stack,
			K_THREAD_STACK_SIZEOF(g_mavlink_rx_stack),
			[](void *p1, void *p2, void *p3) {
				ARG_UNUSED(p1);
				ARG_UNUSED(p2);
				ARG_UNUSED(p3);
				MavlinkRxThreadMain();
			},
			nullptr,
			nullptr,
			nullptr,
			K_PRIO_PREEMPT(kRxThreadPrio),
			0,
			K_NO_WAIT);

	k_thread_name_set(&g_mavlink_rx_thread, "mavlink_rx");
	g_mavlink_rx_thread_started.store(true);
}

int CmdMavlinkStatus(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(shell,
		    "enabled=%d rx_msg=%u tx_msg=%u parse_drop=%u",
		    g_mavlink_enabled.load() ? 1 : 0,
		    (unsigned int)g_rx_msg_count,
		    (unsigned int)g_tx_msg_count,
		    (unsigned int)g_mavlink_parse_status.packet_rx_drop_count);
	return 0;
}

int CmdMavlinkOn(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (g_mavlink_enabled.load()) {
		shell_print(shell, "mavlink mode already enabled");
		return 0;
	}

	if ((g_shell_uart_dev == nullptr) || !device_is_ready(g_shell_uart_dev)) {
		shell_error(shell, "shell uart not ready");
		return -ENODEV;
	}

	g_mavlink_parse_status = {};
	g_rx_msg_count = 0U;
	g_tx_msg_count = 0U;
	g_mavlink_enabled.store(true);

	StartMavlinkTxThreadOnce();
	StartMavlinkRxThreadOnce();

	shell_print(shell, "mavlink mode enabled (background)");
	shell_print(shell, "TX/RX running independently of shell session");
	shell_print(shell, "Screen can be disconnected now");
	shell_print(shell, "Use 'mavlink status' to check state");
	return 0;
}

int CmdMavlinkOff(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!g_mavlink_enabled.load()) {
		shell_print(shell, "mavlink mode already disabled");
		return 0;
	}

	g_mavlink_enabled.store(false);
	shell_print(shell, "mavlink mode disabled");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_mavlink,
	SHELL_CMD(on, NULL, "Enable MAVLink (background mode)", CmdMavlinkOn),
	SHELL_CMD(off, NULL, "Disable MAVLink", CmdMavlinkOff),
	SHELL_CMD(status, NULL, "Show MAVLink status", CmdMavlinkStatus),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(mavlink, &sub_mavlink, "MAVLink mode control", NULL);

int MavlinkShellInit(void)
{
	g_shell_uart_dev = DEVICE_DT_GET(WBR_CONTROL_MAVLINK_UART_NODE);
	return 0;
}

SYS_INIT(MavlinkShellInit, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

}  // namespace
