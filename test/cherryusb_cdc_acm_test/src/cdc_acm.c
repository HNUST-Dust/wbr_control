/*
 * Copyright (c) 2022-2023 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file test/cherryusb_cdc_acm_test/src/cdc_acm.c
 * @brief 实现 CherryUSB CDC ACM 性能测试的数据收发逻辑。
 * @details 该文件属于独立 Zephyr 测试镜像，只验证指定外设或算法路径，不会链接进主固件。测试会直接访问目标硬件并通过串口输出判定结果。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>

#include "usbd_core.h"
#include "usbd_cdc_acm.h"

/*!< endpoint address */
#define CDC_IN_EP  0x81
#define CDC_OUT_EP 0x01
#define CDC_INT_EP 0x83

/*!< config descriptor size */
#define USB_CONFIG_SIZE (9 + CDC_ACM_DESCRIPTOR_LEN)

static const uint8_t device_descriptor[] = {
	USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01)
};

static const uint8_t config_descriptor_hs[] = {
	USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
	CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_HS, 0x02),
};

static const uint8_t config_descriptor_fs[] = {
	USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
	CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_FS, 0x02),
};

static const uint8_t device_quality_descriptor[] = {
	USB_DEVICE_QUALIFIER_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, 0x01),
};

static const uint8_t other_speed_config_descriptor_hs[] = {
	USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
	CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_FS, 0x02),
};

static const uint8_t other_speed_config_descriptor_fs[] = {
	USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
	CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_BULK_EP_MPS_HS, 0x02),
};

static const char *string_descriptors[] = {
	(const char[]){ 0x09, 0x04 }, /* Langid */
	"HPMicro",                    /* Manufacturer */
	"HPMicro CDC DEMO",           /* Product */
	"2024051702",                 /* Serial Number */
};

static const uint8_t *device_descriptor_callback(uint8_t speed)
{
	(void)speed;

	return device_descriptor;
}

static const uint8_t *config_descriptor_callback(uint8_t speed)
{
	if (speed == USB_SPEED_HIGH) {
		return config_descriptor_hs;
	} else if (speed == USB_SPEED_FULL) {
		return config_descriptor_fs;
	} else {
		return NULL;
	}
}

static const uint8_t *device_quality_descriptor_callback(uint8_t speed)
{
	(void)speed;

	return device_quality_descriptor;
}

static const uint8_t *other_speed_config_descriptor_callback(uint8_t speed)
{
	if (speed == USB_SPEED_HIGH) {
		return other_speed_config_descriptor_hs;
	} else if (speed == USB_SPEED_FULL) {
		return other_speed_config_descriptor_fs;
	} else {
		return NULL;
	}
}

static const char *string_descriptor_callback(uint8_t speed, uint8_t index)
{
	(void)speed;

	if (index >= (sizeof(string_descriptors) / sizeof(char *))) {
		return NULL;
	}
	return string_descriptors[index];
}

const struct usb_descriptor cdc_descriptor = {
	.device_descriptor_callback = device_descriptor_callback,
	.config_descriptor_callback = config_descriptor_callback,
	.device_quality_descriptor_callback = device_quality_descriptor_callback,
	.other_speed_descriptor_callback = other_speed_config_descriptor_callback,
	.string_descriptor_callback = string_descriptor_callback,
};

enum perf_mode {
	PERF_MODE_ECHO,
	PERF_MODE_RX_SINK,
	PERF_MODE_TX_STREAM,
};

#define PERF_TX_CHUNK_SIZE 16384U
#define PERF_READ_BUFFER_SIZE 16384U
#define PERF_STATUS_SIZE   96U

#ifndef PERF_EXPERIMENTAL_DOUBLE_OUT
#define PERF_EXPERIMENTAL_DOUBLE_OUT 0
#endif

#if PERF_EXPERIMENTAL_DOUBLE_OUT
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t read_buffer[2][PERF_READ_BUFFER_SIZE];
#else
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t read_buffer[PERF_READ_BUFFER_SIZE];
#endif
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t tx_buffer[PERF_TX_CHUNK_SIZE];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t status_buffer[PERF_STATUS_SIZE];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t pending_status_buffer[PERF_STATUS_SIZE];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t echo_buffer[PERF_READ_BUFFER_SIZE];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t pending_echo_buffer[PERF_READ_BUFFER_SIZE];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t command_buffer[64];

static volatile bool in_ep_busy;
static volatile bool pending_status_valid;
static uint32_t pending_status_len;
static volatile bool pending_echo_valid;
static uint32_t pending_echo_len;
static uint32_t command_len;
static enum perf_mode current_mode = PERF_MODE_ECHO;
static uint64_t rx_expected_bytes;
static uint64_t rx_remaining_bytes;
static uint64_t rx_received_bytes;
static int64_t rx_start_ms;
static uint64_t tx_remaining_bytes;
static uint64_t tx_sent_bytes;
#if PERF_EXPERIMENTAL_DOUBLE_OUT
static uint8_t rx_out_submit_index;
static bool rx_double_out_primed;
static uint32_t rx_out_submit_count;
static uint32_t rx_out_complete_count;
static uint32_t rx_out_submit_error_count;
#endif

static void start_next_tx_chunk(uint8_t busid);
static void send_status_line(uint8_t busid, const char *line);
static bool start_in_write(uint8_t busid, const uint8_t *data, uint32_t len);
static bool start_pending_status(uint8_t busid);
static bool start_pending_echo(uint8_t busid);
static void start_out_read(uint8_t busid, uint8_t ep);
static void maintain_out_reads(uint8_t busid, uint8_t ep);
static uint8_t *out_read_buffer(void);
static uint8_t *out_complete_buffer(void);
static bool process_command_bytes(uint8_t busid, const uint8_t *data, uint32_t nbytes);

static uint64_t parse_u64(const char *text)
{
	uint64_t value = 0U;

	while (*text == ' ') {
		text++;
	}

	while (*text >= '0' && *text <= '9') {
		value = (value * 10U) + (uint64_t)(*text - '0');
		text++;
	}

	return value;
}

static bool start_in_write(uint8_t busid, const uint8_t *data, uint32_t len)
{
	if (usbd_ep_start_write(busid, CDC_IN_EP, data, len) != 0) {
		in_ep_busy = false;
		return false;
	}

	in_ep_busy = true;
	return true;
}

static bool start_pending_status(uint8_t busid)
{
	if (!pending_status_valid || in_ep_busy) {
		return false;
	}

	memcpy(status_buffer, pending_status_buffer, pending_status_len);
	pending_status_valid = false;
	return start_in_write(busid, status_buffer, pending_status_len);
}

static bool start_pending_echo(uint8_t busid)
{
	if (!pending_echo_valid || in_ep_busy || current_mode != PERF_MODE_ECHO) {
		return false;
	}

	pending_echo_valid = false;
	return start_in_write(busid, pending_echo_buffer, pending_echo_len);
}

static uint8_t *out_read_buffer(void)
{
#if PERF_EXPERIMENTAL_DOUBLE_OUT
	if (current_mode == PERF_MODE_RX_SINK) {
		uint8_t *buffer = read_buffer[rx_out_submit_index];

		rx_out_submit_index ^= 1U;
		return buffer;
	}

	return read_buffer[0];
#else
	return read_buffer;
#endif
}

static uint8_t *out_complete_buffer(void)
{
#if PERF_EXPERIMENTAL_DOUBLE_OUT
	/*
	 * Outside RX sink the experimental path keeps one OUT read armed on
	 * buffer 0. RX sink discards data, so it does not need to identify the
	 * completed buffer.
	 */
	return read_buffer[0];
#else
	return read_buffer;
#endif
}

static void start_out_read(uint8_t busid, uint8_t ep)
{
	uint32_t read_len = PERF_READ_BUFFER_SIZE;
	int ret;

	if (current_mode != PERF_MODE_RX_SINK) {
		read_len = usbd_get_ep_mps(busid, ep);
		if (read_len == 0U || read_len > PERF_READ_BUFFER_SIZE) {
			read_len = PERF_READ_BUFFER_SIZE;
		}
	}

	/* Default mode re-arms after completion so HPM port state stays tied to
	 * one buffer. PERF_EXPERIMENTAL_DOUBLE_OUT deliberately violates this
	 * only inside RX sink mode to reproduce/measure the risky path.
	 */
	ret = usbd_ep_start_read(busid, ep, out_read_buffer(), read_len);
#if PERF_EXPERIMENTAL_DOUBLE_OUT
	if (current_mode == PERF_MODE_RX_SINK) {
		if (ret == 0) {
			rx_out_submit_count++;
		} else {
			rx_out_submit_error_count++;
		}
	}
#endif
}

static void maintain_out_reads(uint8_t busid, uint8_t ep)
{
#if PERF_EXPERIMENTAL_DOUBLE_OUT
	if (current_mode == PERF_MODE_RX_SINK && !rx_double_out_primed) {
		start_out_read(busid, ep);
		start_out_read(busid, ep);
		rx_double_out_primed = true;
		return;
	}
#endif

	start_out_read(busid, ep);
}

static void send_status_line(uint8_t busid, const char *line)
{
	size_t len = strlen(line);

	if (len >= sizeof(status_buffer)) {
		len = sizeof(status_buffer) - 1U;
	}

	if (in_ep_busy) {
		memcpy(pending_status_buffer, line, len);
		pending_status_len = (uint32_t)len;
		pending_status_valid = true;
		return;
	}

	memcpy(status_buffer, line, len);
	start_in_write(busid, status_buffer, (uint32_t)len);
}

static void start_rx_test(uint8_t busid, uint64_t bytes)
{
	rx_expected_bytes = bytes;
	rx_remaining_bytes = bytes;
	rx_received_bytes = 0U;
	rx_start_ms = 0;
	current_mode = PERF_MODE_RX_SINK;
#if PERF_EXPERIMENTAL_DOUBLE_OUT
	rx_out_submit_index = 0U;
	rx_double_out_primed = false;
	rx_out_submit_count = 0U;
	rx_out_complete_count = 0U;
	rx_out_submit_error_count = 0U;
#endif

#if PERF_EXPERIMENTAL_DOUBLE_OUT
	snprintf((char *)status_buffer, sizeof(status_buffer), "RX_READY %llu DOUBLE_OUT\r\n",
		 (unsigned long long)bytes);
#else
	snprintf((char *)status_buffer, sizeof(status_buffer), "RX_READY %llu\r\n",
		 (unsigned long long)bytes);
#endif
	send_status_line(busid, (const char *)status_buffer);
}

static void start_tx_test(uint8_t busid, uint64_t bytes)
{
	tx_remaining_bytes = bytes;
	tx_sent_bytes = 0U;
	current_mode = PERF_MODE_TX_STREAM;

	for (uint32_t i = 0; i < sizeof(tx_buffer); i++) {
		tx_buffer[i] = (uint8_t)i;
	}

	send_status_line(busid, "TX_READY\r\n");
	start_next_tx_chunk(busid);
}

static void start_next_tx_chunk(uint8_t busid)
{
	uint32_t chunk_len;

	if (in_ep_busy || tx_remaining_bytes == 0U) {
		return;
	}

	chunk_len = (tx_remaining_bytes > sizeof(tx_buffer)) ? sizeof(tx_buffer) :
							      (uint32_t)tx_remaining_bytes;

	tx_remaining_bytes -= chunk_len;
	tx_sent_bytes += chunk_len;
	start_in_write(busid, tx_buffer, chunk_len);
}

static void finish_rx_test(uint8_t busid)
{
	int64_t elapsed_ms = k_uptime_get() - rx_start_ms;

	if (elapsed_ms <= 0) {
		elapsed_ms = 1;
	}

#if PERF_EXPERIMENTAL_DOUBLE_OUT
	snprintf((char *)status_buffer, sizeof(status_buffer),
		 "RX_DONE %llu %lld submit=%u complete=%u err=%u\r\n",
		 (unsigned long long)rx_received_bytes, (long long)elapsed_ms,
		 rx_out_submit_count, rx_out_complete_count, rx_out_submit_error_count);
#else
	snprintf((char *)status_buffer, sizeof(status_buffer), "RX_DONE %llu %lld\r\n",
		 (unsigned long long)rx_received_bytes, (long long)elapsed_ms);
#endif
	current_mode = PERF_MODE_ECHO;
#if PERF_EXPERIMENTAL_DOUBLE_OUT
	rx_double_out_primed = false;
#endif
	send_status_line(busid, (const char *)status_buffer);
}

static void process_command_line(uint8_t busid, const uint8_t *data, uint32_t nbytes)
{
	char command[64];
	uint32_t copy_len = (nbytes < (sizeof(command) - 1U)) ? nbytes :
							     (sizeof(command) - 1U);

	memcpy(command, data, copy_len);
	command[copy_len] = '\0';

	if (strncmp(command, "$PING", 5) == 0) {
		send_status_line(busid, "PONG\r\n");
	} else if (strncmp(command, "$ECHO", 5) == 0) {
		current_mode = PERF_MODE_ECHO;
		send_status_line(busid, "ECHO_READY\r\n");
	} else if (strncmp(command, "$RX ", 4) == 0) {
		start_rx_test(busid, parse_u64(&command[4]));
	} else if (strncmp(command, "$TX ", 4) == 0) {
		start_tx_test(busid, parse_u64(&command[4]));
	} else {
		send_status_line(busid, "ERR unknown command\r\n");
	}
}

static bool process_command_bytes(uint8_t busid, const uint8_t *data, uint32_t nbytes)
{
	if (nbytes == 0U) {
		return false;
	}

	if (command_len == 0U && data[0] != '$') {
		return false;
	}

	for (uint32_t i = 0U; i < nbytes; i++) {
		uint8_t ch = data[i];
		if (ch == '\n' || ch == '\r') {
			if (command_len > 0U) {
				process_command_line(busid, command_buffer, command_len);
				command_len = 0U;
			}
			continue;
		}

		if (command_len == 0U && ch != '$') {
			continue;
		}

		if (command_len >= sizeof(command_buffer)) {
			command_len = 0U;
			send_status_line(busid, "ERR command too long\r\n");
			continue;
		}

		command_buffer[command_len++] = ch;
	}

	return true;
}

static void usbd_event_handler(uint8_t busid, uint8_t event)
{
	switch (event) {
	case USBD_EVENT_RESET:
		break;
	case USBD_EVENT_CONNECTED:
		break;
	case USBD_EVENT_DISCONNECTED:
		break;
	case USBD_EVENT_RESUME:
		break;
	case USBD_EVENT_SUSPEND:
		break;
	case USBD_EVENT_CONFIGURED:
		in_ep_busy = false;
			pending_status_valid = false;
			pending_echo_valid = false;
			command_len = 0U;
			current_mode = PERF_MODE_ECHO;
#if PERF_EXPERIMENTAL_DOUBLE_OUT
			rx_out_submit_index = 0U;
			rx_double_out_primed = false;
			rx_out_submit_count = 0U;
			rx_out_complete_count = 0U;
			rx_out_submit_error_count = 0U;
#endif
			printf("USB configured, speed=%d\n", usbd_get_port_speed(busid));
			start_out_read(busid, CDC_OUT_EP);
			break;
	case USBD_EVENT_SET_REMOTE_WAKEUP:
		break;
	case USBD_EVENT_CLR_REMOTE_WAKEUP:
		break;
	default:
		break;
	}
}

void usbd_cdc_acm_bulk_out(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
	uint8_t *buffer = out_complete_buffer();

	if (current_mode != PERF_MODE_RX_SINK && nbytes > 0U &&
	    process_command_bytes(busid, buffer, nbytes)) {
	} else if (current_mode == PERF_MODE_RX_SINK) {
#if PERF_EXPERIMENTAL_DOUBLE_OUT
		rx_out_complete_count++;
#endif
		if (rx_received_bytes == 0U) {
			rx_start_ms = k_uptime_get();
		}

		if (nbytes >= rx_remaining_bytes) {
			rx_received_bytes += rx_remaining_bytes;
			rx_remaining_bytes = 0U;
			finish_rx_test(busid);
		} else {
			rx_received_bytes += nbytes;
			rx_remaining_bytes -= nbytes;
		}
	} else if (!in_ep_busy) {
		memcpy(echo_buffer, buffer, nbytes);
		start_in_write(busid, echo_buffer, nbytes);
	} else if (nbytes <= sizeof(pending_echo_buffer)) {
		memcpy(pending_echo_buffer, buffer, nbytes);
		pending_echo_len = nbytes;
		pending_echo_valid = true;
	}

	maintain_out_reads(busid, ep);
}

void usbd_cdc_acm_bulk_in(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
	in_ep_busy = false;

	if (start_pending_status(busid)) {
		return;
	}

	if (start_pending_echo(busid)) {
		return;
	}

	if (current_mode == PERF_MODE_TX_STREAM) {
		if (tx_remaining_bytes > 0U) {
			start_next_tx_chunk(busid);
		} else {
			if ((nbytes % usbd_get_ep_mps(busid, ep)) == 0 && nbytes) {
				start_in_write(busid, NULL, 0);
			}
			current_mode = PERF_MODE_ECHO;
		}
		return;
	}

	if ((nbytes % usbd_get_ep_mps(busid, ep)) == 0 && nbytes) {
		start_in_write(busid, NULL, 0);
	}
}

/*!< endpoint call back */
struct usbd_endpoint cdc_out_ep = {
	.ep_addr = CDC_OUT_EP,
	.ep_cb = usbd_cdc_acm_bulk_out
};

struct usbd_endpoint cdc_in_ep = {
	.ep_addr = CDC_IN_EP,
	.ep_cb = usbd_cdc_acm_bulk_in
};

static struct usbd_interface intf0;
static struct usbd_interface intf1;

void cdc_acm_init(uint8_t busid, uint32_t reg_base)
{
	usbd_desc_register(busid, &cdc_descriptor);
	usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf0));
	usbd_add_interface(busid, usbd_cdc_acm_init_intf(busid, &intf1));
	usbd_add_endpoint(busid, &cdc_out_ep);
	usbd_add_endpoint(busid, &cdc_in_ep);
	usbd_initialize(busid, reg_base, usbd_event_handler);
}
