/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file test/cherryusb_interrupt_test/src/int_test.c
 * @brief 实现 CherryUSB 中断传输测试逻辑。
 * @details 该文件属于独立 Zephyr 测试镜像，只验证指定外设或算法路径，不会链接进主固件。测试会直接访问目标硬件并通过串口输出判定结果。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>

#include "usbd_core.h"

#define INT_OUT_EP 0x01
#define INT_IN_EP  0x81

#define INT_TEST_INTERFACE 0x00
#define INT_TEST_PACKET_HS 1024U
#define INT_TEST_PACKET_FS 64U
#define INT_TEST_INTERVAL  0x01
#define INT_TEST_CONFIG_SIZE (9 + 9 + 7 + 7)
#define INT_TEST_HEADER_SIZE 24U

enum int_test_cmd {
	INT_CMD_ECHO = 0U,
	INT_CMD_STREAM_IN_START = 1U,
	INT_CMD_STREAM_IN_STOP = 2U,
	INT_CMD_STREAM_OUT_START = 3U,
	INT_CMD_STREAM_OUT_STOP = 4U,
	INT_CMD_STREAM_OUT_DATA = 5U,
	INT_CMD_RESET = 6U,
};

struct int_test_frame {
	uint32_t magic;
	uint32_t seq;
	uint32_t host_us;
	uint32_t device_ms;
	uint32_t payload_len;
	uint32_t crc;
	uint8_t payload[INT_TEST_PACKET_HS - 24U];
} __packed;

#define INT_TEST_MAGIC 0x49544e54U /* "ITNT" little-endian marker */

static const uint8_t device_descriptor[] = {
	USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, USB_DEVICE_CLASS_VEND_SPECIFIC, 0x00, 0x00,
				   USBD_VID, USBD_PID, 0x0100, 0x01)
};

static const uint8_t config_descriptor_hs[] = {
	USB_CONFIG_DESCRIPTOR_INIT(INT_TEST_CONFIG_SIZE, 0x01, 0x01,
				   USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
	USB_INTERFACE_DESCRIPTOR_INIT(INT_TEST_INTERFACE, 0x00, 0x02,
				      USB_DEVICE_CLASS_VEND_SPECIFIC, 0x00, 0x00, 0x00),
	USB_ENDPOINT_DESCRIPTOR_INIT(INT_OUT_EP, USB_ENDPOINT_TYPE_INTERRUPT,
				     INT_TEST_PACKET_HS, INT_TEST_INTERVAL),
	USB_ENDPOINT_DESCRIPTOR_INIT(INT_IN_EP, USB_ENDPOINT_TYPE_INTERRUPT,
				     INT_TEST_PACKET_HS, INT_TEST_INTERVAL),
};

static const uint8_t config_descriptor_fs[] = {
	USB_CONFIG_DESCRIPTOR_INIT(INT_TEST_CONFIG_SIZE, 0x01, 0x01,
				   USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
	USB_INTERFACE_DESCRIPTOR_INIT(INT_TEST_INTERFACE, 0x00, 0x02,
				      USB_DEVICE_CLASS_VEND_SPECIFIC, 0x00, 0x00, 0x00),
	USB_ENDPOINT_DESCRIPTOR_INIT(INT_OUT_EP, USB_ENDPOINT_TYPE_INTERRUPT,
				     INT_TEST_PACKET_FS, INT_TEST_INTERVAL),
	USB_ENDPOINT_DESCRIPTOR_INIT(INT_IN_EP, USB_ENDPOINT_TYPE_INTERRUPT,
				     INT_TEST_PACKET_FS, INT_TEST_INTERVAL),
};

static const uint8_t device_quality_descriptor[] = {
	USB_DEVICE_QUALIFIER_DESCRIPTOR_INIT(USB_2_0, USB_DEVICE_CLASS_VEND_SPECIFIC,
					     0x00, 0x00, 0x01),
};

static const uint8_t other_speed_config_descriptor_hs[] = {
	USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT(INT_TEST_CONFIG_SIZE, 0x01, 0x01,
					      USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
	USB_INTERFACE_DESCRIPTOR_INIT(INT_TEST_INTERFACE, 0x00, 0x02,
				      USB_DEVICE_CLASS_VEND_SPECIFIC, 0x00, 0x00, 0x00),
	USB_ENDPOINT_DESCRIPTOR_INIT(INT_OUT_EP, USB_ENDPOINT_TYPE_INTERRUPT,
				     INT_TEST_PACKET_FS, INT_TEST_INTERVAL),
	USB_ENDPOINT_DESCRIPTOR_INIT(INT_IN_EP, USB_ENDPOINT_TYPE_INTERRUPT,
				     INT_TEST_PACKET_FS, INT_TEST_INTERVAL),
};

static const uint8_t other_speed_config_descriptor_fs[] = {
	USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT(INT_TEST_CONFIG_SIZE, 0x01, 0x01,
					      USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
	USB_INTERFACE_DESCRIPTOR_INIT(INT_TEST_INTERFACE, 0x00, 0x02,
				      USB_DEVICE_CLASS_VEND_SPECIFIC, 0x00, 0x00, 0x00),
	USB_ENDPOINT_DESCRIPTOR_INIT(INT_OUT_EP, USB_ENDPOINT_TYPE_INTERRUPT,
				     INT_TEST_PACKET_HS, INT_TEST_INTERVAL),
	USB_ENDPOINT_DESCRIPTOR_INIT(INT_IN_EP, USB_ENDPOINT_TYPE_INTERRUPT,
				     INT_TEST_PACKET_HS, INT_TEST_INTERVAL),
};

static const char *string_descriptors[] = {
	(const char[]){ 0x09, 0x04 },
	"HPMicro",
	"HPMicro Interrupt Test",
	"2026061401",
};

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX static uint8_t out_buffer[INT_TEST_PACKET_HS];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX static uint8_t in_buffer[INT_TEST_PACKET_HS];

static volatile bool in_busy;
static volatile bool out_ready;
static volatile bool stream_in_enabled;
static volatile bool stream_out_sink_enabled;
static volatile bool reset_ack_pending;
static uint32_t configured_mps = INT_TEST_PACKET_FS;
static uint32_t rx_count;
static uint32_t tx_count;
static uint32_t stream_in_seq;
static uint32_t stream_in_packet_len = INT_TEST_PACKET_FS;
static uint32_t stream_out_count;

static const uint8_t *device_descriptor_callback(uint8_t speed)
{
	(void)speed;
	return device_descriptor;
}

static const uint8_t *config_descriptor_callback(uint8_t speed)
{
	if (speed == USB_SPEED_HIGH) {
		return config_descriptor_hs;
	}
	if (speed == USB_SPEED_FULL) {
		return config_descriptor_fs;
	}
	return NULL;
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
	}
	if (speed == USB_SPEED_FULL) {
		return other_speed_config_descriptor_fs;
	}
	return NULL;
}

static const char *string_descriptor_callback(uint8_t speed, uint8_t index)
{
	(void)speed;

	if (index >= (sizeof(string_descriptors) / sizeof(char *))) {
		return NULL;
	}
	return string_descriptors[index];
}

static const struct usb_descriptor int_test_descriptor = {
	.device_descriptor_callback = device_descriptor_callback,
	.config_descriptor_callback = config_descriptor_callback,
	.device_quality_descriptor_callback = device_quality_descriptor_callback,
	.other_speed_descriptor_callback = other_speed_config_descriptor_callback,
	.string_descriptor_callback = string_descriptor_callback,
};

static uint32_t checksum32(const uint8_t *data, uint32_t len)
{
	uint32_t sum = 0U;

	for (uint32_t i = 0U; i < len; i++) {
		sum = (sum << 5) | (sum >> 27);
		sum += data[i];
	}

	return sum;
}

static void arm_out(uint8_t busid)
{
	int ret = usbd_ep_start_read(busid, INT_OUT_EP, out_buffer, configured_mps);

	out_ready = (ret == 0);
}

static void fill_frame(uint8_t *buffer, uint32_t seq, enum int_test_cmd cmd, uint32_t payload_len)
{
	struct int_test_frame *frame = (struct int_test_frame *)buffer;
	uint32_t total_len = payload_len + INT_TEST_HEADER_SIZE;

	if (total_len > configured_mps) {
		payload_len = configured_mps - INT_TEST_HEADER_SIZE;
	}

	memset(buffer, 0, configured_mps);
	frame->magic = INT_TEST_MAGIC;
	frame->seq = seq;
	frame->host_us = (uint32_t)cmd;
	frame->device_ms = (uint32_t)k_uptime_get_32();
	frame->payload_len = payload_len;
	if (payload_len > 0U) {
		frame->payload[0] = (uint8_t)cmd;
		for (uint32_t i = 1U; i < payload_len; i++) {
			frame->payload[i] = (uint8_t)(seq + i);
		}
	}
	frame->crc = checksum32(buffer, offsetof(struct int_test_frame, crc));
}

static void start_in_transfer(uint8_t busid, uint32_t len)
{
	in_busy = (usbd_ep_start_write(busid, INT_IN_EP, in_buffer, len) == 0);
	if (in_busy) {
		tx_count++;
	}
}

static void send_response(uint8_t busid, uint32_t nbytes)
{
	struct int_test_frame *frame = (struct int_test_frame *)in_buffer;
	uint32_t copy_len = nbytes;

	if (copy_len > configured_mps) {
		copy_len = configured_mps;
	}

	memcpy(in_buffer, out_buffer, copy_len);
	if (copy_len >= sizeof(*frame) - sizeof(frame->payload)) {
		frame->magic = INT_TEST_MAGIC;
		frame->device_ms = (uint32_t)k_uptime_get_32();
		frame->payload_len = copy_len - (sizeof(*frame) - sizeof(frame->payload));
		frame->crc = checksum32(in_buffer, offsetof(struct int_test_frame, crc));
	}

	start_in_transfer(busid, copy_len);
}

static void send_stream_in_frame(uint8_t busid)
{
	if (in_busy || !stream_in_enabled) {
		return;
	}

	fill_frame(in_buffer, stream_in_seq++, INT_CMD_STREAM_IN_START,
		   stream_in_packet_len - INT_TEST_HEADER_SIZE);
	start_in_transfer(busid, stream_in_packet_len);
}

static void send_reset_ack(uint8_t busid)
{
	if (in_busy) {
		reset_ack_pending = true;
		return;
	}

	reset_ack_pending = false;
	fill_frame(in_buffer, 0U, INT_CMD_RESET, 1U);
	start_in_transfer(busid, configured_mps);
}

static void int_out_callback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
	const struct int_test_frame *frame = (const struct int_test_frame *)out_buffer;
	enum int_test_cmd cmd = INT_CMD_ECHO;

	(void)ep;

	out_ready = false;
	rx_count++;

	if (nbytes > INT_TEST_HEADER_SIZE && frame->magic == INT_TEST_MAGIC) {
		cmd = (enum int_test_cmd)frame->payload[0];
	}

	if (cmd == INT_CMD_RESET) {
		stream_in_enabled = false;
		stream_out_sink_enabled = false;
		stream_in_seq = 0U;
		stream_out_count = 0U;
		send_reset_ack(busid);
	} else if (cmd == INT_CMD_STREAM_IN_START) {
		stream_in_enabled = true;
		stream_in_seq = 0U;
		stream_in_packet_len = nbytes;
		if (stream_in_packet_len < INT_TEST_HEADER_SIZE) {
			stream_in_packet_len = INT_TEST_HEADER_SIZE;
		}
		if (stream_in_packet_len > configured_mps) {
			stream_in_packet_len = configured_mps;
		}
		send_stream_in_frame(busid);
	} else if (cmd == INT_CMD_STREAM_IN_STOP) {
		stream_in_enabled = false;
		if (!in_busy) {
			send_response(busid, nbytes);
		}
	} else if (cmd == INT_CMD_STREAM_OUT_START) {
		stream_out_sink_enabled = true;
		stream_out_count = 0U;
	} else if (cmd == INT_CMD_STREAM_OUT_STOP) {
		stream_out_sink_enabled = false;
		if (!in_busy) {
			fill_frame(in_buffer, stream_out_count, INT_CMD_STREAM_OUT_STOP, 1U);
			start_in_transfer(busid, configured_mps);
		}
	} else if (stream_out_sink_enabled || cmd == INT_CMD_STREAM_OUT_DATA) {
		stream_out_count++;
	} else if (!in_busy) {
		send_response(busid, nbytes);
	}

	arm_out(busid);
}

static void int_in_callback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
	(void)busid;
	(void)ep;
	(void)nbytes;

	in_busy = false;
	if (reset_ack_pending) {
		send_reset_ack(busid);
		return;
	}
	send_stream_in_frame(busid);
}

static void int_test_event_handler(uint8_t busid, uint8_t event)
{
	switch (event) {
	case USBD_EVENT_CONFIGURED:
		configured_mps = usbd_get_ep_mps(busid, INT_OUT_EP);
		if (configured_mps == 0U || configured_mps > sizeof(out_buffer)) {
			configured_mps = INT_TEST_PACKET_FS;
		}
			in_busy = false;
			out_ready = false;
			stream_in_enabled = false;
			stream_out_sink_enabled = false;
			reset_ack_pending = false;
			rx_count = 0U;
			tx_count = 0U;
			stream_in_seq = 0U;
			stream_in_packet_len = configured_mps;
			stream_out_count = 0U;
		printf("USB interrupt configured, speed=%d, mps=%u\n",
		       usbd_get_port_speed(busid), configured_mps);
		arm_out(busid);
		break;
	case USBD_EVENT_RESET:
	case USBD_EVENT_DISCONNECTED:
		in_busy = false;
		out_ready = false;
		stream_in_enabled = false;
		stream_out_sink_enabled = false;
		reset_ack_pending = false;
		break;
	default:
		break;
	}
}

static struct usbd_interface intf0;

static struct usbd_endpoint int_out_ep = {
	.ep_addr = INT_OUT_EP,
	.ep_cb = int_out_callback,
};

static struct usbd_endpoint int_in_ep = {
	.ep_addr = INT_IN_EP,
	.ep_cb = int_in_callback,
};

void int_test_init(uint8_t busid, uint32_t reg_base)
{
	memset(&intf0, 0, sizeof(intf0));

	usbd_desc_register(busid, &int_test_descriptor);
	usbd_add_interface(busid, &intf0);
	usbd_add_endpoint(busid, &int_out_ep);
	usbd_add_endpoint(busid, &int_in_ep);
	usbd_initialize(busid, reg_base, int_test_event_handler);
}
