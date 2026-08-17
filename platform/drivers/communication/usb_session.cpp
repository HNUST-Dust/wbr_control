/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <channels/booster_state.hpp>
#include <channels/gimbal_state.hpp>
#include <channels/hi91_imu_sample.hpp>
#include <channels/pc_auto_aim_command.hpp>
#include <channels/remote_input_state.hpp>
#include <channels/usb_raw_frame_queue.h>
#include <platform/drivers/communication/usb_session.h>
#include <protocols/pc_link/pc_link.h>
#include <scheduling/periodic_schedule.h>
#include <scheduling/thread_priorities.h>

#include "usbd_core.h"
#include "usbd_cdc_acm.h"

LOG_MODULE_REGISTER(usb_session, LOG_LEVEL_INF);

namespace {

#if defined(CONFIG_CHERRYUSB) && CONFIG_CHERRYUSB && defined(CONFIG_CHERRYUSB_DEVICE) &&                 \
	CONFIG_CHERRYUSB_DEVICE && defined(CONFIG_CHERRYUSB_DEVICE_CDC_ACM) &&                             \
	CONFIG_CHERRYUSB_DEVICE_CDC_ACM

constexpr uint8_t kBusId = 0U;

#if DT_NODE_EXISTS(DT_NODELABEL(cherryusb_usb0))
constexpr uintptr_t kUsbRegBase = DT_REG_ADDR(DT_NODELABEL(cherryusb_usb0));
constexpr bool kHasUsbNode = true;
#else
constexpr uintptr_t kUsbRegBase = 0U;
constexpr bool kHasUsbNode = false;
#endif

constexpr uint8_t kCdcInEp = 0x81U;
constexpr uint8_t kCdcOutEp = 0x01U;
constexpr uint8_t kCdcIntEp = 0x83U;
constexpr size_t kCdcBufferSize = channels::kUsbRawChunkSize;

#define RM_TEST_USB_CONFIG_SIZE (9 + CDC_ACM_DESCRIPTOR_LEN)

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t g_read_buffer[2][kCdcBufferSize];
uint8_t g_tx_buffer[kCdcBufferSize];

volatile bool g_started = false;
volatile bool g_configured = false;
volatile bool g_tx_busy = false;
volatile uint8_t g_read_index = 0U;

static const uint8_t g_device_descriptor[] = {
	USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01)
};

static const uint8_t g_config_descriptor_hs[] = {
	USB_CONFIG_DESCRIPTOR_INIT(
		RM_TEST_USB_CONFIG_SIZE,
		0x02,
		0x01,
		USB_CONFIG_BUS_POWERED,
		USBD_MAX_POWER),
	CDC_ACM_DESCRIPTOR_INIT(0x00, kCdcIntEp, kCdcOutEp, kCdcInEp, USB_BULK_EP_MPS_HS, 0x02),
};

static const uint8_t g_config_descriptor_fs[] = {
	USB_CONFIG_DESCRIPTOR_INIT(
		RM_TEST_USB_CONFIG_SIZE,
		0x02,
		0x01,
		USB_CONFIG_BUS_POWERED,
		USBD_MAX_POWER),
	CDC_ACM_DESCRIPTOR_INIT(0x00, kCdcIntEp, kCdcOutEp, kCdcInEp, USB_BULK_EP_MPS_FS, 0x02),
};

static const uint8_t g_device_quality_descriptor[] = {
	USB_DEVICE_QUALIFIER_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, 0x01),
};

static const uint8_t g_other_speed_config_descriptor_hs[] = {
	USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT(
		RM_TEST_USB_CONFIG_SIZE,
		0x02,
		0x01,
		USB_CONFIG_BUS_POWERED,
		USBD_MAX_POWER),
	CDC_ACM_DESCRIPTOR_INIT(0x00, kCdcIntEp, kCdcOutEp, kCdcInEp, USB_BULK_EP_MPS_FS, 0x02),
};

static const uint8_t g_other_speed_config_descriptor_fs[] = {
	USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT(
		RM_TEST_USB_CONFIG_SIZE,
		0x02,
		0x01,
		USB_CONFIG_BUS_POWERED,
		USBD_MAX_POWER),
	CDC_ACM_DESCRIPTOR_INIT(0x00, kCdcIntEp, kCdcOutEp, kCdcInEp, USB_BULK_EP_MPS_HS, 0x02),
};

static const char *g_string_descriptors[] = {
	(const char[]){0x09, 0x04},
	"HPMicro",
	"rm_test USB Session",
	"0001",
};

static const uint8_t *DeviceDescriptorCallback(uint8_t speed)
{
	ARG_UNUSED(speed);
	return g_device_descriptor;
}

static const uint8_t *ConfigDescriptorCallback(uint8_t speed)
{
	if (speed == USB_SPEED_HIGH) {
		return g_config_descriptor_hs;
	}

	if (speed == USB_SPEED_FULL) {
		return g_config_descriptor_fs;
	}

	return nullptr;
}

static const uint8_t *DeviceQualityDescriptorCallback(uint8_t speed)
{
	ARG_UNUSED(speed);
	return g_device_quality_descriptor;
}

static const uint8_t *OtherSpeedDescriptorCallback(uint8_t speed)
{
	if (speed == USB_SPEED_HIGH) {
		return g_other_speed_config_descriptor_hs;
	}

	if (speed == USB_SPEED_FULL) {
		return g_other_speed_config_descriptor_fs;
	}

	return nullptr;
}

static const char *StringDescriptorCallback(uint8_t speed, uint8_t index)
{
	ARG_UNUSED(speed);

	if (index >= ARRAY_SIZE(g_string_descriptors)) {
		return nullptr;
	}

	return g_string_descriptors[index];
}

const struct usb_descriptor g_cdc_descriptor = {
	.device_descriptor_callback = DeviceDescriptorCallback,
	.config_descriptor_callback = ConfigDescriptorCallback,
	.device_quality_descriptor_callback = DeviceQualityDescriptorCallback,
	.other_speed_descriptor_callback = OtherSpeedDescriptorCallback,
	.string_descriptor_callback = StringDescriptorCallback,
};

void UsbEventHandler(uint8_t busid, uint8_t event)
{
	ARG_UNUSED(busid);

	switch (event) {
	case USBD_EVENT_CONFIGURED:
		g_configured = true;
		g_read_index = 0U;
		(void)usbd_ep_start_read(
			kBusId,
			kCdcOutEp,
			&g_read_buffer[g_read_index][0],
			usbd_get_ep_mps(kBusId, kCdcOutEp));
		break;
	case USBD_EVENT_RESET:
	case USBD_EVENT_DISCONNECTED:
		g_configured = false;
		g_tx_busy = false;
		break;
	default:
		break;
	}
}

void UsbBulkOutCallback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
	ARG_UNUSED(busid);

	const uint8_t index = g_read_index;
	const size_t copy_len = MIN(static_cast<size_t>(nbytes), kCdcBufferSize);

	if (copy_len > 0U) {
		channels::UsbRawFrameMessage frame = {};
		frame.len = static_cast<uint16_t>(copy_len);
		memcpy(frame.data, &g_read_buffer[index][0], copy_len);
		(void)channels::EnqueueForCdcAcm(&frame);
	}

	g_read_index = (index == 0U) ? 1U : 0U;
	(void)usbd_ep_start_read(
		kBusId,
		ep,
		&g_read_buffer[g_read_index][0],
		usbd_get_ep_mps(kBusId, ep));
}

void UsbBulkInCallback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
	if ((nbytes > 0U) && ((nbytes % usbd_get_ep_mps(busid, ep)) == 0U)) {
		(void)usbd_ep_start_write(busid, ep, nullptr, 0U);
		return;
	}

	g_tx_busy = false;
}

struct usbd_endpoint g_cdc_out_ep = {
	.ep_addr = kCdcOutEp,
	.ep_cb = UsbBulkOutCallback,
};

struct usbd_endpoint g_cdc_in_ep = {
	.ep_addr = kCdcInEp,
	.ep_cb = UsbBulkInCallback,
};

static struct usbd_interface g_intf0;
static struct usbd_interface g_intf1;

#if defined(CONFIG_RM_TEST_PC_LINK) && CONFIG_RM_TEST_PC_LINK
constexpr uint32_t kPcLinkPeriodMs = CONFIG_RM_TEST_PC_LINK_PERIOD_MS;

K_THREAD_STACK_DEFINE(g_pc_link_stack, 1536);
k_thread g_pc_link_thread;
bool g_pc_link_started = false;

uint8_t g_pc_rx_buf[protocols::kPcCommRecvPacketSize];
uint8_t g_pc_rx_state = 0U;
uint8_t g_pc_rx_pos = 0U;

void PublishAutoAimCommand(const protocols::PCRecvAutoAimData *recv)
{
	channels::PcAutoAimCommand command = {};
	command.mode = recv->mode;
	command.yaw_angle = recv->yaw.yaw_ang;
	command.yaw_velocity = recv->yaw.yaw_vel;
	command.yaw_acceleration = recv->yaw.yaw_acc;
	command.pitch_angle = recv->pitch.pitch_ang;
	command.pitch_velocity = recv->pitch.pitch_vel;
	command.pitch_acceleration = recv->pitch.pitch_acc;
	channels::latest_pc_auto_aim_command.write(command);
}

void FeedPcRxByte(uint8_t byte)
{
	switch (g_pc_rx_state) {
	case 0U: /* wait 'S' */
		if (byte == 'S') {
			g_pc_rx_buf[0] = byte;
			g_pc_rx_pos = 1U;
			g_pc_rx_state = 1U;
		}
		break;
	case 1U: /* wait 'P' */
		if (byte == 'P') {
			g_pc_rx_buf[1] = byte;
			g_pc_rx_pos = 2U;
			g_pc_rx_state = 2U;
		} else if (byte == 'S') {
			g_pc_rx_buf[0] = byte;
			g_pc_rx_pos = 1U;
		} else {
			g_pc_rx_state = 0U;
			g_pc_rx_pos = 0U;
		}
		break;
	case 2U: /* collect the rest of the fixed-size packet */
		g_pc_rx_buf[g_pc_rx_pos++] = byte;
		if (g_pc_rx_pos == protocols::kPcCommRecvPacketSize) {
			protocols::PCRecvAutoAimData recv = {};
			if (protocols::DecodePcCommRecv(g_pc_rx_buf, g_pc_rx_pos, &recv) == 0) {
				PublishAutoAimCommand(&recv);
			}
			g_pc_rx_state = 0U;
			g_pc_rx_pos = 0U;
		}
		break;
	default:
		g_pc_rx_state = 0U;
		g_pc_rx_pos = 0U;
		break;
	}
}

void DrainPcRx()
{
	channels::UsbRawFrameMessage chunk = {};
	while (channels::DequeueForCdcAcm(&chunk, 0) == 0) {
		for (uint16_t i = 0U; i < chunk.len; ++i) {
			FeedPcRxByte(chunk.data[i]);
		}
	}
}

void SendPcLinkFrame()
{
	protocols::PCSendAutoAimData data = {};

	channels::RemoteInputState input = {};
	if (::latest_remote_state.read(input)) {
		data.mode = input.auto_aim ? 1U : 0U;
	}

	channels::Hi91ImuSample imu = {};
	if (channels::latest_hi91_imu_sample.read(imu)) {
		memcpy(data.q, imu.quat, sizeof(data.q));
	}

	channels::GimbalState gimbal = {};
	if (channels::latest_gimbal_state.read(gimbal)) {
		data.yaw.yaw_ang = gimbal.yaw_angle;
		data.yaw.yaw_vel = gimbal.yaw_velocity;
		data.pitch.pitch_ang = gimbal.pitch_angle;
		data.pitch.pitch_vel = gimbal.pitch_velocity;
	}

	channels::BoosterState booster = {};
	if (channels::latest_booster_state.read(booster)) {
		data.bullet.bullet_speed = booster.bullet_speed;
		data.bullet.bullet_count = booster.bullet_count;
	}

	uint8_t packet[protocols::kPcCommSendPacketSize];
	size_t packet_len = 0U;
	if (protocols::EncodePcCommSend(&data, packet, sizeof(packet), &packet_len) != 0) {
		return;
	}

	(void)platform::SendUsb(packet, packet_len);
}

void PcLinkLoop(void *, void *, void *)
{
	wbr_control::scheduling::AbsolutePeriodicSchedule release(
		kPcLinkPeriodMs, wbr_control::scheduling::thread_phase_ms::kPcLink);
	for (;;) {
		(void)release.WaitForNextRelease();
		DrainPcRx();
		SendPcLinkFrame();
	}
}

int StartPcLinkWorker()
{
	if (g_pc_link_started) {
		return 0;
	}

	const k_tid_t tid = k_thread_create(
		&g_pc_link_thread, g_pc_link_stack, K_THREAD_STACK_SIZEOF(g_pc_link_stack),
		PcLinkLoop, nullptr, nullptr, nullptr,
		K_PRIO_PREEMPT(wbr_control::scheduling::thread_priority::kPcLink), 0, K_NO_WAIT);
	if (tid == nullptr) {
		return -EINVAL;
	}

	k_thread_name_set(tid, "pc_link");
	g_pc_link_started = true;
	LOG_INF("pc_link worker started period=%u ms",
		static_cast<unsigned int>(kPcLinkPeriodMs));
	return 0;
}
#endif /* CONFIG_RM_TEST_PC_LINK */

#endif

}  // namespace

namespace platform {

int InitializeUsbSession()
{
#if defined(CONFIG_CHERRYUSB) && CONFIG_CHERRYUSB && defined(CONFIG_CHERRYUSB_DEVICE) &&                 \
	CONFIG_CHERRYUSB_DEVICE && defined(CONFIG_CHERRYUSB_DEVICE_CDC_ACM) &&                             \
	CONFIG_CHERRYUSB_DEVICE_CDC_ACM
	if (g_started) {
		return 0;
	}

	if (!kHasUsbNode) {
		return -ENODEV;
	}

	usbd_desc_register(kBusId, &g_cdc_descriptor);
	usbd_add_interface(kBusId, usbd_cdc_acm_init_intf(kBusId, &g_intf0));
	usbd_add_interface(kBusId, usbd_cdc_acm_init_intf(kBusId, &g_intf1));
	usbd_add_endpoint(kBusId, &g_cdc_out_ep);
	usbd_add_endpoint(kBusId, &g_cdc_in_ep);

	const int rc = usbd_initialize(kBusId, kUsbRegBase, UsbEventHandler);
	if (rc < 0) {
		return rc;
	}

	g_started = true;

#if defined(CONFIG_RM_TEST_PC_LINK) && CONFIG_RM_TEST_PC_LINK
	{
		const int pc_rc = StartPcLinkWorker();
		if (pc_rc != 0) {
			return pc_rc;
		}
	}
#endif

	return 0;
#else
	return -ENOTSUP;
#endif
}

bool IsUsbConfigured()
{
#if defined(CONFIG_CHERRYUSB) && CONFIG_CHERRYUSB && defined(CONFIG_CHERRYUSB_DEVICE) &&                 \
	CONFIG_CHERRYUSB_DEVICE && defined(CONFIG_CHERRYUSB_DEVICE_CDC_ACM) &&                             \
	CONFIG_CHERRYUSB_DEVICE_CDC_ACM
	return g_started && g_configured;
#else
	return false;
#endif
}

int SendUsb(const uint8_t *data, size_t len)
{
#if defined(CONFIG_CHERRYUSB) && CONFIG_CHERRYUSB && defined(CONFIG_CHERRYUSB_DEVICE) &&                 \
	CONFIG_CHERRYUSB_DEVICE && defined(CONFIG_CHERRYUSB_DEVICE_CDC_ACM) &&                             \
	CONFIG_CHERRYUSB_DEVICE_CDC_ACM
	if ((data == nullptr) || (len == 0U)) {
		return -EINVAL;
	}

	if (!IsUsbConfigured()) {
		return -EAGAIN;
	}

	if (len > sizeof(g_tx_buffer)) {
		return -EMSGSIZE;
	}

	if (g_tx_busy) {
		return -EBUSY;
	}

	memcpy(g_tx_buffer, data, len);
	g_tx_busy = true;

	const int rc = usbd_ep_start_write(kBusId, kCdcInEp, g_tx_buffer, static_cast<uint32_t>(len));
	if (rc < 0) {
		g_tx_busy = false;
		return rc;
	}

	return static_cast<int>(len);
#else
	ARG_UNUSED(data);
	ARG_UNUSED(len);
	return -ENOTSUP;
#endif
}

int ReceiveUsb(uint8_t *out, size_t capacity, size_t *out_len, int32_t timeout_ms)
{
	if ((out == nullptr) || (out_len == nullptr) || (capacity == 0U)) {
		return -EINVAL;
	}

	*out_len = 0U;

#if defined(CONFIG_CHERRYUSB) && CONFIG_CHERRYUSB && defined(CONFIG_CHERRYUSB_DEVICE) &&                 \
	CONFIG_CHERRYUSB_DEVICE && defined(CONFIG_CHERRYUSB_DEVICE_CDC_ACM) &&                             \
	CONFIG_CHERRYUSB_DEVICE_CDC_ACM
	channels::UsbRawFrameMessage chunk = {};
	const int rc = channels::DequeueForCdcAcm(&chunk, timeout_ms);
	if (rc != 0) {
		return rc;
	}

	const size_t copy_len = MIN(capacity, static_cast<size_t>(chunk.len));
	memcpy(out, chunk.data, copy_len);
	*out_len = copy_len;
	return (copy_len == static_cast<size_t>(chunk.len)) ? 0 : -EMSGSIZE;
#else
	ARG_UNUSED(timeout_ms);
	return -ENOTSUP;
#endif
}

}  // namespace platform
