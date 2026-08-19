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

LOG_MODULE_REGISTER(usb_session, LOG_LEVEL_INF);

namespace {

#if defined(CONFIG_CHERRYUSB) && CONFIG_CHERRYUSB && defined(CONFIG_CHERRYUSB_DEVICE) && \
	CONFIG_CHERRYUSB_DEVICE

constexpr uint8_t kBusId = 0U;

#if DT_NODE_EXISTS(DT_NODELABEL(cherryusb_usb0))
constexpr uintptr_t kUsbRegBase = DT_REG_ADDR(DT_NODELABEL(cherryusb_usb0));
constexpr bool kHasUsbNode = true;
#else
constexpr uintptr_t kUsbRegBase = 0U;
constexpr bool kHasUsbNode = false;
#endif

constexpr uint8_t kVendorInEp = 0x81U;
constexpr uint8_t kVendorOutEp = 0x01U;
constexpr uint8_t kVendorInterface = 0x00U;
constexpr uint16_t kVendorPacketSize = 64U;
constexpr uint8_t kVendorIntervalFs = 0x01U; /* 1 frame = 1 ms */
constexpr uint8_t kVendorIntervalHs = 0x04U; /* 8 microframes = 1 ms */

#define WBR_CONTROL_VENDOR_INTERRUPT_DESCRIPTOR_LEN (9 + 7 + 7)
#define WBR_CONTROL_USB_CONFIG_SIZE (9 + WBR_CONTROL_VENDOR_INTERRUPT_DESCRIPTOR_LEN)

static_assert(protocols::kPcCommRecvPacketSize <= kVendorPacketSize,
	      "PC command must fit one Vendor Interrupt OUT packet");
static_assert(protocols::kPcCommSendPacketSize <= kVendorPacketSize,
	      "PC telemetry must fit one Vendor Interrupt IN packet");

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX
uint8_t g_vendor_read_buffer[protocols::kPcCommRecvPacketSize];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t g_vendor_tx_buffer[kVendorPacketSize];

volatile bool g_started = false;
volatile bool g_configured = false;
volatile bool g_vendor_tx_busy = false;
volatile bool g_vendor_out_armed = false;

#if defined(CONFIG_WBR_CONTROL_PC_LINK) && CONFIG_WBR_CONTROL_PC_LINK
K_SEM_DEFINE(g_pc_link_tx_kick, 0, 1);
volatile bool g_pc_command_clear_requested = true;
bool g_has_valid_pc_command = false;
uint32_t g_last_valid_pc_command_ms = 0U;
uint32_t g_pc_command_sequence = 0U;
#endif

static const uint8_t g_device_descriptor[] = {
	USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, USBD_VID, USBD_PID, 0x0100, 0x01)
};

static const uint8_t g_config_descriptor_hs[] = {
	USB_CONFIG_DESCRIPTOR_INIT(
		WBR_CONTROL_USB_CONFIG_SIZE,
		0x01,
		0x01,
		USB_CONFIG_BUS_POWERED,
		USBD_MAX_POWER),
	USB_INTERFACE_DESCRIPTOR_INIT(kVendorInterface, 0x00, 0x02,
		USB_DEVICE_CLASS_VEND_SPECIFIC, 0x00, 0x00, 0x00),
	USB_ENDPOINT_DESCRIPTOR_INIT(kVendorOutEp, USB_ENDPOINT_TYPE_INTERRUPT,
		kVendorPacketSize, kVendorIntervalHs),
	USB_ENDPOINT_DESCRIPTOR_INIT(kVendorInEp, USB_ENDPOINT_TYPE_INTERRUPT,
		kVendorPacketSize, kVendorIntervalHs),
};

static const uint8_t g_config_descriptor_fs[] = {
	USB_CONFIG_DESCRIPTOR_INIT(
		WBR_CONTROL_USB_CONFIG_SIZE,
		0x01,
		0x01,
		USB_CONFIG_BUS_POWERED,
		USBD_MAX_POWER),
	USB_INTERFACE_DESCRIPTOR_INIT(kVendorInterface, 0x00, 0x02,
		USB_DEVICE_CLASS_VEND_SPECIFIC, 0x00, 0x00, 0x00),
	USB_ENDPOINT_DESCRIPTOR_INIT(kVendorOutEp, USB_ENDPOINT_TYPE_INTERRUPT,
		kVendorPacketSize, kVendorIntervalFs),
	USB_ENDPOINT_DESCRIPTOR_INIT(kVendorInEp, USB_ENDPOINT_TYPE_INTERRUPT,
		kVendorPacketSize, kVendorIntervalFs),
};

static const uint8_t g_device_quality_descriptor[] = {
	USB_DEVICE_QUALIFIER_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, 0x01),
};

static const uint8_t g_other_speed_config_descriptor_hs[] = {
	USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT(
		WBR_CONTROL_USB_CONFIG_SIZE,
		0x01,
		0x01,
		USB_CONFIG_BUS_POWERED,
		USBD_MAX_POWER),
	USB_INTERFACE_DESCRIPTOR_INIT(kVendorInterface, 0x00, 0x02,
		USB_DEVICE_CLASS_VEND_SPECIFIC, 0x00, 0x00, 0x00),
	USB_ENDPOINT_DESCRIPTOR_INIT(kVendorOutEp, USB_ENDPOINT_TYPE_INTERRUPT,
		kVendorPacketSize, kVendorIntervalFs),
	USB_ENDPOINT_DESCRIPTOR_INIT(kVendorInEp, USB_ENDPOINT_TYPE_INTERRUPT,
		kVendorPacketSize, kVendorIntervalFs),
};

static const uint8_t g_other_speed_config_descriptor_fs[] = {
	USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT(
		WBR_CONTROL_USB_CONFIG_SIZE,
		0x01,
		0x01,
		USB_CONFIG_BUS_POWERED,
		USBD_MAX_POWER),
	USB_INTERFACE_DESCRIPTOR_INIT(kVendorInterface, 0x00, 0x02,
		USB_DEVICE_CLASS_VEND_SPECIFIC, 0x00, 0x00, 0x00),
	USB_ENDPOINT_DESCRIPTOR_INIT(kVendorOutEp, USB_ENDPOINT_TYPE_INTERRUPT,
		kVendorPacketSize, kVendorIntervalHs),
	USB_ENDPOINT_DESCRIPTOR_INIT(kVendorInEp, USB_ENDPOINT_TYPE_INTERRUPT,
		kVendorPacketSize, kVendorIntervalHs),
};

static const char *g_string_descriptors[] = {
	(const char[]){0x09, 0x04},
	"HPMicro",
	"wbr_control USB Session",
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

const struct usb_descriptor g_usb_descriptor = {
	.device_descriptor_callback = DeviceDescriptorCallback,
	.config_descriptor_callback = ConfigDescriptorCallback,
	.device_quality_descriptor_callback = DeviceQualityDescriptorCallback,
	.other_speed_descriptor_callback = OtherSpeedDescriptorCallback,
	.string_descriptor_callback = StringDescriptorCallback,
};

int ArmVendorOut()
{
	if (!g_configured || g_vendor_out_armed) {
		return 0;
	}

	const int rc = usbd_ep_start_read(
		kBusId, kVendorOutEp, g_vendor_read_buffer, sizeof(g_vendor_read_buffer));
	if (rc < 0) {
		return rc;
	}

	g_vendor_out_armed = true;
	return 0;
}

void UsbEventHandler(uint8_t busid, uint8_t event)
{
	ARG_UNUSED(busid);

	switch (event) {
	case USBD_EVENT_CONFIGURED:
		g_configured = true;
		g_vendor_tx_busy = false;
		g_vendor_out_armed = false;
	#if defined(CONFIG_WBR_CONTROL_PC_LINK) && CONFIG_WBR_CONTROL_PC_LINK
		g_pc_command_clear_requested = true;
		k_sem_give(&g_pc_link_tx_kick);
	#endif
		(void)ArmVendorOut();
		break;
	case USBD_EVENT_RESET:
		g_configured = false;
		g_vendor_tx_busy = false;
		g_vendor_out_armed = false;
	#if defined(CONFIG_WBR_CONTROL_PC_LINK) && CONFIG_WBR_CONTROL_PC_LINK
		g_pc_command_clear_requested = true;
	#endif
		break;
	case USBD_EVENT_DISCONNECTED:
		g_configured = false;
		g_vendor_tx_busy = false;
		g_vendor_out_armed = false;
	#if defined(CONFIG_WBR_CONTROL_PC_LINK) && CONFIG_WBR_CONTROL_PC_LINK
		g_pc_command_clear_requested = true;
	#endif
		break;
	default:
		break;
	}
}

void EnqueueUsbRx(const uint8_t *data, size_t len)
{
	const size_t copy_len = MIN(len, channels::kUsbRawChunkSize);
	if ((data == nullptr) || (copy_len == 0U)) {
		return;
	}

	channels::UsbRawFrameMessage frame = {};
	frame.len = static_cast<uint16_t>(copy_len);
	memcpy(frame.data, data, copy_len);
	(void)channels::EnqueueUsbRawFrame(&frame);
}

void UsbInterruptOutCallback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
	ARG_UNUSED(busid);

	g_vendor_out_armed = false;
	EnqueueUsbRx(g_vendor_read_buffer, MIN(static_cast<size_t>(nbytes),
					       sizeof(g_vendor_read_buffer)));

	/*
	 * Do not reuse the HPM qTD from inside its completion callback.  The HPM
	 * port invokes this callback before USBD_IRQHandler() has finished walking
	 * and retiring the completed qTD.  Re-priming the same endpoint here can
	 * therefore be reported as successful while the new qTD never becomes
	 * active.  PcLinkLoop observes g_vendor_out_armed == false and submits the
	 * next receive after the USB ISR has returned.
	 */
	ARG_UNUSED(ep);
}

void UsbInterruptInCallback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
	ARG_UNUSED(busid);
	ARG_UNUSED(ep);
	ARG_UNUSED(nbytes);
	g_vendor_tx_busy = false;
	#if defined(CONFIG_WBR_CONTROL_PC_LINK) && CONFIG_WBR_CONTROL_PC_LINK
	k_sem_give(&g_pc_link_tx_kick);
	#endif
}

struct usbd_endpoint g_vendor_out_ep = {
	.ep_addr = kVendorOutEp,
	.ep_cb = UsbInterruptOutCallback,
};

struct usbd_endpoint g_vendor_in_ep = {
	.ep_addr = kVendorInEp,
	.ep_cb = UsbInterruptInCallback,
};

static struct usbd_interface g_vendor_intf;

#if defined(CONFIG_WBR_CONTROL_PC_LINK) && CONFIG_WBR_CONTROL_PC_LINK
constexpr uint32_t kPcLinkPeriodMs = CONFIG_WBR_CONTROL_PC_LINK_PERIOD_MS;
constexpr uint32_t kPcCommandTimeoutMs = CONFIG_WBR_CONTROL_PC_LINK_COMMAND_TIMEOUT_MS;

K_THREAD_STACK_DEFINE(g_pc_link_stack, 1536);
k_thread g_pc_link_thread;
bool g_pc_link_started = false;
K_THREAD_STACK_DEFINE(g_pc_link_tx_stack, 1536);
k_thread g_pc_link_tx_thread;

void PublishIdlePcCommand()
{
	channels::PcAutoAimCommand command = {};
	command.sequence = ++g_pc_command_sequence;
	channels::latest_pc_auto_aim_command.write(command);
	g_has_valid_pc_command = false;
}

void PublishAutoAimCommand(const protocols::PCRecvAutoAimData *recv)
{
	channels::PcAutoAimCommand command = {};
	command.sequence = ++g_pc_command_sequence;
	command.mode = recv->mode;
	command.yaw_angle = recv->yaw.yaw_ang;
	command.yaw_velocity = recv->yaw.yaw_vel;
	command.yaw_acceleration = recv->yaw.yaw_acc;
	command.pitch_angle = recv->pitch.pitch_ang;
	command.pitch_velocity = recv->pitch.pitch_vel;
	command.pitch_acceleration = recv->pitch.pitch_acc;
	channels::latest_pc_auto_aim_command.write(command);
	g_last_valid_pc_command_ms = k_uptime_get_32();
	g_has_valid_pc_command = true;
}

void EnforcePcCommandSafety()
{
	if (g_pc_command_clear_requested) {
		g_pc_command_clear_requested = false;
		PublishIdlePcCommand();
		return;
	}

	if (g_has_valid_pc_command &&
	    ((k_uptime_get_32() - g_last_valid_pc_command_ms) > kPcCommandTimeoutMs)) {
		PublishIdlePcCommand();
	}
}

void DrainPcRx()
{
	channels::UsbRawFrameMessage chunk = {};
	while (channels::DequeueUsbRawFrame(&chunk, 0) == 0) {
		if (chunk.len != protocols::kPcCommRecvPacketSize) {
			continue;
		}

		if ((chunk.data[0] != 'S') || (chunk.data[1] != 'P')) {
			continue;
		}

		protocols::PCRecvAutoAimData recv = {};
		if (protocols::DecodePcCommRecv(chunk.data, chunk.len, &recv) == 0) {
			PublishAutoAimCommand(&recv);
		}
	}
}

int SendPcLinkFrame()
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
		return -EINVAL;
	}

	return platform::SendUsb(packet, packet_len);
}

void PcLinkLoop(void *, void *, void *)
{
	wbr_control::scheduling::AbsolutePeriodicSchedule release(
		kPcLinkPeriodMs, wbr_control::scheduling::thread_phase_ms::kPcLink);
	for (;;) {
		(void)release.WaitForNextRelease();
		if (g_configured && !g_vendor_out_armed) {
			(void)ArmVendorOut();
		}
		EnforcePcCommandSafety();
		DrainPcRx();
	}
}

void PcLinkTxLoop(void *, void *, void *)
{
	for (;;) {
		(void)k_sem_take(&g_pc_link_tx_kick, K_FOREVER);
		if (!g_configured) {
			continue;
		}

		const int rc = SendPcLinkFrame();
		if ((rc < 0) && (rc != -EBUSY) && g_configured) {
			/* A start failure has no completion callback to kick the next try. */
			k_sleep(K_MSEC(1));
			k_sem_give(&g_pc_link_tx_kick);
		}
	}
}

int StartPcLinkWorker()
{
	if (g_pc_link_started) {
		return 0;
	}

	const k_tid_t tx_tid = k_thread_create(
		&g_pc_link_tx_thread, g_pc_link_tx_stack,
		K_THREAD_STACK_SIZEOF(g_pc_link_tx_stack), PcLinkTxLoop,
		nullptr, nullptr, nullptr,
		K_PRIO_PREEMPT(wbr_control::scheduling::thread_priority::kPcLinkTx),
		0, K_NO_WAIT);
	if (tx_tid == nullptr) {
		return -EINVAL;
	}
	k_thread_name_set(tx_tid, "pc_link_tx");

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
#endif /* CONFIG_WBR_CONTROL_PC_LINK */

#endif

}  // namespace

namespace platform {

int InitializeUsbSession()
{
#if defined(CONFIG_CHERRYUSB) && CONFIG_CHERRYUSB && defined(CONFIG_CHERRYUSB_DEVICE) && \
	CONFIG_CHERRYUSB_DEVICE
	if (g_started) {
		return 0;
	}

	if (!kHasUsbNode) {
		return -ENODEV;
	}

	usbd_desc_register(kBusId, &g_usb_descriptor);
	usbd_add_interface(kBusId, &g_vendor_intf);
	usbd_add_endpoint(kBusId, &g_vendor_out_ep);
	usbd_add_endpoint(kBusId, &g_vendor_in_ep);

	const int rc = usbd_initialize(kBusId, kUsbRegBase, UsbEventHandler);
	if (rc < 0) {
		return rc;
	}

	g_started = true;

#if defined(CONFIG_WBR_CONTROL_PC_LINK) && CONFIG_WBR_CONTROL_PC_LINK
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
#if defined(CONFIG_CHERRYUSB) && CONFIG_CHERRYUSB && defined(CONFIG_CHERRYUSB_DEVICE) && \
	CONFIG_CHERRYUSB_DEVICE
	return g_started && g_configured;
#else
	return false;
#endif
}

int SendUsb(const uint8_t *data, size_t len)
{
#if defined(CONFIG_CHERRYUSB) && CONFIG_CHERRYUSB && defined(CONFIG_CHERRYUSB_DEVICE) && \
	CONFIG_CHERRYUSB_DEVICE
	if ((data == nullptr) || (len == 0U)) {
		return -EINVAL;
	}

	if (!IsUsbConfigured()) {
		return -EAGAIN;
	}

	if (len > sizeof(g_vendor_tx_buffer)) {
		return -EMSGSIZE;
	}

	if (g_vendor_tx_busy) {
		return -EBUSY;
	}

	memcpy(g_vendor_tx_buffer, data, len);
	g_vendor_tx_busy = true;

	const int rc = usbd_ep_start_write(
		kBusId, kVendorInEp, g_vendor_tx_buffer, static_cast<uint32_t>(len));
	if (rc < 0) {
		g_vendor_tx_busy = false;
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

#if defined(CONFIG_CHERRYUSB) && CONFIG_CHERRYUSB && defined(CONFIG_CHERRYUSB_DEVICE) && \
	CONFIG_CHERRYUSB_DEVICE
	channels::UsbRawFrameMessage chunk = {};
	const int rc = channels::DequeueUsbRawFrame(&chunk, timeout_ms);
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
