/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <channels/usb_raw_frame_queue.h>
#include <platform/drivers/communication/usb_session.h>

#include "usbd_core.h"
#include "usbd_cdc_acm.h"

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
