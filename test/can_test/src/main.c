/**
 * @file test/can_test/src/main.c
 * @brief 实现应用或测试程序的入口与初始化流程。
 * @details 该文件属于独立 Zephyr 测试镜像，只验证指定外设或算法路径，不会链接进主固件。测试会直接访问目标硬件并通过串口输出判定结果。
 */

#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define TEST_CAN_ID 0x123U

// #if DT_NODE_HAS_STATUS(DT_ALIAS(canbus), okay)
// #define TEST_CAN_NODE DT_ALIAS(canbus)
// #elif DT_NODE_HAS_STATUS(DT_NODELABEL(can1), okay)
// #define TEST_CAN_NODE DT_NODELABEL(can1)
// #if DT_NODE_HAS_STATUS(DT_NODELABEL(can0), okay)
// #define TEST_CAN_NODE DT_NODELABEL(can0)
// #elif DT_NODE_HAS_STATUS(DT_NODELABEL(can2), okay)
// #define TEST_CAN_NODE DT_NODELABEL(can2)
#if DT_NODE_HAS_STATUS(DT_NODELABEL(can0), okay)
#define TEST_CAN_NODE DT_NODELABEL(can0)
#else
#define TEST_CAN_NODE DT_INVALID_NODE
#endif

#define TEST_CAN_BITRATE DT_PROP_OR(TEST_CAN_NODE, bitrate, 1000000)
#define TEST_CAN_PAYLOAD_BYTES 8U

#define TEST_CAN_XCVR_ENABLE_NODE DT_ALIAS(can_transceiver_enable)

K_SEM_DEFINE(g_tx_done_sem, 0, 1);
static volatile int g_tx_result;

static const char *CanStateToStr(enum can_state state)
{
	if (state == CAN_STATE_ERROR_ACTIVE) {
		return "error-active";
	}
	if (state == CAN_STATE_ERROR_WARNING) {
		return "error-warning";
	}
	if (state == CAN_STATE_ERROR_PASSIVE) {
		return "error-passive";
	}
	if (state == CAN_STATE_BUS_OFF) {
		return "bus-off";
	}
	if (state == CAN_STATE_STOPPED) {
		return "stopped";
	}

	return "unknown";
}

static void PrintCanState(const struct device *can_dev, const char *tag)
{
	enum can_state state;
	struct can_bus_err_cnt err_cnt;
	int rc = can_get_state(can_dev, &state, &err_cnt);

	if (rc == 0) {
		printk("[STATE] %s: %s tec=%u rec=%u\n", tag, CanStateToStr(state),
		       err_cnt.tx_err_cnt, err_cnt.rx_err_cnt);
	} else {
		printk("[STATE] %s: can_get_state rc=%d\n", tag, rc);
	}
}

static void OnTxDone(const struct device *dev, int error, void *user_data)
{
	(void)dev;
	(void)user_data;
	g_tx_result = error;
	k_sem_give(&g_tx_done_sem);
}

static void OnRx(const struct device *dev, struct can_frame *frame, void *user_data)
{
	(void)dev;
	(void)user_data;

	printk("[RX] id=0x%x dlc=%u data=", frame->id, frame->dlc);
	for (uint8_t i = 0; i < can_dlc_to_bytes(frame->dlc); i++) {
		printk("%02x ", frame->data[i]);
	}
	printk("\n");
}

const struct device *FindCanDevice(void)
{
#if DT_NODE_HAS_STATUS(TEST_CAN_NODE, okay)
	const struct device *dev = DEVICE_DT_GET(TEST_CAN_NODE);
	if (device_is_ready(dev)) {
		return dev;
	}
#endif

	return NULL;
}

static int EnableExternalTransceiver(void)
{
#if DT_NODE_HAS_STATUS(TEST_CAN_XCVR_ENABLE_NODE, okay)
	const struct gpio_dt_spec enable_gpio = GPIO_DT_SPEC_GET(TEST_CAN_XCVR_ENABLE_NODE, gpios);
	int rc;

	if (!gpio_is_ready_dt(&enable_gpio)) {
		printk("[FAIL] CAN transceiver enable GPIO is not ready\n");
		return -ENODEV;
	}

	rc = gpio_pin_configure_dt(&enable_gpio, GPIO_OUTPUT_ACTIVE);
	if (rc != 0) {
		printk("[FAIL] configure CAN transceiver enable GPIO rc=%d\n", rc);
		return rc;
	}

	printk("[INFO] CAN transceiver enable GPIO asserted\n");
#else
	printk("[INFO] no can-transceiver-enable alias; assuming transceiver is hardware-enabled\n");
#endif

	return 0;
}

static void PrintCanConfig(const struct device *can_dev)
{
	uint32_t core_clock = 0U;
	int rc = can_get_core_clock(can_dev, &core_clock);

	if (rc == 0) {
		printk("[INFO] CAN core clock: %u Hz\n", core_clock);
	} else {
		printk("[WARN] can_get_core_clock rc=%d\n", rc);
	}

	printk("[INFO] requested bitrate: %u bit/s\n", TEST_CAN_BITRATE);
}

int main(void)
{
	printk("can_test booted\n");

	const struct device *can_dev = FindCanDevice();
	if (can_dev == NULL) {
		printk("[FAIL] CAN device not found\n");
		return -ENODEV;
	}

	printk("using can device: %s\n", can_dev->name);
	PrintCanConfig(can_dev);

	int rc = can_set_mode(can_dev, CAN_MODE_NORMAL);
	if (rc != 0) {
		printk("[FAIL] can_set_mode(normal) rc=%d\n", rc);
		return rc;
	}

	rc = can_set_bitrate(can_dev, TEST_CAN_BITRATE);
	if (rc != 0) {
		printk("[FAIL] can_set_bitrate(%u) rc=%d\n", TEST_CAN_BITRATE, rc);
		return rc;
	}

	rc = EnableExternalTransceiver();
	if (rc != 0) {
		return rc;
	}

	// int rc = can_set_mode(can_dev, CAN_MODE_LOOPBACK);
	// if (rc != 0) {
	// 	printk("[FAIL] can_set_mode(loopback) rc=%d\n", rc);
	// 	return rc;
	// }

	rc = can_start(can_dev);
	if ((rc != 0) && (rc != -EALREADY)) {
		printk("[FAIL] can_start rc=%d\n", rc);
		return rc;
	}

	PrintCanState(can_dev, "after start");

	/* Add RX filter for our test ID so we can see loopback/echo replies. */
	struct can_filter rx_filter = {
		.id = TEST_CAN_ID,
		.mask = CAN_STD_ID_MASK,
		.flags = 0U,
	};
	int filter_id = can_add_rx_filter(can_dev, OnRx, NULL, &rx_filter);
	if (filter_id < 0) {
		printk("[WARN] can_add_rx_filter failed: %d\n", filter_id);
	} else {
		printk("[INFO] RX filter installed for id=0x%x (filter_id=%d)\n", TEST_CAN_ID, filter_id);
	}

	printk("[INFO] physical CAN TX/RX test started (id=0x%x)\n", TEST_CAN_ID);
	printk("[INFO] connect CAN transceiver and a peer/analyzer at same baud\n");

	uint32_t seq = 0U;
	bool tx_blocked = false;
	while (true) {
		if (tx_blocked) {
			PrintCanState(can_dev, "waiting (tx blocked)");
			k_sleep(K_MSEC(1000));
			continue;
		}

		struct can_frame tx = {
			.flags = 0U,
			.id = TEST_CAN_ID,
			.dlc = can_bytes_to_dlc(TEST_CAN_PAYLOAD_BYTES),
			.data = {
				(uint8_t)(seq & 0xFFU),
				(uint8_t)((seq >> 8) & 0xFFU),
				0x32U,
				0x43U,
				0x54U,
				0x65U,
				0x76U,
				0x87U,
			},
		};

		while (k_sem_take(&g_tx_done_sem, K_NO_WAIT) == 0) {
			/* Drain stale completion events before next frame. */
		}

		g_tx_result = -EINPROGRESS;
		rc = can_send(can_dev, &tx, K_MSEC(50), OnTxDone, NULL);
		if (rc != 0) {
			printk("[ERR] can_send enqueue rc=%d (mailbox/state/wiring)\n", rc);
			PrintCanState(can_dev, "enqueue failed");
		} else {
			rc = k_sem_take(&g_tx_done_sem, K_MSEC(200));
			if (rc == 0) {
				if (g_tx_result == 0) {
					printk("[TX] seq=%u id=0x%x dlc=%u\n", seq, tx.id, tx.dlc);
				} else {
					printk("[ERR] tx callback error=%d (likely ACK/bus issue)\n", g_tx_result);
					PrintCanState(can_dev, "tx callback error");
				}
			} else {
				printk("[WARN] tx completion timeout (likely no ACK / transceiver missing)\n");
				PrintCanState(can_dev, "tx timeout");
				rc = can_stop(can_dev);
				if ((rc != 0) && (rc != -EALREADY)) {
					printk("[WARN] can_stop after tx timeout rc=%d\n", rc);
				} else {
					printk("[WARN] CAN stopped after tx timeout to halt hardware retransmit\n");
				}
				printk("[WARN] stop re-sending, keep monitoring state only\n");
				tx_blocked = true;
			}
		}

		seq++;
		k_sleep(K_MSEC(500));
	}
}
