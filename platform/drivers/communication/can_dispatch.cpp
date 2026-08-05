/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/can/hpmicro_can_batch.h>

#include <channels/chassismotors_feedback_raw.hpp>
#include <channels/chassismotors_send_raw.hpp>
#include <platform/drivers/communication/can_dispatch.h>
#include <scheduling/thread_priorities.h>

LOG_MODULE_REGISTER(can_dispatch, LOG_LEVEL_INF);

namespace
{

constexpr uint8_t kBusCount = 4U;
constexpr size_t kTxSlotCount = static_cast<size_t>(platform::CanTxSlot::kCount);
constexpr uint16_t kLeftWheelId = 0x201U;
constexpr uint16_t kRightWheelId = 0x201U;
constexpr uint16_t kLeftJointBMasterId = 0x10U;
constexpr uint16_t kLeftJointDMasterId = 0x13U;
constexpr uint16_t kRightJointBMasterId = 0x11U;
constexpr uint16_t kRightJointDMasterId = 0x12U;
constexpr uint32_t kDefaultCanBitrate = 1000000U;
constexpr uint32_t kTxErrorLogPeriod = 100U;
constexpr uint32_t kRxUnmatchedLogPeriod = 1000U;
/* The latest-value mailbox bounds each active bus to three pending actuator
 * slots, so CAN no longer needs to preempt the 1 kHz chassis loop to prevent
 * an unbounded stale-command backlog. Let workers use the remainder of each
 * chassis period instead. */
constexpr size_t kTxQueueDepth = 16U;
constexpr size_t kMaxTxBatchFrames = 3U;

struct BusRxContext {
	uint8_t bus;
};

struct TxBatchCallbackContext {
	uint8_t bus;
	uint8_t count;
	uint8_t slots[kMaxTxBatchFrames];
};

struct QueuedTxFrame {
	uint8_t slot;
};

struct BusRxStats {
	uint32_t total = 0U;
	uint32_t routed = 0U;
	uint32_t unmatched = 0U;
};

const struct device *g_can_dev[kBusCount] = {nullptr, nullptr, nullptr, nullptr};
bool g_started = false;
k_thread g_tx_thread[kBusCount];
K_THREAD_STACK_DEFINE(g_can_tx0_stack, 1024);
K_THREAD_STACK_DEFINE(g_can_tx1_stack, 1024);
K_THREAD_STACK_DEFINE(g_can_tx2_stack, 1024);
K_THREAD_STACK_DEFINE(g_can_tx3_stack, 1024);
K_MSGQ_DEFINE(g_can_tx0_queue, sizeof(QueuedTxFrame), kTxQueueDepth, 4);
K_MSGQ_DEFINE(g_can_tx1_queue, sizeof(QueuedTxFrame), kTxQueueDepth, 4);
K_MSGQ_DEFINE(g_can_tx2_queue, sizeof(QueuedTxFrame), kTxQueueDepth, 4);
K_MSGQ_DEFINE(g_can_tx3_queue, sizeof(QueuedTxFrame), kTxQueueDepth, 4);
struct k_sem g_tx_done_sem[kBusCount];
bool g_tx_thread_started[kBusCount] = {};
uint32_t g_tx_enqueue_error_count = 0U;
atomic_t g_tx_async_error_count[kBusCount] = {};
atomic_t g_tx_submitted_count[kTxSlotCount] = {};
atomic_t g_tx_enqueued_count[kTxSlotCount] = {};
atomic_t g_tx_completed_count[kTxSlotCount] = {};
atomic_t g_rx_received_count[kTxSlotCount] = {};
atomic_t g_tx_coalesced_count[kTxSlotCount] = {};
atomic_t g_tx_slot_pending[kTxSlotCount] = {};
atomic_t g_tx_slot_generation[kTxSlotCount] = {};
atomic_t g_max_tx_queue_depth[kBusCount] = {};
atomic_t g_max_rx_interval_us[kTxSlotCount] = {};
uint64_t g_last_rx_timestamp_us[kTxSlotCount] = {};
TxBatchCallbackContext g_tx_callback_context[kBusCount] = {};
BusRxContext g_rx_context[kBusCount] = {
	{0U},
	{1U},
	{2U},
	{3U},
};
BusRxStats g_rx_stats[kBusCount] = {};

void AtomicUpdateMax(atomic_t *target, uint32_t candidate)
{
	atomic_val_t current = atomic_get(target);
	while (candidate > static_cast<uint32_t>(current) &&
	       !atomic_cas(target, current, static_cast<atomic_val_t>(candidate))) {
		current = atomic_get(target);
	}
}

SeqlockValue<ChassisMotorSendRawFrame> *TxSlotStorage(uint8_t slot)
{
	switch (static_cast<platform::CanTxSlot>(slot)) {
	case platform::CanTxSlot::kLeftWheel:
		return &left_wheel_send_raw;
	case platform::CanTxSlot::kRightWheel:
		return &right_wheel_send_raw;
	case platform::CanTxSlot::kLeftJointB:
		return &left_b_motor_send_raw;
	case platform::CanTxSlot::kLeftJointD:
		return &left_d_motor_send_raw;
	case platform::CanTxSlot::kRightJointB:
		return &right_b_motor_send_raw;
	case platform::CanTxSlot::kRightJointD:
		return &right_d_motor_send_raw;
	default:
		return nullptr;
	}
}

struct k_msgq *TxQueueForBus(uint8_t bus)
{
	if (bus == 0U)
		return &g_can_tx0_queue;
	if (bus == 1U)
		return &g_can_tx1_queue;
	if (bus == 2U)
		return &g_can_tx2_queue;
	if (bus == 3U)
		return &g_can_tx3_queue;
	return nullptr;
}

int EnqueueLatestSlot(uint8_t bus, uint8_t slot)
{
	struct k_msgq *queue = TxQueueForBus(bus);
	if ((queue == nullptr) || (slot >= kTxSlotCount)) {
		return -EINVAL;
	}

	const QueuedTxFrame queued = {
		.slot = slot,
	};
	const int queue_rc = k_msgq_put(queue, &queued, K_NO_WAIT);
	if (queue_rc == 0) {
		AtomicUpdateMax(&g_max_tx_queue_depth[bus],
				static_cast<uint32_t>(k_msgq_num_used_get(queue)));
	}
	return queue_rc;
}

k_thread_stack_t *TxStackForBus(uint8_t bus)
{
	if (bus == 0U) {
		return g_can_tx0_stack;
	}
	if (bus == 1U) {
		return g_can_tx1_stack;
	}
	if (bus == 2U) {
		return g_can_tx2_stack;
	}
	if (bus == 3U) {
		return g_can_tx3_stack;
	}
	return nullptr;
}

size_t TxStackSizeForBus(uint8_t bus)
{
	if (bus == 0U) {
		return K_THREAD_STACK_SIZEOF(g_can_tx0_stack);
	}
	if (bus == 1U) {
		return K_THREAD_STACK_SIZEOF(g_can_tx1_stack);
	}
	if (bus == 2U) {
		return K_THREAD_STACK_SIZEOF(g_can_tx2_stack);
	}
	if (bus == 3U) {
		return K_THREAD_STACK_SIZEOF(g_can_tx3_stack);
	}
	return 0U;
}

const char *TxThreadNameForBus(uint8_t bus)
{
	if (bus == 0U) {
		return "can_tx0";
	}
	if (bus == 1U) {
		return "can_tx1";
	}
	if (bus == 2U) {
		return "can_tx2";
	}
	if (bus == 3U) {
		return "can_tx3";
	}
	return "can_tx";
}

uint32_t ConfiguredBitrateForBus(uint8_t bus)
{
	if (bus == 0U) {
		return DT_PROP_OR(DT_NODELABEL(can0), bitrate, kDefaultCanBitrate);
	}
	if (bus == 1U) {
		return DT_PROP_OR(DT_NODELABEL(can1), bitrate, kDefaultCanBitrate);
	}
	if (bus == 2U) {
		return DT_PROP_OR(DT_NODELABEL(can2), bitrate, kDefaultCanBitrate);
	}
	if (bus == 3U) {
		return DT_PROP_OR(DT_NODELABEL(can3), bitrate, kDefaultCanBitrate);
	}
	return kDefaultCanBitrate;
}

const char *CanStateToString(enum can_state state)
{
	switch (state) {
	case CAN_STATE_ERROR_ACTIVE:
		return "error-active";
	case CAN_STATE_ERROR_WARNING:
		return "error-warning";
	case CAN_STATE_ERROR_PASSIVE:
		return "error-passive";
	case CAN_STATE_BUS_OFF:
		return "bus-off";
	case CAN_STATE_STOPPED:
		return "stopped";
	default:
		return "unknown";
	}
}

void PrintCanState(const struct device *dev, uint8_t bus, const char *tag)
{
	enum can_state state;
	struct can_bus_err_cnt err_cnt;
	const int rc = can_get_state(dev, &state, &err_cnt);
	if (rc == 0) {
		LOG_INF("bus%u %s: %s tec=%u rec=%u", static_cast<unsigned int>(bus), tag,
			CanStateToString(state), static_cast<unsigned int>(err_cnt.tx_err_cnt),
			static_cast<unsigned int>(err_cnt.rx_err_cnt));
	} else {
		LOG_WRN("bus%u %s: can_get_state rc=%d", static_cast<unsigned int>(bus), tag, rc);
	}
}

const struct device *FindCanDeviceForBus(uint8_t bus)
{
	if (bus == 0U) {
#if DT_NODE_HAS_STATUS(DT_NODELABEL(can0), okay)
		return DEVICE_DT_GET(DT_NODELABEL(can0));
#else
		return nullptr;
#endif
	}
	if (bus == 1U) {
#if DT_NODE_HAS_STATUS(DT_NODELABEL(can1), okay)
		return DEVICE_DT_GET(DT_NODELABEL(can1));
#else
		return nullptr;
#endif
	}
	if (bus == 2U) {
#if DT_NODE_HAS_STATUS(DT_NODELABEL(can2), okay)
		return DEVICE_DT_GET(DT_NODELABEL(can2));
#else
		return nullptr;
#endif
	}
	if (bus == 3U) {
#if DT_NODE_HAS_STATUS(DT_NODELABEL(can3), okay)
		return DEVICE_DT_GET(DT_NODELABEL(can3));
#else
		return nullptr;
#endif
	}

	return nullptr;
}

void OnTxDone(const struct device *dev, int error, void *user_data)
{
	ARG_UNUSED(dev);

	const TxBatchCallbackContext *context =
		static_cast<const TxBatchCallbackContext *>(user_data);
	if ((context != nullptr) && (context->bus < kBusCount) &&
	    (context->count <= kMaxTxBatchFrames)) {
		if (error != 0) {
			atomic_inc(&g_tx_async_error_count[context->bus]);
		} else {
			for (uint8_t i = 0U; i < context->count; ++i) {
				if (context->slots[i] < kTxSlotCount) {
					atomic_inc(&g_tx_completed_count[context->slots[i]]);
				}
			}
		}
		k_sem_give(&g_tx_done_sem[context->bus]);
	}
}

int SendStdBatch(uint8_t bus, const struct can_frame *frames, const uint8_t *slots,
		 uint8_t frame_count)
{
	if ((bus >= kBusCount) || (frames == nullptr) || (slots == nullptr) ||
	    (frame_count == 0U) || (frame_count > kMaxTxBatchFrames)) {
		return -EINVAL;
	}

	const struct device *dev = g_can_dev[bus];
	if (dev == nullptr) {
		return -ENODEV;
	}

	TxBatchCallbackContext *context = &g_tx_callback_context[bus];
	context->bus = bus;
	context->count = frame_count;
	for (uint8_t i = 0U; i < frame_count; ++i) {
		context->slots[i] = slots[i];
	}

	const int rc = hpm_can_send_batch(dev, frames, frame_count, K_FOREVER, OnTxDone, context);
	if (rc == 0) {
		return 0;
	}

	const uint32_t count = ++g_tx_enqueue_error_count;
	if ((count <= 10U) || ((count % kTxErrorLogPeriod) == 0U)) {
		LOG_WRN("can batch failed bus%u frames=%u rc=%d count=%u",
			static_cast<unsigned int>(bus), static_cast<unsigned int>(frame_count), rc,
			static_cast<unsigned int>(count));
		PrintCanState(dev, bus, "batch failed");
	}
	return rc;
}

void CanTxLoop(void *bus_arg, void *, void *)
{
	const uint8_t worker_bus = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(bus_arg));
	if (worker_bus >= kBusCount) {
		return;
	}

	struct k_msgq *queue = TxQueueForBus(worker_bus);
	if (queue == nullptr) {
		return;
	}

	for (;;) {
		QueuedTxFrame queued[kMaxTxBatchFrames] = {};
		if (k_msgq_get(queue, &queued[0], K_FOREVER) != 0) {
			continue;
		}

		uint8_t queued_count = 1U;
		while ((queued_count < kMaxTxBatchFrames) &&
		       (k_msgq_get(queue, &queued[queued_count], K_NO_WAIT) == 0)) {
			++queued_count;
		}

		struct can_frame tx_frames[kMaxTxBatchFrames] = {};
		uint8_t sent_slots[kMaxTxBatchFrames] = {};
		atomic_val_t sent_generations[kMaxTxBatchFrames] = {};
		uint8_t frame_count = 0U;

		for (uint8_t i = 0U; i < queued_count; ++i) {
			const uint8_t slot = queued[i].slot;
			if (slot >= kTxSlotCount) {
				continue;
			}

			SeqlockValue<ChassisMotorSendRawFrame> *storage = TxSlotStorage(slot);
			ChassisMotorSendRawFrame source = {};
			const atomic_val_t generation = atomic_get(&g_tx_slot_generation[slot]);
			if ((storage == nullptr) || !storage->read(source) ||
			    (source.bus != worker_bus) || (source.dlc > 8U)) {
				atomic_clear(&g_tx_slot_pending[slot]);
				continue;
			}

			struct can_frame &target = tx_frames[frame_count];
			target.flags = 0U;
			target.id = source.can_id;
			target.dlc = source.dlc;
			for (uint8_t byte = 0U; byte < source.dlc; ++byte) {
				target.data[byte] = source.data[byte];
			}
			sent_slots[frame_count] = slot;
			sent_generations[frame_count] = generation;
			++frame_count;
		}

		if (frame_count == 0U) {
			continue;
		}

		/*
		 * Load every current actuator snapshot into STB first, then assert
		 * TSALL once.  The driver reports one completion for this explicit
		 * batch, eliminating three per-frame scheduling/wait cycles.
		 */
		const int send_rc = SendStdBatch(worker_bus, tx_frames, sent_slots, frame_count);
		if (send_rc == 0) {
			for (uint8_t i = 0U; i < frame_count; ++i) {
				atomic_inc(&g_tx_enqueued_count[sent_slots[i]]);
			}
			(void)k_sem_take(&g_tx_done_sem[worker_bus], K_FOREVER);
		}

		for (uint8_t i = 0U; i < frame_count; ++i) {
			const uint8_t slot = sent_slots[i];
			/*
			 * If the mailbox changed while this batch was in flight, append
			 * that slot once at the tail. A failed batch is also retried
			 * from the newest mailbox value.
			 */
			atomic_clear(&g_tx_slot_pending[slot]);
			if ((send_rc != 0) ||
			    (atomic_get(&g_tx_slot_generation[slot]) != sent_generations[i])) {
				if (atomic_cas(&g_tx_slot_pending[slot], 0, 1)) {
					const int queue_rc = EnqueueLatestSlot(worker_bus, slot);
					if (queue_rc != 0) {
						atomic_clear(&g_tx_slot_pending[slot]);
					}
				}
			}
		}
	}
}

void CanRxCallback(const struct device *dev, struct can_frame *frame, void *user_data)
{
	ARG_UNUSED(dev);

	if ((frame == nullptr) || ((frame->flags & CAN_FRAME_IDE) != 0U)) {
		return;
	}

	const BusRxContext *context = static_cast<const BusRxContext *>(user_data);
	if ((context == nullptr) || (context->bus >= kBusCount)) {
		return;
	}
	const uint8_t bus = context->bus;
	BusRxStats &stats = g_rx_stats[bus];
	++stats.total;

	ChassisMotorFeedbackRawFrame rx_frame = {};
	rx_frame.timestamp_us = k_ticks_to_us_floor64(k_uptime_ticks());
	rx_frame.precise_timestamp_us = k_cyc_to_us_floor64(k_cycle_get_64());
	rx_frame.valid = true;
	for (uint8_t i = 0U; (i < frame->dlc) && (i < sizeof(rx_frame.data)); ++i) {
		rx_frame.data[i] = frame->data[i];
	}

	bool routed = false;
	size_t routed_slot = kTxSlotCount;
	if (bus == 0U) {

	} else if (bus == 1U) {
		switch (static_cast<uint16_t>(frame->id)) {
		case kRightWheelId:
			right_wheel_feedback_raw.write(rx_frame);
			routed = true;
			routed_slot = static_cast<size_t>(platform::CanTxSlot::kRightWheel);
			break;
		default:
			break;
		}
		if (!routed && (frame->id == kRightJointBMasterId)) {
			right_b_motor_feedback_raw.write(rx_frame);
			routed = true;
			routed_slot = static_cast<size_t>(platform::CanTxSlot::kRightJointB);
		} else if (!routed && (frame->id == kRightJointDMasterId)) {
			right_d_motor_feedback_raw.write(rx_frame);
			routed = true;
			routed_slot = static_cast<size_t>(platform::CanTxSlot::kRightJointD);
		}
	} else if (bus == 2U) {

	} else if (bus == 3U) {
		switch (static_cast<uint16_t>(frame->id)) {
		case kLeftWheelId:
			left_wheel_feedback_raw.write(rx_frame);
			routed = true;
			routed_slot = static_cast<size_t>(platform::CanTxSlot::kLeftWheel);
			break;
		default:
			break;
		}
		if (!routed && (frame->id == kLeftJointBMasterId)) {
			left_b_motor_feedback_raw.write(rx_frame);
			routed = true;
			routed_slot = static_cast<size_t>(platform::CanTxSlot::kLeftJointB);
		} else if (!routed && (frame->id == kLeftJointDMasterId)) {
			left_d_motor_feedback_raw.write(rx_frame);
			routed = true;
			routed_slot = static_cast<size_t>(platform::CanTxSlot::kLeftJointD);
		}
	} else {
		return;
	}

	if (routed) {
		if (routed_slot < kTxSlotCount) {
			const uint64_t previous_us = g_last_rx_timestamp_us[routed_slot];
			if (previous_us != 0U && rx_frame.precise_timestamp_us >= previous_us) {
				const uint64_t interval_us =
					rx_frame.precise_timestamp_us - previous_us;
				AtomicUpdateMax(
					&g_max_rx_interval_us[routed_slot],
					static_cast<uint32_t>(MIN(
						interval_us, static_cast<uint64_t>(INT32_MAX))));
			}
			g_last_rx_timestamp_us[routed_slot] = rx_frame.precise_timestamp_us;
			atomic_inc(&g_rx_received_count[routed_slot]);
		}
		++stats.routed;
		return;
	}

	const uint32_t unmatched = ++stats.unmatched;
	if ((unmatched <= 20U) || ((unmatched % kRxUnmatchedLogPeriod) == 0U)) {
		LOG_WRN("rx unmatched bus%u id=0x%x dlc=%u total=%u unmatched=%u",
			static_cast<unsigned int>(bus), static_cast<unsigned int>(frame->id),
			static_cast<unsigned int>(frame->dlc),
			static_cast<unsigned int>(stats.total),
			static_cast<unsigned int>(unmatched));
	}
}

int AddDefaultFilters(const struct device *dev, uint8_t bus)
{
	if ((dev == nullptr) || (bus >= kBusCount)) {
		return -EINVAL;
	}

	const struct can_filter all_standard_frames = {
		.id = 0U,
		.mask = 0U,
		.flags = 0U,
	};

	const int filter_id =
		can_add_rx_filter(dev, CanRxCallback, &g_rx_context[bus], &all_standard_frames);
	if (filter_id < 0) {
		LOG_ERR("bus%u add catch-all rx filter failed: %d", static_cast<unsigned int>(bus),
			filter_id);
		return filter_id;
	}

	LOG_INF("bus%u catch-all rx filter installed id=%d", static_cast<unsigned int>(bus),
		filter_id);
	return 0;
}

int StartTxWorker(uint8_t bus)
{
	if (bus >= kBusCount) {
		return -EINVAL;
	}
	if (g_tx_thread_started[bus]) {
		return 0;
	}

	k_thread_stack_t *stack = TxStackForBus(bus);
	if (stack == nullptr) {
		return -EINVAL;
	}

	k_tid_t tid = k_thread_create(
		&g_tx_thread[bus], stack, TxStackSizeForBus(bus), CanTxLoop,
		reinterpret_cast<void *>(static_cast<uintptr_t>(bus)), nullptr, nullptr,
		K_PRIO_PREEMPT(wbr_control::scheduling::thread_priority::kCanTx), 0, K_NO_WAIT);
	k_thread_name_set(tid, TxThreadNameForBus(bus));
	g_tx_thread_started[bus] = true;
	return 0;
}

} // namespace

namespace platform
{

int InitializeCanDispatch()
{
	if (g_started) {
		return 0;
	}

	bool has_any_bus = false;
	for (uint8_t bus = 0U; bus < kBusCount; ++bus) {
		k_sem_init(&g_tx_done_sem[bus], 0, 1);
		g_tx_callback_context[bus] = {
			.bus = bus,
			.count = 0U,
			.slots = {},
		};
	}
	for (uint8_t bus = 0U; bus < kBusCount; ++bus) {
		g_can_dev[bus] = FindCanDeviceForBus(bus);
		if ((g_can_dev[bus] == nullptr) || !device_is_ready(g_can_dev[bus])) {
			LOG_WRN("bus%u device not ready", static_cast<unsigned int>(bus));
			continue;
		}

		has_any_bus = true;

		const int mode_rc = can_set_mode(g_can_dev[bus], CAN_MODE_NORMAL);
		if (mode_rc != 0) {
			LOG_ERR("bus%u can_set_mode(normal) failed: %d",
				static_cast<unsigned int>(bus), mode_rc);
			return mode_rc;
		}

		const uint32_t bitrate = ConfiguredBitrateForBus(bus);
		const int bitrate_rc = can_set_bitrate(g_can_dev[bus], bitrate);
		if (bitrate_rc != 0) {
			LOG_ERR("bus%u can_set_bitrate(%u) failed: %d",
				static_cast<unsigned int>(bus), static_cast<unsigned int>(bitrate),
				bitrate_rc);
			return bitrate_rc;
		}

		const int rc = can_start(g_can_dev[bus]);
		if ((rc != 0) && (rc != -EALREADY)) {
			LOG_ERR("bus%u can_start failed: %d", static_cast<unsigned int>(bus), rc);
			return rc;
		}

		LOG_INF("bus%u started dev=%s bitrate=%u", static_cast<unsigned int>(bus),
			g_can_dev[bus]->name, static_cast<unsigned int>(bitrate));
		PrintCanState(g_can_dev[bus], bus, "after start");

		const int filter_rc = AddDefaultFilters(g_can_dev[bus], bus);
		if (filter_rc != 0) {
			return filter_rc;
		}

		const int tx_worker_rc = StartTxWorker(bus);
		if (tx_worker_rc != 0) {
			LOG_ERR("bus%u tx worker start failed: %d", static_cast<unsigned int>(bus),
				tx_worker_rc);
			return tx_worker_rc;
		}
	}

	if (!has_any_bus) {
		return -ENODEV;
	}

	g_started = true;
	return 0;
}

void NotifyCanTxPending()
{
	// Kept for source compatibility. Queue insertion wakes the worker directly.
}

int SubmitCanStandardFrame(CanTxSlot slot, uint8_t bus, uint16_t can_id, const uint8_t *data,
			   uint8_t dlc)
{
	const uint8_t slot_index = static_cast<uint8_t>(slot);
	if ((bus >= kBusCount) || (slot_index >= kTxSlotCount) || (data == nullptr) || (dlc > 8U)) {
		return -EINVAL;
	}

	ChassisMotorSendRawFrame frame = {};
	frame.bus = bus;
	frame.can_id = can_id;
	frame.dlc = dlc;
	for (uint8_t i = 0U; i < dlc; ++i) {
		frame.data[i] = data[i];
	}

	SeqlockValue<ChassisMotorSendRawFrame> *storage = TxSlotStorage(slot_index);
	if (storage == nullptr) {
		return -EINVAL;
	}

	/* Publish the newest value before advertising the slot as pending. */
	storage->write(frame);
	atomic_inc(&g_tx_submitted_count[slot_index]);
	atomic_inc(&g_tx_slot_generation[slot_index]);

	/* A pending or in-flight slot already owns one queue position. Updating
	 * its mailbox coalesces the old command instead of replaying stale data. */
	if (!atomic_cas(&g_tx_slot_pending[slot_index], 0, 1)) {
		atomic_inc(&g_tx_coalesced_count[slot_index]);
		return 0;
	}

	const int queue_rc = EnqueueLatestSlot(bus, slot_index);
	if (queue_rc != 0) {
		atomic_clear(&g_tx_slot_pending[slot_index]);
		atomic_inc(&g_tx_coalesced_count[slot_index]);
		return queue_rc;
	}
	return 0;
}

int ReadCanBusHealth(uint8_t bus, CanBusHealth *health)
{
	if ((bus >= kBusCount) || (health == nullptr)) {
		return -EINVAL;
	}
	*health = {};
	const struct device *dev = g_can_dev[bus];
	if (dev == nullptr) {
		return -ENODEV;
	}
	enum can_state state;
	struct can_bus_err_cnt error_count = {};
	const int rc = can_get_state(dev, &state, &error_count);
	if (rc != 0) {
		return rc;
	}
	health->valid = true;
	health->state = static_cast<uint8_t>(state);
	health->tx_error_count = error_count.tx_err_cnt;
	health->rx_error_count = error_count.rx_err_cnt;
	health->async_tx_error_count =
		static_cast<uint32_t>(atomic_get(&g_tx_async_error_count[bus]));
	return 0;
}

uint32_t ReadCanAsyncTxErrorCount(uint8_t bus)
{
	if (bus >= kBusCount) {
		return 0U;
	}
	return static_cast<uint32_t>(atomic_get(&g_tx_async_error_count[bus]));
}

int ReadCanTxSlotStats(CanTxSlot slot, CanTxSlotStats *stats)
{
	const uint8_t slot_index = static_cast<uint8_t>(slot);
	if ((slot_index >= kTxSlotCount) || (stats == nullptr)) {
		return -EINVAL;
	}
	stats->submitted_count =
		static_cast<uint32_t>(atomic_get(&g_tx_submitted_count[slot_index]));
	stats->enqueued_count = static_cast<uint32_t>(atomic_get(&g_tx_enqueued_count[slot_index]));
	stats->completed_count =
		static_cast<uint32_t>(atomic_get(&g_tx_completed_count[slot_index]));
	stats->received_count = static_cast<uint32_t>(atomic_get(&g_rx_received_count[slot_index]));
	stats->coalesced_count =
		static_cast<uint32_t>(atomic_get(&g_tx_coalesced_count[slot_index]));
	stats->max_rx_interval_us =
		static_cast<uint32_t>(atomic_get(&g_max_rx_interval_us[slot_index]));
	return 0;
}

int ReadCanTxQueueStats(uint8_t bus, CanTxQueueStats *stats)
{
	if ((bus >= kBusCount) || (stats == nullptr)) {
		return -EINVAL;
	}
	struct k_msgq *queue = TxQueueForBus(bus);
	if (queue == nullptr) {
		return -EINVAL;
	}
	stats->current_depth = static_cast<uint32_t>(k_msgq_num_used_get(queue));
	stats->max_depth = static_cast<uint32_t>(atomic_get(&g_max_tx_queue_depth[bus]));
	/* Each active chassis bus owns three actuator mailboxes. */
	stats->capacity = 3U;
	return 0;
}

void ResetCanRxIntervalStats()
{
	const unsigned int key = irq_lock();
	for (size_t slot = 0U; slot < kTxSlotCount; ++slot) {
		atomic_clear(&g_max_rx_interval_us[slot]);
		g_last_rx_timestamp_us[slot] = 0U;
	}
	irq_unlock(key);
}

void ResetCanTxDiagnosticStats()
{
	for (size_t slot = 0U; slot < kTxSlotCount; ++slot) {
		atomic_clear(&g_tx_coalesced_count[slot]);
	}
	for (size_t bus = 0U; bus < kBusCount; ++bus) {
		atomic_clear(&g_max_tx_queue_depth[bus]);
	}
}

} // namespace platform
