/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file test/wheel_current_mapping_test/src/main.cpp
 * @brief 实现应用或测试程序的入口与初始化流程。
 * @details 该文件属于独立 Zephyr 测试镜像，只验证指定外设或算法路径，不会链接进主固件。测试会直接访问目标硬件并通过串口输出判定结果。
 */

#include <errno.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>

#include <channels/oscilloscope_sample.hpp>
#include <channels/remote_input_state.hpp>
#include <protocols/motors/dji_motor_protocol.h>
#include <modules/oscilloscope/oscilloscope_module.h>
#include <modules/remote_input/remote_input_module.h>

namespace
{

constexpr uint8_t kLeftBus = 0U;
constexpr uint8_t kRightBus = 1U;
constexpr uint16_t kWheelCanId = 0x201U;
constexpr uint16_t kCurrentCommandCanId = 0x200U;
constexpr uint32_t kControlPeriodMs = 1U;
constexpr uint32_t kRemoteTimeoutMs = 100U;
constexpr uint32_t kFeedbackTimeoutMs = 100U;
constexpr uint32_t kInitialSettleMs = 1000U;
constexpr uint32_t kPulseMs = 600U;
constexpr uint32_t kZeroMs = 800U;
constexpr int16_t kMaximumCommand = 2000;
constexpr double kRpmToRadPerSec = 0.10471975511965977;
constexpr double kAccelerationFilterTimeConstantSec = 0.020;
constexpr double kReductionRatio = 268.0 / 17.0;
constexpr int16_t kSafeReverseMotorRpm = 100;

struct TestStep {
	int16_t command;
	uint32_t duration_ms;
};

constexpr TestStep kSteps[] = {
	{0, kInitialSettleMs}, {200, kPulseMs},  {0, kZeroMs}, {-200, kPulseMs},
	{0, kZeroMs},          {400, kPulseMs},  {0, kZeroMs}, {-400, kPulseMs},
	{0, kZeroMs},          {800, kPulseMs},  {0, kZeroMs}, {-800, kPulseMs},
	{0, kZeroMs},          {1200, kPulseMs}, {0, kZeroMs}, {-1200, kPulseMs},
	{0, kZeroMs},          {1600, kPulseMs}, {0, kZeroMs}, {-1600, kPulseMs},
	{0, kZeroMs},          {2000, kPulseMs}, {0, kZeroMs}, {-2000, kPulseMs},
	{0, kZeroMs},
};

static_assert(kMaximumCommand <= 2048,
	      "test command must remain below the provisional 2.5 A limit");

struct FeedbackSlot {
	protocols::DjiMotorFeedback value = {};
	uint32_t sequence = 0U;
	uint32_t timestamp_ms = 0U;
};

struct FeedbackSnapshot {
	protocols::DjiMotorFeedback value = {};
	uint32_t sequence = 0U;
	uint32_t timestamp_ms = 0U;
};

struct BusContext {
	uint8_t bus;
};

const struct device *g_can_devices[2] = {};
BusContext g_bus_contexts[2] = {{kLeftBus}, {kRightBus}};
FeedbackSlot g_feedback[2];
struct k_spinlock g_feedback_lock;

uint32_t g_oscilloscope_sequence = 0U;
uint32_t g_last_remote_sequence = 0U;
uint32_t g_last_remote_update_ms = 0U;
bool g_previous_enable = false;
bool g_run_active = false;
bool g_run_complete = false;
uint8_t g_active_bus = kLeftBus;
size_t g_step_index = 0U;
uint32_t g_step_started_ms = 0U;
int16_t g_command = 0;
double g_previous_speed_rad_s = 0.0;
double g_filtered_acceleration_rad_s2 = 0.0;
uint32_t g_previous_speed_timestamp_ms = 0U;

const struct device *CanDeviceForBus(uint8_t bus)
{
	if (bus == kLeftBus) {
		return DEVICE_DT_GET(DT_NODELABEL(can0));
	}
	return DEVICE_DT_GET(DT_NODELABEL(can1));
}

void OnCanRx(const struct device *dev, struct can_frame *frame, void *user_data)
{
	ARG_UNUSED(dev);
	if ((frame == nullptr) || (frame->id != kWheelCanId) ||
	    ((frame->flags & CAN_FRAME_IDE) != 0U)) {
		return;
	}

	const auto *context = static_cast<const BusContext *>(user_data);
	if ((context == nullptr) || (context->bus > kRightBus)) {
		return;
	}

	protocols::DjiMotorFeedback decoded = {};
	if (protocols::DecodeDjiFeedback(frame->data, frame->dlc, &decoded) != 0) {
		return;
	}

	const k_spinlock_key_t key = k_spin_lock(&g_feedback_lock);
	g_feedback[context->bus].value = decoded;
	++g_feedback[context->bus].sequence;
	g_feedback[context->bus].timestamp_ms = k_uptime_get_32();
	k_spin_unlock(&g_feedback_lock, key);
}

FeedbackSnapshot ReadFeedback(uint8_t bus)
{
	FeedbackSnapshot snapshot = {};
	if (bus > kRightBus) {
		return snapshot;
	}
	const k_spinlock_key_t key = k_spin_lock(&g_feedback_lock);
	snapshot.value = g_feedback[bus].value;
	snapshot.sequence = g_feedback[bus].sequence;
	snapshot.timestamp_ms = g_feedback[bus].timestamp_ms;
	k_spin_unlock(&g_feedback_lock, key);
	return snapshot;
}

int SendCurrent(uint8_t bus, int16_t command)
{
	if ((bus > kRightBus) || (g_can_devices[bus] == nullptr)) {
		return -ENODEV;
	}

	uint8_t payload[8] = {};
	const int rc = protocols::WriteDjiCurrentCommandToSlot(kWheelCanId, command, payload);
	if (rc != 0) {
		return rc;
	}

	struct can_frame frame = {};
	frame.flags = 0U;
	frame.id = kCurrentCommandCanId;
	frame.dlc = can_bytes_to_dlc(8U);
	for (size_t i = 0U; i < sizeof(payload); ++i) {
		frame.data[i] = payload[i];
	}
	return can_send(g_can_devices[bus], &frame, K_MSEC(1), nullptr, nullptr);
}

void SendAllZero()
{
	(void)SendCurrent(kLeftBus, 0);
	(void)SendCurrent(kRightBus, 0);
	g_command = 0;
}

int ConfigureCan()
{
	for (uint8_t bus = kLeftBus; bus <= kRightBus; ++bus) {
		g_can_devices[bus] = CanDeviceForBus(bus);
		if ((g_can_devices[bus] == nullptr) || !device_is_ready(g_can_devices[bus])) {
			return -ENODEV;
		}

		struct can_filter filter = {};
		filter.flags = 0U;
		filter.id = kWheelCanId;
		filter.mask = CAN_STD_ID_MASK;
		if (can_add_rx_filter(g_can_devices[bus], OnCanRx, &g_bus_contexts[bus], &filter) <
		    0) {
			return -EIO;
		}

		const int rc = can_start(g_can_devices[bus]);
		if ((rc != 0) && (rc != -EALREADY)) {
			return rc;
		}
	}
	return 0;
}

bool ReadFreshRemote(uint32_t now_ms, channels::RemoteInputState &state)
{
	if (!latest_remote_state.read(state)) {
		return false;
	}
	if ((state.sequence != 0U) && (state.sequence != g_last_remote_sequence)) {
		g_last_remote_sequence = state.sequence;
		g_last_remote_update_ms = now_ms;
	}
	return (state.sequence != 0U) && ((now_ms - g_last_remote_update_ms) <= kRemoteTimeoutMs);
}

void ResetAccelerationEstimator()
{
	g_previous_speed_rad_s = 0.0;
	g_filtered_acceleration_rad_s2 = 0.0;
	g_previous_speed_timestamp_ms = 0U;
}

void StartRun(uint32_t now_ms)
{
	g_run_active = true;
	g_run_complete = false;
	g_active_bus = kLeftBus;
	g_step_index = 0U;
	g_step_started_ms = now_ms;
	g_command = 0;
	ResetAccelerationEstimator();
	SendAllZero();
}

void StopRun()
{
	g_run_active = false;
	g_run_complete = false;
	g_step_index = 0U;
	SendAllZero();
	ResetAccelerationEstimator();
}

void AdvanceStep(uint32_t now_ms)
{
	++g_step_index;
	if (g_step_index < std::size(kSteps)) {
		g_step_started_ms = now_ms;
		return;
	}

	if (g_active_bus == kLeftBus) {
		g_active_bus = kRightBus;
		g_step_index = 0U;
		g_step_started_ms = now_ms;
		ResetAccelerationEstimator();
		return;
	}

	g_run_active = false;
	g_run_complete = true;
	g_command = 0;
	SendAllZero();
}

bool StepMayAdvance(const FeedbackSnapshot &feedback, uint32_t now_ms)
{
	if ((now_ms - g_step_started_ms) < kSteps[g_step_index].duration_ms) {
		return false;
	}

	const bool next_step_is_nonzero =
		(g_step_index + 1U < std::size(kSteps)) && (kSteps[g_step_index + 1U].command != 0);
	if ((kSteps[g_step_index].command == 0) && next_step_is_nonzero &&
	    (std::abs(static_cast<int>(feedback.value.omega)) > kSafeReverseMotorRpm)) {
		return false;
	}
	return true;
}

float StageValue()
{
	if (g_run_complete) {
		return 999.0F;
	}
	if (!g_run_active) {
		return 0.0F;
	}
	const uint32_t base = (g_active_bus == kLeftBus) ? 1U : 101U;
	return static_cast<float>(base + static_cast<uint32_t>(g_step_index));
}

void PublishOscilloscope(const FeedbackSnapshot &feedback, double acceleration_rad_s2)
{
	channels::OscilloscopeSample sample = {};
	sample.sequence = ++g_oscilloscope_sequence;
	sample.uptime_ms = k_uptime_get_32();
	sample.channel_count = 6U;
	sample.value[0] = static_cast<float>(g_command);
	sample.value[1] = static_cast<float>(feedback.value.current);
	sample.value[2] = static_cast<float>(feedback.value.omega);
	sample.value[3] =
		static_cast<float>(static_cast<double>(feedback.value.omega) / kReductionRatio);
	sample.value[4] = static_cast<float>(acceleration_rad_s2);
	sample.value[5] = StageValue();
	channels::latest_oscilloscope_sample.write(sample);
}

double UpdateAcceleration(const FeedbackSnapshot &feedback)
{
	if ((feedback.sequence == 0U) || (feedback.timestamp_ms == g_previous_speed_timestamp_ms)) {
		return g_filtered_acceleration_rad_s2;
	}

	const double output_speed_rad_s =
		static_cast<double>(feedback.value.omega) * kRpmToRadPerSec / kReductionRatio;
	if (g_previous_speed_timestamp_ms != 0U) {
		const uint32_t elapsed_ms = feedback.timestamp_ms - g_previous_speed_timestamp_ms;
		if ((elapsed_ms > 0U) && (elapsed_ms <= kFeedbackTimeoutMs)) {
			const double dt = static_cast<double>(elapsed_ms) / 1000.0;
			const double raw_acceleration =
				(output_speed_rad_s - g_previous_speed_rad_s) / dt;
			const double alpha =
				1.0 - std::exp(-dt / kAccelerationFilterTimeConstantSec);
			g_filtered_acceleration_rad_s2 +=
				alpha * (raw_acceleration - g_filtered_acceleration_rad_s2);
		}
	}
	g_previous_speed_rad_s = output_speed_rad_s;
	g_previous_speed_timestamp_ms = feedback.timestamp_ms;
	return g_filtered_acceleration_rad_s2;
}

void RunControlIteration()
{
	const uint32_t now_ms = k_uptime_get_32();
	channels::RemoteInputState remote = {};
	const bool remote_fresh = ReadFreshRemote(now_ms, remote);
	const bool enabled = remote_fresh && remote.robot_enable;

	if (!enabled) {
		if (g_previous_enable || g_run_active || (g_command != 0)) {
			StopRun();
		} else {
			SendAllZero();
		}
		g_previous_enable = false;
		const FeedbackSnapshot feedback = ReadFeedback(g_active_bus);
		PublishOscilloscope(feedback, 0.0);
		return;
	}

	if (!g_previous_enable) {
		StartRun(now_ms);
	}
	g_previous_enable = true;

	if (!g_run_active) {
		SendAllZero();
		const FeedbackSnapshot feedback = ReadFeedback(g_active_bus);
		PublishOscilloscope(feedback, 0.0);
		return;
	}

	FeedbackSnapshot feedback = ReadFeedback(g_active_bus);
	const bool feedback_fresh = (feedback.sequence != 0U) &&
				    ((now_ms - feedback.timestamp_ms) <= kFeedbackTimeoutMs);
	if (!feedback_fresh) {
		StopRun();
		PublishOscilloscope(feedback, 0.0);
		return;
	}

	while (g_run_active && StepMayAdvance(feedback, now_ms)) {
		AdvanceStep(now_ms);
	}

	if (!g_run_active) {
		PublishOscilloscope(feedback, 0.0);
		return;
	}

	g_command = std::clamp(kSteps[g_step_index].command, static_cast<int16_t>(-kMaximumCommand),
			       kMaximumCommand);
	const int active_send_rc = SendCurrent(g_active_bus, g_command);
	const int idle_send_rc = SendCurrent(g_active_bus == kLeftBus ? kRightBus : kLeftBus, 0);
	if ((active_send_rc != 0) || (idle_send_rc != 0)) {
		StopRun();
		PublishOscilloscope(feedback, 0.0);
		return;
	}

	feedback = ReadFeedback(g_active_bus);
	const double acceleration = UpdateAcceleration(feedback);
	PublishOscilloscope(feedback, acceleration);
}

} // namespace

int main()
{
	int rc = ConfigureCan();
	if (rc != 0) {
		SendAllZero();
		return rc;
	}

	static modules::RemoteInputModule remote_input_module;
	rc = remote_input_module.Start();
	if (rc != 0) {
		SendAllZero();
		return rc;
	}

	static modules::OscilloscopeModule oscilloscope_module;
	rc = oscilloscope_module.Start();
	if (rc != 0) {
		SendAllZero();
		return rc;
	}

	SendAllZero();
	for (;;) {
		RunControlIteration();
		k_sleep(K_MSEC(kControlPeriodMs));
	}
}
