/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#include <errno.h>

#include <algorithm>
#include <cmath>
#include <channels/chassismotors_feedback_raw.hpp>
#include <channels/chassismotors_send_raw.hpp>
#include <channels/hi91_imu_sample.hpp>
#include <channels/oscilloscope_sample.hpp>
#include <channels/remote_input_state.hpp>
#include <modules/thread_utils.h>
#include <modules/chassis/chassis_module.h>
#include <platform/drivers/communication/can_dispatch.h>
#include <protocols/motors/dji_motor_protocol.h>
#include <protocols/motors/dm_motor_protocol.h>

LOG_MODULE_REGISTER(chassis_module, LOG_LEVEL_INF);

namespace {

K_THREAD_STACK_DEFINE(g_chassis_module_stack, 4096);
constexpr uint8_t kLeftLegBus = 0U;
constexpr uint8_t kRightLegBus = 1U;
constexpr uint16_t kLeftJointBCanId = 0x00U;
constexpr uint16_t kLeftJointBMasterId = 0x10U;
constexpr uint16_t kLeftJointDCanId = 0x03U;
constexpr uint16_t kLeftJointDMasterId = 0x13U;
constexpr uint16_t kRightJointBCanId = 0x01U;
constexpr uint16_t kRightJointBMasterId = 0x11U;
constexpr uint16_t kRightJointDCanId = 0x02U;
constexpr uint16_t kRightJointDMasterId = 0x12U;
constexpr uint16_t kLeftWheelCanId = 0x201U;
constexpr uint16_t kRightWheelCanId = 0x201U;
constexpr uint16_t kDjiCurrentCommandCanId = 0x200U;

constexpr double kDefaultTargetLegLength = 0.3;
constexpr double kDefaultTargetLegAngle = 0.0;
constexpr double kMinTargetLegLength = 0.10;
constexpr double kMaxTargetLegLength = 0.40;
constexpr double kRemoteLegLengthRate = 0.06;
constexpr double kRemoteMaxLinearVelocity = 0.8;
constexpr double kRemoteMaxYawRate = 3.0;
constexpr double kDjiCurrentCommandPerNm = 1200.0;
constexpr int16_t kDjiWheelCurrentLimit = 5000;
constexpr bool kDisableWheelCurrentForLegDebug = true;
constexpr bool kWheelOpenLoopBypassEnabled = true;
constexpr double kWheelOpenLoopDeadband = 0.05;
constexpr int16_t kWheelOpenLoopCurrentLimit = 1200;
constexpr double kDmJointTorqueSlewRate = 125.0;
constexpr double kDefaultControlDt = 0.001;
constexpr uint32_t kLegFeedbackTracePeriod = 1000U;
constexpr uint32_t kLegTorqueTracePeriod = 1000U;
constexpr int kLeftLegKinematicBranch = 1;
constexpr int kRightLegKinematicBranch = -1;
constexpr double kLegAngleWrap = 6.28318530717958647692;
constexpr double kLeftLegAngleOffset = -0.036063;
constexpr double kRightLegAngleOffset = -3.121010;
constexpr uint8_t kLeftJointBTorqueIndex = 0U;
constexpr uint8_t kLeftJointDTorqueIndex = 1U;
constexpr uint8_t kRightJointBTorqueIndex = 2U;
constexpr uint8_t kRightJointDTorqueIndex = 3U;

constexpr protocols::motors::dm::DmMitRange kDmJointMitRange = {
	.p_min = -12.56637f,
	.p_max = 12.56637f,
	.v_min = -45.0f,
	.v_max = 45.0f,
	.kp_min = 0.0f,
	.kp_max = 500.0f,
	.kd_min = 0.0f,
	.kd_max = 5.0f,
	.t_min = -54.0f,
	.t_max = 54.0f,
};

float UIntToFloat(uint16_t value, float min_value, float max_value, uint8_t bits)
{
	const float span = max_value - min_value;
	const float max_int = static_cast<float>((1U << bits) - 1U);
	return static_cast<float>(value) * span / max_int + min_value;
}

double DmPositionRad(const protocols::motors::dm::DmMotorFeedbackNormal &feedback)
{
	return UIntToFloat(feedback.angle, kDmJointMitRange.p_min, kDmJointMitRange.p_max, 16);
}

double DmVelocityRadPerSec(const protocols::motors::dm::DmMotorFeedbackNormal &feedback)
{
	return UIntToFloat(feedback.omega, kDmJointMitRange.v_min, kDmJointMitRange.v_max, 12);
}

int16_t WheelTorqueToDjiCurrent(double torque)
{
	const double current = std::clamp(torque * kDjiCurrentCommandPerNm,
					 -static_cast<double>(kDjiWheelCurrentLimit),
					 static_cast<double>(kDjiWheelCurrentLimit));
	return static_cast<int16_t>(current);
}

double ApplyAxisDeadband(double axis)
{
	if (std::abs(axis) < kWheelOpenLoopDeadband) {
		return 0.0;
	}
	return std::clamp(axis, -1.0, 1.0);
}

int16_t AxisToOpenLoopWheelCurrent(double axis)
{
	const double current = ApplyAxisDeadband(axis) *
		static_cast<double>(kWheelOpenLoopCurrentLimit);
	return static_cast<int16_t>(std::clamp(
		current,
		-static_cast<double>(kWheelOpenLoopCurrentLimit),
		static_cast<double>(kWheelOpenLoopCurrentLimit)));
}

void PublishRawCanFrame(SeqlockValue<ChassisMotorSendRawFrame> &slot,
			uint8_t bus,
			uint16_t can_id,
			const uint8_t data[8])
{
	ChassisMotorSendRawFrame frame = {};
	frame.bus = bus;
	frame.can_id = can_id;
	frame.dlc = 8U;
	for (size_t i = 0U; i < sizeof(frame.data); ++i) {
		frame.data[i] = data[i];
	}
	slot.write(frame);
	platform::drivers::communication::can_dispatch::NotifyTxPending();
}

void PublishLegOscilloscopeSample(const wbr::v2::GroundBalanceInput &input,
				  const WbrControllerV2Telemetry &telemetry,
				  float leg_length_delta)
{
	static uint32_t sequence = 0U;
	channels::Hi91ImuSample imu_sample = {};
	(void)channels::latest_hi91_imu_sample.read(imu_sample);

	channels::OscilloscopeSample sample = {};
	sample.sequence = ++sequence;
	sample.uptime_ms = k_uptime_get_32();
	sample.channel_count = channels::kOscilloscopeMaxChannels;
	sample.value[0] = leg_length_delta;
	sample.value[1] = static_cast<float>(telemetry.commanded_leg_length);
	sample.value[2] = static_cast<float>(input.leg[0].length);
	sample.value[3] = static_cast<float>(input.leg[1].length);
	sample.value[4] = static_cast<float>(telemetry.commanded_leg_length - input.leg[0].length);
	sample.value[5] = static_cast<float>(telemetry.commanded_leg_length - input.leg[1].length);
	sample.value[6] = static_cast<float>(input.leg[0].angle);
	sample.value[7] = static_cast<float>(input.leg[1].angle);
	sample.value[8] = static_cast<float>(telemetry.leg_angle_error[0]);
	sample.value[9] = static_cast<float>(telemetry.leg_angle_error[1]);
	sample.value[10] = imu_sample.valid ? imu_sample.roll_deg : 0.0F;
	sample.value[11] = imu_sample.valid ? imu_sample.pitch_deg : 0.0F;
	sample.value[12] = imu_sample.valid ? imu_sample.yaw_deg : 0.0F;
	channels::latest_oscilloscope_sample.write(sample);
}



}  // namespace

namespace modules::chassis {

int ChassisModule::Initialize()
{
	started_ = false;
	loop_ticks_ = 0U;
	mit_enter_ticks_ = 0U;
	left_wheel_state_ = {};
	right_wheel_state_ = {};
	left_joint_B_state = {};
	left_joint_D_state = {};
	right_joint_B_state = {};
	right_joint_D_state = {};
	k_timer_init(&loop_timer_, nullptr, nullptr);
	target_leg_length_ = kDefaultTargetLegLength;
	target_linear_velocity_ = 0.0;
	target_yaw_rate_ = 0.0;
	ResetDmTorqueSlew();
	leg_length_controller_.Reset();
	leg_length_controller_.SetLqrEnabled(false);
	leg_length_controller_.SetYawEnabled(false);
	leg_length_controller_.InitializeLegTarget(kDefaultTargetLegLength, kDefaultTargetLegAngle);

	return 0;
}


int ChassisModule::Start()
{
	if (started_) {
		return 0;
	}

	::modules::StartMemberThread<ChassisModule, &ChassisModule::RunLoop>(
		&thread_,
		g_chassis_module_stack,
		K_THREAD_STACK_SIZEOF(g_chassis_module_stack),
		this,
		K_PRIO_PREEMPT(8),
		"chassis_module");
	started_ = true;
	return 0;
}

void ChassisModule::RunLoop()
{
	LOG_INF("chassis module started");
	k_timer_start(&loop_timer_, K_MSEC(1), K_MSEC(1));

	ChassisMotorFeedbackRawFrame left_wheel_feedback_local = {};
	ChassisMotorFeedbackRawFrame right_wheel_feedback_local = {};
	ChassisMotorFeedbackRawFrame left_B_motor_feedback_local = {};
	ChassisMotorFeedbackRawFrame left_D_motor_feedback_local = {};
	ChassisMotorFeedbackRawFrame right_B_motor_feedback_local = {};
	ChassisMotorFeedbackRawFrame right_D_motor_feedback_local = {};

	for (;;) {

		const uint32_t left_B_feedback_sequence = left_B_motor_feedback_raw.sequence();
		const uint32_t left_D_feedback_sequence = left_D_motor_feedback_raw.sequence();
		const uint32_t right_B_feedback_sequence = right_B_motor_feedback_raw.sequence();
		const uint32_t right_D_feedback_sequence = right_D_motor_feedback_raw.sequence();

		left_wheel_feedback_raw.read(left_wheel_feedback_local);
		right_wheel_feedback_raw.read(right_wheel_feedback_local);
		left_B_motor_feedback_raw.read(left_B_motor_feedback_local);
		left_D_motor_feedback_raw.read(left_D_motor_feedback_local);
		right_B_motor_feedback_raw.read(right_B_motor_feedback_local);
		right_D_motor_feedback_raw.read(right_D_motor_feedback_local);

		(void)protocols::motors::dji::DecodeFeedback(left_wheel_feedback_local.data, 8, &left_wheel_state_);
		(void)protocols::motors::dji::DecodeFeedback(right_wheel_feedback_local.data, 8, &right_wheel_state_);
		(void)protocols::motors::dm::DecodeFeedbackNormal(left_B_motor_feedback_local.data, 8, &left_joint_B_state);
		(void)protocols::motors::dm::DecodeFeedbackNormal(left_D_motor_feedback_local.data, 8, &left_joint_D_state);
		(void)protocols::motors::dm::DecodeFeedbackNormal(right_B_motor_feedback_local.data, 8, &right_joint_B_state);
		(void)protocols::motors::dm::DecodeFeedbackNormal(right_D_motor_feedback_local.data, 8, &right_joint_D_state);

		channels::RemoteInputState remote_state = {};
		latest_remote_state.read(remote_state);
		const bool robot_enabled =
			(remote_state.sequence != 0U) && remote_state.robot_enable;
		if (robot_enabled) {
			target_leg_length_ = std::clamp(
				target_leg_length_ +
					static_cast<double>(remote_state.leg_length_delta) *
						kRemoteLegLengthRate * kDefaultControlDt,
				kMinTargetLegLength,
				kMaxTargetLegLength);
			target_linear_velocity_ =
				static_cast<double>(remote_state.chassis_x) *
				kRemoteMaxLinearVelocity;
			target_yaw_rate_ =
				static_cast<double>(remote_state.chassis_rotate) *
				kRemoteMaxYawRate;
		} else {
			target_linear_velocity_ = 0.0;
			target_yaw_rate_ = 0.0;
		}
		leg_length_controller_.SetVelocityCommand(target_linear_velocity_,
							  target_yaw_rate_);

		wbr::v2::GroundBalanceInput controller_input = {};
		controller_input.control_dt = kDefaultControlDt;
		controller_input.target_leg_length = target_leg_length_;
		controller_input.target_leg_angle = kDefaultTargetLegAngle;
		controller_input.total_mass = 8.18;
		controller_input.gravity_magnitude = 9.80665;
		if ((left_B_feedback_sequence != 0U) && (left_D_feedback_sequence != 0U)) {
			controller_input.leg_valid[0] = wbr::v2::ComputeLegKinematics(
				DmPositionRad(left_joint_D_state),
				DmPositionRad(left_joint_B_state),
				DmVelocityRadPerSec(left_joint_D_state),
				DmVelocityRadPerSec(left_joint_B_state),
				kLeftLegKinematicBranch,
				controller_input.leg[0]);
			if (controller_input.leg_valid[0]) {
				controller_input.leg[0].angle = std::remainder(
					controller_input.leg[0].angle - kLeftLegAngleOffset,
					kLegAngleWrap);
			}
		}
		if ((right_B_feedback_sequence != 0U) && (right_D_feedback_sequence != 0U)) {
			controller_input.leg_valid[1] = wbr::v2::ComputeLegKinematics(
				DmPositionRad(right_joint_D_state),
				DmPositionRad(right_joint_B_state),
				DmVelocityRadPerSec(right_joint_D_state),
				DmVelocityRadPerSec(right_joint_B_state),
				kRightLegKinematicBranch,
				controller_input.leg[1]);
			if (controller_input.leg_valid[1]) {
				controller_input.leg[1].angle = std::remainder(
					controller_input.leg[1].angle - kRightLegAngleOffset,
					kLegAngleWrap);
			}
		}

		if ((loop_ticks_ % kLegFeedbackTracePeriod) == 0U) {
			LOG_INF("leg feedback seq LB=%u LD=%u RB=%u RD=%u valid L=%u R=%u",
				static_cast<unsigned int>(left_B_feedback_sequence),
				static_cast<unsigned int>(left_D_feedback_sequence),
				static_cast<unsigned int>(right_B_feedback_sequence),
				static_cast<unsigned int>(right_D_feedback_sequence),
				controller_input.leg_valid[0] ? 1U : 0U,
				controller_input.leg_valid[1] ? 1U : 0U);
		}

		const wbr::v2::GroundBalanceOutput controller_output =
			leg_length_controller_.Update(controller_input);
		PublishLegOscilloscopeSample(controller_input,
					     leg_length_controller_.telemetry(),
					     remote_state.leg_length_delta);
		const auto motor_index = [](wbr::control::MotorId id) {
			return static_cast<int>(id);
		};

		if (!robot_enabled) {
			SendDmExitFrames();
			if (!kDisableWheelCurrentForLegDebug) {
				SendDjiWheelCurrentCommand(kLeftLegBus, kLeftWheelCanId, 0);
				SendDjiWheelCurrentCommand(kRightLegBus, kRightWheelCanId, 0);
			}
			ResetDmTorqueSlew();
			mit_enter_ticks_ = 0U;
		} else if (mit_enter_ticks_ < kMitEnterRepeatTicks) {
			SendDmEnterFrames();
			if (!kDisableWheelCurrentForLegDebug) {
				SendDjiWheelCurrentCommand(kLeftLegBus, kLeftWheelCanId, 0);
				SendDjiWheelCurrentCommand(kRightLegBus, kRightWheelCanId, 0);
			}
			ResetDmTorqueSlew();
			++mit_enter_ticks_;
		} else {
			const auto &left_b = controller_output.actuator.motor[
				motor_index(wbr::control::MotorId::kLeftJointB)];
			const auto &left_d = controller_output.actuator.motor[
				motor_index(wbr::control::MotorId::kLeftJointD)];
			const auto &right_b = controller_output.actuator.motor[
				motor_index(wbr::control::MotorId::kRightJointB)];
			const auto &right_d = controller_output.actuator.motor[
				motor_index(wbr::control::MotorId::kRightJointD)];
			const auto &left_wheel = controller_output.actuator.motor[
				motor_index(wbr::control::MotorId::kLeftWheel)];
			const auto &right_wheel = controller_output.actuator.motor[
				motor_index(wbr::control::MotorId::kRightWheel)];
			int16_t left_wheel_current = 0;
			int16_t right_wheel_current = 0;
			if (kWheelOpenLoopBypassEnabled) {
				left_wheel_current = AxisToOpenLoopWheelCurrent(
					-static_cast<double>(remote_state.chassis_x) -
					static_cast<double>(remote_state.chassis_rotate));
				right_wheel_current = AxisToOpenLoopWheelCurrent(
					(static_cast<double>(remote_state.chassis_x) -
					  static_cast<double>(remote_state.chassis_rotate)));
			} else {
				left_wheel_current = left_wheel.enabled ?
					WheelTorqueToDjiCurrent(left_wheel.torque) : 0;
				right_wheel_current = right_wheel.enabled ?
					WheelTorqueToDjiCurrent(right_wheel.torque) : 0;
			}
			const double left_b_torque =
				SlewDmTorque(kLeftJointBTorqueIndex,
					     left_b.enabled ? left_b.torque : 0.0);
			const double left_d_torque =
				SlewDmTorque(kLeftJointDTorqueIndex,
					     left_d.enabled ? left_d.torque : 0.0);
			const double right_b_torque =
				SlewDmTorque(kRightJointBTorqueIndex,
					     right_b.enabled ? right_b.torque : 0.0);
			const double right_d_torque =
				SlewDmTorque(kRightJointDTorqueIndex,
					     right_d.enabled ? right_d.torque : 0.0);
			if ((loop_ticks_ % kLegTorqueTracePeriod) == 0U) {
				const auto &telemetry = leg_length_controller_.telemetry();
				LOG_INF("leg_pid target=%d target_rate=%d cmd=%d len L=%d R=%d err L=%d R=%d rate L=%d R=%d axial L=%d R=%d int L=%d R=%d tq_cmd LB=%d LD=%d RB=%d RD=%d tq_out LB=%d LD=%d RB=%d RD=%d valid=%u%u en=%u%u%u%u",
					static_cast<int>(telemetry.commanded_leg_length * 1000.0),
					static_cast<int>(telemetry.commanded_leg_length_rate * 1000.0),
					static_cast<int>(remote_state.leg_length_delta * 1000.0f),
					static_cast<int>(controller_input.leg[0].length * 1000.0),
					static_cast<int>(controller_input.leg[1].length * 1000.0),
					static_cast<int>((telemetry.commanded_leg_length -
							  controller_input.leg[0].length) * 1000.0),
					static_cast<int>((telemetry.commanded_leg_length -
							  controller_input.leg[1].length) * 1000.0),
					static_cast<int>(controller_input.leg[0].length_rate * 1000.0),
					static_cast<int>(controller_input.leg[1].length_rate * 1000.0),
					static_cast<int>(telemetry.axial_force[0]),
					static_cast<int>(telemetry.axial_force[1]),
					static_cast<int>(telemetry.integral_force[0]),
					static_cast<int>(telemetry.integral_force[1]),
					static_cast<int>(left_b.torque * 1000.0),
					static_cast<int>(left_d.torque * 1000.0),
					static_cast<int>(right_b.torque * 1000.0),
					static_cast<int>(right_d.torque * 1000.0),
					static_cast<int>(left_b_torque * 1000.0),
					static_cast<int>(left_d_torque * 1000.0),
					static_cast<int>(right_b_torque * 1000.0),
					static_cast<int>(right_d_torque * 1000.0),
					controller_input.leg_valid[0] ? 1U : 0U,
					controller_input.leg_valid[1] ? 1U : 0U,
					left_b.enabled ? 1U : 0U,
					left_d.enabled ? 1U : 0U,
					right_b.enabled ? 1U : 0U,
					right_d.enabled ? 1U : 0U);
			}
			SendDmTorqueCommand(kLeftLegBus, kLeftJointBCanId,
					    left_b_torque);
			SendDmTorqueCommand(kLeftLegBus, kLeftJointDCanId,
					    left_d_torque);
			SendDmTorqueCommand(kRightLegBus, kRightJointBCanId,
					    right_b_torque);
			SendDmTorqueCommand(kRightLegBus, kRightJointDCanId,
					    right_d_torque);
			if (!kDisableWheelCurrentForLegDebug) {
				SendDjiWheelCurrentCommand(kLeftLegBus, kLeftWheelCanId,
							   left_wheel_current);
				SendDjiWheelCurrentCommand(kRightLegBus, kRightWheelCanId,
							   right_wheel_current);
			}
		}
		++loop_ticks_;

		(void)k_timer_status_sync(&loop_timer_);
	}
}

void ChassisModule::SendDmEnterFrames()
{
	uint8_t data[8] = {};
	if (protocols::motors::dm::GetControlCommandFrame(
		    protocols::motors::dm::DmControlCommand::kEnter, data) != 0) {
		return;
	}
	PublishRawCanFrame(left_B_motor_send_raw, kLeftLegBus, kLeftJointBCanId, data);
	PublishRawCanFrame(left_D_motor_send_raw, kLeftLegBus, kLeftJointDCanId, data);
	PublishRawCanFrame(right_B_motor_send_raw, kRightLegBus, kRightJointBCanId, data);
	PublishRawCanFrame(right_D_motor_send_raw, kRightLegBus, kRightJointDCanId, data);
}

void ChassisModule::SendDmExitFrames()
{
	uint8_t data[8] = {};
	if (protocols::motors::dm::GetControlCommandFrame(
		    protocols::motors::dm::DmControlCommand::kExit, data) != 0) {
		return;
	}
	PublishRawCanFrame(left_B_motor_send_raw, kLeftLegBus, kLeftJointBCanId, data);
	PublishRawCanFrame(left_D_motor_send_raw, kLeftLegBus, kLeftJointDCanId, data);
	PublishRawCanFrame(right_B_motor_send_raw, kRightLegBus, kRightJointBCanId, data);
	PublishRawCanFrame(right_D_motor_send_raw, kRightLegBus, kRightJointDCanId, data);
}

void ChassisModule::SendDmTorqueCommand(uint8_t bus, uint16_t can_id, double torque)
{
	protocols::motors::dm::DmMitCommand command = {};
	command.position = 0.0f;
	command.velocity = 0.0f;
	command.kp = 0.0f;
	command.kd = 0.0f;
	command.torque = static_cast<float>(torque);

	uint8_t data[8] = {};
	if (protocols::motors::dm::PackMitCommand(&command, &kDmJointMitRange, data) != 0) {
		return;
	}

	if ((bus == kLeftLegBus) && (can_id == kLeftJointBCanId)) {
		PublishRawCanFrame(left_B_motor_send_raw, bus, can_id, data);
	} else if ((bus == kLeftLegBus) && (can_id == kLeftJointDCanId)) {
		PublishRawCanFrame(left_D_motor_send_raw, bus, can_id, data);
	} else if ((bus == kRightLegBus) && (can_id == kRightJointBCanId)) {
		PublishRawCanFrame(right_B_motor_send_raw, bus, can_id, data);
	} else if ((bus == kRightLegBus) && (can_id == kRightJointDCanId)) {
		PublishRawCanFrame(right_D_motor_send_raw, bus, can_id, data);
	}
}

double ChassisModule::SlewDmTorque(uint8_t joint_index, double target_torque)
{
	if (joint_index >= (sizeof(last_dm_torque_) / sizeof(last_dm_torque_[0]))) {
		return target_torque;
	}

	const double max_step = kDmJointTorqueSlewRate * kDefaultControlDt;
	last_dm_torque_[joint_index] = std::clamp(
		target_torque,
		last_dm_torque_[joint_index] - max_step,
		last_dm_torque_[joint_index] + max_step);
	return last_dm_torque_[joint_index];
}

void ChassisModule::ResetDmTorqueSlew()
{
	for (double &torque : last_dm_torque_) {
		torque = 0.0;
	}
}

void ChassisModule::SendDjiWheelCurrentCommand(uint8_t bus,
					       uint16_t motor_can_id,
					       int16_t current)
{
	uint8_t data[8] = {};
	if (protocols::motors::dji::WriteCurrentCommandToSlot(
		    motor_can_id, current, data) != 0) {
		return;
	}

	if ((bus == kLeftLegBus) && (motor_can_id == kLeftWheelCanId)) {
		PublishRawCanFrame(left_wheel_send_raw, bus, kDjiCurrentCommandCanId,
				   data);
	} else if ((bus == kRightLegBus) && (motor_can_id == kRightWheelCanId)) {
		PublishRawCanFrame(right_wheel_send_raw, bus,
				   kDjiCurrentCommandCanId, data);
	}
}



}  // namespace modules::chassis
