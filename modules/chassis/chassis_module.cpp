/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <modules/chassis/chassis_module.h>

#include <algorithm>
#include <cmath>

#include <zephyr/logging/log.h>

#include <channels/chassismotors_feedback_raw.hpp>
#include <channels/hi91_imu_sample.hpp>
#include <channels/oscilloscope_sample.hpp>
#include <channels/remote_input_state.hpp>
#include <modules/chassis/lqr_schedule.h>
#include <modules/chassis/leg_vmc.h>
#include <modules/thread_utils.h>
#include <platform/drivers/communication/can_dispatch.h>

LOG_MODULE_REGISTER(chassis_module, LOG_LEVEL_INF);

namespace {

K_THREAD_STACK_DEFINE(g_chassis_stack, 4096);

// CAN 总线号和ID
constexpr uint8_t kLeftLegBus = 3U;
constexpr uint8_t kRightLegBus = 1U;
constexpr uint16_t kLeftJointBId = 0x00U;
constexpr uint16_t kLeftJointDId = 0x03U;
constexpr uint16_t kRightJointBId = 0x01U;
constexpr uint16_t kRightJointDId = 0x02U;
constexpr uint16_t kWheelMotorId = 0x201U;
constexpr uint16_t kWheelCommandId = 0x200U;

// 运动求解器和控制器参数
constexpr int kLeftBranch = 1;
constexpr int kRightBranch = -1;
constexpr double kLeftLegAngleOffset = -0.036063;
constexpr double kRightLegAngleOffset = -3.121010;
constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kDegToRad = 0.01745329251994329577;
constexpr double kDpsToRadPerSec = kDegToRad;
constexpr double kRpmToRadPerSec = 0.10471975511965977;
constexpr double kWheelReduction = 268.0 / 17.0;
constexpr double kWheelRadius = 0.058;
constexpr double kGravity = 9.80665;
constexpr double kRobotMass = 10.1;
constexpr double kDefaultDt = 0.001;
constexpr double kTargetLegLengthMin = 0.15133;
constexpr double kTargetLegLengthMax = 0.30347;
constexpr double kTargetLegLengthRate = 0.06;

constexpr double kPerSideGainScale = 0.5;
constexpr double kPositionErrorLimit = 0.15;
constexpr double kLegCoordinateKp = 6.0;
constexpr double kLegCoordinateKd = 0.6;
constexpr double kLegCoordinateTorqueLimit = 2.0;
constexpr double kDjiCurrentPerNm = 3450.0;
/* DJI's 0x200 current command uses the complete signed ±16384 command
 * domain. This is a protocol-range guard, not a chassis torque limit. */
constexpr int16_t kDjiProtocolCurrentLimit = 16384;
constexpr double kJointTorqueLimit = 54.0;

constexpr uint32_t kRemoteTimeoutMs = 100U;
constexpr uint32_t kImuTimeoutMs = 30U;
constexpr uint64_t kMotorTimeoutUs = 20000ULL;
/* Stool-mode stand-up can traverse a large body pitch.  Keep an automatic
 * cutoff for genuinely abnormal motion, while allowing the ground-to-stool
 * transition to complete; the remote enable switch remains the primary stop. */
constexpr double kTiltCutoffDeg = 75.0;
constexpr uint32_t kDmClearTicks = 50U;
constexpr uint32_t kDmArmTicks = 300U;
constexpr uint32_t kDmModePeriodTicks = 10U;

// 平衡点
constexpr double kThetaBalanceBias = 0.0 * kDegToRad;

constexpr protocols::DmMitRange kDmRange = {
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

}  // namespace

namespace modules {

int ChassisModule::Initialize()
{
	started_ = false;
	last_requested_enable_ = false;
	tilt_fault_latched_ = false;
	loop_ticks_ = 0U;
	dm_arm_ticks_ = 0U;
	last_remote_sequence_ = 0U;
	last_remote_update_ms_ = 0U;
	last_imu_sequence_ = 0U;
	last_imu_update_ms_ = 0U;
	last_loop_time_us_ = 0U;
	startup_phase_ = StartupPhase::kStool;
	stool_ready_ = false;
	target_leg_length_ = StoolController::kTargetLegLength;
	stool_controller_.Reset();
	ResetControlState();
	return 0;
}

int ChassisModule::Start()
{
	if (started_) {
		return 0;
	}
	::modules::StartMemberThread<ChassisModule, &ChassisModule::RunLoop>(
		&thread_, g_chassis_stack,
		K_THREAD_STACK_SIZEOF(g_chassis_stack), this,
		K_PRIO_PREEMPT(8), "chassis");
	started_ = true;
	return 0;
}

void ChassisModule::RunLoop()
{
	LOG_INF("chassis started");

	ChassisMotorFeedbackRawFrame wheel_frame[2] = {};
	ChassisMotorFeedbackRawFrame joint_frame[4] = {};
	channels::Hi91ImuSample imu = {};
	int64_t next_release_tick = k_uptime_ticks();

	for (;;) {
		// 计算本周期实际 dt，限制异常调度延迟对积分器和滤波器的影响。
		const uint64_t cycle_start = k_cycle_get_64();
		const uint64_t now_us = k_cyc_to_us_floor64(cycle_start);
		const uint32_t now_ms = k_uptime_get_32();
		double dt = kDefaultDt;
		if (last_loop_time_us_ != 0U && now_us > last_loop_time_us_) {
			dt = std::clamp(static_cast<double>(now_us - last_loop_time_us_) * 1.0e-6,
					0.0005, 0.01);
		}
		last_loop_time_us_ = now_us;

		// 获取六个执行器的最新原始 CAN 帧，并解码成协议层反馈。
		left_wheel_feedback_raw.read(wheel_frame[0]);
		right_wheel_feedback_raw.read(wheel_frame[1]);
		left_B_motor_feedback_raw.read(joint_frame[0]);
		left_D_motor_feedback_raw.read(joint_frame[1]);
		right_B_motor_feedback_raw.read(joint_frame[2]);
		right_D_motor_feedback_raw.read(joint_frame[3]);

		const bool wheel_decoded[2] = {
			wheel_frame[0].valid && protocols::DecodeDjiFeedback(
				wheel_frame[0].data, 8U, &left_wheel_) == 0,
			wheel_frame[1].valid && protocols::DecodeDjiFeedback(
				wheel_frame[1].data, 8U, &right_wheel_) == 0,
		};
		const bool joint_decoded[4] = {
			joint_frame[0].valid && protocols::DecodeDmFeedbackNormal(
				joint_frame[0].data, 8U, &left_b_) == 0,
			joint_frame[1].valid && protocols::DecodeDmFeedbackNormal(
				joint_frame[1].data, 8U, &left_d_) == 0,
			joint_frame[2].valid && protocols::DecodeDmFeedbackNormal(
				joint_frame[2].data, 8U, &right_b_) == 0,
			joint_frame[3].valid && protocols::DecodeDmFeedbackNormal(
				joint_frame[3].data, 8U, &right_d_) == 0,
		};

		// 遥控器和 IMU 通过 sequence 判断是否真的收到过新样本，避免把
		// 通道中残留的旧值继续当成有效控制输入。
		channels::RemoteInputState remote = {};
		latest_remote_state.read(remote);
		if (remote.sequence != 0U && remote.sequence != last_remote_sequence_) {
			last_remote_sequence_ = remote.sequence;
			last_remote_update_ms_ = now_ms;
		}
		const bool remote_fresh = last_remote_sequence_ != 0U &&
			(now_ms - last_remote_update_ms_) <= kRemoteTimeoutMs;

		channels::Hi91ImuSample new_imu = {};
		if (channels::latest_hi91_imu_sample.read(new_imu) && new_imu.sequence != 0U) {
			imu = new_imu;
			if (imu.sequence != last_imu_sequence_) {
				last_imu_sequence_ = imu.sequence;
				last_imu_update_ms_ = now_ms;
			}
		}
		const bool imu_fresh = imu.valid && last_imu_sequence_ != 0U &&
			(now_ms - last_imu_update_ms_) <= kImuTimeoutMs;

		// 电机反馈同时要求“解码成功”和“时间戳未超时”。任一电机掉线，
		// 本周期都不允许进入闭环控制。
		bool motors_fresh = true;
		for (int index = 0; index < 2; ++index) {
			motors_fresh = motors_fresh && wheel_decoded[index] &&
				wheel_frame[index].IsFresh(now_us, kMotorTimeoutUs);
		}
		bool joint_fresh[4] = {};
		for (int index = 0; index < 4; ++index) {
			joint_fresh[index] = joint_decoded[index] &&
				joint_frame[index].IsFresh(now_us, kMotorTimeoutUs);
			motors_fresh = motors_fresh && joint_fresh[index];
		}

		// 正运动学把四个髋关节状态转换为左右腿的长度、腿角及其速度；
		// 安装偏置和镜像在这里一次性处理，后续控制统一使用同一坐标约定。
		LegKinematics leg[2] = {};
		bool leg_valid[2] = {};
		if (joint_decoded[0] && joint_decoded[1]) {
			leg_valid[0] = ComputeLegKinematics(
				protocols::DmFeedbackPosition(left_d_, kDmRange),
				protocols::DmFeedbackPosition(left_b_, kDmRange),
				protocols::DmFeedbackVelocity(left_d_, kDmRange),
				protocols::DmFeedbackVelocity(left_b_, kDmRange),
				kLeftBranch, leg[0]);
			if (leg_valid[0]) {
				leg[0].angle = -std::remainder(leg[0].angle - kLeftLegAngleOffset,
							      kTwoPi);
				leg[0].angle_rate = -leg[0].angle_rate;
			}
		}
		if (joint_decoded[2] && joint_decoded[3]) {
			leg_valid[1] = ComputeLegKinematics(
				protocols::DmFeedbackPosition(right_d_, kDmRange),
				protocols::DmFeedbackPosition(right_b_, kDmRange),
				protocols::DmFeedbackVelocity(right_d_, kDmRange),
				protocols::DmFeedbackVelocity(right_b_, kDmRange),
				kRightBranch, leg[1]);
			if (leg_valid[1]) {
				leg[1].angle = std::remainder(leg[1].angle - kRightLegAngleOffset,
							     kTwoPi);
			}
		}

		const bool feedback_valid = motors_fresh && imu_fresh &&
			leg_valid[0] && leg_valid[1];
		const bool requested_enable = remote_fresh && remote.robot_enable;

		// 处理使能沿和倾倒保护。每次重新使能都从 Stool 阶段开始，清除
		// 上一次运行留下的估计器、积分器和关节目标状态。
		if (!requested_enable && tilt_fault_latched_) {
			tilt_fault_latched_ = false;
		}

		if (requested_enable != last_requested_enable_) {
			ResetControlState();
			dm_arm_ticks_ = 0U;
			startup_phase_ = StartupPhase::kStool;
			stool_ready_ = false;
			stool_controller_.Reset();
			target_leg_length_ = StoolController::kTargetLegLength;
			last_requested_enable_ = requested_enable;
			LOG_INF("chassis request %s", requested_enable ? "enabled" : "disabled");
		}
		if (requested_enable && imu_fresh) {
			const double raw_pitch_deg = static_cast<double>(imu.pitch_deg);
			if (!tilt_fault_latched_ &&
				std::abs(raw_pitch_deg) >= kTiltCutoffDeg) {
				tilt_fault_latched_ = true;
				LOG_ERR("tilt fault pitch_mdeg=%d", static_cast<int>(raw_pitch_deg * 1000.0));
			}
		}

		const bool dm_ready = left_b_.control_status == 1U &&
			left_d_.control_status == 1U && right_b_.control_status == 1U &&
			right_d_.control_status == 1U;
		const bool arm_complete = dm_arm_ticks_ >= kDmArmTicks;
		const bool control_enabled = requested_enable && feedback_valid &&
			!tilt_fault_latched_ && arm_complete && dm_ready;

		// 整理控制器公共输入和本周期输出缓冲区。theta 是腿相对世界竖直
		// 方向的角度，满足 theta = alpha - pitch。
		const double pitch = imu_fresh ? static_cast<double>(imu.pitch_deg) * kDegToRad : 0.0;
		const double pitch_rate = imu_fresh ?
			static_cast<double>(imu.gyro_dps[1]) * kDpsToRadPerSec : 0.0;
		double theta[2] = {};
		double theta_rate[2] = {};
		double common_theta = 0.0;
		double common_theta_rate = 0.0;
		double physical_wheel_torque[2] = {};
		double sent_wheel_torque[2] = {};
		bool wheel_torque_saturated[2] = {};
		double requested_body_on_leg_torque[2] = {};
		double body_on_leg_torque[2] = {};
		double joint_torque[4] = {};
		bool stool_target_initialized = false;
		const std::array<double, 4> joint_position = {
			protocols::DmFeedbackPosition(left_b_, kDmRange),
			protocols::DmFeedbackPosition(left_d_, kDmRange),
			protocols::DmFeedbackPosition(right_b_, kDmRange),
			protocols::DmFeedbackPosition(right_d_, kDmRange),
		};
		const std::array<double, 4> joint_velocity = {
			protocols::DmFeedbackVelocity(left_b_, kDmRange),
			protocols::DmFeedbackVelocity(left_d_, kDmRange),
			protocols::DmFeedbackVelocity(right_b_, kDmRange),
			protocols::DmFeedbackVelocity(right_d_, kDmRange),
		};

		if (feedback_valid) {
			/* The article's coordinates are alpha = theta + phi, hence
			 * theta_i = alpha_i - phi and d_theta_i = d_alpha_i - d_phi.
			 * leg[].angle is the calibrated alpha defined above. */
			theta[0] = std::remainder(leg[0].angle - pitch, kTwoPi);
			theta[1] = std::remainder(leg[1].angle - pitch, kTwoPi);
			theta_rate[0] = leg[0].angle_rate - pitch_rate;
			theta_rate[1] = leg[1].angle_rate - pitch_rate;
			common_theta = std::remainder(
				0.5 * (leg[0].angle + leg[1].angle) - pitch,
				kTwoPi);
			common_theta_rate =
				0.5 * (leg[0].angle_rate + leg[1].angle_rate) - pitch_rate;
		}

		// 启动阶段：StoolController 负责腿长收缩、腿角对齐和关节级联 PID。
		// 两侧 theta 收敛后，本周期结束时直接切换到完整 Balance LQR。
		if (control_enabled && startup_phase_ == StartupPhase::kStool) {
			target_leg_length_ = StoolController::kTargetLegLength;
			StoolControllerInput stool_input = {};
			stool_input.pitch = pitch;
			stool_input.dt = dt;
			stool_input.leg = {leg[0], leg[1]};
			stool_input.joint_position = joint_position;
			stool_input.joint_velocity = joint_velocity;
			const StoolControllerOutput stool = stool_controller_.Update(stool_input);
			std::copy(stool.joint_torque.begin(), stool.joint_torque.end(), joint_torque);
			stool_target_initialized = stool.target_initialized;
			stool_ready_ = stool.ready;
			if (stool.ready) {
				startup_phase_ = StartupPhase::kBalance;
				target_leg_length_ = 0.5 * (leg[0].length + leg[1].length);
				ResetControlState();
				LOG_INF("stool theta aligned; entering full LQR");
			}
		} else if (control_enabled) {
			// 平衡阶段第一步：结合轮速和腿部运动补偿，估计机体前向速度。
			const double raw_wheel_omega[2] = {
				static_cast<double>(left_wheel_.omega) * kRpmToRadPerSec /
					kWheelReduction,
				static_cast<double>(right_wheel_.omega) * kRpmToRadPerSec /
					kWheelReduction,
			};
			/* Match SPR's chassis_speed_calc(). Raw wheel speed is not body speed
			 * while the leg angle changes: express each wheel in world frame
			 * (omega + phi_dot - alpha_dot), then add hip motion. */
			const double wheel_world_omega[2] = {
				raw_wheel_omega[0] + pitch_rate - leg[0].angle_rate,
				-raw_wheel_omega[1] + pitch_rate - leg[1].angle_rate,
			};
			double body_speed[2] = {};
			for (int side = 0; side < 2; ++side) {
				body_speed[side] = -kWheelRadius * wheel_world_omega[side] +
					leg[side].length_rate * std::sin(theta[side]) +
					leg[side].length * theta_rate[side] * std::cos(theta[side]);
				}
			const double raw_common_x_speed = 0.5 * (body_speed[0] + body_speed[1]);
			/* The IMU forward axis has not yet been calibrated for this chassis.
			 * Pass zero rather than inventing an axis sign; SPR's acceleration
			 * measurement noise is intentionally large. */
			const BodyMotionState &motion =
				body_motion_estimator_.Update(raw_common_x_speed, 0.0, dt);
			const double position_error = std::clamp(
				motion.position, -kPositionErrorLimit, kPositionErrorLimit);

			// 平衡阶段第二步：按当前平均腿长调度 LQR 增益，分别计算每侧
			// 轮毂驱动力矩和作用于腿部的姿态力矩。
			const double state_error[6] = {
				common_theta - kThetaBalanceBias,
				common_theta_rate,
				position_error,
				motion.speed,
				pitch,
				pitch_rate,
			};
			const double common_leg_length = 0.5 * (leg[0].length + leg[1].length);
			double side_gain[2][2][6] = {};
			EvaluateLqrGain(common_leg_length, side_gain[0]);
			EvaluateLqrGain(common_leg_length, side_gain[1]);
			for (int side = 0; side < 2; ++side) {
				double wheel_sum = 0.0;
				double leg_sum = 0.0;
				for (int state = 0; state < 6; ++state) {
					wheel_sum += side_gain[side][0][state] * state_error[state];
					leg_sum += side_gain[side][1][state] * state_error[state];
				}
				/* No chassis wheel-torque clamp: LQR may use the full wheel
				 * authority needed to recover the stool handoff.  CAN conversion
				 * below still guards the finite DJI protocol command range. */
				const double wheel_torque = -kPerSideGainScale * wheel_sum;
				physical_wheel_torque[side] =
					std::isfinite(wheel_torque) ? wheel_torque : 0.0;
				requested_body_on_leg_torque[side] =
					std::isfinite(leg_sum) ?
						-kPerSideGainScale * leg_sum : 0.0;
				/* Keep the LQR leg-posture authority intact. The final per-joint
				 * kJointTorqueLimit remains the sole actuator protection. */
				body_on_leg_torque[side] = requested_body_on_leg_torque[side];
			}

			// 左右腿协调项抑制两侧腿角分离，只在两侧之间重新分配力矩。
			const double coordinate_torque_request =
				kLegCoordinateKp *
					std::remainder(leg[0].angle - leg[1].angle, kTwoPi) +
					kLegCoordinateKd * (leg[0].angle_rate - leg[1].angle_rate);
			const double coordinate_torque = std::isfinite(coordinate_torque_request) ?
				std::clamp(coordinate_torque_request, -kLegCoordinateTorqueLimit,
					kLegCoordinateTorqueLimit) : 0.0;
			body_on_leg_torque[0] -= coordinate_torque;
			body_on_leg_torque[1] += coordinate_torque;

			// 平衡阶段第三步：VMC 将支撑力、腿长闭环和腿部姿态力矩转换为
			// 四个髋关节的最终力矩命令。
			for (int side = 0; side < 2; ++side) {
				const double length_error = target_leg_length_ - leg[side].length;
				side_[side].leg_length_integral_force =
					UpdateLegLengthIntegral(
						side_[side].leg_length_integral_force,
						length_error, dt);
				const double support_force = 0.5 * kRobotMass * kGravity /
					std::max(std::cos(theta[side]), 0.5);
				// Tp is body-on-leg. ComputeLegVmc consumes leg-on-body. The
				// right five-bar generalized angle is mirrored once more.
				const double vmc_angle_torque = side == 0 ?
					-body_on_leg_torque[side] : body_on_leg_torque[side];
				const LegVmcOutput vmc = ComputeLegVmc(
					leg[side], target_leg_length_, support_force,
					side_[side].leg_length_integral_force,
					vmc_angle_torque, leg[side].length_rate, 0.0);
				const int base = 2 * side;
				joint_torque[base] = std::isfinite(vmc.joint_torque[1]) ?
					std::clamp(vmc.joint_torque[1], -kJointTorqueLimit,
						kJointTorqueLimit) : 0.0;
				joint_torque[base + 1] = std::isfinite(vmc.joint_torque[0]) ?
					std::clamp(vmc.joint_torque[0], -kJointTorqueLimit,
						kJointTorqueLimit) : 0.0;
			}
		}

		// 最终安全门控优先级：停机/故障 -> DM 重新使能 -> Stool 输出 ->
		// Balance 输出。只有最后两条路径会向执行器发送非零控制量。
		if (!requested_enable || tilt_fault_latched_ || !imu_fresh) {
			if ((loop_ticks_ % 100U) == 0U) {
				SendDmControl(protocols::DmControlCommand::kExit);
			}
			if ((loop_ticks_ % 10U) == 0U) {
				SendWheelCurrent(kLeftLegBus, kWheelMotorId, 0);
				SendWheelCurrent(kRightLegBus, kWheelMotorId, 0);
			}
			dm_arm_ticks_ = 0U;
			ResetControlState();
		} else if (!arm_complete || !dm_ready || !feedback_valid) {
			// Do not require fresh DM feedback before sending Enter.  A motor
			// that was explicitly exited may not publish fresh feedback until
			// it receives the mode command again.
			if ((loop_ticks_ % kDmModePeriodTicks) == 0U) {
				SendDmControl(dm_arm_ticks_ < kDmClearTicks ?
					protocols::DmControlCommand::kClearError :
					protocols::DmControlCommand::kEnter);
			}
			SendWheelCurrent(kLeftLegBus, kWheelMotorId, 0);
			SendWheelCurrent(kRightLegBus, kWheelMotorId, 0);
			ResetControlState();
			if (dm_arm_ticks_ < kDmArmTicks) {
				++dm_arm_ticks_;
			}
		} else if (startup_phase_ == StartupPhase::kStool &&
			stool_target_initialized) {
			/* The joint targets above are regulated by our software cascade;
			 * Send only torque-mode MIT frames.  Wheels remain exactly zero until
			 * the stool convergence test has passed. */
			SendScheduledOutputs(joint_torque, 0, 0);
		} else {
			/* The motor-current mapping follows the original left/right 3508
			 * installation. Keep this independent from the LQR coordinates: output
			 * authority is tuned by the explicit torque/current limits above. */
			const double left_motor_torque = -physical_wheel_torque[0];
			const double right_motor_torque = physical_wheel_torque[1];
			const auto torque_to_current = [](double torque) {
				const double current = std::isfinite(torque) ? std::clamp(
					torque * kDjiCurrentPerNm,
					-static_cast<double>(kDjiProtocolCurrentLimit),
					static_cast<double>(kDjiProtocolCurrentLimit)) : 0.0;
				return static_cast<int16_t>(current);
			};
			const int16_t left_wheel_current = torque_to_current(left_motor_torque);
			const int16_t right_wheel_current = torque_to_current(right_motor_torque);
			wheel_torque_saturated[0] =
				std::abs(static_cast<int32_t>(left_wheel_current)) >=
				kDjiProtocolCurrentLimit;
			wheel_torque_saturated[1] =
				std::abs(static_cast<int32_t>(right_wheel_current)) >=
				kDjiProtocolCurrentLimit;
			/* Telemetry must expose what reaches CAN, not an LQR request that
			 * may be much larger than the 3508 command domain. Convert back to
			 * the common physical forward-torque convention after the per-side
			 * motor mounting signs. */
			sent_wheel_torque[0] =
				-static_cast<double>(left_wheel_current) / kDjiCurrentPerNm;
			sent_wheel_torque[1] =
				static_cast<double>(right_wheel_current) / kDjiCurrentPerNm;
			SendScheduledOutputs(
				joint_torque,
				left_wheel_current, right_wheel_current);
		}

		// 发布本周期状态、请求力矩和实际下发力矩，供 VOFA 定位交接、
		// 饱和及安全门控问题；遥测不参与控制反馈。
		static uint32_t sequence = 0U;
		float balance_state_x100 = 0.0F;
		if (requested_enable) {
			/* Split the former ambiguous state 100 so a single VOFA trace tells
			 * whether a restart came from the DM arming sequence, missing feedback,
			 * or another control gate. */
			balance_state_x100 = tilt_fault_latched_ ? 400.0F :
				(!arm_complete ? 110.0F :
				 (!feedback_valid ? 120.0F :
				  (!control_enabled ? 130.0F :
				   (startup_phase_ == StartupPhase::kStool ? 200.0F : 300.0F))));
		}

		channels::OscilloscopeSample sample = {};
		sample.sequence = ++sequence;
		sample.uptime_ms = k_uptime_get_32();
		sample.channel_count = 14U;
		sample.value[0] = balance_state_x100;
		sample.value[1] = stool_ready_ ? 1.0F : 0.0F;
		/* Publish the model state that must converge during the handoff. */
		sample.value[2] = static_cast<float>(common_theta / kDegToRad);
		sample.value[3] = static_cast<float>(physical_wheel_torque[0]);
		sample.value[4] = static_cast<float>(physical_wheel_torque[1]);
		sample.value[5] = static_cast<float>(sent_wheel_torque[0]);
		sample.value[6] = static_cast<float>(sent_wheel_torque[1]);
		sample.value[7] = static_cast<float>(pitch / kDegToRad);
		sample.value[8] = wheel_torque_saturated[0] ? 1.0F : 0.0F;
		sample.value[9] = wheel_torque_saturated[1] ? 1.0F : 0.0F;
		sample.value[10] = static_cast<float>(requested_body_on_leg_torque[0]);
		sample.value[11] = static_cast<float>(body_on_leg_torque[0]);
		sample.value[12] = static_cast<float>(requested_body_on_leg_torque[1]);
		sample.value[13] = static_cast<float>(body_on_leg_torque[1]);
		channels::latest_oscilloscope_sample.write(sample);

		++loop_ticks_;
		// 使用绝对 1 ms 时刻释放，避免相对睡眠向上取整后退化成 500 Hz。
		++next_release_tick;
		const int64_t current_tick = k_uptime_ticks();
		if (next_release_tick <= current_tick) {
			next_release_tick = current_tick + 1;
		}
		k_sleep(K_TIMEOUT_ABS_TICKS(next_release_tick));
	}
}

void ChassisModule::ResetControlState()
{
	body_motion_estimator_.Reset();
	for (SideState &side : side_) {
		side = {};
	}
}

void ChassisModule::SendDmControl(protocols::DmControlCommand command)
{
	uint8_t data[8] = {};
	if (protocols::GetDmControlCommandFrame(command, data) != 0) {
		return;
	}
	k_sched_lock();
	using platform::SubmitCanStandardFrame;
	using platform::CanTxSlot;
	(void)SubmitCanStandardFrame(CanTxSlot::kLeftJointB,
		kLeftLegBus, kLeftJointBId, data, sizeof(data));
	(void)SubmitCanStandardFrame(CanTxSlot::kRightJointB,
		kRightLegBus, kRightJointBId, data, sizeof(data));
	(void)SubmitCanStandardFrame(CanTxSlot::kLeftJointD,
		kLeftLegBus, kLeftJointDId, data, sizeof(data));
	(void)SubmitCanStandardFrame(CanTxSlot::kRightJointD,
		kRightLegBus, kRightJointDId, data, sizeof(data));
	k_sched_unlock();
}

void ChassisModule::SendScheduledOutputs(const double joint_torque[4],
					 int16_t left_wheel_current,
					 int16_t right_wheel_current)
{
	if (joint_torque == nullptr) {
		return;
	}

	// Keep the six submissions atomic with respect to the higher-priority CAN
	// workers so every control cycle is published as one coherent output set.
	k_sched_lock();
	SendDmTorque(kLeftLegBus, kLeftJointBId, joint_torque[0]);
	SendDmTorque(kRightLegBus, kRightJointBId, joint_torque[2]);
	SendDmTorque(kLeftLegBus, kLeftJointDId, joint_torque[1]);
	SendDmTorque(kRightLegBus, kRightJointDId, joint_torque[3]);
	SendWheelCurrent(kLeftLegBus, kWheelMotorId, left_wheel_current);
	SendWheelCurrent(kRightLegBus, kWheelMotorId, right_wheel_current);
	k_sched_unlock();
}

void ChassisModule::SendDmTorque(uint8_t bus, uint16_t can_id, double torque)
{
	protocols::DmMitCommand command = {};
	command.torque = static_cast<float>(torque);
	uint8_t data[8] = {};
	if (protocols::PackDmMitCommand(&command, &kDmRange, data) != 0) {
		return;
	}
	platform::CanTxSlot slot =
		platform::CanTxSlot::kLeftJointB;
	if (bus == kLeftLegBus && can_id == kLeftJointDId) {
		slot = platform::CanTxSlot::kLeftJointD;
	} else if (bus == kRightLegBus && can_id == kRightJointBId) {
		slot = platform::CanTxSlot::kRightJointB;
	} else if (bus == kRightLegBus && can_id == kRightJointDId) {
		slot = platform::CanTxSlot::kRightJointD;
	}
	(void)platform::SubmitCanStandardFrame(
		slot, bus, can_id, data, sizeof(data));
}

void ChassisModule::SendWheelCurrent(uint8_t bus, uint16_t motor_can_id,
				     int16_t current)
{
	uint8_t data[8] = {};
	if (protocols::WriteDjiCurrentCommandToSlot(
		    motor_can_id, current, data) != 0) {
		return;
	}
	const auto slot = bus == kLeftLegBus ?
		platform::CanTxSlot::kLeftWheel :
		platform::CanTxSlot::kRightWheel;
	(void)platform::SubmitCanStandardFrame(
		slot, bus, kWheelCommandId, data, sizeof(data));
}
}  // namespace modules
