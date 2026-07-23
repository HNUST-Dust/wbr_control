/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>

#include <zephyr/kernel.h>

#include <modules/chassis/leg_kinematics.h>
#include <modules/chassis/body_motion_estimator.h>
#include <modules/chassis/stool_controller.h>
#include <protocols/motors/dji_motor_protocol.h>
#include <protocols/motors/dm_motor_protocol.h>

namespace modules {

class ChassisModule {
public:
	const char *Name() const { return "chassis"; }
	int Initialize();
	int Start();

private:
	enum class StartupPhase : uint8_t {
		kStool = 0U,
		kBalance = 1U,
	};

	struct SideState {
		double leg_length_integral_force = 0.0;
	};

	void RunLoop();
	void ResetControlState();
	void SendDmControl(protocols::DmControlCommand command);
	void SendScheduledOutputs(const double joint_torque[4],
				 int16_t left_wheel_current,
				 int16_t right_wheel_current);
	void SendDmTorque(uint8_t bus, uint16_t can_id, double torque);
	void SendWheelCurrent(uint8_t bus, uint16_t motor_can_id, int16_t current);

	struct k_thread thread_;
	bool started_ = false;
	bool last_requested_enable_ = false;
	bool tilt_fault_latched_ = false;
	uint32_t loop_ticks_ = 0U;
	uint32_t dm_arm_ticks_ = 0U;
	uint32_t last_remote_sequence_ = 0U;
	uint32_t last_remote_update_ms_ = 0U;
	uint32_t last_imu_sequence_ = 0U;
	uint32_t last_imu_update_ms_ = 0U;
	uint64_t last_loop_time_us_ = 0U;
	StartupPhase startup_phase_ = StartupPhase::kStool;
	StoolController stool_controller_;
	BodyMotionEstimator body_motion_estimator_;
	bool stool_ready_ = false;
	double target_leg_length_ = StoolController::kTargetLegLength;
	SideState side_[2];

	protocols::DjiMotorFeedback left_wheel_{};
	protocols::DjiMotorFeedback right_wheel_{};
	protocols::DmMotorFeedbackNormal left_b_{};
	protocols::DmMotorFeedbackNormal left_d_{};
	protocols::DmMotorFeedbackNormal right_b_{};
	protocols::DmMotorFeedbackNormal right_d_{};
};

}  // namespace modules
