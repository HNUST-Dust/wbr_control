/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <zephyr/kernel.h>

#include <modules/chassis/ground_balance_controller.h>
#include <protocols/motors/dji_motor_protocol.h>
#include <protocols/motors/dm_motor_protocol.h>
namespace modules::chassis {

class ChassisModule {
public:
	const char *Name() const { return "chassis"; }
	int Initialize();
	int Start();

private:
	static constexpr uint32_t kMitEnterRepeatTicks = 1000U;

	void RunLoop();
	void SendDmEnterFrames();
	void SendDmExitFrames();
	void SendDmTorqueCommand(uint8_t bus, uint16_t can_id, double torque);
	double SlewDmTorque(uint8_t joint_index, double target_torque);
	void ResetDmTorqueSlew();
	void SendDjiWheelCurrentCommand(uint8_t bus, uint16_t motor_can_id,
					int16_t current);

	struct k_thread thread_;
	struct k_timer loop_timer_;
	bool started_ = false;
	uint32_t loop_ticks_ = 0U;
	uint32_t mit_enter_ticks_ = 0U;
	double target_leg_length_ = 0.18;
	double target_linear_velocity_ = 0.0;
	double target_yaw_rate_ = 0.0;
	double last_dm_torque_[4] = {};

	protocols::motors::dji::DjiMotorFeedback left_wheel_state_;
	protocols::motors::dji::DjiMotorFeedback right_wheel_state_;
	protocols::motors::dm::DmMotorFeedbackNormal left_joint_B_state;
	protocols::motors::dm::DmMotorFeedbackNormal right_joint_B_state;
	protocols::motors::dm::DmMotorFeedbackNormal left_joint_D_state;
	protocols::motors::dm::DmMotorFeedbackNormal right_joint_D_state;
	wbr::v2::GroundBalanceController leg_length_controller_;

};

}  // namespace modules::chassis
