/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include <platform/drivers/communication/can_dispatch.h>
#include <protocols/motors/dm_motor_protocol.h>

#include <chassis_params_generated.h>

#include "chassis_types.h"

namespace modules::chassis_config
{

using namespace chassis_params;

struct MotorAddress {
	uint8_t bus;
	uint16_t can_id;
	platform::CanTxSlot tx_slot;
};

struct LegHardwareMap {
	MotorAddress wheel;
	JointPair<MotorAddress> joint;
	int kinematic_branch;
	double leg_angle_offset;
	double leg_coordinate_sign;
	double wheel_feedback_sign;
	double vmc_torque_sign;
	double wheel_command_sign;
};

inline constexpr SidePair<LegHardwareMap> kLegHardware = {
	.left = {{kLeftWheelBus, kLeftWheelCanId, platform::CanTxSlot::kLeftWheel},
		 {{kLeftJointBBus, kLeftJointBCanId, platform::CanTxSlot::kLeftJointB},
		  {kLeftJointDBus, kLeftJointDCanId, platform::CanTxSlot::kLeftJointD}},
		 kLeftKinematicBranch, kLeftLegAngleOffset, kLeftLegCoordinateSign,
		 kLeftWheelFeedbackSign, kLeftVmcTorqueSign, kLeftWheelCommandSign},
	.right = {{kRightWheelBus, kRightWheelCanId, platform::CanTxSlot::kRightWheel},
		  {{kRightJointBBus, kRightJointBCanId, platform::CanTxSlot::kRightJointB},
		   {kRightJointDBus, kRightJointDCanId, platform::CanTxSlot::kRightJointD}},
		  kRightKinematicBranch, kRightLegAngleOffset, kRightLegCoordinateSign,
		  kRightWheelFeedbackSign, kRightVmcTorqueSign, kRightWheelCommandSign},
};

inline constexpr protocols::DmMitRange kDmRange = {
	.p_min = kDmPositionMin, .p_max = kDmPositionMax,
	.v_min = kDmVelocityMin, .v_max = kDmVelocityMax,
	.kp_min = kDmKpMin, .kp_max = kDmKpMax,
	.kd_min = kDmKdMin, .kd_max = kDmKdMax,
	.t_min = kDmTorqueMin, .t_max = kDmTorqueMax,
};

inline constexpr double kTwoPi = 6.28318530717958647692;
inline constexpr double kDegToRad = 0.01745329251994329577;
inline constexpr double kDpsToRadPerSec = kDegToRad;
inline constexpr double kRpmToRadPerSec = 0.10471975511965977;
inline constexpr double kMaxYawRate = kMaxTurnWheelSpeed / kHalfWheelTrack;
inline constexpr double kYawAccelerationLimit = kTurnAccelerationLimit / kHalfWheelTrack;
inline constexpr double kYawTrackingErrorLimit = kYawTrackingErrorLimitDeg * kDegToRad;
inline constexpr double kWheelTorqueLimit =
	static_cast<double>(kDjiProtocolCurrentLimit) / kDjiCurrentPerNm;
inline constexpr double kThetaBalanceBias = kThetaBalanceBiasDeg * kDegToRad;

} // namespace modules::chassis_config
