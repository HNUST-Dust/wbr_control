/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>

#include <msg/chassismotors_feedback_raw.hpp>
#include <msg/remote_input_state.hpp>
#include <protocols/motors/dji_motor_protocol.h>
#include <protocols/motors/dm_motor_protocol.h>

#include "leg_kinematics.h"

namespace modules
{

enum class Side : uint8_t {
	kLeft = 0U,
	kRight,
};

enum class Joint : uint8_t {
	kB = 0U,
	kD,
};

template <typename T> struct SidePair {
	T left{};
	T right{};

	T &Get(Side side) { return side == Side::kLeft ? left : right; }
	const T &Get(Side side) const { return side == Side::kLeft ? left : right; }
};

template <typename T> struct JointPair {
	T b{};
	T d{};

	T &Get(Joint joint) { return joint == Joint::kB ? b : d; }
	const T &Get(Joint joint) const { return joint == Joint::kB ? b : d; }
};

enum class ChassisControlState : uint8_t {
	kDisabled = 0U,
	kSafetyStop,
	kDmArming,
	kWaitingFeedback,
	kStool,
	kBalance,
	kTiltFault,
};

enum class DmArmResetReason : uint8_t {
	kStartup = 0U,
	kRemoteEnableEdge,
	kImuFreshness,
	kTiltFault,
};

struct ChassisImuSample {
	uint32_t sequence = 0U;
	uint64_t precise_timestamp_us = 0U;
	uint32_t max_sensor_interval_ms = 0U;
	uint32_t crc_error_count = 0U;
	uint32_t publish_gap_with_crc_count = 0U;
	uint32_t publish_gap_without_crc_count = 0U;
	uint32_t max_publish_interval_us = 0U;
	uint32_t transport_error_count = 0U;
	bool valid = false;
	float pitch_deg = 0.0F;
	float yaw_deg = 0.0F;
	float pitch_rate_rad_s = 0.0F;
	float yaw_rate_rad_s = 0.0F;
};

/** Frozen input for one control cycle. */
struct ChassisCycleInput {
	SidePair<msg::ChassisMotorFeedbackRawFrame> wheel_frame;
	SidePair<JointPair<msg::ChassisMotorFeedbackRawFrame>> joint_frame;
	ChassisImuSample imu;
	msg::RemoteInputState remote = {};
	SidePair<LegKinematics> leg;
	SidePair<double> wheel_motor_speed_rpm;
	SidePair<JointPair<double>> joint_position;
	SidePair<JointPair<double>> joint_velocity;
	double dt = 0.001;
	double pitch = 0.0;
	double pitch_rate = 0.0;
	SidePair<double> theta;
	SidePair<double> theta_rate;
	double common_theta = 0.0;
	double common_theta_rate = 0.0;
	bool imu_fresh = false;
	uint64_t imu_age_us = UINT64_MAX;
	uint32_t remote_age_ms = UINT32_MAX;
	bool feedback_valid = false;
	bool requested_enable = false;
	bool dm_ready = false;
	bool arm_complete = false;
};

/** Physical actuator requests and diagnostics produced by one cycle. */
struct ChassisControlOutput {
	SidePair<double> physical_wheel_torque;
	SidePair<double> sent_wheel_torque;
	SidePair<bool> wheel_torque_saturated;
	SidePair<double> requested_body_on_leg_torque;
	SidePair<double> body_on_leg_torque;
	SidePair<double> leg_axial_force;
	SidePair<JointPair<double>> joint_torque;
	double raw_body_speed = 0.0;
	double estimated_body_speed = 0.0;
	double measured_turn_wheel_speed = 0.0;
	double requested_turn_torque = 0.0;
	double allocated_turn_torque = 0.0;
	double differential_leg_torque = 0.0;
};

} // namespace modules
