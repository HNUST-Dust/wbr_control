/* SPDX-License-Identifier: Apache-2.0 */

#include "chassis_imu_adapter.h"

#include <algorithm>
#include <cmath>

#include <msg/hi91_imu_sample.hpp>
#include <msg/onboard_imu_sample.hpp>
#include <zephyr/sys/util.h>

#include <tf_tree.h>

#include "chassis_config.h"

namespace modules
{
namespace
{

#if defined(CONFIG_WBR_CONTROL_CHASSIS_IMU_ONBOARD_EKF)
bool TransformOnboardImuToChassis(const msg::OnboardImuSample &imu,
				  ChassisImuSample &sample)
{
	constexpr float kRadToDeg = 57.295779513082320876F;
	const float roll = imu.euler_deg[0] * static_cast<float>(chassis_config::kDegToRad);
	const float pitch = imu.euler_deg[1] * static_cast<float>(chassis_config::kDegToRad);
	const float yaw = imu.euler_deg[2] * static_cast<float>(chassis_config::kDegToRad);
	const float cr = std::cos(roll);
	const float sr = std::sin(roll);
	const float cp = std::cos(pitch);
	const float sp = std::sin(pitch);
	const float cy = std::cos(yaw);
	const float sy = std::sin(yaw);

	wbr_control::TfTree::Matrix3 imu_attitude;
	imu_attitude << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
		sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
		-sp, cp * sr, cp * cr;
	wbr_control::TfTree::Matrix3 chassis_attitude;
	if (!wbr_control::robot_tf_tree.TransformAttitude(
		    wbr_control::TfTree::Frame::kChassis,
		    wbr_control::TfTree::Frame::kChassisImu,
		    imu_attitude, chassis_attitude)) {
		return false;
	}

	wbr_control::TfTree::Vector3 chassis_gyro;
	if (!wbr_control::robot_tf_tree.TransformVector(
		    wbr_control::TfTree::Frame::kChassis,
		    wbr_control::TfTree::Frame::kChassisImu,
		    wbr_control::TfTree::Vector3(imu.gyro_rad_s[0], imu.gyro_rad_s[1],
					     imu.gyro_rad_s[2]),
		    chassis_gyro)) {
		return false;
	}

	const float pitch_sine = std::clamp(-chassis_attitude(2, 0), -1.0F, 1.0F);
	sample.pitch_deg = std::asin(pitch_sine) * kRadToDeg;
	sample.roll_deg = std::atan2(chassis_attitude(2, 1), chassis_attitude(2, 2)) * kRadToDeg;
	sample.yaw_deg = std::atan2(chassis_attitude(1, 0), chassis_attitude(0, 0)) * kRadToDeg;
	sample.pitch_rate_rad_s = chassis_gyro.y();
	sample.roll_rate_rad_s = chassis_gyro.x();
	sample.yaw_rate_rad_s = chassis_gyro.z();
	// The onboard channel does not publish raw acceleration yet. Recovery will
	// fail closed instead of guessing the upright side when this source is used.
	sample.vertical_accel_g = 0.0F;
	return std::isfinite(sample.pitch_deg) && std::isfinite(sample.roll_deg) &&
	       std::isfinite(sample.yaw_deg) &&
	       std::isfinite(sample.pitch_rate_rad_s) &&
	       std::isfinite(sample.yaw_rate_rad_s);
}
#endif

} // namespace

void InitializeChassisImuAdapter()
{
#if defined(CONFIG_WBR_CONTROL_CHASSIS_IMU_ONBOARD_EKF)
	static constexpr float kChassisFromImuQuaternion[4] = {0.0F, 0.0F, 1.0F, 0.0F};
	const Eigen::Map<const wbr_control::TfTree::Rotation> chassis_from_imu(
		kChassisFromImuQuaternion);
	wbr_control::robot_tf_tree.ConfigureChassisBranch(
		wbr_control::TfTree::Rotation(chassis_from_imu));
#endif
}

bool ReadChassisImuSample(ChassisImuSample &sample)
{
	sample = {};
#if defined(CONFIG_WBR_CONTROL_CHASSIS_IMU_ONBOARD_EKF)
	msg::OnboardImuSample imu = {};
	if (!msg::latest_onboard_imu_sample.read(imu) || imu.timestamp_us == 0U) {
		return false;
	}
	sample.sequence = 1U;
	sample.precise_timestamp_us = imu.timestamp_us;
	const bool transform_valid = TransformOnboardImuToChassis(imu, sample);
	sample.valid = imu.valid && transform_valid &&
		IS_ENABLED(CONFIG_WBR_CONTROL_ONBOARD_IMU_BODY_MAP_CONFIRMED);
	return true;
#elif defined(CONFIG_WBR_CONTROL_CHASSIS_IMU_HI91)
	msg::Hi91ImuSample imu = {};
	if (!msg::latest_hi91_imu_sample.read(imu) || imu.sequence == 0U) {
		return false;
	}
	sample.sequence = imu.sequence;
	sample.precise_timestamp_us = imu.precise_timestamp_us;
	sample.max_sensor_interval_ms = imu.max_sensor_interval_ms;
	sample.crc_error_count = imu.crc_error_count;
	sample.publish_gap_with_crc_count = imu.publish_gap_with_crc_count;
	sample.publish_gap_without_crc_count = imu.publish_gap_without_crc_count;
	sample.max_publish_interval_us = imu.max_publish_interval_us;
	sample.transport_error_count = imu.rx_drop_count + imu.rx_stop_count +
		imu.rx_buf_rsp_error_count;
	sample.valid = imu.valid;
	sample.pitch_deg = imu.pitch_deg;
	sample.roll_deg = imu.roll_deg;
	sample.yaw_deg = imu.yaw_deg;
	sample.pitch_rate_rad_s = static_cast<float>(
		static_cast<double>(imu.gyro_dps[0]) * chassis_config::kDpsToRadPerSec);
	sample.roll_rate_rad_s = static_cast<float>(
		static_cast<double>(imu.gyro_dps[1]) * chassis_config::kDpsToRadPerSec);
	sample.yaw_rate_rad_s = static_cast<float>(
		static_cast<double>(imu.gyro_dps[2]) * chassis_config::kDpsToRadPerSec);
	for (size_t axis = 0U; axis < 3U; ++axis) {
		sample.accel_g[axis] = imu.accel_g[axis];
	}
	// HI91 Z acceleration is used as the chassis vertical direction. Its sign
	// must be confirmed by a face-up/face-down bench test before recovery use.
	sample.vertical_accel_g = imu.accel_g[2];
	return true;
#else
#error "The chassis module requires one configured IMU source"
#endif
}

msg::ChassisImuSource SelectedChassisImuSource()
{
#if defined(CONFIG_WBR_CONTROL_CHASSIS_IMU_ONBOARD_EKF)
	return msg::ChassisImuSource::kOnboardEkf;
#elif defined(CONFIG_WBR_CONTROL_CHASSIS_IMU_HI91)
	return msg::ChassisImuSource::kHi91;
#else
#error "The chassis module requires one configured IMU source"
#endif
}

} // namespace modules
