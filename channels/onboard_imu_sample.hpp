/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

#include <channels/comm/seqlock_value.hpp>

namespace channels {

/** Compact 1 kHz onboard attitude snapshot. */
struct OnboardImuSample {
	uint64_t timestamp_us; ///< Physical sample time derived from the DRDY edge.
	float euler_deg[3]; ///< Chassis-IMU-frame EKF roll, pitch, yaw in degrees.
	float gyro_rad_s[3]; ///< Chassis-IMU-frame, EKF bias-corrected angular velocity.
	bool valid; ///< Calibration, timing, sensor, and EKF state are valid.
};

static_assert(sizeof(OnboardImuSample) == 40U,
	      "Keep the 1 kHz onboard IMU snapshot compact");

extern SeqlockValue<OnboardImuSample> latest_onboard_imu_sample;

}  // namespace channels
