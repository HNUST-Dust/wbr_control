/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <stdint.h>

#include <modules/ahrs/onboard_imu_module.h>
#include <modules/ahrs/quaternion_ekf.h>
#include <modules/module_base.h>

namespace modules {

/**
 * Board AHRS: owns the IMU and quaternion EKF and runs them in one thread so
 * each estimate is produced directly from the completed DMA sample.
 */
class Ahrs final : public ModuleBase {
public:
	int Start() override;
	void RunLoop() override;

private:
	void ProcessBurst(const OnboardImu::Burst &burst);
	void PublishSample(uint32_t data_ready_cycle);
	void PublishTelemetry();
	float UpdateSampleInterval(uint32_t data_ready_cycle);
	bool InitializeAttitude(const float accel_mps2[3]);
	void UpdateEstimator(const float gyro_rad_s[3], const float accel_mps2[3],
			     float dt_seconds);

	OnboardImu imu_;
	QuaternionEkf ekf_;

	uint32_t calibration_sample_count_ = 0U;
	float gyro_bias_sum_[3] = {};
	float accel_sum_[3] = {};
	float gyro_bias_rad_s_[3] = {};
	float accel_scale_ = 1.0F;
	bool calibrated_ = false;
	bool attitude_initialized_ = false;
	bool timing_valid_ = false;
	uint32_t last_data_ready_cycle_ = 0U;
	uint32_t sample_interval_us_ = 0U;
	bool sensor_data_valid_ = false;
};

}  // namespace modules
