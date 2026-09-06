/* SPDX-License-Identifier: Apache-2.0 */

/**
 * @file ahrs.cpp
 * @ingroup wbr_modules
 * @brief Implements the 1 kHz AHRS thread, calibration, estimation, and publication.
 */

#include "modules/ahrs/quaternion_ekf.h"
#include "zephyr/kernel.h"
#include <modules/ahrs/ahrs.h>

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/timing/timing.h>

#include <channels/onboard_imu_sample.hpp>
#include <channels/oscilloscope_sample.hpp>
#include <scheduling/periodic_schedule.h>
#include <scheduling/thread_priorities.h>

#if defined(CONFIG_WBR_CONTROL_MODULE_AHRS)

namespace {
constexpr size_t kAxisCount = 3U;
constexpr uint32_t kCalibrationSampleCount =
	CONFIG_WBR_CONTROL_ONBOARD_IMU_CALIBRATION_SAMPLES;
constexpr float kGravityMps2 = 9.80665F;
constexpr float kAccelLsbPerG = 4096.0F;
constexpr float kGyroLsbPerDps = 16.384F;
constexpr float kDegToRad = 0.01745329251994329577F;
constexpr float kSampleFrequencyHz = 1000.0F;
constexpr uint32_t kAhrsPeriodMs = 1U;
constexpr uint32_t kNominalSampleIntervalUs = 1000U;
constexpr uint32_t kMinimumSaneSampleIntervalUs = 400U;
constexpr uint32_t kMaximumSaneSampleIntervalUs = 5000U;
/* The HXY part can carry a sizeable board-level acceleration scale/offset
 * error. Accept a stationary 1 g vector here, then estimate one scalar gain
 * from the complete calibration window before feeding acceleration to EKF.
 */
constexpr float kCalibrationAccelToleranceMps2 = 2.0F;
constexpr float kCalibrationGyroLimitRadS = 0.1F;
K_THREAD_STACK_DEFINE(g_ahrs_stack, 4096);

int16_t DecodeBigEndian(const uint8_t *bytes)
{
	return static_cast<int16_t>((static_cast<uint16_t>(bytes[0]) << 8U) |
				    static_cast<uint16_t>(bytes[1]));
}
}  // namespace

namespace modules {

int Ahrs::Start()
{
	if (started_) {
		return 0;
	}

	int rc = imu_.Init();
	if (rc != 0) {
		return rc;
	}

	/* Proven 1 kHz tuning from the previous BMI088 implementation.  The
	 * surrounding port keeps the newer finite-value, covariance and timing
	 * guards while retaining the on-robot noise balance.
	 */
	QuaternionEkf::Params ekf_params = {
		.process_noise_quat_ = 10.0F,
		.process_noise_gyro_bias_ = 1.0e-3F,
		.measure_noise_accel_ = 1.0e7F,
		.fading_lambda_ = 1.0F,
		.dt_ = 1.0F / kSampleFrequencyHz,
		.accel_lpf_coef_ = 0.0F,
	};
	ekf_.Init(ekf_params);

	return CreateThread(g_ahrs_stack, K_THREAD_STACK_SIZEOF(g_ahrs_stack),
			    K_PRIO_PREEMPT(wbr_control::scheduling::thread_priority::kAhrs),
			    "ahrs");
}

void Ahrs::RunLoop()
{
	wbr_control::scheduling::AbsolutePeriodicSchedule release(kAhrsPeriodMs, 0U);
	for (;;) {
		(void)release.WaitForNextRelease();
		OnboardImu::Burst burst = {};
		const bool sample_ready = imu_.TryTakeCompleted(burst);
		(void)imu_.TryStartAsync();
		if (sample_ready) {
			ProcessBurst(burst);
		}
	}
}
void Ahrs::PublishTelemetry()
{
	static uint32_t sequence = 0U;
	channels::OscilloscopeSample sample = {};
	sample.sequence = ++ sequence;
	sample.uptime_ms = k_uptime_get_32();
	sample.channel_count = 3;

	sample.value[0] = ekf_.PitchDeg();
	sample.value[1] = ekf_.YawDeg();
	sample.value[2] = ekf_.RollDeg();
	channels::latest_oscilloscope_sample.write(sample);
}

void Ahrs::ProcessBurst(const OnboardImu::Burst &burst)
{
	const uint8_t *const rx_data = burst.rx;
	const uint32_t data_ready_cycle = burst.data_ready_cycle;
	int16_t sensor_accel_raw[kAxisCount];
	int16_t sensor_gyro_raw[kAxisCount];
	float sensor_accel_mps2[kAxisCount];
	float sensor_gyro_rad_s[kAxisCount];

	for (size_t axis = 0U; axis < kAxisCount; ++axis) {
		sensor_accel_raw[axis] = DecodeBigEndian(&rx_data[1U + axis * 2U]);
		sensor_gyro_raw[axis] = DecodeBigEndian(&rx_data[7U + axis * 2U]);
		sensor_accel_mps2[axis] = static_cast<float>(sensor_accel_raw[axis]) *
					   (kGravityMps2 / kAccelLsbPerG);
		sensor_gyro_rad_s[axis] = static_cast<float>(sensor_gyro_raw[axis]) *
						  (kDegToRad / kGyroLsbPerDps);
	}
	float accel_mps2[kAxisCount];
	float gyro_rad_s[kAxisCount];
	memcpy(accel_mps2, sensor_accel_mps2, sizeof(accel_mps2));
	memcpy(gyro_rad_s, sensor_gyro_rad_s, sizeof(gyro_rad_s));
	sensor_data_valid_ = true;
	bool all_axes_zero = true;
	for (size_t axis = 0U; axis < kAxisCount; ++axis) {
		if (sensor_accel_raw[axis] == INT16_MIN ||
		    sensor_accel_raw[axis] == INT16_MAX ||
		    sensor_gyro_raw[axis] == INT16_MIN ||
		    sensor_gyro_raw[axis] == INT16_MAX) {
			sensor_data_valid_ = false;
		}
		all_axes_zero = all_axes_zero && sensor_accel_raw[axis] == 0 &&
				sensor_gyro_raw[axis] == 0;
	}
	sensor_data_valid_ = sensor_data_valid_ && !all_axes_zero;
	if (!sensor_data_valid_) {
		PublishSample(data_ready_cycle);
		return;
	}
	const float dt_seconds = UpdateSampleInterval(data_ready_cycle);

	if (!calibrated_) {
		const float accel_norm = sqrtf(accel_mps2[0] * accel_mps2[0] +
					       accel_mps2[1] * accel_mps2[1] +
					       accel_mps2[2] * accel_mps2[2]);
		const float gyro_norm = sqrtf(gyro_rad_s[0] * gyro_rad_s[0] +
					      gyro_rad_s[1] * gyro_rad_s[1] +
					      gyro_rad_s[2] * gyro_rad_s[2]);
		if ((fabsf(accel_norm - kGravityMps2) >
		     kCalibrationAccelToleranceMps2) ||
		    (gyro_norm > kCalibrationGyroLimitRadS)) {
			calibration_sample_count_ = 0U;
			memset(gyro_bias_sum_, 0, sizeof(gyro_bias_sum_));
			memset(accel_sum_, 0, sizeof(accel_sum_));
			PublishSample(data_ready_cycle);
			return;
		}
		for (size_t axis = 0U; axis < kAxisCount; ++axis) {
			gyro_bias_sum_[axis] += gyro_rad_s[axis];
			accel_sum_[axis] += accel_mps2[axis];
		}
		++calibration_sample_count_;
		if (calibration_sample_count_ == kCalibrationSampleCount) {
			float average_accel[kAxisCount];
			for (size_t axis = 0U; axis < kAxisCount; ++axis) {
				gyro_bias_rad_s_[axis] =
					gyro_bias_sum_[axis] /
					static_cast<float>(kCalibrationSampleCount);
				average_accel[axis] = accel_sum_[axis] /
					      static_cast<float>(kCalibrationSampleCount);
			}
			const float average_accel_norm =
				sqrtf(average_accel[0] * average_accel[0] +
				      average_accel[1] * average_accel[1] +
				      average_accel[2] * average_accel[2]);
			if (!std::isfinite(average_accel_norm) || average_accel_norm < 1.0F) {
				calibration_sample_count_ = 0U;
				memset(gyro_bias_sum_, 0, sizeof(gyro_bias_sum_));
				memset(accel_sum_, 0, sizeof(accel_sum_));
				PublishSample(data_ready_cycle);
				return;
			}
			accel_scale_ = kGravityMps2 / average_accel_norm;
			for (size_t axis = 0U; axis < kAxisCount; ++axis) {
				average_accel[axis] *= accel_scale_;
			}
			calibrated_ = true;
			for (size_t axis = 0U; axis < kAxisCount; ++axis) {
				gyro_rad_s[axis] -= gyro_bias_rad_s_[axis];
			}
			attitude_initialized_ = InitializeAttitude(average_accel);
		}
		PublishSample(data_ready_cycle);
		return;
	}

	for (size_t axis = 0U; axis < kAxisCount; ++axis) {
		gyro_rad_s[axis] -= gyro_bias_rad_s_[axis];
		accel_mps2[axis] *= accel_scale_;
	}
	if (!timing_valid_ || !attitude_initialized_) {
		attitude_initialized_ = InitializeAttitude(accel_mps2);
	} else {
		UpdateEstimator(gyro_rad_s, accel_mps2, dt_seconds);
		if (!ekf_.Healthy()) {
			attitude_initialized_ = InitializeAttitude(accel_mps2);
			timing_valid_ = false;
		}
	}
	PublishSample(data_ready_cycle);
	PublishTelemetry();
}

float Ahrs::UpdateSampleInterval(uint32_t data_ready_cycle)
{
	sample_interval_us_ = kNominalSampleIntervalUs;
	timing_valid_ = false;
	if (last_data_ready_cycle_ != 0U) {
		sample_interval_us_ = k_cyc_to_us_floor32(data_ready_cycle - last_data_ready_cycle_);
		timing_valid_ = sample_interval_us_ >= kMinimumSaneSampleIntervalUs &&
				sample_interval_us_ <= kMaximumSaneSampleIntervalUs;
	}
	last_data_ready_cycle_ = data_ready_cycle;
	return static_cast<float>(sample_interval_us_) * 1.0e-6F;
}

bool Ahrs::InitializeAttitude(const float accel_mps2[3])
{
	ekf_.Reset();
	return ekf_.InitFromAccel(accel_mps2[0], accel_mps2[1], accel_mps2[2]);
}

void Ahrs::UpdateEstimator(const float gyro_rad_s[3], const float accel_mps2[3],
			   float dt_seconds)
{
	/* One physical sample produces exactly one correction. Replaying the same
	 * accelerometer measurement to subdivide a delayed interval would
	 * artificially over-weight it and can create catch-up CPU bursts.
	 */
	ekf_.Update(gyro_rad_s[0], gyro_rad_s[1], gyro_rad_s[2],
		    accel_mps2[0], accel_mps2[1], accel_mps2[2], dt_seconds);
}

void Ahrs::PublishSample(uint32_t data_ready_cycle)
{
	channels::OnboardImuSample sample = {};
	const uint64_t publish_cycle = k_cycle_get_64();
	const uint32_t acquisition_cycles =
		static_cast<uint32_t>(publish_cycle) - data_ready_cycle;
	sample.timestamp_us = k_cyc_to_us_floor64(publish_cycle - acquisition_cycles);
	const auto gyro_rad_s = ekf_.GyroRadS();
	memcpy(sample.gyro_rad_s, gyro_rad_s.data(), sizeof(sample.gyro_rad_s));
	sample.euler_deg[0] = ekf_.RollDeg();
	sample.euler_deg[1] = ekf_.PitchDeg();
	sample.euler_deg[2] = ekf_.YawDeg();
	sample.valid = calibrated_ && timing_valid_ && attitude_initialized_ &&
		       sensor_data_valid_ && ekf_.Healthy();
	channels::latest_onboard_imu_sample.write(sample);
}
}  // namespace modules

#else

namespace modules {

int Ahrs::Start()
{
	return -ENOTSUP;
}

void Ahrs::RunLoop()
{
}

void Ahrs::ProcessBurst(const OnboardImu::Burst &) {}
void Ahrs::PublishSample(uint32_t)
{
}
float Ahrs::UpdateSampleInterval(uint32_t) { return 0.0F; }
bool Ahrs::InitializeAttitude(const float[3]) { return false; }
void Ahrs::UpdateEstimator(const float[3], const float[3], float) {}

}  // namespace modules

#endif
