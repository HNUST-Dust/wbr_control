//=============================================================================================
// Mahony attitude estimator for the onboard IMU pipeline.
// Based on Madgwick's implementation of Mahony's AHRS algorithm.
//=============================================================================================

#pragma once

#include <array>

namespace modules {

class MahonyAttitudeEstimator {
public:
	// Gyroscope inputs are rad/s. Accelerometer inputs are m/s^2 (only the
	// direction is used). Magnetometer inputs may use any consistent unit.
	void Init(float sample_frequency_hz);
	void InitFromAccMag(float ax, float ay, float az, float mx, float my, float mz);

	void Update(float gx, float gy, float gz, float ax, float ay, float az,
		    float mx, float my, float mz);
	void UpdateImu(float gx, float gy, float gz, float ax, float ay, float az);

	void ComputeAngles();

	float RollDeg();
	float PitchDeg();
	float YawDeg();

	std::array<float, 4> Quat() const { return {q0_, q1_, q2_, q3_}; }

private:
	static float InvSqrt(float x);

	static constexpr float kRad2Deg = 57.29578f;
	static constexpr float kTwoKp = 2.0f * 0.5f;
	static constexpr float kTwoKiDefault = 2.0f * 0.0f;
	static constexpr float kVectorNormEpsilon = 1.0e-12f;

	float two_ki_ = kTwoKiDefault;

	float q0_ = 1.0f;
	float q1_ = 0.0f;
	float q2_ = 0.0f;
	float q3_ = 0.0f;

	float integral_fbx_ = 0.0f;
	float integral_fby_ = 0.0f;
	float integral_fbz_ = 0.0f;

	float inv_sample_freq_ = 0.0f;

	float roll_deg_ = 0.0f;
	float pitch_deg_ = 0.0f;
	float yaw_deg_ = 0.0f;
	bool angles_computed_ = false;
};

} // namespace modules
