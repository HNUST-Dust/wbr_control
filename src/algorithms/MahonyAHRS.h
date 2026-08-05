//=============================================================================================
// MahonyAHRS.h
//=============================================================================================
//
// Madgwick's implementation of Mahony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// Date            Author          Notes
// 29/09/2011      SOH Madgwick    Initial release
// 02/10/2011      SOH Madgwick    Optimised for reduced CPU load
//
//=============================================================================================

#pragma once

// Public reusable attitude-estimation interface.

#define ARM_MATH_CM7

#include <array>

namespace alg {

class MahonyAhrs {
public:
	void Init(float sampleFrequencyHz);
	void InitFromAccMag(float ax, float ay, float az, float mx, float my, float mz);

	void Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
	void UpdateImu(float gx, float gy, float gz, float ax, float ay, float az);

	void ComputeAngles();

	float RollDeg();
	float PitchDeg();
	float YawDeg();

	std::array<float, 4> Quat() const { return {q0_, q1_, q2_, q3_}; }

	static float InvSqrt(float x);

private:
	static constexpr float kRad2Deg = 57.29578f;
	static constexpr float kTwoKp = (2.0f * 0.5f);
	static constexpr float kTwoKiDefault = (2.0f * 0.0f);

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

} // namespace alg
