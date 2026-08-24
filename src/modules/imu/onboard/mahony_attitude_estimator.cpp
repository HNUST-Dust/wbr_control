/**
* @file src/modules/imu/onboard/mahony_attitude_estimator.cpp
 * @ingroup wbr_modules
 * @brief 实现基于 Mahony 滤波器的姿态估计。
 * @details 实现运行在模块自有 Zephyr 线程或其驱动回调中。回调路径只完成有界的数据搬运和通知，耗时解析与控制计算留在线程上下文执行。
 */

// Mahony attitude estimator used by the onboard IMU pipeline.

#include <modules/imu/onboard/mahony_attitude_estimator.h>

#include <algorithm>
#include <cmath>

namespace modules {

void MahonyAttitudeEstimator::Init(float sample_frequency_hz)
{
	two_ki_ = kTwoKiDefault;
	q0_ = 1.0f;
	q1_ = 0.0f;
	q2_ = 0.0f;
	q3_ = 0.0f;
	integral_fbx_ = 0.0f;
	integral_fby_ = 0.0f;
	integral_fbz_ = 0.0f;
	inv_sample_freq_ = (sample_frequency_hz > 0.0f) ? (1.0f / sample_frequency_hz) : 0.0f;
	roll_deg_ = 0.0f;
	pitch_deg_ = 0.0f;
	yaw_deg_ = 0.0f;
	angles_computed_ = false;
}

float MahonyAttitudeEstimator::InvSqrt(float x)
{
	return (x > kVectorNormEpsilon) ? (1.0f / std::sqrt(x)) : 0.0f;
}

void MahonyAttitudeEstimator::InitFromAccMag(float ax, float ay, float az,
					     float mx, float my, float mz)
{
	const float accel_inv_norm = InvSqrt(ax * ax + ay * ay + az * az);
	if (accel_inv_norm == 0.0f) {
		return;
	}
	ax *= accel_inv_norm;
	ay *= accel_inv_norm;
	az *= accel_inv_norm;

	const float init_pitch = std::atan2(-ax, az);
	const float init_roll = std::atan2(ay, az);
	float init_yaw = 0.0f;

	const float mag_inv_norm = InvSqrt(mx * mx + my * my + mz * mz);
	if (mag_inv_norm != 0.0f) {
		mx *= mag_inv_norm;
		my *= mag_inv_norm;
		mz *= mag_inv_norm;

		const float sin_roll = std::sin(init_roll);
		const float cos_roll = std::cos(init_roll);
		const float sin_pitch = std::sin(init_pitch);
		const float cos_pitch = std::cos(init_pitch);
		const float mag_x = mx * cos_pitch + my * sin_pitch * sin_roll +
				    mz * sin_pitch * cos_roll;
		const float mag_y = my * cos_roll - mz * sin_roll;
		init_yaw = std::atan2(-mag_y, mag_x);
	}

	const float cr2 = std::cos(init_roll * 0.5f);
	const float cp2 = std::cos(init_pitch * 0.5f);
	const float cy2 = std::cos(init_yaw * 0.5f);
	const float sr2 = std::sin(init_roll * 0.5f);
	const float sp2 = std::sin(init_pitch * 0.5f);
	const float sy2 = std::sin(init_yaw * 0.5f);

	q0_ = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
	q1_ = sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
	q2_ = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
	q3_ = cr2 * cp2 * sy2 - sr2 * sp2 * cy2;

	const float quat_inv_norm = InvSqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
	q0_ *= quat_inv_norm;
	q1_ *= quat_inv_norm;
	q2_ *= quat_inv_norm;
	q3_ *= quat_inv_norm;
	angles_computed_ = false;
}

void MahonyAttitudeEstimator::Update(float gx, float gy, float gz,
				     float ax, float ay, float az,
				     float mx, float my, float mz)
{
	const float mag_inv_norm = InvSqrt(mx * mx + my * my + mz * mz);
	if (mag_inv_norm == 0.0f) {
		UpdateImu(gx, gy, gz, ax, ay, az);
		return;
	}

	const float accel_inv_norm = InvSqrt(ax * ax + ay * ay + az * az);
	if (accel_inv_norm != 0.0f) {
		ax *= accel_inv_norm;
		ay *= accel_inv_norm;
		az *= accel_inv_norm;
		mx *= mag_inv_norm;
		my *= mag_inv_norm;
		mz *= mag_inv_norm;

		const float q0q0 = q0_ * q0_;
		const float q0q1 = q0_ * q1_;
		const float q0q2 = q0_ * q2_;
		const float q0q3 = q0_ * q3_;
		const float q1q1 = q1_ * q1_;
		const float q1q2 = q1_ * q2_;
		const float q1q3 = q1_ * q3_;
		const float q2q2 = q2_ * q2_;
		const float q2q3 = q2_ * q3_;
		const float q3q3 = q3_ * q3_;

		const float hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) +
					my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		const float hy = 2.0f * (mx * (q1q2 + q0q3) +
					my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		const float bx = std::sqrt(hx * hx + hy * hy);
		const float bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) +
					 mz * (0.5f - q1q1 - q2q2));

		const float half_vx = q1q3 - q0q2;
		const float half_vy = q0q1 + q2q3;
		const float half_vz = q0q0 - 0.5f + q3q3;
		const float half_wx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		const float half_wy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		const float half_wz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		const float half_ex = (ay * half_vz - az * half_vy) +
				      (my * half_wz - mz * half_wy);
		const float half_ey = (az * half_vx - ax * half_vz) +
				      (mz * half_wx - mx * half_wz);
		const float half_ez = (ax * half_vy - ay * half_vx) +
				      (mx * half_wy - my * half_wx);

		if (two_ki_ > 0.0f) {
			integral_fbx_ += two_ki_ * half_ex * inv_sample_freq_;
			integral_fby_ += two_ki_ * half_ey * inv_sample_freq_;
			integral_fbz_ += two_ki_ * half_ez * inv_sample_freq_;
			gx += integral_fbx_;
			gy += integral_fby_;
			gz += integral_fbz_;
		} else {
			integral_fbx_ = 0.0f;
			integral_fby_ = 0.0f;
			integral_fbz_ = 0.0f;
		}

		gx += kTwoKp * half_ex;
		gy += kTwoKp * half_ey;
		gz += kTwoKp * half_ez;
	}

	const float half_dt = 0.5f * inv_sample_freq_;
	gx *= half_dt;
	gy *= half_dt;
	gz *= half_dt;
	const float qa = q0_;
	const float qb = q1_;
	const float qc = q2_;
	q0_ += -qb * gx - qc * gy - q3_ * gz;
	q1_ += qa * gx + qc * gz - q3_ * gy;
	q2_ += qa * gy - qb * gz + q3_ * gx;
	q3_ += qa * gz + qb * gy - qc * gx;

	const float quat_inv_norm = InvSqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
	if (quat_inv_norm != 0.0f) {
		q0_ *= quat_inv_norm;
		q1_ *= quat_inv_norm;
		q2_ *= quat_inv_norm;
		q3_ *= quat_inv_norm;
	}
	angles_computed_ = false;
}

void MahonyAttitudeEstimator::UpdateImu(float gx, float gy, float gz,
					float ax, float ay, float az)
{
	const float accel_inv_norm = InvSqrt(ax * ax + ay * ay + az * az);
	if (accel_inv_norm != 0.0f) {
		ax *= accel_inv_norm;
		ay *= accel_inv_norm;
		az *= accel_inv_norm;

		const float half_vx = q1_ * q3_ - q0_ * q2_;
		const float half_vy = q0_ * q1_ + q2_ * q3_;
		const float half_vz = q0_ * q0_ - 0.5f + q3_ * q3_;
		const float half_ex = ay * half_vz - az * half_vy;
		const float half_ey = az * half_vx - ax * half_vz;
		const float half_ez = ax * half_vy - ay * half_vx;

		if (two_ki_ > 0.0f) {
			integral_fbx_ += two_ki_ * half_ex * inv_sample_freq_;
			integral_fby_ += two_ki_ * half_ey * inv_sample_freq_;
			integral_fbz_ += two_ki_ * half_ez * inv_sample_freq_;
			gx += integral_fbx_;
			gy += integral_fby_;
			gz += integral_fbz_;
		} else {
			integral_fbx_ = 0.0f;
			integral_fby_ = 0.0f;
			integral_fbz_ = 0.0f;
		}

		gx += kTwoKp * half_ex;
		gy += kTwoKp * half_ey;
		gz += kTwoKp * half_ez;
	}

	const float half_dt = 0.5f * inv_sample_freq_;
	gx *= half_dt;
	gy *= half_dt;
	gz *= half_dt;
	const float qa = q0_;
	const float qb = q1_;
	const float qc = q2_;
	q0_ += -qb * gx - qc * gy - q3_ * gz;
	q1_ += qa * gx + qc * gz - q3_ * gy;
	q2_ += qa * gy - qb * gz + q3_ * gx;
	q3_ += qa * gz + qb * gy - qc * gx;

	const float quat_inv_norm = InvSqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
	if (quat_inv_norm != 0.0f) {
		q0_ *= quat_inv_norm;
		q1_ *= quat_inv_norm;
		q2_ *= quat_inv_norm;
		q3_ *= quat_inv_norm;
	}
	angles_computed_ = false;
}

void MahonyAttitudeEstimator::ComputeAngles()
{
	roll_deg_ = kRad2Deg * std::atan2(q0_ * q1_ + q2_ * q3_,
					  0.5f - q1_ * q1_ - q2_ * q2_);
	const float pitch_sine = std::clamp(-2.0f * (q1_ * q3_ - q0_ * q2_), -1.0f, 1.0f);
	pitch_deg_ = kRad2Deg * std::asin(pitch_sine);
	yaw_deg_ = kRad2Deg * std::atan2(q1_ * q2_ + q0_ * q3_,
				       0.5f - q2_ * q2_ - q3_ * q3_);
	angles_computed_ = true;
}

float MahonyAttitudeEstimator::RollDeg()
{
	if (!angles_computed_) {
		ComputeAngles();
	}
	return roll_deg_;
}

float MahonyAttitudeEstimator::PitchDeg()
{
	if (!angles_computed_) {
		ComputeAngles();
	}
	return pitch_deg_;
}

float MahonyAttitudeEstimator::YawDeg()
{
	if (!angles_computed_) {
		ComputeAngles();
	}
	return yaw_deg_;
}

} // namespace modules
