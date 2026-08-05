#include <algorithms/MahonyAHRS.h>

#include <cmath>
#include <cstdint>
#include <cstring>

namespace alg {

void MahonyAhrs::Init(float sampleFrequencyHz)
{
	two_ki_ = kTwoKiDefault;
	q0_ = 1.0f;
	q1_ = 0.0f;
	q2_ = 0.0f;
	q3_ = 0.0f;
	integral_fbx_ = 0.0f;
	integral_fby_ = 0.0f;
	integral_fbz_ = 0.0f;
	angles_computed_ = false;
	inv_sample_freq_ = 1.0f / sampleFrequencyHz;
}

float MahonyAhrs::InvSqrt(float x)
{
	float half_x = 0.5f * x;
	float y = x;

	static_assert(sizeof(float) == sizeof(std::uint32_t));
	std::uint32_t i = 0;
	std::memcpy(&i, &y, sizeof(i));
	i = 0x5f3759dfU - (i >> 1);
	std::memcpy(&y, &i, sizeof(y));

	y = y * (1.5f - (half_x * y * y));
	y = y * (1.5f - (half_x * y * y));
	return y;
}

void MahonyAhrs::InitFromAccMag(float ax, float ay, float az, float mx, float my, float mz)
{
	float recip_norm;
	float init_yaw, init_pitch, init_roll;
	float cr2, cp2, cy2, sr2, sp2, sy2;
	float sin_roll, cos_roll, sin_pitch, cos_pitch;
	float mag_x, mag_y;

	recip_norm = InvSqrt(ax * ax + ay * ay + az * az);
	ax *= recip_norm;
	ay *= recip_norm;
	az *= recip_norm;

	if ((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f))
	{
		recip_norm = InvSqrt(mx * mx + my * my + mz * mz);
		mx *= recip_norm;
		my *= recip_norm;
		mz *= recip_norm;
	}

	init_pitch = std::atan2(-ax, az);
	init_roll = std::atan2(ay, az);

	    sin_roll = std::sin(init_roll);
	    cos_roll = std::cos(init_roll);
	    cos_pitch = std::cos(init_pitch);
	    sin_pitch = std::sin(init_pitch);

	if ((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f))
	{
		mag_x = mx * cos_pitch + my * sin_pitch * sin_roll + mz * sin_pitch * cos_roll;
		mag_y = my * cos_roll - mz * sin_roll;
		init_yaw = std::atan2(-mag_y, mag_x);
	}
	else
	{
		init_yaw = 0.0f;
	}

	cr2 = std::cos(init_roll * 0.5f);
	cp2 = std::cos(init_pitch * 0.5f);
	cy2 = std::cos(init_yaw * 0.5f);
	sr2 = std::sin(init_roll * 0.5f);
	sp2 = std::sin(init_pitch * 0.5f);
	sy2 = std::sin(init_yaw * 0.5f);

	q0_ = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
	q1_ = sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
	q2_ = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
	q3_ = cr2 * cp2 * sy2 - sr2 * sp2 * cy2;

	recip_norm = InvSqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
	q0_ *= recip_norm;
	q1_ *= recip_norm;
	q2_ *= recip_norm;
	q3_ *= recip_norm;

	angles_computed_ = false;
}

void MahonyAhrs::Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	float recip_norm;
	float q0_q0, q0_q1, q0_q2, q0_q3, q1_q1, q1_q2, q1_q3, q2_q2, q2_q3, q3_q3;
	float h_x, h_y, b_x, b_z;
	float half_vx, half_vy, half_vz, half_wx, half_wy, half_wz;
	float half_ex, half_ey, half_ez;
	float q_a, q_b, q_c;

	if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
	{
		UpdateImu(gx, gy, gz, ax, ay, az);
		return;
	}

	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{
		recip_norm = InvSqrt(ax * ax + ay * ay + az * az);
		ax *= recip_norm;
		ay *= recip_norm;
		az *= recip_norm;

		recip_norm = InvSqrt(mx * mx + my * my + mz * mz);
		mx *= recip_norm;
		my *= recip_norm;
		mz *= recip_norm;

		q0_q0 = q0_ * q0_;
		q0_q1 = q0_ * q1_;
		q0_q2 = q0_ * q2_;
		q0_q3 = q0_ * q3_;
		q1_q1 = q1_ * q1_;
		q1_q2 = q1_ * q2_;
		q1_q3 = q1_ * q3_;
		q2_q2 = q2_ * q2_;
		q2_q3 = q2_ * q3_;
		q3_q3 = q3_ * q3_;

		h_x = 2.0f * (mx * (0.5f - q2_q2 - q3_q3) + my * (q1_q2 - q0_q3) + mz * (q1_q3 + q0_q2));
		h_y = 2.0f * (mx * (q1_q2 + q0_q3) + my * (0.5f - q1_q1 - q3_q3) + mz * (q2_q3 - q0_q1));
		b_x = std::sqrt(h_x * h_x + h_y * h_y);
		b_z = 2.0f * (mx * (q1_q3 - q0_q2) + my * (q2_q3 + q0_q1) + mz * (0.5f - q1_q1 - q2_q2));

		half_vx = q1_q3 - q0_q2;
		half_vy = q0_q1 + q2_q3;
		half_vz = q0_q0 - 0.5f + q3_q3;
		half_wx = b_x * (0.5f - q2_q2 - q3_q3) + b_z * (q1_q3 - q0_q2);
		half_wy = b_x * (q1_q2 - q0_q3) + b_z * (q0_q1 + q2_q3);
		half_wz = b_x * (q0_q2 + q1_q3) + b_z * (0.5f - q1_q1 - q2_q2);

		half_ex = (ay * half_vz - az * half_vy) + (my * half_wz - mz * half_wy);
		half_ey = (az * half_vx - ax * half_vz) + (mz * half_wx - mx * half_wz);
		half_ez = (ax * half_vy - ay * half_vx) + (mx * half_wy - my * half_wx);

		if (two_ki_ > 0.0f)
		{
			integral_fbx_ += two_ki_ * half_ex * inv_sample_freq_;
			integral_fby_ += two_ki_ * half_ey * inv_sample_freq_;
			integral_fbz_ += two_ki_ * half_ez * inv_sample_freq_;
			gx += integral_fbx_;
			gy += integral_fby_;
			gz += integral_fbz_;
		}
		else
		{
			integral_fbx_ = 0.0f;
			integral_fby_ = 0.0f;
			integral_fbz_ = 0.0f;
		}

		gx += kTwoKp * half_ex;
		gy += kTwoKp * half_ey;
		gz += kTwoKp * half_ez;
	}

	gx *= (0.5f * inv_sample_freq_);
	gy *= (0.5f * inv_sample_freq_);
	gz *= (0.5f * inv_sample_freq_);
	q_a = q0_;
	q_b = q1_;
	q_c = q2_;
	q0_ += (-q_b * gx - q_c * gy - q3_ * gz);
	q1_ += (q_a * gx + q_c * gz - q3_ * gy);
	q2_ += (q_a * gy - q_b * gz + q3_ * gx);
	q3_ += (q_a * gz + q_b * gy - q_c * gx);

	recip_norm = InvSqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
	q0_ *= recip_norm;
	q1_ *= recip_norm;
	q2_ *= recip_norm;
	q3_ *= recip_norm;
	angles_computed_ = false;
}

void MahonyAhrs::UpdateImu(float gx, float gy, float gz, float ax, float ay, float az)
{
	float recip_norm;
	float half_vx, half_vy, half_vz;
	float half_ex, half_ey, half_ez;
	float q_a, q_b, q_c;

	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{
		recip_norm = InvSqrt(ax * ax + ay * ay + az * az);
		ax *= recip_norm;
		ay *= recip_norm;
		az *= recip_norm;

		half_vx = q1_ * q3_ - q0_ * q2_;
		half_vy = q0_ * q1_ + q2_ * q3_;
		half_vz = q0_ * q0_ - 0.5f + q3_ * q3_;

		half_ex = (ay * half_vz - az * half_vy);
		half_ey = (az * half_vx - ax * half_vz);
		half_ez = (ax * half_vy - ay * half_vx);

		if (two_ki_ > 0.0f)
		{
			integral_fbx_ += two_ki_ * half_ex * inv_sample_freq_;
			integral_fby_ += two_ki_ * half_ey * inv_sample_freq_;
			integral_fbz_ += two_ki_ * half_ez * inv_sample_freq_;
			gx += integral_fbx_;
			gy += integral_fby_;
			gz += integral_fbz_;
		}
		else
		{
			integral_fbx_ = 0.0f;
			integral_fby_ = 0.0f;
			integral_fbz_ = 0.0f;
		}

		gx += kTwoKp * half_ex;
		gy += kTwoKp * half_ey;
		gz += kTwoKp * half_ez;
	}

	gx *= (0.5f * inv_sample_freq_);
	gy *= (0.5f * inv_sample_freq_);
	gz *= (0.5f * inv_sample_freq_);
	q_a = q0_;
	q_b = q1_;
	q_c = q2_;
	q0_ += (-q_b * gx - q_c * gy - q3_ * gz);
	q1_ += (q_a * gx + q_c * gz - q3_ * gy);
	q2_ += (q_a * gy - q_b * gz + q3_ * gx);
	q3_ += (q_a * gz + q_b * gy - q_c * gx);

	recip_norm = InvSqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
	q0_ *= recip_norm;
	q1_ *= recip_norm;
	q2_ *= recip_norm;
	q3_ *= recip_norm;
	angles_computed_ = false;
}

void MahonyAhrs::ComputeAngles()
{
	roll_deg_ = std::atan2(q0_ * q1_ + q2_ * q3_, 0.5f - q1_ * q1_ - q2_ * q2_) * kRad2Deg;
	pitch_deg_ = std::asin(-2.0f * (q1_ * q3_ - q0_ * q2_)) * kRad2Deg;
	yaw_deg_ = std::atan2(q1_ * q2_ + q0_ * q3_, 0.5f - q2_ * q2_ - q3_ * q3_) * kRad2Deg;
	angles_computed_ = true;
}

float MahonyAhrs::RollDeg()
{
	if (!angles_computed_)
	{
		ComputeAngles();
	}
	return roll_deg_;
}

float MahonyAhrs::PitchDeg()
{
	if (!angles_computed_)
	{
		ComputeAngles();
	}
	return pitch_deg_;
}

float MahonyAhrs::YawDeg()
{
	if (!angles_computed_)
	{
		ComputeAngles();
	}
	return yaw_deg_;
}

} // namespace alg
