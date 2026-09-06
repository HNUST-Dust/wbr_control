#include <modules/ahrs/quaternion_ekf.h>
#include <tf_tree.h>

#include <cmath>
#include <limits>

#include <zephyr/ztest.h>

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kGravity = 9.80665f;

void ExpectNear(const char *name, float actual, float expected, float tolerance)
{
	zassert_true(std::isfinite(actual), "%s is not finite", name);
	zassert_within(actual, expected, tolerance, "%s", name);
}

void ExpectTrue(const char *name, bool value)
{
	zassert_true(value, "%s", name);
}

template <typename Quaternion>
float QuaternionNorm(const Quaternion &q)
{
	return std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

void InitEkf(modules::QuaternionEkf &ekf)
{
	ekf.Init({
		.process_noise_quat_ = 10.0f,
		.process_noise_gyro_bias_ = 1.0e-3f,
		.measure_noise_accel_ = 1.0e7f,
		.fading_lambda_ = 1.0f,
		.dt_ = 0.001f,
		.accel_lpf_coef_ = 0.0f,
	});
}

void GravityForEuler(float roll_deg, float pitch_deg, float accel[3])
{
	const float roll = roll_deg * kPi / 180.0f;
	const float pitch = pitch_deg * kPi / 180.0f;
	accel[0] = -std::sin(pitch) * kGravity;
	accel[1] = std::sin(roll) * std::cos(pitch) * kGravity;
	accel[2] = std::cos(roll) * std::cos(pitch) * kGravity;
}

ZTEST(onboard_imu_filter, tilted_initialization)
{
	float accel[3];
	GravityForEuler(30.0f, -20.0f, accel);
	modules::QuaternionEkf ekf;
	InitEkf(ekf);
	ExpectTrue("ekf tilted init", ekf.InitFromAccel(accel[0], accel[1], accel[2]));
	ExpectNear("ekf init roll", ekf.RollDeg(), 30.0f, 0.05f);
	ExpectNear("ekf init pitch", ekf.PitchDeg(), -20.0f, 0.05f);
	ExpectTrue("ekf init healthy", ekf.Healthy());
}

ZTEST(onboard_imu_filter, tf_tree)
{
	using Tree = wbr_control::TfTree;
	alignas(16) static constexpr float kChassisFromImu[4] = {0.0f, 0.0f,
								 1.0f, 0.0f};
	const Eigen::Map<const Tree::Rotation> chassis_from_imu(kChassisFromImu);
	Tree tree;
	tree.ConfigureChassisBranch(Tree::Rotation(chassis_from_imu));

	const Tree::Vector3 sensor(1.0f, 2.0f, 3.0f);
	Tree::Vector3 body;
	ExpectTrue("TF chassis branch available", tree.TransformVector(
		Tree::Frame::kChassis, Tree::Frame::kChassisImu, sensor, body));
	ExpectNear("TF IMU->chassis x", body.x(), -1.0f, 1.0e-6f);
	ExpectNear("TF IMU->chassis y", body.y(), -2.0f, 1.0e-6f);
	ExpectNear("TF IMU->chassis z", body.z(), 3.0f, 1.0e-6f);

	Tree::Vector3 recovered;
	ExpectTrue("TF chassis reverse available", tree.TransformVector(
		Tree::Frame::kChassisImu, Tree::Frame::kChassis, body, recovered));
	ExpectNear("TF chassis->IMU x", recovered.x(), sensor.x(), 1.0e-6f);
	ExpectNear("TF chassis->IMU y", recovered.y(), sensor.y(), 1.0e-6f);
	ExpectNear("TF chassis->IMU z", recovered.z(), sensor.z(), 1.0e-6f);

	Tree::Matrix3 identity_attitude;
	ExpectTrue("TF chassis attitude available", tree.TransformAttitude(
		Tree::Frame::kChassis, Tree::Frame::kChassisImu,
		Tree::Matrix3::Identity(), identity_attitude));
	ExpectNear("TF identity attitude xx", identity_attitude(0, 0), 1.0f, 1.0e-6f);
	ExpectNear("TF identity attitude yy", identity_attitude(1, 1), 1.0f, 1.0e-6f);
	ExpectNear("TF identity attitude zz", identity_attitude(2, 2), 1.0f, 1.0e-6f);

	constexpr float kTiltRad = 30.0f * kPi / 180.0f;
	Tree::Matrix3 positive_imu_roll;
	positive_imu_roll << 1.0f, 0.0f, 0.0f,
		0.0f, std::cos(kTiltRad), -std::sin(kTiltRad),
		0.0f, std::sin(kTiltRad), std::cos(kTiltRad);
	Tree::Matrix3 chassis_attitude;
	ExpectTrue("TF chassis tilted attitude available", tree.TransformAttitude(
		Tree::Frame::kChassis, Tree::Frame::kChassisImu,
		positive_imu_roll, chassis_attitude));
	ExpectNear("TF post-EKF chassis roll sine", chassis_attitude(2, 1),
		   -std::sin(kTiltRad), 1.0e-6f);
	ExpectNear("TF post-EKF chassis roll cosine", chassis_attitude(2, 2),
		   std::cos(kTiltRad), 1.0e-6f);

	Tree::Matrix3 positive_imu_pitch;
	positive_imu_pitch << std::cos(kTiltRad), 0.0f, std::sin(kTiltRad),
		0.0f, 1.0f, 0.0f,
		-std::sin(kTiltRad), 0.0f, std::cos(kTiltRad);
	Tree::Matrix3 chassis_pitch_attitude;
	ExpectTrue("TF chassis pitch attitude available", tree.TransformAttitude(
		Tree::Frame::kChassis, Tree::Frame::kChassisImu,
		positive_imu_pitch, chassis_pitch_attitude));
	ExpectNear("TF post-EKF chassis pitch sine", chassis_pitch_attitude(2, 0),
		   std::sin(kTiltRad), 1.0e-6f);

	Tree::Vector3 disconnected_output(9.0f, 8.0f, 7.0f);
	ExpectTrue("TF unconfigured gimbal branch rejected", !tree.TransformVector(
		Tree::Frame::kPcLink, Tree::Frame::kGimbalImu, sensor,
		disconnected_output));
	ExpectTrue("TF chassis-to-pc_link cross branch rejected", !tree.TransformVector(
		Tree::Frame::kPcLink, Tree::Frame::kChassis, sensor,
		disconnected_output));
	ExpectNear("TF rejected output remains unchanged", disconnected_output.x(),
		   9.0f, 0.0f);

	tree.ConfigureGimbalBranch(Tree::Rotation(), Tree::Rotation(chassis_from_imu));
	Tree::Vector3 pc_link_vector;
	ExpectTrue("TF reserved gimbal-to-pc_link chain available", tree.TransformVector(
		Tree::Frame::kPcLink, Tree::Frame::kGimbalImu, sensor,
		pc_link_vector));
	ExpectNear("TF pc_link x", pc_link_vector.x(), -1.0f, 1.0e-6f);
	ExpectNear("TF pc_link y", pc_link_vector.y(), -2.0f, 1.0e-6f);
	ExpectNear("TF pc_link z", pc_link_vector.z(), 3.0f, 1.0e-6f);
	ExpectTrue("TF cross branch remains rejected", !tree.TransformVector(
		Tree::Frame::kPcLink, Tree::Frame::kChassis, sensor,
		disconnected_output));
}

ZTEST(onboard_imu_filter, variable_dt_yaw_integration)
{
	modules::QuaternionEkf ekf;
	InitEkf(ekf);
	ExpectTrue("ekf flat init", ekf.InitFromAccel(0.0f, 0.0f, kGravity));

	const float yaw_rate = 0.5f * kPi;
	float elapsed = 0.0f;
	for (int i = 0; i < 100; ++i) {
		const float dt = (i % 2 == 0) ? 0.004f : 0.016f;
		elapsed += dt;
		ekf.Update(0.0f, 0.0f, yaw_rate, 0.0f, 0.0f, kGravity, dt);
	}
	ExpectNear("variable dt elapsed", elapsed, 1.0f, 1.0e-5f);
	ExpectNear("ekf variable dt yaw", ekf.YawDeg(), 90.0f, 0.3f);
	ExpectTrue("ekf variable dt healthy", ekf.Healthy());
}

ZTEST(onboard_imu_filter, one_khz_yaw_integration)
{
	modules::QuaternionEkf ekf;
	InitEkf(ekf);
	ExpectTrue("ekf 1 kHz init", ekf.InitFromAccel(0.0f, 0.0f, kGravity));

	const float yaw_rate = 0.5f * kPi;
	for (int i = 0; i < 1000; ++i) {
		ekf.Update(0.0f, 0.0f, yaw_rate, 0.0f, 0.0f, kGravity, 0.001f);
	}
	ExpectNear("ekf 1 kHz yaw", ekf.YawDeg(), 90.0f, 0.2f);
	ExpectTrue("ekf 1 kHz healthy", ekf.Healthy());
}

ZTEST(onboard_imu_filter, dynamic_acceleration_rejection)
{
	modules::QuaternionEkf ekf;
	InitEkf(ekf);
	ekf.InitFromAccel(0.0f, 0.0f, kGravity);
	for (int i = 0; i < 200; ++i) {
		ekf.Update(0.0f, 0.0f, 0.0f, 10.0f, 0.0f, kGravity, 0.005f);
	}
	ExpectNear("ekf dynamic accel pitch", ekf.PitchDeg(), 0.0f, 0.5f);
}

ZTEST(onboard_imu_filter, long_running_irregular_integration)
{
	modules::QuaternionEkf ekf;
	InitEkf(ekf);
	ExpectTrue("ekf long init", ekf.InitFromAccel(0.0f, 0.0f, kGravity));

	const float yaw_rate = 0.25f * kPi;
	float elapsed = 0.0f;
	for (int i = 0; i < 4000; ++i) {
		const float dt_pattern[] = {0.003f, 0.005f, 0.007f, 0.005f};
		const float dt = dt_pattern[i % 4];
		elapsed += dt;
		ekf.Update(0.0f, 0.0f, yaw_rate, 0.0f, 0.0f, kGravity, dt);
	}
	const float expected_total_yaw = elapsed * yaw_rate * 180.0f / kPi;
	ExpectNear("long elapsed", elapsed, 20.0f, 0.002f);
	ExpectNear("ekf continuous yaw", ekf.YawTotalDeg(), expected_total_yaw, 0.5f);
	ExpectNear("ekf long quaternion norm", QuaternionNorm(ekf.Quat()), 1.0f,
		   1.0e-4f);
	ExpectTrue("ekf long healthy", ekf.Healthy());
}

ZTEST(onboard_imu_filter, invalid_interval_is_ignored)
{
	modules::QuaternionEkf ekf;
	InitEkf(ekf);
	ekf.InitFromAccel(0.0f, 0.0f, kGravity);
	ekf.Update(0.0f, 0.0f, kPi, 0.0f, 0.0f, kGravity, -0.1f);
	ExpectNear("ekf invalid dt ignored", ekf.YawDeg(), 0.0f, 1.0e-5f);
}

ZTEST(onboard_imu_filter, health_detection)
{
	modules::QuaternionEkf ekf;
	InitEkf(ekf);
	ekf.InitFromAccel(0.0f, 0.0f, kGravity);
	const float nan = std::numeric_limits<float>::quiet_NaN();
	ekf.Update(nan, 0.0f, 0.0f, 0.0f, 0.0f, kGravity, 0.005f);
	ExpectTrue("ekf detects NaN", !ekf.Healthy());
}

}  // namespace

ZTEST_SUITE(onboard_imu_filter, nullptr, nullptr, nullptr, nullptr, nullptr);
