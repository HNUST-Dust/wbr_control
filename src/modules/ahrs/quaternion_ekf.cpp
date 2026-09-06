/**
 * @file quaternion_ekf.cpp
 * @ingroup wbr_modules
 * @brief 实现六状态四元数 EKF 的预测、重力观测校正和姿态输出。
 * @details 算法以陀螺仪角速度积分四元数，只在准静止条件下使用加速度重力方向；
 *          创新门控拒绝不可信观测，Joseph 形式更新与切空间投影用于保持协方差稳定。
 */

#include <modules/ahrs/quaternion_ekf.h>

#include <Eigen/Core>
#include <Eigen/LU>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace {
/**
 * @brief 安全计算平方根倒数。
 * @param x 待开方的非负标量。
 * @return x 有效且大于阈值时返回 `1/sqrt(x)`，否则返回 0。
 */
float InvSqrt(float x)
{
	return (std::isfinite(x) && x > 1.0e-12f) ? (1.0f / std::sqrt(x)) : 0.0f;
}

// 初始协方差 P0，按行优先存储（与论文/旧实现一致）。
constexpr float kInitialCovarianceData[36] = {
	100000.0f, 0, 0, 0, 0, 0,
	0, 100000.0f, 0, 0, 0, 0,
	0, 0, 100000.0f, 0, 0, 0,
	0, 0, 0, 100000.0f, 0, 0,
	0, 0, 0, 0, 100.0f, 0,
	0, 0, 0, 0, 0, 100.0f,
};

constexpr float kGravityMps2 = 9.80665f;
constexpr float kStableAccelToleranceMps2 = 0.5f;
constexpr float kStableGyroLimitRadS = 0.3f;

// 将行优先存储的初始协方差常量复制为 Eigen 列优先矩阵。
/**
 * @brief 将行优先常量转换为 Eigen 使用的初始协方差矩阵。
 * @return 六状态 EKF 的初始协方差 P0。
 */
Eigen::Matrix<float, 6, 6> MakeInitialCovariance()
{
    Eigen::Matrix<float, 6, 6> p;
    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 6; ++c)
        {
            p(r, c) = kInitialCovarianceData[r * 6 + c];
        }
    }
    return p;
}

} // namespace

/** @brief 销毁固定尺寸 EKF 状态；对象不持有动态资源。 */
modules::QuaternionEkf::~QuaternionEkf()
{
}

/** @brief 将内部向量、矩阵和单周期控制标志清零。 */
void modules::QuaternionEkf::ResetFilterState()
{
	skip_eq5_ = false;
	measurement_valid_num_ = 0U;
	filtered_value_.setZero();
	measured_vector_.setZero();
	xhat_.setZero();
	xhatminus_.setZero();
	z_.setZero();
	p_.setZero();
	pminus_.setZero();
	f_.setZero();
	h_.setZero();
	process_noise_.setZero();
	measurement_noise_.setZero();
	kalman_gain_.setZero();
	innovation_covariance_.setZero();
}

/**
 * @brief 执行一次完整 EKF 内部更新流水线。
 * @details 顺序为锁存观测、状态预测、协方差预测、构造 H、门控校正、Joseph
 *          协方差更新、四元数切空间投影和输出锁存。无有效加速度观测时仅预测。
 */
void modules::QuaternionEkf::RunFilterUpdate()
{
	z_ = measured_vector_;
	measured_vector_.setZero();
	xhatminus_ = f_ * xhat_;
	ApplyPredictionConstraints();
	pminus_ = f_ * p_ * f_.transpose() + process_noise_;
	BuildObservationMatrix();

	if (measurement_valid_num_ != 0U) {
		ApplyMeasurementCorrection();
		if (!skip_eq5_) {
			const MatX residual = MatX::Identity() - kalman_gain_ * h_;
			p_ = residual * pminus_ * residual.transpose() +
			     kalman_gain_ * measurement_noise_ * kalman_gain_.transpose();
			p_ = 0.5f * (p_ + p_.transpose()).eval();
		}
	} else {
		xhat_ = xhatminus_;
		p_ = pminus_;
	}
	NormalizeCovariance();
	filtered_value_ = xhat_;
}

/**
 * @brief 对预测四元数及其线性化模型施加单位模和零偏协方差约束。
 * @details 归一化先验四元数，更新 F 中四元数对 X/Y 零偏的 4x2 耦合块，
 *          将四元数过程噪声投影到单位球切空间，并限制零偏方差避免发散。
 */
void modules::QuaternionEkf::ApplyPredictionConstraints()
{
	auto &kf = *this;
	auto &ins = ins_;
	const float q_inv_norm = InvSqrt(kf.xhatminus().template head<4>().squaredNorm());
	Eigen::Matrix<float, 4, 4> normalization_jacobian =
		Eigen::Matrix<float, 4, 4>::Identity();
	if (q_inv_norm == 0.0f) {
		kf.xhatminus().setZero();
		kf.xhatminus()(0) = 1.0f;
	} else {
		kf.xhatminus().template head<4>() *= q_inv_norm;
		const Eigen::Matrix<float, 4, 1> normalized_q =
			kf.xhatminus().template head<4>();
		normalization_jacobian =
			q_inv_norm * (Eigen::Matrix<float, 4, 4>::Identity() -
				      normalized_q * normalized_q.transpose());
	}
	const float q0 = kf.xhatminus()(0);
	const float q1 = kf.xhatminus()(1);
	const float q2 = kf.xhatminus()(2);
	const float q3 = kf.xhatminus()(3);

	// quaternion normalize

    // F 右上角 4x2 分块（四元数对零漂的耦合）。
    kf.F()(0, 4) = (q1 * ins.dt) * 0.5f;
    kf.F()(0, 5) = q2 * ins.dt * 0.5f;

    kf.F()(1, 4) = -q0 * ins.dt * 0.5f;
    kf.F()(1, 5) = q3 * ins.dt * 0.5f;

    kf.F()(2, 4) = -q3 * ins.dt * 0.5f;
    kf.F()(2, 5) = -q0 * ins.dt * 0.5f;

    kf.F()(3, 4) = q2 * ins.dt * 0.5f;
    kf.F()(3, 5) = -q1 * ins.dt * 0.5f;

	/* xhatminus is normalized after the linear prediction. Apply the same
	 * Jacobian to the covariance transition and keep quaternion process noise
	 * in the three-dimensional tangent space instead of injecting an
	 * unobservable radial variance.
	 */
	kf.F().template topRows<4>() =
		(normalization_jacobian * kf.F().template topRows<4>()).eval();
	const Eigen::Matrix<float, 4, 1> normalized_q =
		kf.xhatminus().template head<4>();
	kf.Q().template topLeftCorner<4, 4>() =
		(ins.q1 * ins.dt) *
		(Eigen::Matrix<float, 4, 4>::Identity() -
		 normalized_q * normalized_q.transpose());

    // fading filter,防止零飘参数过度收敛
    kf.P()(4, 4) *= ins.lambda;
    kf.P()(5, 5) *= ins.lambda;

    // 限幅,防止发散
    if (kf.P()(4, 4) > 10000)
    {
        kf.P()(4, 4) = 10000;
    }
    if (kf.P()(5, 5) > 10000)
    {
        kf.P()(5, 5) = 10000;
    }
}

/**
 * @brief 在当前先验四元数处计算重力方向观测函数的 Jacobian H。
 */
void modules::QuaternionEkf::BuildObservationMatrix()
{
	auto &kf = *this;
    volatile float double_q0, double_q1, double_q2, double_q3;

    double_q0 = 2 * kf.xhatminus()(0);
    double_q1 = 2 * kf.xhatminus()(1);
    double_q2 = 2 * kf.xhatminus()(2);
    double_q3 = 2 * kf.xhatminus()(3);

    kf.H().setZero();

    kf.H()(0, 0) = -double_q2;
    kf.H()(0, 1) = double_q3;
    kf.H()(0, 2) = -double_q0;
    kf.H()(0, 3) = double_q1;

    kf.H()(1, 0) = double_q1;
    kf.H()(1, 1) = double_q0;
    kf.H()(1, 2) = double_q3;
    kf.H()(1, 3) = double_q2;

    kf.H()(2, 0) = double_q0;
    kf.H()(2, 1) = -double_q1;
    kf.H()(2, 2) = -double_q2;
    kf.H()(2, 3) = double_q3;
}

/**
 * @brief 根据归一化重力观测计算后验状态校正。
 * @details 计算创新及其卡方统计量；超过硬门限时保留纯预测结果，位于软门限
 *          区间时按比例衰减 Kalman 增益。X/Y 零偏单周期校正被限幅，加速度
 *          不直接修正 yaw 对应的四元数分量。
 */
void modules::QuaternionEkf::ApplyMeasurementCorrection()
{
	auto &kf = *this;
	auto &ins = ins_;
	const float q0 = kf.xhatminus()(0);
	const float q1 = kf.xhatminus()(1);
	const float q2 = kf.xhatminus()(2);
	const float q3 = kf.xhatminus()(3);
	kf.SetSkipEq5(false);

    // 创新协方差 S = H P^- H^T + R，及其逆。
    kf.S() = kf.H() * kf.Pminus() * kf.H().transpose() + kf.R();
	const Eigen::Matrix<float, 3, 3> s_inv = kf.S().inverse();
	if (!s_inv.allFinite()) {
		kf.xhat() = kf.xhatminus();
		kf.P() = kf.Pminus();
		kf.SetSkipEq5(true);
		ins.converge_flag = 0;
		return;
	}

    // 计算预测得到的重力加速度方向(通过姿态获取的)
    Eigen::Matrix<float, 3, 1> h_pred;
    h_pred(0) = 2 * (q1 * q3 - q0 * q2);
    h_pred(1) = 2 * (q0 * q1 + q2 * q3);
    h_pred(2) = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 利用加速度计数据修正：创新 = z - h(x^-)
    const Eigen::Matrix<float, 3, 1> innovation = kf.z() - h_pred;

    // chi-square test,卡方检验：chi_square = innovation^T S^-1 innovation
    ins.chi_square = innovation.dot(s_inv * innovation);
	if (!std::isfinite(ins.chi_square) ||
	    ins.chi_square > ins.chi_square_test_threshold) {
		kf.xhat() = kf.xhatminus();
		kf.P() = kf.Pminus();
		kf.SetSkipEq5(true);
		return;
	}
	ins.converge_flag = 1;
	if (ins.chi_square > ins.chi_square_soft_threshold) {
		ins.adaptive_gain_scale =
			(ins.chi_square_test_threshold - ins.chi_square) /
			(ins.chi_square_test_threshold - ins.chi_square_soft_threshold);
	} else {
		ins.adaptive_gain_scale = 1.0f;
	}

    // cal kf-gain K = P^- H^T S^-1
    kf.K() = kf.Pminus() * kf.H().transpose() * s_inv;

    // implement adaptive
    kf.K() *= ins.adaptive_gain_scale;

    // 后验估计：xhat = xhatminus + K * innovation
    Eigen::Matrix<float, 6, 1> correction = kf.K() * innovation;

    // 零漂修正限幅,一般不会有过大的漂移
    if (ins.converge_flag)
    {
        for (uint8_t i = 4; i < 6; i++)
        {
            if (correction(i) > 1e-2f * ins.dt)
            {
                correction(i) = 1e-2f * ins.dt;
            }
            if (correction(i) < -1e-2f * ins.dt)
            {
                correction(i) = -1e-2f * ins.dt;
            }
        }
    }

	/* Gravity contains no heading observation.  Retain the field-proven EKF
	 * rule that prevents the accelerometer correction from directly changing
	 * the yaw quaternion component; yaw is propagated by the calibrated gyro.
	 */
	correction(3) = 0.0f;

	kf.xhat() = kf.xhatminus() + correction;
	const float quat_inv_norm = InvSqrt(kf.xhat().template head<4>().squaredNorm());
	if (quat_inv_norm == 0.0f || !kf.xhat().allFinite()) {
		kf.xhat() = kf.xhatminus();
		kf.P() = kf.Pminus();
		kf.SetSkipEq5(true);
		ins.converge_flag = 0;
		return;
	}
	kf.xhat().template head<4>() *= quat_inv_norm;
}

/**
 * @brief 将四元数协方差投影到单位四元数切空间。
 * @details 同步投影四元数—零偏互协方差，并在最后恢复矩阵对称性。
 */
void modules::QuaternionEkf::NormalizeCovariance()
{
	auto &kf = *this;
	const Eigen::Matrix<float, 4, 1> q = kf.xhat().template head<4>();
	if (!q.allFinite()) {
		return;
	}
	const Eigen::Matrix<float, 4, 4> tangent =
		Eigen::Matrix<float, 4, 4>::Identity() - q * q.transpose();
	const Eigen::Matrix<float, 4, 2> quat_bias =
		tangent * kf.P().template block<4, 2>(0, 4);
	kf.P().template topLeftCorner<4, 4>() =
		(tangent * kf.P().template topLeftCorner<4, 4>() * tangent).eval();
	kf.P().template block<4, 2>(0, 4) = quat_bias;
	kf.P().template block<2, 4>(4, 0) = quat_bias.transpose();
	kf.P() = 0.5f * (kf.P() + kf.P().transpose()).eval();
}

namespace modules {

/**
 * @brief 设置参数并初始化滤波状态、姿态和协方差。
 * @param params EKF 噪声、衰减、周期及加速度低通参数。
 */
void QuaternionEkf::Init(const Params &params)
{
	ResetFilterState();

	ins_.q1 = std::max(0.0f, params.process_noise_quat_);
	ins_.q2 = std::max(0.0f, params.process_noise_gyro_bias_);
	ins_.r = std::max(1.0e-6f, params.measure_noise_accel_);
	/* Keep the innovation gate paired with the proven 1e7 accelerometer noise
	 * tuning.  With S expressed in that scale, the legacy 1e-8 threshold is
	 * intentionally small and turns a direction mismatch into prediction-only
	 * operation instead of feeding linear acceleration into attitude.
	 */
	ins_.chi_square_test_threshold = 1.0e-8f;
	ins_.chi_square_soft_threshold = 1.0e-9f;
    ins_.converge_flag = 0;
    ins_.update_count = 0;
	ins_.dt = std::clamp(params.dt_, 0.0001f, 0.05f);
	ins_.acc_lpf_coef = std::clamp(params.accel_lpf_coef_, 0.0f, 1.0f);
    ins_.yaw_round_count = 0;
    ins_.yaw_angle_last = 0.0f;
    ins_.yaw_total_angle = 0.0f;
	ins_.adaptive_gain_scale = 1.0f;
	ins_.chi_square = 0.0f;

	std::memset(ins_.q, 0, sizeof(ins_.q));
	ins_.q[0] = 1.0f;
	std::memset(ins_.gyro_bias, 0, sizeof(ins_.gyro_bias));
	std::memset(ins_.gyro, 0, sizeof(ins_.gyro));
	std::memset(ins_.accel, 0, sizeof(ins_.accel));

	ins_.lambda = std::max(1.0f, params.fading_lambda_);

    // 姿态初始化
    xhat()(0) = 1;
    xhat()(1) = 0;
    xhat()(2) = 0;
    xhat()(3) = 0;

    F().setIdentity();
    P() = MakeInitialCovariance();
}

/**
 * @brief 根据静止加速度方向初始化 roll/pitch，yaw 置零。
 * @param ax X 轴加速度，单位为 m/s²。
 * @param ay Y 轴加速度，单位为 m/s²。
 * @param az Z 轴加速度，单位为 m/s²。
 * @return 输入向量可归一化时返回 true，否则保持原状态并返回 false。
 */
bool QuaternionEkf::InitFromAccel(float ax, float ay, float az)
{
	const float inv_norm = InvSqrt(ax * ax + ay * ay + az * az);
	if (inv_norm == 0.0f) {
		return false;
	}
	ax *= inv_norm;
	ay *= inv_norm;
	az *= inv_norm;
	const float roll = std::atan2(ay, az);
	const float pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
	const float cr = std::cos(roll * 0.5f);
	const float sr = std::sin(roll * 0.5f);
	const float cp = std::cos(pitch * 0.5f);
	const float sp = std::sin(pitch * 0.5f);
	const float q[4] = {cr * cp, sr * cp, cr * sp, -sr * sp};
	for (std::size_t i = 0; i < 4U; ++i) {
		xhat()(i) = q[i];
		FilteredValue()(i) = q[i];
		ins_.q[i] = q[i];
	}
	/* The initial quaternion is already normalized, so its covariance must
	 * start in the same three-dimensional tangent space as every subsequent
	 * prediction/update.  Otherwise the first published state carries a
	 * purely artificial radial variance until Update() runs once.
	 */
	NormalizeCovariance();
	ins_.accel[0] = ax * 9.80665f;
	ins_.accel[1] = ay * 9.80665f;
	ins_.accel[2] = az * 9.80665f;
	ins_.update_count = 1U;
	ins_.roll = roll * 57.29578f;
	ins_.pitch = pitch * 57.29578f;
	ins_.yaw = 0.0f;
	ins_.yaw_angle_last = 0.0f;
	ins_.yaw_total_angle = 0.0f;
	return true;
}

/**
 * @brief 清空运行状态并恢复单位四元数和初始协方差。
 * @details Init() 设置的噪声、周期和门限参数保持不变。
 */
void QuaternionEkf::Reset()
{
	ResetFilterState();
    ins_.converge_flag = 0;
    ins_.update_count = 0;
    ins_.yaw_round_count = 0;
    ins_.yaw_angle_last = 0.0f;
    ins_.yaw_total_angle = 0.0f;
	ins_.adaptive_gain_scale = 1.0f;
	ins_.chi_square = 0.0f;
	std::memset(ins_.q, 0, sizeof(ins_.q));
	ins_.q[0] = 1.0f;
	std::memset(ins_.gyro_bias, 0, sizeof(ins_.gyro_bias));
	std::memset(ins_.gyro, 0, sizeof(ins_.gyro));
	std::memset(ins_.accel, 0, sizeof(ins_.accel));

    // 姿态初始化
    xhat()(0) = 1;
    xhat()(1) = 0;
    xhat()(2) = 0;
    xhat()(3) = 0;

    F().setIdentity();
    P() = MakeInitialCovariance();
}

/**
 * @brief 使用当前默认周期更新一次姿态。
 * @param gx X 轴角速度，单位为 rad/s。
 * @param gy Y 轴角速度，单位为 rad/s。
 * @param gz Z 轴角速度，单位为 rad/s。
 * @param ax X 轴加速度，单位为 m/s²。
 * @param ay Y 轴加速度，单位为 m/s²。
 * @param az Z 轴加速度，单位为 m/s²。
 */
void QuaternionEkf::Update(float gx, float gy, float gz, float ax, float ay, float az)
{
	Update(gx, gy, gz, ax, ay, az, ins_.dt);
}

/**
 * @brief 使用实测周期更新一次姿态和 X/Y 陀螺仪零偏估计。
 * @param gx X 轴角速度，单位为 rad/s。
 * @param gy Y 轴角速度，单位为 rad/s。
 * @param gz Z 轴角速度，单位为 rad/s。
 * @param ax X 轴加速度，单位为 m/s²。
 * @param ay Y 轴加速度，单位为 m/s²。
 * @param az Z 轴加速度，单位为 m/s²。
 * @param dt_seconds 当前样本周期，单位为秒。
 * @details 周期限制在 `[0.1 ms, 50 ms]`。只有角速度小于 0.3 rad/s 且
 *          加速度模长处于 `1g ± 0.5 m/s²` 时，才启用重力观测校正。
 */
void QuaternionEkf::Update(float gx, float gy, float gz, float ax, float ay, float az,
			   float dt_seconds)
{
	if (!std::isfinite(dt_seconds) || dt_seconds <= 0.0f) {
		return;
	}
	ins_.dt = std::clamp(dt_seconds, 0.0001f, 0.05f);
    // 0.5(Ohm-Ohm^bias)*deltaT,用于更新工作点处的状态转移F矩阵
    volatile float half_gx_dt, half_gy_dt, half_gz_dt;
    volatile float accel_inv_norm;

    ins_.gyro[0] = gx - ins_.gyro_bias[0];
    ins_.gyro[1] = gy - ins_.gyro_bias[1];
    ins_.gyro[2] = gz - ins_.gyro_bias[2];

    // set F：先复位为单位阵（右下角 2x2 单位阵对应零漂随机游走），再填四元数动力学左上角 4x4 分块。
    half_gx_dt = 0.5f * ins_.gyro[0] * ins_.dt;
    half_gy_dt = 0.5f * ins_.gyro[1] * ins_.dt;
    half_gz_dt = 0.5f * ins_.gyro[2] * ins_.dt;

    F().setIdentity();

    F()(0, 1) = -half_gx_dt;
    F()(0, 2) = -half_gy_dt;
    F()(0, 3) = -half_gz_dt;

    F()(1, 0) = half_gx_dt;
    F()(1, 2) = half_gz_dt;
    F()(1, 3) = -half_gy_dt;

    F()(2, 0) = half_gy_dt;
    F()(2, 1) = -half_gz_dt;
    F()(2, 3) = half_gx_dt;

    F()(3, 0) = half_gz_dt;
    F()(3, 1) = half_gy_dt;
    F()(3, 2) = -half_gx_dt;

	const float raw_accel_norm = std::sqrt(ax * ax + ay * ay + az * az);
	const float gyro_norm = std::sqrt(ins_.gyro[0] * ins_.gyro[0] +
					ins_.gyro[1] * ins_.gyro[1] +
					ins_.gyro[2] * ins_.gyro[2]);
	/* Preserve the proven BMI088 estimator's observation policy: gravity is
	 * allowed to correct the prediction only while the body is quasi-static.
	 * During translation or fast rotation the update remains gyro-only, so
	 * linear acceleration cannot be mistaken for tilt.
	 */
	const bool raw_accel_trusted = std::isfinite(raw_accel_norm) &&
		std::isfinite(gyro_norm) &&
		std::fabs(raw_accel_norm - kGravityMps2) <= kStableAccelToleranceMps2 &&
		gyro_norm < kStableGyroLimitRadS;

	// accel low pass filter,加速度过一下低通滤波平滑数据,降低撞击和异常的影响
	if (ins_.update_count == 0) // 如果是第一次进入,需要初始化低通滤波
    {
        ins_.accel[0] = ax;
        ins_.accel[1] = ay;
        ins_.accel[2] = az;
        ins_.update_count++;
    }
	if (raw_accel_trusted) {
		const float temp_quick = 1.f / (ins_.dt + ins_.acc_lpf_coef);
		ins_.accel[0] = ins_.accel[0] * ins_.acc_lpf_coef * temp_quick + ax * ins_.dt * temp_quick;
		ins_.accel[1] = ins_.accel[1] * ins_.acc_lpf_coef * temp_quick + ay * ins_.dt * temp_quick;
		ins_.accel[2] = ins_.accel[2] * ins_.acc_lpf_coef * temp_quick + az * ins_.dt * temp_quick;
	}

    // set z,单位化重力加速度向量
    ins_.accl_norm = std::sqrt(ins_.accel[0] * ins_.accel[0] + ins_.accel[1] * ins_.accel[1] + ins_.accel[2] * ins_.accel[2]);
	if (raw_accel_trusted && ins_.accl_norm > 1.0e-6f &&
	    std::fabs(ins_.accl_norm - kGravityMps2) <= kStableAccelToleranceMps2)
    {
        accel_inv_norm = 1.0f / ins_.accl_norm;
        MeasuredVector()(0) = ins_.accel[0] * accel_inv_norm;
        MeasuredVector()(1) = ins_.accel[1] * accel_inv_norm;
        MeasuredVector()(2) = ins_.accel[2] * accel_inv_norm;
		MeasurementValidNum() = 3U;
	}
	else
	{
		MeasuredVector().setZero();
		MeasurementValidNum() = 0U;
	}

    // set Q R,过程噪声和观测噪声矩阵
	Q().setZero();
    Q()(4, 4) = ins_.q2 * ins_.dt;
    Q()(5, 5) = ins_.q2 * ins_.dt;
    R()(0, 0) = ins_.r;
    R()(1, 1) = ins_.r;
    R()(2, 2) = ins_.r;

	RunFilterUpdate();

    // 获取融合后的数据,包括四元数和xy零飘值
    ins_.q[0] = FilteredValue()(0);
    ins_.q[1] = FilteredValue()(1);
    ins_.q[2] = FilteredValue()(2);
    ins_.q[3] = FilteredValue()(3);

    ins_.roll = std::atan2(
        ins_.q[0] * ins_.q[1] + ins_.q[2] * ins_.q[3],
        0.5f - ins_.q[1] * ins_.q[1] - ins_.q[2] * ins_.q[2]
    );

    ins_.roll *= 57.29578f;
	const float pitch_sine = std::clamp(
		-2.0f * (ins_.q[1] * ins_.q[3] - ins_.q[0] * ins_.q[2]),
		-1.0f, 1.0f);
	ins_.pitch = 57.29578f * std::asin(pitch_sine);
    ins_.yaw = std::atan2(ins_.q[1] * ins_.q[2] + ins_.q[0] * ins_.q[3],
                      0.5f - ins_.q[2] * ins_.q[2] - ins_.q[3] * ins_.q[3]);
    ins_.yaw *= 57.29578f;
    ins_.gyro_bias[0] = FilteredValue()(4);
    ins_.gyro_bias[1] = FilteredValue()(5);
    ins_.gyro_bias[2] = 0; // 大部分时候z轴通天,无法观测yaw的漂移

    // get Yaw total, yaw数据可能会超过360,处理一下方便其他功能使用(如小陀螺)
    if (ins_.yaw - ins_.yaw_angle_last > 180.0f)
    {
        ins_.yaw_round_count--;
    }
    else if (ins_.yaw - ins_.yaw_angle_last < -180.0f)
    {
        ins_.yaw_round_count++;
    }
    ins_.yaw_total_angle = 360.0f * ins_.yaw_round_count + ins_.yaw;
    ins_.yaw_angle_last = ins_.yaw;
}

/**
 * @brief 检查滤波输出和协方差的数值健康状态。
 * @return 四元数单位模、姿态有限、协方差有限且近似对称半正定时返回 true。
 */
bool QuaternionEkf::Healthy() const
{
	const float norm_squared = ins_.q[0] * ins_.q[0] + ins_.q[1] * ins_.q[1] +
				   ins_.q[2] * ins_.q[2] + ins_.q[3] * ins_.q[3];
	const auto &covariance = P();
	return std::isfinite(norm_squared) && std::fabs(norm_squared - 1.0f) < 1.0e-3f &&
	       std::isfinite(ins_.roll) && std::isfinite(ins_.pitch) &&
	       std::isfinite(ins_.yaw) && covariance.allFinite() &&
	       covariance.isApprox(covariance.transpose(), 1.0e-4f) &&
	       covariance.diagonal().minCoeff() >= -1.0e-5f;
}

/** @return `[qw, qx, qy, qz]` 顺序的当前单位四元数。 */
std::array<float, 4> QuaternionEkf::Quat() const
{
    return {ins_.q[0], ins_.q[1], ins_.q[2], ins_.q[3]};
}

/** @return 当前横滚角，单位为度。 */
float QuaternionEkf::RollDeg() const { return ins_.roll; }
/** @return 当前俯仰角，单位为度。 */
float QuaternionEkf::PitchDeg() const { return ins_.pitch; }
/** @return 当前 `[-180, 180]` 航向角，单位为度。 */
float QuaternionEkf::YawDeg() const { return ins_.yaw; }
/** @return 连续展开的航向角，单位为度。 */
float QuaternionEkf::YawTotalDeg() const { return ins_.yaw_total_angle; }
/** @return 去除内部零偏后的 Z 轴角速度，单位为 rad/s。 */
float QuaternionEkf::YawOmegaRad() const { return ins_.gyro[2]; }
/** @return 去除内部零偏后的 Y 轴角速度，单位为 rad/s。 */
float QuaternionEkf::PitchOmegaRad() const { return ins_.gyro[1]; }

/** @return IMU 坐标系下去除 EKF 内部残余零偏后的三轴角速度，单位为 rad/s。 */
std::array<float, 3> QuaternionEkf::GyroRadS() const
{
	return {ins_.gyro[0], ins_.gyro[1], ins_.gyro[2]};
}

} // namespace modules
