/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include <Eigen/Core>

namespace modules {

/**
 * @brief 使用加速度重力方向修正的六状态四元数扩展卡尔曼滤波器。
 * @details 状态为 `[qw, qx, qy, qz, bias_x, bias_y]`。陀螺仪负责姿态预测，
 *          准静止时的归一化加速度负责修正 roll/pitch；由于仅靠重力无法观测
 *          heading，Z 轴零偏和 yaw 不由加速度观测直接修正。类内全部矩阵尺寸
 *          固定，不使用动态内存，适合在 1 kHz AHRS 线程中调用。
 */
class QuaternionEkf final {
public:
	/** @brief EKF 噪声、衰减和采样参数。 */
	struct Params {
		float process_noise_quat_; ///< 四元数过程噪声系数 Q1。
		float process_noise_gyro_bias_; ///< X/Y 陀螺仪零偏随机游走系数 Q2。
		float measure_noise_accel_; ///< 归一化加速度方向的观测噪声 R。
		float fading_lambda_; ///< 零偏协方差衰减系数，内部限制为不小于 1。
		float dt_; ///< 默认更新周期，单位为秒。
		float accel_lpf_coef_; ///< 一阶加速度低通时间常数，单位为秒；0 表示不滤波。
	};

	/** @brief 构造处于零初始化状态的 EKF 对象；使用前必须调用 Init()。 */
	QuaternionEkf() = default;
	/** @brief 销毁 EKF 对象。 */
	~QuaternionEkf();
	QuaternionEkf(const QuaternionEkf &) = delete;
	QuaternionEkf &operator=(const QuaternionEkf &) = delete;
	QuaternionEkf(QuaternionEkf &&) = delete;
	QuaternionEkf &operator=(QuaternionEkf &&) = delete;

	/**
	 * @brief 设置滤波参数并初始化状态和协方差。
	 * @param params 噪声、采样周期及低通参数。
	 */
	void Init(const Params &params);
	/**
	 * @brief 用一帧静止加速度初始化 roll/pitch，并将 yaw 设为零。
	 * @param ax X 轴加速度，单位为 m/s²。
	 * @param ay Y 轴加速度，单位为 m/s²。
	 * @param az Z 轴加速度，单位为 m/s²。
	 * @return 加速度向量有效且初始化成功时返回 true。
	 */
	bool InitFromAccel(float ax, float ay, float az);
	/** @brief 清空动态状态并恢复 Init() 设置的参数和初始协方差。 */
	void Reset();
	/**
	 * @brief 使用 Init() 设置的默认周期完成一次预测与可选重力观测更新。
	 * @param gx X 轴角速度，单位为 rad/s。
	 * @param gy Y 轴角速度，单位为 rad/s。
	 * @param gz Z 轴角速度，单位为 rad/s。
	 * @param ax X 轴加速度，单位为 m/s²。
	 * @param ay Y 轴加速度，单位为 m/s²。
	 * @param az Z 轴加速度，单位为 m/s²。
	 */
	void Update(float gx, float gy, float gz, float ax, float ay, float az);
	/**
	 * @brief 使用实测周期完成一次预测与可选重力观测更新。
	 * @param gx X 轴角速度，单位为 rad/s。
	 * @param gy Y 轴角速度，单位为 rad/s。
	 * @param gz Z 轴角速度，单位为 rad/s。
	 * @param ax X 轴加速度，单位为 m/s²。
	 * @param ay Y 轴加速度，单位为 m/s²。
	 * @param az Z 轴加速度，单位为 m/s²。
	 * @param dt_seconds 当前样本周期，单位为秒；非正数或非有限值会被忽略。
	 */
	void Update(float gx, float gy, float gz, float ax, float ay, float az,
		    float dt_seconds);
	/** @return 四元数、欧拉角和协方差均有限且满足约束时返回 true。 */
	bool Healthy() const;
	/** @return `[qw, qx, qy, qz]` 顺序的单位四元数。 */
	std::array<float, 4> Quat() const;
	/** @return 横滚角，单位为度。 */
	float RollDeg() const;
	/** @return 俯仰角，单位为度。 */
	float PitchDeg() const;
	/** @return `[-180, 180]` 范围内的航向角，单位为度。 */
	float YawDeg() const;
	/** @return 跨越正负 180 度后连续展开的航向角，单位为度。 */
	float YawTotalDeg() const;
	/** @return 去除内部零偏估计后的 Z 轴角速度，单位为 rad/s。 */
	float YawOmegaRad() const;
	/** @return 去除内部零偏估计后的 Y 轴角速度，单位为 rad/s。 */
	float PitchOmegaRad() const;
	/** @return IMU 坐标系下去除 EKF 内部残余零偏后的三轴角速度，单位为 rad/s。 */
	std::array<float, 3> GyroRadS() const;

private:
	static constexpr std::size_t kStateSize = 6U; ///< 四元数四维加 X/Y 零偏二维。
	static constexpr std::size_t kMeasurementSize = 3U; ///< 三轴重力方向观测维数。
	using VecX = Eigen::Matrix<float, kStateSize, 1>;
	using VecZ = Eigen::Matrix<float, kMeasurementSize, 1>;
	using MatX = Eigen::Matrix<float, kStateSize, kStateSize>;
	using MatZX = Eigen::Matrix<float, kMeasurementSize, kStateSize>;
	using MatXZ = Eigen::Matrix<float, kStateSize, kMeasurementSize>;
	using MatZ = Eigen::Matrix<float, kMeasurementSize, kMeasurementSize>;

	/** @brief 滤波所需的标量参数、门控状态和输出；矩阵状态直接属于 QuaternionEkf。 */
	struct QekfIns {
		std::uint8_t converge_flag = 0U; ///< 最近一次创新通过门控的标志。
		std::uint64_t update_count = 0U; ///< 加速度低通初始化和更新计数。
		float q[4] = {}; ///< 输出四元数 `[qw, qx, qy, qz]`。
		float gyro_bias[3] = {}; ///< 内部陀螺仪零偏估计，单位为 rad/s；Z 固定为零。
		float gyro[3] = {}; ///< 去除内部零偏后的角速度，单位为 rad/s。
		float accel[3] = {}; ///< 低通后的加速度，单位为 m/s²。
		float acc_lpf_coef = 0.0f; ///< 加速度一阶低通时间常数，单位为秒。
		float accl_norm = 0.0f; ///< 低通加速度向量模长，单位为 m/s²。
		float adaptive_gain_scale = 1.0f; ///< 创新软门控产生的增益缩放系数。
		float roll = 0.0f; ///< 横滚角，单位为度。
		float pitch = 0.0f; ///< 俯仰角，单位为度。
		float yaw = 0.0f; ///< 航向角，单位为度。
		float yaw_total_angle = 0.0f; ///< 连续展开航向角，单位为度。
		float q1 = 0.0f; ///< 四元数过程噪声系数。
		float q2 = 0.0f; ///< 零偏过程噪声系数。
		float r = 0.0f; ///< 加速度方向观测噪声。
		float dt = 0.001f; ///< 当前更新周期，单位为秒。
		float chi_square = 0.0f; ///< 当前创新卡方统计量。
		float chi_square_test_threshold = 0.0f; ///< 创新硬拒绝阈值。
		float chi_square_soft_threshold = 0.0f; ///< 创新增益衰减起始阈值。
		float lambda = 1.0f; ///< 零偏协方差衰减系数。
		std::int16_t yaw_round_count = 0; ///< 航向跨越正负 180 度的累计圈数。
		float yaw_angle_last = 0.0f; ///< 上一帧航向角，单位为度。
	};

	/** @brief 归一化先验四元数，并更新状态转移和过程噪声约束。 */
	void ApplyPredictionConstraints();
	/** @brief 在当前先验四元数处构造重力观测 Jacobian。 */
	void BuildObservationMatrix();
	/** @brief 执行创新门控、自适应 Kalman 增益和后验状态校正。 */
	void ApplyMeasurementCorrection();
	/** @brief 将四元数协方差投影到单位四元数切空间并恢复对称性。 */
	void NormalizeCovariance();
	/** @brief 清空所有固定尺寸向量和矩阵状态。 */
	void ResetFilterState();
	/** @brief 按预测、观测、Joseph 协方差更新顺序执行一次内部流水线。 */
	void RunFilterUpdate();

	/** @brief 设置是否跳过本周期 Joseph 协方差校正。 */
	void SetSkipEq5(bool skip) { skip_eq5_ = skip; }
	/** @return 后验状态向量的可写引用。 */
	VecX &xhat() { return xhat_; }
	/** @return 后验状态向量的只读引用。 */
	const VecX &xhat() const { return xhat_; }
	/** @return 先验状态向量的可写引用。 */
	VecX &xhatminus() { return xhatminus_; }
	/** @return 当前重力方向观测向量的可写引用。 */
	VecZ &z() { return z_; }
	/** @return 后验协方差矩阵的可写引用。 */
	MatX &P() { return p_; }
	/** @return 后验协方差矩阵的只读引用。 */
	const MatX &P() const { return p_; }
	/** @return 先验协方差矩阵的可写引用。 */
	MatX &Pminus() { return pminus_; }
	/** @return 状态转移矩阵的可写引用。 */
	MatX &F() { return f_; }
	/** @return 重力观测 Jacobian 的可写引用。 */
	MatZX &H() { return h_; }
	/** @return 过程噪声矩阵的可写引用。 */
	MatX &Q() { return process_noise_; }
	/** @return 观测噪声矩阵的可写引用。 */
	MatZ &R() { return measurement_noise_; }
	/** @return Kalman 增益矩阵的可写引用。 */
	MatXZ &K() { return kalman_gain_; }
	/** @return 创新协方差矩阵的可写引用。 */
	MatZ &S() { return innovation_covariance_; }
	/** @return 下一周期输入的归一化加速度观测。 */
	VecZ &MeasuredVector() { return measured_vector_; }
	/** @return 最终滤波状态的可写引用。 */
	VecX &FilteredValue() { return filtered_value_; }
	/** @return 最终滤波状态的只读引用。 */
	const VecX &FilteredValue() const { return filtered_value_; }
	/** @return 当前周期有效观测维数的可写引用。 */
	std::uint8_t &MeasurementValidNum() { return measurement_valid_num_; }

	QekfIns ins_{}; ///< 标量配置、门控状态和输出。
	bool skip_eq5_ = false; ///< 本周期跳过协方差校正的标志。
	std::uint8_t measurement_valid_num_ = 0U; ///< 当前有效观测维数，0 或 3。
	VecX filtered_value_ = VecX::Zero(); ///< 对外输出的六维后验状态。
	VecZ measured_vector_ = VecZ::Zero(); ///< 下一更新周期使用的重力方向观测。
	VecX xhat_ = VecX::Zero(); ///< 六维后验状态估计。
	VecX xhatminus_ = VecX::Zero(); ///< 六维先验状态估计。
	VecZ z_ = VecZ::Zero(); ///< 当前更新周期锁存的重力方向观测。
	MatX p_ = MatX::Zero(); ///< 后验状态协方差。
	MatX pminus_ = MatX::Zero(); ///< 先验状态协方差。
	MatX f_ = MatX::Zero(); ///< 线性化状态转移矩阵。
	MatZX h_ = MatZX::Zero(); ///< 重力观测 Jacobian。
	MatX process_noise_ = MatX::Zero(); ///< 过程噪声协方差 Q。
	MatZ measurement_noise_ = MatZ::Zero(); ///< 观测噪声协方差 R。
	MatXZ kalman_gain_ = MatXZ::Zero(); ///< Kalman 增益 K。
	MatZ innovation_covariance_ = MatZ::Zero(); ///< 创新协方差 S。
};

}  // namespace modules
