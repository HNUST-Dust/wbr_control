/**
* @file src/modules/imu/onboard/mahony_attitude_estimator.h
 * @ingroup wbr_modules
 * @brief 实现基于 Mahony 滤波器的姿态估计。
 * @details 模块遵循 `ModuleBase` 生命周期：`Start()` 只负责一次性资源初始化和线程创建，`RunLoop()` 持有周期状态。跨线程数据通过 channels 层交换。
 */

//=============================================================================================
// Mahony attitude estimator for the onboard IMU pipeline.
// Based on Madgwick's implementation of Mahony's AHRS algorithm.
//=============================================================================================

#pragma once

#include <array>

namespace modules {

/** @brief 使用陀螺仪和加速度计估计姿态的 Mahony 滤波器。 */
class MahonyAttitudeEstimator {
public:
	// Gyroscope inputs are rad/s. Accelerometer inputs are m/s^2 (only the
	// direction is used). Magnetometer inputs may use any consistent unit.
	/**
	 * @brief 使用给定参数初始化姿态估计器。
	 * @param sample_frequency_hz 姿态更新频率，单位为赫兹且必须大于零。
	 */
	void Init(float sample_frequency_hz);
	/**
	 * @brief 根据静止加速度和磁场测量初始化姿态。
	 * @param ax 加速度计 X 轴测量值。
	 * @param ay 加速度计 Y 轴测量值。
	 * @param az 加速度计 Z 轴测量值。
	 * @param mx 磁力计 X 轴测量值。
	 * @param my 磁力计 Y 轴测量值。
	 * @param mz 磁力计 Z 轴测量值。
	 */
	void InitFromAccMag(float ax, float ay, float az, float mx, float my, float mz);

	/**
	 * @brief 使用当前采样更新内部状态并返回本周期结果。
	 * @param gx 陀螺仪 X 轴角速度，单位为弧度每秒。
	 * @param gy 陀螺仪 Y 轴角速度，单位为弧度每秒。
	 * @param gz 陀螺仪 Z 轴角速度，单位为弧度每秒。
	 * @param ax 加速度计 X 轴测量值。
	 * @param ay 加速度计 Y 轴测量值。
	 * @param az 加速度计 Z 轴测量值。
	 * @param mx 磁力计 X 轴测量值。
	 * @param my 磁力计 Y 轴测量值。
	 * @param mz 磁力计 Z 轴测量值。
	 */
	void Update(float gx, float gy, float gz, float ax, float ay, float az,
		    float mx, float my, float mz);
	/**
	 * @brief 仅使用陀螺仪和加速度计更新姿态。
	 * @param gx 陀螺仪 X 轴角速度，单位为弧度每秒。
	 * @param gy 陀螺仪 Y 轴角速度，单位为弧度每秒。
	 * @param gz 陀螺仪 Z 轴角速度，单位为弧度每秒。
	 * @param ax 加速度计 X 轴测量值。
	 * @param ay 加速度计 Y 轴测量值。
	 * @param az 加速度计 Z 轴测量值。
	 */
	void UpdateImu(float gx, float gy, float gz, float ax, float ay, float az);

	/**
	 * @brief 由当前四元数更新欧拉角缓存。
	 */
	void ComputeAngles();

	/**
	 * @brief 读取横滚角。
	 * @return 横滚角，单位为度。
	 */
	float RollDeg();
	/**
	 * @brief 读取俯仰角。
	 * @return 俯仰角，单位为度。
	 */
	float PitchDeg();
	/**
	 * @brief 读取归一化航向角。
	 * @return 范围约为 `[-180, 180]` 的航向角，单位为度。
	 */
	float YawDeg();

	/**
	 * @brief 读取当前姿态四元数。
	 * @return 按 `[w, x, y, z]` 排列的单位四元数。
	 */
	std::array<float, 4> Quat() const { return {q0_, q1_, q2_, q3_}; }

private:
	/**
	 * @brief 计算用于向量归一化的快速平方根倒数。
	 * @param x 待计算平方根倒数的正数。
	 * @return 输入值平方根的倒数近似值。
	 */
	static float InvSqrt(float x);

	static constexpr float kRad2Deg = 57.29578f; ///< 弧度转换为角度的比例系数。
	static constexpr float kTwoKp = 2.0f * 0.5f; ///< `kTwoKp` 比例增益。
	static constexpr float kTwoKiDefault = 2.0f * 0.0f; ///< Mahony 误差积分项默认双倍增益；零值表示默认关闭积分。
	static constexpr float kVectorNormEpsilon = 1.0e-12f; ///< 向量归一化前用于拒绝近零模长的平方阈值。

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
