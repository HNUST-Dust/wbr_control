/**
 ******************************************************************************
* @file    quaternion_ekf.h
 * @ingroup wbr_modules
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2022/3/8
 * @brief   声明带陀螺仪零偏估计和卡方检验的四元数姿态滤波器。
 * @details 调用方以弧度每秒输入角速度，并按固定周期调用 `Update()`；输出四元数采用 `[w, x, y, z]` 排列。
 ******************************************************************************
 */

#pragma once

#include <array>
#include <cstdint>

#include <modules/imu/onboard/quaternion_ekf_filter.hpp>

namespace modules {

/** @brief 基于四元数状态的扩展卡尔曼姿态滤波器。 */
class QuaternionEkf
{
public:
    /** @brief 四元数 EKF 的噪声、增益和限幅参数。 */
    struct Params
    {
        float process_noise_quat_; ///< 四元数状态过程噪声方差。
        float process_noise_gyro_bias_; ///< 陀螺仪零偏过程噪声方差。
        float measure_noise_accel_; ///< 加速度观测噪声方差。
        float fading_lambda_; ///< 协方差衰减系数，大于等于 1。
        float dt_; ///< 滤波器当前使用的采样周期，单位为秒。
        float accel_lpf_coef_; ///< 加速度低通滤波系数，范围为 0 至 1。
    };

    // Update inputs: gyro in rad/s, acceleration in m/s^2.
    /**
     * @brief 使用给定参数初始化姿态估计器。
     * @param[in] params 滤波器噪声、阈值和初始协方差参数。
     */
    void Init(const Params &params);
    /**
     * @brief 清空内部状态并恢复到初始条件。
     */
    void Reset();
    /**
     * @brief 使用当前采样更新内部状态并返回本周期结果。
     * @param gx 陀螺仪 X 轴角速度，单位为弧度每秒。
     * @param gy 陀螺仪 Y 轴角速度，单位为弧度每秒。
     * @param gz 陀螺仪 Z 轴角速度，单位为弧度每秒。
     * @param ax 加速度计 X 轴测量值。
     * @param ay 加速度计 Y 轴测量值。
     * @param az 加速度计 Z 轴测量值。
     */
    void Update(float gx, float gy, float gz, float ax, float ay, float az);

    /**
     * @brief 读取当前姿态四元数。
     * @return 按 `[w, x, y, z]` 排列的单位四元数。
     */
    std::array<float, 4> Quat() const;
    /**
     * @brief 读取横滚角。
     * @return 横滚角，单位为度。
     */
    float RollDeg() const;
    /**
     * @brief 读取俯仰角。
     * @return 俯仰角，单位为度。
     */
    float PitchDeg() const;
    /**
     * @brief 读取归一化航向角。
     * @return 范围约为 `[-180, 180]` 的航向角，单位为度。
     */
    float YawDeg() const;
    /**
     * @brief 读取跨越正负 180° 后连续累计的航向角。
     * @return 连续累计航向角，单位为度。
     */
    float YawTotalDeg() const;
    /**
     * @brief 读取航向轴角速度。
     * @return 航向轴角速度，单位为弧度每秒。
     */
    float YawOmegaRad() const;
    /**
     * @brief 读取俯仰轴角速度。
     * @return 俯仰轴角速度，单位为弧度每秒。
     */
    float PitchOmegaRad() const;

	QuaternionEkf() = default;
	/** @brief 释放 EKF 私有实现持有的滤波器状态。 */
	~QuaternionEkf();

    QuaternionEkf(const QuaternionEkf &) = delete;
    QuaternionEkf &operator=(const QuaternionEkf &) = delete;
    QuaternionEkf(QuaternionEkf &&) = delete;
    QuaternionEkf &operator=(QuaternionEkf &&) = delete;

private:
    using ImuKf = QuaternionEkfFilter;

    /** @brief 四元数 EKF 的内部状态、协方差及中间矩阵。 */
    struct QekfIns
    {
        std::uint8_t initialized; ///< 内部参考状态已经初始化的标志。
        ImuKf imu_quaternion_ekf; ///< 四元数 EKF 的滤波器实例。
        std::uint8_t converge_flag; ///< 滤波器创新量已满足收敛判据的标志。
        std::uint8_t stable_flag; ///< 姿态和速度已进入稳定判据范围的标志。
        std::uint64_t error_count; ///< 累计事件次数；计数达到类型上限后按实现方式饱和或回绕。
        std::uint64_t update_count; ///< 累计事件次数；计数达到类型上限后按实现方式饱和或回绕。

        float q[4]; ///< 四元数状态向量。
        float gyro_bias[3]; ///< 三轴陀螺仪零偏估计，单位为弧度每秒。

        float gyro[3]; ///< 三轴陀螺仪角速度向量，单位为弧度每秒。
        float accel[3]; ///< 三轴加速度计测量向量。

        float orientation_cosine[3]; ///< 腿轴方向与竖直方向夹角的余弦。

        float acc_lpf_coef; ///< 加速度低通滤波系数，范围为 0 至 1。
        float gyro_norm; ///< 三轴角速度向量的模。
        float accl_norm; ///< 三轴加速度向量的模。
        float adaptive_gain_scale; ///< 控制器使用的比例或缩放系数。

        float roll; ///< 机体横滚角，单位为弧度。
        float pitch; ///< 机体俯仰角，单位为弧度。
        float yaw; ///< 机体航向角，单位为弧度。

        float yaw_total_angle; ///< 角度，单位为弧度。

        float q1; ///< 滤波计算使用的第一组四元数中间量。
        float q2; ///< 滤波计算使用的第二组四元数中间量。
        float r; ///< 滤波器观测噪声协方差矩阵。

        float dt; ///< 控制周期，单位为秒且必须大于零。
        float chi_square; ///< 当前观测创新的卡方统计量。
        float chi_square_test_threshold; ///< 控制器使用的可配置阈值。
        float lambda; ///< 协方差衰减或观测调整系数。

        std::int16_t yaw_round_count; ///< 累计事件次数；计数达到类型上限后按实现方式饱和或回绕。
        float yaw_angle_last; ///< 角度，单位为弧度。

        // Debug/telemetry snapshots (kept per-instance)
        float imu_quaternion_ekf_p[36]; ///< 四元数 EKF 状态协方差矩阵。
        float imu_quaternion_ekf_k[18]; ///< 四元数 EKF 卡尔曼增益矩阵。
        float imu_quaternion_ekf_h[18]; ///< 四元数 EKF 观测矩阵。
    };

    static QekfIns &InsFromKf(ImuKf *kf);

    /**
     * @brief 根据当前姿态更新 EKF 观测量。
     * @param[in,out] kf 正在执行当前阶段的 EKF 实例。
     */
    static void ObserveCb(ImuKf &kf);
    /**
     * @brief 更新状态转移雅可比并执行协方差衰减。
     * @param[in,out] kf 正在执行当前阶段的 EKF 实例。
     */
    static void FLinearizationPFadingCb(ImuKf &kf);
    /**
     * @brief 更新 EKF 观测矩阵。
     * @param[in,out] kf 正在执行当前阶段的 EKF 实例。
     */
    static void SetHCb(ImuKf &kf);
    /**
     * @brief 执行四元数状态更新后的归一化处理。
     * @param[in,out] kf 正在执行当前阶段的 EKF 实例。
     */
    static void XhatUpdateCb(ImuKf &kf);

    QekfIns ins_{};
};

} // namespace modules
