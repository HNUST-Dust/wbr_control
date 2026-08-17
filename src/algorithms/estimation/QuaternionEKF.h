/**
 ******************************************************************************
 * @file    QuaternionEKF.h
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2022/3/8
 * @brief   attitude update with gyro bias estimate and chi-square test
 ******************************************************************************
 */

#pragma once

#include <array>
#include <cstdint>

#include <algorithms/filter/kalman_filter.hpp>

namespace alg {

class QuaternionEkf
{
public:
    struct Params
    {
        float process_noise_quat_;
        float process_noise_gyro_bias_;
        float measure_noise_accel_;
        float fading_lambda_;
        float dt_;
        float accel_lpf_coef_;
    };

    void Init(const Params &params);
    void Reset();
    void Update(float gx, float gy, float gz, float ax, float ay, float az);

    std::array<float, 4> Quat() const;
    float RollDeg() const;
    float PitchDeg() const;
    float YawDeg() const;
    float YawTotalDeg() const;
    float YawOmegaRad() const;
    float PitchOmegaRad() const;

    QuaternionEkf() = default;
    ~QuaternionEkf();

    QuaternionEkf(const QuaternionEkf &) = delete;
    QuaternionEkf &operator=(const QuaternionEkf &) = delete;
    QuaternionEkf(QuaternionEkf &&) = delete;
    QuaternionEkf &operator=(QuaternionEkf &&) = delete;

private:
    using ImuKf = alg::KalmanFilter<6, 0, 3>;

    struct QekfIns
    {
        std::uint8_t initialized;
        ImuKf imu_quaternion_ekf;
        std::uint8_t converge_flag;
        std::uint8_t stable_flag;
        std::uint64_t error_count;
        std::uint64_t update_count;

        float q[4];
        float gyro_bias[3];

        float gyro[3];
        float accel[3];

        float orientation_cosine[3];

        float acc_lpf_coef;
        float gyro_norm;
        float accl_norm;
        float adaptive_gain_scale;

        float roll;
        float pitch;
        float yaw;

        float yaw_total_angle;

        float q1;
        float q2;
        float r;

        float dt;
        float chi_square;
        float chi_square_test_threshold;
        float lambda;

        std::int16_t yaw_round_count;
        float yaw_angle_last;

        // Debug/telemetry snapshots (kept per-instance)
        float imu_quaternion_ekf_p[36];
        float imu_quaternion_ekf_k[18];
        float imu_quaternion_ekf_h[18];
    };

    static QekfIns &InsFromKf(ImuKf *kf);

    static void ObserveCb(ImuKf &kf);
    static void FLinearizationPFadingCb(ImuKf &kf);
    static void SetHCb(ImuKf &kf);
    static void XhatUpdateCb(ImuKf &kf);

    QekfIns ins_{};
};

} // namespace alg
