/**
 ******************************************************************************
 * @file    quaternion_ekf_filter.hpp
 * @brief   Fixed-size filter storage and update pipeline for QuaternionEkf
 ******************************************************************************
 */

#pragma once

// This is deliberately specific to the onboard IMU: 6 states (quaternion + XY gyro bias)
// and 3 accelerometer measurements. It is not a reusable Kalman library.

#include <Eigen/Core>
#include <Eigen/LU>

#include <cstddef>
#include <cstdint>

namespace modules {

class QuaternionEkfFilter
{
public:
    static constexpr std::size_t kStateSize = 6U;
    static constexpr std::size_t kMeasurementSize = 3U;

    using VecX = Eigen::Matrix<float, kStateSize, 1>;
    using VecZ = Eigen::Matrix<float, kMeasurementSize, 1>;
    using MatX = Eigen::Matrix<float, kStateSize, kStateSize>;
    using MatZX = Eigen::Matrix<float, kMeasurementSize, kStateSize>;
    using MatXZ = Eigen::Matrix<float, kStateSize, kMeasurementSize>;
    using MatZ = Eigen::Matrix<float, kMeasurementSize, kMeasurementSize>;

    using Callback = void (*)(QuaternionEkfFilter &);

    QuaternionEkfFilter() = default;

    void Reset()
    {
        skip_eq1_ = skip_eq2_ = skip_eq3_ = skip_eq4_ = skip_eq5_ = 0;
        filtered_value_.setZero();
        measured_vector_.setZero();
        xhat_.setZero();
        xhatminus_.setZero();
        z_.setZero();

        p_.setZero();
        pminus_.setZero();
        f_.setZero();
        h_.setZero();
        q_.setZero();
        r_.setZero();
        k_.setZero();
        s_.setZero();

        state_min_variance_.setZero();
    }

    float *Update()
    {
        Measure_();
        if (user_func0_ != nullptr)
            user_func0_(*this);

        XhatMinusUpdate_();
        if (user_func1_ != nullptr)
            user_func1_(*this);

        PminusUpdate_();
        if (user_func2_ != nullptr)
            user_func2_(*this);

        if (measurement_valid_num_ != 0 || use_auto_adjustment_ == 0)
        {
            SetK_();
            if (user_func3_ != nullptr)
                user_func3_(*this);

            XhatUpdate_();
            if (user_func4_ != nullptr)
                user_func4_(*this);

            PUpdate_();
        }
        else
        {
            xhat_ = xhatminus_;
            p_ = pminus_;
        }

        if (user_func5_ != nullptr)
            user_func5_(*this);

        // 抑制滤波器过度收敛：限制协方差对角下界。
        for (std::size_t i = 0; i < kStateSize; ++i)
        {
            if (p_(i, i) < state_min_variance_(i))
            {
                p_(i, i) = state_min_variance_(i);
            }
        }

        filtered_value_ = xhat_;

        if (user_func6_ != nullptr)
            user_func6_(*this);

        return filtered_value_.data();
    }

    // 标志位 / 配置
    void SetUseAutoAdjustment(bool enabled) { use_auto_adjustment_ = enabled ? 1U : 0U; }
    std::uint8_t UseAutoAdjustment() const { return use_auto_adjustment_; }

    void SetSkipEq1(bool skip) { skip_eq1_ = skip ? 1U : 0U; }
    void SetSkipEq2(bool skip) { skip_eq2_ = skip ? 1U : 0U; }
    void SetSkipEq3(bool skip) { skip_eq3_ = skip ? 1U : 0U; }
    void SetSkipEq4(bool skip) { skip_eq4_ = skip ? 1U : 0U; }
    void SetSkipEq5(bool skip) { skip_eq5_ = skip ? 1U : 0U; }

    std::uint8_t SkipEq1() const { return skip_eq1_; }
    std::uint8_t SkipEq2() const { return skip_eq2_; }
    std::uint8_t SkipEq3() const { return skip_eq3_; }
    std::uint8_t SkipEq4() const { return skip_eq4_; }
    std::uint8_t SkipEq5() const { return skip_eq5_; }

    void SetUserFunc0(Callback cb) { user_func0_ = cb; }
    void SetUserFunc1(Callback cb) { user_func1_ = cb; }
    void SetUserFunc2(Callback cb) { user_func2_ = cb; }
    void SetUserFunc3(Callback cb) { user_func3_ = cb; }
    void SetUserFunc4(Callback cb) { user_func4_ = cb; }
    void SetUserFunc5(Callback cb) { user_func5_ = cb; }
    void SetUserFunc6(Callback cb) { user_func6_ = cb; }

    // 向量访问
    VecX &xhat() { return xhat_; }
    const VecX &xhat() const { return xhat_; }
    VecX &xhatminus() { return xhatminus_; }
    const VecX &xhatminus() const { return xhatminus_; }
    VecZ &z() { return z_; }
    const VecZ &z() const { return z_; }

    // 矩阵访问
    MatX &P() { return p_; }
    MatX &Pminus() { return pminus_; }
    MatX &F() { return f_; }
    MatZX &H() { return h_; }
    MatX &Q() { return q_; }
    MatZ &R() { return r_; }
    MatXZ &K() { return k_; }
    MatZ &S() { return s_; }

    // 测量 / 控制 / 输出
    VecZ &MeasuredVector() { return measured_vector_; }
    VecX &FilteredValue() { return filtered_value_; }
    const VecX &FilteredValue() const { return filtered_value_; }

    std::uint8_t &MeasurementValidNum() { return measurement_valid_num_; }

private:
    void Measure_()
    {
        z_ = measured_vector_;
        measured_vector_.setZero();
    }

    void XhatMinusUpdate_()
    {
        if (skip_eq1_ != 0)
        {
            return;
        }
        xhatminus_ = f_ * xhat_;
    }

    void PminusUpdate_()
    {
        if (skip_eq2_ != 0)
        {
            return;
        }
        pminus_ = f_ * p_ * f_.transpose() + q_;
    }

    void SetK_()
    {
        if (skip_eq3_ != 0)
        {
            return;
        }
        s_ = h_ * pminus_ * h_.transpose() + r_;
        k_ = pminus_ * h_.transpose() * s_.inverse();
    }

    void XhatUpdate_()
    {
        if (skip_eq4_ != 0)
        {
            return;
        }
        xhat_ = xhatminus_ + k_ * (z_ - h_ * xhatminus_);
    }

    void PUpdate_()
    {
        if (skip_eq5_ != 0)
        {
            return;
        }
        p_ = pminus_ - k_ * h_ * pminus_;
    }

    std::uint8_t use_auto_adjustment_ = 0;
    std::uint8_t measurement_valid_num_ = 0;

    std::uint8_t skip_eq1_ = 0;
    std::uint8_t skip_eq2_ = 0;
    std::uint8_t skip_eq3_ = 0;
    std::uint8_t skip_eq4_ = 0;
    std::uint8_t skip_eq5_ = 0;

    Callback user_func0_ = nullptr;
    Callback user_func1_ = nullptr;
    Callback user_func2_ = nullptr;
    Callback user_func3_ = nullptr;
    Callback user_func4_ = nullptr;
    Callback user_func5_ = nullptr;
    Callback user_func6_ = nullptr;

    VecX filtered_value_;
    VecZ measured_vector_;
    VecX xhat_;
    VecX xhatminus_;
    VecZ z_;

    MatX p_;
    MatX pminus_;
    MatX f_;
    MatZX h_;
    MatX q_;
    MatZ r_;
    MatXZ k_;
    MatZ s_;

    VecX state_min_variance_;
};

} // namespace modules
