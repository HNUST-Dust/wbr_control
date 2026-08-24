/**
 ******************************************************************************
* @file    quaternion_ekf_filter.hpp
 * @ingroup wbr_modules
 * @brief   提供 QuaternionEkf 使用的定长滤波存储和更新流水线。
 * @details 模板在编译期固定状态、控制和观测维度，避免目标机动态分配；用户回调分别挂接预测、观测矩阵和状态后处理阶段。
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

/** @brief 实现四元数 EKF 预测与更新的定维模板。 */
class QuaternionEkfFilter
{
public:
    static constexpr std::size_t kStateSize = 6U; ///< 状态维数：四元数四维加 X/Y 陀螺零偏二维。
    static constexpr std::size_t kMeasurementSize = 3U; ///< 加速度计重力方向观测维数。

    using VecX = Eigen::Matrix<float, kStateSize, 1>; ///< 六维状态向量类型。
    using VecZ = Eigen::Matrix<float, kMeasurementSize, 1>; ///< 三维观测向量类型。
    using MatX = Eigen::Matrix<float, kStateSize, kStateSize>; ///< 状态空间方阵类型。
    using MatZX = Eigen::Matrix<float, kMeasurementSize, kStateSize>; ///< 观测矩阵类型。
    using MatXZ = Eigen::Matrix<float, kStateSize, kMeasurementSize>; ///< 卡尔曼增益矩阵类型。
    using MatZ = Eigen::Matrix<float, kMeasurementSize, kMeasurementSize>; ///< 观测空间方阵类型。

    using Callback = void (*)(QuaternionEkfFilter &); ///< 各 EKF 阶段执行的用户回调类型。

    QuaternionEkfFilter() = default;

    /**
     * @brief 清空内部状态并恢复到初始条件。
     */
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

    /**
     * @brief 使用当前采样更新内部状态并返回本周期结果。
     * @return 指向内部六维滤波结果连续存储区的指针；有效期与滤波器对象一致。
     */
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
    /**
     * @brief 配置是否自动调整观测噪声。
     * @param enabled 为 `true` 时启用该控制功能。
     */
    void SetUseAutoAdjustment(bool enabled) { use_auto_adjustment_ = enabled ? 1U : 0U; }
    /**
     * @brief 查询是否启用了观测噪声自动调整。
     * @return 启用时返回 1，否则返回 0。
     */
    std::uint8_t UseAutoAdjustment() const { return use_auto_adjustment_; }

    /**
     * @brief 配置是否跳过滤波流程第 1 步。
     * @param skip 为 `true` 时跳过对应滤波步骤。
     */
    void SetSkipEq1(bool skip) { skip_eq1_ = skip ? 1U : 0U; }
    /**
     * @brief 配置是否跳过滤波流程第 2 步。
     * @param skip 为 `true` 时跳过对应滤波步骤。
     */
    void SetSkipEq2(bool skip) { skip_eq2_ = skip ? 1U : 0U; }
    /**
     * @brief 配置是否跳过滤波流程第 3 步。
     * @param skip 为 `true` 时跳过对应滤波步骤。
     */
    void SetSkipEq3(bool skip) { skip_eq3_ = skip ? 1U : 0U; }
    /**
     * @brief 配置是否跳过滤波流程第 4 步。
     * @param skip 为 `true` 时跳过对应滤波步骤。
     */
    void SetSkipEq4(bool skip) { skip_eq4_ = skip ? 1U : 0U; }
    /**
     * @brief 配置是否跳过滤波流程第 5 步。
     * @param skip 为 `true` 时跳过对应滤波步骤。
     */
    void SetSkipEq5(bool skip) { skip_eq5_ = skip ? 1U : 0U; }

    /**
     * @brief 查询是否跳过滤波流程第 1 步。
     * @return 跳过时返回 1，否则返回 0。
     */
    std::uint8_t SkipEq1() const { return skip_eq1_; }
    /**
     * @brief 查询是否跳过滤波流程第 2 步。
     * @return 跳过时返回 1，否则返回 0。
     */
    std::uint8_t SkipEq2() const { return skip_eq2_; }
    /**
     * @brief 查询是否跳过滤波流程第 3 步。
     * @return 跳过时返回 1，否则返回 0。
     */
    std::uint8_t SkipEq3() const { return skip_eq3_; }
    /**
     * @brief 查询是否跳过滤波流程第 4 步。
     * @return 跳过时返回 1，否则返回 0。
     */
    std::uint8_t SkipEq4() const { return skip_eq4_; }
    /**
     * @brief 查询是否跳过滤波流程第 5 步。
     * @return 跳过时返回 1，否则返回 0。
     */
    std::uint8_t SkipEq5() const { return skip_eq5_; }

    /**
     * @brief 设置滤波流程第 0 阶段的用户回调。
     * @param cb 对应滤波阶段调用的回调函数。
     */
    void SetUserFunc0(Callback cb) { user_func0_ = cb; }
    /**
     * @brief 设置滤波流程第 1 阶段的用户回调。
     * @param cb 对应滤波阶段调用的回调函数。
     */
    void SetUserFunc1(Callback cb) { user_func1_ = cb; }
    /**
     * @brief 设置滤波流程第 2 阶段的用户回调。
     * @param cb 对应滤波阶段调用的回调函数。
     */
    void SetUserFunc2(Callback cb) { user_func2_ = cb; }
    /**
     * @brief 设置滤波流程第 3 阶段的用户回调。
     * @param cb 对应滤波阶段调用的回调函数。
     */
    void SetUserFunc3(Callback cb) { user_func3_ = cb; }
    /**
     * @brief 设置滤波流程第 4 阶段的用户回调。
     * @param cb 对应滤波阶段调用的回调函数。
     */
    void SetUserFunc4(Callback cb) { user_func4_ = cb; }
    /**
     * @brief 设置滤波流程第 5 阶段的用户回调。
     * @param cb 对应滤波阶段调用的回调函数。
     */
    void SetUserFunc5(Callback cb) { user_func5_ = cb; }
    /**
     * @brief 设置滤波流程第 6 阶段的用户回调。
     * @param cb 对应滤波阶段调用的回调函数。
     */
    void SetUserFunc6(Callback cb) { user_func6_ = cb; }

    // 向量访问
    /** @brief 取得后验状态估计的可修改引用。 @return 后验状态向量。 */
    VecX &xhat() { return xhat_; }
    /** @brief 取得后验状态估计的只读引用。 @return 后验状态向量。 */
    const VecX &xhat() const { return xhat_; }
    /** @brief 取得先验状态估计的可修改引用。 @return 先验状态向量。 */
    VecX &xhatminus() { return xhatminus_; }
    /** @brief 取得先验状态估计的只读引用。 @return 先验状态向量。 */
    const VecX &xhatminus() const { return xhatminus_; }
    /** @brief 取得当前观测向量的可修改引用。 @return 当前观测向量。 */
    VecZ &z() { return z_; }
    /** @brief 取得当前观测向量的只读引用。 @return 当前观测向量。 */
    const VecZ &z() const { return z_; }

    // 矩阵访问
    /** @brief 取得后验状态协方差矩阵。 @return 后验协方差矩阵的可修改引用。 */
    MatX &P() { return p_; }
    /** @brief 取得先验状态协方差矩阵。 @return 先验协方差矩阵的可修改引用。 */
    MatX &Pminus() { return pminus_; }
    /** @brief 取得状态转移矩阵。 @return 状态转移矩阵的可修改引用。 */
    MatX &F() { return f_; }
    /** @brief 取得观测矩阵。 @return 观测矩阵的可修改引用。 */
    MatZX &H() { return h_; }
    /** @brief 取得过程噪声协方差矩阵。 @return 过程噪声协方差矩阵的可修改引用。 */
    MatX &Q() { return q_; }
    /** @brief 取得观测噪声协方差矩阵。 @return 观测噪声协方差矩阵的可修改引用。 */
    MatZ &R() { return r_; }
    /** @brief 取得卡尔曼增益矩阵。 @return 卡尔曼增益矩阵的可修改引用。 */
    MatXZ &K() { return k_; }
    /** @brief 取得新息协方差矩阵。 @return 新息协方差矩阵的可修改引用。 */
    MatZ &S() { return s_; }

    // 测量 / 控制 / 输出
    /** @brief 取得下一更新周期使用的测量输入。 @return 测量输入向量的可修改引用。 */
    VecZ &MeasuredVector() { return measured_vector_; }
    /** @brief 取得滤波输出。 @return 滤波输出向量的可修改引用。 */
    VecX &FilteredValue() { return filtered_value_; }
    /** @brief 取得滤波输出的只读视图。 @return 滤波输出向量的常量引用。 */
    const VecX &FilteredValue() const { return filtered_value_; }

    /** @brief 取得本周期有效观测量数量。 @return 有效观测量计数的可修改引用。 */
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
