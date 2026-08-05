/**
 ******************************************************************************
 * @file    kalman_filter.h
 * @brief   Pure C++ kalman filter (static-size, no heap, no legacy C API)
 ******************************************************************************
 */

#pragma once

// Public fixed-size Kalman filter template.

#include <array>
#include <cstdint>
#include <cstring>

#include "riscv_dsp_matrix_math.h"

typedef struct {
    std::uint16_t numRows;
    std::uint16_t numCols;
    float *pData;
} mat;

namespace alg {

using MatrixView = ::mat;
using MatrixStatus = int32_t;

constexpr MatrixStatus kMatrixStatusOk = 0;
constexpr MatrixStatus kMatrixStatusSizeMismatch = -3;
constexpr MatrixStatus kMatrixStatusArgError = -1;

inline void InitializeMatrixView(MatrixView &m, std::uint16_t rows, std::uint16_t cols, float *data)
{
    m.numRows = rows;
    m.numCols = cols;
    m.pData = data;
}

inline MatrixStatus AddMatrices(const MatrixView &a, const MatrixView &b, MatrixView &out)
{
    if (a.numRows != b.numRows || a.numCols != b.numCols ||
        out.numRows != a.numRows || out.numCols != a.numCols) {
        return kMatrixStatusSizeMismatch;
    }

    riscv_dsp_mat_add_f32(a.pData, b.pData, out.pData, a.numRows, a.numCols);
    return kMatrixStatusOk;
}

inline MatrixStatus SubtractMatrices(const MatrixView &a, const MatrixView &b, MatrixView &out)
{
    if (a.numRows != b.numRows || a.numCols != b.numCols ||
        out.numRows != a.numRows || out.numCols != a.numCols) {
        return kMatrixStatusSizeMismatch;
    }

    riscv_dsp_mat_sub_f32(a.pData, b.pData, out.pData, a.numRows, a.numCols);
    return kMatrixStatusOk;
}

inline MatrixStatus MultiplyMatrices(const MatrixView &a, const MatrixView &b, MatrixView &out)
{
    if (a.numCols != b.numRows || out.numRows != a.numRows || out.numCols != b.numCols) {
        return kMatrixStatusSizeMismatch;
    }

    riscv_dsp_mat_mul_f32(a.pData, b.pData, out.pData, a.numRows, a.numCols, b.numCols);
    return kMatrixStatusOk;
}

inline MatrixStatus TransposeMatrix(const MatrixView &a, MatrixView &out)
{
    if (out.numRows != a.numCols || out.numCols != a.numRows) {
        return kMatrixStatusSizeMismatch;
    }

    riscv_dsp_mat_trans_f32(a.pData, out.pData, a.numRows, a.numCols);
    return kMatrixStatusOk;
}

inline MatrixStatus InvertMatrix(const MatrixView &a, MatrixView &out)
{
    if (a.numRows != a.numCols || out.numRows != a.numRows || out.numCols != a.numCols) {
        return kMatrixStatusSizeMismatch;
    }

    constexpr std::size_t kMaxElems = 64;
    const std::size_t elems = static_cast<std::size_t>(a.numRows) * a.numCols;
    if (elems > kMaxElems) {
        return kMatrixStatusArgError;
    }

    float src_copy[kMaxElems] = {0.0f};
    std::memcpy(src_copy, a.pData, elems * sizeof(float));
    return riscv_dsp_mat_inv_f32(src_copy, out.pData, a.numRows);
}

} // namespace alg

namespace alg {

template <std::size_t XHAT, std::size_t U, std::size_t Z>
class KalmanFilter
{
public:
    static constexpr std::uint8_t xhat_size = static_cast<std::uint8_t>(XHAT);
    static constexpr std::uint8_t u_size = static_cast<std::uint8_t>(U);
    static constexpr std::uint8_t z_size = static_cast<std::uint8_t>(Z);

    using Callback = void (*)(KalmanFilter &);

    KalmanFilter() { InitMatrices_(); }

    void Reset()
    {
        skip_eq1_ = 0;
        skip_eq2_ = 0;
        skip_eq3_ = 0;
        skip_eq4_ = 0;
        skip_eq5_ = 0;

        std::memset(filtered_value_.data(), 0, sizeof(float) * XHAT);
        std::memset(measured_vector_.data(), 0, sizeof(float) * Z);
        if constexpr (U > 0)
        {
            std::memset(control_vector_.data(), 0, sizeof(float) * U);
        }

        std::memset(xhat_data_.data(), 0, sizeof(float) * XHAT);
        std::memset(xhatminus_data_.data(), 0, sizeof(float) * XHAT);
        if constexpr (U > 0)
        {
            std::memset(u_data_.data(), 0, sizeof(float) * U);
        }
        std::memset(z_data_.data(), 0, sizeof(float) * Z);

        std::memset(p_data_.data(), 0, sizeof(float) * XHAT * XHAT);
        std::memset(pminus_data_.data(), 0, sizeof(float) * XHAT * XHAT);
        std::memset(f_data_.data(), 0, sizeof(float) * XHAT * XHAT);
        std::memset(ft_data_.data(), 0, sizeof(float) * XHAT * XHAT);
        if constexpr (U > 0)
        {
            std::memset(b_data_.data(), 0, sizeof(float) * XHAT * U);
        }
        std::memset(h_data_.data(), 0, sizeof(float) * Z * XHAT);
        std::memset(ht_data_.data(), 0, sizeof(float) * XHAT * Z);
        std::memset(q_data_.data(), 0, sizeof(float) * XHAT * XHAT);
        std::memset(r_data_.data(), 0, sizeof(float) * Z * Z);
        std::memset(k_data_.data(), 0, sizeof(float) * XHAT * Z);

        std::memset(state_min_variance_.data(), 0, sizeof(float) * XHAT);

        std::memset(s_data_.data(), 0, sizeof(float) * XHAT * XHAT);
        std::memset(temp_matrix_data_.data(), 0, sizeof(float) * XHAT * XHAT);
        std::memset(temp_matrix_data1_.data(), 0, sizeof(float) * XHAT * XHAT);
        std::memset(temp_vector_data_.data(), 0, sizeof(float) * XHAT);
        std::memset(temp_vector_data1_.data(), 0, sizeof(float) * XHAT);

        // Reset dynamic dims back to defaults (fixed maximum).
        h_.numRows = Z;
        h_.numCols = XHAT;
        ht_.numRows = XHAT;
        ht_.numCols = Z;
        r_.numRows = Z;
        r_.numCols = Z;
        k_.numRows = XHAT;
        k_.numCols = Z;
        z_.numRows = Z;
        z_.numCols = 1;
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
            std::memcpy(xhat_data_.data(), xhatminus_data_.data(), sizeof(float) * XHAT);
            std::memcpy(p_data_.data(), pminus_data_.data(), sizeof(float) * XHAT * XHAT);
        }

        if (user_func5_ != nullptr)
            user_func5_(*this);

        // suppress filter excessive convergence
        for (std::size_t i = 0; i < XHAT; i++)
        {
            float &p_ii = p_data_[i * XHAT + i];
            if (p_ii < state_min_variance_[i])
            {
                p_ii = state_min_variance_[i];
            }
        }

        std::memcpy(filtered_value_.data(), xhat_data_.data(), sizeof(float) * XHAT);

        if (user_func6_ != nullptr)
            user_func6_(*this);

        return filtered_value_.data();
    }

    // Flags / configuration
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

    // Vectors
    float *FilteredValue() { return filtered_value_.data(); }
    const float *FilteredValue() const { return filtered_value_.data(); }
    float *MeasuredVector() { return measured_vector_.data(); }
    const float *MeasuredVector() const { return measured_vector_.data(); }
    float *ControlVector()
    {
        if constexpr (U > 0)
        {
            return control_vector_.data();
        }
        else
        {
            return nullptr;
        }
    }

    const float *ControlVector() const
    {
        if constexpr (U > 0)
        {
            return control_vector_.data();
        }
        else
        {
            return nullptr;
        }
    }

    // Raw data access (for EKF custom steps).
    float *XhatData() { return xhat_data_.data(); }
    float *XhatMinusData() { return xhatminus_data_.data(); }
    float *UData()
    {
        if constexpr (U > 0)
        {
            return u_data_.data();
        }
        else
        {
            return nullptr;
        }
    }
    float *ZData() { return z_data_.data(); }
    float *PData() { return p_data_.data(); }
    float *PminusData() { return pminus_data_.data(); }
    float *FData() { return f_data_.data(); }
    float *FTData() { return ft_data_.data(); }
    float *BData()
    {
        if constexpr (U > 0)
        {
            return b_data_.data();
        }
        else
        {
            return nullptr;
        }
    }
    float *HData() { return h_data_.data(); }
    float *HTData() { return ht_data_.data(); }
    float *QData() { return q_data_.data(); }
    float *RData() { return r_data_.data(); }
    float *KData() { return k_data_.data(); }

    // Matrices
    ::mat &xhat() { return xhat_; }
    ::mat &xhatminus() { return xhatminus_; }
    ::mat &u() { return u_; }
    ::mat &z() { return z_; }
    ::mat &P() { return p_; }
    ::mat &Pminus() { return pminus_; }
    ::mat &F() { return f_; }
    ::mat &FT() { return ft_; }
    ::mat &B() { return b_; }
    ::mat &H() { return h_; }
    ::mat &HT() { return ht_; }
    ::mat &Q() { return q_; }
    ::mat &R() { return r_; }
    ::mat &K() { return k_; }
    ::mat &S() { return s_; }
    ::mat &TempMatrix() { return temp_matrix_; }
    ::mat &TempMatrix1() { return temp_matrix1_; }
    ::mat &TempVector() { return temp_vector_; }
    ::mat &TempVector1() { return temp_vector1_; }

    int8_t &MatStatus() { return mat_status_; }
    std::uint8_t &MeasurementValidNum() { return measurement_valid_num_; }
    const std::array<float, XHAT> &StateMinVariance() const { return state_min_variance_; }
    std::array<float, XHAT> &StateMinVariance() { return state_min_variance_; }

private:
    void InitMatrices_()
    {
        InitializeMatrixView(xhat_, XHAT, 1, xhat_data_.data());
        InitializeMatrixView(xhatminus_, XHAT, 1, xhatminus_data_.data());
        if constexpr (U > 0)
        {
            InitializeMatrixView(u_, U, 1, u_data_.data());
            InitializeMatrixView(b_, XHAT, U, b_data_.data());
        }
        else
        {
            static float dummy = 0.0f;
            InitializeMatrixView(u_, 0, 0, &dummy);
            InitializeMatrixView(b_, 0, 0, &dummy);
        }
        InitializeMatrixView(z_, Z, 1, z_data_.data());

        InitializeMatrixView(p_, XHAT, XHAT, p_data_.data());
        InitializeMatrixView(pminus_, XHAT, XHAT, pminus_data_.data());
        InitializeMatrixView(f_, XHAT, XHAT, f_data_.data());
        InitializeMatrixView(ft_, XHAT, XHAT, ft_data_.data());
        InitializeMatrixView(h_, Z, XHAT, h_data_.data());
        InitializeMatrixView(ht_, XHAT, Z, ht_data_.data());
        InitializeMatrixView(q_, XHAT, XHAT, q_data_.data());
        InitializeMatrixView(r_, Z, Z, r_data_.data());
        InitializeMatrixView(k_, XHAT, Z, k_data_.data());

        InitializeMatrixView(s_, XHAT, XHAT, s_data_.data());
        InitializeMatrixView(temp_matrix_, XHAT, XHAT, temp_matrix_data_.data());
        InitializeMatrixView(temp_matrix1_, XHAT, XHAT, temp_matrix_data1_.data());
        InitializeMatrixView(temp_vector_, XHAT, 1, temp_vector_data_.data());
        InitializeMatrixView(temp_vector1_, XHAT, 1, temp_vector_data1_.data());
    }

    void Measure_()
    {
        // Auto adjustment not implemented in this static-size filter.
        std::memcpy(z_data_.data(), measured_vector_.data(), sizeof(float) * Z);
        std::memset(measured_vector_.data(), 0, sizeof(float) * Z);
        if constexpr (U > 0)
        {
            std::memcpy(u_data_.data(), control_vector_.data(), sizeof(float) * U);
            std::memset(control_vector_.data(), 0, sizeof(float) * U);
        }
    }

    void XhatMinusUpdate_()
    {
        if (skip_eq1_ != 0)
        {
            return;
        }

        if constexpr (U > 0)
        {
            temp_vector_.numRows = XHAT;
            temp_vector_.numCols = 1;
            mat_status_ = static_cast<int8_t>(MultiplyMatrices(f_, xhat_, temp_vector_));
            temp_vector1_.numRows = XHAT;
            temp_vector1_.numCols = 1;
            mat_status_ = static_cast<int8_t>(MultiplyMatrices(b_, u_, temp_vector1_));
            mat_status_ = static_cast<int8_t>(AddMatrices(temp_vector_, temp_vector1_, xhatminus_));
        }
        else
        {
            mat_status_ = static_cast<int8_t>(MultiplyMatrices(f_, xhat_, xhatminus_));
        }
    }

    void PminusUpdate_()
    {
        if (skip_eq2_ != 0)
        {
            return;
        }

        mat_status_ = static_cast<int8_t>(TransposeMatrix(f_, ft_));
        mat_status_ = static_cast<int8_t>(MultiplyMatrices(f_, p_, pminus_));
        temp_matrix_.numRows = pminus_.numRows;
        temp_matrix_.numCols = ft_.numCols;
        mat_status_ = static_cast<int8_t>(MultiplyMatrices(pminus_, ft_, temp_matrix_));
        mat_status_ = static_cast<int8_t>(AddMatrices(temp_matrix_, q_, pminus_));
    }

    void SetK_()
    {
        if (skip_eq3_ != 0)
        {
            return;
        }

        mat_status_ = static_cast<int8_t>(TransposeMatrix(h_, ht_));
        temp_matrix_.numRows = h_.numRows;
        temp_matrix_.numCols = pminus_.numCols;
        mat_status_ = static_cast<int8_t>(MultiplyMatrices(h_, pminus_, temp_matrix_));
        temp_matrix1_.numRows = temp_matrix_.numRows;
        temp_matrix1_.numCols = ht_.numCols;
        mat_status_ = static_cast<int8_t>(MultiplyMatrices(temp_matrix_, ht_, temp_matrix1_));
        s_.numRows = r_.numRows;
        s_.numCols = r_.numCols;
        mat_status_ = static_cast<int8_t>(AddMatrices(temp_matrix1_, r_, s_));
        mat_status_ = static_cast<int8_t>(InvertMatrix(s_, temp_matrix1_));
        temp_matrix_.numRows = pminus_.numRows;
        temp_matrix_.numCols = ht_.numCols;
        mat_status_ = static_cast<int8_t>(MultiplyMatrices(pminus_, ht_, temp_matrix_));
        mat_status_ = static_cast<int8_t>(MultiplyMatrices(temp_matrix_, temp_matrix1_, k_));
    }

    void XhatUpdate_()
    {
        if (skip_eq4_ != 0)
        {
            return;
        }

        temp_vector_.numRows = h_.numRows;
        temp_vector_.numCols = 1;
        mat_status_ = static_cast<int8_t>(MultiplyMatrices(h_, xhatminus_, temp_vector_));
        temp_vector1_.numRows = z_.numRows;
        temp_vector1_.numCols = 1;
        mat_status_ = static_cast<int8_t>(SubtractMatrices(z_, temp_vector_, temp_vector1_));
        temp_vector_.numRows = k_.numRows;
        temp_vector_.numCols = 1;
        mat_status_ = static_cast<int8_t>(MultiplyMatrices(k_, temp_vector1_, temp_vector_));
        mat_status_ = static_cast<int8_t>(AddMatrices(xhatminus_, temp_vector_, xhat_));
    }

    void PUpdate_()
    {
        if (skip_eq5_ != 0)
        {
            return;
        }

        temp_matrix_.numRows = k_.numRows;
        temp_matrix_.numCols = h_.numCols;
        temp_matrix1_.numRows = temp_matrix_.numRows;
        temp_matrix1_.numCols = pminus_.numCols;
        mat_status_ = static_cast<int8_t>(MultiplyMatrices(k_, h_, temp_matrix_));
        mat_status_ = static_cast<int8_t>(MultiplyMatrices(temp_matrix_, pminus_, temp_matrix1_));
        mat_status_ = static_cast<int8_t>(SubtractMatrices(pminus_, temp_matrix1_, p_));
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

    int8_t mat_status_ = 0;

    std::array<float, XHAT> filtered_value_{};
    std::array<float, Z> measured_vector_{};
    std::array<float, U> control_vector_{};

    std::array<float, XHAT> xhat_data_{};
    std::array<float, XHAT> xhatminus_data_{};
    std::array<float, U> u_data_{};
    std::array<float, Z> z_data_{};

    std::array<float, XHAT * XHAT> p_data_{};
    std::array<float, XHAT * XHAT> pminus_data_{};
    std::array<float, XHAT * XHAT> f_data_{};
    std::array<float, XHAT * XHAT> ft_data_{};
    std::array<float, XHAT * U> b_data_{};
    std::array<float, Z * XHAT> h_data_{};
    std::array<float, XHAT * Z> ht_data_{};
    std::array<float, XHAT * XHAT> q_data_{};
    std::array<float, Z * Z> r_data_{};
    std::array<float, XHAT * Z> k_data_{};

    std::array<float, XHAT> state_min_variance_{};

    // Workspace buffers (sized to XHAT; supports Z<=XHAT use-cases like QuaternionEKF).
    std::array<float, XHAT * XHAT> s_data_{};
    std::array<float, XHAT * XHAT> temp_matrix_data_{};
    std::array<float, XHAT * XHAT> temp_matrix_data1_{};
    std::array<float, XHAT> temp_vector_data_{};
    std::array<float, XHAT> temp_vector_data1_{};

    ::mat xhat_{};
    ::mat xhatminus_{};
    ::mat u_{};
    ::mat z_{};
    ::mat p_{};
    ::mat pminus_{};
    ::mat f_{};
    ::mat ft_{};
    ::mat b_{};
    ::mat h_{};
    ::mat ht_{};
    ::mat q_{};
    ::mat r_{};
    ::mat k_{};
    ::mat s_{};
    ::mat temp_matrix_{};
    ::mat temp_matrix1_{};
    ::mat temp_vector_{};
    ::mat temp_vector1_{};
};

} // namespace alg
