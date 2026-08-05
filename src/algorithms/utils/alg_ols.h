/**
 * @file alg_ols.h
 * @brief Ordinary Least Squares（用于导数估计/平滑）的固定数组实现
 *
 * 设计思路：
 * =========
 * - 用固定大小数组（`kMaxOrder`）替代动态分配，适合嵌入式实时环境。
 * - 提供面向对象接口（`alg::OrdinaryLeastSquares`）与历史 C 风格 wrapper（`OLS_*`）。
 *
 * 线程模型：
 * =========
 * - 实例包含内部窗口数据与拟合结果，不可重入；每个任务/通道应持有独立实例。
 */

#pragma once

// Public fixed-storage ordinary least-squares interface.

#include <cstdint>

namespace alg {

struct OrdinaryLeastSquares {
    static constexpr uint16_t kMaxOrder = 16;

    uint16_t Order = 0;
    uint32_t Count = 0;

    float x[kMaxOrder] = {0};
    float y[kMaxOrder] = {0};

    float k = 0.0f;
    float b = 0.0f;
    float StandardDeviation = 0.0f;
    float t[4] = {0};

    void init(uint16_t order);
    void update(float deltax, float y_sample);
    float derivative(float deltax, float y_sample);
    float smooth(float deltax, float y_sample);
    float last_derivative() const { return k; }
    float last_smooth() const { return (Order >= 2) ? (k * x[Order - 1] + b) : 0.0f; }
};

void OLS_Init(OrdinaryLeastSquares* ols, uint16_t order);
void OLS_Update(OrdinaryLeastSquares* ols, float deltax, float y);
float OLS_Derivative(OrdinaryLeastSquares* ols, float deltax, float y);
float OLS_Smooth(OrdinaryLeastSquares* ols, float deltax, float y);
float Get_OLS_Derivative(OrdinaryLeastSquares* ols);
float Get_OLS_Smooth(OrdinaryLeastSquares* ols);

} // namespace alg

// Legacy alias/exports
using Ordinary_Least_Squares_t = alg::OrdinaryLeastSquares;
using alg::OLS_Init;
using alg::OLS_Update;
using alg::OLS_Derivative;
using alg::OLS_Smooth;
using alg::Get_OLS_Derivative;
using alg::Get_OLS_Smooth;
