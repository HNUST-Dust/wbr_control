/**
 * @file alg_ols.cpp
 * @brief `utils/alg_ols.h` 的实现（固定窗口 OLS 导数/平滑）
 *
 * 设计要点：
 * =========
 * - 维护最近 `Order` 个采样点的窗口（x/y），通过最小二乘拟合得到斜率 k 与截距 b。
 * - `update()`：更新窗口并计算 k/b；`derivative()`：返回当前拟合斜率；`smooth()`：返回窗口末端的拟合值。
 * - 采用固定数组，避免动态内存。
 */

#include <algorithms/utils/alg_ols.h>

#include <cmath>
#include <cstring>

namespace alg {

void OrdinaryLeastSquares::init(uint16_t order)
{
    if (order > kMaxOrder) {
        order = kMaxOrder;
    }

    Order = order;
    Count = 0;
    k = 0.0f;
    b = 0.0f;
    StandardDeviation = 0.0f;

    std::memset(x, 0, sizeof(x));
    std::memset(y, 0, sizeof(y));
    std::memset(t, 0, sizeof(t));
}

void OLS_Init(OrdinaryLeastSquares* ols, uint16_t order)
{
    if (!ols) {
        return;
    }
    ols->init(order);
}

void OrdinaryLeastSquares::update(float deltax, float y_sample)
{
    if (Order < 2) {
        return;
    }

    const float temp = x[1];
    for (uint16_t i = 0; i < Order - 1; ++i) {
        x[i] = x[i + 1] - temp;
        y[i] = y[i + 1];
    }

    x[Order - 1] = x[Order - 2] + deltax;
    y[Order - 1] = y_sample;

    if (Count < Order) {
        Count++;
    }

    std::memset(t, 0, sizeof(t));
    for (uint16_t i = Order - Count; i < Order; ++i) {
        t[0] += x[i] * x[i];
        t[1] += x[i];
        t[2] += x[i] * y[i];
        t[3] += y[i];
    }

    const float denom = (t[0] * Order - t[1] * t[1]);
    if (denom == 0.0f) {
        k = 0.0f;
        b = 0.0f;
        return;
    }

    k = (t[2] * Order - t[1] * t[3]) / denom;
    b = (t[0] * t[3] - t[1] * t[2]) / denom;

    StandardDeviation = 0.0f;
    for (uint16_t i = Order - Count; i < Order; ++i) {
        StandardDeviation += std::fabs(k * x[i] + b - y[i]);
    }
    StandardDeviation /= static_cast<float>(Order);
}

void OLS_Update(OrdinaryLeastSquares* ols, float deltax, float y)
{
    if (!ols) {
        return;
    }
    ols->update(deltax, y);
}

float OrdinaryLeastSquares::derivative(float deltax, float y_sample)
{
    if (Order < 2) {
        return 0.0f;
    }

    const float temp = x[1];
    for (uint16_t i = 0; i < Order - 1; ++i) {
        x[i] = x[i + 1] - temp;
        y[i] = y[i + 1];
    }

    x[Order - 1] = x[Order - 2] + deltax;
    y[Order - 1] = y_sample;

    if (Count < Order) {
        Count++;
    }

    std::memset(t, 0, sizeof(t));
    for (uint16_t i = Order - Count; i < Order; ++i) {
        t[0] += x[i] * x[i];
        t[1] += x[i];
        t[2] += x[i] * y[i];
        t[3] += y[i];
    }

    const float denom = (t[0] * Order - t[1] * t[1]);
    if (denom == 0.0f) {
        k = 0.0f;
        return 0.0f;
    }

    k = (t[2] * Order - t[1] * t[3]) / denom;

    StandardDeviation = 0.0f;
    for (uint16_t i = Order - Count; i < Order; ++i) {
        StandardDeviation += std::fabs(k * x[i] + b - y[i]);
    }
    StandardDeviation /= static_cast<float>(Order);

    return k;
}

float OLS_Derivative(OrdinaryLeastSquares* ols, float deltax, float y)
{
    if (!ols) {
        return 0.0f;
    }
    return ols->derivative(deltax, y);
}

float Get_OLS_Derivative(OrdinaryLeastSquares* ols)
{
    if (!ols) {
        return 0.0f;
    }
    return ols->k;
}

float OrdinaryLeastSquares::smooth(float deltax, float y_sample)
{
    if (Order < 2) {
        return 0.0f;
    }

    const float temp = x[1];
    for (uint16_t i = 0; i < Order - 1; ++i) {
        x[i] = x[i + 1] - temp;
        y[i] = y[i + 1];
    }

    x[Order - 1] = x[Order - 2] + deltax;
    y[Order - 1] = y_sample;

    if (Count < Order) {
        Count++;
    }

    std::memset(t, 0, sizeof(t));
    for (uint16_t i = Order - Count; i < Order; ++i) {
        t[0] += x[i] * x[i];
        t[1] += x[i];
        t[2] += x[i] * y[i];
        t[3] += y[i];
    }

    const float denom = (t[0] * Order - t[1] * t[1]);
    if (denom == 0.0f) {
        k = 0.0f;
        b = 0.0f;
        return 0.0f;
    }

    k = (t[2] * Order - t[1] * t[3]) / denom;
    b = (t[0] * t[3] - t[1] * t[2]) / denom;

    StandardDeviation = 0.0f;
    for (uint16_t i = Order - Count; i < Order; ++i) {
        StandardDeviation += std::fabs(k * x[i] + b - y[i]);
    }
    StandardDeviation /= static_cast<float>(Order);

    return k * x[Order - 1] + b;
}

float OLS_Smooth(OrdinaryLeastSquares* ols, float deltax, float y)
{
    if (!ols) {
        return 0.0f;
    }
    return ols->smooth(deltax, y);
}

float Get_OLS_Smooth(OrdinaryLeastSquares* ols)
{
    if (!ols || ols->Order < 2) {
        return 0.0f;
    }
    return ols->k * ols->x[ols->Order - 1] + ols->b;
}

} // namespace alg
