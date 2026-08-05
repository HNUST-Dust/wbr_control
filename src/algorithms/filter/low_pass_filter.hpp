/**
 * @file low_pass_filter.hpp
 * @brief 一阶低通滤波器（新增 C++ API + 旧接口 wrapper）
 *
 * 设计思路：
 * =========
 * - 用于对传感器/控制量进行一阶平滑：$y[n]=\alpha x[n] + (1-\alpha) y[n-1]$。
 * - `alg::LowPassFilter` 提供 C++ 风格接口；文件末尾 `::LowPassFilter` 为历史兼容包装。
 *
 * 参数含义：
 * =========
 * - `cutoff_hz`：截止频率（Hz）。
 * - `dt`：采样周期（s）。
 * - 计算方式：$rc = 1/(2\pi f_c)$，$\alpha = dt/(dt+rc)$。
 *
 * 注意事项：
 * =========
 * - 若 `dt<=0` 或 `cutoff_hz<=0`，会退化为“直接输出输入”（`alpha=1`），通常意味着配置错误。
 * - 该类包含内部状态，不是线程安全；建议按控制回路独立实例化。
 */

#ifndef WBR_CONTROL_ALGORITHMS_FILTER_LOW_PASS_FILTER_HPP_
#define WBR_CONTROL_ALGORITHMS_FILTER_LOW_PASS_FILTER_HPP_

#include <algorithms/common/alg_common.h>

namespace alg {

struct LowPassFilterConfig {
    float cutoff_hz = 0.0f;
    float dt = 0.0f;
};

class LowPassFilter {
public:
    constexpr LowPassFilter() = default;

    void configure(const LowPassFilterConfig& cfg)
    {
        configure(cfg.cutoff_hz, cfg.dt);
    }

    void configure(float cutoff_hz, float dt)
    {
        if (dt <= 0.0f) {
            alpha_ = 1.0f;
        } else if (cutoff_hz <= 0.0f) {
            alpha_ = 1.0f;
        } else {
            const float rc = 1.0f / (TWO_PI * cutoff_hz);
            alpha_ = dt / (dt + rc);
        }
        output_ = 0.0f;
        configured_ = true;
    }

    float update(float input)
    {
        if (!configured_) {
            return input;
        }
        output_ = alpha_ * input + (1.0f - alpha_) * output_;
        return output_;
    }

    float value() const { return output_; }

    void reset(float value = 0.0f)
    {
        output_ = value;
    }

    bool configured() const { return configured_; }

private:
    float alpha_ = 1.0f;
    float output_ = 0.0f;
    bool configured_ = false;
};

} // namespace alg

// Legacy wrapper: keep old method names to minimize churn.
class LowPassFilter {
public:
    void Init(float cutoff_freq, float dt) { impl_.configure(cutoff_freq, dt); }
    float Update(float input) { return impl_.update(input); }
    float GetOutput() const { return impl_.value(); }
    void Reset(float value = 0.0f) { impl_.reset(value); }

private:
    alg::LowPassFilter impl_{};
};

#endif /* WBR_CONTROL_ALGORITHMS_FILTER_LOW_PASS_FILTER_HPP_ */
