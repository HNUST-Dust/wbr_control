/**
 * @file alg_pid.cpp
 * @brief PID 控制器实现（对应 `control/alg_pid.h`）
 *
 * 设计要点：
 * =========
 * - `Pid::update()` 走标准 PID：误差 = target-now；再执行积分/微分/前馈与限幅。
 * - `Pid::update_angle()` 用于角度控制：内部会处理跨零/跨 $\pi$ 的角度误差归一化。
 * - 支持：死区、积分限幅、积分分离、变速积分、微分先行、D 低通（由配置项控制）。
 *
 * 注意事项：
 * =========
 * - 本实现依赖 `math/alg_math.h` 的部分角度工具函数；因此角度单位需与调用侧一致。
 * - PID 实例包含历史状态，不应跨任务共享。
 */

/* Includes ------------------------------------------------------------------*/

#include <algorithms/control/alg_pid.h>

namespace alg {

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief PID初始化
 *
 * @param k_p P值
 * @param k_i I值
 * @param k_d D值
 * @param k_f 前馈
 * @param i_out_max 积分限幅
 * @param out_max 输出限幅
 * @param d_t 时间片长度
 */
void Pid::Init(
    float k_p, 
    float k_i, 
    float k_d, 
    float k_f, 
    float i_out_max, 
    float out_max, 
    float d_t, 
    float dead_zone, 
    float i_variable_speed_A, 
    float i_variable_speed_B, 
    float i_separate_threshold, 
    DFirst d_first,
    float d_lpf_tau
)
{
    k_p_ = k_p;
    k_i_ = k_i;
    k_d_ = k_d;
    k_f_ = k_f;
    i_out_max_ = i_out_max;
    out_max_ = out_max;
    d_t_ = d_t;
    dead_zone_ = dead_zone;
    i_variable_speed_A_ = i_variable_speed_A;
    i_variable_speed_B_ = i_variable_speed_B;
    i_separate_threshold_ = i_separate_threshold;
    d_first_ = d_first;
    d_lpf_tau_ = d_lpf_tau;
}

void Pid::configure(const PidConfig& cfg)
{
    Init(cfg.kp,
         cfg.ki,
         cfg.kd,
         cfg.kf,
         cfg.i_out_max,
         cfg.out_max,
         cfg.dt,
         cfg.dead_zone,
         cfg.i_variable_speed_A,
         cfg.i_variable_speed_B,
         cfg.i_separate_threshold,
         cfg.d_first,
         cfg.d_lpf_tau);
}

void Pid::reset()
{
    pre_now_ = 0.0f;
    pre_target_ = 0.0f;
    pre_out_ = 0.0f;
    pre_error_ = 0.0f;

    out_ = 0.0f;
    integral_error_ = 0.0f;
    d_lpf_output_ = 0.0f;
}

float Pid::update(float target, float now)
{
    SetTarget(target);
    SetNow(now);
    CalculatePeriodElapsedCallback();
    return GetOut();
}

float Pid::update_angle(float target, float now)
{
    SetTarget(target);
    SetNow(now);
    CalculateAnglePid();
    return GetOut();
}

/**
 * @brief 获取输出值
 *
 * @return float 输出值
 */
float Pid::GetIntegralError()
{
    return (integral_error_);
}

/**
 * @brief 获取输出值
 *
 * @return float 输出值
 */
float Pid::GetOut()
{
    return (out_);
}

/**
 * @brief 设定PID的P
 *
 * @param k_p_ PID的P
 */
void Pid::SetKp(float k_p)
{
    k_p_ = k_p;
}

/**
 * @brief 设定PID的I
 *
 * @param k_i PID的I
 */
void Pid::SetKi(float k_i)
{
    k_i_ = k_i;
}

/**
 * @brief 设定PID的D
 *
 * @param k_d PID的D
 */
void Pid::SetKd(float k_d)
{
    k_d_ = k_d;
}

/**
 * @brief 设定前馈
 *
 * @param k_d 前馈
 */
void Pid::SetKf(float k_f)
{
    k_f_ = k_f;
}

/**
 * @brief 设定积分限幅, 0为不限制
 *
 * @param i_out_max 积分限幅, 0为不限制
 */
void Pid::SetIOutMax(float i_out_max)
{
    i_out_max_ = i_out_max;
}

/**
 * @brief 设定输出限幅, 0为不限制
 *
 * @param out_max 输出限幅, 0为不限制
 */
void Pid::SetOutMax(float out_max)
{
    out_max_ = out_max;
}

/**
 * @brief 设定定速内段阈值, 0为不限制
 *
 * @param i_variable_speed_A 定速内段阈值, 0为不限制
 */
void Pid::SetIVariableSpeedA(float i_variable_speed_A)
{
    i_variable_speed_A_ = i_variable_speed_A;
}

/**
 * @brief 设定变速区间, 0为不限制
 *
 * @param i_variable_speed_B 变速区间, 0为不限制
 */
void Pid::SetIVariableSpeedB(float i_variable_speed_B)
{
    i_variable_speed_B_ = i_variable_speed_B;
}

/**
 * @brief 设定积分分离阈值，需为正数, 0为不限制
 *
 * @param i_separate_threshold 积分分离阈值，需为正数, 0为不限制
 */
void Pid::SetISeparateThreshold(float i_separate_threshold)
{
    i_separate_threshold_ = i_separate_threshold;
}

/**
 * @brief 设定目标值
 *
 * @param target 目标值
 */
void Pid::SetTarget(float target)
{
    target_ = target;
}

/**
 * @brief 设定当前值
 *
 * @param now 当前值
 */
void Pid::SetNow(float now)
{
    now_ = now;
}

/**
 * @brief 设定积分, 一般用于积分清零
 *
 * @param integral_error 积分值
 */
void Pid::SetIntegralError(float integral_error)
{
    integral_error_ = integral_error;
}

/**
 * @brief PID调整值, 计算周期与d_t_相同
 *
 * @return float 输出值
 */
void Pid::CalculatePeriodElapsedCallback()
{
    // P输出
    float p_out = 0.0f;
    // I输出
    float i_out = 0.0f;
    // D输出
    float d_out = 0.0f;
    // F输出
    float f_out = 0.0f;
    // 误差
    float error;
    // 绝对值误差
    float abs_error;
    // 线性变速积分
    float speed_ratio = 1.0f;

    float d_raw;  // 未滤波的微分信号
    float alpha;  // 滤波系数
    
    error = target_ - now_;
    abs_error = math_abs(error);

    // 判断死区
    if (abs_error < dead_zone_)
    {
        target_ = now_;
        error = 0.0f;
        abs_error = 0.0f;
    }
    else if (error > 0.0f && abs_error > dead_zone_)
    {
        error -= dead_zone_;
    }
    else if (error < 0.0f && abs_error > dead_zone_)
    {
        error += dead_zone_;
    }

    // 计算p项

    p_out = k_p_ * error;

    // 计算i项

    if (i_variable_speed_A_ == 0.0f && i_variable_speed_B_ == 0.0f)
    {
        // 非变速积分
        speed_ratio = 1.0f;
    }
    else
    {
        // 变速积分
        if (abs_error <= i_variable_speed_A_)
        {
            speed_ratio = 1.0f;
        }
        else if (i_variable_speed_A_ < abs_error && abs_error < i_variable_speed_B_)
        {
            speed_ratio = (i_variable_speed_B_ - abs_error) / (i_variable_speed_B_ - i_variable_speed_A_);
        }
        else if (abs_error >= i_variable_speed_B_)
        {
            speed_ratio = 0.0f;
        }
    }
    // 积分限幅
    if (i_out_max_ != 0.0f)
    {
        math_constrain(&integral_error_, -i_out_max_ / k_i_, i_out_max_ / k_i_);
    }
    if (i_separate_threshold_ == 0.0f)
    {
        // 没有积分分离
        integral_error_ += speed_ratio * d_t_ * error;
        i_out = k_i_ * integral_error_;
    }
    else
    {
        // 有积分分离
        if (abs_error < i_separate_threshold_)
        {
            // 不在积分分离区间上
            integral_error_ += speed_ratio * d_t_ * error;
            i_out = k_i_ * integral_error_;
        }
        else
        {
            // 在积分分离区间上
            integral_error_ = 0.0f;
            i_out = 0.0f;
        }
    }

    // 计算d项

    if (d_first_ == DFirst::Disable)
    {
        d_raw = k_d_ * (error - pre_error_) / d_t_;
    }
    else
    {
        d_raw = -k_d_ * (now_ - pre_now_) / d_t_;
    }

    // 一阶低通滤波器: y_k = α*y_{k-1} + (1-α)*x_k
    if (d_lpf_tau_ > 0.0f)
    {
        alpha = d_lpf_tau_ / (d_lpf_tau_ + d_t_);
        d_out = alpha * d_lpf_output_ + (1.0f - alpha) * d_raw;
        d_lpf_output_ = d_out;  // 更新滤波输出缓存
    }
    else
    {
        // 没设滤波常数则直接用原始值
        d_out = d_raw;
    }

    // 计算前馈

    f_out = k_f_ * (target_ - pre_target_);

    // 计算输出
    out_ = p_out + i_out + d_out + f_out;

    // 输出限幅
    if (out_max_ != 0.0f)
    {
        math_constrain(&out_, -out_max_, out_max_);
    }

    // 善后工作
    pre_now_ = now_;
    pre_target_ = target_;
    pre_out_ = out_;
    pre_error_ = error;
}

/**
 * @brief 带角度过零处理（劣弧解算）的 PID 计算函数
 *
 * @note 适用于输入为角度（范围 -π ~ π）的云台位置环
 *       输出为角速度或电流
 */
void Pid::CalculateAnglePid()
{
    float p_out = 0.0f;
    float i_out = 0.0f;
    float d_out = 0.0f;
    float f_out = 0.0f;
    float error;
    float abs_error;
    float speed_ratio;

    /*----------------------------------------
     * 1. 劣弧角度差计算 (-π, π)
     *----------------------------------------*/
    error = target_ - now_;

    if (error > PI)
        error -= 2.0f * PI;
    else if (error < -PI)
        error += 2.0f * PI;

    abs_error = math_abs(error);

    /*----------------------------------------
     * 2. 死区判断
     *----------------------------------------*/
    if (abs_error < dead_zone_)
    {
        target_ = now_;
        error = 0.0f;
        abs_error = 0.0f;
    }
    else if (error > 0.0f && abs_error > dead_zone_)
    {
        error -= dead_zone_;
    }
    else if (error < 0.0f && abs_error > dead_zone_)
    {
        error += dead_zone_;
    }

    /*----------------------------------------
     * 3. P项
     *----------------------------------------*/
    p_out = k_p_ * error;

    /*----------------------------------------
     * 4. I项（带变速积分 + 可选积分清零）
     *----------------------------------------*/
    if (i_variable_speed_A_ == 0.0f && i_variable_speed_B_ == 0.0f)
    {
        speed_ratio = 1.0f;
    }
    else
    {
        if (abs_error <= i_variable_speed_A_)
        {
            speed_ratio = 1.0f;
        }
        else if (i_variable_speed_A_ < abs_error && abs_error < i_variable_speed_B_)
        {
            speed_ratio = (i_variable_speed_B_ - abs_error) /
                          (i_variable_speed_B_ - i_variable_speed_A_);
        }
        else
        {
            speed_ratio = 0.0f;
        }
    }

    // 积分限幅
    if (i_out_max_ != 0.0f)
    {
        math_constrain(&integral_error_, -i_out_max_ / k_i_, i_out_max_ / k_i_);
    }

    // 当输出饱和或误差反向时清零（防卡角）
    if ((out_ >= out_max_ || out_ <= -out_max_) ||
        ((pre_error_ > 0 && error < 0) || (pre_error_ < 0 && error > 0)))
    {
        integral_error_ = 0.0f;
    }

    if (i_separate_threshold_ == 0.0f)
    {
        integral_error_ += speed_ratio * d_t_ * error;
        i_out = k_i_ * integral_error_;
    }
    else
    {
        if (abs_error < i_separate_threshold_)
        {
            integral_error_ += speed_ratio * d_t_ * error;
            i_out = k_i_ * integral_error_;
        }
        else
        {
            integral_error_ = 0.0f;
            i_out = 0.0f;
        }
    }

    /*----------------------------------------
     * 5. D项
     *----------------------------------------*/
    if (d_first_ == DFirst::Disable)
    {
        d_out = k_d_ * (error - pre_error_) / d_t_;
    }
    else
    {
        d_out = -k_d_ * (now_ - pre_now_) / d_t_;
    }

    /*----------------------------------------
     * 6. F项（前馈）
     *----------------------------------------*/
    f_out = k_f_ * (target_ - pre_target_);

    /*----------------------------------------
     * 7. 输出合成与限幅
     *----------------------------------------*/
    out_ = p_out + i_out + d_out + f_out;

    if (out_max_ != 0.0f)
    {
        math_constrain(&out_, -out_max_, out_max_);
    }

    /*----------------------------------------
     * 8. 善后
     *----------------------------------------*/
    pre_now_ = now_;
    pre_target_ = target_;
    pre_out_ = out_;
    pre_error_ = error;
}

}  // namespace alg

/************************ COPYRIGHT(C) HNUST-DUST **************************/
