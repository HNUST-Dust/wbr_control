/**
 * @file alg_pid.h
 * @brief PID 控制器
 *
 * 设计思路：
 * =========
 * - 提供可复用的 PID 实现，支持：积分限幅、积分分离、变速积分、死区、微分先行、D 低通。
 * - 对外推荐使用 C++ 接口：`alg::PidConfig + alg::Pid::configure/reset/update`。
 * - 同时保留历史接口（`Init/SetKp/...`）以减少业务侧改动成本。
 *
 * 线程模型：
 * =========
 * - `alg::Pid` 内部持有状态（积分、上次误差/输入等），不是线程安全的。
 * - 建议每个控制回路/任务持有各自实例；不要跨任务共享同一个实例（除非自行加锁）。
 *
 * 注意事项：
 * =========
 * - `update()` 计算普通误差：`target-now`。
 * - `update_angle()` 用于角度控制：内部会对角度误差进行归一化处理，适用于跨 0/跨 $\pi$ 的情况。
 * - 建议在模式切换/目标突变时调用 `reset()`，避免历史积分导致输出突跳。
 */
#ifndef WBR_CONTROL_ALGORITHMS_CONTROL_ALG_PID_H_
#define WBR_CONTROL_ALGORITHMS_CONTROL_ALG_PID_H_

/* Includes ------------------------------------------------------------------*/

#include <algorithms/math/alg_math.h>

#include <cstdint>

/* Exported types ------------------------------------------------------------*/

namespace alg {

/**
 * @brief 微分先行
 */
enum class DFirst : unsigned char {
    Disable = 0,
    Enable,
};

struct PidConfig {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float kf = 0.0f;

    float i_out_max = 0.0f;
    float out_max = 0.0f;

    float dt = 0.001f;
    float dead_zone = 0.0f;

    float i_variable_speed_A = 0.0f;
    float i_variable_speed_B = 0.0f;
    float i_separate_threshold = 0.0f;

    DFirst d_first = DFirst::Disable;
    float d_lpf_tau = 0.0f;
};

/**
 * @brief Reusable, PID算法
 *
 */
class Pid
{
public:
        void configure(const PidConfig& cfg);

        void reset();

        float update(float target, float now);
        float update_angle(float target, float now);

    void Init(float k_p, 
              float k_i,
              float k_d, 
              float k_f = 0.0f,
              float i_out_max = 0.0f, 
              float out_max = 0.0f, 
              float d_t = 0.001f, 
              float dead_zone = 0.0f, 
              float i_variable_speed_A = 0.0f, 
              float i_variable_speed_B = 0.0f, 
              float i_separate_threshold = 0.0f, 
              DFirst d_first = DFirst::Disable,
              float d_lpf_tau = 0.0f
            );

    float GetIntegralError();
    float GetOut();

    void SetKp(float k_p);
    void SetKi(float k_i);
    void SetKd(float k_d);
    void SetKf(float k_f);
    void SetIOutMax(float i_out_max);
    void SetOutMax(float out_max);
    void SetIVariableSpeedA(float variable_speed_I_A);
    void SetIVariableSpeedB(float variable_speed_I_B);
    void SetISeparateThreshold(float i_separate_threshold);
    void SetTarget(float target);
    void SetNow(float now);
    void SetIntegralError(float integral_error);

    void CalculatePeriodElapsedCallback();

    void CalculateAnglePid();

protected:
    //初始化相关常量

    // PID计时器周期, s
    float d_t_;
    //死区, Error在其绝对值内不输出
    float dead_zone_;
    //微分先行
    DFirst d_first_;
    // d的低通滤波系数
    float d_lpf_tau_;
    float d_lpf_output_;

    //常量

    //内部变量

    //之前的当前值
    float pre_now_ = 0.0f;
    //之前的目标值
    float pre_target_ = 0.0f;
    //之前的输出值
    float pre_out_ = 0.0f;
    //前向误差
    float pre_error_ = 0.0f;

    //读变量

    //输出值
    float out_ = 0.0f;

    //写变量

    // PID的P
    float k_p_ = 0.0f;
    // PID的I
    float k_i_ = 0.0f;
    // PID的D
    float k_d_ = 0.0f;
    //前馈
    float k_f_ = 0.0f;

    //积分限幅, 0为不限制
    float i_out_max_ = 0;
    //输出限幅, 0为不限制
    float out_max_ = 0;

    //变速积分定速内段阈值, 0为不限制
    float i_variable_speed_A_ = 0.0f;
    //变速积分变速区间, 0为不限制
    float i_variable_speed_B_ = 0.0f;
    //积分分离阈值，需为正数, 0为不限制
    float i_separate_threshold_ = 0.0f;

    //目标值
    float target_ = 0.0f;
    //当前值
    float now_ = 0.0f;

    //读写变量

    //积分值
    float integral_error_ = 0.0f;

    //内部函数
};

}  // namespace alg

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) HNUST-DUST **************************/
