/**
 ******************************************************************************
 * @file    QuaternionEKF.c
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2022/3/8
 * @brief   attitude update with gyro bias estimate and chi-square test
 ******************************************************************************
 * @attention
 * 1st order LPF transfer function:
 *     1
 *  ———————
 *  as + 1
 ******************************************************************************
 */
#include <algorithms/MahonyAHRS.h>
#include <algorithms/estimation/QuaternionEKF.h>

#include <Eigen/Core>
#include <Eigen/LU>

#include <cmath>
#include <cstddef>
#include <cstring>
#include <type_traits>

namespace {
// 初始协方差 P0，按行优先存储（与论文/旧实现一致）。
constexpr float imu_quaternion_ekf_p_const[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                                  0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                                  0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                                  0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                                  0.1, 0.1, 0.1, 0.1, 100, 0.1,
                                                  0.1, 0.1, 0.1, 0.1, 0.1, 100};

// 将行优先存储的初始协方差常量复制为 Eigen 列优先矩阵。
Eigen::Matrix<float, 6, 6> MakeInitialCovariance()
{
    Eigen::Matrix<float, 6, 6> p;
    for (int r = 0; r < 6; ++r)
    {
        for (int c = 0; c < 6; ++c)
        {
            p(r, c) = imu_quaternion_ekf_p_const[r * 6 + c];
        }
    }
    return p;
}

} // namespace

alg::QuaternionEkf::~QuaternionEkf()
{
}

alg::QuaternionEkf::QekfIns &alg::QuaternionEkf::InsFromKf(ImuKf *kf)
{
    static_assert(std::is_standard_layout_v<QekfIns>, "QekfIns must be standard-layout for offsetof/container_of");
    auto *bytes = reinterpret_cast<std::uint8_t *>(kf);
    auto *ins = reinterpret_cast<QekfIns *>(bytes - offsetof(QekfIns, imu_quaternion_ekf));
    return *ins;
}

/**
 * @brief 用于更新线性化后的状态转移矩阵F右上角的一个4x2分块矩阵,稍后用于协方差矩阵P的更新;
 *        并对零漂的方差进行限制,防止过度收敛并限幅防止发散
 */
void alg::QuaternionEkf::FLinearizationPFadingCb(ImuKf &kf)
{
    auto &ins = InsFromKf(&kf);
    volatile float q0, q1, q2, q3;
    volatile float q_inv_norm;

    q0 = kf.xhatminus()(0);
    q1 = kf.xhatminus()(1);
    q2 = kf.xhatminus()(2);
    q3 = kf.xhatminus()(3);

    // quaternion normalize
    q_inv_norm = alg::MahonyAhrs::InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    for (uint8_t i = 0; i < 4; i++)
    {
        kf.xhatminus()(i) *= q_inv_norm;
    }

    // F 右上角 4x2 分块（四元数对零漂的耦合）。
    kf.F()(0, 4) = (q1 * ins.dt) * 0.5f;
    kf.F()(0, 5) = q2 * ins.dt * 0.5f;

    kf.F()(1, 4) = -q0 * ins.dt * 0.5f;
    kf.F()(1, 5) = q3 * ins.dt * 0.5f;

    kf.F()(2, 4) = -q3 * ins.dt * 0.5f;
    kf.F()(2, 5) = -q0 * ins.dt * 0.5f;

    kf.F()(3, 4) = q2 * ins.dt * 0.5f;
    kf.F()(3, 5) = -q1 * ins.dt * 0.5f;

    // fading filter,防止零飘参数过度收敛
    kf.P()(4, 4) *= ins.lambda;
    kf.P()(5, 5) *= ins.lambda;

    // 限幅,防止发散
    if (kf.P()(4, 4) > 10000)
    {
        kf.P()(4, 4) = 10000;
    }
    if (kf.P()(5, 5) > 10000)
    {
        kf.P()(5, 5) = 10000;
    }
}

/**
 * @brief 在工作点处计算观测函数h(x)的Jacobi矩阵H
 */
void alg::QuaternionEkf::SetHCb(ImuKf &kf)
{
    volatile float double_q0, double_q1, double_q2, double_q3;

    double_q0 = 2 * kf.xhatminus()(0);
    double_q1 = 2 * kf.xhatminus()(1);
    double_q2 = 2 * kf.xhatminus()(2);
    double_q3 = 2 * kf.xhatminus()(3);

    kf.H().setZero();

    kf.H()(0, 0) = -double_q2;
    kf.H()(0, 1) = double_q3;
    kf.H()(0, 2) = -double_q0;
    kf.H()(0, 3) = double_q1;

    kf.H()(1, 0) = double_q1;
    kf.H()(1, 1) = double_q0;
    kf.H()(1, 2) = double_q3;
    kf.H()(1, 3) = double_q2;

    kf.H()(2, 0) = double_q0;
    kf.H()(2, 1) = -double_q1;
    kf.H()(2, 2) = -double_q2;
    kf.H()(2, 3) = double_q3;
}

/**
 * @brief 利用观测值和先验估计得到最优的后验估计
 *        加入了卡方检验以判断融合加速度的条件是否满足
 *        同时引入发散保护保证恶劣工况下的必要量测更新
 */
void alg::QuaternionEkf::XhatUpdateCb(ImuKf &kf)
{
    auto &ins = InsFromKf(&kf);
    volatile float q0, q1, q2, q3;

    q0 = kf.xhatminus()(0);
    q1 = kf.xhatminus()(1);
    q2 = kf.xhatminus()(2);
    q3 = kf.xhatminus()(3);

    // 创新协方差 S = H P^- H^T + R，及其逆。
    kf.S() = kf.H() * kf.Pminus() * kf.H().transpose() + kf.R();
    const Eigen::Matrix<float, 3, 3> s_inv = kf.S().inverse();

    // 计算预测得到的重力加速度方向(通过姿态获取的)
    Eigen::Matrix<float, 3, 1> h_pred;
    h_pred(0) = 2 * (q1 * q3 - q0 * q2);
    h_pred(1) = 2 * (q0 * q1 + q2 * q3);
    h_pred(2) = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 计算预测值和各个轴的方向余弦
    for (uint8_t i = 0; i < 3; i++)
    {
        ins.orientation_cosine[i] = std::cos(std::fabs(h_pred(i)));
    }

    // 利用加速度计数据修正：创新 = z - h(x^-)
    const Eigen::Matrix<float, 3, 1> innovation = kf.z() - h_pred;

    // chi-square test,卡方检验：chi_square = innovation^T S^-1 innovation
    ins.chi_square = innovation.dot(s_inv * innovation);
    // rk is small,filter converged/converging
    if (ins.chi_square < 0.5f * ins.chi_square_test_threshold)
    {
        ins.converge_flag = 1;
    }
    // rk is bigger than thre but once converged
    if (ins.chi_square > ins.chi_square_test_threshold && ins.converge_flag)
    {
        if (ins.stable_flag)
        {
            ins.error_count++; // 载体静止时仍无法通过卡方检验
        }
        else
        {
            ins.error_count = 0;
        }

        if (ins.error_count > 50)
        {
            // 滤波器发散
            ins.converge_flag = 0;
            kf.SetSkipEq5(false); // step-5 is cov mat P updating
        }
        else
        {
            //  残差未通过卡方检验 仅预测
            //  xhat(k) = xhat'(k)
            //  P(k) = P'(k)
            kf.xhat() = kf.xhatminus();
            kf.P() = kf.Pminus();
            kf.SetSkipEq5(true); // part5 is P updating
            return;
        }
    }
    else // if divergent or rk is not that big/acceptable,use adaptive gain
    {
        // scale adaptive,rk越小则增益越大,否则更相信预测值
        if (ins.chi_square > 0.1f * ins.chi_square_test_threshold && ins.converge_flag)
        {
            ins.adaptive_gain_scale = (ins.chi_square_test_threshold - ins.chi_square) / (0.9f * ins.chi_square_test_threshold);
        }
        else
        {
            ins.adaptive_gain_scale = 1;
        }
        ins.error_count = 0;
        kf.SetSkipEq5(false);
    }

    // cal kf-gain K = P^- H^T S^-1
    kf.K() = kf.Pminus() * kf.H().transpose() * s_inv;

    // implement adaptive
    kf.K() *= ins.adaptive_gain_scale;
    for (uint8_t i = 4; i < 6; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            kf.K()(i, j) *= ins.orientation_cosine[i - 4] * (1 / 1.5707963f); // 1 rad
        }
    }

    // 后验估计：xhat = xhatminus + K * innovation
    Eigen::Matrix<float, 6, 1> correction = kf.K() * innovation;

    // 零漂修正限幅,一般不会有过大的漂移
    if (ins.converge_flag)
    {
        for (uint8_t i = 4; i < 6; i++)
        {
            if (correction(i) > 1e-2f * ins.dt)
            {
                correction(i) = 1e-2f * ins.dt;
            }
            if (correction(i) < -1e-2f * ins.dt)
            {
                correction(i) = -1e-2f * ins.dt;
            }
        }
    }

    // 不修正yaw轴数据
    correction(3) = 0;
    kf.xhat() = kf.xhatminus() + correction;
}

/**
 * @brief EKF观测环节,其实就是把数据复制一下
 */
void alg::QuaternionEkf::ObserveCb(ImuKf &kf)
{
    auto &ins = InsFromKf(&kf);
    // 注意：Eigen 按列优先存储，快照布局与旧的行优先不同；这些字段当前仅供调试，无人读取。
    std::memcpy(ins.imu_quaternion_ekf_p, kf.P().data(), sizeof(ins.imu_quaternion_ekf_p));
    std::memcpy(ins.imu_quaternion_ekf_k, kf.K().data(), sizeof(ins.imu_quaternion_ekf_k));
    std::memcpy(ins.imu_quaternion_ekf_h, kf.H().data(), sizeof(ins.imu_quaternion_ekf_h));
}

namespace alg {

void QuaternionEkf::Init(const Params &params)
{
    ins_.imu_quaternion_ekf.Reset();

    ins_.initialized = 1;
    ins_.q1 = params.process_noise_quat_;
    ins_.q2 = params.process_noise_gyro_bias_;
    ins_.r = params.measure_noise_accel_;
    ins_.chi_square_test_threshold = 1e-8;
    ins_.converge_flag = 0;
    ins_.error_count = 0;
    ins_.update_count = 0;
    ins_.dt = params.dt_;
    ins_.acc_lpf_coef = (params.accel_lpf_coef_ < 0.0f) ? 0.0f : params.accel_lpf_coef_;
    ins_.yaw_round_count = 0;
    ins_.yaw_angle_last = 0.0f;
    ins_.yaw_total_angle = 0.0f;
    ins_.adaptive_gain_scale = 1.0f;

    float lambda = params.fading_lambda_;
    if (lambda > 1)
    {
        lambda = 1;
    }
    ins_.lambda = 1.f / lambda; //倒数

    // 姿态初始化
    ins_.imu_quaternion_ekf.xhat()(0) = 1;
    ins_.imu_quaternion_ekf.xhat()(1) = 0;
    ins_.imu_quaternion_ekf.xhat()(2) = 0;
    ins_.imu_quaternion_ekf.xhat()(3) = 0;

    // 自定义函数初始化,用于扩展或增加kf的基础功能
    ins_.imu_quaternion_ekf.SetUserFunc0(&ObserveCb);
    ins_.imu_quaternion_ekf.SetUserFunc1(&FLinearizationPFadingCb);
    ins_.imu_quaternion_ekf.SetUserFunc2(&SetHCb);
    ins_.imu_quaternion_ekf.SetUserFunc3(&XhatUpdateCb);

    // 设定标志位,用自定函数替换kf标准步骤中的SetK(计算增益)以及xhatupdate(后验估计/融合)
    ins_.imu_quaternion_ekf.SetSkipEq3(true);
    ins_.imu_quaternion_ekf.SetSkipEq4(true);

    ins_.imu_quaternion_ekf.F().setIdentity();
    ins_.imu_quaternion_ekf.P() = MakeInitialCovariance();
    std::memcpy(ins_.imu_quaternion_ekf_p, imu_quaternion_ekf_p_const, sizeof(ins_.imu_quaternion_ekf_p));
    std::memset(ins_.imu_quaternion_ekf_k, 0, sizeof(ins_.imu_quaternion_ekf_k));
    std::memset(ins_.imu_quaternion_ekf_h, 0, sizeof(ins_.imu_quaternion_ekf_h));
}

void QuaternionEkf::Reset()
{
    ins_.imu_quaternion_ekf.Reset();

    std::memcpy(ins_.imu_quaternion_ekf_p, imu_quaternion_ekf_p_const, sizeof(ins_.imu_quaternion_ekf_p));

    ins_.converge_flag = 0;
    ins_.error_count = 0;
    ins_.update_count = 0;
    ins_.yaw_round_count = 0;
    ins_.yaw_angle_last = 0.0f;
    ins_.yaw_total_angle = 0.0f;
    ins_.adaptive_gain_scale = 1.0f;

    // 姿态初始化
    ins_.imu_quaternion_ekf.xhat()(0) = 1;
    ins_.imu_quaternion_ekf.xhat()(1) = 0;
    ins_.imu_quaternion_ekf.xhat()(2) = 0;
    ins_.imu_quaternion_ekf.xhat()(3) = 0;

    // 设定标志位,用自定函数替换kf标准步骤中的SetK(计算增益)以及xhatupdate(后验估计/融合)
    ins_.imu_quaternion_ekf.SetSkipEq3(true);
    ins_.imu_quaternion_ekf.SetSkipEq4(true);

    ins_.imu_quaternion_ekf.F().setIdentity();
    ins_.imu_quaternion_ekf.P() = MakeInitialCovariance();
}

void QuaternionEkf::Update(float gx, float gy, float gz, float ax, float ay, float az)
{
    // 0.5(Ohm-Ohm^bias)*deltaT,用于更新工作点处的状态转移F矩阵
    volatile float half_gx_dt, half_gy_dt, half_gz_dt;
    volatile float accel_inv_norm;

    ins_.gyro[0] = gx - ins_.gyro_bias[0];
    ins_.gyro[1] = gy - ins_.gyro_bias[1];
    ins_.gyro[2] = gz - ins_.gyro_bias[2];

    // set F：先复位为单位阵（右下角 2x2 单位阵对应零漂随机游走），再填四元数动力学左上角 4x4 分块。
    half_gx_dt = 0.5f * ins_.gyro[0] * ins_.dt;
    half_gy_dt = 0.5f * ins_.gyro[1] * ins_.dt;
    half_gz_dt = 0.5f * ins_.gyro[2] * ins_.dt;

    ins_.imu_quaternion_ekf.F().setIdentity();

    ins_.imu_quaternion_ekf.F()(0, 1) = -half_gx_dt;
    ins_.imu_quaternion_ekf.F()(0, 2) = -half_gy_dt;
    ins_.imu_quaternion_ekf.F()(0, 3) = -half_gz_dt;

    ins_.imu_quaternion_ekf.F()(1, 0) = half_gx_dt;
    ins_.imu_quaternion_ekf.F()(1, 2) = half_gz_dt;
    ins_.imu_quaternion_ekf.F()(1, 3) = -half_gy_dt;

    ins_.imu_quaternion_ekf.F()(2, 0) = half_gy_dt;
    ins_.imu_quaternion_ekf.F()(2, 1) = -half_gz_dt;
    ins_.imu_quaternion_ekf.F()(2, 3) = half_gx_dt;

    ins_.imu_quaternion_ekf.F()(3, 0) = half_gz_dt;
    ins_.imu_quaternion_ekf.F()(3, 1) = half_gy_dt;
    ins_.imu_quaternion_ekf.F()(3, 2) = -half_gx_dt;

    // accel low pass filter,加速度过一下低通滤波平滑数据,降低撞击和异常的影响
    if (ins_.update_count == 0) // 如果是第一次进入,需要初始化低通滤波
    {
        ins_.accel[0] = ax;
        ins_.accel[1] = ay;
        ins_.accel[2] = az;
        ins_.update_count++;
    }
    const float temp_quick = 1.f / (ins_.dt + ins_.acc_lpf_coef); // 加速
    ins_.accel[0] = ins_.accel[0] * ins_.acc_lpf_coef * temp_quick + ax * ins_.dt * temp_quick;
    ins_.accel[1] = ins_.accel[1] * ins_.acc_lpf_coef * temp_quick + ay * ins_.dt * temp_quick;
    ins_.accel[2] = ins_.accel[2] * ins_.acc_lpf_coef * temp_quick + az * ins_.dt * temp_quick;

    // set z,单位化重力加速度向量
    ins_.accl_norm = std::sqrt(ins_.accel[0] * ins_.accel[0] + ins_.accel[1] * ins_.accel[1] + ins_.accel[2] * ins_.accel[2]);
    accel_inv_norm = 1.0f / ins_.accl_norm;

    ins_.imu_quaternion_ekf.MeasuredVector()(0) = ins_.accel[0] * accel_inv_norm; // 用加速度向量更新量测值
    ins_.imu_quaternion_ekf.MeasuredVector()(1) = ins_.accel[1] * accel_inv_norm;
    ins_.imu_quaternion_ekf.MeasuredVector()(2) = ins_.accel[2] * accel_inv_norm;

    // get body state
    ins_.gyro_norm = std::sqrt(ins_.gyro[0] * ins_.gyro[0] + ins_.gyro[1] * ins_.gyro[1] + ins_.gyro[2] * ins_.gyro[2]);

    // 如果角速度小于阈值且加速度处于设定范围内,认为运动稳定,加速度可以用于修正角速度
    if (ins_.gyro_norm < 0.3f && ins_.accl_norm > 9.8f - 0.5f && ins_.accl_norm < 9.8f + 0.5f)
    {
        ins_.stable_flag = 1;
    }
    else
    {
        ins_.stable_flag = 0;
    }

    // set Q R,过程噪声和观测噪声矩阵
    ins_.imu_quaternion_ekf.Q()(0, 0) = ins_.q1 * ins_.dt;
    ins_.imu_quaternion_ekf.Q()(1, 1) = ins_.q1 * ins_.dt;
    ins_.imu_quaternion_ekf.Q()(2, 2) = ins_.q1 * ins_.dt;
    ins_.imu_quaternion_ekf.Q()(3, 3) = ins_.q1 * ins_.dt;
    ins_.imu_quaternion_ekf.Q()(4, 4) = ins_.q2 * ins_.dt;
    ins_.imu_quaternion_ekf.Q()(5, 5) = ins_.q2 * ins_.dt;
    ins_.imu_quaternion_ekf.R()(0, 0) = ins_.r;
    ins_.imu_quaternion_ekf.R()(1, 1) = ins_.r;
    ins_.imu_quaternion_ekf.R()(2, 2) = ins_.r;

    ins_.imu_quaternion_ekf.Update();

    // 获取融合后的数据,包括四元数和xy零飘值
    ins_.q[0] = ins_.imu_quaternion_ekf.FilteredValue()(0);
    ins_.q[1] = ins_.imu_quaternion_ekf.FilteredValue()(1);
    ins_.q[2] = ins_.imu_quaternion_ekf.FilteredValue()(2);
    ins_.q[3] = ins_.imu_quaternion_ekf.FilteredValue()(3);

    ins_.roll = std::atan2(
        ins_.q[0] * ins_.q[1] + ins_.q[2] * ins_.q[3],
        0.5f - ins_.q[1] * ins_.q[1] - ins_.q[2] * ins_.q[2]
    );

    ins_.roll *= 57.29578f;
    ins_.pitch = 57.29578f * std::asin(-2.0f * (ins_.q[1] * ins_.q[3] - ins_.q[0] * ins_.q[2]));
    ins_.yaw = std::atan2(ins_.q[1] * ins_.q[2] + ins_.q[0] * ins_.q[3],
                      0.5f - ins_.q[2] * ins_.q[2] - ins_.q[3] * ins_.q[3]);
    ins_.yaw *= 57.29578f;
    ins_.gyro_bias[0] = ins_.imu_quaternion_ekf.FilteredValue()(4);
    ins_.gyro_bias[1] = ins_.imu_quaternion_ekf.FilteredValue()(5);
    ins_.gyro_bias[2] = 0; // 大部分时候z轴通天,无法观测yaw的漂移

    // get Yaw total, yaw数据可能会超过360,处理一下方便其他功能使用(如小陀螺)
    if (ins_.yaw - ins_.yaw_angle_last > 180.0f)
    {
        ins_.yaw_round_count--;
    }
    else if (ins_.yaw - ins_.yaw_angle_last < -180.0f)
    {
        ins_.yaw_round_count++;
    }
    ins_.yaw_total_angle = 360.0f * ins_.yaw_round_count + ins_.yaw;
    ins_.yaw_angle_last = ins_.yaw;
}

std::array<float, 4> QuaternionEkf::Quat() const
{
    return {ins_.q[0], ins_.q[1], ins_.q[2], ins_.q[3]};
}

float QuaternionEkf::RollDeg() const { return ins_.roll; }
float QuaternionEkf::PitchDeg() const { return ins_.pitch; }
float QuaternionEkf::YawDeg() const { return ins_.yaw; }
float QuaternionEkf::YawTotalDeg() const { return ins_.yaw_total_angle; }
float QuaternionEkf::YawOmegaRad() const { return ins_.gyro[2]; }
float QuaternionEkf::PitchOmegaRad() const { return ins_.gyro[1]; }

} // namespace alg
