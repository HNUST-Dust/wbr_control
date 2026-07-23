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
#include "QuaternionEKF.h"
#include "MahonyAHRS.h"

#include <cmath>
#include <cstddef>
#include <cstring>
#include <type_traits>

namespace {
constexpr float imu_quaternion_ekf_f[36] = {1, 0, 0, 0, 0, 0,
                                          0, 1, 0, 0, 0, 0,
                                          0, 0, 1, 0, 0, 0,
                                          0, 0, 0, 1, 0, 0,
                                          0, 0, 0, 0, 1, 0,
                                          0, 0, 0, 0, 0, 1};

constexpr float imu_quaternion_ekf_p_const[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                                 0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                                 0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                                 0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                                 0.1, 0.1, 0.1, 0.1, 100, 0.1,
                                                 0.1, 0.1, 0.1, 0.1, 0.1, 100};

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
 *
 * @param kf
 */
void alg::QuaternionEkf::FLinearizationPFadingCb(ImuKf &kf)
{
    auto &ins = InsFromKf(&kf);
    volatile float q0, q1, q2, q3;
    volatile float q_inv_norm;

    q0 = kf.XhatMinusData()[0];
    q1 = kf.XhatMinusData()[1];
    q2 = kf.XhatMinusData()[2];
    q3 = kf.XhatMinusData()[3];

    // quaternion normalize
    q_inv_norm = alg::MahonyAhrs::InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    for (uint8_t i = 0; i < 4; i++)
    {
        kf.XhatMinusData()[i] *= q_inv_norm;
    }
    /*  F, number with * represent vals to be set
     0     1     2     3     4*     5*
     6     7     8     9    10*    11*
    12    13    14    15    16*    17*
    18    19    20    21    22*    23*
    24    25    26    27    28     29
    30    31    32    33    34     35
    */
    // set F
    kf.FData()[4] = (q1 * ins.dt) * 0.5f;
    kf.FData()[5] = q2 * ins.dt * 0.5f;

    kf.FData()[10] = -q0 * ins.dt * 0.5f;
    kf.FData()[11] = q3 * ins.dt * 0.5f;

    kf.FData()[16] = -q3 * ins.dt * 0.5f;
    kf.FData()[17] = -q0 * ins.dt * 0.5f;

    kf.FData()[22] = q2 * ins.dt * 0.5f;
    kf.FData()[23] = -q1 * ins.dt * 0.5f;

    // fading filter,防止零飘参数过度收敛
    kf.PData()[28] *= ins.lambda;
    kf.PData()[35] *= ins.lambda;

    // 限幅,防止发散
    if (kf.PData()[28] > 10000)
    {
        kf.PData()[28] = 10000;
    }
    if (kf.PData()[35] > 10000)
    {
        kf.PData()[35] = 10000;
    }
}

/**
 * @brief 在工作点处计算观测函数h(x)的Jacobi矩阵H
 *
 * @param kf
 */
void alg::QuaternionEkf::SetHCb(ImuKf &kf)
{
    volatile float double_q0, double_q1, double_q2, double_q3;
    /* H
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    last two cols are zero
    */
    // set H
    double_q0 = 2 * kf.XhatMinusData()[0];
    double_q1 = 2 * kf.XhatMinusData()[1];
    double_q2 = 2 * kf.XhatMinusData()[2];
    double_q3 = 2 * kf.XhatMinusData()[3];

    std::memset(kf.HData(), 0, sizeof(float) * ImuKf::z_size * ImuKf::xhat_size);

    kf.HData()[0] = -double_q2;
    kf.HData()[1] = double_q3;
    kf.HData()[2] = -double_q0;
    kf.HData()[3] = double_q1;

    kf.HData()[6] = double_q1;
    kf.HData()[7] = double_q0;
    kf.HData()[8] = double_q3;
    kf.HData()[9] = double_q2;

    kf.HData()[12] = double_q0;
    kf.HData()[13] = -double_q1;
    kf.HData()[14] = -double_q2;
    kf.HData()[15] = double_q3;
}

/**
 * @brief 利用观测值和先验估计得到最优的后验估计
 *        加入了卡方检验以判断融合加速度的条件是否满足
 *        同时引入发散保护保证恶劣工况下的必要量测更新
 *
 * @param kf
 */
void alg::QuaternionEkf::XhatUpdateCb(ImuKf &kf)
{
    auto &ins = InsFromKf(&kf);
    volatile float q0, q1, q2, q3;

    kf.MatStatus() = static_cast<int8_t>(alg::TransposeMatrix(kf.H(), kf.HT())); // z|x => x|z
    kf.TempMatrix().numRows = kf.H().numRows;
    kf.TempMatrix().numCols = kf.Pminus().numCols;
    kf.MatStatus() = static_cast<int8_t>(alg::MultiplyMatrices(kf.H(), kf.Pminus(), kf.TempMatrix()));
    kf.TempMatrix1().numRows = kf.TempMatrix().numRows;
    kf.TempMatrix1().numCols = kf.HT().numCols;
    kf.MatStatus() = static_cast<int8_t>(alg::MultiplyMatrices(kf.TempMatrix(), kf.HT(), kf.TempMatrix1()));
    kf.S().numRows = kf.R().numRows;
    kf.S().numCols = kf.R().numCols;
    kf.MatStatus() = static_cast<int8_t>(alg::AddMatrices(kf.TempMatrix1(), kf.R(), kf.S()));
    kf.MatStatus() = static_cast<int8_t>(alg::InvertMatrix(kf.S(), kf.TempMatrix1()));

    q0 = kf.XhatMinusData()[0];
    q1 = kf.XhatMinusData()[1];
    q2 = kf.XhatMinusData()[2];
    q3 = kf.XhatMinusData()[3];

    kf.TempVector().numRows = kf.H().numRows;
    kf.TempVector().numCols = 1;
    // 计算预测得到的重力加速度方向(通过姿态获取的)
    kf.TempVector().pData[0] = 2 * (q1 * q3 - q0 * q2);
    kf.TempVector().pData[1] = 2 * (q0 * q1 + q2 * q3);
    kf.TempVector().pData[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 计算预测值和各个轴的方向余弦
    for (uint8_t i = 0; i < 3; i++)
    {
        ins.orientation_cosine[i] = std::cos(std::fabs(kf.TempVector().pData[i]));
    }

    // 利用加速度计数据修正
    kf.TempVector1().numRows = kf.z().numRows;
    kf.TempVector1().numCols = 1;
    kf.MatStatus() = static_cast<int8_t>(alg::SubtractMatrices(kf.z(), kf.TempVector(), kf.TempVector1()));

    // chi-square test,卡方检验
    kf.TempMatrix().numRows = kf.TempVector1().numRows;
    kf.TempMatrix().numCols = 1;
    kf.MatStatus() = static_cast<int8_t>(alg::MultiplyMatrices(kf.TempMatrix1(), kf.TempVector1(), kf.TempMatrix()));
    kf.TempVector().numRows = 1;
    kf.TempVector().numCols = kf.TempVector1().numRows;
    kf.MatStatus() = static_cast<int8_t>(alg::TransposeMatrix(kf.TempVector1(), kf.TempVector()));
    kf.MatStatus() = static_cast<int8_t>(alg::MultiplyMatrices(kf.TempVector(), kf.TempMatrix(), ins.chi_square));
    // rk is small,filter converged/converging
    if (ins.chi_square_data[0] < 0.5f * ins.chi_square_test_threshold)
    {
        ins.converge_flag = 1;
    }
    // rk is bigger than thre but once converged
    if (ins.chi_square_data[0] > ins.chi_square_test_threshold && ins.converge_flag)
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
            std::memcpy(kf.XhatData(), kf.XhatMinusData(), sizeof(float) * ImuKf::xhat_size);
            std::memcpy(kf.PData(), kf.PminusData(), sizeof(float) * ImuKf::xhat_size * ImuKf::xhat_size);
            kf.SetSkipEq5(true); // part5 is P updating
            return;
        }
    }
    else // if divergent or rk is not that big/acceptable,use adaptive gain
    {
        // scale adaptive,rk越小则增益越大,否则更相信预测值
        if (ins.chi_square_data[0] > 0.1f * ins.chi_square_test_threshold && ins.converge_flag)
        {
            ins.adaptive_gain_scale = (ins.chi_square_test_threshold - ins.chi_square_data[0]) / (0.9f * ins.chi_square_test_threshold);
        }
        else
        {
            ins.adaptive_gain_scale = 1;
        }
        ins.error_count = 0;
        kf.SetSkipEq5(false);
    }

    // cal kf-gain K
    kf.TempMatrix().numRows = kf.Pminus().numRows;
    kf.TempMatrix().numCols = kf.HT().numCols;
    kf.MatStatus() = static_cast<int8_t>(alg::MultiplyMatrices(kf.Pminus(), kf.HT(), kf.TempMatrix()));
    kf.MatStatus() = static_cast<int8_t>(alg::MultiplyMatrices(kf.TempMatrix(), kf.TempMatrix1(), kf.K()));

    // implement adaptive
    for (uint8_t i = 0; i < kf.K().numRows * kf.K().numCols; i++)
    {
        kf.KData()[i] *= ins.adaptive_gain_scale;
    }
    for (uint8_t i = 4; i < 6; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            kf.KData()[i * 3 + j] *= ins.orientation_cosine[i - 4] * (1 / 1.5707963f); // 1 rad
        }
    }

    kf.TempVector().numRows = kf.K().numRows;
    kf.TempVector().numCols = 1;
    kf.MatStatus() = static_cast<int8_t>(alg::MultiplyMatrices(kf.K(), kf.TempVector1(), kf.TempVector()));

    // 零漂修正限幅,一般不会有过大的漂移
    if (ins.converge_flag)
    {
        for (uint8_t i = 4; i < 6; i++)
        {
            if (kf.TempVector().pData[i] > 1e-2f * ins.dt)
            {
                kf.TempVector().pData[i] = 1e-2f * ins.dt;
            }
            if (kf.TempVector().pData[i] < -1e-2f * ins.dt)
            {
                kf.TempVector().pData[i] = -1e-2f * ins.dt;
            }
        }
    }

    // 不修正yaw轴数据
    kf.TempVector().pData[3] = 0;
    kf.MatStatus() = static_cast<int8_t>(alg::AddMatrices(kf.xhatminus(), kf.TempVector(), kf.xhat()));
}

/**
 * @brief EKF观测环节,其实就是把数据复制一下
 *
 * @param kf kf类型定义
 */
void alg::QuaternionEkf::ObserveCb(ImuKf &kf)
{
    auto &ins = InsFromKf(&kf);
    std::memcpy(ins.imu_quaternion_ekf_p, kf.PData(), sizeof(ins.imu_quaternion_ekf_p));
    std::memcpy(ins.imu_quaternion_ekf_k, kf.KData(), sizeof(ins.imu_quaternion_ekf_k));
    std::memcpy(ins.imu_quaternion_ekf_h, kf.HData(), sizeof(ins.imu_quaternion_ekf_h));
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

    // 初始化矩阵维度信息（静态尺寸 KalmanFilter<6,0,3>）
    alg::InitializeMatrixView(ins_.chi_square, 1, 1, (float *)ins_.chi_square_data);

    // 姿态初始化
    ins_.imu_quaternion_ekf.XhatData()[0] = 1;
    ins_.imu_quaternion_ekf.XhatData()[1] = 0;
    ins_.imu_quaternion_ekf.XhatData()[2] = 0;
    ins_.imu_quaternion_ekf.XhatData()[3] = 0;

    // 自定义函数初始化,用于扩展或增加kf的基础功能
    ins_.imu_quaternion_ekf.SetUserFunc0(&ObserveCb);
    ins_.imu_quaternion_ekf.SetUserFunc1(&FLinearizationPFadingCb);
    ins_.imu_quaternion_ekf.SetUserFunc2(&SetHCb);
    ins_.imu_quaternion_ekf.SetUserFunc3(&XhatUpdateCb);

    // 设定标志位,用自定函数替换kf标准步骤中的SetK(计算增益)以及xhatupdate(后验估计/融合)
    ins_.imu_quaternion_ekf.SetSkipEq3(true);
    ins_.imu_quaternion_ekf.SetSkipEq4(true);

    memcpy(ins_.imu_quaternion_ekf.FData(), imu_quaternion_ekf_f, sizeof(imu_quaternion_ekf_f));
    memcpy(ins_.imu_quaternion_ekf_p, imu_quaternion_ekf_p_const, sizeof(ins_.imu_quaternion_ekf_p));
    memset(ins_.imu_quaternion_ekf_k, 0, sizeof(ins_.imu_quaternion_ekf_k));
    memset(ins_.imu_quaternion_ekf_h, 0, sizeof(ins_.imu_quaternion_ekf_h));
    memcpy(ins_.imu_quaternion_ekf.PData(), ins_.imu_quaternion_ekf_p, sizeof(ins_.imu_quaternion_ekf_p));
}

void QuaternionEkf::Reset()
{
    ins_.imu_quaternion_ekf.Reset();

    memcpy(ins_.imu_quaternion_ekf_p, imu_quaternion_ekf_p_const, sizeof(ins_.imu_quaternion_ekf_p));

    ins_.converge_flag = 0;
    ins_.error_count = 0;
    ins_.update_count = 0;
    ins_.yaw_round_count = 0;
    ins_.yaw_angle_last = 0.0f;
    ins_.yaw_total_angle = 0.0f;
    ins_.adaptive_gain_scale = 1.0f;

    // 姿态初始化
    ins_.imu_quaternion_ekf.XhatData()[0] = 1;
    ins_.imu_quaternion_ekf.XhatData()[1] = 0;
    ins_.imu_quaternion_ekf.XhatData()[2] = 0;
    ins_.imu_quaternion_ekf.XhatData()[3] = 0;

    // 设定标志位,用自定函数替换kf标准步骤中的SetK(计算增益)以及xhatupdate(后验估计/融合)
    ins_.imu_quaternion_ekf.SetSkipEq3(true);
    ins_.imu_quaternion_ekf.SetSkipEq4(true);

    memcpy(ins_.imu_quaternion_ekf.FData(), imu_quaternion_ekf_f, sizeof(imu_quaternion_ekf_f));
    memcpy(ins_.imu_quaternion_ekf.PData(), ins_.imu_quaternion_ekf_p, sizeof(ins_.imu_quaternion_ekf_p));
}

void QuaternionEkf::Update(float gx, float gy, float gz, float ax, float ay, float az)
{
    // 0.5(Ohm-Ohm^bias)*deltaT,用于更新工作点处的状态转移F矩阵
    volatile float half_gx_dt, half_gy_dt, half_gz_dt;
    volatile float accel_inv_norm;

    /*   F, number with * represent vals to be set
     0      1*     2*     3*     4     5
     6*     7      8*     9*    10    11
    12*    13*    14     15*    16    17
    18*    19*    20*    21     22    23
    24     25     26     27     28    29
    30     31     32     33     34    35
    */

    ins_.gyro[0] = gx - ins_.gyro_bias[0];
    ins_.gyro[1] = gy - ins_.gyro_bias[1];
    ins_.gyro[2] = gz - ins_.gyro_bias[2];

    // set F
    half_gx_dt = 0.5f * ins_.gyro[0] * ins_.dt;
    half_gy_dt = 0.5f * ins_.gyro[1] * ins_.dt;
    half_gz_dt = 0.5f * ins_.gyro[2] * ins_.dt;

    // 此部分设定状态转移矩阵F的左上角部分 4x4子矩阵,即0.5(Ohm-Ohm^bias)*deltaT,右下角有一个2x2单位阵已经初始化好了
    // 注意在predict步F的右上角是4x2的零矩阵,因此每次predict的时候都会调用memcpy用单位阵覆盖前一轮线性化后的矩阵
    memcpy(ins_.imu_quaternion_ekf.FData(), imu_quaternion_ekf_f, sizeof(imu_quaternion_ekf_f));

    ins_.imu_quaternion_ekf.FData()[1] = -half_gx_dt;
    ins_.imu_quaternion_ekf.FData()[2] = -half_gy_dt;
    ins_.imu_quaternion_ekf.FData()[3] = -half_gz_dt;

    ins_.imu_quaternion_ekf.FData()[6] = half_gx_dt;
    ins_.imu_quaternion_ekf.FData()[8] = half_gz_dt;
    ins_.imu_quaternion_ekf.FData()[9] = -half_gy_dt;

    ins_.imu_quaternion_ekf.FData()[12] = half_gy_dt;
    ins_.imu_quaternion_ekf.FData()[13] = -half_gz_dt;
    ins_.imu_quaternion_ekf.FData()[15] = half_gx_dt;

    ins_.imu_quaternion_ekf.FData()[18] = half_gz_dt;
    ins_.imu_quaternion_ekf.FData()[19] = half_gy_dt;
    ins_.imu_quaternion_ekf.FData()[20] = -half_gx_dt;

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


    ins_.imu_quaternion_ekf.MeasuredVector()[0] = ins_.accel[0] * accel_inv_norm; // 用加速度向量更新量测值
    ins_.imu_quaternion_ekf.MeasuredVector()[1] = ins_.accel[1] * accel_inv_norm;
    ins_.imu_quaternion_ekf.MeasuredVector()[2] = ins_.accel[2] * accel_inv_norm;

    // get body state
    ins_.gyro_norm = std::sqrt(ins_.gyro[0] * ins_.gyro[0] + ins_.gyro[1] * ins_.gyro[1] + ins_.gyro[2] * ins_.gyro[2]);


    // 如果角速度小于阈值且加速度处于设定范围内,认为运动稳定,加速度可以用于修正角速度
    // 稍后在最后的姿态更新部分会利用StableFlag来确定
    if (ins_.gyro_norm < 0.3f && ins_.accl_norm > 9.8f - 0.5f && ins_.accl_norm < 9.8f + 0.5f)
    {
        ins_.stable_flag = 1;
    }
    else
    {
        ins_.stable_flag = 0;
    }

    // set Q R,过程噪声和观测噪声矩阵
    ins_.imu_quaternion_ekf.QData()[0] = ins_.q1 * ins_.dt;
    ins_.imu_quaternion_ekf.QData()[7] = ins_.q1 * ins_.dt;
    ins_.imu_quaternion_ekf.QData()[14] = ins_.q1 * ins_.dt;
    ins_.imu_quaternion_ekf.QData()[21] = ins_.q1 * ins_.dt;
    ins_.imu_quaternion_ekf.QData()[28] = ins_.q2 * ins_.dt;
    ins_.imu_quaternion_ekf.QData()[35] = ins_.q2 * ins_.dt;
    ins_.imu_quaternion_ekf.RData()[0] = ins_.r;
    ins_.imu_quaternion_ekf.RData()[4] = ins_.r;
    ins_.imu_quaternion_ekf.RData()[8] = ins_.r;

    ins_.imu_quaternion_ekf.Update();

    // 获取融合后的数据,包括四元数和xy零飘值
    ins_.q[0] = ins_.imu_quaternion_ekf.FilteredValue()[0];
    ins_.q[1] = ins_.imu_quaternion_ekf.FilteredValue()[1];
    ins_.q[2] = ins_.imu_quaternion_ekf.FilteredValue()[2];
    ins_.q[3] = ins_.imu_quaternion_ekf.FilteredValue()[3];
		
    ins_.roll = std::atan2(
        ins_.q[0] * ins_.q[1] + ins_.q[2] * ins_.q[3],
        0.5f - ins_.q[1] * ins_.q[1] - ins_.q[2] * ins_.q[2]
    );

    ins_.roll *= 57.29578f;
    ins_.pitch = 57.29578f * std::asin(-2.0f * (ins_.q[1] * ins_.q[3] - ins_.q[0] * ins_.q[2]));
    ins_.yaw = std::atan2(ins_.q[1] * ins_.q[2] + ins_.q[0] * ins_.q[3],
                      0.5f - ins_.q[2] * ins_.q[2] - ins_.q[3] * ins_.q[3]);
    ins_.yaw *= 57.29578f;
    ins_.gyro_bias[0] = ins_.imu_quaternion_ekf.FilteredValue()[4];
    ins_.gyro_bias[1] = ins_.imu_quaternion_ekf.FilteredValue()[5];
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

// /**
//  * @brief 自定义1/sqrt(x),速度更快
//  *
//  * @param x x
//  * @return float
//  */
// static float invSqrt(float x)
// {
// 	volatile float tmp = 1.0f;
// 	tmp /= __sqrtf(x);
// 	return tmp;
// }
