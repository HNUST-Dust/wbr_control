/**
 * @file alg_math.cpp
 * @brief `math/alg_math.h` 的实现（数学/角度工具，历史 C 风格 API）
 *
 * 设计思路：
 * =========
 * - 将分散在业务侧的“通用数学小工具”集中到算法层，供控制/估计复用。
 * - 该文件以全局 `math_*` 函数为主（历史包袱），避免在上层重复造轮子。
 *
 * 注意事项：
 * =========
 * - 角度相关函数混用 `PI` 与 `M_PI`：`PI` 来自 `common/alg_common.h`，而 `M_PI` 取决于
 *   标准库宏开关。工程当前可编译，但后续建议统一使用 `PI` 以避免可移植性问题。
 */
/* Includes ------------------------------------------------------------------*/

#include <algorithms/math/alg_math.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 布尔值反转
 *
 * @param Value 布尔值地址
 */
void math_boolean_logical_not(bool *value)
{
    if (*value == false)
    {
        *value = true;
    }
    else if (*value == true)
    {
        *value = false;
    }
}

/**
 * @brief 16位大小端转换
 *
 * @param Address 地址
 */
void math_endian_reverse_16(void *address)
{
    uint8_t *temp_address_8 = (uint8_t *) address;
    uint16_t *temp_address_16 = (uint16_t *) address;
    *temp_address_16 = temp_address_8[0] << 8 | temp_address_8[1];
}

/**
 * @brief 16位大小端转换
 *
 * @param source 源数据地址
 * @param destination 目标存储地址
 * @return uint16_t 结果
 */
uint16_t math_endian_reverse_16(void *source, void *destination)
{
    uint8_t *temp_address_8 = (uint8_t *) source;
    uint16_t temp_address_16;
    temp_address_16 = temp_address_8[0] << 8 | temp_address_8[1];

    if (destination != nullptr)
    {
        uint8_t *temp_source, *temp_destination;
        temp_source = (uint8_t *) source;
        temp_destination = (uint8_t *) destination;

        temp_destination[0] = temp_source[1];
        temp_destination[1] = temp_source[0];
    }

    return temp_address_16;
}

/**
 * @brief 32位大小端转换
 *
 * @param address 地址
 */
void math_endian_reverse_32(void *address)
{
    uint8_t *temp_address_8 = (uint8_t *) address;
    uint32_t *temp_address_32 = (uint32_t *) address;
    *temp_address_32 = temp_address_8[0] << 24 | temp_address_8[1] << 16 | temp_address_8[2] << 8 | temp_address_8[3];
}

/**
 * @brief 32位大小端转换
 *
 * @param Source 源数据地址
 * @param Destination 目标存储地址
 * @return uint32_t 结果
 */
uint32_t math_endian_reverse_32(void *source, void *destination)
{
    uint8_t *temp_address_8 = (uint8_t *) source;
    uint32_t temp_address_32;
    temp_address_32 = temp_address_8[0] << 24 | temp_address_8[1] << 16 | temp_address_8[2] << 8 | temp_address_8[3];

    if (destination != nullptr)
    {
        uint8_t *temp_source, *temp_destination;
        temp_source = (uint8_t *) source;
        temp_destination = (uint8_t *) destination;

        temp_destination[0] = temp_source[3];
        temp_destination[1] = temp_source[2];
        temp_destination[2] = temp_source[1];
        temp_destination[3] = temp_source[0];
    }

    return temp_address_32;
}

/**
 * @brief 求和
 *
 * @param address 起始地址
 * @param aength 被加的数据的数量, 注意不是字节数
 * @return uint8_t 结果
 */
uint8_t math_sum_8(uint8_t *address, uint32_t length)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < length; i++)
    {
        sum += address[i];
    }
    return (sum);
}

/**
 * @brief 求和
 *
 * @param address 起始地址
 * @param length 被加的数据的数量, 注意不是字节数
 * @return uint16_t 结果
 */
uint16_t math_sum_16(uint16_t *address, uint32_t length)
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < length; i++)
    {
        sum += address[i];
    }
    return (sum);
}

/**
 * @brief 求和
 *
 * @param address 起始地址
 * @param length 被加的数据的数量, 注意不是字节数
 * @return uint32_t 结果
 */
uint32_t math_sum_32(uint32_t *address, uint32_t length)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < length; i++)
    {
        sum += address[i];
    }
    return (sum);
}

/**
 * @brief sinc函数的实现
 *
 * @param x 输入
 * @return float 输出
 */
float math_sinc(float x)
{
    // 分母为0则按极限求法
    if (math_abs(x) <= 2.0f * FLT_EPSILON)
    {
        return (1.0f);
    }

    return (sinf(x) / x);
}

/**
 * @brief 将浮点数映射到整型
 *
 * @param x 浮点数
 * @param float_min 浮点数最小值
 * @param float_max 浮点数最大值
 * @param fnt_min 整型最小值
 * @param int_max 整型最大值
 * @return int32_t 整型
 */
int32_t math_float_to_int(float x, float float_min, float float_max, int32_t int_min, int32_t int_max)
{
    float tmp = (x - float_min) / (float_max - float_min);
    int32_t out = tmp * (float) (int_max - int_min) + int_min;
    return (out);
}

/**
 * @brief 将整型映射到浮点数
 *
 * @param x 整型
 * @param int_min 整型最小值
 * @param int_max 整型最大值
 * @param float_min 浮点数最小值
 * @param float_max 浮点数最大值
 * @return float 浮点数
 */
float math_int_to_float(int32_t x, int32_t int_min, int32_t int_max, float float_min, float float_max)
{
    float tmp = (float) (x - int_min) / (float) (int_max - int_min);
    float out = tmp * (float_max - float_min) + float_min;
    return (out);
}

// 返回值范围: -π ~ π 获取相对于上电时候的角度
float get_relative_angle_pm_pi(float now_angle_cum, float zero_angle)
{
    float rel = fmodf(now_angle_cum - zero_angle, 2.0f * PI); // [-2π, 2π)
    if (rel > PI)
        rel -= 2.0f * PI;
    else if (rel < -PI)
        rel += 2.0f * PI;
    return rel;
}

float normalize_angle_diff(float target, float now)
{
    float error = target - now;
    if (error > PI)
        error -= 2.0f * PI;
    else if (error < -PI)
        error += 2.0f * PI;
    return error;
}
/**
 * @brief 计算云台yaw轴角度误差（考虑跨零与跨π情况）
 * @param target 目标角度（单位：rad，范围 -π~π）
 * @param now 当前角度（单位：rad，范围 -π~π）
 * @return 劣弧方向的角度误差（范围 -π~π）
 */
float CalcYawError(float target, float now)
{
    float err = target - now;

    // 角度归一化到 (-π, π]
    while (err > PI)  err -= 2.0f * PI;
    while (err < -PI) err += 2.0f * PI;

    return err;
}

float normalize_angle_pm_pi(float angle)
{
    float m = fmod(angle, 2.0f * PI);
    if (m > PI){
        m -= 2.0f * PI;
    }else if(m < -PI){
        m += 2.0f * PI;
    }
    return m;
}

float slew_limit(float cmd, float prev, float dt, float max_rate) 
{
    float maxd = max_rate * dt;
    float d = cmd - prev;
    if (d > maxd) d = maxd;
    else if (d < -maxd) d = -maxd;
    return prev + d;
}
/************************ COPYRIGHT(C) HNUST-DUST **************************/
