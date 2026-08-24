/**
* @file src/modules/chassis/lqr_schedule.h
 * @ingroup wbr_modules
 * @brief 提供随腿长变化的 LQR 增益调度。
 * @details 模块遵循 `ModuleBase` 生命周期：`Start()` 只负责一次性资源初始化和线程创建，`RunLoop()` 持有周期状态。跨线程数据通过 channels 层交换。
 */

#ifndef WBR_CONTROL_CORE_LQR_SCHEDULE_H_
#define WBR_CONTROL_CORE_LQR_SCHEDULE_H_

namespace modules
{

/**
 * @brief 按腿长计算当前 LQR 增益矩阵。
 * @param leg_length 当前等效腿长，单位为米；超出标定区间时按边界处理。
 * @param[out] gain 输出 2×6 状态反馈增益矩阵，第一行为车轮力矩增益，第二行为髋关节力矩增益。
 */
void EvaluateLqrGain(double leg_length, double gain[2][6]);

} // namespace modules

#endif // WBR_CONTROL_CORE_LQR_SCHEDULE_H_
