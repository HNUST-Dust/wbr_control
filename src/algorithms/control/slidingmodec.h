/**
 * @file slidingmodec.h
 * @brief 滑模控制（SMC，历史实现）
 *
 * 设计思路：
 * =========
 * - 基于滑模面 $s$ 与饱和/符号函数生成控制量，用于对模型不确定性更鲁棒的控制。
 * - 当前实现以角度（deg）与角速度（deg/s）为主要输入，内部参数含惯量估计等。
 *
 * 平台依赖：
 * =========
 * - 本文件仍包含 HAL 头文件（`stm32h7xx_hal.h`），属于“算法层历史遗留的非平台无关文件”。
 *   本次文档化不改逻辑；若后续要进一步平台无关化，可把 HAL 依赖移出头文件。
 *
 * 线程模型：
 * =========
 * - `Smc` 实例含内部状态（误差/目标历史值等），不应跨任务共享。
 */

#ifndef WBR_CONTROL_ALGORITHMS_CONTROL_SLIDINGMODEC_H_
#define WBR_CONTROL_ALGORITHMS_CONTROL_SLIDINGMODEC_H_

#include <cstdint>
#include <cmath>

class Smc{
 public:
	float C;
	float K;
	float ref; //初始目标值
	float error_eps;//误差下限
	float u_max;//输出最大值
	float J;//估计惯量
	float angle; //角度反馈，°
	float ang_vel;//角速度反馈，°/s
	float epsilon;

	float u;
	//初始化列表
	Smc(float C,float K,float ref,float error_eps,float u_max,float J,float epsilon):
	C(C),K(K),ref(ref),error_eps(error_eps),u_max(u_max),J(J),epsilon(epsilon){};
	//更新函数
	void Smc_Tick(float angle_now,float angle_vel);

 private:
	float error;
	float error_last;
	float dref;//目标值一阶导
	float ddref;//目标值二阶导
	float refl;//上一次的目标值

	float s;//滑模面

	// 饱和函数
	float Sat(float y)
	{
        const float delta = 0.05f; // 边界层宽度，可调参数
		if (fabs(y) <= delta)
			return y/delta;
		else
			return Signal(y);
	}
	// 符号函数,若有抖动可以换个陡峭的饱和函数
	int8_t Signal(float y)
	{
		if (y > 0)
			return 1;
		else if (y == 0)
			return 0;
		else
			return -1;
	}
};
extern Smc YawSMC;

#endif /* WBR_CONTROL_ALGORITHMS_CONTROL_SLIDINGMODEC_H_ */
