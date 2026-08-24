/**
* @file src/modules/chassis/lqr_schedule.cc
 * @ingroup wbr_modules
 * @brief 提供随腿长变化的 LQR 增益调度。
 * @details 实现运行在模块自有 Zephyr 线程或其驱动回调中。回调路径只完成有界的数据搬运和通知，耗时解析与控制计算留在线程上下文执行。
 */

#include "lqr_schedule.h"

#include <algorithm>

namespace modules
{
namespace
{

constexpr double kMinScheduledLegLength = 0.15362;
constexpr double kMaxScheduledLegLength = 0.31101;
constexpr double kLegLengthCenter = 0.25;
constexpr double kLegLengthHalfRange = 0.15;
constexpr double kGainPolynomial[2][6][4] = {
    {
        {-0.617932895906852, 1.53974338004564, -3.14793388732218, -16.1893105224962},
        {0.0138557034291979, 0.0187460457506314, -0.312891364097118, -1.07204686406826},
        {-0.40628745044225, 0.825140348759628, -1.03672429497862, -4.26355780804648},
        {-0.229685547242376, 0.493684268281397, -0.74331240665236, -3.06204103719155},
        {0.733287407631373, 1.54720425321872, -5.07658591074389, 6.54515540829415},
        {-0.00398866611724548, 0.208325621367484, -0.600883691797323, 0.795437082504926},
    },
    {
        {3.18711819678451, 5.69658453985927, -21.8775194688484, 33.2899122552148},
        {-0.0795356776824446, 0.415127979005251, -1.11277640930955, 2.15582308025295},
        {1.28610254290234, 4.11353556874322, -13.2345046442921, 20.242581544885},
        {0.619715921492056, 2.71282800964243, -8.40181960109248, 13.2403871405992},
        {7.55297829449997, -17.4029831142366, 21.6771251620606, 122.427745603489},
        {1.10383985732405, -1.85670574379471, 2.07360306752513, 5.23672347911667},
    }
};

double EvaluateCubic(const double coefficients[4], double x)
{
	return ((coefficients[0] * x + coefficients[1]) * x + coefficients[2]) * x +
	       coefficients[3];
}

} // namespace

void EvaluateLqrGain(double leg_length, double gain[2][6])
{
	const double scheduled_leg_length =
		std::clamp(leg_length, kMinScheduledLegLength, kMaxScheduledLegLength);
	const double normalized_leg_length =
		(scheduled_leg_length - kLegLengthCenter) / kLegLengthHalfRange;

	for (int input = 0; input < 2; ++input) {
		for (int state = 0; state < 6; ++state) {
			gain[input][state] =
				EvaluateCubic(kGainPolynomial[input][state], normalized_leg_length);
		}
	}
}

} // namespace modules
