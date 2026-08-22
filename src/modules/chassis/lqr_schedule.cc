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
        {-0.685478467851538, 1.50799832740696, -3.06465451568562, -16.1608822449108},
        {0.0141202571942007, 0.0127341938220569, -0.308942795685155, -1.04678297832038},
        {-0.452725558746392, 0.815623615250679, -0.988940259883513, -4.29885642851716},
        {-0.256364376301681, 0.48451373332641, -0.713302127223568, -3.07052686061394},
        {0.556185800650799, 1.65583379421802, -4.97966570474401, 6.33727871677954},
        {-0.00780067550010779, 0.216349306093107, -0.603317557877636, 0.790653049909422},
    },
    {
        {2.56748711975694, 5.9186517958355, -21.0054439005816, 31.8081885375668},
        {-0.107271756489953, 0.359733905368126, -0.928079059259584, 1.92623262962995},
        {0.928689031076881, 4.37138680332936, -13.0568258753847, 19.7910202099994},
        {0.402437650267507, 2.8213147547289, -8.18253151249825, 12.8400709405733},
        {8.81576917965081, -17.2759947598341, 20.6492463364113, 123.151887277967},
        {1.25242331530089, -1.93970544659419, 2.09495415787803, 5.23103557888204},
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
