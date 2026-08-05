#include "lqr_schedule.h"

#include <algorithm>

namespace modules
{
namespace
{

constexpr double kMinScheduledLegLength = 0.15105;
constexpr double kMaxScheduledLegLength = 0.30273;
constexpr double kLegLengthCenter = 0.25;
constexpr double kLegLengthHalfRange = 0.15;
constexpr double kGainPolynomial[2][6][4] = {
    {
        {-0.707546734387229, 1.05070633342823, -2.10150793271256, -13.8616584803847},
        {0.00638742209834923, 0.0066504576690061, -0.200074429449787, -0.736793331776394},
        {-0.4639329600113, 0.563876085562364, -0.593039276351116, -4.60425269826635},
        {-0.246214172175071, 0.306088258551846, -0.413033187346267, -3.01964871226941},
        {-0.0954464334094279, 1.99750692649159, -4.25184326067819, 4.91060941567267},
        {-0.0552597102453925, 0.185644783312713, -0.397433326229983, 0.423994664721313},
    },
    {
        {0.33787383360288, 6.25004020583139, -15.2322314986336, 20.4495985540348},
        {-0.103217730230956, 0.251697676947074, -0.509172114107813, 1.03091632285471},
        {-0.665999451509117, 5.15618664661316, -11.1148969044366, 15.1105581185322},
        {-0.623920237856495, 3.14600821640441, -6.57144399487403, 9.24109191198662},
        {9.45032212264213, -12.0734737865849, 12.4184578240092, 130.347789941987},
        {0.875278419429029, -0.926433645220042, 0.806206736270484, 4.30475001117095},
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
