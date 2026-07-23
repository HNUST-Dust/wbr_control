#include "lqr_schedule.h"

#include <algorithm>

namespace modules {
namespace {

constexpr double kMinScheduledLegLength = 0.15133;
constexpr double kMaxScheduledLegLength = 0.30347;
constexpr double kLegLengthCenter = 0.25;
constexpr double kLegLengthHalfRange = 0.15;


constexpr double kGainPolynomial[2][6][4] = {
  {
      {-1.00598641519278, 1.37368975658577, -3.72818342452613, -18.881307175512},
      {0.00478444462995508, 0.0130514330630057, -1.06192013956714, -2.17960818095631},
      {-0.464543225564795, 0.48567749043846, -0.463308865796654, -3.7885075704009},
      {-0.331441077715527, 0.343149884017749, -0.506297748279386, -3.41594520763882},
      {-0.284858413589816, 1.75045798105555, -3.09203913436471, 2.45248745809249},
      {-0.0548463943902765, 0.188523824123468, -0.372521110404252, 0.288156921860711},
  },
  {
      {0.0767671056435291, 10.9110315315922, -24.6206456184752, 33.4387300683968},
      {0.294868421114949, 0.30541633490697, -1.49612148070953, 4.77996017171149},
      {-1.21864950680155, 5.83956412222717, -11.3119918762291, 14.4456602945481},
      {-1.35587933329927, 4.76107826277765, -8.97166607289147, 11.7491098718828},
      {9.5073927997292, -9.92594496564063, 8.45188767306746, 133.794957178618},
      {1.03243449978301, -0.964929505215573, 0.700088303635305, 4.48358514049062},
  }
};

double EvaluateCubic(const double coefficients[4], double x) {
  return ((coefficients[0] * x + coefficients[1]) * x +
          coefficients[2]) * x + coefficients[3];
}

}  // namespace

void EvaluateLqrGain(double leg_length, double gain[2][6]) {
  const double scheduled_leg_length = std::clamp(
      leg_length, kMinScheduledLegLength, kMaxScheduledLegLength);
  const double normalized_leg_length =
      (scheduled_leg_length - kLegLengthCenter) / kLegLengthHalfRange;

  for (int input = 0; input < 2; ++input) {
    for (int state = 0; state < 6; ++state) {
      gain[input][state] = EvaluateCubic(
          kGainPolynomial[input][state], normalized_leg_length);
    }
  }
}

}  // namespace modules
