/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "unified_lqr_schedule.h"

#include <algorithm>

namespace modules
{
namespace
{

constexpr double kMinScheduledLegLength = 0.15362;
constexpr double kMaxScheduledLegLength = 0.31101;
constexpr double kLegLengthCenter = 0.25;
constexpr double kLegLengthHalfRange = 0.15;

#include "unified_lqr_coefficients.inc"

double EvaluateCubic(const double coefficients[4], double x)
{
	return ((coefficients[0] * x + coefficients[1]) * x + coefficients[2]) * x +
	       coefficients[3];
}

} // namespace

void EvaluateUnifiedLqrGain(double leg_length, double gain[4][10])
{
	const double scheduled =
		std::clamp(leg_length, kMinScheduledLegLength, kMaxScheduledLegLength);
	const double normalized =
		(scheduled - kLegLengthCenter) / kLegLengthHalfRange;
	for (int output = 0; output < 4; ++output) {
		for (int state = 0; state < 10; ++state) {
			gain[output][state] =
				EvaluateCubic(kGainPolynomial[output][state], normalized);
		}
	}
}

} // namespace modules
