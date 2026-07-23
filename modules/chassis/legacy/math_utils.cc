// Legacy controller stack; excluded from the firmware build.
#include "math_utils.h"

#include <cmath>

namespace wbr::v2 {

double Clamp(double value, double lo, double hi) {
  return std::fmin(std::fmax(value, lo), hi);
}

double MoveTowards(double value, double target, double max_step) {
  return value + Clamp(target - value, -max_step, max_step);
}

double FadeAuthority(double magnitude, double soft_limit, double hard_limit) {
  return 1.0 - Clamp((magnitude - soft_limit) /
                         (hard_limit - soft_limit),
                     0.0, 1.0);
}

double RiseAuthority(double value, double hard_limit, double soft_limit) {
  return Clamp((value - hard_limit) / (soft_limit - hard_limit), 0.0, 1.0);
}

}  // namespace wbr::v2
