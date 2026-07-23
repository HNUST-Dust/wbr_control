// Legacy controller stack; excluded from the firmware build.
#ifndef WBR_CONTROL_CORE_MATH_UTILS_H_
#define WBR_CONTROL_CORE_MATH_UTILS_H_

namespace wbr::v2 {

double Clamp(double value, double lo, double hi);
double MoveTowards(double value, double target, double max_step);
double FadeAuthority(double magnitude, double soft_limit, double hard_limit);
double RiseAuthority(double value, double hard_limit, double soft_limit);

}  // namespace wbr::v2

#endif  // WBR_CONTROL_CORE_MATH_UTILS_H_
