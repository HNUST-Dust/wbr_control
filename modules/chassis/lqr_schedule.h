#ifndef WBR_CONTROL_CORE_LQR_SCHEDULE_H_
#define WBR_CONTROL_CORE_LQR_SCHEDULE_H_

namespace modules {

void EvaluateLqrGain(double leg_length, double gain[2][6]);

}  // namespace modules

#endif  // WBR_CONTROL_CORE_LQR_SCHEDULE_H_
