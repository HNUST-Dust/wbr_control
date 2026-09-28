/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef WBR_CONTROL_UNIFIED_LQR_SCHEDULE_H_
#define WBR_CONTROL_UNIFIED_LQR_SCHEDULE_H_

namespace modules
{

/**
 * @brief Evaluate the complete four-input, ten-state gain matrix.
 *
 * State order is position, speed, yaw, yaw rate, left/right relative leg
 * angle theta = alpha - pitch and their rates, body pitch and pitch rate. Output order is left wheel,
 * right wheel, left virtual-leg torque and right virtual-leg torque.
 */
void EvaluateUnifiedLqrGain(double leg_length, double gain[4][10]);

} // namespace modules

#endif // WBR_CONTROL_UNIFIED_LQR_SCHEDULE_H_
