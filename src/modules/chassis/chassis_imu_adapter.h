/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <msg/chassis_realtime_status.hpp>

#include "chassis_types.h"

namespace modules
{

/** Configure the selected IMU mounting transform during module startup. */
void InitializeChassisImuAdapter();

/** Read and map the selected IMU channel into the chassis physical frame. */
bool ReadChassisImuSample(ChassisImuSample &sample);

/** IMU source identifier published in chassis real-time diagnostics. */
msg::ChassisImuSource SelectedChassisImuSource();

} // namespace modules
