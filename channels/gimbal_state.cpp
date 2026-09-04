/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file channels/gimbal_state.cpp
 * @ingroup wbr_channels
 * @brief 定义云台姿态与运动状态通道。
 * @details 本文件仅定义对应通道的全局存储实例，不启动线程、不访问硬件，也不改变消息字段。
 */

#include <channels/gimbal_state.hpp>

namespace channels {

SeqlockValue<GimbalState> latest_gimbal_state;

}  // namespace channels
