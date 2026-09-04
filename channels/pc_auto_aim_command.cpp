/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file channels/pc_auto_aim_command.cpp
 * @ingroup wbr_channels
 * @brief 定义上位机自瞄控制指令通道。
 * @details 本文件仅定义对应通道的全局存储实例，不启动线程、不访问硬件，也不改变消息字段。
 */

#include <channels/pc_auto_aim_command.hpp>

namespace channels {

SeqlockValue<PcAutoAimCommand> latest_pc_auto_aim_command;

}  // namespace channels
