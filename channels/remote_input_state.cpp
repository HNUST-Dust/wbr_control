/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file channels/remote_input_state.cpp
 * @ingroup wbr_channels
 * @brief 定义遥控输入的统一状态通道。
 * @details 本文件仅定义对应通道的全局存储实例，不启动线程、不访问硬件，也不改变消息字段。
 */

#include <channels/remote_input_state.hpp>

SeqlockValue<channels::RemoteInputState> latest_remote_state;
