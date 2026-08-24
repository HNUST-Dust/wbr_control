/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file channels/oscilloscope_sample.cpp
 * @ingroup wbr_channels
 * @brief 定义调试示波器采样数据通道。
 * @details 本文件仅定义对应通道的全局存储实例，不启动线程、不访问硬件，也不改变消息字段。
 */

#include <channels/oscilloscope_sample.hpp>

namespace channels {

SeqlockValue<OscilloscopeSample> latest_oscilloscope_sample;

}  // namespace channels
