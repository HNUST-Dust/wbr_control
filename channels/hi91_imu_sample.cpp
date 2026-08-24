/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file channels/hi91_imu_sample.cpp
 * @ingroup wbr_channels
 * @brief 定义 HI91 惯性测量单元采样数据通道。
 * @details 本文件仅定义对应通道的全局存储实例，不启动线程、不访问硬件，也不改变消息字段。
 */

#include <channels/hi91_imu_sample.hpp>

namespace channels {

SeqlockValue<Hi91ImuSample> latest_hi91_imu_sample;

}  // namespace channels
