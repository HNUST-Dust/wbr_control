/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file channels/hi91_imu_sample.hpp
 * @ingroup wbr_channels
 * @brief 定义 HI91 惯性测量单元采样数据通道。
 * @details 声明跨线程交换的数据快照及其唯一全局通道。写入方发布完整对象，读取方不得保存内部存储地址；使用顺序锁的通道允许读取失败，调用方应保留上一份有效快照。
 */

#pragma once

#include <stdint.h>

#include <channels/comm/seqlock_value.hpp>

namespace channels {

/** @brief HI91 IMU 的一次完整测量快照。 */
struct Hi91ImuSample {
	uint32_t sequence; ///< 发布序号；每产生一个新快照递增一次。
	uint32_t uptime_ms; ///< 时间长度，单位为毫秒。
	uint64_t precise_timestamp_us; ///< 时间长度，单位为微秒。
	uint32_t max_publish_interval_us; ///< 时间长度，单位为微秒。
	uint32_t max_sensor_interval_ms; ///< 时间长度，单位为毫秒。
	uint32_t parse_error_count; ///< 累计事件次数；计数达到类型上限后按实现方式饱和或回绕。
	uint32_t crc_error_count; ///< 累计事件次数；计数达到类型上限后按实现方式饱和或回绕。
	uint32_t publish_gap_with_crc_count; ///< 累计事件次数；计数达到类型上限后按实现方式饱和或回绕。
	uint32_t publish_gap_without_crc_count; ///< 累计事件次数；计数达到类型上限后按实现方式饱和或回绕。
	uint32_t frame_format_error_count; ///< 累计事件次数；计数达到类型上限后按实现方式饱和或回绕。
	uint32_t invalid_sample_count; ///< 累计事件次数；计数达到类型上限后按实现方式饱和或回绕。
	uint32_t rx_drop_count; ///< 累计事件次数；计数达到类型上限后按实现方式饱和或回绕。
	uint32_t rx_stop_count; ///< 累计事件次数；计数达到类型上限后按实现方式饱和或回绕。
	uint32_t rx_buf_rsp_error_count; ///< 累计事件次数；计数达到类型上限后按实现方式饱和或回绕。
	uint32_t system_time_ms; ///< 时间长度，单位为毫秒。
	bool valid; ///< 数据有效标志；为 `false` 时其余字段不得用于控制。
	float roll_deg; ///< 机体横滚角，单位为度。
	float pitch_deg; ///< 机体俯仰角，单位为度。
	float yaw_deg; ///< 归一化航向角，单位为度。
	float quat[4]; ///< 按 `[w, x, y, z]` 排列的姿态单位四元数。
	float gyro_dps[3]; ///< 三轴陀螺仪角速度，单位为度每秒。
	float accel_g[3]; ///< 以标准重力加速度为单位的三轴加速度。
};

/** @brief 全局最新 HI91 IMU 测量快照。 */
extern SeqlockValue<Hi91ImuSample> latest_hi91_imu_sample;

}  // namespace channels
