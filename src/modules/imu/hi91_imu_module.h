/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file src/modules/imu/hi91_imu_module.h
 * @ingroup wbr_modules
 * @brief 实现 HI91 IMU 的采集、校验与状态发布模块。
 * @details 模块遵循 `ModuleBase` 生命周期：`Start()` 只负责一次性资源初始化和线程创建，`RunLoop()` 持有周期状态。跨线程数据通过 channels 层交换。
 */

#pragma once

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include "../module_base.h"
#include <protocols/imu/hi91_protocol.h>

namespace modules
{

/** @brief 采集 HI91 数据并向系统发布姿态的应用模块。 */
class Hi91ImuModule : public ModuleBase
{
public:
	Hi91ImuModule();
	/**
	 * @brief 初始化模块资源并创建工作线程。
	 * @return 成功返回 0；初始化或线程创建失败返回负 errno 错误码。
	 */
	int Start() override;
	/**
	 * @brief 执行模块线程的周期主循环。
	 */
	void RunLoop() override;

private:
	/*
	 * HI91 接收采用“硬件循环 DMA + 软件读指针”：
	 *
	 *   UART2 硬件 FIFO
	 *       --HDMA持续搬运--> .nocache DMA ring[write_pos]
	 *       --linked descriptor--> 到末尾后硬件自动回到 ring 起点
	 *       --HI91线程每1ms读取DMA写指针--> 处理 read_pos..write_pos
	 *       --逐字节组帧--> frame_buf_
	 *
	 * DMA 只启动一次，descriptor 之间的切换完全由硬件完成，所以不存在
	 * stop/reconfigure/start 窗口。UART 的 4 字节 timeout 只负责促使 FIFO
	 * 尾数产生 DMA request，不再被当作帧边界。协议状态机从连续字节流中
	 * 自行寻找 5A A5、读取长度并完成 CRC 校验。
	 */
	static constexpr size_t kRxDmaRingSize = 4096U; ///< UART DMA 接收环形缓冲区容量，单位为字节。
	/*
	 * TRANSIZE 可能先于最后几个 AHB 写事务更新。CPU 始终落后 DMA 至少
	 * 16 字节，避免读取尚未提交、仍保留上一圈内容的 ring 前沿。
	 */
	static constexpr size_t kRxCommitGuardSize = 16U; ///< 提交 DMA 数据前保留的并发写入保护区，单位为字节。
	static constexpr size_t kFrameBufferSize =
		protocols::kHi91FrameHeaderSize + protocols::kHi91MaxPayloadLength;
	/**
	 * @brief 按顺序将接收字节送入 HI91 流式解析器。
	 * @param[in] data 输入字节缓冲区；长度由相邻长度参数给出。
	 * @param size 输入数据的字节数。
	 */
	void ProcessBytes(const uint8_t *data, size_t size);
	/**
	 * @brief 向 HI91 流式解析状态机输入一个字节。
	 * @param byte 送入流式解析器的单个字节。
	 */
	void ProcessByte(uint8_t byte);
	/**
	 * @brief 将协议采样转换为系统 IMU 快照并发布。
	 * @param[in] sample 当前控制或传感器采样。
	 */
	void PublishSample(const protocols::Hi91Sample &sample);
	/**
	 * @brief 记录并限频报告协议解析错误。
	 * @param error 协议解析返回的错误码。
	 */
	void ReportParseIssue(int error);

	const struct device *uart_dev_ = nullptr;
	size_t rx_read_pos_ = 0U;
	size_t rx_observed_write_pos_ = 0U;
	size_t rx_available_bytes_ = 0U;
	/* 跨多次 UART_RX_RDY 保存一个尚未收完整的 HI91 协议帧。 */
	uint8_t frame_buf_[kFrameBufferSize] = {};
	uint8_t frame_state_ = 0U;
	uint8_t frame_pos_ = 0U;
	uint16_t frame_remaining_ = 0U;
	uint32_t sample_sequence_ = 0U;
	uint32_t parse_error_count_ = 0U;
	uint32_t crc_error_count_ = 0U;
	uint32_t crc_errors_since_last_publish_ = 0U;
	uint32_t publish_gap_with_crc_count_ = 0U;
	uint32_t publish_gap_without_crc_count_ = 0U;
	uint32_t frame_format_error_count_ = 0U;
	uint32_t invalid_sample_count_ = 0U;
	uint32_t rx_drop_count_ = 0U;
	uint32_t rx_stop_count_ = 0U;
	uint32_t rx_buf_rsp_error_count_ = 0U;
	uint32_t last_sensor_time_ms_ = 0U;
	uint32_t max_sensor_interval_ms_ = 0U;
	uint64_t last_publish_time_us_ = 0U;
	uint32_t max_publish_interval_us_ = 0U;
};

} // namespace modules
