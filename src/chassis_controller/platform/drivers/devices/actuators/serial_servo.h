/* SPDX-License-Identifier: Apache-2.0 */

/**
* @file platform/drivers/devices/actuators/serial_servo.h
 * @ingroup wbr_platform
 * @brief 实现串行舵机的数据发送驱动。
 * @details 这是业务层可见的平台边界。返回负 errno 表示参数、设备或传输失败；调用方不得绕过该接口直接依赖具体驱动实例。
 */

#ifndef WBR_CONTROL_PLATFORM_SERIAL_SERVO_H_
#define WBR_CONTROL_PLATFORM_SERIAL_SERVO_H_

#include <stdint.h>

namespace platform {

/**
 * @brief 初始化串行舵机通信接口。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int InitializeSerialServo();
/**
 * @brief 命令指定舵机在给定时间内转到目标角度。
 * @param id 目标串行舵机的总线 ID。
 * @param degrees 目标舵机角度，单位为度。
 * @param time_ms 舵机完成动作的期望时间，单位为毫秒。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int MoveSerialServoToAngle(uint8_t id, float degrees, uint16_t time_ms);
/**
 * @brief 设置指定串行舵机的转动速度。
 * @param id 目标串行舵机的总线 ID。
 * @param speed 舵机目标速度，符号表示转动方向。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int SetSerialServoSpeed(uint8_t id, int16_t speed);
/**
 * @brief 停止指定串行舵机。
 * @param id 目标串行舵机的总线 ID。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int StopSerialServo(uint8_t id);
/**
 * @brief 查询舵机当前使用的总线 ID。
 * @param query_id 用于发起查询的舵机 ID，广播查询时使用协议广播值。
 * @param[out] out_id 接收查询所得舵机 ID 的指针；不得为空。
 * @param timeout_ms 等待超时时间，单位为毫秒；负值表示永久等待。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int ReadSerialServoId(uint8_t query_id, uint8_t *out_id,
		      uint32_t timeout_ms);

}  // namespace platform

#endif /* WBR_CONTROL_PLATFORM_SERIAL_SERVO_H_ */
