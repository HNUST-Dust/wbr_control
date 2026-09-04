/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file platform/storage/filesystem/littlefs_service.h
 * @ingroup wbr_platform
 * @brief 封装 LittleFS 文件系统的挂载与访问服务。
 * @details 这是业务层可见的平台边界。返回负 errno 表示参数、设备或传输失败；调用方不得绕过该接口直接依赖具体驱动实例。
 */

#ifndef WBR_CONTROL_PLATFORM_LITTLEFS_SERVICE_H_
#define WBR_CONTROL_PLATFORM_LITTLEFS_SERVICE_H_

namespace platform {

/**
 * @brief 挂载并初始化应用使用的 LittleFS 分区。
 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
 */
int InitializeLittlefs();
/**
 * @brief 判断 LittleFS 是否已经成功挂载。
 * @return 文件系统已成功挂载时返回 `true`。
 */
bool IsLittlefsReady();
/**
 * @brief 获取 LittleFS 的挂载点路径。
 * @return 指向静态挂载点字符串的只读指针。
 */
const char *LittlefsMountPoint();

}  // namespace platform

#endif /* WBR_CONTROL_PLATFORM_LITTLEFS_SERVICE_H_ */
