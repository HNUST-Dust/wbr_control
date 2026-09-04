/*
 * SPDX-License-Identifier: Apache-2.0
 */

/**
* @file src/modules/module_base.h
 * @ingroup wbr_modules
 * @brief 定义应用功能模块的统一生命周期接口。
 * @details 模块遵循 `ModuleBase` 生命周期：`Start()` 只负责一次性资源初始化和线程创建，`RunLoop()` 持有周期状态。跨线程数据通过 channels 层交换。
 */

#pragma once

#include <errno.h>

#include <zephyr/kernel.h>

namespace modules
{

/*
 * 常驻线程模块的最小抽象基类。
 *
 * 基类定义模块统一的启动和运行接口，并消除线程对象、重复启动保护、线程创建
 * 和入口转发样板；不管理模块的初始化顺序、运行状态或退出流程。
 */
/** @brief 所有周期应用模块必须实现的生命周期基类。 */
class ModuleBase
{
public:
	virtual ~ModuleBase() = default;

	/**
	 * @brief 初始化模块资源并创建工作线程。
	 * @return 成功返回 0；初始化或线程创建失败返回负 errno 错误码。
	 */
	virtual int Start() = 0;
	/**
	 * @brief 执行模块线程的周期主循环。
	 */
	virtual void RunLoop() = 0;

protected:
	/**
	 * @brief 创建并启动绑定当前模块实例的 Zephyr 线程。
	 * @param[in,out] stack Zephyr 线程栈内存。
	 * @param stack_size 线程栈容量，单位为字节。
	 * @param priority Zephyr 抢占式线程优先级。
	 * @param[in] thread_name 线程诊断名称；字符串必须在创建调用期间有效。
	 * @return 成功返回 0，参数无效或底层操作失败时返回负 errno 错误码。
	 */
	int CreateThread(k_thread_stack_t *stack, size_t stack_size, int priority,
			 const char *thread_name)
	{
		if (started_) {
			return 0;
		}

		k_tid_t thread_id =
			k_thread_create(&thread_, stack, stack_size, ThreadEntry,
					this, nullptr, nullptr, priority, 0U, K_NO_WAIT);
		if (thread_id == nullptr) {
			return -ENOMEM;
		}

		(void)k_thread_name_set(thread_id, thread_name);
		started_ = true;
		return 0;
	}

	struct k_thread thread_; ///< Zephyr 工作线程控制块。
	bool started_ = false; ///< 防止模块被重复启动的生命周期标志。

private:
	/**
	 * @brief 将 Zephyr C 风格线程入口转发到模块实例。
	 * @param[in,out] instance 模块实例指针。
	 */
	static void ThreadEntry(void *instance, void *, void *)
	{
		static_cast<ModuleBase *>(instance)->RunLoop();
	}
};

} // namespace modules
