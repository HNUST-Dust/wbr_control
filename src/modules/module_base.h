/*
 * SPDX-License-Identifier: Apache-2.0
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
class ModuleBase
{
public:
	virtual ~ModuleBase() = default;

	virtual int Start() = 0;
	virtual void RunLoop() = 0;

protected:
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

	struct k_thread thread_;
	bool started_ = false;

private:
	static void ThreadEntry(void *instance, void *, void *)
	{
		static_cast<ModuleBase *>(instance)->RunLoop();
	}
};

} // namespace modules
