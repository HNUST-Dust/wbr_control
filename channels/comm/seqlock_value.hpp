/* SPDX-License-Identifier: Apache-2.0 */

/**
* @file channels/comm/seqlock_value.hpp
 * @ingroup wbr_channels
 * @brief 提供适用于单写多读场景的有锁快照容器。
 * @details 写入和读取在有界的 spinlock 临界区内复制完整对象，避免 C++
 * 非原子对象并发读写的未定义行为。读取方不得保存内部存储地址。
 */

#pragma once

#include <atomic>
#include <cstdint>

#include <zephyr/kernel.h>

template <typename T>
/** @brief 使用 spinlock 发布和读取一致性快照的泛型容器。 */
class SeqlockValue {
public:
    SeqlockValue() = default;

    /**
     * @brief 发布一个新的完整快照。
     * @param[in] value 待发布或参与计算的输入值。
     */
    void write(const T& value)
    {
		const k_spinlock_key_t key = k_spin_lock(&lock_);
		seq_.fetch_add(1, std::memory_order_relaxed); // odd: writing
        data_ = value;
		seq_.fetch_add(1, std::memory_order_release); // even: stable
		k_spin_unlock(&lock_, key);
    }

    /**
     * @brief 读取前后一致的最新快照。
     * @param[out] out 接收结果的输出对象；不得为空。
     * @return 读取完成后始终返回 `true`。
     */
    bool read(T& out) const
    {
		/* A spinlock is an IRQ/preemption lock on this single-core target and
		 * an actual SMP lock on multi-core targets.  It makes the non-atomic T
		 * copy well-defined in C++ while keeping the critical section bounded.
		 */
		const k_spinlock_key_t key = k_spin_lock(&lock_);
		out = data_;
		k_spin_unlock(&lock_, key);
		return true;
    }

    /**
     * @brief 读取当前顺序锁序号。
     * @return 当前顺序号；偶数表示没有写操作正在进行。
     */
    uint32_t sequence() const
    {
        return seq_.load(std::memory_order_acquire);
    }

private:
	mutable struct k_spinlock lock_ = {};
    mutable std::atomic<uint32_t> seq_{0};
    T data_{};
};
