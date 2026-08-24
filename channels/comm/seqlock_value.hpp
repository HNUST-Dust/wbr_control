/* SPDX-License-Identifier: Apache-2.0 */

/**
* @file channels/comm/seqlock_value.hpp
 * @ingroup wbr_channels
 * @brief 提供适用于单写多读场景的顺序锁值容器。
 * @details 声明跨线程交换的数据快照及其唯一全局通道。写入方发布完整对象，读取方不得保存内部存储地址；使用顺序锁的通道允许读取失败，调用方应保留上一份有效快照。
 */

#pragma once

#include <atomic>
#include <cstdint>

#include <zephyr/kernel.h>

template <typename T>
/** @brief 使用顺序锁发布和读取一致性快照的泛型容器。 */
class SeqlockValue {
public:
    SeqlockValue() = default;

    /**
     * @brief 发布一个新的完整快照。
     * @param[in] value 待发布或参与计算的输入值。
     */
    void write(const T& value)
    {
        const unsigned int key = irq_lock();
        seq_.fetch_add(1, std::memory_order_release); // odd: writing
        data_ = value;
        seq_.fetch_add(1, std::memory_order_release); // even: stable
        irq_unlock(key);
    }

    /**
     * @brief 尝试读取前后一致的最新快照。
     * @param[out] out 接收结果的输出对象；不得为空。
     * @return 读到前后一致的快照时返回 `true`；重试耗尽返回 `false`。
     */
    bool read(T& out) const
    {
        // A real-time reader must never spin forever waiting for a lower
        // priority writer.  The writer is IRQ-protected on this single-core
        // target, so contention is normally shorter than one attempt; retain
        // a small retry budget for a writer that ran between the two loads.
        constexpr uint32_t kMaximumAttempts = 3U; ///< `kMaximumAttempts` 输出或状态的安全上限。
        for (uint32_t attempt = 0U; attempt < kMaximumAttempts; ++attempt) {
            const uint32_t before = seq_.load(std::memory_order_acquire);
            if (before & 1U) {
                continue;
            }
            const T snapshot = data_;
            const uint32_t after = seq_.load(std::memory_order_acquire);
            if ((before == after) && ((after & 1U) == 0U)) {
                out = snapshot;
                return true;
            }
        }

        return false;
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
    mutable std::atomic<uint32_t> seq_{0};
    T data_{};
};
