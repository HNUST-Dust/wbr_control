/* SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include <atomic>
#include <cstdint>

#include <zephyr/kernel.h>

template <typename T>
class SeqlockValue {
public:
    SeqlockValue() = default;

    void write(const T& value)
    {
        const unsigned int key = irq_lock();
        seq_.fetch_add(1, std::memory_order_release); // odd: writing
        data_ = value;
        seq_.fetch_add(1, std::memory_order_release); // even: stable
        irq_unlock(key);
    }

    bool read(T& out) const
    {
        // A real-time reader must never spin forever waiting for a lower
        // priority writer.  The writer is IRQ-protected on this single-core
        // target, so contention is normally shorter than one attempt; retain
        // a small retry budget for a writer that ran between the two loads.
        constexpr uint32_t kMaximumAttempts = 3U;
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

    uint32_t sequence() const
    {
        return seq_.load(std::memory_order_acquire);
    }

private:
    mutable std::atomic<uint32_t> seq_{0};
    T data_{};
};
