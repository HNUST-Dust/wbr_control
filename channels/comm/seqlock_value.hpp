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
        uint32_t before;
        uint32_t after = 0;

        do {
            before = seq_.load(std::memory_order_acquire);
            if (before & 1U) {
                continue;
            }
            out = data_;
            after = seq_.load(std::memory_order_acquire);
        } while (before != after);

        return true;
    }

    uint32_t sequence() const
    {
        return seq_.load(std::memory_order_acquire);
    }

private:
    mutable std::atomic<uint32_t> seq_{0};
    T data_{};
};
