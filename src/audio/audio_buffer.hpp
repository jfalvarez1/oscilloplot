#pragma once

#include <vector>
#include <atomic>
#include <cstddef>

namespace oscilloplot {

// Lock-free circular buffer for audio visualization
template<typename T>
class CircularBuffer {
public:
    explicit CircularBuffer(size_t capacity = 8192)
        : m_buffer(capacity), m_capacity(capacity) {}

    void push(T value) {
        size_t write = m_writePos.load(std::memory_order_relaxed);
        m_buffer[write] = value;
        m_writePos.store((write + 1) % m_capacity, std::memory_order_release);
    }

    void push(const T* data, size_t count) {
        for (size_t i = 0; i < count; ++i) {
            push(data[i]);
        }
    }

    bool pop(T& value) {
        size_t read = m_readPos.load(std::memory_order_relaxed);
        size_t write = m_writePos.load(std::memory_order_acquire);

        if (read == write) return false; // empty

        value = m_buffer[read];
        m_readPos.store((read + 1) % m_capacity, std::memory_order_release);
        return true;
    }

    size_t available() const {
        size_t write = m_writePos.load(std::memory_order_acquire);
        size_t read = m_readPos.load(std::memory_order_relaxed);
        return (write >= read) ? (write - read) : (m_capacity - read + write);
    }

    void clear() {
        m_readPos.store(0, std::memory_order_release);
        m_writePos.store(0, std::memory_order_release);
    }

private:
    std::vector<T> m_buffer;
    size_t m_capacity;
    std::atomic<size_t> m_readPos{0};
    std::atomic<size_t> m_writePos{0};
};

} // namespace oscilloplot
